import os
import sys
import json
import argparse
import subprocess
import tempfile
import uuid
import xml.etree.ElementTree as ET
from collections import deque

from flask import Flask, render_template, request, jsonify

from osm_graph import (
    build_contracted_graph, select_connected_subgraph, name_map_for,
    classify_zones, compute_intersection_density, compute_congestion_delay,
    compute_vehicle_counts, compute_reachable, auto_start_goal,
    select_local_subgraph,
)
from sumo_signals import (
    signal_delay_for, assign_movement_signal_delay, turn_time_s,
    SUMO_MOVEMENTS,
)
from pddl_writer import write_pddl, route_metrics
from enhsp_runner import (
    DOMAIN_PATH, ENHSP_TIMEOUT, MAX_SOLVABLE_NODES, trova_enhsp,
    diagnose_enhsp, parse_plan, run_enhsp, run_enhsp_output,
)

app = Flask(__name__)

PROJECT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
PDDL_DIR = os.path.join(PROJECT_ROOT, 'pddl_files')

# store in memoria: token → dati grafo
graph_store = {}


# ── routes ────────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/generate', methods=['POST'])
def generate():
    osm_file  = request.files.get('osm_file')
    max_nodes = int(request.form.get('max_nodes', 50))
    zone      = request.form.get('zone', 'custom') or 'custom'

    if not osm_file:
        return jsonify({'error': 'Nessun file caricato'}), 400

    tmp_dir  = tempfile.mkdtemp()
    osm_path = os.path.join(tmp_dir, 'input.osm')
    osm_file.save(osm_path)

    try:
        node_data, adj, signal_node_ids, edge_highway = build_contracted_graph(osm_path)
        if not adj:
            return jsonify({'error': 'Nessuna strada percorribile trovata nel file OSM'}), 400

        selected = select_connected_subgraph(node_data, adj, max_nodes)
        sel_set  = set(selected)

        edges = {}
        for a in selected:
            for b, (d, spd) in adj.get(a, {}).items():
                if b in sel_set:
                    edges[(a, b)] = (d, spd)

        start_osm, goal_osm = auto_start_goal(selected, edges, node_data)
        nm     = name_map_for(selected, node_data)
        nm_inv = {v: k for k, v in nm.items()}
        signal_nodes_in_subgraph = signal_node_ids & sel_set

        # pre-calcolo congestione (salvato nel token)
        peripheral  = classify_zones(selected, node_data)
        density     = compute_intersection_density(selected, node_data)
        sub_hw      = {(a, b): edge_highway.get((a, b), "unclassified") for (a, b) in edges}
        cong_delays = compute_congestion_delay(selected, edges, peripheral, density, sub_hw)
        vc          = compute_vehicle_counts(selected, edges)

        token = str(uuid.uuid4())
        graph_store[token] = {
            'node_data':   node_data,
            'edges':       edges,
            'selected':    selected,
            'nm':          nm,
            'nm_inv':      nm_inv,
            'zone':        zone,
            'signal_nodes': signal_nodes_in_subgraph,
            'peripheral':  peripheral,
            'density':     density,
            'cong_delays': cong_delays,
            'vehicle_counts': vc,
            'edge_highway': sub_hw,
        }

        nodes_out = [{
            'id': nm[nd], 'lat': node_data[nd]['lat'], 'lon': node_data[nd]['lon'],
            'name': node_data[nd]['name'], 'is_start': nd == start_osm,
            'is_goal': nd == goal_osm, 'is_signal': nd in signal_nodes_in_subgraph,
            'is_peripheral': nd in peripheral,
            'congestion_delay': cong_delays.get(nd, 0),
            'intersection_density': density.get(nd, 0),
        } for nd in selected]

        edges_out = [{
            'from': nm[a], 'to': nm[b], 'distance': d, 'speed': spd,
            'vehicle_count': vc.get((a, b), 0),
            'congestion_factor': round(1.0 + vc.get((a, b), 0) / 10.0, 2),
        } for (a, b), (d, spd) in edges.items()]

        return jsonify({
            'success': True, 'token': token,
            'nodes': nodes_out, 'edges': edges_out,
            'auto_start': nm[start_osm], 'auto_goal': nm[goal_osm],
            'stats': {'n_nodes': len(selected), 'n_edges': len(edges)},
        })

    except ET.ParseError as e:
        return jsonify({'error': f'File OSM non valido: {e}'}), 400
    except Exception as e:
        import traceback
        return jsonify({'error': str(e), 'trace': traceback.format_exc()}), 500


@app.route('/api/solve', methods=['POST'])
def solve():
    data       = request.get_json()
    token      = data.get('token')
    start_pddl = data.get('start')
    goal_pddl  = data.get('goal')

    store = graph_store.get(token)
    if not store:
        return jsonify({'error': 'Sessione scaduta, ricarica il file OSM'}), 400

    nm           = store['nm']
    nm_inv       = store['nm_inv']
    node_data    = store['node_data']
    edges        = store['edges']
    selected     = store['selected']
    zone         = store['zone']
    signal_nodes = store.get('signal_nodes', set())
    peripheral   = store.get('peripheral', set())
    density      = store.get('density', {})
    cong_delays  = store.get('cong_delays', {})
    vc           = store.get('vehicle_counts', {})
    sub_hw       = store.get('edge_highway', {})

    start_osm = nm_inv.get(start_pddl)
    goal_osm  = nm_inv.get(goal_pddl)

    if not start_osm:
        return jsonify({'error': f'Start "{start_pddl}" non trovato'}), 400
    if not goal_osm:
        return jsonify({'error': f'Goal "{goal_pddl}" non trovato'}), 400
    if start_osm == goal_osm:
        return jsonify({'error': 'Start e Goal devono essere nodi diversi'}), 400

    reach = compute_reachable(start_osm, edges)
    if goal_osm not in reach:
        return jsonify({'error': f'Il goal "{goal_pddl}" non è raggiungibile da "{start_pddl}"'}), 400

    # Sottografo locale per la coppia start/goal, NON l'intero grafo caricato
    # (che con "tutti i nodi" su una zona grande puo' avere migliaia di nodi
    # anche per un tragitto di poche centinaia di metri) — il costo del solve
    # deve dipendere dalla distanza reale del tragitto, non dalla dimensione
    # della mappa mostrata.
    local_nodes = select_local_subgraph(start_osm, goal_osm, edges, node_data)
    local_set = set(local_nodes)
    local_edges = {(a, b): v for (a, b), v in edges.items() if a in local_set and b in local_set}

    if len(local_nodes) > MAX_SOLVABLE_NODES:
        return jsonify({'error':
            f'Problema troppo grande per ENHSP: {len(local_nodes)} nodi nel '
            f'sottografo locale start-goal (massimo {MAX_SOLVABLE_NODES}). '
            f'La distanza tra i due punti richiede troppe deviazioni. '
            f'Aumenta la heap con ENHSP_HEAP e alza MAX_SOLVABLE_NODES.'}), 400

    pddl_content = write_pddl(
        zone, local_nodes, node_data, local_edges, start_osm, goal_osm, nm,
        signal_nodes=signal_nodes,
        congestion_delays=cong_delays,
        vehicle_counts=vc,
        intersection_density=density,
        peripheral=peripheral,
        edge_highway=sub_hw,
    )

    custom_pddl_path = os.path.join(PDDL_DIR, 'problem_custom.pddl')
    try:
        os.makedirs(PDDL_DIR, exist_ok=True)
        with open(custom_pddl_path, 'w', encoding='utf-8') as f:
            f.write(pddl_content)
    except Exception:
        pass

    tmp_dir   = tempfile.mkdtemp()
    pddl_path = os.path.join(tmp_dir, 'problem.pddl')
    with open(pddl_path, 'w', encoding='utf-8') as f:
        f.write(pddl_content)

    plan_text = route = total_dist = travel_time = None
    signals_crossed = signal_delay_total = plan_time_ms = None
    enhsp_error = None

    jar        = trova_enhsp()
    domain_abs = os.path.abspath(DOMAIN_PATH)

    if not jar:
        enhsp_error = "ENHSP non trovato — installa con: pip install up-enhsp"
    elif not os.path.exists(domain_abs):
        enhsp_error = f"domain.pddl non trovato in: {domain_abs}"
    else:
        try:
            output = run_enhsp_output(jar, domain_abs, pddl_path, ENHSP_TIMEOUT)

            if "Problem Solved" in output:
                plan_text, route, plan_time_ms = parse_plan(output)

                # salva il percorso pianificato (sequenza di nodi PDDL) cosi'
                # che SUMO possa seguire ESATTAMENTE le stesse strade del piano,
                # invece di ricalcolare un proprio Dijkstra start->goal.
                route_path = os.path.join(PDDL_DIR, 'route_custom.json')
                try:
                    with open(route_path, 'w', encoding='utf-8') as f:
                        json.dump({'route': route or []}, f)
                except Exception:
                    pass

                if route and len(route) >= 2:
                    total_dist = 0; travel_time = 0.0
                    signals_crossed = 0; signal_delay_total = 0.0
                    turn_delay_total = 0.0
                    route_osm = [nm_inv.get(r) for r in route]
                    for i in range(len(route) - 1):
                        a_osm = route_osm[i]; b_osm = route_osm[i + 1]
                        if a_osm and b_osm and (a_osm, b_osm) in edges:
                            d, spd = edges[(a_osm, b_osm)]
                            total_dist  += d
                            vc_arc       = vc.get((a_osm, b_osm), 0)
                            cf           = 1.0 + vc_arc / 10.0
                            eff_spd      = spd / cf
                            if eff_spd > 0:
                                travel_time += d / eff_spd
                        if not (a_osm and b_osm):
                            continue
                        # ritardo semaforico del movimento (prev,a_osm,b_osm) — pagato
                        # partendo da a_osm, come in domain.pddl 'start-move' (sez. 3.1
                        # di 2_traffic_signal_optimization.md). i==0: nessun prev reale
                        # (start-move fittizio start->start->b_osm).
                        if i == 0:
                            sd = assign_movement_signal_delay(a_osm, a_osm, b_osm, node_data,
                                                               SUMO_MOVEMENTS, is_first=True)
                        else:
                            p_osm = route_osm[i - 1]
                            turn_delay_total += turn_time_s(p_osm, a_osm, b_osm, node_data)
                            sd = assign_movement_signal_delay(p_osm, a_osm, b_osm, node_data, SUMO_MOVEMENTS)
                        if sd is None:
                            sd = signal_delay_for(a_osm, signal_nodes)
                        if sd > 0:
                            signals_crossed    += 1
                            signal_delay_total += sd
                    for node_osm in route_osm[1:]:
                        if node_osm:
                            travel_time += cong_delays.get(node_osm, 0)
                    travel_time += signal_delay_total + turn_delay_total
                    signal_delay_total = round(signal_delay_total, 1)
            else:
                enhsp_error = diagnose_enhsp(output)
        except subprocess.TimeoutExpired:
            enhsp_error = (f"ENHSP ha superato il timeout ({ENHSP_TIMEOUT}s) — "
                           "riduci i nodi oppure alza ENHSP_TIMEOUT")
        except FileNotFoundError:
            enhsp_error = "Java non trovato — installa Java 17+"

    # calcola sommario congestione sul percorso
    congestion_on_route = []
    if route and len(route) >= 2:
        for i in range(len(route) - 1):
            a_osm = nm_inv.get(route[i])
            b_osm = nm_inv.get(route[i + 1])
            if a_osm and b_osm and (a_osm, b_osm) in edges:
                vc_arc = vc.get((a_osm, b_osm), 0)
                congestion_on_route.append({
                    'from': route[i], 'to': route[i+1],
                    'vehicle_count': vc_arc,
                    'congestion_factor': round(1.0 + vc_arc / 10.0, 2),
                })

    congestion_delay_total = sum(
        cong_delays.get(nm_inv.get(node_name), 0)
        for node_name in (route[1:] if route else [])
    )
    n_peripheral_on_route = sum(
        1 for node_name in (route or [])
        if nm_inv.get(node_name) in peripheral
    )

    return jsonify({
        'success': True,
        'pddl_content': pddl_content,
        'plan_text':    plan_text,
        'route':        route,
        'enhsp_error':  enhsp_error,
        'congestion_on_route': congestion_on_route,
        'stats': {
            'total_dist':              total_dist,
            'travel_time':             round(travel_time, 1) if travel_time is not None else None,
            'signals_crossed':         signals_crossed if plan_text else None,
            'signal_delay_total':      signal_delay_total if plan_text else None,
            'congestion_delay_total':  congestion_delay_total if plan_text else None,
            'n_peripheral_on_route':   n_peripheral_on_route if plan_text else None,
            'plan_time_ms':            plan_time_ms,
            'start':                   start_pddl,
            'goal':                    goal_pddl,
        }
    })


@app.route('/api/replan', methods=['POST'])
def replan():
    """Ricalcola il percorso evitando strade/incroci resi non percorribili.

    Il replanning NON riparte dall'origine: simula un veicolo gia' in viaggio
    che trova la strada chiusa. Si individua il primo elemento bloccato lungo
    il piano corrente e si ripianifica **dal nodo immediatamente precedente**
    (l'ultimo punto raggiungibile), passando ad ENHSP anche il nodo di
    provenienza, cosi' il costo della prima svolta e' quello reale."""
    data = request.get_json() or {}
    token = data.get('token')
    store = graph_store.get(token)
    if not store:
        return jsonify({'error': 'Sessione scaduta, ricarica il file OSM'}), 400

    nm, nm_inv = store['nm'], store['nm_inv']
    node_data, edges, selected = store['node_data'], store['edges'], store['selected']
    zone = store['zone']
    signal_nodes = store.get('signal_nodes', set())
    peripheral = store.get('peripheral', set())
    density = store.get('density', {})
    cong_delays = store.get('cong_delays', {})
    vc = store.get('vehicle_counts', {})
    sub_hw = store.get('edge_highway', {})

    route = data.get('route') or []
    goal_pddl = data.get('goal')
    goal_osm = nm_inv.get(goal_pddl)
    if len(route) < 2:
        return jsonify({'error': 'Nessun percorso da ricalcolare: risolvi prima il problema'}), 400
    if not goal_osm:
        return jsonify({'error': f'Goal "{goal_pddl}" non trovato'}), 400

    # --- insieme degli archi bloccati (in entrambi i sensi: strada chiusa) ---
    blocked_edges = set()
    for pair in data.get('blocked_edges') or []:
        a, b = nm_inv.get(pair[0]), nm_inv.get(pair[1])
        if a and b:
            blocked_edges.add((a, b))
            blocked_edges.add((b, a))
    blocked_nodes = {nm_inv[x] for x in (data.get('blocked_nodes') or []) if x in nm_inv}
    if not blocked_edges and not blocked_nodes:
        return jsonify({'error': 'Nessuna strada o incrocio bloccato'}), 400
    if goal_osm in blocked_nodes:
        return jsonify({'error': 'Il goal stesso e\' bloccato: scegli un altro punto'}), 400

    def is_blocked(a, b):
        return (a, b) in blocked_edges or a in blocked_nodes or b in blocked_nodes

    # --- primo punto del piano corrente colpito dal blocco ---
    osm_route = [nm_inv.get(r) for r in route]
    hit = None
    for i in range(len(osm_route) - 1):
        a, b = osm_route[i], osm_route[i + 1]
        if a and b and is_blocked(a, b):
            hit = i
            break
    if hit is None:
        return jsonify({'success': True, 'no_impact': True,
                        'message': 'Il percorso attuale non attraversa nessuna delle '
                                   'strade bloccate: nessun ricalcolo necessario.'})

    replan_from = osm_route[hit]
    prev_osm = osm_route[hit - 1] if hit > 0 else None
    if replan_from in blocked_nodes:
        return jsonify({'error': 'Il punto di partenza del ricalcolo e\' bloccato'}), 400

    # --- grafo senza gli elementi bloccati ---
    open_edges = {(a, b): v for (a, b), v in edges.items() if not is_blocked(a, b)}
    reach = {replan_from}
    q = deque([replan_from])
    while q:
        cur = q.popleft()
        for (a, b) in open_edges:
            if a == cur and b not in reach:
                reach.add(b)
                q.append(b)
    if goal_osm not in reach:
        return jsonify({
            'error': 'Con queste chiusure il goal non e\' piu\' raggiungibile: '
                     'il blocco isola la destinazione.',
            'unreachable': True,
            'blocked_at': nm.get(replan_from),
        }), 400

    # stesso sottografo locale usato da /api/solve — vedi commento li'. prev_osm
    # va sempre incluso come oggetto PDDL: write_pddl lo referenzia in (prev ...)
    # e nelle turn-time anche se non ha piu' un arco verso replan_from (strada
    # chiusa), quindi deve comunque comparire tra gli (:objects ...).
    local_nodes = select_local_subgraph(replan_from, goal_osm, open_edges, node_data)
    if prev_osm and prev_osm not in local_nodes:
        local_nodes = local_nodes + [prev_osm]
    local_set = set(local_nodes)
    local_open_edges = {(a, b): v for (a, b), v in open_edges.items() if a in local_set and b in local_set}

    pddl_content = write_pddl(
        zone, local_nodes, node_data, local_open_edges, replan_from, goal_osm, nm,
        signal_nodes=signal_nodes, congestion_delays=cong_delays,
        vehicle_counts=vc, intersection_density=density,
        peripheral=peripheral, edge_highway=sub_hw, prev_osm=prev_osm,
    )

    plan_text, new_route, plan_ms, err = run_enhsp(pddl_content)
    if err:
        return jsonify({'error': err}), 400

    # salva accanto agli altri, cosi' anche SUMO puo' visualizzare il nuovo piano
    try:
        os.makedirs(PDDL_DIR, exist_ok=True)
        with open(os.path.join(PDDL_DIR, 'problem_custom.pddl'), 'w', encoding='utf-8') as f:
            f.write(pddl_content)
        # Per SUMO si salva SOLO il nuovo percorso, non l'intero viaggio.
        # Motivo: problem_custom.pddl parte da replan_from, e sumo_visualize
        # centra la vista sullo start del PDDL. Salvando anche il tratto gia'
        # percorso, il veicolo nasceva lontano dall'inquadratura (fuori
        # schermo) e la rotta conteneva un ritorno sugli stessi archi, cioe'
        # un'inversione a U che SUMO non sempre puo' eseguire.
        # Cosi' invece l'auto parte esattamente dove avviene il ricalcolo.
        with open(os.path.join(PDDL_DIR, 'route_custom.json'), 'w', encoding='utf-8') as f:
            json.dump({'route': new_route or []}, f)
    except Exception:
        pass

    old_m = route_metrics(route, nm_inv, edges, vc, cong_delays, signal_nodes, node_data)
    new_full = route[:hit] + (new_route or [])      # tratto percorso + nuovo
    new_m = route_metrics(new_full, nm_inv, edges, vc, cong_delays, signal_nodes, node_data)

    return jsonify({
        'success': True,
        'blocked_at': nm.get(replan_from),          # ultimo nodo raggiungibile
        'blocked_edge': [nm.get(osm_route[hit]), nm.get(osm_route[hit + 1])],
        'hit_index': hit,
        'travelled': route[:hit + 1],               # gia' percorso (invariato)
        'new_route': new_route,                     # da blocked_at al goal
        'full_route': new_full,
        'plan_text': plan_text,
        'plan_time_ms': plan_ms,
        'pddl_content': pddl_content,
        'old_metrics': old_m,
        'new_metrics': new_m,
        'n_blocked_edges': len(blocked_edges) // 2,
        'n_blocked_nodes': len(blocked_nodes),
    })


@app.route('/api/sumo', methods=['POST'])
def launch_sumo():
    """Apre il percorso in sumo-gui.

    variant = 'optimized' (default) -> semafori ottimizzati (punto 3), cioe'
                                       carica cfg_files/tls_<zona>.add.xml
    variant = 'baseline'            -> semafori originali del net.xml
    In entrambi i casi si usa lo stesso sumo_visualize.py di sempre: cambia
    solo il flag --baseline."""
    data    = request.get_json(silent=True) or {}
    variant = data.get('variant', 'optimized')

    base   = os.path.dirname(os.path.abspath(__file__))
    script = os.path.join(base, '..', 'scripts', 'sumo_visualize.py')
    pddl   = os.path.join(base, '..', 'pddl_files', 'problem_custom.pddl')

    if not os.path.exists(script):
        return jsonify({'error': 'scripts/sumo_visualize.py non trovato'}), 400
    if not os.path.exists(pddl):
        return jsonify({'error': 'problem_custom.pddl non trovato'}), 400

    cmd = [sys.executable or 'python', os.path.abspath(script),
           'pddl', os.path.abspath(pddl), 'piccola']
    if variant == 'baseline':
        cmd.append('--baseline')

    try:
        subprocess.Popen(cmd, cwd=os.path.dirname(os.path.abspath(script)))
        return jsonify({'success': True, 'variant': variant})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


@app.route('/api/compare_sumo', methods=['POST'])
def compare_sumo_api():
    """PUNTO 4 — esegue in SUMO il confronto baseline vs semafori ottimizzati
    per la zona scelta e restituisce le metriche misurate.

    Il confronto usa il campione O-D condiviso della zona
    (sumo_extracted/demand_<zona>.json), non il singolo percorso disegnato
    dall'utente: misura la qualita' del PIANO SEMAFORICO, non di una singola
    corsa."""
    data = request.get_json(silent=True) or {}
    zone = data.get('zone', 'piccola')
    if zone not in ('piccola', 'media', 'grande'):
        return jsonify({'error': f'zona non valida: {zone}'}), 400

    root   = os.path.abspath(PROJECT_ROOT)
    script = os.path.join(root, 'scripts', 'compare_sumo.py')
    if not os.path.exists(script):
        return jsonify({'error': 'scripts/compare_sumo.py non trovato'}), 400

    add_file = os.path.join(root, 'cfg_files', f'tls_{zone}.add.xml')
    if not os.path.exists(add_file):
        return jsonify({'error': f"Semafori ottimizzati mancanti per '{zone}'. "
                                 f"Esegui: python scripts/inject_signal_plan.py {zone}"}), 400

    try:
        r = subprocess.run([sys.executable or 'python', script, zone],
                           capture_output=True, text=True, timeout=1800, cwd=root)
    except subprocess.TimeoutExpired:
        return jsonify({'error': 'Confronto SUMO oltre il timeout (30 min)'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500

    res_path = os.path.join(root, 'sumo_comparison', 'results.json')
    if not os.path.exists(res_path):
        return jsonify({'error': 'Nessun risultato prodotto',
                        'log': (r.stdout + r.stderr)[-800:]}), 500
    try:
        with open(res_path, encoding='utf-8') as f:
            all_res = json.load(f)
    except Exception as e:
        return jsonify({'error': f'results.json illeggibile: {e}'}), 500

    entry = next((x for x in all_res if x.get('zone') == zone), None)
    if not entry:
        return jsonify({'error': f"Nessun risultato per la zona '{zone}'",
                        'log': (r.stdout + r.stderr)[-800:]}), 500

    return jsonify({'success': True, 'zone': zone, 'result': entry,
                    'log': (r.stdout or '')[-1500:]})


def _parse_args():
    parser = argparse.ArgumentParser(
        prog='app.py',
        description='Interfaccia web (Flask + Leaflet) per Map Construction in PDDL+.',
    )
    parser.add_argument('--debug', action='store_true',
                         help='avvia Flask in debug mode (auto-reload + debugger '
                              'interattivo Werkzeug). Da usare solo in sviluppo, '
                              'non se il server e\' esposto oltre localhost.')
    return parser.parse_args()


if __name__ == '__main__':
    args = _parse_args()
    print(f"Server avviato su http://localhost:5000 (debug={'on' if args.debug else 'off'})")
    app.run(debug=args.debug, port=5000)

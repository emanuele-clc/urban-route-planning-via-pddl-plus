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

# in-memory store: token -> graph data
graph_store = {}


def route_seg_geom(route, nm_inv, edge_geom, node_data):
    """Geometria reale (polilinea lat/lon) di ogni tratto del percorso, nello
    stesso ordine del piano. Serve a SUMO per agganciare il percorso alle
    strade ESATTE scelte dal planner (map-matching in sumo_visualize): senza
    la forma della via, fra due incroci SUMO potrebbe imboccare una parallela
    piu' corta. Se un tratto non ha geometria salvata si ripiega sul segmento
    dritto fra i due nodi."""
    segs = []
    for i in range(len(route) - 1):
        a = nm_inv.get(route[i]); b = nm_inv.get(route[i + 1])
        g = None
        if a and b:
            g = edge_geom.get((a, b))
            if g is None and edge_geom.get((b, a)):
                g = list(reversed(edge_geom[(b, a)]))
        if not g and a and b and a in node_data and b in node_data:
            g = [[node_data[a]['lat'], node_data[a]['lon']],
                 [node_data[b]['lat'], node_data[b]['lon']]]
        segs.append(g or [])
    return segs


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
        return jsonify({'error': 'No file uploaded'}), 400

    tmp_dir  = tempfile.mkdtemp()
    osm_path = os.path.join(tmp_dir, 'input.osm')
    osm_file.save(osm_path)

    try:
        node_data, adj, signal_node_ids, edge_highway, edge_geom = build_contracted_graph(osm_path)
        if not adj:
            return jsonify({'error': 'No drivable road found in the OSM file'}), 400

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

        # pre-compute congestion (saved in the token)
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
            'edge_geom':    edge_geom,
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
            # real road geometry (lat/lon shape points) so the map can draw the
            # route along the actual streets, matching SUMO
            'geom': edge_geom.get((a, b)),
        } for (a, b), (d, spd) in edges.items()]

        return jsonify({
            'success': True, 'token': token,
            'nodes': nodes_out, 'edges': edges_out,
            'auto_start': nm[start_osm], 'auto_goal': nm[goal_osm],
            'stats': {'n_nodes': len(selected), 'n_edges': len(edges)},
        })

    except ET.ParseError as e:
        return jsonify({'error': f'Invalid OSM file: {e}'}), 400
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
        return jsonify({'error': 'Session expired, reload the OSM file'}), 400

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
    edge_geom    = store.get('edge_geom', {})

    start_osm = nm_inv.get(start_pddl)
    goal_osm  = nm_inv.get(goal_pddl)

    if not start_osm:
        return jsonify({'error': f'Start "{start_pddl}" not found'}), 400
    if not goal_osm:
        return jsonify({'error': f'Goal "{goal_pddl}" not found'}), 400
    if start_osm == goal_osm:
        return jsonify({'error': 'Start and Goal must be different nodes'}), 400

    reach = compute_reachable(start_osm, edges)
    if goal_osm not in reach:
        return jsonify({'error': f'Goal "{goal_pddl}" is not reachable from "{start_pddl}"'}), 400

    # Local subgraph for the start/goal pair, NOT the entire loaded graph
    # (which with "all nodes" on a large zone can have thousands of nodes
    # even for a route of a few hundred meters) — the cost of solving must
    # depend on the actual distance of the trip, not on the size of the
    # displayed map.
    local_nodes = select_local_subgraph(start_osm, goal_osm, edges, node_data)
    local_set = set(local_nodes)
    local_edges = {(a, b): v for (a, b), v in edges.items() if a in local_set and b in local_set}

    if len(local_nodes) > MAX_SOLVABLE_NODES:
        return jsonify({'error':
            f'Problem too large for ENHSP: {len(local_nodes)} nodes in the '
            f'local start-goal subgraph (maximum {MAX_SOLVABLE_NODES}). '
            f'The distance between the two points requires too many detours. '
            f'Increase the heap with ENHSP_HEAP and raise MAX_SOLVABLE_NODES.'}), 400

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
    used_fallback_heuristic = False

    jar        = trova_enhsp()
    domain_abs = os.path.abspath(DOMAIN_PATH)

    if not jar:
        enhsp_error = "ENHSP not found — install with: pip install up-enhsp"
    elif not os.path.exists(domain_abs):
        enhsp_error = f"domain.pddl not found at: {domain_abs}"
    else:
        try:
            output, used_fallback_heuristic = run_enhsp_output(jar, domain_abs, pddl_path, ENHSP_TIMEOUT)

            if "Problem Solved" in output:
                plan_text, route, plan_time_ms = parse_plan(output)

                # save the planned route (sequence of PDDL nodes) so that
                # SUMO can follow EXACTLY the same roads as the plan,
                # instead of recomputing its own start->goal Dijkstra.
                route_path = os.path.join(PDDL_DIR, 'route_custom.json')
                try:
                    # salvo anche le coordinate di ogni nodo: servono a SUMO
                    # per agganciare alla junction giusta i nodi che netconvert
                    # ha semplificato (start/goal compresi), altrimenti l'auto
                    # partirebbe da un altro punto.
                    coords = {}
                    for r in (route or []):
                        o = nm_inv.get(r)
                        if o and o in node_data:
                            coords[r] = [node_data[o]['lat'], node_data[o]['lon']]
                    seg_geom = route_seg_geom(route or [], nm_inv, edge_geom, node_data)
                    with open(route_path, 'w', encoding='utf-8') as f:
                        json.dump({'route': route or [], 'coords': coords,
                                   'seg_geom': seg_geom}, f)
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
                        # signal delay of the movement (prev,a_osm,b_osm) — paid
                        # when leaving a_osm, as in domain.pddl 'start-move' (sec. 3.1
                        # of 2_traffic_signal_optimization.md). i==0: no real prev
                        # (fictitious start-move start->start->b_osm).
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
            enhsp_error = (f"ENHSP exceeded the timeout ({ENHSP_TIMEOUT}s) — "
                           "reduce the nodes or raise ENHSP_TIMEOUT")
        except FileNotFoundError:
            enhsp_error = "Java not found — install Java 17+"

    # compute congestion summary along the route
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
        'used_fallback_heuristic': used_fallback_heuristic,
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
    """Recompute the route around roads/intersections that are now closed.

    The car is already mid-trip, so we don't restart from the origin: we find
    the first closed step of the current plan and replan from the node just
    before it. We also pass that previous node to ENHSP so the first turn
    isn't free."""
    data = request.get_json() or {}
    token = data.get('token')
    store = graph_store.get(token)
    if not store:
        return jsonify({'error': 'Session expired, reload the OSM file'}), 400

    nm, nm_inv = store['nm'], store['nm_inv']
    node_data, edges, selected = store['node_data'], store['edges'], store['selected']
    zone = store['zone']
    signal_nodes = store.get('signal_nodes', set())
    peripheral = store.get('peripheral', set())
    density = store.get('density', {})
    cong_delays = store.get('cong_delays', {})
    vc = store.get('vehicle_counts', {})
    sub_hw = store.get('edge_highway', {})
    edge_geom = store.get('edge_geom', {})

    route = data.get('route') or []
    goal_pddl = data.get('goal')
    goal_osm = nm_inv.get(goal_pddl)
    if len(route) < 2:
        return jsonify({'error': 'No route to recalculate: solve the problem first'}), 400
    if not goal_osm:
        return jsonify({'error': f'Goal "{goal_pddl}" not found'}), 400

    # a closed road is closed both ways
    blocked_edges = set()
    for pair in data.get('blocked_edges') or []:
        a, b = nm_inv.get(pair[0]), nm_inv.get(pair[1])
        if a and b:
            blocked_edges.add((a, b))
            blocked_edges.add((b, a))
    blocked_nodes = {nm_inv[x] for x in (data.get('blocked_nodes') or []) if x in nm_inv}
    if not blocked_edges and not blocked_nodes:
        return jsonify({'error': 'No road or intersection blocked'}), 400
    if goal_osm in blocked_nodes:
        return jsonify({'error': 'The goal itself is blocked: choose another point'}), 400

    def is_blocked(a, b):
        return (a, b) in blocked_edges or a in blocked_nodes or b in blocked_nodes

    # walk the current plan until we hit a closed road
    osm_route = [nm_inv.get(r) for r in route]
    hit = None
    for i in range(len(osm_route) - 1):
        a, b = osm_route[i], osm_route[i + 1]
        if a and b and is_blocked(a, b):
            hit = i
            break
    if hit is None:
        return jsonify({'success': True, 'no_impact': True,
                        'message': 'The current route does not cross any of the '
                                   'blocked roads: no recalculation needed.'})

    replan_from = osm_route[hit]
    prev_osm = osm_route[hit - 1] if hit > 0 else None
    if replan_from in blocked_nodes:
        return jsonify({'error': 'The starting point of the recalculation is blocked'}), 400

    # graph without the closed roads, then a BFS to make sure the goal is
    # still reachable from where we restart
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
            'error': 'With these closures the goal is no longer reachable: '
                     'the block isolates the destination.',
            'unreachable': True,
            'blocked_at': nm.get(replan_from),
        }), 400

    # only feed ENHSP the area around the route, like /api/solve does.
    # keep prev_osm among the objects even if its road is now closed:
    # write_pddl still uses it in (prev ...) and in the turn-times.
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

    plan_text, new_route, plan_ms, err, used_fallback_heuristic = run_enhsp(pddl_content)
    if err:
        return jsonify({'error': err}), 400

    # also save it so "open in SUMO" can show the new plan
    try:
        os.makedirs(PDDL_DIR, exist_ok=True)
        with open(os.path.join(PDDL_DIR, 'problem_custom.pddl'), 'w', encoding='utf-8') as f:
            f.write(pddl_content)
        # save only the new leg, not the whole trip: the pddl starts at
        # replan_from and SUMO centres the view there, so the car has to spawn
        # at the reroute point (adding the old leg also made it do a U-turn).
        coords = {}
        for r in (new_route or []):
            o = nm_inv.get(r)
            if o and o in node_data:
                coords[r] = [node_data[o]['lat'], node_data[o]['lon']]
        seg_geom = route_seg_geom(new_route or [], nm_inv, edge_geom, node_data)
        with open(os.path.join(PDDL_DIR, 'route_custom.json'), 'w', encoding='utf-8') as f:
            json.dump({'route': new_route or [], 'coords': coords,
                       'seg_geom': seg_geom}, f)
    except Exception:
        pass

    old_m = route_metrics(route, nm_inv, edges, vc, cong_delays, signal_nodes, node_data)
    new_full = route[:hit] + (new_route or [])      # travelled segment + new one
    new_m = route_metrics(new_full, nm_inv, edges, vc, cong_delays, signal_nodes, node_data)

    return jsonify({
        'success': True,
        'blocked_at': nm.get(replan_from),          # last reachable node
        'blocked_edge': [nm.get(osm_route[hit]), nm.get(osm_route[hit + 1])],
        'hit_index': hit,
        'travelled': route[:hit + 1],               # already travelled (unchanged)
        'new_route': new_route,                     # from blocked_at to the goal
        'full_route': new_full,
        'plan_text': plan_text,
        'plan_time_ms': plan_ms,
        'used_fallback_heuristic': used_fallback_heuristic,
        'pddl_content': pddl_content,
        'old_metrics': old_m,
        'new_metrics': new_m,
        'n_blocked_edges': len(blocked_edges) // 2,
        'n_blocked_nodes': len(blocked_nodes),
    })


def _sumo_cmd(variant, traffic_scale):
    """Builds the common argv prefix for scripts/sumo_visualize.py shared by
    the live (/api/sumo) and video (/api/sumo_video) endpoints.

    variant = 'optimized' (default) -> optimized signals (point 3), i.e.
                                       loads cfg_files/tls_<zone>.add.xml
    variant = 'baseline'            -> original signals from net.xml
    traffic_scale = 0    -> no background traffic (--traffic 0)
    traffic_scale = 1    -> default congestion (unchanged from before)
    traffic_scale > 1    -> proportionally more background vehicles, see
        generate_background_traffic in sumo_visualize.py and
        5_traffico_sfondo_sumo.md §11.

    Every call gets its own --run-id: without it, two requests fired close
    together (live+video, or two videos) would regenerate the SAME fixed
    cfg/rou/gui/frames files for the zone and could overwrite each other's
    inputs while still running (see 5_traffico_sfondo_sumo.md §12)."""
    base   = os.path.dirname(os.path.abspath(__file__))
    script = os.path.join(base, '..', 'scripts', 'sumo_visualize.py')
    pddl   = os.path.join(base, '..', 'pddl_files', 'problem_custom.pddl')

    if not os.path.exists(script):
        return None, None, None, 'scripts/sumo_visualize.py not found'
    if not os.path.exists(pddl):
        return None, None, None, 'problem_custom.pddl not found'

    run_id = uuid.uuid4().hex[:8]
    cmd = [sys.executable or 'python', os.path.abspath(script),
           'pddl', os.path.abspath(pddl), 'piccola']
    if variant == 'baseline':
        cmd.append('--baseline')
    cmd += ['--traffic', str(traffic_scale), '--run-id', run_id]
    return cmd, os.path.dirname(os.path.abspath(script)), run_id, None


@app.route('/api/sumo', methods=['POST'])
def launch_sumo():
    """Opens the route in an interactive sumo-gui window (fire-and-forget:
    the process keeps running after this request returns). See _sumo_cmd
    for variant/traffic_scale semantics."""
    data    = request.get_json(silent=True) or {}
    variant = data.get('variant', 'optimized')
    traffic_scale = data.get('traffic_scale', 1)

    cmd, cwd, run_id, err = _sumo_cmd(variant, traffic_scale)
    if err:
        return jsonify({'error': err}), 400

    try:
        subprocess.Popen(cmd, cwd=cwd)
        return jsonify({'success': True, 'variant': variant})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


VIDEO_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), 'static', 'videos')


@app.route('/api/sumo_video', methods=['POST'])
def launch_sumo_video():
    """Same simulation as /api/sumo, but instead of an interactive window it
    records it (sumo-gui automated snapshots + ffmpeg, see --video in
    sumo_visualize.py) and returns a URL to the resulting mp4. This request
    blocks until the video is ready (typically a few seconds), unlike
    /api/sumo which returns immediately. Decoupling capture from real-time
    GUI rendering is what lets this mode handle much more background
    traffic without becoming unwatchable — see 5_traffico_sfondo_sumo.md §11."""
    data    = request.get_json(silent=True) or {}
    variant = data.get('variant', 'optimized')
    traffic_scale = data.get('traffic_scale', 1)

    cmd, cwd, run_id, err = _sumo_cmd(variant, traffic_scale)
    if err:
        return jsonify({'error': err}), 400

    os.makedirs(VIDEO_DIR, exist_ok=True)
    final_path = os.path.join(VIDEO_DIR, f'sumo_{variant}.mp4')
    # scrive prima su un percorso univoco per questa richiesta (run_id): se
    # un'altra richiesta per la STESSA variant e' in corso in parallelo, i
    # due ffmpeg non scrivono mai sullo stesso file a meta' (vedi §12).
    tmp_path = os.path.join(VIDEO_DIR, f'.tmp_{variant}_{run_id}.mp4')
    cmd += ['--video-out', tmp_path]

    try:
        r = subprocess.run(cmd, cwd=cwd, capture_output=True, text=True, timeout=180)
    except subprocess.TimeoutExpired:
        if os.path.exists(tmp_path):
            os.remove(tmp_path)
        return jsonify({'error': 'Video generation exceeded the timeout (180s)'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500

    if not os.path.exists(tmp_path):
        return jsonify({'error': 'Video was not generated',
                        'log': (r.stdout + r.stderr)[-1500:]}), 500

    # pubblicazione atomica: il file servito al posto fisso (sumo_<variant>.mp4)
    # passa in un solo colpo dalla versione precedente a quella nuova, mai a
    # meta' scrittura, anche se un'altra richiesta lo sta leggendo in quel momento.
    os.replace(tmp_path, final_path)

    return jsonify({
        'success': True, 'variant': variant,
        'url': f'/static/videos/sumo_{variant}.mp4?t={int(os.path.getmtime(final_path))}',
    })


@app.route('/api/compare_sumo', methods=['POST'])
def compare_sumo_api():
    """POINT 4 — runs the baseline vs optimized-signals comparison in SUMO
    for the chosen zone and returns the measured metrics.

    The comparison uses the zone's shared O-D sample
    (sumo_extracted/demand_<zone>.json), not the single route drawn by the
    user: it measures the quality of the SIGNAL PLAN, not of a single
    run."""
    data = request.get_json(silent=True) or {}
    zone = data.get('zone', 'piccola')
    if zone not in ('piccola', 'media', 'grande'):
        return jsonify({'error': f'invalid zone: {zone}'}), 400

    root   = os.path.abspath(PROJECT_ROOT)
    script = os.path.join(root, 'scripts', 'compare_sumo.py')
    if not os.path.exists(script):
        return jsonify({'error': 'scripts/compare_sumo.py not found'}), 400

    add_file = os.path.join(root, 'cfg_files', f'tls_{zone}.add.xml')
    if not os.path.exists(add_file):
        return jsonify({'error': f"Optimized signals missing for '{zone}'. "
                                 f"Run: python scripts/inject_signal_plan.py {zone}"}), 400

    try:
        r = subprocess.run([sys.executable or 'python', script, zone],
                           capture_output=True, text=True, timeout=1800, cwd=root)
    except subprocess.TimeoutExpired:
        return jsonify({'error': 'SUMO comparison exceeded the timeout (30 min)'}), 500
    except Exception as e:
        return jsonify({'error': str(e)}), 500

    res_path = os.path.join(root, 'sumo_comparison', 'results.json')
    if not os.path.exists(res_path):
        return jsonify({'error': 'No result produced',
                        'log': (r.stdout + r.stderr)[-800:]}), 500
    try:
        with open(res_path, encoding='utf-8') as f:
            all_res = json.load(f)
    except Exception as e:
        return jsonify({'error': f'results.json unreadable: {e}'}), 500

    entry = next((x for x in all_res if x.get('zone') == zone), None)
    if not entry:
        return jsonify({'error': f"No result for zone '{zone}'",
                        'log': (r.stdout + r.stderr)[-800:]}), 500

    return jsonify({'success': True, 'zone': zone, 'result': entry,
                    'log': (r.stdout or '')[-1500:]})


def _parse_args():
    parser = argparse.ArgumentParser(
        prog='app.py',
        description='Web interface (Flask + Leaflet) for Map Construction in PDDL+.',
    )
    parser.add_argument('--debug', action='store_true',
                         help='start Flask in debug mode (auto-reload + interactive '
                              'Werkzeug debugger). Use only in development, '
                              'not if the server is exposed beyond localhost.')
    return parser.parse_args()


if __name__ == '__main__':
    args = _parse_args()
    print(f"Server started at http://localhost:5000 (debug={'on' if args.debug else 'off'})")
    app.run(debug=args.debug, port=5000)

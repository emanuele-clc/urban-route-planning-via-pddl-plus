"""
pddl_writer.py
--------------
Generazione del problema PDDL+ dal grafo contratto (write_pddl) e
scomposizione del costo di un percorso secondo la stessa formula del
dominio (route_metrics), usata dal pannello di replanning per confrontare
piano vecchio e nuovo. Le due funzioni condividono la logica di ritardo
semaforico/svolta: vanno tenute in sincronia — vedi i commenti in
domain.pddl e 2_traffic_signal_optimization.md, sez. 1.
Estratto da webapp/app.py.
"""
from collections import defaultdict

from osm_graph import N_VEHICLES, PERIPH_RADIUS_M, DENSITY_RADIUS_M, RANDOM_SEED
from sumo_signals import (
    TURN_RATE_DPS, DEFAULT_SIGNAL_DELAY, SUMO_DELAYS, SUMO_MOVEMENTS,
    signal_delay_for, assign_movement_signal_delay, turn_time_s,
)


def write_pddl(zone, selected, node_data, edges, start_osm, goal_osm, nm,
               signal_nodes=None, congestion_delays=None, vehicle_counts=None,
               intersection_density=None, peripheral=None, edge_highway=None,
               prev_osm=None):
    """prev_osm: nodo da cui si proviene arrivando in start_osm. Serve al
    REPLANNING: ripartendo da meta' percorso il veicolo ha gia' un
    orientamento, quindi la prima svolta ha un costo reale. Se None (piano
    iniziale, veicolo fermo) si usa start_osm stesso e la prima svolta e' 0."""

    if signal_nodes         is None: signal_nodes         = set()
    if congestion_delays    is None: congestion_delays    = {}
    if vehicle_counts       is None: vehicle_counts       = {}
    if intersection_density is None: intersection_density = {}
    if peripheral           is None: peripheral           = set()
    if edge_highway         is None: edge_highway         = {}

    def n(x): return nm[x]
    se = sorted(edges.keys(), key=lambda e: (n(e[0]), n(e[1])))
    selected_signals = [nd for nd in selected if nd in signal_nodes]

    lines = []
    lines.append(f"(define (problem dublin-{zone})")
    lines.append("  (:domain dublin-navigation)")
    lines.append("")
    lines.append(f"  ; {len(selected)} nodi — zona {zone}")
    lines.append(f"  ; START: {n(start_osm)}   GOAL: {n(goal_osm)}")
    lines.append(f"  ; Congestione: {N_VEHICLES} veicoli random, raggio periferia {PERIPH_RADIUS_M}m")
    lines.append("")
    lines.append("  (:objects")
    for nd in selected:
        nd_name = node_data[nd]["name"]
        lines.append(f"    {n(nd)}  ; {nd_name}" if nd_name else f"    {n(nd)}")
    lines.append("    - location")
    lines.append("  )")
    lines.append("")
    prev_node = prev_osm if (prev_osm and prev_osm in nm) else start_osm
    lines.append("  (:init")
    lines.append(f"    (at {n(start_osm)})")
    if prev_node == start_osm:
        lines.append(f"    (prev {n(start_osm)})   ; nessuna svolta prima del primo arco")
    else:
        lines.append(f"    (prev {n(prev_node)})   ; replanning: si proviene da qui")
    lines.append("    (= (total-dist) 0)")
    lines.append("    (= (total-time) 0)")

    periph_in_sub = [nd for nd in selected if nd in peripheral]
    if periph_in_sub:
        lines.append("")
        lines.append(f"    ; Zona periferica (> {PERIPH_RADIUS_M}m dal centroide)")
        for nd in periph_in_sub:
            lines.append(f"    (peripheral {n(nd):<28})")

    lines.append("")
    lines.append("    ; Progress = 0 per ogni tratto")
    for a, b in se:
        lines.append(f"    (= (progress {n(a):<28} {n(b)}) 0)")
    lines.append("")
    lines.append("    ; Strade")
    for a, b in se:
        lines.append(f"    (road {n(a):<28} {n(b)})")
    lines.append("")
    lines.append("    ; Distanze (metri)")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (distance {n(a):<28} {n(b)}) {d})")
    lines.append("")
    lines.append("    ; Velocita' base (m/s)")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (speed {n(a):<28} {n(b)}) {spd})")
    # --- out_adj/in_adj/triples: servono sia al blocco signal-delay
    # (per movimento) sia al blocco turn-time piu' sotto ---
    out_adj = defaultdict(list); in_adj = defaultdict(list)
    for (a, b) in edges:
        out_adj[a].append(b); in_adj[b].append(a)
    triples = [(a, b, c) for b in selected
               for a in in_adj.get(b, [])
               for c in out_adj.get(b, [])]

    def _signal_delay_fallback(nd):
        return signal_delay_for(nd, signal_nodes)

    lines.append("")
    n_from_sumo = sum(1 for nd in selected if nd in SUMO_DELAYS)
    n_with_movements = sum(1 for nd in selected if nd in SUMO_MOVEMENTS)
    lines.append(f"    ; Ritardo semaforico (s) per MOVIMENTO (prev,from,to) — "
                 f"{len(selected_signals)}/{len(selected)} nodi")
    lines.append(f"    ; Realistico da SUMO/SCATS per fase/direzione "
                 f"({n_with_movements} nodi con dati di movimento, {n_from_sumo} con media); "
                 f"default {DEFAULT_SIGNAL_DELAY}s per gli altri semafori")
    for c in sorted(out_adj.get(start_osm, []), key=n):
        delay = assign_movement_signal_delay(start_osm, start_osm, c, node_data, SUMO_MOVEMENTS, is_first=True)
        if delay is None:
            delay = _signal_delay_fallback(start_osm)
        lines.append(f"    (= (signal-delay {n(start_osm):<20} {n(start_osm):<20} {n(c)}) {delay})")
    for a, b, c in sorted(triples, key=lambda t: (n(t[0]), n(t[1]), n(t[2]))):
        delay = assign_movement_signal_delay(a, b, c, node_data, SUMO_MOVEMENTS)
        if delay is None:
            delay = _signal_delay_fallback(b)
        lines.append(f"    (= (signal-delay {n(a):<20} {n(b):<20} {n(c)}) {delay})")
    lines.append("")
    lines.append("    ; Ritardo congestione statico (s)")
    for nd in selected:
        cd = congestion_delays.get(nd, 0)
        lines.append(f"    (= (congestion-delay {n(nd):<28}) {cd})")
    lines.append("")
    lines.append(f"    ; Densita' incroci (n. nodi entro {DENSITY_RADIUS_M}m)")
    for nd in selected:
        dens = intersection_density.get(nd, 0)
        lines.append(f"    (= (intersection-density {n(nd):<28}) {dens})")
    lines.append("")
    lines.append(f"    ; Veicoli per arco ({N_VEHICLES} Dijkstra random, seed={RANDOM_SEED})")
    for a, b in se:
        vc = vehicle_counts.get((a, b), 0)
        lines.append(f"    (= (vehicle-count {n(a):<28} {n(b)}) {vc})")
    lines.append("")
    lines.append("    ; Fattore congestione = 1 + vehicle-count/10")
    for a, b in se:
        vc = vehicle_counts.get((a, b), 0)
        cf = round(1.0 + vc / 10.0, 2)
        lines.append(f"    (= (congestion-factor {n(a):<28} {n(b)}) {cf})")
    lines.append("")
    lines.append("    ; Velocita' effettiva (m/s) = speed / congestion-factor  [usata dal processo]")
    for a, b in se:
        d, spd = edges[(a, b)]
        vc = vehicle_counts.get((a, b), 0)
        cf = 1.0 + vc / 10.0
        eff = round(spd / cf, 4)
        lines.append(f"    (= (effective-speed {n(a):<28} {n(b)}) {eff})")
    lines.append("")
    lines.append("    ; Tempo di percorrenza (s) = distance / effective-speed  [usato nell'evento]")
    for a, b in se:
        d, spd = edges[(a, b)]
        vc = vehicle_counts.get((a, b), 0)
        cf = 1.0 + vc / 10.0
        eff = spd / cf
        arc_t = round(d / eff, 4) if eff > 0 else 0
        lines.append(f"    (= (arc-time {n(a):<28} {n(b)}) {arc_t})")

    # ── Turn-time: tempo di svolta per ogni tripla di nodi consecutivi ──────
    # (out_adj/in_adj/triples gia' calcolati sopra, per il blocco signal-delay)
    lines.append("")
    lines.append(f"    ; Tempo di svolta (s) = angolo / {TURN_RATE_DPS:.0f} deg/s  [turn rate]")
    emitted = set()
    for c in sorted(out_adj.get(start_osm, []), key=n):
        lines.append(f"    (= (turn-time {n(start_osm):<20} {n(start_osm):<20} {n(c)}) 0)")
        emitted.add((start_osm, start_osm, c))
    # Replanning: il nodo di provenienza puo' non avere piu' un arco verso
    # start (es. proprio quella strada e' stata chiusa), quindi le triple
    # (prev, start, *) vanno emesse esplicitamente: senza, ENHSP troverebbe
    # turn-time non definita sulla prima mossa.
    if prev_node != start_osm:
        for c in sorted(out_adj.get(start_osm, []), key=n):
            if (prev_node, start_osm, c) in emitted:
                continue
            tt = turn_time_s(prev_node, start_osm, c, node_data)
            lines.append(f"    (= (turn-time {n(prev_node):<20} {n(start_osm):<20} {n(c)}) {tt})")
            emitted.add((prev_node, start_osm, c))
    for a, b, c in sorted(triples, key=lambda t: (n(t[0]), n(t[1]), n(t[2]))):
        if (a, b, c) in emitted:
            continue
        tt = turn_time_s(a, b, c, node_data)
        lines.append(f"    (= (turn-time {n(a):<20} {n(b):<20} {n(c)}) {tt})")
        emitted.add((a, b, c))

    lines.append("")
    lines.append("  )")
    lines.append("")
    lines.append(f"  (:goal (at {n(goal_osm)}))")
    lines.append("")
    lines.append("  (:metric minimize (total-time))")
    lines.append(")")
    return "\n".join(lines)


def route_metrics(route, nm_inv, edges, vc, cong_delays, signal_nodes, node_data):
    """Scompone il costo di un percorso (stessa formula del dominio PDDL+):
    guida + semafori + congestione + svolte. Il ritardo semaforico usa
    assign_movement_signal_delay (per movimento, come /api/solve e come il
    dominio PDDL+ reale) e ricade su signal_delay_for (media per nodo) solo
    se il nodo non ha dati di movimento SUMO — stessa logica dei due punti,
    per non riportare nel pannello di replanning una stima meno precisa di
    quella usata per generare il piano."""
    out = {'dist': 0, 'drive': 0.0, 'signal': 0.0, 'cong': 0.0, 'turn': 0.0,
           'signals_crossed': 0}
    osm = [nm_inv.get(r) for r in (route or [])]
    for i in range(len(osm) - 1):
        a, b = osm[i], osm[i + 1]
        if a and b and (a, b) in edges:
            d, spd = edges[(a, b)]
            out['dist'] += d
            cf = 1.0 + vc.get((a, b), 0) / 10.0
            eff = spd / cf
            if eff > 0:
                out['drive'] += d / eff
        if not (a and b):
            continue
        if i == 0:
            sd = assign_movement_signal_delay(a, a, b, node_data, SUMO_MOVEMENTS, is_first=True)
        else:
            p = osm[i - 1]
            out['turn'] += turn_time_s(p, a, b, node_data)
            sd = assign_movement_signal_delay(p, a, b, node_data, SUMO_MOVEMENTS)
        if sd is None:
            sd = signal_delay_for(a, signal_nodes)
        if sd > 0:
            out['signals_crossed'] += 1
            out['signal'] += sd
    for nd in osm[1:]:
        if nd:
            out['cong'] += cong_delays.get(nd, 0)
    out['total'] = out['drive'] + out['signal'] + out['cong'] + out['turn']
    for k in ('drive', 'signal', 'cong', 'turn', 'total'):
        out[k] = round(out[k], 1)
    return out

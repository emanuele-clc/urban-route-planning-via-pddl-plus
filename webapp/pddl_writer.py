"""
pddl_writer.py
--------------
Generation of the PDDL+ problem from the contracted graph (write_pddl) and
decomposition of a route's cost following the same formula as the domain
(route_metrics), used by the replanning panel to compare the old and new
plan. The two functions share the signal/turn delay logic: they must be
kept in sync — see the comments in domain.pddl and
2_traffic_signal_optimization.md, sec. 1.
Extracted from webapp/app.py.
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
    """prev_osm is the node we come from when arriving at start_osm. It's used
    for replanning: mid-route the car already has a heading, so its first turn
    costs something. When it's None (fresh plan, car stopped) we just use
    start_osm and the first turn is 0."""

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
    lines.append(f"  ; {len(selected)} nodes — zone {zone}")
    lines.append(f"  ; START: {n(start_osm)}   GOAL: {n(goal_osm)}")
    lines.append(f"  ; Congestion: {N_VEHICLES} random vehicles, peripheral radius {PERIPH_RADIUS_M}m")
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
        lines.append(f"    (prev {n(start_osm)})   ; no turn before the first edge")
    else:
        lines.append(f"    (prev {n(prev_node)})   ; replanning: coming from here")
    lines.append("    (= (total-dist) 0)")
    lines.append("    (= (total-time) 0)")

    periph_in_sub = [nd for nd in selected if nd in peripheral]
    if periph_in_sub:
        lines.append("")
        lines.append(f"    ; Peripheral zone (> {PERIPH_RADIUS_M}m from centroid)")
        for nd in periph_in_sub:
            lines.append(f"    (peripheral {n(nd):<28})")

    lines.append("")
    lines.append("    ; Progress = 0 for each segment")
    for a, b in se:
        lines.append(f"    (= (progress {n(a):<28} {n(b)}) 0)")
    lines.append("")
    lines.append("    ; Roads")
    for a, b in se:
        lines.append(f"    (road {n(a):<28} {n(b)})")
    lines.append("")
    lines.append("    ; Distances (meters)")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (distance {n(a):<28} {n(b)}) {d})")
    lines.append("")
    lines.append("    ; Base speed (m/s)")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (speed {n(a):<28} {n(b)}) {spd})")
    # --- out_adj/in_adj/triples: needed both for the signal-delay block
    # (per movement) and for the turn-time block further below ---
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
    lines.append(f"    ; Signal delay (s) per MOVEMENT (prev,from,to) — "
                 f"{len(selected_signals)}/{len(selected)} nodes")
    lines.append(f"    ; Realistic from SUMO/SCATS per phase/direction "
                 f"({n_with_movements} nodes with movement data, {n_from_sumo} with average); "
                 f"default {DEFAULT_SIGNAL_DELAY}s for other signals")
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
    lines.append("    ; Static congestion delay (s)")
    for nd in selected:
        cd = congestion_delays.get(nd, 0)
        lines.append(f"    (= (congestion-delay {n(nd):<28}) {cd})")
    lines.append("")
    lines.append(f"    ; Intersection density (n. nodes within {DENSITY_RADIUS_M}m)")
    for nd in selected:
        dens = intersection_density.get(nd, 0)
        lines.append(f"    (= (intersection-density {n(nd):<28}) {dens})")
    lines.append("")
    lines.append(f"    ; Vehicles per edge ({N_VEHICLES} random Dijkstra, seed={RANDOM_SEED})")
    for a, b in se:
        vc = vehicle_counts.get((a, b), 0)
        lines.append(f"    (= (vehicle-count {n(a):<28} {n(b)}) {vc})")
    lines.append("")
    lines.append("    ; Congestion factor = 1 + vehicle-count/10")
    for a, b in se:
        vc = vehicle_counts.get((a, b), 0)
        cf = round(1.0 + vc / 10.0, 2)
        lines.append(f"    (= (congestion-factor {n(a):<28} {n(b)}) {cf})")
    lines.append("")
    lines.append("    ; Effective speed (m/s) = speed / congestion-factor  [used by the process]")
    for a, b in se:
        d, spd = edges[(a, b)]
        vc = vehicle_counts.get((a, b), 0)
        cf = 1.0 + vc / 10.0
        eff = round(spd / cf, 4)
        lines.append(f"    (= (effective-speed {n(a):<28} {n(b)}) {eff})")
    lines.append("")
    lines.append("    ; Travel time (s) = distance / effective-speed  [used in the event]")
    for a, b in se:
        d, spd = edges[(a, b)]
        vc = vehicle_counts.get((a, b), 0)
        cf = 1.0 + vc / 10.0
        eff = spd / cf
        arc_t = round(d / eff, 4) if eff > 0 else 0
        lines.append(f"    (= (arc-time {n(a):<28} {n(b)}) {arc_t})")

    # ── Turn-time: turning time for every triple of consecutive nodes ──────
    # (out_adj/in_adj/triples already computed above, for the signal-delay block)
    lines.append("")
    lines.append(f"    ; Turn time (s) = angle / {TURN_RATE_DPS:.0f} deg/s  [turn rate]")
    emitted = set()
    for c in sorted(out_adj.get(start_osm, []), key=n):
        lines.append(f"    (= (turn-time {n(start_osm):<20} {n(start_osm):<20} {n(c)}) 0)")
        emitted.add((start_osm, start_osm, c))
    # Replanning: the node of origin may no longer have an edge toward start
    # (e.g. that very road was closed), so the (prev, start, *) triples must
    # be emitted explicitly: without them, ENHSP would find turn-time
    # undefined on the first move.
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
    """Decomposes a route's cost (same formula as the PDDL+ domain): driving
    + signals + congestion + turns. The signal delay uses
    assign_movement_signal_delay (per movement, as in /api/solve and in the
    real PDDL+ domain) and falls back to signal_delay_for (per-node
    average) only if the node has no SUMO movement data — same logic as
    both places, to avoid showing the replanning panel a less accurate
    estimate than the one used to generate the plan."""
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

"""
signal_optimization/webster_screen.py
======================================
Stadio 1 (economico) della pipeline di valutazione candidati (Criticita' #4,
P1 — 2_traffic_signal_optimization.md sez. 3.4): stima il costo aggregato di
una configurazione candidata sommando i ritardi di Webster lungo percorsi di
riferimento PRECALCOLATI (Dijkstra su distanza, come proxy del percorso che
il planner sceglierebbe) — nessuna invocazione di ENHSP. Usato per scartare
le configurazioni chiaramente dominate prima della validazione costosa con
ENHSP (stadio 2, vedi enhsp_eval.py).
"""

import os
import sys
import json
from collections import defaultdict

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(BASE, "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

import build_problems as bp  # noqa: E402
import extract_sumo_data as esd  # noqa: E402
from extract_sumo_data import uniform_delay, REAL_CYCLE_S  # noqa: E402
from generate_demand import ZONE_CONFIGS  # noqa: E402

OSM_DIR = os.path.join(BASE, "osm_files")
SUMO_DIR = os.path.join(BASE, "sumo_extracted")


def bearing_bucket(angle_deg, n_buckets=8):
    if angle_deg is None:
        return None
    step = 360.0 / n_buckets
    a = angle_deg % 360.0
    return int(round(a / step)) % n_buckets * step


def circ_dist(a, b):
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def load_movements_by_node(sumo_data):
    """{osm_id: [(tl_id, movement), ...]} — stessa espansione cluster di
    build_problems.load_sumo_movements, qui tenendo anche il tl_id (serve
    per guardare la durata di fase del PIANO CANDIDATO, non solo il
    delay_s statico gia' presente nel json)."""
    out = defaultdict(list)
    for tid, tl in sumo_data.get("traffic_lights", {}).items():
        members = esd.cluster_member_ids(tid)
        for mv in tl.get("movements", []):
            if mv.get("bearing_in_bucket") is None or mv.get("bearing_out_bucket") is None:
                continue
            for nid in members:
                out[nid].append((tid, mv))
    return dict(out)


def best_matching_movement(a, b, c, node_data, movements_by_node, is_first=False):
    """(tl_id, movement) piu' vicino per bearing al movimento reale (a,b,c)
    — stessa logica di build_problems.assign_movement_signal_delay, ma
    ritorna anche il tl_id (necessario per rileggere la fase dal piano
    candidato)."""
    mv_list = movements_by_node.get(b)
    if not mv_list:
        return None
    out_bearing = bp.bearing(node_data[b]["lat"], node_data[b]["lon"],
                             node_data[c]["lat"], node_data[c]["lon"])
    out_bucket = bearing_bucket(out_bearing)
    if is_first:
        return min(mv_list, key=lambda tm: circ_dist(tm[1]["bearing_out_bucket"], out_bucket))
    in_bearing = bp.bearing(node_data[a]["lat"], node_data[a]["lon"],
                            node_data[b]["lat"], node_data[b]["lon"])
    in_bucket = bearing_bucket(in_bearing)
    return min(mv_list, key=lambda tm: circ_dist(tm[1]["bearing_in_bucket"], in_bucket)
                                       + circ_dist(tm[1]["bearing_out_bucket"], out_bucket))


def movement_delay_for_plan(tid, mv, plan, real_cycle=REAL_CYCLE_S):
    """Ritardo di Webster del movimento 'mv' (del tlLogic 'tid') SOTTO il
    piano candidato 'plan' = {tl_id: {phase_idx: duration_s}}. Se il
    movimento non e' controllato dal piano (fase fissa/non ottimizzata),
    ricade sul delay_s statico gia' calcolato da extract_sumo_data.py."""
    phase_idx = mv.get("phase_idx")
    if phase_idx is None or tid not in plan or phase_idx not in plan[tid]:
        return mv["delay_s"]
    green = plan[tid][phase_idx]
    red = max(real_cycle - green, 0.0)
    return uniform_delay(red, real_cycle)


def path_triples(path):
    """[(prev, from, to, is_first)] lungo un percorso di nodi — stessa
    struttura (prima tripla fittizia start-start-c) usata da
    build_problems.write_pddl per turn-time/signal-delay."""
    triples = []
    if len(path) < 2:
        return triples
    triples.append((path[0], path[0], path[1], True))
    for i in range(1, len(path) - 1):
        triples.append((path[i - 1], path[i], path[i + 1], False))
    return triples


def build_zone_context(zone):
    """Ricostruisce lo stesso sottografo PDDL usato da build_problems.py
    per la zona, piu' i dati di movimento SUMO — contesto condiviso da
    screening (qui) e validazione ENHSP (enhsp_eval.py)."""
    osm_file, max_nodes = ZONE_CONFIGS[zone]
    osm_path = os.path.join(OSM_DIR, osm_file)
    node_data, adj, signal_ids, edge_hw = bp.build_contracted_graph(osm_path)
    selected = bp.select_connected_subgraph(node_data, adj, max_nodes)
    sel_set = set(selected)
    edges = {}
    for a in selected:
        for b, (d, spd) in adj.get(a, {}).items():
            if b in sel_set:
                edges[(a, b)] = (d, spd)

    sumo_data_path = os.path.join(SUMO_DIR, f"sumo_data_{zone}.json")
    with open(sumo_data_path, encoding="utf-8") as f:
        sumo_data = json.load(f)

    edge_highway = {(a, b): edge_hw.get((a, b), "unclassified") for (a, b) in edges}

    return {
        "zone": zone,
        "node_data": node_data,
        "selected": selected,
        "edges": edges,
        "edge_highway": edge_highway,
        "signal_node_ids": signal_ids & sel_set,
        "sumo_data": sumo_data,
        "tls_data": sumo_data.get("traffic_lights", {}),
        "movements_by_node": load_movements_by_node(sumo_data),
    }


def reference_paths(ctx, od_pairs):
    """Percorso Dijkstra (su distanza, proxy del percorso scelto dal
    planner) per ciascuna coppia O-D del campione condiviso
    (generate_demand.py, criticita' #3). O-D irraggiungibili sono
    scartate silenziosamente."""
    adj_sub = defaultdict(dict)
    for (a, b), (d, _spd) in ctx["edges"].items():
        adj_sub[a][b] = d

    paths = []
    for pair in od_pairs:
        s, g = pair["start"], pair["goal"]
        if s not in ctx["selected"] or g not in ctx["selected"]:
            continue
        _dist, prev = bp.dijkstra(s, adj_sub)
        if g != s and g not in prev:
            continue
        path = bp.reconstruct_path(prev, g)
        if len(path) >= 2:
            paths.append(path)
    return paths


def per_path_scores(ctx, plan, paths, real_cycle=REAL_CYCLE_S):
    """Costo di segnalazione (somma ritardi Webster) per ciascun percorso,
    sotto il piano candidato 'plan'."""
    scores = []
    for path in paths:
        total = 0.0
        for a, b, c, is_first in path_triples(path):
            match = best_matching_movement(a, b, c, ctx["node_data"],
                                            ctx["movements_by_node"], is_first=is_first)
            if match is None:
                continue
            tid, mv = match
            total += movement_delay_for_plan(tid, mv, plan, real_cycle)
        scores.append(total)
    return scores


def score_plan(ctx, plan, paths, real_cycle=REAL_CYCLE_S):
    """Fitness aggregata (media del costo di segnalazione sui percorsi di
    riferimento) — piu' basso e' meglio. Usata per ordinare/filtrare i
    candidati prima della validazione ENHSP (stadio 2)."""
    scores = per_path_scores(ctx, plan, paths, real_cycle)
    return sum(scores) / len(scores) if scores else 0.0


def dominates(scores_a, scores_b, tol=1e-6):
    """True se 'a' e' <= 'b' su OGNI percorso e < su almeno uno (dominanza
    in senso Pareto sul campione O-D) — permette di scartare candidati
    strettamente peggiori senza ricorrere alla sola media aggregata."""
    if len(scores_a) != len(scores_b):
        return False
    all_leq = all(a <= b + tol for a, b in zip(scores_a, scores_b))
    any_lt = any(a < b - tol for a, b in zip(scores_a, scores_b))
    return all_leq and any_lt


def filter_dominated(scored_candidates):
    """scored_candidates: [(plan, per_path_scores), ...]. Rimuove i
    candidati dominati da un altro candidato dello stesso pool (fronte di
    Pareto sul campione O-D) — riduce il lavoro prima del top-K per la
    validazione ENHSP."""
    survivors = []
    for i, (plan_i, scores_i) in enumerate(scored_candidates):
        dominated = False
        for j, (_plan_j, scores_j) in enumerate(scored_candidates):
            if i != j and dominates(scores_j, scores_i):
                dominated = True
                break
        if not dominated:
            survivors.append((plan_i, scores_i))
    return survivors


def rank_candidates(ctx, plans, paths, real_cycle=REAL_CYCLE_S, top_k=None,
                     use_dominance_filter=True):
    """Valuta una lista di piani candidati {tl_id: {phase_idx: duration}}
    (tipicamente varianti di un solo tlLogic alla volta, vedi search.py) e
    ritorna [(plan, aggregate_score), ...] ordinati dal migliore, troncati
    a top_k se specificato."""
    scored = [(plan, per_path_scores(ctx, plan, paths, real_cycle)) for plan in plans]
    if use_dominance_filter and len(scored) > 1:
        scored = filter_dominated(scored)
    ranked = sorted(
        ((plan, sum(s) / len(s) if s else 0.0) for plan, s in scored),
        key=lambda t: t[1],
    )
    return ranked[:top_k] if top_k else ranked

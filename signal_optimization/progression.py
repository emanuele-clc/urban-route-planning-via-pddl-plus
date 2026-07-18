"""
signal_optimization/progression.py
===================================
Criticita' #2 (P2, non bloccante — 2_traffic_signal_optimization.md sez.
3.5): termine di punteggio analitico per il coordinamento (green wave) tra
semafori adiacenti su uno stesso percorso, calcolato SEPARATAMENTE dal
dominio PDDL+ (che non modella il tempo assoluto di arrivo rispetto alla
fase del semaforo — richiederebbe processi ciclici per ogni incrocio,
troppo costoso per ENHSP con -s aibr) e sommato come penalita' aggiuntiva
alla fitness aggregata (webster_screen.score_plan / enhsp_eval.evaluate_plan).

Formula: per due semafori successivi lungo un percorso, con tempo di
percorrenza noto tra loro e ciclo comune, la progressione ideale ("onda
verde") richiede che l'offset tra l'inizio del verde delle due giunzioni
sia congruo (mod ciclo) al tempo di percorrenza dell'arco che le separa.
Lo scostamento da questa condizione e' la penalita' di banda persa.

Nota di scope (v1): un 'corridoio' qui e' semplicemente la sequenza di
semafori incontrati in ordine lungo UN percorso di riferimento, non
necessariamente sulla stessa via OSM — l'identificazione di corridoi
stradali reali e' un raffinamento rimandato a una versione successiva
(non blocca l'uso di questo modulo come bonus/penalita' nella fitness).
"""

import os
import sys

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

import build_problems as bp  # noqa: E402
from extract_sumo_data import REAL_CYCLE_S  # noqa: E402


def offset_mismatch(travel_time_s, offset_diff_s, cycle_s):
    """Scostamento (s, in [0, cycle/2]) tra l'offset REALE tra due
    semafori successivi e quello IDEALE (= travel_time_s mod ciclo) per
    l'onda verde. 0 = progressione perfetta, cycle/2 = caso peggiore
    (verde dell'uno esattamente in fase col rosso dell'altro)."""
    if cycle_s <= 0:
        return 0.0
    ideal = travel_time_s % cycle_s
    actual = offset_diff_s % cycle_s
    diff = abs(ideal - actual) % cycle_s
    return min(diff, cycle_s - diff)


def progression_penalty(corridor_tls, offsets_by_tl, travel_times, cycle_s=REAL_CYCLE_S):
    """corridor_tls: lista ordinata di tl_id lungo un percorso.
    offsets_by_tl: {tl_id: offset_s} (offset assoluto di inizio ciclo, gia'
    presente in extract_sumo_data.py::load_net -> tls[id]['offset']).
    travel_times: {(tl_a, tl_b): tempo_percorrenza_s} tra coppie di
    semafori successivi del corridoio (vedi travel_time_between_tls sotto).
    Ritorna la penalita' totale (s) di banda persa sul corridoio — piu'
    basso e' meglio, 0 = green wave perfetta su tutte le coppie."""
    total = 0.0
    for a, b in zip(corridor_tls, corridor_tls[1:]):
        if (a, b) not in travel_times:
            continue
        off_a = offsets_by_tl.get(a, 0.0)
        off_b = offsets_by_tl.get(b, 0.0)
        total += offset_mismatch(travel_times[(a, b)], off_b - off_a, cycle_s)
    return total


def corridor_from_path(path, tl_by_node):
    """Sequenza di tl_id incontrati in ordine lungo un percorso di nodi.
    tl_by_node: {osm_id: tl_id} — costruibile da ctx['movements_by_node']
    (webster_screen.build_zone_context) prendendo il primo tl_id per nodo."""
    return [tl_by_node[n] for n in path if n in tl_by_node]


def tl_by_node_map(movements_by_node):
    """{osm_id: tl_id} dal 'movements_by_node' di webster_screen (un nodo
    puo' avere piu' tl_id se e' membro di cluster diversi mappati sullo
    stesso id OSM: si prende il primo, caso raro nei dati del progetto)."""
    return {nid: mv_list[0][0] for nid, mv_list in movements_by_node.items() if mv_list}


def travel_time_between(path, node_data, edges, vehicle_counts=None):
    """Tempo di percorrenza cumulato (arc-time, con congestion-factor, +
    turn-time) lungo un sotto-percorso di nodi consecutivi — stessa
    formula usata da build_problems.write_pddl per arc-time/turn-time."""
    if vehicle_counts is None:
        vehicle_counts = {}
    total = 0.0
    for i in range(len(path) - 1):
        a, b = path[i], path[i + 1]
        if (a, b) not in edges:
            continue
        d, spd = edges[(a, b)]
        vc = vehicle_counts.get((a, b), 0)
        cf = 1.0 + vc / 10.0
        eff = spd / cf if cf > 0 else spd
        total += d / eff if eff > 0 else 0.0
        if i > 0:
            total += bp.turn_time_s(path[i - 1], a, b, node_data)
    return total


def travel_times_between_tls(path, tl_by_node, node_data, edges, vehicle_counts=None):
    """{(tl_a, tl_b): tempo_percorrenza_s} tra coppie di semafori
    SUCCESSIVI incontrati lungo 'path' (salta i nodi non semaforizzati in
    mezzo, sommandone comunque il tempo di percorrenza nel tratto)."""
    signal_positions = [(i, tl_by_node[n]) for i, n in enumerate(path) if n in tl_by_node]
    out = {}
    for (i_a, tl_a), (i_b, tl_b) in zip(signal_positions, signal_positions[1:]):
        sub_path = path[i_a:i_b + 1]
        out[(tl_a, tl_b)] = travel_time_between(sub_path, node_data, edges, vehicle_counts)
    return out


def path_progression_penalty(path, ctx, offsets_by_tl, cycle_s=REAL_CYCLE_S, vehicle_counts=None):
    """Scorciatoia: penalita' di progressione lungo UN percorso di
    riferimento, a partire dal contesto di zona di webster_screen.py."""
    tl_by_node = tl_by_node_map(ctx["movements_by_node"])
    corridor = corridor_from_path(path, tl_by_node)
    if len(corridor) < 2:
        return 0.0
    travel_times = travel_times_between_tls(path, tl_by_node, ctx["node_data"],
                                             ctx["edges"], vehicle_counts)
    return progression_penalty(corridor, offsets_by_tl, travel_times, cycle_s)


def offsets_from_tls_data(tls_data):
    """{tl_id: offset_s} dai dati SUMO estratti (sumo_data_<zona>.json,
    campo 'offset_s' di ogni tlLogic — extract_sumo_data.py::load_net)."""
    return {tid: tl.get("offset_s", 0.0) for tid, tl in tls_data.items()}

"""
signal_optimization/search.py
==============================
Ricerca locale (coordinate search / hill-climbing per giunzione — sez. 1
punto 5 e sez. 3.4 punto 6 di 2_traffic_signal_optimization.md): ottimizza
UN tlLogic alla volta (coordinate descent a livello di giunzione), tenendo
fissi gli altri, e itera SOLO sui tlLogic attraversati da almeno un
percorso di riferimento del campione O-D condiviso (generate_demand.py) —
"scope locale della ricerca" (criticita' #4, punto 6): quando si valuta
una modifica a una giunzione, si ri-esegue ENHSP solo per gli O-D il cui
percorso di riferimento la attraversa, non sull'intero campione.

Per ogni giunzione, ogni round:
  1. genera i vicini del candidato corrente (candidates.neighbors — sposta
     DURATION_STEP_S tra due fasi verdi, vincolo di ciclo sempre rispettato
     per costruzione, criticita' #6);
  2. li ordina con lo screening analitico di Webster, economico
     (webster_screen.rank_candidates — nessun ENHSP);
  3. valida in ordine di rank con ENHSP (costoso, enhsp_eval.evaluate_plan)
     SOLO sugli O-D locali a quella giunzione, fino al primo miglioramento
     (hill-climbing, first-improvement) o fino a esaurire il top-K;
  4. accetta il miglior vicino trovato, altrimenti la giunzione resta
     invariata (ottimo locale raggiunto per quel tlLogic).
"""

import os
import sys
from collections import defaultdict

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(BASE, "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

import build_problems as bp  # noqa: E402

from signal_optimization import candidates as C  # noqa: E402
from signal_optimization import webster_screen as ws  # noqa: E402
from signal_optimization import enhsp_eval as ee  # noqa: E402

MAX_ROUNDS_PER_TL = 6        # tetto ai passi di hill-climbing per giunzione
LOCAL_OD_CAP = 12            # tetto agli O-D locali rivalutati con ENHSP per giunzione


def reference_paths_with_od(ctx, od_pairs):
    """Come webster_screen.reference_paths, ma mantiene l'associazione
    (coppia O-D -> percorso) — necessaria per lo scope locale della
    ricerca (sapere quali O-D attraversano una data giunzione)."""
    adj_sub = defaultdict(dict)
    for (a, b), (d, _spd) in ctx["edges"].items():
        adj_sub[a][b] = d

    out = []
    for pair in od_pairs:
        s, g = pair["start"], pair["goal"]
        if s not in ctx["selected"] or g not in ctx["selected"]:
            continue
        _dist, prev = bp.dijkstra(s, adj_sub)
        if g != s and g not in prev:
            continue
        path = bp.reconstruct_path(prev, g)
        if len(path) >= 2:
            out.append((pair, path))
    return out


def tls_local_pairs(ctx, pairs_paths):
    """{tl_id: [(pair, path), ...]} — per ogni tlLogic controllabile, quali
    coppie O-D (con relativo percorso) lo attraversano. Base dello 'scope
    locale della ricerca' (criticita' #4, punto 6)."""
    out = defaultdict(list)
    seen = defaultdict(set)
    for pair, path in pairs_paths:
        for a, b, c, is_first in ws.path_triples(path):
            match = ws.best_matching_movement(a, b, c, ctx["node_data"],
                                               ctx["movements_by_node"], is_first=is_first)
            if not match:
                continue
            tid = match[0]
            key = (pair["start"], pair["goal"])
            if key not in seen[tid]:
                seen[tid].add(key)
                out[tid].append((pair, path))
    return dict(out)


def hill_climb_junction(zone, ctx, plan, tid, local_pairs_paths, top_k=5,
                         max_workers=4, timeout=60, max_rounds=MAX_ROUNDS_PER_TL,
                         od_cap=LOCAL_OD_CAP):
    """Ottimizza le durate di fase di UN tlLogic (tenendo fisso il resto
    del piano), valutando solo gli O-D locali a quella giunzione. Ritorna
    (piano_aggiornato, metrica_finale_locale, n_round_di_miglioramento)."""
    tl = ctx["tls_data"][tid]
    green_idxs = C.green_phase_indices(tl)
    if len(green_idxs) < 2 or not local_pairs_paths:
        return plan, None, 0

    eval_pairs = [pair for pair, _path in local_pairs_paths[:od_cap]]
    local_paths = [path for _pair, path in local_pairs_paths]

    base_eval = ee.evaluate_plan(zone, ctx, plan, eval_pairs, max_workers=max_workers, timeout=timeout)
    best_metric = base_eval["mean_metric"]
    if best_metric is None:
        return plan, None, 0

    n_improved = 0
    for _round in range(max_rounds):
        neighbor_durs = C.neighbors(plan[tid], green_idxs)
        cand_plans = [dict(plan, **{tid: nb}) for nb in neighbor_durs]
        ranked = ws.rank_candidates(ctx, cand_plans, local_paths, top_k=top_k)

        accepted = False
        for cand_plan, _wscore in ranked:
            ev = ee.evaluate_plan(zone, ctx, cand_plan, eval_pairs, max_workers=max_workers, timeout=timeout)
            m = ev["mean_metric"]
            if m is not None and m < best_metric - 1e-6:
                best_metric = m
                plan = cand_plan
                accepted = True
                n_improved += 1
                break
        if not accepted:
            break

    return plan, best_metric, n_improved


def local_search(zone, ctx, od_pairs, initial_plan=None, top_k=5, max_workers=4,
                  timeout=60, max_rounds=MAX_ROUNDS_PER_TL, od_cap=LOCAL_OD_CAP,
                  progress_cb=None):
    """Coordinate search sull'intera zona: itera sui tlLogic controllabili
    e attraversati da almeno un O-D di riferimento, ottimizzando ciascuno
    con hill_climb_junction. progress_cb(tid, i, n_tot, metric): callback
    opzionale per il logging dell'avanzamento (usata da optimize.py)."""
    pairs_paths = reference_paths_with_od(ctx, od_pairs)
    local_pairs = tls_local_pairs(ctx, pairs_paths)

    plan = {tid: dict(durs) for tid, durs in
            (initial_plan or C.baseline_plan(ctx["tls_data"])).items()}

    controllable = [tid for tid in C.controllable_tls(ctx["tls_data"]) if tid in local_pairs]
    log = []
    for i, tid in enumerate(controllable):
        plan, metric, n_improved = hill_climb_junction(
            zone, ctx, plan, tid, local_pairs[tid],
            top_k=top_k, max_workers=max_workers, timeout=timeout,
            max_rounds=max_rounds, od_cap=od_cap,
        )
        entry = {
            "tl_id": tid,
            "n_local_od": len(local_pairs[tid]),
            "local_metric": metric,
            "n_improving_steps": n_improved,
            "final_durations": dict(plan[tid]),
        }
        log.append(entry)
        if progress_cb:
            progress_cb(tid, i + 1, len(controllable), entry)

    return plan, log

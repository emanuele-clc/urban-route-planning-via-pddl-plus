"""
signal_optimization/enhsp_eval.py
==================================
Stadio 2 (costoso) della pipeline di valutazione candidati (Criticita' #4,
P1 — 2_traffic_signal_optimization.md sez. 3.4): rigenera un
problem_<zona>.pddl per una specifica coppia O-D con i signal-delay del
piano candidato iniettati (riusando build_problems.write_pddl e
correggendo solo le righe signal-delay in un secondo passaggio, per non
duplicare l'intera logica di generazione), invoca ENHSP riusando
trova_enhsp/solve_problem gia' presenti in pddl_files/run.py (nessuna
duplicazione della logica di discovery/invocazione ENHSP — vedi sez. 4),
e ne legge 'total-time' (Metric).

Include: cache in memoria per (zona, start, goal, piano) per evitare run
duplicati tra iterazioni della ricerca locale (search.py), e valutazione
in parallelo degli O-D indipendenti (ogni run ENHSP e' un sottoprocesso,
quindi il GIL non e' un collo di bottiglia — ThreadPoolExecutor basta).
"""

import os
import re
import sys
import shutil
import tempfile
from concurrent.futures import ThreadPoolExecutor, as_completed

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)
PDDL_DIR = os.path.join(BASE, "pddl_files")
if PDDL_DIR not in sys.path:
    sys.path.insert(0, PDDL_DIR)

import build_problems as bp  # noqa: E402
from extract_sumo_data import REAL_CYCLE_S  # noqa: E402
import run as pddl_run  # noqa: E402  (pddl_files/run.py: trova_enhsp, solve_problem)
from signal_optimization.webster_screen import (  # noqa: E402
    best_matching_movement, movement_delay_for_plan,
)

DOMAIN_PATH = os.path.join(PDDL_DIR, "domain.pddl")

_SIGNAL_DELAY_RE = re.compile(
    r"\(= \(signal-delay\s+(\S+)\s+(\S+)\s+(\S+)\)\s+[\d.]+\)"
)

_CACHE = {}   # (zona, start, goal, piano) -> risultato solve_problem
_JAR = None   # cache del path di enhsp.jar (discovery costosa una tantum)


def _plan_cache_key(plan):
    """Rappresentazione hashable e ordine-indipendente del piano, per la
    cache — due piani con le stesse durate (in ordine diverso nel dict)
    devono mappare alla stessa chiave."""
    return tuple(sorted(
        (tid, tuple(sorted(durs.items())))
        for tid, durs in plan.items()
    ))


def patch_signal_delay(pddl_path, ctx, plan, nm_inv, real_cycle=REAL_CYCLE_S):
    """Riscrive le righe '(= (signal-delay a b c) V)' di un problem.pddl
    gia' generato da build_problems.write_pddl, sostituendo V con il
    ritardo ricalcolato sotto il piano candidato 'plan' (stesso match per
    bearing usato dallo screening — sez. 3.1/3.4)."""
    with open(pddl_path, encoding="utf-8") as f:
        text = f.read()

    def _sub(m):
        n1, n2, n3 = m.group(1), m.group(2), m.group(3)
        a, b, c = nm_inv.get(n1), nm_inv.get(n2), nm_inv.get(n3)
        if not (a and b and c):
            return m.group(0)
        is_first = (n1 == n2)
        match = best_matching_movement(a, b, c, ctx["node_data"],
                                        ctx["movements_by_node"], is_first=is_first)
        if match is None:
            return m.group(0)
        tid, mv = match
        delay = movement_delay_for_plan(tid, mv, plan, real_cycle)
        return f"(= (signal-delay {n1} {n2} {n3}) {delay})"

    new_text = _SIGNAL_DELAY_RE.sub(_sub, text)
    with open(pddl_path, "w", encoding="utf-8") as f:
        f.write(new_text)


def build_problem_file(zone, ctx, start, goal, plan, out_path, real_cycle=REAL_CYCLE_S):
    """Genera un problem.pddl per la coppia (start,goal) con i
    signal-delay del piano candidato 'plan' gia' iniettati."""
    node_data, selected, edges = ctx["node_data"], ctx["selected"], ctx["edges"]
    nm = bp.name_map_for(selected, node_data)
    nm_inv = {v: k for k, v in nm.items()}

    peripheral = bp.classify_zones(selected, node_data)
    density = bp.compute_intersection_density(selected, node_data)
    cong_delays = bp.compute_congestion_delay(selected, node_data, edges, peripheral,
                                               density, ctx["edge_highway"])
    vc = bp.compute_vehicle_counts(selected, edges, start)
    sumo_delays = bp.load_sumo_signal_delays(zone)
    sumo_movements = bp.load_sumo_movements(zone)

    bp.write_pddl(zone, selected, node_data, edges, start, goal, out_path,
                   signal_nodes=ctx["signal_node_ids"],
                   congestion_delays=cong_delays,
                   vehicle_counts=vc,
                   intersection_density=density,
                   peripheral=peripheral,
                   sumo_delays=sumo_delays,
                   sumo_movements=sumo_movements)
    patch_signal_delay(out_path, ctx, plan, nm_inv, real_cycle)


def evaluate_od(zone, ctx, start, goal, plan, jar=None, real_cycle=REAL_CYCLE_S, timeout=60):
    """Risolve un singolo O-D sotto il piano candidato. Cache per
    (zona, start, goal, piano) — vedi _plan_cache_key."""
    key = (zone, start, goal, _plan_cache_key(plan))
    if key in _CACHE:
        return _CACHE[key]

    tmp_dir = tempfile.mkdtemp(prefix="tsc_eval_")
    try:
        problem_path = os.path.join(tmp_dir, "problem.pddl")
        build_problem_file(zone, ctx, start, goal, plan, problem_path, real_cycle)
        res = pddl_run.solve_problem(problem_path, domain_path=DOMAIN_PATH, jar=jar, timeout=timeout)
    finally:
        shutil.rmtree(tmp_dir, ignore_errors=True)

    _CACHE[key] = res
    return res


def evaluate_plan(zone, ctx, plan, od_pairs, real_cycle=REAL_CYCLE_S,
                   max_workers=4, timeout=60):
    """Valuta un piano candidato sull'intero campione O-D (in parallelo —
    ogni O-D e' un run ENHSP indipendente, vedi sez. 3.4 punto 5) e
    ritorna la metrica aggregata (media di 'metric'=total-time sugli O-D
    risolti) + i risultati per singolo O-D."""
    global _JAR
    if _JAR is None:
        _JAR = pddl_run.trova_enhsp()

    results = {}
    with ThreadPoolExecutor(max_workers=max_workers) as ex:
        futures = {
            ex.submit(evaluate_od, zone, ctx, pair["start"], pair["goal"], plan,
                      _JAR, real_cycle, timeout): (pair["start"], pair["goal"])
            for pair in od_pairs
        }
        for fut in as_completed(futures):
            od = futures[fut]
            results[od] = fut.result()

    metrics = [r["metric"] for r in results.values() if r.get("solved") and r.get("metric") is not None]
    n_solved = sum(1 for r in results.values() if r.get("solved"))
    return {
        "mean_metric": sum(metrics) / len(metrics) if metrics else None,
        "n_solved": n_solved,
        "n_total": len(od_pairs),
        "per_od": results,
    }

"""
signal_optimization/optimize.py
================================
Orchestratore della pipeline di ottimizzazione semaforica (punto 2 della
roadmap — vedi 2_traffic_signal_optimization.md, proposta concettuale
sez. 1 e artefatti attesi sez. 4):

    candidati vincolati (candidates.py, criticita' #6)
        -> screening analitico Webster (webster_screen.py, criticita' #4.1)
        -> validazione/ricerca ENHSP (enhsp_eval.py + search.py, criticita' #4.2/#4.6)
        -> confronto baseline vs ottimizzato sull'intero campione O-D condiviso
           (generate_demand.py, criticita' #3)
        -> output sumo_extracted/signal_plan_<zona>.json

Il file di output e' l'input diretto atteso dal punto 3 (iniezione in
SUMO): {tlLogic_id: {phase_idx: duration_s}}.

Uso:
    python -m signal_optimization.optimize piccola
    python -m signal_optimization.optimize piccola media grande --max-workers 4
"""

import os
import sys
import json
import time
import argparse

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
if BASE not in sys.path:
    sys.path.insert(0, BASE)

from signal_optimization import candidates as C  # noqa: E402
from signal_optimization import webster_screen as ws  # noqa: E402
from signal_optimization import enhsp_eval as ee  # noqa: E402
from signal_optimization import search as se  # noqa: E402
from signal_optimization import progression as pg  # noqa: E402

import generate_demand as gd  # noqa: E402

SUMO_DIR = os.path.join(BASE, "sumo_extracted")


def _load_or_generate_demand(zone, n_samples):
    path = os.path.join(SUMO_DIR, f"demand_{zone}.json")
    if os.path.exists(path):
        with open(path, encoding="utf-8") as f:
            data = json.load(f)
        if data.get("n_samples", 0) >= n_samples:
            return data["od_pairs"][:n_samples]
    _out_path, data = gd.generate(zone, n_samples=n_samples)
    return data["od_pairs"]


def optimize_zone(zone, n_samples=gd.N_TSC_SAMPLES, top_k=5, max_workers=4,
                   timeout=60, verbose=True):
    t0 = time.time()
    if verbose:
        print(f"[{zone}] costruzione contesto (grafo PDDL + dati SUMO)...")
    ctx = ws.build_zone_context(zone)
    od_pairs = _load_or_generate_demand(zone, n_samples)
    if verbose:
        print(f"[{zone}] campione O-D condiviso: {len(od_pairs)} coppie")

    baseline_plan = C.baseline_plan(ctx["tls_data"])

    def _progress(tid, i, n_tot, entry):
        if verbose:
            print(f"[{zone}] ({i}/{n_tot}) {tid}: metrica locale={entry['local_metric']} "
                  f"(+{entry['n_improving_steps']} miglioramenti) -> {entry['final_durations']}")

    optimized_plan, log = se.local_search(
        zone, ctx, od_pairs, initial_plan=baseline_plan,
        top_k=top_k, max_workers=max_workers, timeout=timeout,
        progress_cb=_progress,
    )

    if verbose:
        print(f"[{zone}] valutazione finale su tutto il campione O-D "
              f"({len(od_pairs)} coppie, baseline vs ottimizzato)...")
    final_baseline = ee.evaluate_plan(zone, ctx, baseline_plan, od_pairs,
                                       max_workers=max_workers, timeout=timeout)
    final_optimized = ee.evaluate_plan(zone, ctx, optimized_plan, od_pairs,
                                        max_workers=max_workers, timeout=timeout)

    # Criticita' #2 (P2): punteggio di progressione, riportato ma non usato
    # come obiettivo primario della ricerca (vedi sez. 3.5 del design doc).
    offsets = pg.offsets_from_tls_data(ctx["tls_data"])
    pairs_paths = se.reference_paths_with_od(ctx, od_pairs)
    prog_baseline = sum(pg.path_progression_penalty(path, ctx, offsets) for _p, path in pairs_paths)
    prog_optimized = prog_baseline  # la progressione non dipende dalle durate di verde, solo dagli offset

    elapsed = time.time() - t0
    report = {
        "zone": zone,
        "n_od_samples": len(od_pairs),
        "n_tls_optimized": sum(1 for e in log if e["n_improving_steps"] > 0),
        "n_tls_candidates": len(log),
        "baseline_mean_metric": final_baseline["mean_metric"],
        "optimized_mean_metric": final_optimized["mean_metric"],
        "baseline_n_solved": final_baseline["n_solved"],
        "optimized_n_solved": final_optimized["n_solved"],
        "progression_penalty_s": round(prog_baseline, 1),
        "elapsed_s": round(elapsed, 1),
        "junction_log": log,
    }

    if verbose:
        bm, om = report["baseline_mean_metric"], report["optimized_mean_metric"]
        delta = (om - bm) if (bm is not None and om is not None) else None
        print(f"[{zone}] baseline mean total-time={bm}  ottimizzato={om}  "
              f"delta={delta}  ({report['n_tls_optimized']}/{report['n_tls_candidates']} giunzioni migliorate)  "
              f"[{elapsed:.1f}s]")

    return optimized_plan, report


def write_signal_plan(zone, plan, report):
    """Scrive sumo_extracted/signal_plan_<zona>.json nel formato atteso dal
    punto 3 (iniezione in SUMO): {tlLogic_id: {phase_idx: duration_s}}."""
    out = {
        "zone": zone,
        "format": "tlLogic_id -> {phase_idx: duration_s} (solo fasi verdi ottimizzate; "
                  "le fasi non presenti mantengono la durata originale del net.xml)",
        "plan": plan,
        "report": report,
    }
    out_path = os.path.join(SUMO_DIR, f"signal_plan_{zone}.json")
    os.makedirs(SUMO_DIR, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    return out_path


def main():
    parser = argparse.ArgumentParser(description="Ottimizzazione semaforica via PDDL+ (punto 2 della roadmap)")
    parser.add_argument("zones", nargs="*", default=["piccola"], choices=["piccola", "media", "grande"])
    parser.add_argument("--n-samples", type=int, default=gd.N_TSC_SAMPLES)
    parser.add_argument("--top-k", type=int, default=5)
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument("--timeout", type=int, default=60)
    parser.add_argument("--quiet", action="store_true")
    args = parser.parse_args()

    for zone in args.zones:
        plan, report = optimize_zone(zone, n_samples=args.n_samples, top_k=args.top_k,
                                      max_workers=args.max_workers, timeout=args.timeout,
                                      verbose=not args.quiet)
        out_path = write_signal_plan(zone, plan, report)
        print(f"[{zone}] piano salvato in: {out_path}")


if __name__ == "__main__":
    main()

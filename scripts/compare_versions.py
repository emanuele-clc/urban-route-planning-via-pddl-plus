"""
compare_versions.py
====================
Suite di test che confronta la VECCHIA versione del dominio PDDL+
(signal-delay per NODO, media dei movimenti — git HEAD, prima della
modifica descritta in 2_traffic_signal_optimization.md) con la NUOVA
versione (signal-delay per MOVIMENTO (prev,from,to), sez. 3.1) attualmente
nel working tree.

Per ogni mappa (piccola, media, grande):
  1. campiona N coppie (start, goal) raggiungibili nel sottografo PDDL;
  2. per ciascuna coppia, genera il problem.pddl con la logica VECCHIA
     (build_problems.py + domain.pddl di git HEAD) e con la logica NUOVA
     (working tree), risolve entrambi con ENHSP;
  3. ricostruisce il percorso pianificato e ne scompone il costo:
     distanza totale, numero di archi percorsi, tempo di percorrenza
     (arc-time), ritardo di svolta (turn-time), ritardo semaforico
     (signal-delay) e ritardo di congestione (congestion-delay).

Il "vecchio" build_problems.py/domain.pddl vengono caricati dinamicamente
da `git show HEAD:...` (nessun checkout, nessuna modifica allo stato del
repo) cosi' il confronto e' riproducibile anche dopo un commit.

Uso (dalla radice del progetto):
    python scripts/compare_versions.py [piccola] [media] [grande] [--n-samples 10]

Output:
    comparison_results/results_<zona>.json  (dati grezzi, per debug/riuso)
    comparison_results/results.json         (tutte le zone)
"""

import os
import sys
import json
import re
import argparse
import importlib.util
import subprocess
import tempfile
import random
from collections import defaultdict
from concurrent.futures import ThreadPoolExecutor, as_completed

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))         # scripts/
BASE = os.path.dirname(SCRIPT_DIR)                               # radice del progetto
sys.path.insert(0, SCRIPT_DIR)
sys.path.insert(0, os.path.join(BASE, "pddl_files"))

import build_problems as new_bp  # noqa: E402  (working tree = versione NUOVA)
import run as pddl_run  # noqa: E402  (pddl_files/run.py: trova_enhsp, solve_problem)

OSM_DIR = os.path.join(BASE, "osm_files")
PDDL_DIR = os.path.join(BASE, "pddl_files")
SUMO_DIR = os.path.join(BASE, "sumo_extracted")
OUT_DIR = os.path.join(BASE, "comparison_results")

N_SAMPLES_DEFAULT = 10
RANDOM_SEED = 123   # seed dedicato al campionamento dei test, indipendente
                     # da RANDOM_SEED=42 usato per vehicle-count/domanda

ZONE_CONFIGS = {
    "piccola": ("dublin_piccola_centro.osm", 14),
    "media":   ("dublin_media_residenziale.osm", 50),
    "grande":  ("dublin_grande_porto.osm", 120),
}


# ---------------------------------------------------------------------------
# Caricamento dinamico della versione VECCHIA (git HEAD) di build_problems.py
# e domain.pddl, senza toccare lo stato del repo.
# ---------------------------------------------------------------------------
def _git_show(rel_path):
    result = subprocess.run(["git", "show", f"HEAD:{rel_path}"], cwd=BASE,
                             capture_output=True, text=True, check=True)
    return result.stdout


def load_old_build_problems():
    content = _git_show("scripts/build_problems.py")
    tmp_dir = tempfile.mkdtemp(prefix="old_bp_")
    tmp_path = os.path.join(tmp_dir, "build_problems_old.py")
    with open(tmp_path, "w", encoding="utf-8") as f:
        f.write(content)
    spec = importlib.util.spec_from_file_location("build_problems_old", tmp_path)
    mod = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(mod)
    # il modulo calcola SUMO_DIR relativo al proprio __file__ (nella tempdir):
    # va ripuntato alla cartella reale sumo_extracted/ del progetto.
    mod.SUMO_DIR = SUMO_DIR
    return mod


def write_old_domain():
    content = _git_show("pddl_files/domain.pddl")
    tmp_dir = tempfile.mkdtemp(prefix="old_domain_")
    tmp_path = os.path.join(tmp_dir, "domain.pddl")
    with open(tmp_path, "w", encoding="utf-8") as f:
        f.write(content)
    return tmp_path


# ---------------------------------------------------------------------------
# Campionamento coppie O-D raggiungibili nel sottografo PDDL
# ---------------------------------------------------------------------------
def sample_od_pairs(selected, edges, n_samples, seed=RANDOM_SEED):
    adj_sub = defaultdict(dict)
    for (a, b), (d, _spd) in edges.items():
        adj_sub[a][b] = d

    rng = random.Random(seed)
    sel_list = list(selected)
    pairs = []
    seen = set()
    attempts = 0
    while len(pairs) < n_samples and attempts < n_samples * 50:
        attempts += 1
        s = rng.choice(sel_list)
        candidates = [n for n in sel_list if n != s]
        if not candidates:
            continue
        g = rng.choice(candidates)
        if (s, g) in seen:
            continue
        dist, _prev = new_bp.dijkstra(s, adj_sub)
        if g not in dist:
            continue
        seen.add((s, g))
        pairs.append((s, g))
    return pairs


# ---------------------------------------------------------------------------
# Parsing del piano ENHSP -> sequenza di nodi PDDL
# ---------------------------------------------------------------------------
_STARTMOVE_RE = re.compile(r"\(start-move\s+(\S+)\s+(\S+)\s+(\S+)\)", re.IGNORECASE)


def route_from_plan(plan_text):
    route = []
    for line in (plan_text or "").splitlines():
        m = _STARTMOVE_RE.search(line)
        if not m:
            continue
        frm, to = m.group(2).lower(), m.group(3).lower()
        if not route:
            route.append(frm)
        route.append(to)
    return route


# ---------------------------------------------------------------------------
# Scomposizione del costo di un percorso (post-hoc, a partire dalla
# sequenza di nodi OSM) — una versione per la logica VECCHIA (signal-delay
# per nodo, addebitato all'arrivo) e una per la NUOVA (signal-delay per
# movimento, addebitato alla partenza) — stessa logica gia' usata in
# webapp/app.py::solve().
# ---------------------------------------------------------------------------
def decompose_old(route_osm, node_data, edges, vc, cong_delays, signal_nodes,
                   sumo_delays, fallback_delay):
    total_dist = 0.0
    travel_time = 0.0
    turn_delay_total = 0.0
    signal_delay_total = 0.0
    congestion_delay_total = 0.0
    signals_crossed = 0

    for i in range(len(route_osm) - 1):
        a, b = route_osm[i], route_osm[i + 1]
        if (a, b) in edges:
            d, spd = edges[(a, b)]
            total_dist += d
            vc_arc = vc.get((a, b), 0)
            cf = 1.0 + vc_arc / 10.0
            eff = spd / cf
            if eff > 0:
                travel_time += d / eff
        if 0 < i < len(route_osm) - 1:
            p = route_osm[i - 1]
            turn_delay_total += new_bp.turn_time_s(p, a, b, node_data)

    for node in route_osm[1:]:
        if node in sumo_delays:
            sd = sumo_delays[node]
        elif node in signal_nodes:
            sd = fallback_delay
        else:
            sd = 0
        if sd > 0:
            signals_crossed += 1
            signal_delay_total += sd
        congestion_delay_total += cong_delays.get(node, 0)

    total_time = travel_time + turn_delay_total + signal_delay_total + congestion_delay_total
    return {
        "n_edges": len(route_osm) - 1,
        "total_dist_m": round(total_dist, 1),
        "travel_time_s": round(travel_time, 2),
        "turn_delay_s": round(turn_delay_total, 2),
        "signal_delay_s": round(signal_delay_total, 2),
        "signals_crossed": signals_crossed,
        "congestion_delay_s": round(congestion_delay_total, 2),
        "total_time_s": round(total_time, 2),
    }


def decompose_new(route_osm, node_data, edges, vc, cong_delays, signal_nodes,
                   sumo_delays, sumo_movements, fallback_delay):
    total_dist = 0.0
    travel_time = 0.0
    turn_delay_total = 0.0
    signal_delay_total = 0.0
    congestion_delay_total = 0.0
    signals_crossed = 0

    for i in range(len(route_osm) - 1):
        a, b = route_osm[i], route_osm[i + 1]
        if (a, b) in edges:
            d, spd = edges[(a, b)]
            total_dist += d
            vc_arc = vc.get((a, b), 0)
            cf = 1.0 + vc_arc / 10.0
            eff = spd / cf
            if eff > 0:
                travel_time += d / eff

        if i == 0:
            sd = new_bp.assign_movement_signal_delay(a, a, b, node_data, sumo_movements, is_first=True)
        else:
            p = route_osm[i - 1]
            turn_delay_total += new_bp.turn_time_s(p, a, b, node_data)
            sd = new_bp.assign_movement_signal_delay(p, a, b, node_data, sumo_movements)
        if sd is None:
            sd = sumo_delays.get(a, fallback_delay if a in signal_nodes else 0)
        if sd > 0:
            signals_crossed += 1
            signal_delay_total += sd

    for node in route_osm[1:]:
        congestion_delay_total += cong_delays.get(node, 0)

    total_time = travel_time + turn_delay_total + signal_delay_total + congestion_delay_total
    return {
        "n_edges": len(route_osm) - 1,
        "total_dist_m": round(total_dist, 1),
        "travel_time_s": round(travel_time, 2),
        "turn_delay_s": round(turn_delay_total, 2),
        "signal_delay_s": round(signal_delay_total, 2),
        "signals_crossed": signals_crossed,
        "congestion_delay_s": round(congestion_delay_total, 2),
        "total_time_s": round(total_time, 2),
    }


# ---------------------------------------------------------------------------
# Generazione problem.pddl (vecchia/nuova) per una coppia O-D
# ---------------------------------------------------------------------------
def build_old_problem(old_bp, zone, ctx, start, goal, out_path):
    node_data, selected, edges = ctx["node_data"], ctx["selected"], ctx["edges"]
    peripheral = old_bp.classify_zones(selected, node_data)
    density = old_bp.compute_intersection_density(selected, node_data)
    cong_delays = old_bp.compute_congestion_delay(selected, node_data, edges, peripheral,
                                                   density, ctx["edge_highway"])
    vc = old_bp.compute_vehicle_counts(selected, edges, start)
    sumo_delays = old_bp.load_sumo_signal_delays(zone)
    old_bp.write_pddl(zone, selected, node_data, edges, start, goal, out_path,
                       signal_nodes=ctx["signal_nodes"],
                       congestion_delays=cong_delays,
                       vehicle_counts=vc,
                       intersection_density=density,
                       peripheral=peripheral,
                       sumo_delays=sumo_delays)
    return {"vc": vc, "cong_delays": cong_delays, "sumo_delays": sumo_delays}


def build_new_problem(zone, ctx, start, goal, out_path):
    node_data, selected, edges = ctx["node_data"], ctx["selected"], ctx["edges"]
    peripheral = new_bp.classify_zones(selected, node_data)
    density = new_bp.compute_intersection_density(selected, node_data)
    cong_delays = new_bp.compute_congestion_delay(selected, node_data, edges, peripheral,
                                                   density, ctx["edge_highway"])
    vc = new_bp.compute_vehicle_counts(selected, edges, start)
    sumo_delays = new_bp.load_sumo_signal_delays(zone)
    sumo_movements = new_bp.load_sumo_movements(zone)
    new_bp.write_pddl(zone, selected, node_data, edges, start, goal, out_path,
                       signal_nodes=ctx["signal_nodes"],
                       congestion_delays=cong_delays,
                       vehicle_counts=vc,
                       intersection_density=density,
                       peripheral=peripheral,
                       sumo_delays=sumo_delays,
                       sumo_movements=sumo_movements)
    return {"vc": vc, "cong_delays": cong_delays, "sumo_delays": sumo_delays, "sumo_movements": sumo_movements}


# ---------------------------------------------------------------------------
# Valutazione di UNA coppia O-D (vecchia + nuova versione)
# ---------------------------------------------------------------------------
def evaluate_pair(old_bp, old_domain_path, zone, ctx, start, goal, nm_inv, jar,
                   fallback_old, fallback_new, timeout=60):
    record = {"start": start, "goal": goal,
              "start_name": ctx["nm"][start], "goal_name": ctx["nm"][goal]}

    tmp_dir = tempfile.mkdtemp(prefix="cmp_")
    try:
        # --- VECCHIA versione ---
        old_problem = os.path.join(tmp_dir, "old_problem.pddl")
        old_ctx = build_old_problem(old_bp, zone, ctx, start, goal, old_problem)
        old_res = pddl_run.solve_problem(old_problem, domain_path=old_domain_path, jar=jar, timeout=timeout)
        record["old_solved"] = bool(old_res.get("solved"))
        # NB: il campo "Metric (Search)" stampato da ENHSP NON e' il valore
        # finale del fluent (total-time) nello stato goal (verificato per
        # confronto diretto con i valori del problem.pddl: es. su un piano
        # a 4 hop, Metric riportava 13.07 mentre la somma esatta di
        # arc-time+turn-time+signal-delay+congestion-delay lungo il piano
        # era 110.92) — e' tenuto solo come diagnostica grezza di ENHSP,
        # NON usato per le statistiche (vedi *_stats.total_time_s sotto,
        # ricostruito post-hoc dalle formule del dominio).
        record["old_metric_enhsp_raw"] = old_res.get("metric")
        if old_res.get("solved"):
            route_pddl = route_from_plan(old_res.get("plan_text"))
            route_osm = [nm_inv.get(r) for r in route_pddl]
            if all(route_osm) and len(route_osm) >= 2:
                record["old_stats"] = decompose_old(
                    route_osm, ctx["node_data"], ctx["edges"], old_ctx["vc"],
                    old_ctx["cong_delays"], ctx["signal_nodes"], old_ctx["sumo_delays"],
                    fallback_old)

        # --- NUOVA versione ---
        new_problem = os.path.join(tmp_dir, "new_problem.pddl")
        new_ctx = build_new_problem(zone, ctx, start, goal, new_problem)
        new_res = pddl_run.solve_problem(new_problem, domain_path=os.path.join(PDDL_DIR, "domain.pddl"),
                                          jar=jar, timeout=timeout)
        record["new_solved"] = bool(new_res.get("solved"))
        record["new_metric_enhsp_raw"] = new_res.get("metric")
        if new_res.get("solved"):
            route_pddl = route_from_plan(new_res.get("plan_text"))
            route_osm = [nm_inv.get(r) for r in route_pddl]
            if all(route_osm) and len(route_osm) >= 2:
                record["new_stats"] = decompose_new(
                    route_osm, ctx["node_data"], ctx["edges"], new_ctx["vc"],
                    new_ctx["cong_delays"], ctx["signal_nodes"], new_ctx["sumo_delays"],
                    new_ctx["sumo_movements"], fallback_new)
    finally:
        import shutil
        shutil.rmtree(tmp_dir, ignore_errors=True)

    return record


# ---------------------------------------------------------------------------
# Pipeline per zona
# ---------------------------------------------------------------------------
def build_zone_ctx(zone):
    osm_file, max_nodes = ZONE_CONFIGS[zone]
    osm_path = os.path.join(OSM_DIR, osm_file)
    node_data, adj, signal_ids, edge_hw = new_bp.build_contracted_graph(osm_path)
    selected = new_bp.select_connected_subgraph(node_data, adj, max_nodes)
    sel_set = set(selected)
    edges = {}
    for a in selected:
        for b, (d, spd) in adj.get(a, {}).items():
            if b in sel_set:
                edges[(a, b)] = (d, spd)
    nm = new_bp.name_map_for(selected, node_data)
    nm_inv = {v: k for k, v in nm.items()}
    return {
        "node_data": node_data,
        "selected": selected,
        "edges": edges,
        "edge_highway": {(a, b): edge_hw.get((a, b), "unclassified") for (a, b) in edges},
        "signal_nodes": signal_ids & sel_set,
        "nm": nm,
        "nm_inv": nm_inv,
    }


def run_zone(zone, old_bp, old_domain_path, n_samples, max_workers=4, timeout=60, verbose=True):
    ctx = build_zone_ctx(zone)
    pairs = sample_od_pairs(ctx["selected"], ctx["edges"], n_samples)
    if verbose:
        print(f"[{zone}] {len(pairs)} coppie O-D campionate su {len(ctx['selected'])} nodi")

    jar = pddl_run.trova_enhsp()
    fallback_old = old_bp.FALLBACK_SIGNAL_DELAY
    fallback_new = new_bp.FALLBACK_SIGNAL_DELAY

    records = []
    with ThreadPoolExecutor(max_workers=max_workers) as ex:
        futures = {
            ex.submit(evaluate_pair, old_bp, old_domain_path, zone, ctx, s, g, ctx["nm_inv"],
                      jar, fallback_old, fallback_new, timeout): (s, g)
            for s, g in pairs
        }
        for fut in as_completed(futures):
            rec = fut.result()
            records.append(rec)
            if verbose:
                old_t = rec.get("old_stats", {}).get("total_time_s")
                new_t = rec.get("new_stats", {}).get("total_time_s")
                print(f"[{zone}] {rec['start_name']} -> {rec['goal_name']}  "
                      f"vecchia={'OK' if rec['old_solved'] else 'FAIL'}(total-time={old_t})  "
                      f"nuova={'OK' if rec['new_solved'] else 'FAIL'}(total-time={new_t})")

    return {"zone": zone, "n_samples_requested": n_samples, "records": records}


def summarize_zone(zone_result):
    records = [r for r in zone_result["records"] if r.get("old_stats") and r.get("new_stats")]
    metrics = ["total_dist_m", "n_edges", "travel_time_s", "turn_delay_s",
               "signal_delay_s", "signals_crossed", "congestion_delay_s", "total_time_s"]

    def mean(xs):
        return round(sum(xs) / len(xs), 2) if xs else None

    summary = {"zone": zone_result["zone"],
               "n_pairs_both_solved": len(records),
               "n_pairs_total": len(zone_result["records"])}
    for m in metrics:
        old_vals = [r["old_stats"][m] for r in records]
        new_vals = [r["new_stats"][m] for r in records]
        old_mean, new_mean = mean(old_vals), mean(new_vals)
        delta = round(new_mean - old_mean, 2) if (old_mean is not None and new_mean is not None) else None
        pct = round(100 * delta / old_mean, 1) if (delta is not None and old_mean) else None
        summary[m] = {"old_mean": old_mean, "new_mean": new_mean, "delta": delta, "pct_change": pct}
    return summary


def main():
    parser = argparse.ArgumentParser(description="Confronto vecchia vs nuova versione (signal-delay per nodo vs per movimento)")
    parser.add_argument("zones", nargs="*", default=list(ZONE_CONFIGS.keys()), choices=list(ZONE_CONFIGS.keys()))
    parser.add_argument("--n-samples", type=int, default=N_SAMPLES_DEFAULT)
    parser.add_argument("--max-workers", type=int, default=4)
    parser.add_argument("--timeout", type=int, default=60)
    args = parser.parse_args()

    os.makedirs(OUT_DIR, exist_ok=True)
    print("Caricamento versione VECCHIA (git HEAD) di build_problems.py/domain.pddl...")
    old_bp = load_old_build_problems()
    old_domain_path = write_old_domain()

    all_results = {}
    all_summaries = {}
    for zone in args.zones:
        zone_result = run_zone(zone, old_bp, old_domain_path, args.n_samples,
                                max_workers=args.max_workers, timeout=args.timeout)
        summary = summarize_zone(zone_result)
        all_results[zone] = zone_result
        all_summaries[zone] = summary

        with open(os.path.join(OUT_DIR, f"results_{zone}.json"), "w", encoding="utf-8") as f:
            json.dump({"result": zone_result, "summary": summary}, f, ensure_ascii=False, indent=2)

        print(f"\n=== Riepilogo {zone} ({summary['n_pairs_both_solved']}/{summary['n_pairs_total']} coppie risolte da entrambe) ===")
        for m in ["total_time_s", "signal_delay_s", "congestion_delay_s", "n_edges", "total_dist_m"]:
            s = summary[m]
            print(f"  {m}: vecchia={s['old_mean']}  nuova={s['new_mean']}  "
                  f"delta={s['delta']} ({s['pct_change']}%)")

    with open(os.path.join(OUT_DIR, "results.json"), "w", encoding="utf-8") as f:
        json.dump({"results": all_results, "summaries": all_summaries}, f, ensure_ascii=False, indent=2)
    print(f"\nRisultati completi salvati in: {OUT_DIR}/results.json")


if __name__ == "__main__":
    main()

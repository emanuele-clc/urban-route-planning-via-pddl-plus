"""
benchmark.py
============
Benchmark dei tempi del progetto sulle tre mappe (piccola, media, grande).

Misura:
  1. Dimensione del problema (nodi, archi, semafori, fatti turn-time).
  2. Risoluzione con ENHSP: tempo reale (wall), tempo di pianificazione
     riportato da ENHSP, lunghezza del piano, nodi espansi, stati valutati.
     Ogni problema e' risolto piu' volte e si riportano media e minimo.
  3. Tempo della pipeline SUMO: estrazione dati (extract_sumo_data) e
     iniezione del piano (inject_signal_plan).
  4. Scalabilita': tempo di risoluzione al crescere del numero di nodi
     (zona media rigenerata a diverse dimensioni).

Uso:
    python scripts/benchmark.py                 # tutto
    python scripts/benchmark.py --runs 5        # 5 ripetizioni per ENHSP
    python scripts/benchmark.py --no-scaling    # salta la parte di scalabilita'
"""

import os
import re
import sys
import json
import time
import argparse
import subprocess

HERE = os.path.dirname(os.path.abspath(__file__))
ROOT = os.path.dirname(HERE)
PDDL_DIR = os.path.join(ROOT, "pddl_files")
NET_DIR = os.path.join(ROOT, "net_files")
OUT_DIR = os.path.join(ROOT, "sumo_comparison")
DOMAIN = os.path.join(PDDL_DIR, "domain.pddl")

ZONES = ("piccola", "media", "grande")
sys.path.insert(0, ROOT)


# ---------------------------------------------------------------------------
def find_enhsp():
    import site, sysconfig, glob
    cands = []
    try:
        cands.append(site.getusersitepackages())
    except Exception:
        pass
    cands += [sysconfig.get_path("purelib"), sysconfig.get_path("platlib")]
    for base in cands:
        if base and os.path.isdir(base):
            for root, _, files in os.walk(base):
                for f in files:
                    if f.endswith(".jar") and "enhsp" in f.lower():
                        return os.path.join(root, f)
    for base in ["/usr/local/lib", "/usr/lib", os.path.expanduser("~/.local/lib")]:
        for hit in glob.glob(os.path.join(base, "**", "enhsp*.jar"), recursive=True):
            return hit
    return None


def java_cmd():
    """Preferisce jdk4py se presente (utile nei sandbox senza Java 17)."""
    try:
        from jdk4py import JAVA
        return str(JAVA)
    except Exception:
        return "java"


def grep_int(pattern, text, default=None):
    m = re.search(pattern, text)
    return int(m.group(1)) if m else default


def grep_float(pattern, text, default=None):
    m = re.search(pattern, text)
    return float(m.group(1)) if m else default


# ---------------------------------------------------------------------------
def problem_size(zone):
    pf = os.path.join(PDDL_DIR, f"problem_{zone}.pddl")
    nf = os.path.join(NET_DIR, f"{zone}.net.xml")
    txt = open(pf, encoding="utf-8").read() if os.path.exists(pf) else ""
    net = open(nf, encoding="utf-8").read() if os.path.exists(nf) else ""
    n_obj = len(re.findall(r"^\s+n?\w+\s*(?:;.*)?$", txt.split("(:objects")[1].split("- location")[0])) \
        if "(:objects" in txt else None
    return {
        "nodi": grep_int(r";\s*(\d+)\s*nodi", txt),
        "archi": len(re.findall(r"\(road ", txt)),
        "turn_time_facts": len(re.findall(r"\(= \(turn-time", txt)),
        "signal_delay_facts": len(re.findall(r"\(= \(signal-delay", txt)),
        "tlLogic_net": len(re.findall(r"<tlLogic ", net)),
    }


def run_enhsp(zone, jar, java, heap="4g", runs=3):
    pf = os.path.join(PDDL_DIR, f"problem_{zone}.pddl")
    if not os.path.exists(pf):
        return {"error": "problema non trovato"}
    cmd = [java, f"-Xmx{heap}", "-jar", jar, "-o", DOMAIN, "-f", pf, "-s", "aibr"]
    walls, metrics = [], {}
    solved = False
    for _ in range(runs):
        t0 = time.time()
        r = subprocess.run(cmd, capture_output=True, text=True, timeout=600)
        wall = time.time() - t0
        out = r.stdout + r.stderr
        if "Problem Solved" in out:
            solved = True
            walls.append(wall)
            metrics = {
                "plan_length": grep_int(r"Plan-Length:(\d+)", out),
                "expanded_nodes": grep_int(r"Expanded Nodes:(\d+)", out),
                "states_evaluated": grep_int(r"States Evaluated:(\d+)", out),
                "planning_time_ms": grep_float(r"Planning Time \(msec\):\s*([\d.]+)", out),
                "heuristic_time_ms": grep_float(r"Heuristic Time \(msec\):\s*([\d.]+)", out),
                "search_time_ms": grep_float(r"Search Time \(msec\):\s*([\d.]+)", out),
            }
    if not solved:
        return {"solved": False}
    metrics.update({
        "solved": True,
        "runs": len(walls),
        "wall_mean_s": round(sum(walls) / len(walls), 2),
        "wall_min_s": round(min(walls), 2),
    })
    return metrics


# ---------------------------------------------------------------------------
def bench_pipeline(zone):
    """Tempo di estrazione dati SUMO e di iniezione del piano."""
    res = {}
    try:
        import importlib
        ex = importlib.import_module("extract_sumo_data")
        net = os.path.join(NET_DIR, f"{zone}.net.xml")
        t0 = time.time()
        data = ex.process(zone, net, os.path.join(os.path.dirname(HERE), "sumo_extracted")) \
            if hasattr(ex, "process") else None
        res["extract_s"] = round(time.time() - t0, 2) if data is not None else None
    except Exception as e:
        res["extract_s"] = None
        res["extract_err"] = str(e)[:80]
    try:
        import importlib
        inj = importlib.import_module("inject_signal_plan")
        t0 = time.time()
        inj.inject_zone(zone, verbose=False)
        res["inject_s"] = round(time.time() - t0, 2)
    except Exception as e:
        res["inject_s"] = None
        res["inject_err"] = str(e)[:80]
    return res


def bench_scaling(jar, java, sizes, heap="4g"):
    """Tempo di risoluzione al crescere dei nodi (zona media rigenerata)."""
    import build_problems as bp
    import tempfile
    osm = os.path.join(bp.OSM_DIR, "dublin_media_residenziale.osm")
    node_data, adj, sig, ehw = bp.build_contracted_graph(osm)
    rows = []
    for mx in sizes:
        selected = bp.select_connected_subgraph(node_data, adj, mx)
        ss = set(selected)
        edges = {(a, b): v for a in selected for b, v in adj.get(a, {}).items() if b in ss}
        tmp = tempfile.mkdtemp()
        bp.PDDL_DIR = tmp
        try:
            bp.generate("scal", osm, mx)
        except Exception:
            pass
        pf = os.path.join(tmp, "problem_scal.pddl")
        if not os.path.exists(pf):
            rows.append({"nodi": len(selected), "archi": len(edges), "solved": False})
            continue
        cmd = [java, f"-Xmx{heap}", "-jar", jar, "-o", DOMAIN, "-f", pf, "-s", "aibr"]
        t0 = time.time()
        try:
            r = subprocess.run(cmd, capture_output=True, text=True, timeout=300)
            wall = time.time() - t0
            out = r.stdout + r.stderr
            rows.append({
                "nodi": len(selected), "archi": len(edges),
                "solved": "Problem Solved" in out,
                "wall_s": round(wall, 1),
                "expanded": grep_int(r"Expanded Nodes:(\d+)", out),
            })
        except subprocess.TimeoutExpired:
            rows.append({"nodi": len(selected), "archi": len(edges),
                         "solved": False, "timeout": True})
    return rows


# ---------------------------------------------------------------------------
def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=3)
    ap.add_argument("--heap", default="4g")
    ap.add_argument("--no-scaling", action="store_true")
    ap.add_argument("--scaling-sizes", default="50,120,200,300")
    args = ap.parse_args()

    jar = find_enhsp()
    java = java_cmd()
    if not jar:
        print("[ERRORE] ENHSP non trovato (pip install up-enhsp)")
        sys.exit(1)
    print(f"ENHSP: {jar}\nJava:  {java}\n")

    result = {"runs": args.runs, "heap": args.heap, "zones": {}}
    for zone in ZONES:
        print(f"[{zone}] dimensione + ENHSP ({args.runs} run)...")
        size = problem_size(zone)
        enhsp = run_enhsp(zone, jar, java, heap=args.heap, runs=args.runs)
        pipe = bench_pipeline(zone)
        result["zones"][zone] = {"size": size, "enhsp": enhsp, "pipeline": pipe}
        if enhsp.get("solved"):
            print(f"  {size['nodi']} nodi, {size['archi']} archi | "
                  f"wall {enhsp['wall_mean_s']}s (min {enhsp['wall_min_s']}s) | "
                  f"piano {enhsp['plan_length']} | espansi {enhsp['expanded_nodes']}")
        else:
            print(f"  NON risolto")

    if not args.no_scaling:
        sizes = [int(x) for x in args.scaling_sizes.split(",")]
        print(f"\n[scalabilita'] media a {sizes} nodi...")
        result["scaling_media"] = bench_scaling(jar, java, sizes, heap=args.heap)
        for row in result["scaling_media"]:
            print(f"  {row['nodi']:>3} nodi: "
                  + (f"{row.get('wall_s')}s, {row.get('expanded')} espansi"
                     if row.get("solved") else
                     ("TIMEOUT" if row.get("timeout") else "non risolto")))

    cmp_path = os.path.join(OUT_DIR, "results.json")
    if os.path.exists(cmp_path):
        result["sumo_comparison"] = json.load(open(cmp_path, encoding="utf-8"))

    os.makedirs(OUT_DIR, exist_ok=True)
    out = os.path.join(OUT_DIR, "benchmark.json")
    with open(out, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)
    print(f"\nrisultati grezzi: {out}")


if __name__ == "__main__":
    main()

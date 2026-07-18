"""
run.py
------
Trova ENHSP automaticamente e risolve il problema PDDL+ scelto.

Uso:
    python run.py piccola
    python run.py media
    python run.py grande

Le funzioni trova_enhsp() e solve_problem() sono pensate per essere
riusate da altri moduli (es. signal_optimization/) senza duplicare la
logica di discovery/invocazione di ENHSP — vedi
2_traffic_signal_optimization.md, sez. 3.4 e 4.
"""

import sys, os, subprocess, glob, sysconfig
import site

DOMAIN = os.path.join(os.path.dirname(__file__), "domain.pddl")


def trova_enhsp():
    """Cerca enhsp.jar in tutte le cartelle Python (Windows, Mac, Linux)."""
    cartelle = [site.getusersitepackages(), sysconfig.get_path("purelib"), sysconfig.get_path("platlib")]

    # Cerca nelle cartelle site-packages
    for base in cartelle:
        if base and os.path.exists(base):
            for root, dirs, files in os.walk(base):
                for f in files:
                    if f.endswith(".jar") and "enhsp" in f.lower():
                        return os.path.join(root, f)

    # Fallback: versioni comuni di Python su Windows
    for ver in ["313", "312", "311", "310", "39"]:
        for base in [
            os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "Python", f"Python{ver}"),
            os.path.join(os.environ.get("APPDATA", ""), "Python", f"Python{ver}"),
            f"C:\\Python{ver}",
        ]:
            for hit in glob.glob(os.path.join(base, "**", "enhsp*.jar"), recursive=True):
                return hit

    # Fallback: Mac/Linux
    for base in ["/usr/local/lib", "/usr/lib", os.path.expanduser("~/.local/lib")]:
        for hit in glob.glob(os.path.join(base, "**", "enhsp*.jar"), recursive=True):
            return hit

    return None


def solve_problem(problem_path, domain_path=DOMAIN, jar=None, timeout=180):
    """Invoca ENHSP (-s aibr) su (domain_path, problem_path).
    Ritorna dict: {solved, output, plan_text, metric, plan_length,
    elapsed_ms, jar, error}. Non stampa/scrive nulla su disco — a
    differenza del blocco __main__ sotto, pensato per il solo uso CLI.
    Usata da signal_optimization/enhsp_eval.py per la validazione dei
    candidati (2_traffic_signal_optimization.md, sez. 3.4)."""
    if jar is None:
        jar = trova_enhsp()
    if not jar:
        return {"solved": False, "error": "ENHSP non trovato (pip install up-enhsp)"}
    if not os.path.exists(problem_path):
        return {"solved": False, "error": f"file non trovato: {problem_path}"}

    cmd = ["java", "-jar", jar, "-o", domain_path, "-f", problem_path, "-s", "aibr"]
    try:
        result = subprocess.run(cmd, capture_output=True, text=True, timeout=timeout)
    except subprocess.TimeoutExpired:
        return {"solved": False, "error": f"timeout ({timeout}s)"}
    except FileNotFoundError:
        return {"solved": False, "error": "java non trovato"}

    output = result.stdout + result.stderr
    solved = "Problem Solved" in output

    metric = None
    plan_length = None
    for line in output.splitlines():
        if "Metric (Search)" in line:
            try:
                metric = float(line.split(":", 1)[1].strip())
            except (IndexError, ValueError):
                pass
        elif "Plan-Length" in line:
            try:
                plan_length = int(line.split(":", 1)[1].strip())
            except (IndexError, ValueError):
                pass

    plan_lines = []
    in_plan = False
    for line in output.splitlines():
        if "Found Plan:" in line:
            in_plan = True
            continue
        if in_plan:
            if line.strip() == "" or "Plan-Length" in line:
                in_plan = False
            else:
                plan_lines.append(line.strip())

    return {
        "solved": solved,
        "output": output,
        "plan_text": "\n".join(plan_lines) if solved else None,
        "metric": metric,
        "plan_length": plan_length,
        "jar": jar,
        "error": None if solved else "ENHSP non ha trovato soluzione",
    }


def _main():
    zona = sys.argv[1] if len(sys.argv) > 1 else "piccola"
    if zona not in ("piccola", "media", "grande"):
        print("Uso: python run.py [piccola|media|grande]")
        sys.exit(1)

    problem = os.path.join(os.path.dirname(__file__), f"problem_{zona}.pddl")
    if not os.path.exists(problem):
        print(f"[ERRORE] File non trovato: {problem}")
        sys.exit(1)

    print(f"\n{'='*55}")
    print(f"  ENHSP — Zona: {zona.upper()}")
    print(f"  Domain : {DOMAIN}")
    print(f"  Problem: {problem}")
    print(f"{'='*55}\n")

    jar = trova_enhsp()
    if not jar:
        print("[ERRORE] ENHSP non trovato.")
        print("Installa con:  python -m pip install up-enhsp")
        sys.exit(1)
    print(f"ENHSP: {jar}\n")

    res = solve_problem(problem, domain_path=DOMAIN, jar=jar)
    output = res.get("output", res.get("error", ""))

    log_path = os.path.join(os.path.dirname(__file__), f"output_{zona}.txt")
    with open(log_path, "w", encoding="utf-8") as f:
        f.write(output)

    if res["solved"]:
        print("✅ PROBLEMA RISOLTO\n")
        in_plan = False
        for line in output.splitlines():
            if "Found Plan:" in line:
                in_plan = True
                print("── PIANO ───────────────────────────────────────────────")
                continue
            if in_plan:
                if line.strip() == "" or "Plan-Length" in line:
                    in_plan = False
                else:
                    print(" ", line)
        print()
        for line in output.splitlines():
            if any(k in line for k in ("Plan-Length", "Elapsed Time", "Planning Time",
                                        "Expanded Nodes", "Metric")):
                print(" ", line.strip())
    else:
        print("❌ Nessuna soluzione trovata. Log completo:\n")
        print(output)

    print(f"\nLog salvato in: output_{zona}.txt")
    print(f"{'='*55}\n")


if __name__ == "__main__":
    _main()

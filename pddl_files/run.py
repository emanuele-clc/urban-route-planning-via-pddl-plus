"""
run.py
------
Trova ENHSP automaticamente e risolve il problema PDDL+ scelto.

Uso:
    python run.py piccola
    python run.py media
    python run.py grande
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


# ── Argomento ───────────────────────────────────────────────
zona = sys.argv[1] if len(sys.argv) > 1 else "piccola"
if zona not in ("piccola", "media", "grande"):
    print("Uso: python run.py [piccola|media|grande]")
    sys.exit(1)

PROBLEM = os.path.join(os.path.dirname(__file__), f"problem_{zona}.pddl")
if not os.path.exists(PROBLEM):
    print(f"[ERRORE] File non trovato: {PROBLEM}")
    sys.exit(1)

print(f"\n{'='*55}")
print(f"  ENHSP — Zona: {zona.upper()}")
print(f"  Domain : {DOMAIN}")
print(f"  Problem: {PROBLEM}")
print(f"{'='*55}\n")

# ── Trova ENHSP ─────────────────────────────────────────────
jar = trova_enhsp()
if not jar:
    print("[ERRORE] ENHSP non trovato.")
    print("Installa con:  python -m pip install up-enhsp")
    sys.exit(1)

print(f"ENHSP: {jar}\n")

# ── Esegui ENHSP ─────────────────────────────────────────────
cmd = ["java", "-jar", jar, "-o", DOMAIN, "-f", PROBLEM, "-s", "aibr"]
result = subprocess.run(cmd, capture_output=True, text=True)
output = result.stdout + result.stderr

# ── Salva log grezzo ─────────────────────────────────────────
log_path = os.path.join(os.path.dirname(__file__), f"output_{zona}.txt")
with open(log_path, "w", encoding="utf-8") as f:
    f.write(output)

# ── Mostra risultati ─────────────────────────────────────────
if "Problem Solved" in output:
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

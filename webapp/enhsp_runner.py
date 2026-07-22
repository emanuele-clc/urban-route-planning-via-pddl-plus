"""
enhsp_runner.py
----------------
Discovery ed esecuzione di ENHSP, parsing del piano prodotto. Estratto da
webapp/app.py — usato dalle route /api/solve e /api/replan.
"""
import os
import re
import glob
import sysconfig
import subprocess
import tempfile

PROJECT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
DOMAIN_PATH = os.path.join(PROJECT_ROOT, 'pddl_files', 'domain.pddl')

# Heap per ENHSP. Il grounding di start-move (3 argomenti: prev, from, to)
# cresce molto col numero di nodi: con la heap di default la JVM va in
# OutOfMemory gia' sotto i 400 nodi. Con 2 GB si arriva comodamente a ~300.
# Sovrascrivibile con la variabile d'ambiente ENHSP_HEAP (es. "4g").
# 6 GB: -Xmx fissa solo il tetto, la JVM non alloca subito, quindi e' sicuro
# anche su macchine con 8 GB. Se la JVM non parte, abbassalo (ENHSP_HEAP=2g).
ENHSP_HEAP = os.environ.get('ENHSP_HEAP', '6g')

# Tetto sul numero di nodi PASSATI A ENHSP (non alla visualizzazione della
# mappa, che non ha limiti). Il grounding di start-move/signal-delay a tre
# argomenti (prev, from, to) e' pesante: su un ambiente modesto (3 GB, 1 core,
# heap 2 GB) 300 nodi -> 8 s, 400 -> 14 s, 939 -> memoria esaurita. Il default
# e' quindi 1000, che lascia passare l'intera zona media (939 nodi) su un PC
# normale con heap 6 GB; puo' richiedere qualche minuto perche' non c'e'
# timeout. Su grande (3756 nodi) resta un tetto di sicurezza: per solve
# affidabili conviene comunque restare sotto i ~400 nodi.
# Regolabile con MAX_SOLVABLE_NODES; la heap con ENHSP_HEAP.
MAX_SOLVABLE_NODES = int(os.environ.get('MAX_SOLVABLE_NODES', '1000'))

# Timeout di ENHSP in secondi. 0 = nessun limite: sui grafi grandi il grounding
# puo' richiedere parecchi minuti e interromperlo a 180 s buttava via lavoro
# gia' fatto. Impostare ENHSP_TIMEOUT=300 per rimettere un tetto.
_t = int(os.environ.get('ENHSP_TIMEOUT', '0'))
ENHSP_TIMEOUT = _t if _t > 0 else None


def trova_enhsp():
    import site as _site
    cartelle = []
    try: cartelle.append(_site.getusersitepackages())
    except Exception: pass
    cartelle += [sysconfig.get_path("purelib"), sysconfig.get_path("platlib")]
    for base in cartelle:
        if base and os.path.exists(base):
            for root, dirs, files in os.walk(base):
                for f in files:
                    if f.endswith(".jar") and "enhsp" in f.lower():
                        return os.path.join(root, f)
    for ver in ["313", "312", "311", "310", "39"]:
        for base in [
            os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "Python", f"Python{ver}"),
            os.path.join(os.environ.get("APPDATA", ""), "Python", f"Python{ver}"),
            f"C:\\Python{ver}",
        ]:
            for hit in glob.glob(os.path.join(base, "**", "enhsp*.jar"), recursive=True):
                return hit
    for base in ["/usr/local/lib", "/usr/lib", os.path.expanduser("~/.local/lib")]:
        for hit in glob.glob(os.path.join(base, "**", "enhsp*.jar"), recursive=True):
            return hit
    return None


def enhsp_cmd(jar, domain, problem, heuristic=None):
    cmd = ["java", f"-Xmx{ENHSP_HEAP}", "-jar", jar,
           "-o", domain, "-f", problem, "-s", "aibr"]
    if heuristic:
        cmd += ["-h", heuristic]
    return cmd


# Firma dell'overflow numerico dell'euristica 'aibr' (Float.MAX_VALUE):
# su percorsi molto lunghi (~70+ hop) l'euristica interval-based cresce
# esponenzialmente e satura, ed ENHSP legge la saturazione come "irraggiungibile"
# anche quando una soluzione esiste (falso negativo, verificato sperimentalmente
# su dublin_grande_porto.osm). 'blind' (nessuna guida euristica, ma numericamente
# robusta) risolve correttamente questi casi — usata solo come fallback perche'
# su problemi con molta piu' diramazione aibr guida la ricerca meglio.
AIBR_OVERFLOW_SIGNATURE = "3.4028235E38"


def run_enhsp_output(jar, domain_abs, pddl_path, timeout):
    """Esegue ENHSP e ritorna l'output testuale. Se 'aibr' dichiara il
    problema irrisolvibile per overflow (vedi AIBR_OVERFLOW_SIGNATURE),
    ritenta una volta con l'euristica 'blind' prima di arrendersi."""
    result = subprocess.run(enhsp_cmd(jar, domain_abs, pddl_path),
                            capture_output=True, text=True, timeout=timeout)
    output = result.stdout + result.stderr
    if "Problem Solved" not in output and AIBR_OVERFLOW_SIGNATURE in output:
        result = subprocess.run(enhsp_cmd(jar, domain_abs, pddl_path, heuristic="blind"),
                                capture_output=True, text=True, timeout=timeout)
        output = result.stdout + result.stderr
    return output


def diagnose_enhsp(output):
    """Traduce l'esito di ENHSP in un messaggio utile all'utente.
    Distinguere le cause e' importante: 'nessuna soluzione' e 'memoria
    esaurita' richiedono azioni completamente diverse."""
    if 'OutOfMemoryError' in output or 'GC overhead' in output:
        return (f"Memoria insufficiente per ENHSP (heap attuale: {ENHSP_HEAP}). "
                f"Il numero di istanze da generare cresce col cubo dei nodi, "
                f"quindi oltre ~{MAX_SOLVABLE_NODES} nodi non basta aumentare la RAM: "
                f"rigenera il grafo con meno nodi. In alternativa alza "
                f"ENHSP_HEAP e MAX_SOLVABLE_NODES.")
    if 'Problem unsolvable' in output or 'unsolvable' in output.lower():
        return ("ENHSP dichiara il problema irrisolvibile: la destinazione non e' "
                "raggiungibile dal punto di partenza con i vincoli attuali.")
    return ("ENHSP non ha trovato soluzione. Cause tipiche: troppi nodi "
            "selezionati, oppure goal non raggiungibile dallo start.")


def parse_plan(output):
    plan_lines = []; in_plan = False
    for line in output.splitlines():
        if "Found Plan:" in line: in_plan = True; continue
        if in_plan:
            if line.strip() == "" or "Plan-Length" in line: in_plan = False
            else: plan_lines.append(line.strip())

    route_names = []
    for line in plan_lines:
        # start-move a 3 argomenti: (start-move ?prev ?from ?to)
        m = re.search(r'\(start-move\s+(\S+)\s+(\S+)\s+(\S+)\)', line, re.IGNORECASE)
        if m:
            frm = m.group(2).lower(); to = m.group(3).lower()
            if not route_names: route_names.append(frm)
            route_names.append(to)

    plan_time_ms = None
    for line in output.splitlines():
        if "Planning Time (msec)" in line:
            m = re.search(r':\s*([\d.]+)', line)
            if m: plan_time_ms = float(m.group(1)); break

    return "\n".join(plan_lines), route_names, plan_time_ms


def run_enhsp(pddl_content):
    """Risolve un problema PDDL+ con ENHSP. Ritorna (plan_text, route, ms, errore)."""
    jar = trova_enhsp()
    domain_abs = os.path.abspath(DOMAIN_PATH)
    if not jar:
        return None, None, None, "ENHSP non trovato — installa con: pip install up-enhsp"
    if not os.path.exists(domain_abs):
        return None, None, None, f"domain.pddl non trovato in: {domain_abs}"

    tmp_dir = tempfile.mkdtemp()
    pddl_path = os.path.join(tmp_dir, 'problem.pddl')
    with open(pddl_path, 'w', encoding='utf-8') as f:
        f.write(pddl_content)
    try:
        output = run_enhsp_output(jar, domain_abs, pddl_path, ENHSP_TIMEOUT)
    except subprocess.TimeoutExpired:
        return None, None, None, (f"ENHSP ha superato il timeout ({ENHSP_TIMEOUT}s): "
                                  "riduci i nodi o alza ENHSP_TIMEOUT")
    except FileNotFoundError:
        return None, None, None, "Java non trovato — installa Java 17+"

    if "Problem Solved" not in output:
        return None, None, None, diagnose_enhsp(output)
    plan_text, route, ms = parse_plan(output)
    return plan_text, route, ms, None

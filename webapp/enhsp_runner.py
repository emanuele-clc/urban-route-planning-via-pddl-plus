"""
enhsp_runner.py
----------------
Discovery and execution of ENHSP, parsing of the produced plan. Extracted
from webapp/app.py — used by the /api/solve and /api/replan routes.
"""
import os
import re
import glob
import sysconfig
import subprocess
import tempfile

PROJECT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
DOMAIN_PATH = os.path.join(PROJECT_ROOT, 'pddl_files', 'domain.pddl')

# Heap for ENHSP. The grounding of start-move (3 arguments: prev, from, to)
# grows a lot with the number of nodes: with the default heap the JVM goes
# OutOfMemory already under 400 nodes. With 2 GB it comfortably reaches ~300.
# Overridable with the ENHSP_HEAP environment variable (e.g. "4g").
# 6 GB: -Xmx only sets the ceiling, the JVM doesn't allocate it immediately,
# so it's safe even on machines with 8 GB. If the JVM fails to start, lower
# it (ENHSP_HEAP=2g).
ENHSP_HEAP = os.environ.get('ENHSP_HEAP', '6g')

# Cap on the number of nodes PASSED TO ENHSP (not on the map display, which
# has no limits). The grounding of start-move/signal-delay with three
# arguments (prev, from, to) is heavy: on a modest environment (3 GB, 1
# core, 2 GB heap) 300 nodes -> 8 s, 400 -> 14 s, 939 -> out of memory. The
# default is therefore 1000, which lets the entire medium zone (939 nodes)
# through on a normal PC with a 6 GB heap; it can take a few minutes since
# there is no timeout. On the large zone (3756 nodes) it remains a safety
# cap: for reliable solves it's still better to stay under ~400 nodes.
# Adjustable with MAX_SOLVABLE_NODES; the heap with ENHSP_HEAP.
MAX_SOLVABLE_NODES = int(os.environ.get('MAX_SOLVABLE_NODES', '1000'))

# ENHSP timeout in seconds. 0 = no limit: on large graphs grounding can take
# several minutes and interrupting it at 180 s threw away work already
# done. Set ENHSP_TIMEOUT=300 to put a cap back in.
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


# Signature of the 'aibr' heuristic's numeric overflow (Float.MAX_VALUE): on
# very long routes (~70+ hops) the interval-based heuristic grows
# exponentially and saturates, and ENHSP reads the saturation as
# "unreachable" even when a solution exists (false negative, verified
# experimentally on dublin_grande_porto.osm). 'blind' (no heuristic
# guidance, but numerically robust) correctly solves these cases — used
# only as a fallback because on problems with much more branching aibr
# guides the search better.
AIBR_OVERFLOW_SIGNATURE = "3.4028235E38"


def run_enhsp_output(jar, domain_abs, pddl_path, timeout):
    """Runs ENHSP and returns (text output, fallback_used). If 'aibr'
    declares the problem unsolvable due to overflow (see
    AIBR_OVERFLOW_SIGNATURE), retries once with the 'blind' heuristic
    before giving up; fallback_used indicates whether the retry kicked
    in."""
    result = subprocess.run(enhsp_cmd(jar, domain_abs, pddl_path),
                            capture_output=True, text=True, timeout=timeout)
    output = result.stdout + result.stderr
    if "Problem Solved" not in output and AIBR_OVERFLOW_SIGNATURE in output:
        result = subprocess.run(enhsp_cmd(jar, domain_abs, pddl_path, heuristic="blind"),
                                capture_output=True, text=True, timeout=timeout)
        output = result.stdout + result.stderr
        return output, True
    return output, False


def diagnose_enhsp(output):
    """Translates ENHSP's outcome into a message useful to the user.
    Distinguishing the causes matters: 'no solution' and 'out of memory'
    require completely different actions."""
    if 'OutOfMemoryError' in output or 'GC overhead' in output:
        return (f"Insufficient memory for ENHSP (current heap: {ENHSP_HEAP}). "
                f"The number of instances to generate grows with the cube of "
                f"the nodes, so beyond ~{MAX_SOLVABLE_NODES} nodes increasing "
                f"RAM is not enough: regenerate the graph with fewer nodes. "
                f"Alternatively raise ENHSP_HEAP and MAX_SOLVABLE_NODES.")
    if 'Problem unsolvable' in output or 'unsolvable' in output.lower():
        return ("ENHSP declares the problem unsolvable: the destination is "
                "not reachable from the starting point with the current constraints.")
    return ("ENHSP found no solution. Typical causes: too many nodes "
            "selected, or the goal is not reachable from the start.")


def parse_plan(output):
    plan_lines = []; in_plan = False
    for line in output.splitlines():
        if "Found Plan:" in line: in_plan = True; continue
        if in_plan:
            if line.strip() == "" or "Plan-Length" in line: in_plan = False
            else: plan_lines.append(line.strip())

    route_names = []
    for line in plan_lines:
        # start-move with 3 arguments: (start-move ?prev ?from ?to)
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
    """Solves a PDDL+ problem with ENHSP. Returns (plan_text, route, ms, error, fallback_used)."""
    jar = trova_enhsp()
    domain_abs = os.path.abspath(DOMAIN_PATH)
    if not jar:
        return None, None, None, "ENHSP not found — install with: pip install up-enhsp", False
    if not os.path.exists(domain_abs):
        return None, None, None, f"domain.pddl not found at: {domain_abs}", False

    tmp_dir = tempfile.mkdtemp()
    pddl_path = os.path.join(tmp_dir, 'problem.pddl')
    with open(pddl_path, 'w', encoding='utf-8') as f:
        f.write(pddl_content)
    try:
        output, used_fallback = run_enhsp_output(jar, domain_abs, pddl_path, ENHSP_TIMEOUT)
    except subprocess.TimeoutExpired:
        return None, None, None, (f"ENHSP exceeded the timeout ({ENHSP_TIMEOUT}s): "
                                  "reduce the nodes or raise ENHSP_TIMEOUT"), False
    except FileNotFoundError:
        return None, None, None, "Java not found — install Java 17+", False

    if "Problem Solved" not in output:
        return None, None, None, diagnose_enhsp(output), used_fallback
    plan_text, route, ms = parse_plan(output)
    return plan_text, route, ms, None, used_fallback

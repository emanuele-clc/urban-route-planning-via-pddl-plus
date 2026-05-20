import os, sys, subprocess, re, xml.etree.ElementTree as ET
from collections import defaultdict
import heapq

# ── Dijkstra su net.xml ───────────────────────────────────────
def build_sumo_graph(net_path):
    root = ET.parse(net_path).getroot()
    jpos = {}
    for j in root.findall('junction'):
        jpos[j.get('id')] = (float(j.get('x', 0)), float(j.get('y', 0)))
    graph = defaultdict(list)
    for e in root.findall('edge'):
        eid = e.get('id')
        if eid.startswith(':'): continue
        fr, to = e.get('from'), e.get('to')
        if not fr or not to: continue
        lanes = e.findall('lane')
        length = float(lanes[0].get('length', 1)) if lanes else 1.0
        graph[fr].append((to, eid, length))
    return graph, jpos

def dijkstra(graph, start, goal):
    dist = {start: 0}; prev = {}; heap = [(0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist.get(u, float('inf')): continue
        if u == goal: break
        for v, eid, length in graph[u]:
            nd = d + length
            if nd < dist.get(v, float('inf')):
                dist[v] = nd; prev[v] = (u, eid)
                heapq.heappush(heap, (nd, v))
    if goal not in prev: return None
    edges = []
    cur = goal
    while cur in prev:
        p, eid = prev[cur]; edges.append(eid); cur = p
    return list(reversed(edges))

def pddl_name_to_junction(pname, junc_ids):
    """Mappa un nome PDDL (es. n1193756) alla junction SUMO corrispondente."""
    suffix = pname.lstrip('n')
    if suffix in junc_ids: return suffix
    matches = [j for j in junc_ids if j.endswith(suffix) and not j.startswith(':')]
    if matches: return min(matches, key=len)
    return None

def compute_edges_from_pddl(pddl_path, net_path):
    """Legge start/goal dal PDDL, fa Dijkstra sul net.xml, ritorna (edges_str, cfg)."""
    text = open(pddl_path).read()
    # estrai start: (at <nome>)
    m_start = re.search(r'\(at\s+([A-Za-z0-9_]+)\)', text)
    # estrai goal: (:goal (at <nome>))
    m_goal  = re.search(r':goal\s+\(at\s+([A-Za-z0-9_]+)\)', text)
    if not m_start or not m_goal:
        print("[ERRORE] Impossibile trovare start/goal nel PDDL.")
        sys.exit(1)
    start_name = m_start.group(1)
    goal_name  = m_goal.group(1)

    graph, jpos = build_sumo_graph(net_path)
    junc_ids = set(jpos.keys())

    start_j = pddl_name_to_junction(start_name, junc_ids)
    goal_j  = pddl_name_to_junction(goal_name,  junc_ids)
    if not start_j:
        print(f"[ERRORE] Junction per '{start_name}' non trovata in {net_path}")
        sys.exit(1)
    if not goal_j:
        print(f"[ERRORE] Junction per '{goal_name}' non trovata in {net_path}")
        sys.exit(1)

    print(f"  START: {start_name} → {start_j}")
    print(f"  GOAL : {goal_name} → {goal_j}")

    edges = dijkstra(graph, start_j, goal_j)
    if not edges:
        print(f"[ERRORE] Nessun percorso SUMO da {start_j} a {goal_j}")
        sys.exit(1)

    sp = jpos[start_j]; gp = jpos[goal_j]
    return ' '.join(edges), {
        'start': start_name, 'goal': goal_name,
        'x': sp[0], 'y': sp[1],
    }

# ── Argomenti ────────────────────────────────────────────────
# Uso standard:  python sumo_visualize.py [piccola|media|grande]
# Uso dinamico:  python sumo_visualize.py pddl <percorso_pddl> [piccola|media|grande]
zona = sys.argv[1] if len(sys.argv) > 1 else "piccola"
dynamic_pddl = None

if zona == "pddl":
    # modalità dinamica: legge il PDDL e calcola il percorso
    if len(sys.argv) < 3:
        print("Uso dinamico: python sumo_visualize.py pddl <file.pddl> [piccola|media|grande]")
        sys.exit(1)
    dynamic_pddl = sys.argv[2]
    zona = sys.argv[3] if len(sys.argv) > 3 else "piccola"

if zona not in ("piccola", "media", "grande"):
    print("Uso: python sumo_visualize.py [piccola|media|grande]")
    print("     python sumo_visualize.py pddl <file.pddl> [piccola|media|grande]")
    sys.exit(1)

BASE = os.path.dirname(os.path.abspath(__file__))
OUT  = os.path.join(BASE, "cfg_files")
os.makedirs(OUT, exist_ok=True)

CONFIGS = {
    "piccola": {
        "net": os.path.join(BASE, "net_files", "piccola.net.xml"),
        # Dijkstra da 659788 (Liffey St Upper) a cluster Aungier St
        # Percorso: Liffey St → Great Strand St → Capel St → Grattan Bridge
        #           → Essex Quay → Fishamble St → Lord Edward St → Cork Hill
        #           → Dame St → South Great George's St → Aungier St
        "edges": (
            "4396046#0 4396046#1 18927706 1478689539 1062391643#0 "
            "4396056 1288830596 1179644329 1179644328 "
            "1254511872 1254511870 1254511871 125864859 "
            "5976028#2 5976028#3 16247623#1 "
            "4396059#0 4396059#2 846644599 668344588 "
            "-317003249#3 -317003249#2 -369564011 -5826896"
        ),
        "zoom": 3000, "x": 663, "y": 750,
        "dist_m": 1570, "time_s": 194,
        "start": "Liffey Street Upper", "goal": "Aungier Street",
    },
    "media": {
        "net": os.path.join(BASE, "net_files", "media.net.xml"),
        # Dijkstra da 2876012509 (Leeson St Upper) a 8752842641 (Saint Mary's Road)
        "edges": (
            "119860706 612719563#1 283771398#1 14041596#1 8111025#0 "
            "48520199#1 -370154357 -370154355 -128521564 20834536 110407380"
        ),
        "zoom": 3000, "x": 1144, "y": 2131,
        "dist_m": 1623, "time_s": 150,
        "start": "Leeson Street Upper", "goal": "Saint Mary's Road",
    },
    "grande": {
        "net": os.path.join(BASE, "net_files", "grande.net.xml"),
        # Dijkstra da 28244374 (St Patrick's Road) a 11822804242 (Botanic Avenue)
        # Segmento per segmento seguendo il piano ENHSP
        "edges": (
            "1159857185 1159857184 "
            "-130294072#2 -130294072#1 -130294072#0 "
            "-1293323158 -4540453 -1316170357 "
            "378882695#1 378882694 4539231 -130776836 "
            "-56007691#4 -56007691#3 -56007691#2 -56007691#0"
        ),
        "zoom": 3000, "x": 246, "y": 4860,
        "dist_m": 1335, "time_s": 147,
        "start": "St Patrick's Road", "goal": "Botanic Avenue",
    },
}

cfg = CONFIGS[zona]
NET  = cfg["net"]

# ── Modalità dinamica: sovrascrive edges/x/y dal PDDL ────────
if dynamic_pddl:
    print(f"[DINAMICO] Calcolo percorso da: {dynamic_pddl}")
    edges_str, dyn = compute_edges_from_pddl(dynamic_pddl, NET)
    cfg = dict(cfg)  # copia per non modificare l'originale
    cfg['edges'] = edges_str
    cfg['x']     = dyn['x']
    cfg['y']     = dyn['y']
    cfg['start'] = dyn['start']
    cfg['goal']  = dyn['goal']
    cfg['dist_m'] = '?'
    cfg['time_s'] = '?'
    zona = zona + "_custom"

# ── File di route ─────────────────────────────────────────────
ROU_PATH = os.path.join(OUT, f"{zona}_piano.rou.xml")
with open(ROU_PATH, "w") as f:
    f.write("""<?xml version="1.0" encoding="UTF-8"?>
<routes>
    <vType id="auto" accel="1.5" decel="3.0" sigma="0.0"
           length="4.5" maxSpeed="4.0" color="1,0,0"
           width="2.0" shape="passenger"/>
    <route id="piano_enhsp" edges="{edges}"/>
    <vehicle id="veicolo_enhsp" type="auto" route="piano_enhsp"
             depart="1" departSpeed="0"/>
</routes>
""".format(edges=cfg["edges"]))

# ── Impostazioni grafica ──────────────────────────────────────
GUI_PATH = os.path.join(OUT, f"gui_{zona}.xml")
with open(GUI_PATH, "w") as f:
    f.write("""<viewsettings>
    <scheme name="real world"/>
    <delay value="200"/>
    <viewport zoom="{zoom}" x="{x}" y="{y}"/>
    <vehicles vehicleMode="0" vehicleQuality="2"
              vehicleExaggeration="15" showBlinker="true"
              colorScheme="given/assigned vehicle color"/>
</viewsettings>
""".format(**cfg))

# ── Config SUMO ───────────────────────────────────────────────
CFG_PATH = os.path.join(OUT, f"{zona}.sumocfg")
with open(CFG_PATH, "w") as f:
    f.write("""<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="{net}"/>
        <route-files value="{rou}"/>
        <gui-settings-file value="{gui}"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="800"/>
    </time>
</configuration>
""".format(
        net=NET,
        rou=os.path.abspath(ROU_PATH),
        gui=os.path.abspath(GUI_PATH),
    ))

# ── Trova sumo-gui ────────────────────────────────────────────
def trova_sumo_bin(nome):
    sumo_home = os.environ.get("SUMO_HOME", "")
    candidati = []
    if sumo_home:
        candidati += [os.path.join(sumo_home, "bin", nome + ".exe"),
                      os.path.join(sumo_home, "bin", nome)]
    for base in [r"C:\Program Files (x86)\Eclipse\Sumo",
                 r"C:\Program Files\Eclipse\Sumo", r"C:\Sumo"]:
        candidati.append(os.path.join(base, "bin", nome + ".exe"))
    for c in candidati:
        if os.path.exists(c):
            return c
    try:
        subprocess.run([nome, "--version"], capture_output=True, check=True)
        return nome
    except Exception:
        return None

sumo_gui = trova_sumo_bin("sumo-gui")
if not sumo_gui:
    print("[ERRORE] sumo-gui non trovato.")
    sys.exit(1)

# ── Avvio ─────────────────────────────────────────────────────
print(f"Zona: {zona.upper()}")
print(f"  Percorso : {cfg['start']} → {cfg['goal']}")
dist_val = cfg['dist_m']
time_val = cfg['time_s']
if dist_val != '?':
    print(f"  Distanza : {dist_val} m")
    print(f"  Tempo    : {time_val} s (a 30 km/h, senza traffico)")
else:
    print(f"  (distanza/tempo calcolati da ENHSP)")
print()
print(f"File generati:")
print(f"  Route : {ROU_PATH}")
print(f"  Config: {CFG_PATH}")
print()
print("Apro sumo-gui...")
print("→ Premi ▶ Play — l'auto rossa parte al secondo 1")
print("→ Ctrl+A per adattare la vista")
print("→ Click destro sull'auto → Track per seguirla")

subprocess.Popen([sumo_gui, "-c", CFG_PATH])

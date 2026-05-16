import os, sys, subprocess

zona = sys.argv[1] if len(sys.argv) > 1 else "piccola"
if zona not in ("piccola", "media", "grande"):
    print("Uso: python sumo_visualize.py [piccola|media|grande]")
    sys.exit(1)

BASE = os.path.dirname(os.path.abspath(__file__))
OUT  = os.path.join(BASE, "cfg_files")
os.makedirs(OUT, exist_ok=True)

CONFIGS = {
    "piccola": {
        "net": os.path.join(BASE, "net_files", "piccola.net.xml"),
        # Percorso connesso BFS — START: Liffey St Upper → GOAL: Aungier St
        "edges": (
            "39994843 -1126998263#0 1478689539 1062391643#0 "
            "4396056 1288830596 1179644329 1179644328 1254511872 1254511870 1254511871 125864859 5976028#2 5976028#3 "
            "16247623#1 4396059#0 4396059#2 846644599 668344588 "
            "-317003249#3 -317003249#2 -369564011"
        ),
        "zoom": 3000, "x": 663, "y": 749,
        "dist_m": 1570, "time_s": 194,
        "start": "Liffey Street Upper", "goal": "Aungier Street",
    },
    "media": {
        "net": os.path.join(BASE, "net_files", "media.net.xml"),
        # Percorso connesso BFS — START: Leeson St Upper → GOAL: Saint Mary's Road
        "edges": (
            "25466631#1 4934444#0 147463637#0 147463637#1 38864102 "
            "-22716630#2 -22716630#1 -22716630#0 "
            "370154352#1 365945819#0 365945819#1 110407380"
        ),
        "zoom": 2000, "x": 1800, "y": 2200,
        "dist_m": 1623, "time_s": 150,
        "start": "Leeson Street Upper", "goal": "Saint Mary's Road",
    },
    "grande": {
        "net": os.path.join(BASE, "net_files", "grande.net.xml"),
        # Percorso connesso BFS — START: Sherrard St → GOAL: Botanic Avenue
        "edges": (
            "1159857185 1159857184 "
            "-130294072#2 -130294072#1 -130294072#0 "
            "-1293323158 -4540453 -1316170357 "
            "378882695#1 378882694 4539231 -130776836 "
            "-56007691#4 -56007691#3 -56007691#2 -56007691#0"
        ),
        "zoom": 1500, "x": 246, "y": 4860,
        "dist_m": 1335, "time_s": 142,
        "start": "Sherrard Street Lower", "goal": "Botanic Avenue",
    },
}

cfg = CONFIGS[zona]
NET  = cfg["net"]

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
print(f"  Distanza : {cfg['dist_m']} m")
print(f"  Tempo    : {cfg['time_s']} s (a 30 km/h, senza traffico)")
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

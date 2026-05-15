#!/usr/bin/env python3
"""
sumo_validation.py — Valida il piano PDDL+ con simulazione fisica
==================================================================

Confronta il tempo trovato da ENHSP (1430 s) con quello calcolato
simulando il percorso sulla rete stradale reale di Cosenza.

Metodo principale: OSMnx + velocità reali da OSM
  → usa lo stesso grafo del modello PDDL+, velocità dai tag maxspeed OSM,
    fisica semplificata (velocità costante, no semafori) — confronto "puro"
    tra il modello PDDL+ e la realtà stradale.

Metodo bonus: SUMO (se installato)
  → simulazione microscopica con accelerazione, frenata, semafori.
    Se SUMO non è raggiungibile o la rete è disconnessa, viene saltato.

Uso:
    python sumo_validation.py

Output:
    output/sumo_comparison.json   — report confronto
"""

import os, sys, json, subprocess, shutil
import xml.etree.ElementTree as ET
from pathlib import Path

try:
    import osmnx as ox
    import networkx as nx
except ImportError:
    print("❌ Installa dipendenze: pip install osmnx networkx")
    sys.exit(1)

try:
    import requests
except ImportError:
    print("❌ Installa requests: pip install requests")
    sys.exit(1)

# ── Configurazione ─────────────────────────────────────────────────────────────

NODE_MAP_FILE = Path("output/node_map.json")
OUTPUT_DIR    = Path("output/sumo")

ENHSP_TIME = 1240  # secondi (piano trovato da ENHSP sat-hadd — aggiornare dopo re-run)

# Piano ENHSP: sequenza di nodi OSM (da node_map.json)
# NOTA: OSM ID (secondo campo) da aggiornare dopo aver rieseguito osm_to_pddl.py
#       su Dublin e ENHSP. Le coordinate GPS sono approssimative.
PLAN_LOCS = [
    ("loc000", 0,  53.3532, -6.2638),   # START — Parnell Square (nord)
    ("loc021", 0,  53.3498, -6.2620),   # O'Connell Bridge
    ("loc008", 0,  53.3450, -6.2548),   # College Green / Trinity
    ("loc003", 0,  53.3410, -6.2611),   # Dame Street
    ("loc018", 0,  53.3382, -6.2591),   # St. Stephen's Green
    ("loc010", 0,  53.3325, -6.2588),   # Harcourt Street
    ("loc005", 0,  53.3288, -6.2580),   # Ranelagh
    ("loc038", 0,  53.3235, -6.2640),   # Rathmines Road
    ("loc039", 0,  53.3221, -6.2655),   # GOAL — Rathmines (sud)
]

START_LON, START_LAT = -6.2638, 53.3532
GOAL_LON,  GOAL_LAT  = -6.2655, 53.3221

BBOX = (53.30, -6.42, 53.42, -6.17)  # (south, west, north, east) — Dublin

# ═══════════════════════════════════════════════════════════════════════════════
# PARTE 1 — Validazione con OSMnx (metodo principale, sempre disponibile)
# ═══════════════════════════════════════════════════════════════════════════════

def load_osmnx_graph():
    """
    Scarica la rete stradale di Cosenza con OSMnx e calcola le velocità
    e i tempi di percorrenza per ogni arco.

    ox.add_edge_speeds() aggiunge il campo 'speed_kph' basandosi sui tag
    maxspeed di OSM; dove assente usa una velocità di default per tipo di strada.

    ox.add_edge_travel_times() calcola 'travel_time' (secondi) come:
        travel_time = length / (speed_kph / 3.6)
    """
    print("  → Download rete Dublin con OSMnx...")
    G = ox.graph_from_place("Dublin, Ireland", network_type="drive", simplify=True)
    G = ox.add_edge_speeds(G)
    G = ox.add_edge_travel_times(G)
    n_nodes = len(G.nodes)
    n_edges = len(G.edges)
    print(f"  ✓ Grafo caricato: {n_nodes} nodi, {n_edges} archi")
    return G


def find_graph_node(G, osm_id: int, lat: float, lon: float):
    """
    Cerca il nodo OSM nel grafo. Se non trovato (nodo troppo recente o
    rimosso), usa il nodo più vicino alle coordinate GPS.
    """
    if G.has_node(osm_id):
        return osm_id
    # fallback: nodo più vicino per coordinate
    nearest = ox.nearest_nodes(G, lon, lat)
    return nearest


def compute_osmnx_plan_time(G) -> dict:
    """
    Calcola il tempo di percorrenza del piano ENHSP usando i dati reali OSM.

    Per ogni segmento del piano (es. loc000→loc021):
    - Se l'arco esiste nel grafo: usa il tempo diretto
    - Altrimenti: trova il sub-percorso più breve tra i due nodi

    Questo confronto è "onesto" rispetto al modello PDDL+: entrambi usano
    la stessa rete e gli stessi limiti di velocità OSM, ma PDDL+ approssima
    la velocità come costante per ogni arco. OSMnx usa le distanze reali.
    """
    total_time   = 0.0
    total_dist   = 0.0
    segment_data = []

    for i in range(len(PLAN_LOCS) - 1):
        loc_from, osm_from, lat_from, lon_from = PLAN_LOCS[i]
        loc_to,   osm_to,   lat_to,   lon_to   = PLAN_LOCS[i + 1]

        u = find_graph_node(G, osm_from, lat_from, lon_from)
        v = find_graph_node(G, osm_to,   lat_to,   lon_to)

        seg_time = 0.0
        seg_dist = 0.0

        if G.has_edge(u, v):
            # Prendi l'arco con tempo minore (grafo multi-arco)
            best = min(G[u][v].values(),
                       key=lambda e: e.get("travel_time", float("inf")))
            seg_time = best.get("travel_time", 0)
            seg_dist = best.get("length", 0)
        else:
            # Sub-percorso tra i nodi del piano
            try:
                path = nx.shortest_path(G, u, v, weight="travel_time")
                for pu, pv in zip(path[:-1], path[1:]):
                    best = min(G[pu][pv].values(),
                               key=lambda e: e.get("travel_time", float("inf")))
                    seg_time += best.get("travel_time", 0)
                    seg_dist += best.get("length", 0)
            except nx.NetworkXNoPath:
                print(f"    ⚠️  Nessun percorso OSMnx tra {loc_from} e {loc_to}")

        total_time += seg_time
        total_dist += seg_dist
        segment_data.append({
            "from": loc_from, "to": loc_to,
            "time_s": round(seg_time, 1),
            "dist_m": round(seg_dist, 1),
        })

    return {
        "total_time_s": round(total_time, 1),
        "total_dist_m": round(total_dist, 1),
        "segments": segment_data,
    }


def compute_osmnx_optimal_time(G) -> dict:
    """
    Trova il percorso ottimale tra START e GOAL sull'intera rete OSMnx
    (non vincolato alle 8 tappe PDDL+) e ne calcola il tempo.

    Questo rappresenta il "benchmark": il percorso più veloce possibile
    secondo OSMnx, che usa Dijkstra sulla rete completa di Cosenza.
    Confronta la qualità del piano PDDL+ rispetto all'ottimo OSMnx.
    """
    start_node = find_graph_node(G, PLAN_LOCS[0][1], PLAN_LOCS[0][2], PLAN_LOCS[0][3])
    goal_node  = find_graph_node(G, PLAN_LOCS[-1][1], PLAN_LOCS[-1][2], PLAN_LOCS[-1][3])

    try:
        opt_time = nx.shortest_path_length(G, start_node, goal_node, weight="travel_time")
        opt_path = nx.shortest_path(G, start_node, goal_node, weight="travel_time")
        opt_dist = sum(
            min(G[u][v].values(), key=lambda e: e.get("length", 0)).get("length", 0)
            for u, v in zip(opt_path[:-1], opt_path[1:])
        )
        return {
            "time_s": round(opt_time, 1),
            "dist_m": round(opt_dist, 1),
            "n_nodes": len(opt_path),
        }
    except nx.NetworkXNoPath:
        return None


# ═══════════════════════════════════════════════════════════════════════════════
# PARTE 2 — Simulazione SUMO (opzionale)
# ═══════════════════════════════════════════════════════════════════════════════

def find_sumo():
    """Cerca SUMO tra SUMO_HOME, percorsi Windows comuni, PATH di sistema."""
    sumo_home = os.environ.get("SUMO_HOME")
    if sumo_home and Path(sumo_home).exists():
        return Path(sumo_home)
    for c in [r"C:\Program Files (x86)\Eclipse\Sumo",
              r"C:\Program Files\Eclipse\Sumo",
              r"C:\Sumo"]:
        p = Path(c)
        if (p / "bin" / "netconvert.exe").exists():
            return p
    if shutil.which("netconvert"):
        return None
    return False   # non trovato


def sumo_cmd(tool, sumo_home):
    if sumo_home is None:
        return tool
    ext = ".exe" if sys.platform == "win32" else ""
    return str(sumo_home / "bin" / (tool + ext))


def download_osm(bbox, out_file: Path):
    if out_file.exists() and out_file.stat().st_size > 10_000:
        print(f"  ✓ OSM già presente ({out_file.stat().st_size // 1024} KB)")
        return
    south, west, north, east = bbox
    query = f"""
[out:xml][timeout:180];
(way["highway"]({south},{west},{north},{east}); node(w););
out body; >; out skel qt;
""".strip()
    print("  → Download OSM da Overpass API...")
    resp = requests.post("https://overpass-api.de/api/interpreter",
                         data={"data": query}, timeout=180)
    resp.raise_for_status()
    out_file.write_bytes(resp.content)
    print(f"  ✓ Salvato ({out_file.stat().st_size // 1024} KB)")


def run_netconvert(osm_file, net_file, sumo_home):
    if net_file.exists():
        net_file.unlink()
    cmd = [sumo_cmd("netconvert", sumo_home),
           "--osm-files", str(osm_file),
           "--output-file", str(net_file),
           "--geometry.remove", "true",
           "--roundabouts.guess", "true",
           "--junctions.join", "true",
           "--proj.utm", "true",
           "--no-warnings", "true"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print("  ❌ netconvert fallito:", r.stderr[-500:])
        return False
    print(f"  ✓ Rete SUMO creata: {net_file.name}")
    return True


def find_nearest_edges(net, lon, lat, n=10):
    x, y = net.convertLonLat2XY(lon, lat)
    for r in [300, 1000, 5000]:
        edges = net.getNeighboringEdges(x, y, r=r, includeJunctions=False)
        if len(edges) >= 2:
            break
    return [(e[0].getID(), round(e[1], 1))
            for e in sorted(edges, key=lambda e: e[1])[:n]]


def run_sumo_simulation(sumo_home, net_file, osm_file) -> dict | None:
    """
    Tenta la simulazione SUMO. Usa --repair in duarouter per gestire
    eventuali disconnessioni nella rete (aggiunge teleport dove necessario).
    Ritorna il dizionario con i risultati, o None se fallisce.
    """
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # Carica sumolib
    if sumo_home:
        tools = str(sumo_home / "tools")
        if tools not in sys.path:
            sys.path.insert(0, tools)
    try:
        import sumolib
        import warnings
        with warnings.catch_warnings():
            warnings.simplefilter("ignore")
            net = sumolib.net.readNet(str(net_file))
    except Exception as e:
        print(f"  ⚠️  sumolib non disponibile: {e}")
        return None

    # Trova archi più vicini
    start_cands = find_nearest_edges(net, START_LON, START_LAT)
    goal_cands  = find_nearest_edges(net, GOAL_LON,  GOAL_LAT)
    if not start_cands or not goal_cands:
        print("  ⚠️  Nessun arco trovato vicino a START/GOAL")
        return None

    start_edge = start_cands[0][0]
    goal_edge  = goal_cands[0][0]
    print(f"  → START edge: {start_edge}")
    print(f"  → GOAL  edge: {goal_edge}")

    trip_file     = OUTPUT_DIR / "trip.xml"
    vtype_file    = OUTPUT_DIR / "vtype.xml"
    route_file    = OUTPUT_DIR / "route.rou.xml"
    tripinfo_file = OUTPUT_DIR / "tripinfo.xml"
    cfg_file      = OUTPUT_DIR / "cosenza.sumocfg"

    # vType
    vtype_file.write_text("""<?xml version="1.0" encoding="UTF-8"?>
<additional>
    <vType id="auto" vClass="passenger" maxSpeed="50"
           accel="2.6" decel="4.5" sigma="0.0" length="4.5"/>
</additional>""", encoding="utf-8")

    # Trip
    trip_file.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<trips>
    <trip id="piano_enhsp" type="auto" depart="0"
          from="{start_edge}" to="{goal_edge}"/>
</trips>""", encoding="utf-8")

    # duarouter con --repair (gestisce rete disconnessa)
    route_file.unlink(missing_ok=True)
    cmd = [sumo_cmd("duarouter", sumo_home),
           "--net-file", str(net_file),
           "--route-files", str(trip_file),
           "--additional-files", str(vtype_file),
           "--output-file", str(route_file),
           "--repair", "true",
           "--ignore-errors", "true",
           "--no-warnings", "true"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    if not route_file.exists():
        print("  ⚠️  duarouter non ha prodotto output")
        return None
    content = route_file.read_text(encoding="utf-8", errors="ignore")
    has_teleport = "teleport" in content.lower()
    if "<vehicle" not in content and "<route " not in content:
        print("  ⚠️  duarouter: nessun percorso trovato nemmeno con --repair")
        return None

    # Config SUMO
    tripinfo_file.unlink(missing_ok=True)
    cfg_file.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="{net_file.name}"/>
        <route-files value="{route_file.name}"/>
        <additional-files value="{vtype_file.name}"/>
    </input>
    <output><tripinfo-output value="{tripinfo_file.name}"/></output>
    <time><begin value="0"/><end value="7200"/><step-length value="0.1"/></time>
    <report><no-warnings value="true"/><no-step-log value="true"/></report>
</configuration>""", encoding="utf-8")

    # Simulazione
    r = subprocess.run([sumo_cmd("sumo", sumo_home), "-c", cfg_file.name],
                       capture_output=True, text=True, cwd=str(cfg_file.parent))
    if r.returncode != 0:
        print("  ⚠️  SUMO ha fallito:", r.stderr[:300])
        return None

    # Leggi risultati
    if not tripinfo_file.exists():
        return None
    tree = ET.parse(tripinfo_file)
    for ti in tree.getroot().findall("tripinfo"):
        return {
            "duration":     float(ti.get("duration", 0)),
            "route_length": float(ti.get("routeLength", 0)),
            "waiting_time": float(ti.get("waitingTime", 0)),
            "has_teleport": has_teleport,
        }
    return None


# ═══════════════════════════════════════════════════════════════════════════════
# MAIN
# ═══════════════════════════════════════════════════════════════════════════════

def main():
    print()
    print("=" * 65)
    print("   VALIDAZIONE PIANO PDDL+")
    print("   Cosenza — Percorso loc000 → loc039")
    print("=" * 65)
    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # ── PARTE 1: OSMnx ─────────────────────────────────────────────────────
    print("\n── Validazione OSMnx (velocità reali da OSM) ──────────────────")
    G = load_osmnx_graph()

    print("\n  → Calcolo tempo percorso ENHSP su rete reale...")
    plan_result = compute_osmnx_plan_time(G)

    print("  → Calcolo percorso ottimale OSMnx (Dijkstra)...")
    opt_result = compute_osmnx_optimal_time(G)

    # ── PARTE 2: SUMO ──────────────────────────────────────────────────────
    print("\n── Tentativo simulazione SUMO ─────────────────────────────────")
    sumo_home = find_sumo()
    sumo_result = None

    if sumo_home is False:
        print("  ℹ️  SUMO non trovato — parte SUMO saltata")
    else:
        label = str(sumo_home) if sumo_home else "PATH di sistema"
        print(f"  ✓ SUMO trovato: {label}")
        osm_file = OUTPUT_DIR / "cosenza.osm"
        net_file = OUTPUT_DIR / "cosenza.net.xml"
        download_osm(BBOX, osm_file)
        if run_netconvert(osm_file, net_file, sumo_home):
            print("  → Simulazione SUMO in corso...")
            sumo_result = run_sumo_simulation(sumo_home, net_file, osm_file)
            if sumo_result:
                print("  ✓ Simulazione SUMO completata!")
            else:
                print("  ⚠️  Simulazione SUMO non riuscita (rete disconnessa)")

    # ── RISULTATI ──────────────────────────────────────────────────────────
    print()
    print("=" * 65)
    print("   RISULTATI")
    print("=" * 65)

    osmnx_time = plan_result["total_time_s"]
    osmnx_dist = plan_result["total_dist_m"] / 1000

    diff_osmnx     = osmnx_time - ENHSP_TIME
    diff_osmnx_pct = abs(diff_osmnx) / ENHSP_TIME * 100

    print(f"""
  Piano PDDL+  (ENHSP sat-hadd):     {ENHSP_TIME:>6} s  ({ENHSP_TIME/60:.1f} min)
  Percorso PDDL+ su rete OSM reale:  {osmnx_time:>6.0f} s  ({osmnx_time/60:.1f} min)
  Differenza:                         {diff_osmnx:>+6.0f} s  ({diff_osmnx_pct:.1f}%)
  Distanza percorso:                  {osmnx_dist:.2f} km
""")

    if opt_result:
        opt_time = opt_result["time_s"]
        diff_opt = ENHSP_TIME - opt_time
        print(f"  Percorso ottimale OSMnx (Dijkstra): {opt_time:>6.0f} s  ({opt_time/60:.1f} min)")
        print(f"  Piano PDDL+ vs ottimale OSMnx:      {diff_opt:>+6.0f} s  ({abs(diff_opt)/opt_time*100:.1f}%)\n")

    if sumo_result:
        sumo_time = sumo_result["duration"]
        diff_sumo = sumo_time - ENHSP_TIME
        note = " (con teleport — rete parzialmente disconnessa)" if sumo_result["has_teleport"] else ""
        print(f"  Simulazione SUMO:                   {sumo_time:>6.0f} s  ({sumo_time/60:.1f} min){note}")
        print(f"  Differenza SUMO vs ENHSP:            {diff_sumo:>+6.0f} s  ({abs(diff_sumo)/ENHSP_TIME*100:.1f}%)\n")

    # Interpretazione
    print("  " + "─" * 60)
    if diff_osmnx_pct < 15:
        print("  ✅ Il modello PDDL+ è accurato: la velocità costante per")
        print("     arco cattura bene il comportamento reale del traffico.")
    elif diff_osmnx_pct < 35:
        print("  ⚠️  Differenza moderata: PDDL+ usa velocità media costante")
        print("     per ogni arco, OSMnx usa le distanze reali e i maxspeed.")
        print("     La differenza è attesa in planning formale.")
    else:
        print("  ℹ️  Differenza significativa: il piano PDDL+ ottimizza sul")
        print("     modello astratto, non sulla fisica stradale reale.")

    # Report JSON
    report = {
        "descrizione": "Confronto ENHSP vs simulazione fisica su rete reale",
        "percorso": "loc000 (Cosenza nord) → loc039 (Cosenza sud)",
        "enhsp_s":  ENHSP_TIME,
        "osmnx_piano_s":     osmnx_time,
        "osmnx_piano_km":    round(osmnx_dist, 2),
        "diff_osmnx_pct":    round(diff_osmnx_pct, 1),
        "osmnx_ottimale_s":  opt_result["time_s"] if opt_result else None,
        "sumo_s":            sumo_result["duration"] if sumo_result else None,
        "sumo_teleport":     sumo_result["has_teleport"] if sumo_result else None,
        "segmenti":          plan_result["segments"],
    }
    out = Path("output/sumo_comparison.json")
    out.write_text(json.dumps(report, indent=2, ensure_ascii=False), encoding="utf-8")
    print(f"\n  → Report salvato: {out}")
    print()
    print("=" * 65)


if __name__ == "__main__":
    main()

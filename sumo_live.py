#!/usr/bin/env python3
"""
sumo_live.py — Apre SUMO-GUI con il percorso ENHSP animato in tempo reale
==========================================================================

Strategia per garantire la connettività della rete:
  1. Scarica la rete di Cosenza con osmnx (la stessa usata per PDDL+)
  2. La esporta in formato OSM XML preservando la struttura diretta
  3. netconvert la converte in rete SUMO (senza --junctions.join!)
  4. duarouter calcola il percorso START→GOAL (edge-based, poi junction-based)
  5. sumo-gui mostra il veicolo che percorre il piano in tempo reale

Uso:
    python sumo_live.py
"""

import os, sys, subprocess, shutil, warnings
from pathlib import Path

try:
    import osmnx as ox
    import networkx as nx
except ImportError:
    print("❌ Installa osmnx: pip install osmnx")
    sys.exit(1)

# ── Configurazione ─────────────────────────────────────────────────────────────

START_LON, START_LAT = 16.3493669, 39.6418332   # loc000 — Cosenza nord
GOAL_LON,  GOAL_LAT  = 16.3490861, 39.568883    # loc039 — Cosenza sud
START_OSM_ID = 411608790                          # nodo OSM verificato in osmnx

OUTPUT_DIR = Path("output/sumo_live")

# ── Trova SUMO ─────────────────────────────────────────────────────────────────

def find_sumo():
    home = os.environ.get("SUMO_HOME")
    if home and Path(home).exists():
        return Path(home)
    for c in [r"C:\Program Files (x86)\Eclipse\Sumo",
              r"C:\Program Files\Eclipse\Sumo", r"C:\Sumo"]:
        p = Path(c)
        if (p / "bin" / "netconvert.exe").exists():
            return p
    if shutil.which("netconvert"):
        return None
    print("❌ SUMO non trovato. Imposta SUMO_HOME.")
    sys.exit(1)

def sumo_cmd(tool, sumo_home):
    if sumo_home is None:
        return tool
    ext = ".exe" if sys.platform == "win32" else ""
    return str(sumo_home / "bin" / (tool + ext))

# ── Step 1: Scarica grafo osmnx ────────────────────────────────────────────────

def get_graph():
    print("  → Download rete Cosenza con osmnx...")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        G = ox.graph_from_place("Cosenza, Italy", network_type="drive", simplify=True)
    G = ox.add_edge_speeds(G)
    G = ox.add_edge_travel_times(G)
    print(f"  ✓ Grafo: {len(G.nodes)} nodi, {len(G.edges)} archi")

    # START
    if G.has_node(START_OSM_ID):
        start_node = START_OSM_ID
        print(f"  ✓ START: nodo OSM {start_node} trovato nel grafo")
    else:
        start_node = ox.nearest_nodes(G, START_LON, START_LAT)
        print(f"  ✓ START: nodo più vicino = {start_node}")

    # GOAL
    goal_node = ox.nearest_nodes(G, GOAL_LON, GOAL_LAT)
    print(f"  ✓ GOAL:  nodo più vicino = {goal_node}")

    # Verifica percorso
    if not nx.has_path(G, start_node, goal_node):
        print("  ❌ Nessun percorso osmnx tra START e GOAL")
        sys.exit(1)

    opt_time = nx.shortest_path_length(G, start_node, goal_node, weight="travel_time")
    print(f"  ✓ Percorso osmnx: {opt_time:.0f} s ({opt_time/60:.1f} min)")

    return G, start_node, goal_node

# ── Step 2: Esporta grafo osmnx → OSM XML ─────────────────────────────────────

def export_osm_xml(G, filepath: Path):
    """
    Converte il grafo osmnx in formato OSM XML per netconvert.
    Ogni coppia (u,v): se solo u→v → oneway=yes; se anche v→u → bidirezionale.
    """
    print(f"  → Scrittura OSM XML ({len(G.nodes)} nodi, {len(G.edges)} archi)...")
    lines = [
        '<?xml version="1.0" encoding="UTF-8"?>',
        '<osm version="0.6" generator="osmnx-pddl-sumo">',
    ]

    for node_id, data in G.nodes(data=True):
        lat = data.get("y", 0)
        lon = data.get("x", 0)
        lines.append(f'  <node id="{node_id}" lat="{lat}" lon="{lon}" version="1"/>')

    processed = set()
    way_id = 2_000_000_000

    for u, v, data in G.edges(data=True):
        pair = frozenset([u, v])
        if pair in processed:
            continue
        processed.add(pair)

        is_bidir = G.has_edge(v, u)

        hw = data.get("highway", "residential")
        if isinstance(hw, list): hw = hw[0]

        speed = data.get("maxspeed", "")
        if isinstance(speed, list): speed = speed[0]

        name = data.get("name", "")
        if isinstance(name, list): name = name[0]
        name = str(name).replace('"', "'").replace("&", "and") if name else ""

        lines.append(f'  <way id="{way_id}" version="1">')
        lines.append(f'    <nd ref="{u}"/>')
        lines.append(f'    <nd ref="{v}"/>')
        lines.append(f'    <tag k="highway" v="{hw}"/>')
        if not is_bidir:
            lines.append(f'    <tag k="oneway" v="yes"/>')
        if speed:
            lines.append(f'    <tag k="maxspeed" v="{speed}"/>')
        if name:
            lines.append(f'    <tag k="name" v="{name}"/>')
        lines.append(f'  </way>')
        way_id += 1

    lines.append("</osm>")
    filepath.write_text("\n".join(lines), encoding="utf-8")
    print(f"  ✓ Salvato: {filepath.name} ({filepath.stat().st_size // 1024} KB)")

# ── Step 3: netconvert OSM → rete SUMO ────────────────────────────────────────

def run_netconvert(osm_file, net_file, sumo_home):
    net_file.unlink(missing_ok=True)
    cmd = [
        sumo_cmd("netconvert", sumo_home),
        "--osm-files",   str(osm_file),
        "--output-file", str(net_file),
        "--proj.utm",    "true",
        "--no-warnings", "true",
        # NON usare --junctions.join né --geometry.remove:
        # rinominerebbero i junction ID da OSM a "cluster_..." → duarouter fallisce
    ]
    print("  → Esecuzione netconvert (ID OSM preservati)...")
    r = subprocess.run(cmd, capture_output=True, text=True)
    if r.returncode != 0:
        print("❌ netconvert:", r.stderr[-800:])
        sys.exit(1)
    print(f"  ✓ Rete SUMO: {net_file.name}")

# ── Step 4: Trova archi e junction START/GOAL nella rete SUMO ─────────────────

def find_network_endpoints(net, start_lon, start_lat, goal_lon, goal_lat,
                            start_osm_id, goal_osm_id):
    """
    Restituisce (start_edge, goal_edge, start_junc, goal_junc).
    Usa sumolib per trovare archi e junction più vicini alle coordinate GPS.
    """

    def nearest_edge(lon, lat, label):
        x, y = net.convertLonLat2XY(lon, lat)
        for radius in [50, 150, 500, 2000, 8000]:
            edges = net.getNeighboringEdges(x, y, r=radius, includeJunctions=False)
            if edges:
                edge, dist = sorted(edges, key=lambda e: e[1])[0]
                print(f"  ✓ {label} arco  (dist={dist:.0f}m): {edge.getID()}")
                return edge.getID()
        print(f"  ❌ {label}: nessun arco trovato!")
        return None

    def nearest_junction(osm_id, lon, lat, label):
        # Strategia 1: ID OSM diretto
        try:
            node = net.getNode(str(osm_id))
            if node is not None:
                print(f"  ✓ {label} junc  (OSM diretto): {node.getID()}")
                return node.getID()
        except Exception:
            pass
        # Strategia 2: nodo più vicino dagli archi circostanti
        x, y = net.convertLonLat2XY(lon, lat)
        for radius in [100, 500, 2000, 8000]:
            edges = net.getNeighboringEdges(x, y, r=radius, includeJunctions=False)
            if not edges:
                continue
            best_id, best_d = None, float("inf")
            for edge, _ in sorted(edges, key=lambda e: e[1])[:5]:
                for node in [edge.getFromNode(), edge.getToNode()]:
                    nx_, ny_ = node.getCoord()
                    d = ((nx_ - x)**2 + (ny_ - y)**2)**0.5
                    if d < best_d:
                        best_d, best_id = d, node.getID()
            if best_id:
                print(f"  ✓ {label} junc  (nearest, {best_d:.0f}m): {best_id}")
                return best_id
        print(f"  ⚠️  {label}: junction non trovata")
        return None

    start_edge = nearest_edge(start_lon, start_lat, "START")
    goal_edge  = nearest_edge(goal_lon,  goal_lat,  "GOAL ")
    start_junc = nearest_junction(start_osm_id, start_lon, start_lat, "START")
    goal_junc  = nearest_junction(goal_osm_id,  goal_lon,  goal_lat,  "GOAL ")

    return start_edge, goal_edge, start_junc, goal_junc

# ── Step 5a: duarouter edge-based ─────────────────────────────────────────────

def run_duarouter_edges(net_file, trip_file, vtype_file, route_file,
                        sumo_home, start_edge, goal_edge, repair=False):
    """Routing da arco a arco — funziona con tutte le versioni SUMO."""
    trip_file.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<trips>
    <trip id="enhsp_route" type="auto" depart="0"
          from="{start_edge}"
          to="{goal_edge}"/>
</trips>
""", encoding="utf-8")

    _write_vtype(vtype_file)

    route_file.unlink(missing_ok=True)
    cmd = [
        sumo_cmd("duarouter", sumo_home),
        "--net-file",         str(net_file),
        "--route-files",      str(trip_file),
        "--additional-files", str(vtype_file),
        "--output-file",      str(route_file),
        "--no-warnings",      "true",
    ]
    if repair:
        cmd += ["--repair", "true", "--ignore-errors", "true"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    _print_errors(r.stderr)
    return _route_ok(route_file)

# ── Step 5b: duarouter junction-based ─────────────────────────────────────────

def run_duarouter_junctions(net_file, trip_file, vtype_file, route_file,
                             sumo_home, start_junc, goal_junc, repair=False):
    """Routing da junction a junction (richiede SUMO ≥ 1.11)."""
    trip_file.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<trips>
    <trip id="enhsp_route" type="auto" depart="0"
          fromJunction="{start_junc}"
          toJunction="{goal_junc}"/>
</trips>
""", encoding="utf-8")

    _write_vtype(vtype_file)

    route_file.unlink(missing_ok=True)
    cmd = [
        sumo_cmd("duarouter", sumo_home),
        "--net-file",         str(net_file),
        "--route-files",      str(trip_file),
        "--additional-files", str(vtype_file),
        "--output-file",      str(route_file),
        "--no-warnings",      "true",
    ]
    if repair:
        cmd += ["--repair", "true", "--ignore-errors", "true"]
    r = subprocess.run(cmd, capture_output=True, text=True)
    _print_errors(r.stderr)
    return _route_ok(route_file)

def _write_vtype(vtype_file):
    vtype_file.write_text("""<?xml version="1.0" encoding="UTF-8"?>
<additional>
    <vType id="auto" vClass="passenger"
           maxSpeed="50" accel="2.0" decel="4.5"
           sigma="0.2" length="4.5"
           color="255,80,80"/>
</additional>
""", encoding="utf-8")

def _print_errors(stderr):
    if not stderr:
        return
    for line in stderr.splitlines():
        if any(k in line for k in ["Error", "No path", "not found", "Cannot", "Warning"]):
            print("  ℹ️ ", line)

def _route_ok(route_file):
    if not route_file.exists():
        return False
    content = route_file.read_text(encoding="utf-8", errors="ignore")
    return "<vehicle" in content or "<route " in content

# ── Step 6: GUI settings e config SUMO ───────────────────────────────────────

def create_gui_settings(settings_file):
    settings_file.write_text("""<viewsettings>
    <scheme name="pddl-live">
        <background backgroundColor="40,40,55" showGrid="0"/>
        <edges laneShowBorders="1" showLinkDecals="1">
            <colorScheme name="uniform">
                <entry color="180,180,200"/>
            </colorScheme>
        </edges>
        <vehicles vehicleQuality="2" showBlinker="1"
                  vehicleSize.minSize="8"
                  vehicleSize.exaggeration="8">
            <colorScheme name="given/assigned vehicle color">
                <entry color="255,60,60"/>
            </colorScheme>
        </vehicles>
        <junctions drawShape="1">
            <colorScheme name="uniform">
                <entry color="120,120,150"/>
            </colorScheme>
        </junctions>
    </scheme>
    <delay value="50"/>
</viewsettings>
""", encoding="utf-8")


def create_sumo_config(net_file, route_file, vtype_file, cfg_file, settings_file):
    # NOTA: vtype_file NON va in additional-files perché duarouter lo copia già
    # dentro route_file → avere entrambi causa "duplicate vType id" in sumo-gui
    cfg_file.write_text(f"""<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="{net_file.name}"/>
        <route-files value="{route_file.name}"/>
        <gui-settings-file value="{settings_file.name}"/>
    </input>
    <time>
        <begin value="0"/>
        <end value="7200"/>
        <step-length value="0.5"/>
    </time>
    <processing>
        <time-to-teleport value="-1"/>
    </processing>
    <report>
        <no-warnings value="true"/>
        <no-step-log value="true"/>
    </report>
</configuration>
""", encoding="utf-8")

# ── Main ──────────────────────────────────────────────────────────────────────

def main():
    print()
    print("=" * 60)
    print("  SUMO-GUI LIVE — Piano ENHSP — Cosenza")
    print("=" * 60)

    OUTPUT_DIR.mkdir(parents=True, exist_ok=True)

    # [0] SUMO
    print("\n[0/6] Ricerca SUMO...")
    sumo_home = find_sumo()
    print(f"  ✓ SUMO: {sumo_home or 'nel PATH di sistema'}")

    if sumo_home:
        tools = str(sumo_home / "tools")
        if tools not in sys.path:
            sys.path.insert(0, tools)
    try:
        import sumolib
    except ImportError:
        print("❌ sumolib non trovato — controlla SUMO_HOME/tools")
        sys.exit(1)

    # [1] OSMnx
    print("\n[1/6] Download rete osmnx...")
    G, start_node, goal_node = get_graph()

    # [2] OSM XML (usa cache se già presente e valida)
    osm_file = OUTPUT_DIR / "cosenza_osmnx.osm"
    print("\n[2/6] Esportazione OSM XML...")
    if osm_file.exists() and osm_file.stat().st_size > 100_000:
        print(f"  ✓ Già presente: {osm_file.name} ({osm_file.stat().st_size // 1024} KB)")
    else:
        export_osm_xml(G, osm_file)

    # [3] netconvert (sempre rigenerato per applicare le fix)
    net_file = OUTPUT_DIR / "cosenza_live.net.xml"
    print("\n[3/6] Conversione in rete SUMO (netconvert)...")
    run_netconvert(osm_file, net_file, sumo_home)

    # [4] Trova archi e junction
    print("\n[4/6] Ricerca endpoint START/GOAL nella rete SUMO...")
    with warnings.catch_warnings():
        warnings.simplefilter("ignore")
        net = sumolib.net.readNet(str(net_file))

    start_edge, goal_edge, start_junc, goal_junc = find_network_endpoints(
        net, START_LON, START_LAT, GOAL_LON, GOAL_LAT,
        start_node, goal_node
    )

    # [5] duarouter — prova in cascata
    trip_file     = OUTPUT_DIR / "trip_live.xml"
    vtype_file    = OUTPUT_DIR / "vtype_live.xml"
    route_file    = OUTPUT_DIR / "route_live.rou.xml"
    cfg_file      = OUTPUT_DIR / "cosenza_live.sumocfg"
    settings_file = OUTPUT_DIR / "gui_settings.xml"

    print("\n[5/6] Calcolo percorso (duarouter)...")
    ok = False

    # Tentativo 1: edge-based (più compatibile)
    if start_edge and goal_edge:
        print("  → Tentativo 1: routing per archi...")
        ok = run_duarouter_edges(
            net_file, trip_file, vtype_file, route_file,
            sumo_home, start_edge, goal_edge
        )
        if ok:
            print("  ✓ Percorso trovato (edge-based)!")

    # Tentativo 2: junction-based
    if not ok and start_junc and goal_junc:
        print("  → Tentativo 2: routing per junction...")
        ok = run_duarouter_junctions(
            net_file, trip_file, vtype_file, route_file,
            sumo_home, start_junc, goal_junc
        )
        if ok:
            print("  ✓ Percorso trovato (junction-based)!")

    # Tentativo 3: edge-based con --repair
    if not ok and start_edge and goal_edge:
        print("  → Tentativo 3: edge-based con --repair...")
        ok = run_duarouter_edges(
            net_file, trip_file, vtype_file, route_file,
            sumo_home, start_edge, goal_edge, repair=True
        )
        if ok:
            print("  ✓ Percorso trovato (edge-based + repair)!")

    # Tentativo 4: junction-based con --repair
    if not ok and start_junc and goal_junc:
        print("  → Tentativo 4: junction-based con --repair...")
        ok = run_duarouter_junctions(
            net_file, trip_file, vtype_file, route_file,
            sumo_home, start_junc, goal_junc, repair=True
        )
        if ok:
            print("  ✓ Percorso trovato (junction-based + repair)!")

    if not ok:
        print()
        print("  ❌ Nessun percorso trovato dopo tutti i tentativi.")
        print("     Controlla che SUMO possa vedere la rete:")
        print(f"     Net file: {net_file.resolve()}")
        sys.exit(1)

    # [6] Apri sumo-gui
    print("\n[6/6] Apertura SUMO-GUI...")
    create_gui_settings(settings_file)
    create_sumo_config(net_file, route_file, vtype_file, cfg_file, settings_file)

    print()
    print("  " + "─" * 50)
    print("  🚗  SUMO-GUI in apertura...")
    print()
    print("  Come usare sumo-gui:")
    print("  → Premi il tasto  ▶  (Play) per avviare")
    print("  → Usa lo slider in alto per cambiare velocità")
    print("  → Clicca sul veicolo rosso per seguirlo")
    print("  → Ctrl+A per centrare la vista")
    print("  " + "─" * 50)
    print()

    subprocess.Popen(
        [sumo_cmd("sumo-gui", sumo_home), "-c", cfg_file.name],
        cwd=str(cfg_file.parent.resolve())
    )


if __name__ == "__main__":
    main()

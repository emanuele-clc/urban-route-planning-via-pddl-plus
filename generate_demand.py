"""
generate_demand.py
===================
Genera un campione di coppie origine-destinazione (O-D) CONDIVISO tra:
  - punto 2 (signal_optimization/: fitness di ottimizzazione semaforica), e
  - punto 4 (confronto in SUMO dei piani ottimizzati vs baseline).

Risolve la Criticita' #3 di 2_traffic_signal_optimization.md: prima i
campioni O-D usati per valutare i piani PDDL+ erano generati ad-hoc
(build_problems.compute_vehicle_counts, N_VEHICLES=10, uniformi sui nodi
del sottografo PDDL) e disallineati da qualunque domanda usata in SUMO.

Metodo primario: SUMO randomTrips.py sull'INTERA rete della zona
(net_files/<zona>.net.xml) — stessa domanda realistica che verrebbe usata
per una simulazione SUMO di confronto. randomTrips pesa gli edge per
default in base a lunghezza/velocita'/n. corsie, che correla con la
gerarchia stradale (coerente con CONGESTION_DELAY_BY_HIGHWAY). Gli
endpoint (giunzioni SUMO) vengono poi agganciati al nodo PIU' VICINO
(Haversine) nel sottografo PDDL 'selected' — necessario perche' net.xml
copre l'intera zona mentre il sottografo PDDL e' un estratto piu' piccolo
(vedi build_problems.select_connected_subgraph, max_nodes per zona).

Fallback (SUMO/randomTrips.py non trovato): campionamento uniforme di
coppie O-D dentro 'selected' via lo stesso Dijkstra gia' usato da
build_problems.compute_vehicle_counts — meno realistico ma comunque
condiviso/riproducibile (stesso seed) tra punto 2 e punto 4.

Uso:
    python generate_demand.py piccola media grande
Output:
    sumo_extracted/demand_<zona>.json
"""

import os
import sys
import json
import glob
import heapq
import random
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict

BASE = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, BASE)

import build_problems as bp  # noqa: E402  (riuso: build_contracted_graph, select_connected_subgraph, haversine)
import extract_sumo_data as esd  # noqa: E402  (riuso: load_net, cluster_member_ids helper duplicato sotto)

OSM_DIR = os.path.join(BASE, "osm_files")
NET_DIR = os.path.join(BASE, "net_files")
OUT_DIR = os.path.join(BASE, "sumo_extracted")

# Stesse (zona, file_osm, max_nodes) di build_problems.py — l'insieme O-D
# deve riferirsi allo stesso sottografo PDDL usato per generare i problemi.
ZONE_CONFIGS = {
    "piccola": ("dublin_piccola_centro.osm", 14),
    "media":   ("dublin_media_residenziale.osm", 50),
    "grande":  ("dublin_grande_porto.osm", 120),
}

N_TSC_SAMPLES = 60     # numerosita' campione O-D (separato da N_VEHICLES del
                       # modello di congestione esistente, vedi criticita' #3)
RANDOM_SEED = 42       # coerente col resto del progetto


# ---------------------------------------------------------------------------
# Discovery randomTrips.py (stesso pattern di trova_enhsp in pddl_files/run.py)
# ---------------------------------------------------------------------------
def find_randomtrips():
    env_home = os.environ.get("SUMO_HOME")
    candidates = []
    if env_home:
        candidates.append(os.path.join(env_home, "tools", "randomTrips.py"))
    candidates += glob.glob("/opt/homebrew/Cellar/sumo/*/share/sumo/tools/randomTrips.py")
    candidates += glob.glob("/usr/local/Cellar/sumo/*/share/sumo/tools/randomTrips.py")
    candidates += glob.glob("/Library/Frameworks/EclipseSUMO.framework/Versions/*/EclipseSUMO/share/sumo/tools/randomTrips.py")
    candidates += glob.glob("/usr/share/sumo/tools/randomTrips.py")
    candidates += glob.glob("/usr/local/share/sumo/tools/randomTrips.py")
    for c in candidates:
        if os.path.exists(c):
            return c
    return None


# ---------------------------------------------------------------------------
# Metodo primario: SUMO randomTrips.py + snap al sottografo PDDL
# ---------------------------------------------------------------------------
def _edge_endpoints(net_path):
    """{edge_id: (from_junction, to_junction)} per gli edge reali (non interni)."""
    root = ET.parse(net_path).getroot()
    out = {}
    for e in root.findall("edge"):
        eid = e.get("id")
        if eid is None or eid.startswith(":"):
            continue
        out[eid] = (e.get("from"), e.get("to"))
    return out


def _representative_osm_id(junction_id, node_data):
    """Un id-nodo OSM concreto (con lat/lon in node_data) per una giunzione
    SUMO, espandendo i cluster (extract_sumo_data.cluster_member_ids)."""
    for member in esd.cluster_member_ids(junction_id):
        if member in node_data:
            return member
    return None


def _snap_to_subgraph(osm_id, selected, node_data):
    """Nodo di 'selected' piu' vicino (Haversine) a osm_id. None se osm_id
    non ha coordinate note."""
    if osm_id not in node_data:
        return None
    lat, lon = node_data[osm_id]["lat"], node_data[osm_id]["lon"]
    return min(selected, key=lambda n: bp.haversine(
        lat, lon, node_data[n]["lat"], node_data[n]["lon"]))


def generate_via_sumo(zone, osm_path, net_path, selected, node_data, edges,
                       n_samples=N_TSC_SAMPLES, seed=RANDOM_SEED):
    randomtrips = find_randomtrips()
    if not randomtrips:
        return None

    adj_sub = defaultdict(dict)
    for (a, b), (d, _spd) in edges.items():
        adj_sub[a][b] = d

    edge_endpoints = _edge_endpoints(net_path)
    tmp_dir = tempfile.mkdtemp(prefix=f"demand_{zone}_")
    trips_path = os.path.join(tmp_dir, "trips.trips.xml")

    # sovra-campiona: molte O-D SUMO cadranno fuori dal sottografo PDDL una
    # volta agganciate al nodo piu' vicino, e potrebbero collassare su coppie
    # duplicate o start==goal — generiamo un pool ampio e poi filtriamo/tronchiamo.
    n_pool = max(n_samples * 8, 200)
    cmd = [
        sys.executable, randomtrips,
        "-n", net_path,
        "-o", trips_path,
        "-e", str(n_pool),
        "--seed", str(seed),
        "--fringe-factor", "5",
    ]
    # NB: niente --validate: invocherebbe duarouter per instradare le trip,
    # non necessario qui (servono solo gli endpoint edge) ed e' un ulteriore
    # eseguibile esterno che potrebbe non essere disponibile/funzionante.
    try:
        subprocess.run(cmd, capture_output=True, text=True, timeout=120, check=True)
    except Exception:
        return None

    if not os.path.exists(trips_path):
        return None

    try:
        root = ET.parse(trips_path).getroot()
    except ET.ParseError:
        return None

    pairs = []
    seen = set()
    dist_cache = {}
    for trip in root.findall("trip"):
        from_edge = trip.get("from")
        to_edge = trip.get("to")
        if from_edge not in edge_endpoints or to_edge not in edge_endpoints:
            continue
        from_junc = edge_endpoints[from_edge][0]
        to_junc = edge_endpoints[to_edge][1]
        from_osm = _representative_osm_id(from_junc, node_data)
        to_osm = _representative_osm_id(to_junc, node_data)
        if not from_osm or not to_osm:
            continue
        s = _snap_to_subgraph(from_osm, selected, node_data)
        g = _snap_to_subgraph(to_osm, selected, node_data)
        if not s or not g or s == g:
            continue
        key = (s, g)
        if key in seen:
            continue
        # scarta le coppie non raggiungibili nel sottografo PDDL diretto
        # (necessario: net.xml/randomTrips ignora le restrizioni oneway
        # applicate da build_problems.build_contracted_graph)
        if s not in dist_cache:
            dist_cache[s], _ = bp.dijkstra(s, adj_sub)
        if g not in dist_cache[s]:
            continue
        seen.add(key)
        pairs.append({"start": s, "goal": g})
        if len(pairs) >= n_samples:
            break

    if len(pairs) < max(3, n_samples // 4):
        # troppi pochi campioni validi (subgraph molto piccolo/isolato):
        # non e' un metodo affidabile per questa zona, ricadi sul fallback.
        return None

    return {"method": "sumo_randomtrips", "od_pairs": pairs}


# ---------------------------------------------------------------------------
# Fallback: Dijkstra uniforme dentro 'selected' (stesso schema di
# build_problems.compute_vehicle_counts, ma restituisce le coppie O-D
# invece dei soli conteggi per arco)
# ---------------------------------------------------------------------------
def generate_via_dijkstra(selected, edges, n_samples=N_TSC_SAMPLES, seed=RANDOM_SEED):
    rng = random.Random(seed)
    sel_list = list(selected)
    adj_sub = defaultdict(dict)
    for (a, b), (d, _spd) in edges.items():
        adj_sub[a][b] = d

    pairs = []
    seen = set()
    attempts = 0
    while len(pairs) < n_samples and attempts < n_samples * 20:
        attempts += 1
        s = rng.choice(sel_list)
        candidates = [n for n in sel_list if n != s]
        if not candidates:
            continue
        g = rng.choice(candidates)
        if (s, g) in seen:
            continue
        dist, _prev = bp.dijkstra(s, adj_sub)
        if g not in dist:
            continue
        seen.add((s, g))
        pairs.append({"start": s, "goal": g})

    return {"method": "dijkstra_fallback", "od_pairs": pairs}


def generate(zone, n_samples=N_TSC_SAMPLES, seed=RANDOM_SEED):
    osm_file, max_nodes = ZONE_CONFIGS[zone]
    osm_path = os.path.join(OSM_DIR, osm_file)
    net_path = os.path.join(NET_DIR, f"{zone}.net.xml")

    node_data, adj, _signal_ids, _hw = bp.build_contracted_graph(osm_path)
    selected = bp.select_connected_subgraph(node_data, adj, max_nodes)
    sel_set = set(selected)
    edges = {}
    for a in selected:
        for b, (d, spd) in adj.get(a, {}).items():
            if b in sel_set:
                edges[(a, b)] = (d, spd)

    result = None
    if os.path.exists(net_path):
        result = generate_via_sumo(zone, osm_path, net_path, selected, node_data, edges,
                                    n_samples=n_samples, seed=seed)
    if result is None:
        result = generate_via_dijkstra(selected, edges, n_samples=n_samples, seed=seed)

    out = {
        "zone": zone,
        "n_samples_requested": n_samples,
        "n_samples": len(result["od_pairs"]),
        "seed": seed,
        "method": result["method"],
        "selected_nodes": selected,
        "od_pairs": result["od_pairs"],
    }
    out_path = os.path.join(OUT_DIR, f"demand_{zone}.json")
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        json.dump(out, f, ensure_ascii=False, indent=2)
    return out_path, out


if __name__ == "__main__":
    zones = sys.argv[1:] if len(sys.argv) > 1 else list(ZONE_CONFIGS.keys())
    for zone in zones:
        if zone not in ZONE_CONFIGS:
            print(f"[skip] zona sconosciuta: {zone}")
            continue
        out_path, out = generate(zone)
        print(f"[{zone}] metodo={out['method']} campioni={out['n_samples']}/{out['n_samples_requested']} -> {out_path}")

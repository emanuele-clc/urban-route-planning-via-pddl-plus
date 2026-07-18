import os
import math
import re
import json
import glob
import site
import sysconfig
import subprocess
import tempfile
import uuid
import heapq
import random
import xml.etree.ElementTree as ET
from collections import defaultdict, Counter, deque

from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

PROJECT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
DOMAIN_PATH = os.path.join(PROJECT_ROOT, 'pddl_files', 'domain.pddl')
SUMO_DIR    = os.path.join(PROJECT_ROOT, 'sumo_extracted')

# store in memoria: token → dati grafo
graph_store = {}

# ── Turn rate e ritardi realistici (allineati a build_problems.py) ────────────
TURN_RATE_DPS         = 20.0   # gradi/s: velocita' angolare di svolta veicolo
DEFAULT_SIGNAL_DELAY  = 17.1   # ritardo realistico incrocio 2 fasi, ciclo 120s
                               # (usato quando l'incrocio non e' nei dati SUMO)


def bearing(lat1, lon1, lat2, lon2):
    """Rotta (gradi, 0=Nord) dal punto 1 al punto 2."""
    f1 = math.radians(lat1); f2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(f2)
    x = math.cos(f1) * math.sin(f2) - math.sin(f1) * math.cos(f2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def turn_time_s(prev, mid, nxt, node_data, turn_rate=TURN_RATE_DPS):
    """Tempo di svolta (s) = angolo_di_svolta / turn_rate."""
    b_in  = bearing(node_data[prev]["lat"], node_data[prev]["lon"],
                    node_data[mid]["lat"],  node_data[mid]["lon"])
    b_out = bearing(node_data[mid]["lat"],  node_data[mid]["lon"],
                    node_data[nxt]["lat"],  node_data[nxt]["lon"])
    ang = abs((b_out - b_in + 180) % 360 - 180)
    return round(ang / turn_rate, 2)


def load_all_sumo_delays():
    """Mappa unita {id_nodo_OSM: ritardo_s} da tutti i sumo_data_*.json.
    Cosi' un OSM caricato che ricade in una zona nota usa i ritardi reali."""
    merged = {}
    for z in ("piccola", "media", "grande"):
        p = os.path.join(SUMO_DIR, f"sumo_data_{z}.json")
        if not os.path.exists(p):
            continue
        try:
            with open(p, encoding="utf-8") as f:
                data = json.load(f)
            for k, v in data.get("node_signal_delay", {}).items():
                merged[k] = max(merged.get(k, 0.0), round(float(v), 1))
        except Exception:
            continue
    return merged


SUMO_DELAYS = load_all_sumo_delays()


def signal_delay_for(osm_id, signal_nodes):
    """Ritardo semaforico realistico MEDIO per un nodo (fallback quando un
    movimento specifico non e' mappabile — vedi assign_movement_signal_delay):
    - dai dati SUMO se disponibile,
    - altrimenti default 2-fasi se e' un semaforo OSM,
    - altrimenti 0."""
    if osm_id in SUMO_DELAYS:
        return SUMO_DELAYS[osm_id]
    if osm_id in signal_nodes:
        return DEFAULT_SIGNAL_DELAY
    return 0


def cluster_member_ids(junction_id):
    """Da un id junction SUMO ricava gli id-nodo OSM che rappresenta
    (stessa logica di extract_sumo_data.py::cluster_member_ids)."""
    if not junction_id.startswith("cluster_"):
        return [junction_id]
    body = junction_id[len("cluster_"):]
    ids = []
    for tok in body.split("_"):
        if tok.startswith("#"):
            continue
        if tok.isdigit():
            ids.append(tok)
    return ids


def load_all_sumo_movements():
    """Mappa unita {id_nodo_OSM: [movement, ...]} da tutti i sumo_data_*.json,
    espandendo i cluster SUMO. Ogni movement ha delay_s/bearing_in_bucket/
    bearing_out_bucket/dir_label (vedi extract_sumo_data.py). Usata da
    assign_movement_signal_delay per il ritardo per-movimento (prev,from,to)
    invece della media per nodo."""
    merged = defaultdict(list)
    for z in ("piccola", "media", "grande"):
        p = os.path.join(SUMO_DIR, f"sumo_data_{z}.json")
        if not os.path.exists(p):
            continue
        try:
            with open(p, encoding="utf-8") as f:
                data = json.load(f)
            for tid, tl in data.get("traffic_lights", {}).items():
                members = cluster_member_ids(tid)
                for mv in tl.get("movements", []):
                    if mv.get("bearing_in_bucket") is None or mv.get("bearing_out_bucket") is None:
                        continue
                    for nid in members:
                        merged[nid].append(mv)
        except Exception:
            continue
    return dict(merged)


SUMO_MOVEMENTS = load_all_sumo_movements()


def bearing_bucket(angle_deg, n_buckets=8):
    """Arrotonda un bearing (0=Nord, orario) al settore piu' vicino tra
    n_buckets equidistanti — stessa convenzione di extract_sumo_data.py."""
    if angle_deg is None:
        return None
    step = 360.0 / n_buckets
    a = angle_deg % 360.0
    return int(round(a / step)) % n_buckets * step


def circ_dist(a, b):
    """Distanza angolare minima tra due bearing (0-360)."""
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def assign_movement_signal_delay(a, b, c, node_data, movements_by_node, is_first=False):
    """Ritardo semaforico (s) del movimento specifico (a,b,c) attraversando
    il nodo 'b' — stessa logica di build_problems.py::assign_movement_signal_delay.
    Ritorna None se 'b' non ha dati di movimento SUMO."""
    mv_list = movements_by_node.get(b)
    if not mv_list:
        return None
    out_bearing = bearing(node_data[b]["lat"], node_data[b]["lon"],
                          node_data[c]["lat"], node_data[c]["lon"])
    out_bucket = bearing_bucket(out_bearing)
    if is_first:
        best = min(mv_list, key=lambda m: circ_dist(m["bearing_out_bucket"], out_bucket))
        return best["delay_s"]
    in_bearing = bearing(node_data[a]["lat"], node_data[a]["lon"],
                         node_data[b]["lat"], node_data[b]["lon"])
    in_bucket = bearing_bucket(in_bearing)
    best = min(mv_list, key=lambda m: circ_dist(m["bearing_in_bucket"], in_bucket)
                                      + circ_dist(m["bearing_out_bucket"], out_bucket))
    return best["delay_s"]

HIGHWAY_TYPES = {
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "residential", "motorway_link", "trunk_link", "primary_link",
    "secondary_link", "tertiary_link", "unclassified", "living_street",
}

# ── parametri congestione ─────────────────────────────────────────────────────
N_VEHICLES       = 10
PERIPH_RADIUS_M  = 600
DENSITY_RADIUS_M = 200
RANDOM_SEED      = 42

CONGESTION_DELAY_BY_HIGHWAY = {
    "primary": 20, "primary_link": 15,
    "secondary": 10, "secondary_link": 8,
    "tertiary": 5, "tertiary_link": 3,
    "residential": 5, "living_street": 3,
    "motorway": 0, "motorway_link": 0,
    "trunk": 0, "trunk_link": 0, "unclassified": 0,
}


# ── helpers base ──────────────────────────────────────────────────────────────

def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    f1 = math.radians(lat1); f2 = math.radians(lat2)
    df = math.radians(lat2 - lat1); dl = math.radians(lon2 - lon1)
    a = math.sin(df/2)**2 + math.cos(f1) * math.cos(f2) * math.sin(dl/2)**2
    return round(R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))


def slugify(name):
    if not name:
        return ""
    s = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")[:28]
    return s if s else ""


# ── parsing OSM (ora restituisce anche edge_highway) ──────────────────────────

def build_contracted_graph(osm_path):
    with open(osm_path, encoding="utf-8") as f:
        root = ET.parse(f).getroot()

    node_data = {}
    signal_node_ids = set()
    for n in root.findall("node"):
        nid = n.get("id")
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        node_data[nid] = {
            "lat": float(n.get("lat")),
            "lon": float(n.get("lon")),
            "name": tags.get("name", ""),
        }
        if tags.get("highway") == "traffic_signals":
            signal_node_ids.add(nid)

    membership = Counter()
    good_ways = []
    for w in root.findall("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        hw = tags.get("highway", "")
        if hw not in HIGHWAY_TYPES:
            continue
        nds = [nd.get("ref") for nd in w.findall("nd") if nd.get("ref") in node_data]
        if len(nds) < 2:
            continue
        oneway = tags.get("oneway", "no") == "yes"
        try:
            spd = float(tags.get("maxspeed", "30").split()[0])
        except ValueError:
            spd = 30.0
        good_ways.append((nds, round(spd * 1000 / 3600, 2), oneway, hw))
        for nd in nds:
            membership[nd] += 1

    junctions = set()
    for nds, _, _, _ in good_ways:
        junctions.add(nds[0])
        junctions.add(nds[-1])
    junctions |= {n for n, c in membership.items() if c >= 2}

    adj = defaultdict(dict)
    edge_highway = {}
    for nds, spd, oneway, hw in good_ways:
        seg_start = None; seg_dist = 0
        for i, nid in enumerate(nds):
            if i == 0:
                if nid in junctions:
                    seg_start = nid; seg_dist = 0
                continue
            prev = nds[i - 1]
            if prev in node_data and nid in node_data:
                seg_dist += haversine(node_data[prev]["lat"], node_data[prev]["lon"],
                                      node_data[nid]["lat"],  node_data[nid]["lon"])
            if nid in junctions:
                if seg_start and seg_start != nid and seg_dist > 0:
                    if nid not in adj[seg_start] or adj[seg_start][nid][0] > seg_dist:
                        adj[seg_start][nid] = (seg_dist, spd)
                        edge_highway[(seg_start, nid)] = hw
                    if not oneway:
                        if seg_start not in adj[nid] or adj[nid][seg_start][0] > seg_dist:
                            adj[nid][seg_start] = (seg_dist, spd)
                            edge_highway[(nid, seg_start)] = hw
                seg_start = nid; seg_dist = 0

    return node_data, adj, signal_node_ids, edge_highway


# ── calcolo congestione ───────────────────────────────────────────────────────

def classify_zones(selected, node_data):
    lats = [node_data[n]["lat"] for n in selected]
    lons = [node_data[n]["lon"] for n in selected]
    clat = sum(lats) / len(lats); clon = sum(lons) / len(lons)
    return {n for n in selected
            if haversine(clat, clon, node_data[n]["lat"], node_data[n]["lon"]) > PERIPH_RADIUS_M}


def compute_intersection_density(selected, node_data):
    return {nid: sum(1 for o in selected if o != nid and
                     haversine(node_data[nid]["lat"], node_data[nid]["lon"],
                               node_data[o]["lat"],   node_data[o]["lon"]) <= DENSITY_RADIUS_M)
            for nid in selected}


def compute_congestion_delay(selected, edges, peripheral, density, edge_highway):
    incoming = defaultdict(set)
    for (a, b) in edges:
        incoming[b].add(edge_highway.get((a, b), "unclassified"))
    delays = {}
    for n in selected:
        if n in peripheral:
            delays[n] = 0
        else:
            base  = max((CONGESTION_DELAY_BY_HIGHWAY.get(hw, 0) for hw in incoming[n]), default=0)
            bonus = min(density.get(n, 0), 5) * 2
            delays[n] = base + bonus
    return delays


def dijkstra(start, adj_sub):
    dist = {start: 0}; prev = {start: None}; pq = [(0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, float("inf")): continue
        for v, w in adj_sub.get(u, {}).items():
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd; prev[v] = u; heapq.heappush(pq, (nd, v))
    return dist, prev


def reconstruct_path(prev, goal):
    path = []; cur = goal
    while cur is not None: path.append(cur); cur = prev.get(cur)
    return list(reversed(path))


def compute_vehicle_counts(selected, edges):
    rng = random.Random(RANDOM_SEED)
    sel_list = list(selected)
    counts = Counter()
    adj_sub = defaultdict(dict)
    for (a, b), (d, _) in edges.items():
        adj_sub[a][b] = d
    for _ in range(N_VEHICLES):
        s = rng.choice(sel_list)
        cands = [n for n in sel_list if n != s]
        if not cands: continue
        g = rng.choice(cands)
        _, prev = dijkstra(s, adj_sub)
        if g not in prev and g != s: continue
        path = reconstruct_path(prev, g)
        for i in range(len(path) - 1):
            arc = (path[i], path[i+1])
            if arc in edges: counts[arc] += 1
    return counts


# ── altri helpers ─────────────────────────────────────────────────────────────

def select_connected_subgraph(node_data, adj, max_nodes):
    seed = max(adj.keys(), key=lambda n: len(adj[n]))
    selected = [seed]; sel_set = {seed}; frontier = {}
    for b, (d, _) in adj.get(seed, {}).items():
        if b not in sel_set: frontier[b] = d
    while len(selected) < max_nodes and frontier:
        clat = sum(node_data[n]["lat"] for n in selected) / len(selected)
        clon = sum(node_data[n]["lon"] for n in selected) / len(selected)
        best = max(frontier, key=lambda n: haversine(clat, clon, node_data[n]["lat"], node_data[n]["lon"]))
        selected.append(best); sel_set.add(best); del frontier[best]
        for b, (d, _) in adj.get(best, {}).items():
            if b not in sel_set and b not in frontier: frontier[b] = d
    return selected


def name_map_for(selected, node_data):
    used = {}; name_map = {}
    for n in selected:
        base = slugify(node_data[n]["name"]) or "n" + n[-7:]
        slug = base; i = 2
        while slug in used and used[slug] != n:
            slug = base + "_" + str(i); i += 1
        name_map[n] = slug; used[slug] = n
    return name_map


def compute_reachable(start_osm, edges):
    reach = {start_osm}; q = deque([start_osm])
    while q:
        cur = q.popleft()
        for (a, b) in edges:
            if a == cur and b not in reach:
                reach.add(b); q.append(b)
    return reach


def auto_start_goal(selected, edges, node_data):
    out_deg = Counter(a for a, _ in edges)
    start = max(selected, key=lambda n: out_deg[n])
    reach = compute_reachable(start, edges)
    slat = node_data[start]["lat"]; slon = node_data[start]["lon"]
    goal = max(reach - {start},
               key=lambda n: haversine(slat, slon, node_data[n]["lat"], node_data[n]["lon"]))
    return start, goal


# ── generazione PDDL con tutti i fluenti di congestione ──────────────────────

def write_pddl(zone, selected, node_data, edges, start_osm, goal_osm, nm,
               signal_nodes=None, congestion_delays=None, vehicle_counts=None,
               intersection_density=None, peripheral=None, edge_highway=None):

    if signal_nodes         is None: signal_nodes         = set()
    if congestion_delays    is None: congestion_delays    = {}
    if vehicle_counts       is None: vehicle_counts       = {}
    if intersection_density is None: intersection_density = {}
    if peripheral           is None: peripheral           = set()
    if edge_highway         is None: edge_highway         = {}

    def n(x): return nm[x]
    se = sorted(edges.keys(), key=lambda e: (n(e[0]), n(e[1])))
    selected_signals = [nd for nd in selected if nd in signal_nodes]

    lines = []
    lines.append(f"(define (problem dublin-{zone})")
    lines.append("  (:domain dublin-navigation)")
    lines.append("")
    lines.append(f"  ; {len(selected)} nodi — zona {zone}")
    lines.append(f"  ; START: {n(start_osm)}   GOAL: {n(goal_osm)}")
    lines.append(f"  ; Congestione: {N_VEHICLES} veicoli random, raggio periferia {PERIPH_RADIUS_M}m")
    lines.append("")
    lines.append("  (:objects")
    for nd in selected:
        nd_name = node_data[nd]["name"]
        lines.append(f"    {n(nd)}  ; {nd_name}" if nd_name else f"    {n(nd)}")
    lines.append("    - location")
    lines.append("  )")
    lines.append("")
    lines.append("  (:init")
    lines.append(f"    (at {n(start_osm)})")
    lines.append(f"    (prev {n(start_osm)})   ; nessuna svolta prima del primo arco")
    lines.append("    (= (total-dist) 0)")
    lines.append("    (= (total-time) 0)")

    periph_in_sub = [nd for nd in selected if nd in peripheral]
    if periph_in_sub:
        lines.append("")
        lines.append(f"    ; Zona periferica (> {PERIPH_RADIUS_M}m dal centroide)")
        for nd in periph_in_sub:
            lines.append(f"    (peripheral {n(nd):<28})")

    lines.append("")
    lines.append("    ; Progress = 0 per ogni tratto")
    for a, b in se:
        lines.append(f"    (= (progress {n(a):<28} {n(b)}) 0)")
    lines.append("")
    lines.append("    ; Strade")
    for a, b in se:
        lines.append(f"    (road {n(a):<28} {n(b)})")
    lines.append("")
    lines.append("    ; Distanze (metri)")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (distance {n(a):<28} {n(b)}) {d})")
    lines.append("")
    lines.append("    ; Velocita' base (m/s)")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (speed {n(a):<28} {n(b)}) {spd})")
    # --- out_adj/in_adj/triples: servono sia al blocco signal-delay
    # (per movimento) sia al blocco turn-time piu' sotto ---
    out_adj = defaultdict(list); in_adj = defaultdict(list)
    for (a, b) in edges:
        out_adj[a].append(b); in_adj[b].append(a)
    triples = [(a, b, c) for b in selected
               for a in in_adj.get(b, [])
               for c in out_adj.get(b, [])]

    def _signal_delay_fallback(nd):
        return signal_delay_for(nd, signal_nodes)

    lines.append("")
    n_from_sumo = sum(1 for nd in selected if nd in SUMO_DELAYS)
    n_with_movements = sum(1 for nd in selected if nd in SUMO_MOVEMENTS)
    lines.append(f"    ; Ritardo semaforico (s) per MOVIMENTO (prev,from,to) — "
                 f"{len(selected_signals)}/{len(selected)} nodi")
    lines.append(f"    ; Realistico da SUMO/SCATS per fase/direzione "
                 f"({n_with_movements} nodi con dati di movimento, {n_from_sumo} con media); "
                 f"default {DEFAULT_SIGNAL_DELAY}s per gli altri semafori")
    for c in sorted(out_adj.get(start_osm, []), key=n):
        delay = assign_movement_signal_delay(start_osm, start_osm, c, node_data, SUMO_MOVEMENTS, is_first=True)
        if delay is None:
            delay = _signal_delay_fallback(start_osm)
        lines.append(f"    (= (signal-delay {n(start_osm):<20} {n(start_osm):<20} {n(c)}) {delay})")
    for a, b, c in sorted(triples, key=lambda t: (n(t[0]), n(t[1]), n(t[2]))):
        delay = assign_movement_signal_delay(a, b, c, node_data, SUMO_MOVEMENTS)
        if delay is None:
            delay = _signal_delay_fallback(b)
        lines.append(f"    (= (signal-delay {n(a):<20} {n(b):<20} {n(c)}) {delay})")
    lines.append("")
    lines.append("    ; Ritardo congestione statico (s)")
    for nd in selected:
        cd = congestion_delays.get(nd, 0)
        lines.append(f"    (= (congestion-delay {n(nd):<28}) {cd})")
    lines.append("")
    lines.append(f"    ; Densita' incroci (n. nodi entro {DENSITY_RADIUS_M}m)")
    for nd in selected:
        dens = intersection_density.get(nd, 0)
        lines.append(f"    (= (intersection-density {n(nd):<28}) {dens})")
    lines.append("")
    lines.append(f"    ; Veicoli per arco ({N_VEHICLES} Dijkstra random, seed={RANDOM_SEED})")
    for a, b in se:
        vc = vehicle_counts.get((a, b), 0)
        lines.append(f"    (= (vehicle-count {n(a):<28} {n(b)}) {vc})")
    lines.append("")
    lines.append("    ; Fattore congestione = 1 + vehicle-count/10")
    for a, b in se:
        vc = vehicle_counts.get((a, b), 0)
        cf = round(1.0 + vc / 10.0, 2)
        lines.append(f"    (= (congestion-factor {n(a):<28} {n(b)}) {cf})")
    lines.append("")
    lines.append("    ; Velocita' effettiva (m/s) = speed / congestion-factor  [usata dal processo]")
    for a, b in se:
        d, spd = edges[(a, b)]
        vc = vehicle_counts.get((a, b), 0)
        cf = 1.0 + vc / 10.0
        eff = round(spd / cf, 4)
        lines.append(f"    (= (effective-speed {n(a):<28} {n(b)}) {eff})")
    lines.append("")
    lines.append("    ; Tempo di percorrenza (s) = distance / effective-speed  [usato nell'evento]")
    for a, b in se:
        d, spd = edges[(a, b)]
        vc = vehicle_counts.get((a, b), 0)
        cf = 1.0 + vc / 10.0
        eff = spd / cf
        arc_t = round(d / eff, 4) if eff > 0 else 0
        lines.append(f"    (= (arc-time {n(a):<28} {n(b)}) {arc_t})")

    # ── Turn-time: tempo di svolta per ogni tripla di nodi consecutivi ──────
    # (out_adj/in_adj/triples gia' calcolati sopra, per il blocco signal-delay)
    lines.append("")
    lines.append(f"    ; Tempo di svolta (s) = angolo / {TURN_RATE_DPS:.0f} deg/s  [turn rate]")
    for c in sorted(out_adj.get(start_osm, []), key=n):
        lines.append(f"    (= (turn-time {n(start_osm):<20} {n(start_osm):<20} {n(c)}) 0)")
    for a, b, c in sorted(triples, key=lambda t: (n(t[0]), n(t[1]), n(t[2]))):
        tt = turn_time_s(a, b, c, node_data)
        lines.append(f"    (= (turn-time {n(a):<20} {n(b):<20} {n(c)}) {tt})")

    lines.append("")
    lines.append("  )")
    lines.append("")
    lines.append(f"  (:goal (at {n(goal_osm)}))")
    lines.append("")
    lines.append("  (:metric minimize (total-time))")
    lines.append(")")
    return "\n".join(lines)


# ── ricerca ENHSP ─────────────────────────────────────────────────────────────

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


# ── routes ────────────────────────────────────────────────────────────────────

@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/generate', methods=['POST'])
def generate():
    osm_file  = request.files.get('osm_file')
    max_nodes = int(request.form.get('max_nodes', 50))
    zone      = request.form.get('zone', 'custom') or 'custom'

    if not osm_file:
        return jsonify({'error': 'Nessun file caricato'}), 400

    tmp_dir  = tempfile.mkdtemp()
    osm_path = os.path.join(tmp_dir, 'input.osm')
    osm_file.save(osm_path)

    try:
        node_data, adj, signal_node_ids, edge_highway = build_contracted_graph(osm_path)
        if not adj:
            return jsonify({'error': 'Nessuna strada percorribile trovata nel file OSM'}), 400

        selected = select_connected_subgraph(node_data, adj, max_nodes)
        sel_set  = set(selected)

        edges = {}
        for a in selected:
            for b, (d, spd) in adj.get(a, {}).items():
                if b in sel_set:
                    edges[(a, b)] = (d, spd)

        start_osm, goal_osm = auto_start_goal(selected, edges, node_data)
        nm     = name_map_for(selected, node_data)
        nm_inv = {v: k for k, v in nm.items()}
        signal_nodes_in_subgraph = signal_node_ids & sel_set

        # pre-calcolo congestione (salvato nel token)
        peripheral  = classify_zones(selected, node_data)
        density     = compute_intersection_density(selected, node_data)
        sub_hw      = {(a, b): edge_highway.get((a, b), "unclassified") for (a, b) in edges}
        cong_delays = compute_congestion_delay(selected, edges, peripheral, density, sub_hw)
        vc          = compute_vehicle_counts(selected, edges)

        token = str(uuid.uuid4())
        graph_store[token] = {
            'node_data':   node_data,
            'edges':       edges,
            'selected':    selected,
            'nm':          nm,
            'nm_inv':      nm_inv,
            'zone':        zone,
            'signal_nodes': signal_nodes_in_subgraph,
            'peripheral':  peripheral,
            'density':     density,
            'cong_delays': cong_delays,
            'vehicle_counts': vc,
            'edge_highway': sub_hw,
        }

        nodes_out = [{
            'id': nm[nd], 'lat': node_data[nd]['lat'], 'lon': node_data[nd]['lon'],
            'name': node_data[nd]['name'], 'is_start': nd == start_osm,
            'is_goal': nd == goal_osm, 'is_signal': nd in signal_nodes_in_subgraph,
            'is_peripheral': nd in peripheral,
            'congestion_delay': cong_delays.get(nd, 0),
            'intersection_density': density.get(nd, 0),
        } for nd in selected]

        edges_out = [{
            'from': nm[a], 'to': nm[b], 'distance': d, 'speed': spd,
            'vehicle_count': vc.get((a, b), 0),
            'congestion_factor': round(1.0 + vc.get((a, b), 0) / 10.0, 2),
        } for (a, b), (d, spd) in edges.items()]

        return jsonify({
            'success': True, 'token': token,
            'nodes': nodes_out, 'edges': edges_out,
            'auto_start': nm[start_osm], 'auto_goal': nm[goal_osm],
            'stats': {'n_nodes': len(selected), 'n_edges': len(edges)},
        })

    except ET.ParseError as e:
        return jsonify({'error': f'File OSM non valido: {e}'}), 400
    except Exception as e:
        import traceback
        return jsonify({'error': str(e), 'trace': traceback.format_exc()}), 500


@app.route('/api/solve', methods=['POST'])
def solve():
    data       = request.get_json()
    token      = data.get('token')
    start_pddl = data.get('start')
    goal_pddl  = data.get('goal')

    store = graph_store.get(token)
    if not store:
        return jsonify({'error': 'Sessione scaduta, ricarica il file OSM'}), 400

    nm           = store['nm']
    nm_inv       = store['nm_inv']
    node_data    = store['node_data']
    edges        = store['edges']
    selected     = store['selected']
    zone         = store['zone']
    signal_nodes = store.get('signal_nodes', set())
    peripheral   = store.get('peripheral', set())
    density      = store.get('density', {})
    cong_delays  = store.get('cong_delays', {})
    vc           = store.get('vehicle_counts', {})
    sub_hw       = store.get('edge_highway', {})

    start_osm = nm_inv.get(start_pddl)
    goal_osm  = nm_inv.get(goal_pddl)

    if not start_osm:
        return jsonify({'error': f'Start "{start_pddl}" non trovato'}), 400
    if not goal_osm:
        return jsonify({'error': f'Goal "{goal_pddl}" non trovato'}), 400
    if start_osm == goal_osm:
        return jsonify({'error': 'Start e Goal devono essere nodi diversi'}), 400

    reach = compute_reachable(start_osm, edges)
    if goal_osm not in reach:
        return jsonify({'error': f'Il goal "{goal_pddl}" non è raggiungibile da "{start_pddl}"'}), 400

    pddl_content = write_pddl(
        zone, selected, node_data, edges, start_osm, goal_osm, nm,
        signal_nodes=signal_nodes,
        congestion_delays=cong_delays,
        vehicle_counts=vc,
        intersection_density=density,
        peripheral=peripheral,
        edge_highway=sub_hw,
    )

    PDDL_DIR = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..', 'pddl_files')
    custom_pddl_path = os.path.join(PDDL_DIR, 'problem_custom.pddl')
    try:
        os.makedirs(PDDL_DIR, exist_ok=True)
        with open(custom_pddl_path, 'w', encoding='utf-8') as f:
            f.write(pddl_content)
    except Exception:
        pass

    tmp_dir   = tempfile.mkdtemp()
    pddl_path = os.path.join(tmp_dir, 'problem.pddl')
    with open(pddl_path, 'w', encoding='utf-8') as f:
        f.write(pddl_content)

    plan_text = route = total_dist = travel_time = None
    signals_crossed = signal_delay_total = plan_time_ms = None
    enhsp_error = None

    jar        = trova_enhsp()
    domain_abs = os.path.abspath(DOMAIN_PATH)

    if not jar:
        enhsp_error = "ENHSP non trovato — installa con: pip install up-enhsp"
    elif not os.path.exists(domain_abs):
        enhsp_error = f"domain.pddl non trovato in: {domain_abs}"
    else:
        try:
            cmd    = ["java", "-jar", jar, "-o", domain_abs, "-f", pddl_path, "-s", "aibr"]
            result = subprocess.run(cmd, capture_output=True, text=True, timeout=180)
            output = result.stdout + result.stderr

            if "Problem Solved" in output:
                plan_text, route, plan_time_ms = parse_plan(output)

                # salva il percorso pianificato (sequenza di nodi PDDL) cosi'
                # che SUMO possa seguire ESATTAMENTE le stesse strade del piano,
                # invece di ricalcolare un proprio Dijkstra start->goal.
                route_path = os.path.join(PDDL_DIR, 'route_custom.json')
                try:
                    with open(route_path, 'w', encoding='utf-8') as f:
                        json.dump({'route': route or []}, f)
                except Exception:
                    pass

                if route and len(route) >= 2:
                    total_dist = 0; travel_time = 0.0
                    signals_crossed = 0; signal_delay_total = 0.0
                    turn_delay_total = 0.0
                    route_osm = [nm_inv.get(r) for r in route]
                    for i in range(len(route) - 1):
                        a_osm = route_osm[i]; b_osm = route_osm[i + 1]
                        if a_osm and b_osm and (a_osm, b_osm) in edges:
                            d, spd = edges[(a_osm, b_osm)]
                            total_dist  += d
                            vc_arc       = vc.get((a_osm, b_osm), 0)
                            cf           = 1.0 + vc_arc / 10.0
                            eff_spd      = spd / cf
                            if eff_spd > 0:
                                travel_time += d / eff_spd
                        if not (a_osm and b_osm):
                            continue
                        # ritardo semaforico del movimento (prev,a_osm,b_osm) — pagato
                        # partendo da a_osm, come in domain.pddl 'start-move' (sez. 3.1
                        # di 2_traffic_signal_optimization.md). i==0: nessun prev reale
                        # (start-move fittizio start->start->b_osm).
                        if i == 0:
                            sd = assign_movement_signal_delay(a_osm, a_osm, b_osm, node_data,
                                                               SUMO_MOVEMENTS, is_first=True)
                        else:
                            p_osm = route_osm[i - 1]
                            turn_delay_total += turn_time_s(p_osm, a_osm, b_osm, node_data)
                            sd = assign_movement_signal_delay(p_osm, a_osm, b_osm, node_data, SUMO_MOVEMENTS)
                        if sd is None:
                            sd = signal_delay_for(a_osm, signal_nodes)
                        if sd > 0:
                            signals_crossed    += 1
                            signal_delay_total += sd
                    for node_osm in route_osm[1:]:
                        if node_osm:
                            travel_time += cong_delays.get(node_osm, 0)
                    travel_time += signal_delay_total + turn_delay_total
                    signal_delay_total = round(signal_delay_total, 1)
            else:
                enhsp_error = "ENHSP non ha trovato soluzione (problema forse irrisolvibile con i nodi selezionati)"
        except subprocess.TimeoutExpired:
            enhsp_error = "ENHSP ha superato il timeout (180s) — prova con meno nodi"
        except FileNotFoundError:
            enhsp_error = "Java non trovato — installa Java 17+"

    # calcola sommario congestione sul percorso
    congestion_on_route = []
    if route and len(route) >= 2:
        for i in range(len(route) - 1):
            a_osm = nm_inv.get(route[i])
            b_osm = nm_inv.get(route[i + 1])
            if a_osm and b_osm and (a_osm, b_osm) in edges:
                vc_arc = vc.get((a_osm, b_osm), 0)
                congestion_on_route.append({
                    'from': route[i], 'to': route[i+1],
                    'vehicle_count': vc_arc,
                    'congestion_factor': round(1.0 + vc_arc / 10.0, 2),
                })
        for node_name in route[1:]:
            node_osm = nm_inv.get(node_name)

    congestion_delay_total = sum(
        cong_delays.get(nm_inv.get(node_name), 0)
        for node_name in (route[1:] if route else [])
    )
    n_peripheral_on_route = sum(
        1 for node_name in (route or [])
        if nm_inv.get(node_name) in peripheral
    )

    return jsonify({
        'success': True,
        'pddl_content': pddl_content,
        'plan_text':    plan_text,
        'route':        route,
        'enhsp_error':  enhsp_error,
        'congestion_on_route': congestion_on_route,
        'stats': {
            'total_dist':              total_dist,
            'travel_time':             round(travel_time, 1) if travel_time is not None else None,
            'signals_crossed':         signals_crossed if plan_text else None,
            'signal_delay_total':      signal_delay_total if plan_text else None,
            'congestion_delay_total':  congestion_delay_total if plan_text else None,
            'n_peripheral_on_route':   n_peripheral_on_route if plan_text else None,
            'plan_time_ms':            plan_time_ms,
            'start':                   start_pddl,
            'goal':                    goal_pddl,
        }
    })


@app.route('/api/sumo', methods=['POST'])
def launch_sumo():
    base   = os.path.dirname(os.path.abspath(__file__))
    script = os.path.join(base, '..', 'sumo_visualize.py')
    pddl   = os.path.join(base, '..', 'pddl_files', 'problem_custom.pddl')

    if not os.path.exists(script):
        return jsonify({'error': 'sumo_visualize.py non trovato'}), 400
    if not os.path.exists(pddl):
        return jsonify({'error': 'problem_custom.pddl non trovato'}), 400

    try:
        subprocess.Popen(
            ['python', os.path.abspath(script), 'pddl', os.path.abspath(pddl), 'piccola'],
            cwd=os.path.dirname(os.path.abspath(script))
        )
        return jsonify({'success': True})
    except Exception as e:
        return jsonify({'error': str(e)}), 500


if __name__ == '__main__':
    print("Server avviato su http://localhost:5000")
    app.run(debug=True, port=5000)
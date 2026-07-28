"""
osm_graph.py
------------
Parsing of an .osm file into a contracted graph (real intersections only),
subgraph selection, and computation of simulated static congestion.
Extracted from webapp/app.py.
"""
import math
import re
import random
import heapq
import xml.etree.ElementTree as ET
from collections import defaultdict, Counter, deque

HIGHWAY_TYPES = {
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "residential", "motorway_link", "trunk_link", "primary_link",
    "secondary_link", "tertiary_link", "unclassified", "living_street",
}

# ── congestion parameters ─────────────────────────────────────────────────────
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


# ── base helpers ──────────────────────────────────────────────────────────────

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


# ── OSM parsing (now also returns edge_highway) ────────────────────────────────

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


# ── congestion computation ──────────────────────────────────────────────────────

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


# ── other helpers ─────────────────────────────────────────────────────────────

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


def select_local_subgraph(start_osm, goal_osm, edges, node_data,
                           max_nodes=150, margin_factor=1.6):
    """Minimal subgraph to pass to ENHSP to solve start->goal: the shortest
    path (always included) + a margin of plausible detours (elliptical
    corridor dist_from_start + dist_from_goal <= optimum*margin_factor),
    truncated to max_nodes by taking first the nodes closest to the optimal
    path. Decouples the cost of solving from the size of the loaded map
    (with 'all nodes' on large zones the displayed graph can have thousands
    of nodes even for a route of a few hundred meters — see audit on
    dublin_grande_porto.osm)."""
    adj_fwd = defaultdict(dict)
    adj_rev = defaultdict(dict)
    for (a, b), (d, _spd) in edges.items():
        adj_fwd[a][b] = d
        adj_rev[b][a] = d

    dist_s, prev_s = dijkstra(start_osm, adj_fwd)
    if goal_osm not in dist_s:
        return None  # goal not reachable — the caller must have already checked this

    path = reconstruct_path(prev_s, goal_osm)
    if len(path) >= max_nodes:
        return path  # the path alone saturates the cap: no margin possible

    dist_g, _prev_g = dijkstra(goal_osm, adj_rev)
    budget = dist_s[goal_osm] * margin_factor

    candidates = sorted(
        (dist_s[n] + dist_g[n], n) for n in dist_s
        if n in dist_g and dist_s[n] + dist_g[n] <= budget
    )
    return [n for _score, n in candidates[:max_nodes]]

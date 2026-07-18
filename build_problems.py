import os
import sys
import math
import re
import heapq
import random
import xml.etree.ElementTree as ET
from collections import defaultdict, Counter, deque

BASE = os.path.dirname(os.path.abspath(__file__))
OSM_DIR = os.path.join(BASE, "osm_files")
PDDL_DIR = os.path.join(BASE, "pddl_files")
SUMO_DIR = os.path.join(BASE, "sumo_extracted")  # JSON da extract_sumo_data.py

# Ritardo semaforico di fallback (s) quando un incrocio non e' mappabile ai
# dati SUMO (es. membro nascosto di un cluster). Prima era l'unico valore.
FALLBACK_SIGNAL_DELAY = 30


def load_sumo_signal_delays(zone):
    """Carica {id_nodo_OSM: ritardo_realistico_s} da sumo_extracted/.
    Ritorna {} se il file non esiste (il generatore usa il fallback 30s).
    Media per nodo — mantenuta per retro-compatibilita' e come fallback
    quando un movimento specifico non e' mappabile (vedi
    assign_movement_signal_delay)."""
    import json
    path = os.path.join(SUMO_DIR, f"sumo_data_{zone}.json")
    if not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    return {k: round(float(v), 1) for k, v in data.get("node_signal_delay", {}).items()}


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


def load_sumo_movements(zone):
    """Carica {id_nodo_OSM: [movement, ...]} da sumo_extracted/, espandendo
    i cluster SUMO sui rispettivi nodi OSM membri. Ogni 'movement' e' un
    dict con delay_s/bearing_in_bucket/bearing_out_bucket/dir_label (vedi
    extract_sumo_data.py::extract_traffic_lights). Usato da
    assign_movement_signal_delay per attribuire il ritardo semaforico
    corretto a ciascuna tripla PDDL (prev,from,to), invece della media
    per nodo. Ritorna {} se il file non esiste."""
    import json
    path = os.path.join(SUMO_DIR, f"sumo_data_{zone}.json")
    if not os.path.exists(path):
        return {}
    with open(path, encoding="utf-8") as f:
        data = json.load(f)
    out = defaultdict(list)
    for tid, tl in data.get("traffic_lights", {}).items():
        members = cluster_member_ids(tid)
        for mv in tl.get("movements", []):
            if mv.get("bearing_in_bucket") is None or mv.get("bearing_out_bucket") is None:
                continue
            for nid in members:
                out[nid].append(mv)
    return dict(out)


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
    il nodo 'b': calcola i bearing di ingresso (a->b) e uscita (b->c) da
    lat/lon e li confronta con i bucket direzionali (bearing_in_bucket,
    bearing_out_bucket) dei movimenti SUMO disponibili per 'b', scegliendo
    quello con distanza angolare complessiva minima.
    is_first=True (tripla fittizia iniziale start,start,c): non esiste un
    arco reale precedente, quindi il match usa solo il bearing di uscita.
    Ritorna None se 'b' non ha dati di movimento SUMO (nessun match possibile
    -> il chiamante ricade sul valore scalare esistente)."""
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

# tipi di strada che consideriamo percorribili
HIGHWAY_TYPES = {
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "residential", "motorway_link", "trunk_link", "primary_link",
    "secondary_link", "tertiary_link", "unclassified", "living_street",
}

# --- PARAMETRI SVOLTA (turn rate) ---
# Turn rate = velocita' angolare di cambio direzione di un veicolo.
# Yaw rate tipico di un'auto in svolta urbana stretta ~15-20 gradi/s
# (oltre ~30 gradi/s interviene l'ESC). tempo_svolta = angolo / turn_rate.
TURN_RATE_DPS    = 20.0  # gradi/secondo

# --- PARAMETRI CONGESTIONE ---
N_VEHICLES       = 10    # veicoli random aggiuntivi da simulare
PERIPH_RADIUS_M  = 600   # distanza dal centroide oltre cui un nodo è periferico
DENSITY_RADIUS_M = 200   # raggio per contare gli incroci vicini (intersection-density)
RANDOM_SEED      = 42    # riproducibilità

# Ritardo congestione statico in secondi per tipo di strada
CONGESTION_DELAY_BY_HIGHWAY = {
    "primary":        20,
    "primary_link":   15,
    "secondary":      10,
    "secondary_link":  8,
    "tertiary":        5,
    "tertiary_link":   3,
    "residential":     5,
    "living_street":   3,
    "motorway":        0,
    "motorway_link":   0,
    "trunk":           0,
    "trunk_link":      0,
    "unclassified":    0,
}


def haversine(lat1, lon1, lat2, lon2):
    """Distanza in metri tra due punti GPS."""
    R = 6371000
    f1 = math.radians(lat1)
    f2 = math.radians(lat2)
    df = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(df/2)**2 + math.cos(f1) * math.cos(f2) * math.sin(dl/2)**2
    return round(R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))


def bearing(lat1, lon1, lat2, lon2):
    """Rotta (gradi, 0=Nord, senso orario) dal punto 1 al punto 2."""
    f1 = math.radians(lat1)
    f2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(f2)
    x = math.cos(f1) * math.sin(f2) - math.sin(f1) * math.cos(f2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def turn_angle(prev, mid, nxt, node_data):
    """Angolo di svolta (gradi, 0=dritto, 180=inversione) al nodo 'mid'
    arrivando da 'prev' e proseguendo verso 'nxt'."""
    b_in = bearing(node_data[prev]["lat"], node_data[prev]["lon"],
                   node_data[mid]["lat"],  node_data[mid]["lon"])
    b_out = bearing(node_data[mid]["lat"], node_data[mid]["lon"],
                    node_data[nxt]["lat"], node_data[nxt]["lon"])
    a = (b_out - b_in + 180) % 360 - 180   # normalizza in (-180, 180]
    return abs(a)


def turn_time_s(prev, mid, nxt, node_data, turn_rate=TURN_RATE_DPS):
    """Tempo di svolta (s) = angolo / turn_rate."""
    return round(turn_angle(prev, mid, nxt, node_data) / turn_rate, 2)


def slugify(name):
    """Converte il nome della strada in un id pddl valido."""
    if not name:
        return ""
    s = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")[:28]
    return s if s else ""


# ---------------------------------------------------------------------------
# COMPONENTE 4 — Classificazione periferia / centro
# ---------------------------------------------------------------------------

def classify_zones(selected, node_data, periph_radius_m=PERIPH_RADIUS_M):
    """
    Restituisce un set di nodi 'periferici' (distanza dal centroide > soglia).
    I rimanenti sono considerati 'urbani'.
    """
    lats = [node_data[n]["lat"] for n in selected]
    lons = [node_data[n]["lon"] for n in selected]
    clat = sum(lats) / len(lats)
    clon = sum(lons) / len(lons)

    peripheral = set()
    for n in selected:
        d = haversine(clat, clon, node_data[n]["lat"], node_data[n]["lon"])
        if d > periph_radius_m:
            peripheral.add(n)

    return peripheral


# ---------------------------------------------------------------------------
# COMPONENTE 2 — Densità incroci vicini
# ---------------------------------------------------------------------------

def compute_intersection_density(selected, node_data, radius_m=DENSITY_RADIUS_M):
    """
    Per ogni nodo, conta quanti altri nodi del sottografo si trovano
    entro radius_m (distanza Haversine). Restituisce {nid: count}.
    """
    density = {}
    for nid in selected:
        lat = node_data[nid]["lat"]
        lon = node_data[nid]["lon"]
        count = sum(
            1 for other in selected
            if other != nid and
            haversine(lat, lon, node_data[other]["lat"], node_data[other]["lon"]) <= radius_m
        )
        density[nid] = count
    return density


# ---------------------------------------------------------------------------
# COMPONENTE 3 — Congestion-delay statico per nodo
# ---------------------------------------------------------------------------

def compute_congestion_delay(selected, node_data, edges, peripheral,
                              density, edge_highway):
    """
    Calcola il ritardo di congestione statico (secondi) per ogni nodo.
    Regole:
      - Nodo periferico  → delay = 0
      - Nodo urbano:
          * base = max ritardo tra tutti gli archi *entranti* (per tipo di strada)
          * bonus densità = min(density[n], 5) * 2  (0–10 s extra)
    """
    # mappa nodo → tipi di strada degli archi entranti
    incoming_types = defaultdict(set)
    for (a, b) in edges:
        hw = edge_highway.get((a, b), "unclassified")
        incoming_types[b].add(hw)

    delays = {}
    for n in selected:
        if n in peripheral:
            delays[n] = 0
            continue

        # base: ritardo massimo tra le strade entranti
        base = max(
            (CONGESTION_DELAY_BY_HIGHWAY.get(hw, 0) for hw in incoming_types[n]),
            default=0
        )
        # bonus densità: +2 s per ogni incrocio vicino, fino a +10 s
        bonus = min(density.get(n, 0), 5) * 2
        delays[n] = base + bonus

    return delays


# ---------------------------------------------------------------------------
# COMPONENTE 1 — N veicoli random + vehicle-count per arco
# ---------------------------------------------------------------------------

def dijkstra(start, adj_sub):
    """
    Dijkstra su adj_sub = {nodo: {vicino: distanza, ...}}.
    Restituisce {nodo: (dist, predecessore)}.
    """
    dist = {start: 0}
    prev = {start: None}
    pq = [(0, start)]
    while pq:
        d, u = heapq.heappop(pq)
        if d > dist.get(u, float("inf")):
            continue
        for v, w in adj_sub.get(u, {}).items():
            nd = d + w
            if nd < dist.get(v, float("inf")):
                dist[v] = nd
                prev[v] = u
                heapq.heappush(pq, (nd, v))
    return dist, prev


def reconstruct_path(prev, goal):
    path = []
    cur = goal
    while cur is not None:
        path.append(cur)
        cur = prev.get(cur)
    return list(reversed(path))


def compute_vehicle_counts(selected, edges, main_start, n_vehicles=N_VEHICLES,
                            seed=RANDOM_SEED):
    """
    Genera N_VEHICLES veicoli random, calcola il percorso Dijkstra per ognuno
    e incrementa il contatore sull'arco.
    Restituisce {(a,b): count} per gli archi percorsi.
    """
    rng = random.Random(seed)
    sel_list = list(selected)
    counts = Counter()

    # grafo ridotto per Dijkstra (solo distanze)
    adj_sub = defaultdict(dict)
    for (a, b), (d, spd) in edges.items():
        adj_sub[a][b] = d

    for _ in range(n_vehicles):
        # start e goal random tra i nodi selezionati, diversi tra loro
        s = rng.choice(sel_list)
        candidates = [n for n in sel_list if n != s]
        if not candidates:
            continue
        g = rng.choice(candidates)

        _, prev = dijkstra(s, adj_sub)
        if g not in prev and g != s:
            continue  # nodo irraggiungibile, salta

        path = reconstruct_path(prev, g)
        for i in range(len(path) - 1):
            arc = (path[i], path[i+1])
            if arc in edges:
                counts[arc] += 1

    return counts


# ---------------------------------------------------------------------------
# PARSING OSM
# ---------------------------------------------------------------------------

def build_contracted_graph(osm_path):
    with open(osm_path) as f:
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
    # mappa (nodo_start, nodo_end) → tipo di strada
    way_highway = {}

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
    edge_highway = {}  # (a, b) → tipo di strada

    for nds, spd, oneway, hw in good_ways:
        seg_start = None
        seg_dist = 0
        for i, nid in enumerate(nds):
            if i == 0:
                if nid in junctions:
                    seg_start = nid
                    seg_dist = 0
                continue
            prev_n = nds[i - 1]
            if prev_n in node_data and nid in node_data:
                seg_dist += haversine(
                    node_data[prev_n]["lat"], node_data[prev_n]["lon"],
                    node_data[nid]["lat"],    node_data[nid]["lon"],
                )
            if nid in junctions:
                if seg_start and seg_start != nid and seg_dist > 0:
                    if nid not in adj[seg_start] or adj[seg_start][nid][0] > seg_dist:
                        adj[seg_start][nid] = (seg_dist, spd)
                        edge_highway[(seg_start, nid)] = hw
                    if not oneway:
                        if seg_start not in adj[nid] or adj[nid][seg_start][0] > seg_dist:
                            adj[nid][seg_start] = (seg_dist, spd)
                            edge_highway[(nid, seg_start)] = hw
                seg_start = nid
                seg_dist = 0

    return node_data, adj, signal_node_ids, edge_highway


def select_connected_subgraph(node_data, adj, max_nodes):
    seed = max(adj.keys(), key=lambda n: len(adj[n]))
    selected = [seed]
    sel_set = {seed}
    frontier = {}
    for b, (d, _) in adj.get(seed, {}).items():
        if b not in sel_set:
            frontier[b] = d

    while len(selected) < max_nodes and frontier:
        clat = sum(node_data[n]["lat"] for n in selected) / len(selected)
        clon = sum(node_data[n]["lon"] for n in selected) / len(selected)
        best = max(frontier, key=lambda n: haversine(
            clat, clon, node_data[n]["lat"], node_data[n]["lon"]))
        selected.append(best)
        sel_set.add(best)
        del frontier[best]
        for b, (d, _) in adj.get(best, {}).items():
            if b not in sel_set and b not in frontier:
                frontier[b] = d

    return selected


def name_map_for(selected, node_data):
    used = {}
    name_map = {}
    for n in selected:
        base = slugify(node_data[n]["name"]) or "n" + n[-7:]
        slug = base
        i = 2
        while slug in used and used[slug] != n:
            slug = base + "_" + str(i)
            i += 1
        name_map[n] = slug
        used[slug] = n
    return name_map


# ---------------------------------------------------------------------------
# SCRITTURA PDDL
# ---------------------------------------------------------------------------

def write_pddl(zone, selected, node_data, edges, start, goal, out_path,
               signal_nodes=None, congestion_delays=None, vehicle_counts=None,
               intersection_density=None, peripheral=None, sumo_delays=None,
               sumo_movements=None):
    sname = name_map_for(selected, node_data)
    if signal_nodes         is None: signal_nodes         = set()
    if congestion_delays    is None: congestion_delays    = {}
    if vehicle_counts       is None: vehicle_counts       = {}
    if intersection_density is None: intersection_density = {}
    if peripheral           is None: peripheral           = set()
    if sumo_delays          is None: sumo_delays          = {}
    if sumo_movements       is None: sumo_movements       = {}

    def nm(n):
        return sname[n]

    se = sorted(edges.keys(), key=lambda e: (nm(e[0]), nm(e[1])))
    selected_signals = [n for n in selected if n in signal_nodes]

    lines = []
    lines.append(f"(define (problem dublin-{zone})")
    lines.append("  (:domain dublin-navigation)")
    lines.append("")
    lines.append(f"  ; {len(selected)} nodi — zona {zone} di Dublino")
    lines.append(f"  ; Generato automaticamente da OSM via build_problems.py")
    lines.append(f"  ; START: {nm(start)}   GOAL: {nm(goal)}")
    lines.append(f"  ; Congestione: {N_VEHICLES} veicoli random, raggio periferia {PERIPH_RADIUS_M}m")
    lines.append("")
    lines.append("  (:objects")
    for n in selected:
        nd_name = node_data[n]["name"]
        if nd_name:
            lines.append(f"    {nm(n)}  ; {nd_name}")
        else:
            lines.append(f"    {nm(n)}")
    lines.append("    - location")
    lines.append("  )")
    lines.append("")
    lines.append("  (:init")
    lines.append(f"    (at {nm(start)})")
    lines.append(f"    (prev {nm(start)})   ; nessuna svolta prima del primo arco")
    lines.append("    (= (total-dist) 0)")
    lines.append("    (= (total-time) 0)")

    # --- COMPONENTE 4: predicato peripheral ---
    periph_in_sub = [n for n in selected if n in peripheral]
    if periph_in_sub:
        lines.append("")
        lines.append(f"    ; Zona periferica (distanza dal centroide > {PERIPH_RADIUS_M} m)")
        for n in periph_in_sub:
            lines.append(f"    (peripheral {nm(n):<28})")

    lines.append("")
    lines.append("    ; Progress = 0 per ogni tratto")
    for a, b in se:
        lines.append(f"    (= (progress {nm(a):<28} {nm(b)}) 0)")

    lines.append("")
    lines.append("    ; Strade")
    for a, b in se:
        lines.append(f"    (road {nm(a):<28} {nm(b)})")

    lines.append("")
    lines.append("    ; Distanze (metri)")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (distance {nm(a):<28} {nm(b)}) {d})")

    lines.append("")
    lines.append("    ; Velocita' base (m/s) — da OSM maxspeed")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (speed {nm(a):<28} {nm(b)}) {spd})")

    # --- out_adj/in_adj/triples: calcolati qui perche' servono sia al
    # blocco signal-delay (per movimento) sia al blocco turn-time sotto ---
    out_adj = defaultdict(list)
    in_adj  = defaultdict(list)
    for (a, b) in edges:
        out_adj[a].append(b)
        in_adj[b].append(a)
    triples = [(a, b, c) for b in selected
               for a in in_adj.get(b, [])
               for c in out_adj.get(b, [])]

    def _signal_delay_fallback(node):
        if node in sumo_delays:
            return sumo_delays[node]
        if node in signal_nodes:
            return FALLBACK_SIGNAL_DELAY
        return 0

    lines.append("")
    n_from_sumo = sum(1 for n in selected if n in sumo_delays)
    n_with_movements = sum(1 for n in selected if n in sumo_movements)
    lines.append(f"    ; Ritardo semaforico (s) per MOVIMENTO (prev,from,to) — "
                 f"{len(selected_signals)}/{len(selected)} nodi con semaforo")
    lines.append(f"    ; Valori realistici da SUMO/SCATS per fase/direzione "
                 f"({n_with_movements} nodi con dati di movimento, {n_from_sumo} "
                 f"con media per nodo); fallback {FALLBACK_SIGNAL_DELAY}s dove non mappabile")
    for c in sorted(out_adj.get(start, []), key=nm):
        delay = assign_movement_signal_delay(start, start, c, node_data, sumo_movements, is_first=True)
        if delay is None:
            delay = _signal_delay_fallback(start)
        lines.append(f"    (= (signal-delay {nm(start):<20} {nm(start):<20} {nm(c)}) {delay})")
    for a, b, c in sorted(triples, key=lambda t: (nm(t[0]), nm(t[1]), nm(t[2]))):
        delay = assign_movement_signal_delay(a, b, c, node_data, sumo_movements)
        if delay is None:
            delay = _signal_delay_fallback(b)
        lines.append(f"    (= (signal-delay {nm(a):<20} {nm(b):<20} {nm(c)}) {delay})")

    # --- COMPONENTE 3: congestion-delay per nodo ---
    lines.append("")
    lines.append("    ; Ritardo congestione statico (s)  — tipo strada + zona + densita'")
    for n in selected:
        cd = congestion_delays.get(n, 0)
        lines.append(f"    (= (congestion-delay {nm(n):<28}) {cd})")

    # --- COMPONENTE 2: intersection-density per nodo ---
    lines.append("")
    lines.append(f"    ; Densita' incroci (n. nodi entro {DENSITY_RADIUS_M} m)")
    for n in selected:
        dens = intersection_density.get(n, 0)
        lines.append(f"    (= (intersection-density {nm(n):<28}) {dens})")

    # --- COMPONENTI 1+: vehicle-count, congestion-factor, effective-speed ---
    # effective-speed = speed / congestion-factor  (precalcolato in Python)
    # Il processo driving usa effective-speed direttamente → nessuna divisione runtime
    lines.append("")
    lines.append(f"    ; Veicoli per arco — {N_VEHICLES} percorsi Dijkstra random (seed={RANDOM_SEED})")
    for a, b in se:
        vc = vehicle_counts.get((a, b), 0)
        lines.append(f"    (= (vehicle-count {nm(a):<28} {nm(b)}) {vc})")

    lines.append("")
    lines.append("    ; Fattore congestione = 1 + vehicle-count/10")
    for a, b in se:
        vc = vehicle_counts.get((a, b), 0)
        cf = round(1.0 + vc / 10.0, 2)
        lines.append(f"    (= (congestion-factor {nm(a):<28} {nm(b)}) {cf})")

    lines.append("")
    lines.append("    ; Velocita' effettiva (m/s) = speed / congestion-factor  [usata dal processo]")
    for a, b in se:
        d, spd = edges[(a, b)]
        vc = vehicle_counts.get((a, b), 0)
        cf = 1.0 + vc / 10.0
        eff = round(spd / cf, 4)
        lines.append(f"    (= (effective-speed {nm(a):<28} {nm(b)}) {eff})")

    lines.append("")
    lines.append("    ; Tempo di percorrenza (s) = distance / effective-speed  [usato nell'evento]")
    for a, b in se:
        d, spd = edges[(a, b)]
        vc = vehicle_counts.get((a, b), 0)
        cf = 1.0 + vc / 10.0
        eff = spd / cf
        arc_t = round(d / eff, 4) if eff > 0 else 0
        lines.append(f"    (= (arc-time {nm(a):<28} {nm(b)}) {arc_t})")

    # --- Turn-time: tempo di svolta per ogni tripla di nodi consecutivi ---
    # (out_adj/in_adj/triples gia' calcolati sopra, per il blocco signal-delay)
    lines.append("")
    lines.append(f"    ; Tempo di svolta (s) = angolo_svolta / {TURN_RATE_DPS:.0f} deg/s  [turn rate]")
    lines.append(f"    ; prima svolta assente: prev=start, turn-time=0 sul primo arco")
    for c in sorted(out_adj.get(start, []), key=nm):
        lines.append(f"    (= (turn-time {nm(start):<20} {nm(start):<20} {nm(c)}) 0)")
    for a, b, c in sorted(triples, key=lambda t: (nm(t[0]), nm(t[1]), nm(t[2]))):
        tt = turn_time_s(a, b, c, node_data)
        lines.append(f"    (= (turn-time {nm(a):<20} {nm(b):<20} {nm(c)}) {tt})")

    lines.append("")
    lines.append("  )")
    lines.append("")
    lines.append(f"  (:goal (at {nm(goal)}))")
    lines.append("")
    lines.append("  (:metric minimize (total-time))")
    lines.append(")")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


# ---------------------------------------------------------------------------
# PIPELINE PRINCIPALE
# ---------------------------------------------------------------------------

def generate(zone, osm_path, max_nodes):
    print(f"\n[{zone.upper()}] Parsing {os.path.basename(osm_path)}...")
    node_data, adj, signal_node_ids, edge_highway = build_contracted_graph(osm_path)
    print(f"  nodi nel grafo contratto: {len(adj)}, archi: {sum(len(v) for v in adj.values())}")
    print(f"  nodi con traffic_signals nell'OSM: {len(signal_node_ids)}")

    selected = select_connected_subgraph(node_data, adj, max_nodes)
    sel_set = set(selected)
    print(f"  nodi selezionati: {len(selected)}")

    edges = {}
    for a in selected:
        for b, (d, spd) in adj.get(a, {}).items():
            if b in sel_set:
                edges[(a, b)] = (d, spd)
    print(f"  archi: {len(edges)}")

    out_deg = Counter(a for a, _ in edges)
    start = max(selected, key=lambda n: out_deg[n])

    reach = {start}
    q = deque([start])
    while q:
        cur = q.popleft()
        for (a, b) in edges:
            if a == cur and b not in reach:
                reach.add(b)
                q.append(b)

    slat = node_data[start]["lat"]
    slon = node_data[start]["lon"]
    goal = max(reach - {start},
               key=lambda n: haversine(slat, slon, node_data[n]["lat"], node_data[n]["lon"]))

    nm = name_map_for(selected, node_data)
    print(f"  raggiungibili: {len(reach)}/{len(selected)}")
    print(f"  START: {start} ({nm[start]})  GOAL: {goal} ({nm[goal]})")

    signal_nodes_in_subgraph = signal_node_ids & set(selected)
    print(f"  incroci con semaforo nel sottografo: {len(signal_nodes_in_subgraph)}")

    # --- COMPONENTE 4: zone ---
    peripheral = classify_zones(selected, node_data)
    print(f"  nodi periferici: {len(peripheral)}/{len(selected)}")

    # --- COMPONENTE 2: densita' incroci ---
    density = compute_intersection_density(selected, node_data)
    avg_dens = sum(density.values()) / max(len(density), 1)
    print(f"  densita' media incroci: {avg_dens:.1f}")

    # edge_highway solo per archi nel sottografo
    sub_edge_highway = {(a, b): edge_highway.get((a, b), "unclassified")
                        for (a, b) in edges}

    # --- COMPONENTE 3: congestion-delay ---
    cong_delays = compute_congestion_delay(selected, node_data, edges, peripheral,
                                           density, sub_edge_highway)
    nonzero_delays = sum(1 for v in cong_delays.values() if v > 0)
    print(f"  nodi con congestion-delay > 0: {nonzero_delays}/{len(selected)}")

    # --- COMPONENTE 1: vehicle-count ---
    vc = compute_vehicle_counts(selected, edges, start)
    print(f"  archi con vehicle-count > 0: {len(vc)}/{len(edges)}")

    # --- Ritardi semaforici realistici da SUMO (per movimento + media/nodo) ---
    sumo_delays = load_sumo_signal_delays(zone)
    sumo_movements = load_sumo_movements(zone)
    matched = sum(1 for n in selected if n in sumo_delays)
    matched_mv = sum(1 for n in selected if n in sumo_movements)
    if sumo_delays:
        print(f"  ritardi SUMO applicati: {matched}/{len(signal_nodes_in_subgraph)} "
              f"semafori del sottografo ({matched_mv} con dati per movimento)")
    else:
        print(f"  [nota] nessun dato SUMO per '{zone}': uso fallback {FALLBACK_SIGNAL_DELAY}s")

    out_path = os.path.join(PDDL_DIR, f"problem_{zone}.pddl")
    write_pddl(zone, selected, node_data, edges, start, goal, out_path,
               signal_nodes=signal_nodes_in_subgraph,
               congestion_delays=cong_delays,
               vehicle_counts=vc,
               intersection_density=density,
               peripheral=peripheral,
               sumo_delays=sumo_delays,
               sumo_movements=sumo_movements)
    print(f"  salvato: {out_path}")


if __name__ == "__main__":
    os.makedirs(PDDL_DIR, exist_ok=True)

    configs = [
        ("piccola", "dublin_piccola_centro.osm", 14),
        ("media",  "dublin_media_residenziale.osm", 50),
        ("grande", "dublin_grande_porto.osm", 120),
    ]

    for zone, osm_file, max_nodes in configs:
        osm_path = os.path.join(OSM_DIR, osm_file)
        if not os.path.exists(osm_path):
            print(f"[ERRORE] file non trovato: {osm_path}")
            print(f"  esegui prima: python download_dublin_map.py")
            sys.exit(1)
        generate(zone, osm_path, max_nodes)

    print("\ndone.")
import os
import sys
import math
import re
import xml.etree.ElementTree as ET
from collections import defaultdict, Counter, deque

BASE = os.path.dirname(os.path.abspath(__file__))
OSM_DIR = os.path.join(BASE, "osm_files")
PDDL_DIR = os.path.join(BASE, "pddl_files")

# tipi di strada che consideriamo percorribili
HIGHWAY_TYPES = {
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "residential", "motorway_link", "trunk_link", "primary_link",
    "secondary_link", "tertiary_link", "unclassified", "living_street",
}


def haversine(lat1, lon1, lat2, lon2):
    # distanza in metri tra due punti GPS
    R = 6371000
    f1 = math.radians(lat1)
    f2 = math.radians(lat2)
    df = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(df/2)**2 + math.cos(f1) * math.cos(f2) * math.sin(dl/2)**2
    return round(R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))


def slugify(name):
    # converte il nome della strada in un id pddl valido (no spazi, no caratteri speciali)
    if not name:
        return ""
    s = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")[:28]
    return s if s else ""


def build_contracted_graph(osm_path):
    with open(osm_path) as f:
        root = ET.parse(f).getroot()

    # leggo tutti i nodi con coordinate, nome e flag semaforo
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

    # scorro le way e tengo solo quelle percorribili in auto
    membership = Counter()
    good_ways = []
    for w in root.findall("way"):
        tags = {t.get("k"): t.get("v") for t in w.findall("tag")}
        if tags.get("highway", "") not in HIGHWAY_TYPES:
            continue
        nds = [nd.get("ref") for nd in w.findall("nd") if nd.get("ref") in node_data]
        if len(nds) < 2:
            continue
        oneway = tags.get("oneway", "no") == "yes"
        try:
            spd = float(tags.get("maxspeed", "30").split()[0])
        except ValueError:
            spd = 30.0
        good_ways.append((nds, round(spd * 1000 / 3600, 2), oneway))
        for nd in nds:
            membership[nd] += 1

    # un incrocio e' un nodo all'inizio/fine di una strada oppure
    # che compare in almeno 2 strade diverse
    junctions = set()
    for nds, _, _ in good_ways:
        junctions.add(nds[0])
        junctions.add(nds[-1])
    junctions |= {n for n, c in membership.items() if c >= 2}

    # costruisco il grafo contratto: collego direttamente gli incroci
    # saltando i nodi intermedi e accumulando la distanza haversine
    adj = defaultdict(dict)
    for nds, spd, oneway in good_ways:
        seg_start = None
        seg_dist = 0
        for i, nid in enumerate(nds):
            if i == 0:
                if nid in junctions:
                    seg_start = nid
                    seg_dist = 0
                continue
            prev = nds[i - 1]
            if prev in node_data and nid in node_data:
                seg_dist += haversine(
                    node_data[prev]["lat"], node_data[prev]["lon"],
                    node_data[nid]["lat"], node_data[nid]["lon"],
                )
            if nid in junctions:
                if seg_start and seg_start != nid and seg_dist > 0:
                    # tengo l'arco piu' corto se ce ne sono piu' tra gli stessi nodi
                    if nid not in adj[seg_start] or adj[seg_start][nid][0] > seg_dist:
                        adj[seg_start][nid] = (seg_dist, spd)
                    if not oneway:
                        if seg_start not in adj[nid] or adj[nid][seg_start][0] > seg_dist:
                            adj[nid][seg_start] = (seg_dist, spd)
                seg_start = nid
                seg_dist = 0

    return node_data, adj, signal_node_ids


def select_connected_subgraph(node_data, adj, max_nodes):
    # parto dal nodo con piu' connessioni uscenti
    seed = max(adj.keys(), key=lambda n: len(adj[n]))

    selected = [seed]
    sel_set = {seed}
    frontier = {}

    for b, (d, _) in adj.get(seed, {}).items():
        if b not in sel_set:
            frontier[b] = d

    while len(selected) < max_nodes and frontier:
        # calcolo il centroide geografico dei nodi gia' selezionati
        clat = sum(node_data[n]["lat"] for n in selected) / len(selected)
        clon = sum(node_data[n]["lon"] for n in selected) / len(selected)
        # aggiungo il nodo piu' lontano dal centroide (per distribuire geograficamente)
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
    # se il nodo ha un nome su OSM lo uso (slugificato),
    # altrimenti prendo le ultime 7 cifre dell'id osm con una n davanti
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


def write_pddl(zone, selected, node_data, edges, start, goal, out_path, signal_nodes=None):
    sname = name_map_for(selected, node_data)
    if signal_nodes is None:
        signal_nodes = set()

    def nm(n):
        return sname[n]

    se = sorted(edges.keys(), key=lambda e: (nm(e[0]), nm(e[1])))
    # nodi selezionati che hanno un semaforo
    selected_signals = [n for n in selected if n in signal_nodes]

    lines = []
    lines.append(f"(define (problem dublin-{zone})")
    lines.append("  (:domain dublin-navigation)")
    lines.append("")
    lines.append(f"  ; {len(selected)} nodi — zona {zone} di Dublino")
    lines.append(f"  ; Generato automaticamente da OSM via build_problems.py")
    lines.append(f"  ; START: {nm(start)}   GOAL: {nm(goal)}")
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
    lines.append("    (= (total-dist) 0)")
    lines.append("    (= (total-time) 0)")
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
    lines.append("    ; Velocita (m/s)")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (speed {nm(a):<28} {nm(b)}) {spd})")
    lines.append("")
    lines.append(f"    ; Ritardo semaforico in secondi (30 = semaforo OSM, 0 = nessun semaforo)")
    lines.append(f"    ; {len(selected_signals)}/{len(selected)} nodi con semaforo")
    for n in selected:
        delay = 30 if n in signal_nodes else 0
        lines.append(f"    (= (signal-delay {nm(n):<28}) {delay})")
    lines.append("")
    lines.append("  )")
    lines.append("")
    lines.append(f"  (:goal (at {nm(goal)}))")
    lines.append("")
    lines.append("  (:metric minimize (total-time))")
    lines.append(")")

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(lines))


def generate(zone, osm_path, max_nodes):
    print(f"\n[{zone.upper()}] Parsing {os.path.basename(osm_path)}...")
    node_data, adj, signal_node_ids = build_contracted_graph(osm_path)
    print(f"  nodi nel grafo contratto: {len(adj)}, archi: {sum(len(v) for v in adj.values())}")
    print(f"  nodi con traffic_signals nell'OSM: {len(signal_node_ids)}")

    selected = select_connected_subgraph(node_data, adj, max_nodes)
    sel_set = set(selected)
    print(f"  nodi selezionati: {len(selected)}")

    # tengo solo gli archi tra nodi selezionati
    edges = {}
    for a in selected:
        for b, (d, spd) in adj.get(a, {}).items():
            if b in sel_set:
                edges[(a, b)] = (d, spd)
    print(f"  archi: {len(edges)}")

    # start = nodo con piu' archi uscenti nel sottografo
    out_deg = Counter(a for a, _ in edges)
    start = max(selected, key=lambda n: out_deg[n])

    # verifico quali nodi sono raggiungibili da start con una BFS
    reach = {start}
    q = deque([start])
    while q:
        cur = q.popleft()
        for (a, b) in edges:
            if a == cur and b not in reach:
                reach.add(b)
                q.append(b)

    # goal = nodo raggiungibile piu' lontano dallo start
    slat = node_data[start]["lat"]
    slon = node_data[start]["lon"]
    goal = max(reach - {start},
               key=lambda n: haversine(slat, slon, node_data[n]["lat"], node_data[n]["lon"]))

    nm = name_map_for(selected, node_data)
    print(f"  raggiungibili: {len(reach)}/{len(selected)}")
    print(f"  START: {start} ({nm[start]})  GOAL: {goal} ({nm[goal]})")

    # incroci selezionati che sono anche semafori OSM
    signal_nodes_in_subgraph = signal_node_ids & set(selected)
    print(f"  incroci con semaforo nel sottografo: {len(signal_nodes_in_subgraph)}")

    out_path = os.path.join(PDDL_DIR, f"problem_{zone}.pddl")
    write_pddl(zone, selected, node_data, edges, start, goal, out_path,
               signal_nodes=signal_nodes_in_subgraph)
    print(f"  salvato: {out_path}")


if __name__ == "__main__":
    os.makedirs(PDDL_DIR, exist_ok=True)

    configs = [
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

"""
build_problems.py
-----------------
Genera automaticamente problem_media.pddl e problem_grande.pddl
a partire dai file OSM scaricati con download_dublin_map.py.

Uso:
    python build_problems.py

Requisiti:
    pip install osmnx   (già in requirements.txt)

Output:
    files/pddl_files/problem_media.pddl   (~50 nodi, ~93 archi)
    files/pddl_files/problem_grande.pddl  (~120 nodi, ~206 archi)

Algoritmo:
    1. Analizza il file OSM e individua i nodi di incrocio
       (nodi che compaiono in ≥ 2 strade, o che sono agli estremi di una strada)
    2. Costruisce un "grafo contratto": ogni arco connette due incroci
       adiacenti con la distanza Haversine accumulata lungo il tratto
    3. Espande un sottografo connesso di N nodi con BFS diretto,
       scegliendo ogni volta il nodo che massimizza la dispersione geografica
    4. Scrive il file PDDL+ con progress/road/distance/speed per ogni arco
"""

import os, sys, math, re
import xml.etree.ElementTree as ET
from collections import defaultdict, Counter, deque

BASE = os.path.dirname(os.path.abspath(__file__))
OSM_DIR  = os.path.join(BASE, "osm_files")
PDDL_DIR = os.path.join(BASE, "pddl_files")

# ── Costanti ──────────────────────────────────────────────────────────────────

HIGHWAY_TYPES = {
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "residential", "motorway_link", "trunk_link", "primary_link",
    "secondary_link", "tertiary_link", "unclassified", "living_street",
}

# ── Utilità ───────────────────────────────────────────────────────────────────

def haversine(lat1, lon1, lat2, lon2):
    """Distanza in metri tra due coordinate GPS."""
    R = 6371000
    f1, f2 = math.radians(lat1), math.radians(lat2)
    df = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(df/2)**2 + math.cos(f1) * math.cos(f2) * math.sin(dl/2)**2
    return round(R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))

def slugify(name):
    """Converte un nome OSM in un identificatore PDDL valido."""
    if not name:
        return ""
    s = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")[:28]
    return s if s else ""

# ── Costruzione grafo contratto ───────────────────────────────────────────────

def build_contracted_graph(osm_path):
    """
    Legge un file OSM e restituisce:
      - node_data: dict nid → {lat, lon, name}
      - adj: dict nid → dict nid → (dist_m, speed_ms)  (grafo orientato)
    Solo i tipi di strada in HIGHWAY_TYPES vengono considerati.
    """
    with open(osm_path) as f:
        root = ET.parse(f).getroot()

    node_data = {}
    for n in root.findall("node"):
        nid = n.get("id")
        tags = {t.get("k"): t.get("v") for t in n.findall("tag")}
        node_data[nid] = {
            "lat": float(n.get("lat")),
            "lon": float(n.get("lon")),
            "name": tags.get("name", ""),
        }

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

    # Incroci = nodi a fine/inizio di una strada, o presenti in ≥ 2 strade
    junctions = set()
    for nds, _, _ in good_ways:
        junctions.add(nds[0])
        junctions.add(nds[-1])
    junctions |= {n for n, c in membership.items() if c >= 2}

    # Grafo contratto: accumula distanza tra incroci consecutivi lungo ogni strada
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
                    node_data[nid]["lat"],  node_data[nid]["lon"],
                )
            if nid in junctions:
                if seg_start and seg_start != nid and seg_dist > 0:
                    if nid not in adj[seg_start] or adj[seg_start][nid][0] > seg_dist:
                        adj[seg_start][nid] = (seg_dist, spd)
                    if not oneway:
                        if seg_start not in adj[nid] or adj[nid][seg_start][0] > seg_dist:
                            adj[nid][seg_start] = (seg_dist, spd)
                seg_start = nid
                seg_dist = 0

    return node_data, adj

# ── Selezione sottografo connesso ─────────────────────────────────────────────

def select_connected_subgraph(node_data, adj, max_nodes):
    """
    Espande un sottografo connesso usando solo archi in avanti (forward edges).
    Ogni passo aggiunge il nodo più lontano dal centroide corrente,
    garantendo che tutti i nodi selezionati siano raggiungibili dallo start.
    """
    # Seed = nodo con maggior grado uscente
    seed = max(adj.keys(), key=lambda n: len(adj[n]))

    selected = [seed]
    sel_set = {seed}
    frontier = {}  # nodo → distanza dal nodo selezionato più vicino

    for b, (d, _) in adj.get(seed, {}).items():
        if b not in sel_set:
            frontier[b] = d

    while len(selected) < max_nodes and frontier:
        # Centroide della selezione corrente
        clat = sum(node_data[n]["lat"] for n in selected) / len(selected)
        clon = sum(node_data[n]["lon"] for n in selected) / len(selected)
        # Scegli il nodo più lontano dal centroide
        best = max(frontier, key=lambda n: haversine(
            clat, clon, node_data[n]["lat"], node_data[n]["lon"]))
        selected.append(best)
        sel_set.add(best)
        del frontier[best]
        for b, (d, _) in adj.get(best, {}).items():
            if b not in sel_set and b not in frontier:
                frontier[b] = d

    return selected

# ── Scrittura PDDL+ ───────────────────────────────────────────────────────────

def write_pddl(zone, selected, node_data, edges, start, goal, out_path):
    sname = name_map_for(selected, node_data)

    def nm(n):
        return sname[n]

    se = sorted(edges.keys(), key=lambda e: (nm(e[0]), nm(e[1])))
    L = [
        f"(define (problem dublin-{zone})",
        "  (:domain dublin-navigation)",
        "",
        f"  ; {len(selected)} nodi — zona {zone} di Dublino",
        f"  ; Generato automaticamente da OSM via build_problems.py",
        f"  ; START: {nm(start)}   GOAL: {nm(goal)}",
        "",
        "  (:objects",
    ]
    for n in selected:
        nd_name = node_data[n]["name"]
        L.append(f"    {nm(n)}{'  ; ' + nd_name if nd_name else ''}")
    L += ["    - location", "  )", "", "  (:init",
          f"    (at {nm(start)})", "    (= (total-dist) 0)", ""]

    L.append("    ; Progress = 0 per ogni tratto")
    for a, b in se:
        L.append(f"    (= (progress {nm(a):<28} {nm(b)}) 0)")
    L += ["", "    ; Strade"]
    for a, b in se:
        L.append(f"    (road {nm(a):<28} {nm(b)})")
    L += ["", "    ; Distanze (metri)"]
    for a, b in se:
        d, spd = edges[(a, b)]
        L.append(f"    (= (distance {nm(a):<28} {nm(b)}) {d})")
    L += ["", "    ; Velocità (m/s)"]
    for a, b in se:
        d, spd = edges[(a, b)]
        L.append(f"    (= (speed {nm(a):<28} {nm(b)}) {spd})")
    L += ["", "  )", "", f"  (:goal (at {nm(goal)}))", "",
          "  (:metric minimize (total-dist))", ")"]

    with open(out_path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))

def name_map_for(selected, node_data):
    """Genera identificatori PDDL unici per ogni nodo."""
    used = {}
    name_map = {}
    for n in selected:
        base = slugify(node_data[n]["name"]) or f"n{n[-7:]}"
        slug = base
        i = 2
        while slug in used and used[slug] != n:
            slug = f"{base}_{i}"
            i += 1
        name_map[n] = slug
        used[slug] = n
    return name_map

# ── Pipeline principale ────────────────────────────────────────────────────────

def generate(zone, osm_path, max_nodes):
    print(f"\n[{zone.upper()}] Parsing {os.path.basename(osm_path)}...")
    node_data, adj = build_contracted_graph(osm_path)
    print(f"  Grafo contratto: {len(adj)} nodi, {sum(len(v) for v in adj.values())} archi")

    selected = select_connected_subgraph(node_data, adj, max_nodes)
    sel_set = set(selected)
    print(f"  Selezionati: {len(selected)} nodi")

    # Archi diretti tra nodi selezionati
    edges = {}
    for a in selected:
        for b, (d, spd) in adj.get(a, {}).items():
            if b in sel_set:
                edges[(a, b)] = (d, spd)
    print(f"  Archi: {len(edges)}")

    # Start = nodo con maggior grado uscente nel sottografo
    out_deg = Counter(a for a, _ in edges)
    start = max(selected, key=lambda n: out_deg[n])

    # Verifica raggiungibilità
    reach = {start}
    q = deque([start])
    while q:
        cur = q.popleft()
        for (a, b) in edges:
            if a == cur and b not in reach:
                reach.add(b)
                q.append(b)

    # Goal = nodo raggiungibile più lontano dallo start
    slat, slon = node_data[start]["lat"], node_data[start]["lon"]
    goal = max(reach - {start},
               key=lambda n: haversine(slat, slon, node_data[n]["lat"], node_data[n]["lon"]))

    nm = name_map_for(selected, node_data)
    print(f"  Raggiungibili: {len(reach)}/{len(selected)}")
    print(f"  START: {start} ({nm[start]})  GOAL: {goal} ({nm[goal]})")

    out_path = os.path.join(PDDL_DIR, f"problem_{zone}.pddl")
    write_pddl(zone, selected, node_data, edges, start, goal, out_path)
    print(f"  → Salvato: {out_path}")


if __name__ == "__main__":
    os.makedirs(PDDL_DIR, exist_ok=True)

    configs = [
        ("media",  "dublin_media_residenziale.osm", 50),
        ("grande", "dublin_grande_porto.osm",        120),
    ]

    for zone, osm_file, max_nodes in configs:
        osm_path = os.path.join(OSM_DIR, osm_file)
        if not os.path.exists(osm_path):
            print(f"[ERRORE] File non trovato: {osm_path}")
            print(f"  Esegui prima: python download_dublin_map.py")
            sys.exit(1)
        generate(zone, osm_path, max_nodes)

    print("\n✅ Tutti i file PDDL generati con successo.")

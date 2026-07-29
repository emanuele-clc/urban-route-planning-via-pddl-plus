"""
sumo_common.py
===============
Funzioni condivise tra sumo_visualize.py (visualizzazione interattiva,
sumo-gui) e compare_sumo.py (confronto baseline/ottimizzato, sumo
headless): grafo della rete SUMO da net.xml, Dijkstra sugli edge, e
risoluzione di un id di nodo OSM/PDDL alla junction SUMO corrispondente.

Prima di questo refactor le tre funzioni erano duplicate (quasi)
identiche nei due script; qui vengono unificate mantenendo il superset di
quello che serve a entrambi (es. build_sumo_graph ritorna anche eid_len,
usato solo da sumo_visualize.py per calcolare la lunghezza totale del
percorso, ma innocuo per compare_sumo.py che lo ignora).
"""
import heapq
import xml.etree.ElementTree as ET
from collections import defaultdict


def build_sumo_graph(net_path):
    """Legge net.xml e ritorna (graph, jpos, eid_len):
    - graph: {junction_from: [(junction_to, edge_id, length), ...]}
    - jpos:  {junction_id: (x, y)} (esclude le junction interne ':...')
    - eid_len: {edge_id: length}
    """
    root = ET.parse(net_path).getroot()
    jpos = {}
    for j in root.findall('junction'):
        jid = j.get('id')
        if jid is None or jid.startswith(':'):
            continue
        jpos[jid] = (float(j.get('x', 0)), float(j.get('y', 0)))
    graph = defaultdict(list)
    eid_len = {}
    for e in root.findall('edge'):
        eid = e.get('id')
        if eid is None or eid.startswith(':'):
            continue
        fr, to = e.get('from'), e.get('to')
        if not fr or not to:
            continue
        lanes = e.findall('lane')
        length = float(lanes[0].get('length', 1)) if lanes else 1.0
        graph[fr].append((to, eid, length))
        eid_len[eid] = length
    return graph, jpos, eid_len


def dijkstra(graph, start, goal):
    """Percorso a costo minimo start->goal sul grafo SUMO, come lista di
    id di archi. None se non raggiungibile."""
    dist = {start: 0.0}
    prev = {}
    heap = [(0.0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist.get(u, float('inf')):
            continue
        if u == goal:
            break
        for v, eid, length in graph[u]:
            nd = d + length
            if nd < dist.get(v, float('inf')):
                dist[v] = nd
                prev[v] = (u, eid)
                heapq.heappush(heap, (nd, v))
    if goal not in prev:
        return None
    edges = []
    cur = goal
    while cur in prev:
        p, eid = prev[cur]
        edges.append(eid)
        cur = p
    return list(reversed(edges))


def pddl_name_to_junction(pname, junc_ids):
    """Mappa un id di nodo OSM/PDDL (es. 'n1193756' o '1193756') alla
    junction SUMO corrispondente.

    Tre tentativi, dal piu' preciso al piu' permissivo:
      1. id esatto — il nome PDDL e' 'n' + id del nodo OSM;
      2. junction il cui id TERMINA con quel suffisso — il nome PDDL conserva
         solo le ultime 7 cifre dell'id OSM (vedi name_map_for in
         build_problems.py), quindi l'id completo puo' essere piu' lungo;
      3. junction CLUSTER che contiene quell'id fra i propri membri —
         netconvert fonde piu' nodi OSM vicini in un'unica junction chiamata
         'cluster_<id1>_<id2>_...'. Senza questo passo un nodo PDDL fuso in un
         cluster risulta "non trovato" pur essendo presente nella rete.
    """
    suffix = str(pname).lstrip('n')
    if not suffix:
        return None
    if suffix in junc_ids:
        return suffix
    matches = [j for j in junc_ids if j.endswith(suffix) and not j.startswith(':')]
    if matches:
        return min(matches, key=len)
    cluster_hits = []
    for j in junc_ids:
        if not j.startswith('cluster_'):
            continue
        for member in j[len('cluster_'):].split('_'):
            if member == suffix or (member.isdigit() and member.endswith(suffix)):
                cluster_hits.append(j)
                break
    if cluster_hits:
        return min(cluster_hits, key=len)
    return None

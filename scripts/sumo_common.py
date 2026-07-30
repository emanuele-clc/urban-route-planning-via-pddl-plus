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
    """Legge net.xml e ritorna (graph, jpos, eid_len, edge_type, connected_pairs):
    - graph: {junction_from: [(junction_to, edge_id, length), ...]}
    - jpos:  {junction_id: (x, y)} (esclude le junction interne ':...')
    - eid_len: {edge_id: length}
    - edge_type: {edge_id: tipo strada OSM SENZA il prefisso 'highway.' che
      netconvert scrive nell'attributo 'type' (es. 'residential',
      'primary'), cosi' e' direttamente compatibile con le chiavi gia'
      usate da CONGESTION_DELAY_BY_HIGHWAY in webapp/osm_graph.py invece di
      richiedere una normalizzazione (o un mancato match silenzioso) in
      ogni punto che lo consuma — vedi 6_proposte_realismo_traffico.md,
      incoerenza A. Stringa vuota se l'attributo manca.
    - connected_pairs: {(edge_id_from, edge_id_to), ...} dagli elementi
      <connection> del net.xml. 'graph' dice solo quali edge condividono
      una junction (from/to), NON se SUMO permette davvero quel passaggio:
      due edge diversi possono arrivare/partire dalla stessa junction senza
      che esista una <connection> reale fra loro (es. per via di divieti di
      svolta, o — caso osservato — due strade DIVERSE che collegano la
      stessa coppia di junction, dove solo una delle combinazioni e'
      effettivamente connessa). Un percorso che concatena due edge non in
      connected_pairs e' un errore FATALE per SUMO ("Error: Vehicle ... has
      no valid route. No connection between edge X and edge Y" — la
      simulazione si ferma subito, ancora piu' grave del warning di ordine
      di partenza gia' visto), quindi qualunque codice che compone rotte
      arco-per-arco SENZA passare da un Dijkstra sull'intero percorso
      (come dijkstra() qui sotto, che e' comunque a livello di junction e
      non lo garantisce nemmeno lui) deve validare ogni coppia di edge
      adiacenti contro connected_pairs prima di accettarla — vedi
      generate_crossing_traffic/generate_parallel_traffic/
      generate_wander_traffic in sumo_visualize.py.

    ATTENZIONE per chi tocca la firma di questa funzione: NON e' usata solo
    da sumo_visualize.py. scripts/compare_sumo.py la importa con alias
    (`build_sumo_graph as _build_sumo_graph`) e la spacchetta
    posizionalmente in un wrapper locale — vedi incoerenza B nello stesso
    documento. Se si aggiungono altri valori di ritorno, aggiornare anche
    quel wrapper (o farlo con un `grep -rn "build_sumo_graph(" scripts/`
    per essere sicuri di aver trovato tutti i chiamanti).
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
    edge_type = {}
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
        raw_type = e.get('type', '')
        edge_type[eid] = raw_type.split('.', 1)[1] if '.' in raw_type else raw_type
    connected_pairs = set()
    for c in root.findall('connection'):
        fr, to = c.get('from'), c.get('to')
        # gli edge interni (':...', le "vie" dentro l'incrocio) non sono
        # mai id di edge percorribili in una <route>, quindi irrilevanti qui
        if fr and to and not fr.startswith(':') and not to.startswith(':'):
            connected_pairs.add((fr, to))
    return graph, jpos, eid_len, edge_type, connected_pairs


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


def build_edge_endpoints(graph):
    """{edge_id: (junction_from, junction_to)} a partire da 'graph' (vedi
    build_sumo_graph): serve a risalire dagli edge SUMO di un percorso gia'
    calcolato alle junction attraversate, senza doverle ritenere separatamente
    (usato per il traffico 'incrociante'/'parallelo', vedi sumo_visualize.py)."""
    endpoints = {}
    for u, neighbors in graph.items():
        for v, eid, _length in neighbors:
            endpoints[eid] = (u, v)
    return endpoints


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

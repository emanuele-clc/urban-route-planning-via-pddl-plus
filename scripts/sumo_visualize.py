import os, sys, math, random, subprocess, re, json, glob, xml.etree.ElementTree as ET
from collections import defaultdict

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))  # scripts/
sys.path.insert(0, SCRIPT_DIR)
from sumo_common import build_sumo_graph, build_edge_endpoints, dijkstra, pddl_name_to_junction  # noqa: E402

def parse_vehicle_counts(text):
    """{(nome_a, nome_b): vehicle_count} dalle righe
    (= (vehicle-count nome_a nome_b) N) gia' presenti nel PDDL (calcolate da
    webapp/osm_graph.py:compute_vehicle_counts). Usato SOLO per pesare quali
    archi del percorso ricevono piu' finestre di traffico di sfondo (vedi
    generate_background_traffic): un arco che il PDDL considera piu'
    congestionato ne riceve di piu' delle altre, cosi' la densita' visibile in
    SUMO resta coerente con il grado di congestione usato per la
    pianificazione — vedi 5_traffico_sfondo_sumo.md §14."""
    out = {}
    for m in re.finditer(r'\(=\s*\(vehicle-count\s+(\S+)\s+(\S+)\)\s+(\d+)\)', text):
        out[(m.group(1), m.group(2))] = int(m.group(3))
    return out

def fit_lonlat_affine(route_names, route_coords, junc_ids, jpos):
    """Trasformazione affine (lon,lat) -> (x,y) del sistema di coordinate della
    rete SUMO, ricavata ai minimi quadrati dai nodi del piano che mappano per
    id (di cui conosciamo sia lat/lon — salvate dalla webapp — sia la posizione
    rete via jpos). Su un'area piccola la proiezione UTM e' di fatto lineare,
    quindi l'affine e' accuratissima. Ritorna (cx, cy) oppure None."""
    if not route_coords:
        return None
    rows = []; xs = []; ys = []
    for name in (route_names or []):
        j = pddl_name_to_junction(name, junc_ids)
        c = route_coords.get(name)
        if j and j in jpos and c:
            rows.append([float(c[1]), float(c[0]), 1.0])   # lon, lat, 1
            xs.append(jpos[j][0]); ys.append(jpos[j][1])
    if len(rows) < 3:
        return None
    try:
        import numpy as np
        A = np.array(rows, dtype=float)
        cx, *_ = np.linalg.lstsq(A, np.array(xs, dtype=float), rcond=None)
        cy, *_ = np.linalg.lstsq(A, np.array(ys, dtype=float), rcond=None)
        return (cx, cy)
    except Exception:
        return None


def _lonlat_to_xy(lat, lon, coef):
    cx, cy = coef
    return (cx[0] * lon + cx[1] * lat + cx[2],
            cy[0] * lon + cy[1] * lat + cy[2])


def load_edge_shapes(net_path):
    """{edge_id: [(x,y), ...]} — forma della prima corsia di ogni edge normale
    del net.xml (esclusi gli edge interni ':...'). Serve al map-matching per
    riconoscere quale edge SUMO corrisponde a un tratto reale del percorso."""
    root = ET.parse(net_path).getroot()
    shapes = {}
    for e in root.findall('edge'):
        eid = e.get('id')
        if not eid or eid.startswith(':') or e.get('function') == 'internal':
            continue
        lane = e.find('lane')
        if lane is None or not lane.get('shape'):
            continue
        try:
            pts = [tuple(map(float, p.split(','))) for p in lane.get('shape').split()]
        except Exception:
            continue
        if len(pts) >= 2:
            shapes[eid] = pts
    return shapes


def _geo_dijkstra(graph, start, goal, geom_xy, edge_shapes, lam=4.0):
    """Come dijkstra(), ma quando deve RIEMPIRE un buco fra due incroci del
    piano (nodi intermedi che SUMO ha semplificato) sceglie il cammino che
    ADERISCE alla via reale invece del piu' corto in assoluto. 'geom_xy' e' la
    polilinea reale di quel tratto (coordinate rete); il costo di ogni edge e'
    la sua lunghezza + lam * (distanza media della sua forma dalla polilinea),
    cosi' fra due strade parallele vince quella su cui passa davvero il
    percorso della webapp. Localizzato al singolo buco: non introduce le
    deviazioni globali del map-matching su tutto il tragitto."""
    import heapq
    memo = {}

    def pen(eid):
        if eid in memo:
            return memo[eid]
        pts = edge_shapes.get(eid)
        if not pts:
            memo[eid] = 0.0; return 0.0
        step = max(1, len(pts) // 3)
        tot = 0.0; c = 0
        for (x, y) in pts[::step]:
            tot += min((gx - x) ** 2 + (gy - y) ** 2 for gx, gy in geom_xy) ** 0.5
            c += 1
        memo[eid] = tot / c if c else 0.0
        return memo[eid]

    dist = {start: 0.0}; prev = {}; heap = [(0.0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if d > dist.get(u, float('inf')):
            continue
        if u == goal:
            break
        for v, eid, length in graph[u]:
            nd = d + length + lam * pen(eid)
            if nd < dist.get(v, float('inf')):
                dist[v] = nd; prev[v] = (u, eid)
                heapq.heappush(heap, (nd, v))
    if goal not in prev:
        return None
    edges = []; cur = goal
    while cur in prev:
        p, eid = prev[cur]; edges.append(eid); cur = p
    return list(reversed(edges))


def make_snapper(junc_ids, jpos, route_names, route_coords):
    """Ritorna una funzione snap(name) -> id junction SUMO.

    Prima prova la mappatura per id (pddl_name_to_junction: esatta/suffisso/
    cluster). Se fallisce — cioe' netconvert ha semplificato quel nodo OSM e
    non esiste come junction — ripiega su una mappatura GEOGRAFICA: stima la
    posizione del nodo nel sistema di coordinate della rete e prende la
    junction piu' vicina.

    La stima usa una trasformazione affine (lon,lat) -> (x,y) ricavata ai
    minimi quadrati dai nodi del piano che INVECE mappano per id (di cui
    conosciamo sia la coppia lat/lon, salvata dalla webapp in route_custom.json,
    sia le coordinate rete via jpos). Su un'area piccola la proiezione UTM e'
    di fatto lineare, quindi l'affine e' accuratissima.

    Serviva perche' quando lo START non e' una junction SUMO la vecchia logica
    ripiegava sul primo nodo mappabile del piano, facendo partire l'auto piu'
    avanti e nella direzione sbagliata (bug: "parte verso sotto invece che a
    sinistra")."""
    coef = fit_lonlat_affine(route_names, route_coords, junc_ids, jpos)
    jitems = list(jpos.items())

    def snap(name):
        j = pddl_name_to_junction(name, junc_ids)
        if j:
            return j
        c = route_coords.get(name)
        if coef is not None and c and jitems:
            px, py = _lonlat_to_xy(float(c[0]), float(c[1]), coef)
            best_j, _ = min(jitems,
                            key=lambda kv: (kv[1][0] - px) ** 2 + (kv[1][1] - py) ** 2)
            return best_j
        return None

    return snap


def route_to_sumo_edges(route_names, graph, jpos, junc_ids, vehicle_counts=None, snap=None,
                        seg_geom=None, coef=None, edge_shapes=None):
    """Converte l'INTERO piano ENHSP (sequenza di nodi PDDL) in una lista di
    edge SUMO collegati, cosi' SUMO percorre le stesse strade scelte dal
    planner invece di un Dijkstra generico start->goal sulla rete SUMO.
    Ritorna (all_edges, all_weights): 'weights' assegna a ciascun edge la
    somma dei vehicle-count degli hop originali del piano "contratti" in
    quell'edge (vedi sotto), 0 se vehicle_counts non e' passato.

    Molti nodi PDDL "intermedi" sono punti OSM di passaggio che SUMO ha
    semplificato (non sono junction separate): vengono saltati mantenendo
    l'ordine. Tra le junction SUMO realmente mappate che restano si fa
    Dijkstra segmento per segmento e si concatenano i risultati: il percorso
    finale e' sempre una sequenza di edge collegati.

    Il piano ENHSP e i vehicle-count usano la STESSA granularita' (nodi OSM
    del grafo contratto in webapp/osm_graph.py), ma piu' hop del piano
    possono "contrarsi" su un'unica coppia di junction SUMO (quando un nodo
    intermedio non e' mappabile): la congestione di quegli hop viene sommata
    e assegnata per intero a tutti gli edge SUMO del segmento risultante."""
    vehicle_counts = vehicle_counts or {}
    if snap is None:
        snap = lambda name: pddl_name_to_junction(name, junc_ids)
    # ANCORE = solo i nodi che mappano per ID a una junction SUMO: sono punti
    # sicuri del percorso. I nodi intermedi che NON mappano (semplificati da
    # netconvert) NON vengono forzati sulla "junction piu' vicina" — farlo
    # creava piccoli giri a U quando quella junction e' leggermente fuori
    # percorso — ma la loro geometria reale viene comunque usata per riempire
    # il buco fra due ancore (vedi _geo_dijkstra sotto). Lo snap geografico
    # resta solo per START e GOAL, che servono come estremi anche se non
    # mappano.
    n_route = len(route_names)
    junctions = []
    anchor_idx = []
    for idx, name in enumerate(route_names):
        j = pddl_name_to_junction(name, junc_ids)
        if not j and (idx == 0 or idx == n_route - 1):
            j = snap(name)
        if j and (not junctions or j != junctions[-1]):
            junctions.append(j)
            anchor_idx.append(idx)
    if len(junctions) < 2:
        return None, None
    geo_ok = bool(seg_geom and coef is not None and edge_shapes)
    all_edges = []
    all_weights = []
    for i in range(len(junctions) - 1):
        a, b = junctions[i], junctions[i+1]
        # Se le due junction sono collegate DIRETTAMENTE, quell'arco e' la
        # strada scelta dal piano: usalo tale e quale. Se invece c'e' un buco
        # (un nodo intermedio del piano non e' una junction SUMO) lo si riempie
        # col cammino che ADERISCE alla geometria reale di quel tratto
        # (_geo_dijkstra): cosi' fra due parallele si prende quella giusta,
        # non una scorciatoia diversa dal percorso della webapp. Solo se manca
        # la geometria si ripiega sul Dijkstra a lunghezza minima.
        direct = sorted((ln, eid) for (to, eid, ln) in graph.get(a, []) if to == b)
        if direct:
            seg = [direct[0][1]]
        else:
            seg = None
            if geo_ok:
                G = []
                for k in range(anchor_idx[i], anchor_idx[i+1]):
                    if k < len(seg_geom):
                        for p in (seg_geom[k] or []):
                            G.append(_lonlat_to_xy(float(p[0]), float(p[1]), coef))
                if len(G) >= 2:
                    seg = _geo_dijkstra(graph, a, b, G, edge_shapes)
            if not seg:
                seg = dijkstra(graph, a, b)
        if not seg:
            return None, None
        seg_weight = sum(
            vehicle_counts.get((route_names[k], route_names[k+1]), 0)
            for k in range(anchor_idx[i], anchor_idx[i+1]))
        all_edges.extend(seg)
        all_weights.extend([seg_weight] * len(seg))
    return (all_edges, all_weights) if all_edges else (None, None)

def _reverse_id(eid):
    """L'id dell'edge SUMO che percorre la STESSA strada in direzione
    opposta. In queste net (esportate da OSM via netconvert) le due
    direzioni di una via a doppio senso sono quasi sempre lo stesso id a
    meno del segno ('42902482' / '-42902482'): qualunque logica che sceglie
    archi adiacenti in un grafo deve escludere l'id "opposto" dell'arco
    appena scelto sul lato adiacente, altrimenti si ottiene un'inversione a
    U immediata invece di un attraversamento (bug reale trovato e corretto
    in generate_crossing_traffic — vedi 5_traffico_sfondo_sumo.md §14)."""
    return eid[1:] if eid.startswith('-') else '-' + eid


def _templates_and_repeat(n_wanted, max_templates):
    """Da un numero di veicoli 'voluto' (gia' scalato secondo TRAFFIC_SCALE
    e limitato da un tetto assoluto) deriva (n_templates, repeat):
    n_templates rotte DISTINTE da generare, ciascuna ripetuta 'repeat' volte
    nel tempo con un <flow> invece che con un singolo <vehicle> (vedi
    6_proposte_realismo_traffico.md, punto 2). Senza questo, ogni rotta
    generata sarebbe un veicolo unico che parte quasi subito e non si
    ripresenta mai piu': con simulazioni lunghe (sim_end anche 1000+s) il
    traffico di sfondo si dirada nella seconda meta' perche' i veicoli gia'
    inseriti completano la rotta e vengono rimossi senza sostituti. Con
    meno template ripetuti nel tempo, lo stesso budget di veicoli resta
    presente per tutta la durata della simulazione invece di esaurirsi nei
    primi minuti."""
    if n_wanted <= 0:
        return 0, 0
    n_templates = min(max_templates, n_wanted)
    repeat = max(1, round(n_wanted / n_templates))
    return n_templates, repeat


# Bonus di densita' per tipo di strada OSM (vedi build_sumo_graph/edge_type
# in sumo_common.py): stesse chiavi di CONGESTION_DELAY_BY_HIGHWAY in
# webapp/osm_graph.py (il prefisso 'highway.' scritto da netconvert
# nell'attributo 'type' del net.xml viene gia' tolto in build_sumo_graph),
# cosi' le due tabelle restano coerenti senza doverle tradurre a runtime —
# vedi 6_proposte_realismo_traffico.md, punto 3 e incoerenza A. Sono pesi
# relativi (non secondi di ritardo come CONGESTION_DELAY_BY_HIGHWAY): una
# primaria pesa piu' di una residenziale a prescindere dal vehicle-count
# del PDDL, che resta comunque il segnale primario (vedi congestion_bonus
# in generate_background_traffic).
ROAD_TYPE_WEIGHT = {
    "primary": 2.0, "primary_link": 1.5,
    "secondary": 1.2, "secondary_link": 1.0,
    "tertiary": 0.6, "tertiary_link": 0.4,
    "residential": 0.3, "living_street": 0.1,
    "motorway": 0.0, "motorway_link": 0.0,
    "trunk": 0.0, "trunk_link": 0.0, "unclassified": 0.0,
}


def generate_background_traffic(ego_edges, edge_weights=None, edge_type=None, seed=42, scale=1.0,
                                  base_vehicles=28, max_vehicles=170, max_templates=28,
                                  min_span_frac=0.45, max_span_frac=1.0,
                                  congestion_bonus=0.6, road_type_bonus=0.5):
    """Genera veicoli di sfondo le cui rotte sono finestre CONTIGUE del
    percorso REALMENTE seguito dal veicolo ENHSP (ego_edges), invece di
    tragitti punto-a-punto sparsi su tutta la zona come nella versione
    precedente (che usava le coppie (a,b) con dati di congestione del PDDL,
    calcolate pero' su un sottografo dell'INTERA zona): la maggior parte di
    quei veicoli finiva su strade mai inquadrate, perche' sia la vista dal
    vivo sia il video seguono/sono centrati sul veicolo ENHSP (tracking,
    vedi _capture_frames_traci) — vedi 5_traffico_sfondo_sumo.md §13.

    Concentrare il traffico sulle stesse strade della macchina rossa ottiene
    due cose insieme: (a) nessun veicolo sprecato fuori dall'inquadratura ->
    simulazione piu' leggera a parita' di traffico visibile; (b) auto di
    sfondo che percorrono tratti LUNGHI (non un singolo arco isolato)
    sovrapposti al percorso reale -> sembra traffico che condivide la
    strada con l'ego, invece di comparire/sparire su un arco isolato.

    'edge_weights' (edge_id -> vehicle-count dal PDDL, vedi
    parse_vehicle_counts/route_to_sumo_edges) ed 'edge_type' (edge_id ->
    tipo strada OSM, vedi build_sumo_graph) pesano insieme quali finestre
    vengono scelte: ogni arco parte da un peso base 1 (resta comunque
    coperto anche se il PDDL non lo considera congestionato e non e' una
    strada "importante") piu' un bonus proporzionale al suo vehicle-count
    PIU' un bonus per il suo tipo di strada (una primaria pesa piu' di una
    residenziale a prescindere dal campione di 10 O/D del PDDL), cosi' le
    finestre scelte piu' spesso sono coerenti con ENTRAMBI i segnali — §13,
    §14, e punto 3/incoerenza A di 6_proposte_realismo_traffico.md.

    Il numero di veicoli scala con 'scale' (il livello di traffico scelto
    dalla webapp, vedi TRAFFIC_SCALE / --traffic) e con la lunghezza del
    percorso (route piu' lunghe hanno bisogno di piu' auto per sembrare
    trafficate lungo tutta la loro estensione), con un tetto assoluto
    (max_vehicles) per non ingolfare la GUI/il video anche al livello
    'Very high'. Ritorna (routes, repeat): vedi _templates_and_repeat —
    'routes' e' una lista di al piu' 'max_templates' rotte DISTINTE,
    'repeat' quante volte ciascuna va ripetuta nel tempo (punto 2)."""
    rng = random.Random(seed)
    n_edges = len(ego_edges)
    if n_edges < 2 or scale <= 0:
        return [], 0
    edge_weights = edge_weights or {}
    edge_type = edge_type or {}
    w = [1.0 + congestion_bonus * edge_weights.get(e, 0)
             + road_type_bonus * ROAD_TYPE_WEIGHT.get(edge_type.get(e, ''), 0)
         for e in ego_edges]
    prefix = [0.0]
    for wi in w:
        prefix.append(prefix[-1] + wi)

    length_factor = max(1.0, n_edges / 10.0)
    n_vehicles = min(max_vehicles, round(base_vehicles * scale * length_factor))
    n_templates, repeat = _templates_and_repeat(n_vehicles, max_templates)
    routes = []
    for _ in range(n_templates):
        span = round(n_edges * rng.uniform(min_span_frac, max_span_frac))
        span = max(2, min(span, n_edges))
        starts = list(range(0, n_edges - span + 1))
        weights = [prefix[s + span] - prefix[s] for s in starts]
        start_idx = rng.choices(starts, weights=weights, k=1)[0]
        routes.append(ego_edges[start_idx:start_idx + span])
    return routes, repeat


def generate_crossing_traffic(ego_edges, path_junctions, graph, edge_endpoints, connected_pairs,
                                seed=44, scale=1.0, every=3, max_extra_hops=2,
                                max_vehicles=36, max_templates=14):
    """Veicoli che ATTRAVERSANO il percorso dell'ego in una singola junction
    (una traversa/incrocio) senza mai percorrerne un arco: entrano da un arco
    diverso, passano per una junction del percorso rosso ed escono su un
    altro arco diverso. E' il "percorso che interseca quello della macchina
    rossa" richiesto (a differenza di generate_background_traffic, che
    invece SEGUE il percorso rosso) — vedi 5_traffico_sfondo_sumo.md §14.
    'every' sceglie una junction del percorso ogni N (non tutte, altrimenti
    il traffico incrociante diventerebbe piu' fitto di quello principale);
    'max_extra_hops' estende la rotta di qualche arco su entrambi i lati
    quando possibile, cosi' l'auto non compare/sparisce esattamente
    sull'incrocio. Vedi _reverse_id per il bug di inversione a U gia'
    corretto.

    'connected_pairs' (da build_sumo_graph, vedi il docstring li') valida
    ogni transizione arco->arco: 'graph' dice solo che due edge condividono
    una junction, non che SUMO permetta davvero quel passaggio (bug fatale
    osservato: "no connection between edge X and edge Y", la simulazione
    si ferma subito) — ogni candidato in_eid->out_eid e ogni estensione va
    scartato se la coppia non e' in connected_pairs. Ritorna (routes,
    repeat), vedi _templates_and_repeat."""
    rng = random.Random(seed)
    n_vehicles = min(max_vehicles, round(10 * scale))
    if n_vehicles <= 0 or len(path_junctions) < 3 or not graph:
        return [], 0
    ego_set = set(ego_edges)
    reverse = defaultdict(list)  # junction_to -> [(junction_from, edge_id), ...]
    for u, neighbors in graph.items():
        for v, eid, _length in neighbors:
            reverse[v].append((u, eid))

    n_templates, repeat = _templates_and_repeat(n_vehicles, max_templates)
    candidates = list(range(1, len(path_junctions) - 1, every)) or [len(path_junctions) // 2]
    routes = []
    tries = 0
    while len(routes) < n_templates and tries < n_templates * 8:
        tries += 1
        J = path_junctions[rng.choice(candidates)]
        incoming = [(u, eid) for (u, eid) in reverse.get(J, []) if eid not in ego_set]
        outgoing = [(v, eid) for (v, eid, _l) in graph.get(J, []) if eid not in ego_set]
        if not incoming or not outgoing:
            continue
        in_u, in_eid = rng.choice(incoming)
        out_candidates = [(v, eid) for (v, eid) in outgoing
                           if eid != _reverse_id(in_eid) and (in_eid, eid) in connected_pairs]
        if not out_candidates:
            continue
        out_v, out_eid = rng.choice(out_candidates)
        route = [in_eid, out_eid]
        cur = in_u
        for _ in range(rng.randint(0, max_extra_hops)):
            opts = [(u, eid) for (u, eid) in reverse.get(cur, [])
                    if eid not in ego_set and eid not in route and eid != _reverse_id(route[0])
                    and (eid, route[0]) in connected_pairs]
            if not opts:
                break
            u, eid = rng.choice(opts)
            route.insert(0, eid)
            cur = u
        cur = out_v
        for _ in range(rng.randint(0, max_extra_hops)):
            opts = [(v, eid) for (v, eid, _l) in graph.get(cur, [])
                    if eid not in ego_set and eid not in route and eid != _reverse_id(route[-1])
                    and (route[-1], eid) in connected_pairs]
            if not opts:
                break
            v, eid = rng.choice(opts)
            route.append(eid)
            cur = v
        routes.append(route)
    return routes, repeat


def _dist(p, q):
    return math.hypot(p[0] - q[0], p[1] - q[1])


def generate_parallel_traffic(ego_edges, path_junctions, graph, jpos, edge_endpoints, connected_pairs,
                                seed=45, scale=1.0, corridor_radius=180.0,
                                max_vehicles=32, max_hops=40, max_templates=14):
    """Veicoli su strade DIVERSE da quella dell'ego ma che corrono nello
    stesso corridoio (entro corridor_radius metri dal percorso rosso): "auto
    che viaggiano parallelamente... su altre vie/traverse" — vedi
    5_traffico_sfondo_sumo.md §14. A differenza di generate_background_traffic
    (finestre dell'ego stesso), qui la rotta e' un Dijkstra tra due junction
    VICINE al percorso ma non su di esso; viene scartata se si allontana
    troppo dal corridoio o ricalca quasi solo l'ego (altrimenti sarebbe
    indistinguibile dal traffico "sovrapposto" o percorrerebbe mezza citta').

    dijkstra() lavora a livello di junction: due edge consecutivi nel
    risultato condividono sempre una junction, ma questo NON garantisce che
    esista una <connection> reale in quella junction (stesso problema di
    generate_crossing_traffic, vedi 'connected_pairs' in build_sumo_graph)
    — la rotta va quindi validata arco-per-arco anche se e' "un cammino
    minimo" sul grafo. Ritorna (routes, repeat), vedi
    _templates_and_repeat."""
    rng = random.Random(seed)
    n_vehicles = min(max_vehicles, round(9 * scale))
    if n_vehicles <= 0 or not graph or not jpos:
        return [], 0
    path_pts = [jpos[j] for j in path_junctions if j in jpos]
    if len(path_pts) < 2:
        return [], 0

    def nearest_idx(pt):
        return min(range(len(path_pts)), key=lambda i: _dist(pt, path_pts[i]))

    ego_junc_set = set(path_junctions)
    ego_set = set(ego_edges)
    side_by_pos = defaultdict(list)
    for j, pt in jpos.items():
        if j in ego_junc_set:
            continue
        i = nearest_idx(pt)
        if _dist(pt, path_pts[i]) <= corridor_radius:
            side_by_pos[i].append(j)

    positions = sorted(side_by_pos)
    if len(positions) < 2:
        return [], 0

    n_templates, repeat = _templates_and_repeat(n_vehicles, max_templates)
    routes = []
    tries = 0
    while len(routes) < n_templates and tries < n_templates * 10:
        tries += 1
        i0 = rng.choice(positions)
        min_gap = max(2, len(path_pts) // 6)
        far_positions = [p for p in positions if abs(p - i0) >= min_gap]
        if not far_positions:
            continue
        i1 = rng.choice(far_positions)
        start_j = rng.choice(side_by_pos[i0])
        end_j = rng.choice(side_by_pos[i1])
        if start_j == end_j:
            continue
        seg = dijkstra(graph, start_j, end_j)
        if not seg or len(seg) > max_hops:
            continue
        if any((a, b) not in connected_pairs for a, b in zip(seg, seg[1:])):
            continue
        overlap = sum(1 for e in seg if e in ego_set)
        if overlap > len(seg) * 0.5:
            continue
        ok = True
        for e in seg:
            fr, to = edge_endpoints.get(e, (None, None))
            if fr not in jpos or to not in jpos:
                ok = False
                break
            mid = ((jpos[fr][0] + jpos[to][0]) / 2, (jpos[fr][1] + jpos[to][1]) / 2)
            if _dist(mid, path_pts[nearest_idx(mid)]) > corridor_radius * 1.6:
                ok = False
                break
        if not ok:
            continue
        routes.append(seg)
    return routes, repeat


def generate_wander_traffic(path_junctions, graph, jpos, connected_pairs, seed=46, scale=1.0,
                             corridor_radius=250.0, min_hops=4, max_hops=14,
                             max_vehicles=24, max_templates=12):
    """Traffico 'vagante': random-walk sulla rete a partire da una junction
    del percorso ego, scegliendo ad ogni passo un arco in uscita a caso (mai
    l'esatto opposto di quello appena percorso, vedi _reverse_id, e sempre
    con una <connection> reale rispetto all'arco precedente, vedi
    'connected_pairs' in build_sumo_graph — altrimenti SUMO si rifiuta di
    avviare la simulazione con un errore fatale, non solo un warning). E'
    la versione LEGGERA della proposta 4 di 6_proposte_realismo_traffico.md
    (svolte non deterministiche agli incroci): niente probabilita' di
    svolta calibrate ne' un binario esterno (jtrrouter, valutato ma scartato
    per questa prima implementazione — vedi il documento), solo un numero
    di passi scelto a caso tra min_hops e max_hops come criterio di
    terminazione esplicito. Necessario perche' le zone (piccola/media/
    grande) sono sotto-aree ritagliate di una citta', non l'intera rete: un
    random-walk senza limite di passi puo' rimbalzare indefinitamente sulle
    stesse strade invece di attraversarle e uscirne (rischio segnalato nel
    documento). Il walk si ferma anche appena esce da corridor_radius metri
    dal percorso ego, per restare comunque nella zona inquadrata dalla
    telecamera che segue l'ego. Ritorna (routes, repeat), vedi
    _templates_and_repeat."""
    rng = random.Random(seed)
    n_vehicles = min(max_vehicles, round(8 * scale))
    if n_vehicles <= 0 or not graph or not jpos or len(path_junctions) < 2:
        return [], 0
    path_pts = [jpos[j] for j in path_junctions if j in jpos]
    if not path_pts:
        return [], 0

    def near_corridor(j):
        pt = jpos.get(j)
        if not pt:
            return False
        return min(_dist(pt, p) for p in path_pts) <= corridor_radius

    starts = [j for j in path_junctions if graph.get(j)] or list(path_junctions)
    n_templates, repeat = _templates_and_repeat(n_vehicles, max_templates)
    routes = []
    tries = 0
    while len(routes) < n_templates and tries < n_templates * 8:
        tries += 1
        cur = rng.choice(starts)
        n_hops = rng.randint(min_hops, max_hops)
        route = []
        last_eid = None
        for _ in range(n_hops):
            opts = [(v, eid) for (v, eid, _l) in graph.get(cur, [])
                    if last_eid is None or (eid != _reverse_id(last_eid)
                                             and (last_eid, eid) in connected_pairs)]
            if not opts:
                break
            v, eid = rng.choice(opts)
            route.append(eid)
            last_eid = eid
            cur = v
            if not near_corridor(cur):
                break
        if len(route) >= 2:
            routes.append(route)
    return routes, repeat


def compute_edges_from_pddl(pddl_path, net_path):
    """Legge start/goal (e il piano completo, se disponibile) dal PDDL,
    calcola gli edge SUMO corrispondenti, ritorna (edges_str, cfg). cfg
    include anche 'total_length' del percorso, usato sia per la durata della
    simulazione sia per dimensionare il traffico di sfondo (vedi
    generate_background_traffic), e la geometria del percorso (graph, jpos,
    edge_endpoints, path_junctions, edge_weights, edge_type) usata dal traffico
    incrociante/parallelo (generate_crossing_traffic,
    generate_parallel_traffic) — vedi 5_traffico_sfondo_sumo.md §14."""
    text = open(pddl_path).read()
    vehicle_counts = parse_vehicle_counts(text)
    m_start = re.search(r'\(at\s+([A-Za-z0-9_]+)\)', text)
    m_goal  = re.search(r':goal\s+\(at\s+([A-Za-z0-9_]+)\)', text)
    if not m_start or not m_goal:
        print("[ERRORE] Impossibile trovare start/goal nel PDDL.")
        sys.exit(1)
    start_name = m_start.group(1)
    goal_name  = m_goal.group(1)

    # piano completo calcolato da ENHSP (sequenza di nodi), se la webapp
    # lo ha salvato accanto al problem.pddl
    route_names = None
    route_coords = {}
    route_seg_geom = []
    route_path = os.path.join(os.path.dirname(os.path.abspath(pddl_path)), "route_custom.json")
    if os.path.exists(route_path):
        try:
            with open(route_path) as f:
                data = json.load(f)
            r = (data.get('route')) or []
            route_coords = data.get('coords') or {}
            route_seg_geom = data.get('seg_geom') or []
            if len(r) >= 2:
                route_names = r
        except Exception:
            route_names = None
            route_coords = {}
            route_seg_geom = []

    # prova prima la net della zona indicata, poi le altre due
    # NB: lo script sta in scripts/, quindi la radice del progetto (dove c'e'
    # net_files/) e' due livelli sopra, non uno.
    base = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
    net_candidates = [net_path] + [
        os.path.join(base, "net_files", f"{z}.net.xml")
        for z in ("piccola", "media", "grande")
        if os.path.join(base, "net_files", f"{z}.net.xml") != net_path
    ]

    # Scelta della rete: si preferisce quella che mappa piu' nodi del problema.
    # Non basta pretendere che start E goal esistano come junction: netconvert
    # semplifica la rete diversamente da come build_problems.py costruisce il
    # grafo contratto, quindi un nodo PDDL puo' non esistere come junction pur
    # essendo la rete quella giusta. In quel caso si ripiega sul primo/ultimo
    # nodo del piano che risulta mappabile.
    best = None
    for candidate in net_candidates:
        if not os.path.exists(candidate):
            continue
        graph, jpos, eid_len, edge_type, connected_pairs = build_sumo_graph(candidate)
        junc_ids = set(jpos.keys())
        snap = make_snapper(junc_ids, jpos, route_names, route_coords)
        start_j = snap(start_name)
        goal_j = snap(goal_name)

        # junction del piano, in ordine, saltando i nodi non mappabili
        route_js = []
        for nm_ in (route_names or []):
            j = snap(nm_)
            if j and (not route_js or j != route_js[-1]):
                route_js.append(j)

        score = (2 if (start_j and goal_j) else 0) + len(route_js)
        cand = {
            'net': candidate, 'graph': graph, 'jpos': jpos, 'eid_len': eid_len,
            'edge_type': edge_type, 'connected_pairs': connected_pairs,
            'junc_ids': junc_ids, 'start_j': start_j, 'goal_j': goal_j,
            'route_js': route_js, 'score': score, 'snap': snap,
        }
        if best is None or score > best['score']:
            best = cand
        if start_j and goal_j:
            break  # match perfetto: inutile provare le altre reti

    if best is None or best['score'] == 0:
        print(f"[ERRORE] Junction per '{start_name}' o '{goal_name}' non trovata in nessuna net.")
        sys.exit(1)

    graph, jpos = best['graph'], best['jpos']
    eid_len, junc_ids = best['eid_len'], best['junc_ids']
    edge_type = best['edge_type']
    connected_pairs = best['connected_pairs']
    used_net = best['net']
    start_j, goal_j, route_js = best['start_j'], best['goal_j'], best['route_js']
    snap = best['snap']

    if used_net != net_path:
        print(f"  (uso net: {os.path.basename(used_net)})")

    # Fallback: se start/goal non esistono come junction, uso gli estremi
    # mappabili del piano ENHSP.
    if not start_j and route_js:
        start_j = route_js[0]
        print(f"  (start '{start_name}' non e' una junction SUMO: uso {start_j}, "
              f"primo nodo mappabile del piano)")
    if not goal_j and route_js:
        goal_j = route_js[-1]
        print(f"  (goal '{goal_name}' non e' una junction SUMO: uso {goal_j}, "
              f"ultimo nodo mappabile del piano)")

    if not start_j or not goal_j:
        print(f"[ERRORE] Impossibile mappare start/goal su una junction SUMO "
              f"(nessun piano disponibile come ripiego).")
        print(f"         Suggerimento: risolvi prima con la webapp, che salva "
              f"route_custom.json accanto al problema.")
        sys.exit(1)

    print(f"  START: {start_name} → {start_j}")
    print(f"  GOAL : {goal_name} → {goal_j}")

    edges = None
    weights = None
    # Percorso SUMO = piano ENHSP, seguito incrocio per incrocio. Gli archi
    # diretti sono le strade scelte dal planner; i buchi (nodi semplificati da
    # netconvert) si riempiono col cammino che ADERISCE alla geometria reale
    # del tratto (map-matching LOCALE), cosi' SUMO percorre esattamente le
    # stesse vie marcate sulla webapp senza le deviazioni di un map-matching
    # globale.
    if route_names:
        coef = fit_lonlat_affine(route_names, route_coords, junc_ids, jpos)
        edge_shapes = load_edge_shapes(used_net) if route_seg_geom else None
        edges, weights = route_to_sumo_edges(
            route_names, graph, jpos, junc_ids, vehicle_counts, snap=snap,
            seg_geom=route_seg_geom, coef=coef, edge_shapes=edge_shapes)
        if edges:
            print(f"  (percorso SUMO = piano ENHSP, {len(route_names)} nodi, "
                  f"{len(edges)} edge)")
        else:
            print("  (piano non mappabile passo-passo, uso Dijkstra start→goal)")

    if not edges:
        edges = dijkstra(graph, start_j, goal_j)
        weights = None
    if not edges:
        print(f"[ERRORE] Nessun percorso SUMO da {start_j} a {goal_j}")
        sys.exit(1)

    total_length = sum(eid_len.get(e, 0.0) for e in edges)

    # Geometria del percorso, per il traffico incrociante/parallelo (vedi
    # generate_crossing_traffic, generate_parallel_traffic, §14): edge_weights
    # e' 0 per ogni arco quando 'weights' non e' disponibile (fallback
    # Dijkstra senza piano ENHSP), che equivale a nessuna concentrazione
    # extra su nessun arco (comportamento uniforme di prima).
    edge_endpoints = build_edge_endpoints(graph)
    edge_weights = {}
    for e, wgt in zip(edges, weights or [0] * len(edges)):
        edge_weights[e] = max(edge_weights.get(e, 0), wgt)
    path_junctions = []
    for e in edges:
        fr, to = edge_endpoints.get(e, (None, None))
        if fr is None:
            continue
        if not path_junctions:
            path_junctions.append(fr)
        path_junctions.append(to)

    # centra la vista sull'inizio reale del primo edge percorso (col
    # map-matching la partenza puo' non coincidere con la junction 'start_j')
    fr0 = edge_endpoints.get(edges[0], (None, None))[0]
    sp = jpos.get(fr0) or jpos[start_j]
    return ' '.join(edges), {
        'start': start_name, 'goal': goal_name,
        'x': sp[0], 'y': sp[1],
        'net': used_net,
        'total_length': total_length,
        'graph': graph, 'jpos': jpos,
        'edge_endpoints': edge_endpoints,
        'edge_weights': edge_weights,
        'edge_type': edge_type,
        'connected_pairs': connected_pairs,
        'path_junctions': path_junctions,
    }

# ── Argomenti ────────────────────────────────────────────────
# Uso standard:  python scripts/sumo_visualize.py [piccola|media|grande]
# Uso dinamico:  python scripts/sumo_visualize.py pddl <percorso_pddl> [piccola|media|grande]
# Opzione:       --baseline   -> NON carica i semafori ottimizzati (punto 3),
#                                usa il programma originale "0" del net.xml.
# Opzione:       --traffic 0  -> disattiva il traffico di sfondo (solo il
#                                veicolo ENHSP, come prima di questa funzione).
#                                --traffic N (N>0, es. 2.5) scala il numero di
#                                veicoli di sfondo di un fattore N: permette
#                                di scegliere il "grado di congestione" dalla
#                                webapp. Il traffico di sfondo e' un mix di
#                                quattro tipi di rotta (vedi
#                                generate_background_traffic,
#                                generate_crossing_traffic,
#                                generate_parallel_traffic,
#                                generate_wander_traffic, §14 e
#                                6_proposte_realismo_traffico.md): veicoli che
#                                percorrono finestre del percorso reale
#                                dell'ENHSP (pesate secondo il vehicle-count
#                                del PDDL E il tipo di strada OSM), veicoli
#                                che lo attraversano a un incrocio senza
#                                seguirlo, veicoli su altre vie nello stesso
#                                corridoio, e veicoli "vaganti" (random-walk
#                                a lunghezza limitata). Ogni rotta-template
#                                puo' ripetersi nel tempo (<flow>, punto 2),
#                                e i veicoli sono un mix di auto/furgone/moto
#                                (<vTypeDistribution>, punto 1). Default 1.
# Opzione:       --video      -> invece di aprire sumo-gui in modo interattivo,
#                                registra la simulazione a schermate ("snapshot")
#                                e le unisce con ffmpeg in un video mp4 (vedi
#                                generate_video). Utile per mostrare MOLTO piu'
#                                traffico di quanto sia fluido seguire dal vivo
#                                nella GUI (vedi 5_traffico_sfondo_sumo.md §11).
# Opzione:       --video-out <path> -> percorso esplicito del file mp4 di
#                                output (implica --video); default
#                                cfg_files/<zona>_video.mp4.
# Opzione:       --run-id <id> -> suffisso univoco per i file di lavoro
#                                (cfg/rou/gui, frames) di questa invocazione,
#                                cosi' chiamate ravvicinate (dal vivo e/o
#                                video) non si sovrascrivono a vicenda. Solo
#                                modalita' dinamica (pddl). Usato dalla webapp.
USE_OPTIMIZED_TLS = "--baseline" not in sys.argv
sys.argv = [a for a in sys.argv if a != "--baseline"]

USE_BACKGROUND_TRAFFIC = True
TRAFFIC_SCALE = 1.0
if "--traffic" in sys.argv:
    i = sys.argv.index("--traffic")
    val = sys.argv[i + 1] if i + 1 < len(sys.argv) else "1"
    if val in ("0", "false", "False"):
        USE_BACKGROUND_TRAFFIC = False
        TRAFFIC_SCALE = 0.0
    else:
        try:
            TRAFFIC_SCALE = float(val)
        except ValueError:
            TRAFFIC_SCALE = 1.0
        USE_BACKGROUND_TRAFFIC = TRAFFIC_SCALE > 0
    del sys.argv[i:i + 2]

VIDEO_OUT = None
if "--video-out" in sys.argv:
    i = sys.argv.index("--video-out")
    VIDEO_OUT = sys.argv[i + 1] if i + 1 < len(sys.argv) else None
    del sys.argv[i:i + 2]
VIDEO_MODE = ("--video" in sys.argv) or (VIDEO_OUT is not None)
sys.argv = [a for a in sys.argv if a != "--video"]

# --run-id <id> -> suffisso univoco per i file di lavoro (cfg/rou/gui,
# frames) di QUESTA invocazione. Necessario perche' la webapp puo' lanciare
# piu' simulazioni ravvicinate (dal vivo e/o video) sulla stessa zona: senza
# un suffisso univoco condividerebbero gli stessi file fissi e una
# invocazione potrebbe sovrascrivere gli input di un'altra ancora in corso
# (bug riprodotto e descritto in 5_traffico_sfondo_sumo.md §12).
RUN_ID = None
if "--run-id" in sys.argv:
    i = sys.argv.index("--run-id")
    RUN_ID = sys.argv[i + 1] if i + 1 < len(sys.argv) else None
    del sys.argv[i:i + 2]

# Fotogrammi ~costanti indipendentemente dalla durata simulata (sim_end):
# l'intervallo tra due snapshot si allarga per simulazioni piu' lunghe, cosi'
# il tempo di generazione e la lunghezza del video restano prevedibili anche
# per percorsi molto piu' lunghi del solito (vedi 5_traffico_sfondo_sumo.md §11).
VIDEO_TARGET_FRAMES = 200
VIDEO_FPS = 15

zona = sys.argv[1] if len(sys.argv) > 1 else "piccola"
dynamic_pddl = None

if zona == "pddl":
    # modalità dinamica: legge il PDDL e calcola il percorso
    if len(sys.argv) < 3:
        print("Uso dinamico: python scripts/sumo_visualize.py pddl <file.pddl> [piccola|media|grande]")
        sys.exit(1)
    dynamic_pddl = sys.argv[2]
    zona = sys.argv[3] if len(sys.argv) > 3 else "piccola"

if zona not in ("piccola", "media", "grande"):
    print("Uso: python scripts/sumo_visualize.py [piccola|media|grande]")
    print("     python scripts/sumo_visualize.py pddl <file.pddl> [piccola|media|grande]")
    sys.exit(1)

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # radice del progetto
OUT  = os.path.join(BASE, "cfg_files")
os.makedirs(OUT, exist_ok=True)

CONFIGS = {
    "piccola": {
        "net": os.path.join(BASE, "net_files", "piccola.net.xml"),
        # Dijkstra da 411193756 (Ormond Quay / Capel St, nodo più connesso)
        #         a 12015832633 (Aungier Street, punto più lontano raggiungibile)
        # Percorso: Ormond Quay → Capel St → Grattan Bridge → Essex Quay
        #           → Fishamble St → Lord Edward St → Cork Hill → Dame St
        #           → South Great George's St → Aungier St
        "edges": (
            "1293310466 1293310467 12854626#0 12854626#1 "
            "4396056 1288830596 1179644329 1179644328 "
            "1254511872 1254511870 1254511871 125864859 "
            "5976028#2 5976028#3 16247623#1 "
            "4396059#0 4396059#2 846644599 "
            "-317003249#3 -317003249#2 -369564011 -5826896 876578189 4919471"
        ),
        "zoom": 3000, "x": 144, "y": 535,
        "dist_m": 1520, "time_s": 182,
        "start": "Ormond Quay / Capel Street", "goal": "Aungier Street",
    },
    "media": {
        "net": os.path.join(BASE, "net_files", "media.net.xml"),
        # Dijkstra da 2876012509 (Leeson St Upper) a 8752842641 (Saint Mary's Road)
        "edges": (
            "119860706 612719563#1 283771398#1 14041596#1 8111025#0 "
            "48520199#1 -370154357 -370154355 -128521564 20834536 110407380"
        ),
        "zoom": 3000, "x": 1144, "y": 2131,
        "dist_m": 1623, "time_s": 150,
        "start": "Leeson Street Upper", "goal": "Saint Mary's Road",
    },
    "grande": {
        "net": os.path.join(BASE, "net_files", "grande.net.xml"),
        # Dijkstra da 28244374 (St Patrick's Road) a 11822804242 (Botanic Avenue)
        # Segmento per segmento seguendo il piano ENHSP
        "edges": (
            "1159857185 1159857184 "
            "-130294072#2 -130294072#1 -130294072#0 "
            "-1293323158 -4540453 -1316170357 "
            "378882695#1 378882694 4539231 -130776836 "
            "-56007691#4 -56007691#3 -56007691#2 -56007691#0"
        ),
        "zoom": 3000, "x": 246, "y": 4860,
        "dist_m": 1335, "time_s": 147,
        "start": "St Patrick's Road", "goal": "Botanic Avenue",
    },
}

cfg = CONFIGS[zona]
NET  = cfg["net"]

# ── Traffico di sfondo ──────────────────────────────────────
# Solo in modalita' dinamica (§3 di 5_traffico_sfondo_sumo.md): i preset
# statici a CLI non hanno un percorso ENHSP calcolato da cui derivare le
# rotte di sfondo (vedi generate_background_traffic, §13), quindi restano
# con il solo veicolo ENHSP come prima.
background_routes = []

# ── Modalità dinamica: sovrascrive edges/x/y dal PDDL ────────
if dynamic_pddl:
    print(f"[DINAMICO] Calcolo percorso da: {dynamic_pddl}")
    edges_str, dyn = compute_edges_from_pddl(dynamic_pddl, NET)
    cfg = dict(cfg)  # copia per non modificare l'originale
    cfg['edges'] = edges_str
    cfg['x']     = dyn['x']
    cfg['y']     = dyn['y']
    cfg['start'] = dyn['start']
    cfg['goal']  = dyn['goal']
    cfg['dist_m'] = '?'
    cfg['time_s'] = '?'
    NET = dyn['net']   # usa la net dove sono stati trovati i nodi
    zona = zona + "_custom"
    if RUN_ID:
        zona = zona + "_" + RUN_ID

    # ── Durata simulazione dinamica ────────────────────────────
    # Il veicolo "auto" ha maxSpeed=4.0 m/s: con percorsi lunghi gli 800s
    # di default non bastano e SUMO si ferma a 799 con l'auto ancora in
    # viaggio. Stimiamo il tempo di percorrenza dal percorso reale e
    # aggiungiamo un margine generoso per semafori/svolte/code.
    est_time = dyn['total_length'] / 4.0
    cfg['end'] = max(800, (math.ceil(est_time * 1.5 / 100.0) + 1) * 100)
    print(f"  (percorso {dyn['total_length']:.0f} m ~ {est_time:.0f} s a 4 m/s "
          f"→ fine simulazione impostata a {cfg['end']} s)")

    # cfg['end'] NON viene ricalcolato in base al traffico di sfondo: resta
    # ancorato solo al veicolo ENHSP (decisione presa, vedi §4.d).
    if USE_BACKGROUND_TRAFFIC:
        ego_edges_list = edges_str.split()
        overlap_routes, overlap_repeat = generate_background_traffic(
            ego_edges_list, edge_weights=dyn['edge_weights'], edge_type=dyn['edge_type'],
            scale=TRAFFIC_SCALE)
        crossing_routes, crossing_repeat = generate_crossing_traffic(
            ego_edges_list, dyn['path_junctions'], dyn['graph'], dyn['edge_endpoints'],
            dyn['connected_pairs'], scale=TRAFFIC_SCALE)
        parallel_routes, parallel_repeat = generate_parallel_traffic(
            ego_edges_list, dyn['path_junctions'], dyn['graph'], dyn['jpos'],
            dyn['edge_endpoints'], dyn['connected_pairs'], scale=TRAFFIC_SCALE)
        wander_routes, wander_repeat = generate_wander_traffic(
            dyn['path_junctions'], dyn['graph'], dyn['jpos'], dyn['connected_pairs'],
            scale=TRAFFIC_SCALE)
        # 'kind' distingue il ruolo di ciascuna rotta-TEMPLATE per l'orario/
        # la finestra di partenza (vedi sotto, sezione "File di route") e
        # per il tipo di veicolo. Ogni rotta viene ripetuta 'repeat' volte
        # nel tempo (vedi _templates_and_repeat, punto 2 di
        # 6_proposte_realismo_traffico.md) invece di essere un singolo
        # veicolo unico, cosi' il traffico di sfondo resta presente per
        # tutta sim_end invece di esaurirsi nei primi minuti (§13/§14).
        background_routes = (
            [('overlap', r, overlap_repeat) for r in overlap_routes] +
            [('cross', r, crossing_repeat) for r in crossing_routes] +
            [('parallel', r, parallel_repeat) for r in parallel_routes] +
            [('wander', r, wander_repeat) for r in wander_routes])
        n_instances = (len(overlap_routes) * overlap_repeat + len(crossing_routes) * crossing_repeat +
                       len(parallel_routes) * parallel_repeat + len(wander_routes) * wander_repeat)
        print(f"  (traffico di sfondo: {len(overlap_routes)} template sovrapposti al percorso (x{overlap_repeat}) + "
              f"{len(crossing_routes)} incrocianti (x{crossing_repeat}) + "
              f"{len(parallel_routes)} paralleli (x{parallel_repeat}) + "
              f"{len(wander_routes)} vaganti (x{wander_repeat}) "
              f"= {len(background_routes)} rotte, ~{n_instances} veicoli nel tempo, livello x{TRAFFIC_SCALE:g})"
              if background_routes else "  (traffico di sfondo: nessun veicolo generato)")

# ── File di route ─────────────────────────────────────────────
# L'auto rossa parte con un ritardo (EGO_DEPART) solo se c'e' traffico di
# sfondo da mostrare: cosi' le prime auto di sfondo gia' staccate (vedi
# stagger sotto) sono gia' in strada quando parte la rossa ("pre-spawn"
# richiesto dall'utente), invece di comparire dal nulla insieme a lei.
# Senza sfondo il ritardo non ha senso (si vedrebbe solo una strada vuota
# per qualche secondo), si mantiene il vecchio depart="1".
ego_depart = 12.0 if background_routes else 1.0

ROU_PATH = os.path.join(OUT, f"{zona}_piano.rou.xml")
route_lines = ['<?xml version="1.0" encoding="UTF-8"?>', '<routes>',
               '    <vType id="auto" accel="1.5" decel="3.0" sigma="0.0"',
               '           length="4.5" maxSpeed="4.0" color="1,0,0"',
               '           width="2.0" shape="passenger"/>']
if background_routes:
    # Mix di tipi di veicolo invece di un solo vType "traffic" clonato:
    # <vTypeDistribution> fa scegliere a SUMO uno dei tre vType per ogni
    # veicolo/flow che referenzia "traffic_mix" (nessuna logica aggiuntiva
    # lato Python) — vedi 6_proposte_realismo_traffico.md, punto 1.
    # Probabilita' alzate per van/moto (25%/5% -> 30%/15%): con le
    # probabilita' originali, su un campione di ~20-40 veicoli attivi la
    # moto (5%) restava quasi sempre a 1-2 esemplari, poco percepibile.
    # Anche i COLORI erano un problema di percepibilita' a se': van grigio
    # e moto quasi nera si confondevano con le strade (linee nere su sfondo
    # verde nello stile della mappa) anche quando presenti — ora blu/ciano,
    # ben distinti da giallo (auto), rosso (ego), nero (strade) e verde
    # (sfondo). Il colore resta fisso PER TIPO (non per singolo veicolo,
    # limite noto e accettato nel documento). speedFactor (distribuzione
    # normale troncata, campionata una volta per veicolo all'inserimento)
    # resta su ciascun tipo per la variazione di velocita' gia'
    # implementata (§14): ogni auto mantiene per tutto il tragitto un suo
    # fattore (chi piu' lento, chi piu' veloce), invece di viaggiare tutte
    # alla stessa andatura.
    route_lines.append('    <vTypeDistribution id="traffic_mix">')
    route_lines.append('        <vType id="traffic_car" accel="2.6" decel="4.5" sigma="0.4"')
    route_lines.append('               length="4.5" maxSpeed="13.89" color="1,0.8,0"')
    route_lines.append('               speedFactor="normc(1,0.2,0.7,1.4)" probability="0.55"/>')
    route_lines.append('        <vType id="traffic_van" accel="1.8" decel="3.5" sigma="0.3"')
    route_lines.append('               length="6.5" width="2.1" maxSpeed="11.0" color="0.2,0.45,0.95"')
    route_lines.append('               speedFactor="normc(0.9,0.15,0.7,1.1)" probability="0.30"/>')
    route_lines.append('        <vType id="traffic_moto" accel="3.2" decel="5.0" sigma="0.5"')
    route_lines.append('               length="2.2" width="0.9" maxSpeed="15.0" color="0,0.9,0.9"')
    route_lines.append('               speedFactor="normc(1.1,0.25,0.7,1.5)" probability="0.15"/>')
    route_lines.append('    </vTypeDistribution>')
route_lines.append(f'    <route id="piano_enhsp" edges="{cfg["edges"]}"/>')

# SUMO richiede che <vehicle>/<flow> compaiano nel .rou.xml in ordine NON
# DECRESCENTE di partenza (depart per i <vehicle>, begin per i <flow>): se
# non lo sono, non e' un errore fatale, ma SUMO scarta silenziosamente ogni
# elemento "fuori ordine" stampando "Route file should be sorted by
# departure time, ignoring 'bgN'" (bug reale trovato durante la verifica di
# questa implementazione: l'ego con depart=12 veniva scritto PRIMA di
# molte auto di sfondo con depart/begin minore, e i quattro generatori
# scrivevano un blocco dopo l'altro con depart calcolati indipendentemente,
# quindi non globalmente ordinati — la maggior parte del traffico veniva
# silenziosamente scartata senza nessun errore visibile, ne' in modalita'
# video (traci.start non cattura lo stderr del processo sumo-gui che
# lancia) ne' dal vivo). Si raccolgono quindi tutti gli elementi
# <vehicle>/<flow> (ego incluso) con la loro chiave di partenza, e si
# scrivono ordinati per quella chiave invece che nell'ordine di
# generazione.
entries = []  # [(sort_key, [righe xml]), ...]
entries.append((ego_depart, [
    '    <vehicle id="veicolo_enhsp" type="auto" route="piano_enhsp"',
    f'             depart="{ego_depart:g}" departSpeed="0"/>',
]))

depart_rng = random.Random(43)  # seed indipendente dal campionamento delle rotte
sim_end_est = cfg.get('end', 800)
overlap_i = parallel_i = 0
for i, (kind, edges, repeat) in enumerate(background_routes):
    edges_attr = " ".join(edges)
    if repeat > 1:
        # Piu' di un veicolo per questa rotta-template: <flow> invece di un
        # singolo <vehicle> ripetuto a mano, cosi' il traffico resta
        # presente per tutta la durata della simulazione invece di
        # esaurirsi nei primi minuti (vedi generate_background_traffic/
        # _templates_and_repeat, punto 2 di 6_proposte_realismo_traffico.md).
        # "number" (conteggio totale fisso) invece di "period"/"probability"
        # mantiene lo stesso tetto assoluto gia' validato (max_vehicles) —
        # SUMO distribuisce da solo i "repeat" inserimenti fra begin ed end.
        entries.append((0.0, [
            f'    <route id="r_bg{i}" edges="{edges_attr}"/>',
            f'    <flow id="bg{i}" type="traffic_mix" route="r_bg{i}" '
            f'begin="0" end="{sim_end_est:g}" number="{repeat}" '
            f'departPos="random" departSpeed="max"/>',
        ]))
        continue
    # repeat <= 1 (scale bassa: pochi veicoli totali, sotto la soglia per
    # cui vale la pena ripetere un template): resta un singolo <vehicle>
    # con lo stesso schema di partenza per-kind di prima (§13/§14) —
    # - overlap: scaglionate dall'inizio -> le prime precedono ego_depart,
    #   il "pre-spawn lungo il percorso" richiesto;
    # - parallel: scaglionate a parte (stagger piu' largo, rotte piu'
    #   lunghe su altre vie) cosi' non si accumulano tutte all'inizio;
    # - cross/wander: orario casuale su tutta la prima meta' della
    #   simulazione, invece che solo all'inizio.
    if kind == 'overlap':
        depart = round(overlap_i * 2.5 + depart_rng.uniform(0, 1.5), 1)
        overlap_i += 1
    elif kind == 'parallel':
        depart = round(parallel_i * 3.0 + depart_rng.uniform(0, 2.0), 1)
        parallel_i += 1
    else:  # 'cross' o 'wander'
        depart = round(depart_rng.uniform(0, min(300.0, sim_end_est * 0.5)), 1)
    # departPos/departSpeed "random"/"max" invece di partire ferme dal primo
    # metro dell'arco: sembrano auto gia' in marcia trovate lungo la strada,
    # non veicoli che si materializzano fermi accanto all'ego (altra parte
    # della richiesta di "pre-spawn").
    entries.append((depart, [
        f'    <vehicle id="bg{i}" type="traffic_mix" depart="{depart}" '
        f'departPos="random" departSpeed="max">',
        f'        <route edges="{edges_attr}"/>',
        '    </vehicle>',
    ]))

entries.sort(key=lambda entry: entry[0])
for _, lines in entries:
    route_lines.extend(lines)
route_lines.append('</routes>')
with open(ROU_PATH, "w") as f:
    f.write("\n".join(route_lines) + "\n")

# ── Centra la vista sul punto dove nasce l'auto ───────────────
# Il veicolo ENHSP parte all'inizio del PRIMO arco della rotta: leggo la
# prima coordinata della sua corsia dal net.xml e ci centro il viewport,
# cosi' la finestra si apre sempre inquadrando l'auto — in ogni zona e sia
# in modalita' standard sia dinamica (dove il preset x/y fisso spesso non
# cadeva sul punto di partenza reale).
def _first_edge_spawn_xy(net_path, first_edge_id):
    try:
        root = ET.parse(net_path).getroot()
        for e in root.findall('edge'):
            if e.get('id') == first_edge_id:
                lane = e.find('lane')
                if lane is not None and lane.get('shape'):
                    x, y = lane.get('shape').split()[0].split(',')[:2]
                    return float(x), float(y)
    except Exception:
        pass
    return None

_first_edge = (cfg.get('edges', '').split() or [None])[0]
_spawn = _first_edge_spawn_xy(NET, _first_edge) if _first_edge else None
if _spawn:
    cfg['x'], cfg['y'] = _spawn
    cfg['zoom'] = cfg.get('zoom', 3000) or 3000
    print(f"  Vista    : centrata sulla partenza (x={_spawn[0]:.0f}, y={_spawn[1]:.0f})")

# ── Impostazioni grafica ──────────────────────────────────────
GUI_PATH = os.path.join(OUT, f"gui_{zona}.xml")
sim_end = cfg.get('end', 800)

# In modalita' video il tracking (vedi piu' sotto, _capture_frames_traci) e'
# il meccanismo PRIMARIO di cattura: la telecamera segue il veicolo ENHSP
# invece di restare fissa sul preset statico. Gli elementi nativi
# <snapshot file=".." time=".."/> (cattura automatica di sumo-gui in una run
# non interattiva -S -Q, senza bisogno di TraCI) servono solo come RIPIEGO
# se il tracking fallisce (vedi 5_traffico_sfondo_sumo.md §12): per questo il
# file di gui-settings viene scritto la prima volta SENZA, e riscritto con
# gli snapshot solo se serve davvero il ripiego.
FRAMES_DIR = os.path.join(OUT, f"video_frames_{zona}")
video_interval = 1
if VIDEO_MODE:
    os.makedirs(FRAMES_DIR, exist_ok=True)
    for stale in glob.glob(os.path.join(FRAMES_DIR, "frame_*.png")):
        os.remove(stale)
    video_interval = max(1, round(sim_end / VIDEO_TARGET_FRAMES))

# delay=200ms serve solo a rendere gradevole la riproduzione DAL VIVO: in
# modalita' video sumo-gui rispetta questo ritardo anche in modo non
# interattivo (-S -Q), rallentando artificialmente la cattura a ~5 step
# simulati/secondo reale (900s simulati -> 180s reali) e rischiando di
# superare il timeout. In VIDEO_MODE lo si azzera per catturare piu' veloce
# possibile (vedi test in 5_traffico_sfondo_sumo.md §11).
gui_delay = 0 if VIDEO_MODE else 200

def _write_gui_settings(with_snapshots):
    """Scrive/riscrive GUI_PATH. with_snapshots=True include gli elementi
    <snapshot> nativi (serve solo per il ripiego senza tracking, vedi sopra)."""
    snapshot_lines = ""
    if with_snapshots:
        parts = []
        t = 0
        idx = 0
        while t <= sim_end:
            fp = os.path.join(FRAMES_DIR, f"frame_{idx:04d}.png")
            parts.append(f'    <snapshot file="{fp}" time="{t}"/>')
            t += video_interval
            idx += 1
        snapshot_lines = "\n".join(parts)
    with open(GUI_PATH, "w") as f:
        f.write("""<viewsettings>
    <scheme name="real world"/>
    <delay value="{delay}"/>
    <viewport zoom="{zoom}" x="{x}" y="{y}"/>
    <vehicles vehicleMode="0" vehicleQuality="2"
              vehicleExaggeration="15" showBlinker="true"
              colorScheme="given/assigned vehicle color"/>
{snapshots}
</viewsettings>
""".format(delay=gui_delay, snapshots=snapshot_lines, **cfg))

_write_gui_settings(with_snapshots=False)

# ── Semafori ottimizzati (punto 3) ────────────────────────────
# Se esiste cfg_files/tls_<zona>.add.xml (generato da inject_signal_plan.py)
# lo carichiamo come additional-file: SUMO rende attivo il programma
# "optimized" al posto di quello di default del net.xml.
# La zona si ricava dal NET realmente usato, cosi' funziona anche in
# modalita' dinamica (dove NET puo' cambiare rispetto a quello richiesto).
net_zone = os.path.basename(NET).split(".")[0]
ADD_PATH = os.path.join(OUT, f"tls_{net_zone}.add.xml")
use_add = USE_OPTIMIZED_TLS and os.path.exists(ADD_PATH)

additional_line = ""
if use_add:
    additional_line = f'\n        <additional-files value="{os.path.abspath(ADD_PATH)}"/>'

# ── Config SUMO ───────────────────────────────────────────────
CFG_PATH = os.path.join(OUT, f"{zona}.sumocfg")
with open(CFG_PATH, "w") as f:
    f.write("""<?xml version="1.0" encoding="UTF-8"?>
<configuration>
    <input>
        <net-file value="{net}"/>
        <route-files value="{rou}"/>
        <gui-settings-file value="{gui}"/>{additional}
    </input>
    <time>
        <begin value="0"/>
        <end value="{end}"/>
    </time>
</configuration>
""".format(
        net=NET,
        rou=os.path.abspath(ROU_PATH),
        gui=os.path.abspath(GUI_PATH),
        additional=additional_line,
        end=cfg.get('end', 800),
    ))

# ── Trova sumo-gui ────────────────────────────────────────────
def trova_sumo_bin(nome):
    sumo_home = os.environ.get("SUMO_HOME", "")
    candidati = []
    if sumo_home:
        candidati += [os.path.join(sumo_home, "bin", nome + ".exe"),
                      os.path.join(sumo_home, "bin", nome)]
    for base in [r"C:\Program Files (x86)\Eclipse\Sumo",
                 r"C:\Program Files\Eclipse\Sumo", r"C:\Sumo"]:
        candidati.append(os.path.join(base, "bin", nome + ".exe"))
    for c in candidati:
        if os.path.exists(c):
            return c
    try:
        subprocess.run([nome, "--version"], capture_output=True, check=True)
        return nome
    except Exception:
        return None

def trova_ffmpeg():
    try:
        subprocess.run(["ffmpeg", "-version"], capture_output=True, check=True)
        return "ffmpeg"
    except Exception:
        return None

EGO_VEHICLE_ID = "veicolo_enhsp"
VIDEO_ZOOM = 800     # vista de-zoommata: piu' isolati attorno al veicolo tracciato
                     # (calibrato empiricamente, vedi 5_traffico_sfondo_sumo.md §12)
VIDEO_TAIL_S = 15    # secondi extra catturati dopo l'arrivo del veicolo ENHSP

def _capture_frames_traci(sumo_gui_bin, cfg_path, frames_dir, sim_end_s,
                           interval_s, ego_id, zoom, tail_s):
    """Cattura i fotogrammi del video guidando sumo-gui via TraCI: la
    telecamera SEGUE il veicolo ENHSP (trackVehicle) con uno zoom piu' ampio
    del preset statico (setZoom), invece del viewport fisso del
    gui-settings, che spesso non era centrato sul percorso reale in
    modalita' dinamica. Si ferma appena il veicolo arriva a destinazione
    (+ tail_s di margine) invece di andare fino a sim_end_s: la simulazione
    da sola non si fermerebbe (continua per il traffico di sfondo), e
    fermarsi prima riduce anche il tempo di esposizione a un'eventuale
    disconnessione di sumo-gui (vedi 5_traffico_sfondo_sumo.md §12).
    Ritorna il numero di fotogrammi catturati; rilancia l'eccezione di TraCI
    se la connessione cade (il chiamante decide se ritentare o ripiegare
    sulla cattura nativa a schermate fisse)."""
    import traci
    view = "View #0"
    traci.start([sumo_gui_bin, "-c", cfg_path, "--start", "--window-size", "1280,720"])
    tracked = False
    arrived_at = None
    frame_idx = 0
    next_shot = 0
    step = 0
    try:
        while step <= sim_end_s:
            if not tracked:
                if ego_id in traci.vehicle.getIDList():
                    traci.gui.trackVehicle(view, ego_id)
                    traci.gui.setZoom(view, zoom)
                    tracked = True
            elif arrived_at is None and ego_id not in traci.vehicle.getIDList():
                arrived_at = step   # veicolo arrivato (rimosso dalla simulazione)
            if step >= next_shot:
                fp = os.path.join(frames_dir, f"frame_{frame_idx:04d}.png")
                traci.gui.screenshot(view, fp)
                frame_idx += 1
                next_shot += interval_s
            traci.simulationStep()
            step += 1
            if arrived_at is not None and step >= arrived_at + tail_s:
                break
            if traci.simulation.getMinExpectedNumber() <= 0:
                break
    finally:
        traci.close()
    return frame_idx

sumo_gui = trova_sumo_bin("sumo-gui")
if not sumo_gui:
    print("[ERRORE] sumo-gui non trovato.")
    sys.exit(1)

# ── Avvio ─────────────────────────────────────────────────────
print(f"Zona: {zona.upper()}")
print(f"  Percorso : {cfg['start']} → {cfg['goal']}")
dist_val = cfg['dist_m']
time_val = cfg['time_s']
if dist_val != '?':
    print(f"  Distanza : {dist_val} m")
    print(f"  Tempo    : {time_val} s (a 30 km/h, senza traffico)")
else:
    print(f"  (distanza/tempo calcolati da ENHSP)")
if use_add:
    print(f"  Semafori : OTTIMIZZATI (programma 'optimized' da {os.path.basename(ADD_PATH)})")
    print(f"             tasto destro sul semaforo in GUI -> torni al programma '0'")
elif USE_OPTIMIZED_TLS:
    print(f"  Semafori : originali del net.xml "
          f"(nessun {os.path.basename(ADD_PATH)}: genera con inject_signal_plan.py)")
else:
    print(f"  Semafori : originali del net.xml (--baseline)")
print()
print(f"File generati:")
print(f"  Route : {ROU_PATH}")
print(f"  Config: {CFG_PATH}")
print()

if VIDEO_MODE:
    ffmpeg_bin = trova_ffmpeg()
    if not ffmpeg_bin:
        print("[ERRORE] ffmpeg non trovato (richiesto da --video).")
        sys.exit(1)

    out_path = os.path.abspath(VIDEO_OUT or os.path.join(OUT, f"{zona}_video.mp4"))
    os.makedirs(os.path.dirname(out_path), exist_ok=True)

    print(f"Registrazione video: telecamera con tracking del veicolo, "
          f"~1 fotogramma ogni {video_interval}s → {VIDEO_FPS} fps...")

    tracking_ok = False
    for tentativo in (1, 2):
        try:
            n_catturati = _capture_frames_traci(
                sumo_gui, CFG_PATH, FRAMES_DIR, sim_end, video_interval,
                EGO_VEHICLE_ID, VIDEO_ZOOM, VIDEO_TAIL_S)
            if n_catturati > 0:
                tracking_ok = True
                break
            print(f"  (tentativo {tentativo}: nessun fotogramma catturato)")
        except Exception as e:
            print(f"  (tentativo {tentativo} di tracking via TraCI fallito: {e})")
        for stale in glob.glob(os.path.join(FRAMES_DIR, "frame_*.png")):
            os.remove(stale)

    if not tracking_ok:
        # Ripiego: cattura nativa a schermate fisse (senza tracking, vista
        # dal preset statico), affidabile ma senza il seguimento del
        # veicolo — vedi 5_traffico_sfondo_sumo.md §12. Il gui-settings va
        # riscritto per includere gli elementi <snapshot> nativi, esclusi
        # finora perche' il tracking era il percorso primario.
        print("  Tracking via TraCI non disponibile: ripiego sulla cattura nativa "
              "a schermate fisse (senza tracking del veicolo)...")
        _write_gui_settings(with_snapshots=True)
        try:
            cattura = subprocess.run(
                [sumo_gui, "-c", CFG_PATH, "-S", "-Q", "--window-size", "1280,720"],
                capture_output=True, text=True, timeout=120)
        except subprocess.TimeoutExpired:
            print("[ERRORE] sumo-gui ha superato il timeout durante la cattura (120s).")
            sys.exit(1)
        if not sorted(glob.glob(os.path.join(FRAMES_DIR, "frame_*.png"))):
            print("[ERRORE] Nessun fotogramma catturato da sumo-gui.")
            print((cattura.stdout or "")[-1500:])
            print((cattura.stderr or "")[-1500:])
            sys.exit(1)

    frames = sorted(glob.glob(os.path.join(FRAMES_DIR, "frame_*.png")))
    print(f"  {len(frames)} fotogrammi catturati"
          f"{'' if tracking_ok else ' (ripiego senza tracking)'}, "
          f"incapsulo in mp4 con ffmpeg...")

    try:
        enc = subprocess.run(
            [ffmpeg_bin, "-y", "-framerate", str(VIDEO_FPS),
             "-i", os.path.join(FRAMES_DIR, "frame_%04d.png"),
             "-pix_fmt", "yuv420p", "-vf", "scale=1280:-2",
             "-movflags", "+faststart", out_path],
            capture_output=True, text=True, timeout=60)
    except subprocess.TimeoutExpired:
        print("[ERRORE] ffmpeg ha superato il timeout durante la codifica (60s).")
        sys.exit(1)

    if enc.returncode != 0 or not os.path.exists(out_path):
        print("[ERRORE] ffmpeg ha fallito la codifica del video.")
        print((enc.stderr or "")[-1500:])
        sys.exit(1)

    for fpath in frames:
        os.remove(fpath)
    try:
        os.rmdir(FRAMES_DIR)
    except OSError:
        pass
    if RUN_ID:
        # File di lavoro univoci di questa invocazione (vedi --run-id): non
        # servono piu' una volta incapsulato il video. Senza questa pulizia
        # si accumulerebbero indefinitamente in cfg_files/ a ogni richiesta
        # della webapp (prima del run-id erano invece nomi fissi, riusati/
        # sovrascritti a ogni chiamata).
        for p in (ROU_PATH, GUI_PATH, CFG_PATH):
            try:
                os.remove(p)
            except OSError:
                pass

    print(f"Video pronto: {out_path}")
    print(f"VIDEO_OUTPUT:{out_path}")
else:
    print("Apro sumo-gui...")
    print(f"→ Premi ▶ Play — l'auto rossa parte al secondo {ego_depart:g}")
    print("→ Ctrl+A per adattare la vista")
    print("→ Click destro sull'auto → Track per seguirla")
    subprocess.Popen([sumo_gui, "-c", CFG_PATH])

"""
sumo_signals.py
----------------
Geometria (bearing, tempo di svolta) e ritardo semaforico per-movimento
ricavato dai dati SUMO estratti (sumo_extracted/sumo_data_{zona}.json).
Estratto da webapp/app.py: usato sia da pddl_writer.py (generazione del
problema PDDL+) sia direttamente da app.py (route /api/solve).
"""
import os
import json
import math
from collections import defaultdict

PROJECT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SUMO_DIR = os.path.join(PROJECT_ROOT, 'sumo_extracted')

# ── Turn rate e ritardi realistici (allineati a build_problems.py) ────────────
TURN_RATE_DPS = 20.0  # gradi/s: velocita' angolare di svolta veicolo
DEFAULT_SIGNAL_DELAY = 17.1  # ritardo realistico incrocio 2 fasi, ciclo 120s
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

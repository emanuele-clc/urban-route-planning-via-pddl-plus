"""
sumo_signals.py
----------------
Geometry (bearing, turn time) and per-movement signal delay derived from
the extracted SUMO data (sumo_extracted/sumo_data_{zone}.json). Extracted
from webapp/app.py: used both by pddl_writer.py (PDDL+ problem generation)
and directly by app.py (route /api/solve).
"""
import os
import json
import math
from collections import defaultdict

PROJECT_ROOT = os.path.join(os.path.dirname(os.path.abspath(__file__)), '..')
SUMO_DIR = os.path.join(PROJECT_ROOT, 'sumo_extracted')

# ── Turn rate and realistic delays (aligned with build_problems.py) ────────────
TURN_RATE_DPS = 20.0  # degrees/s: vehicle turning angular speed
DEFAULT_SIGNAL_DELAY = 17.1  # realistic delay for a 2-phase intersection, 120s cycle
                              # (used when the intersection is not in the SUMO data)


def bearing(lat1, lon1, lat2, lon2):
    """Bearing (degrees, 0=North) from point 1 to point 2."""
    f1 = math.radians(lat1); f2 = math.radians(lat2)
    dl = math.radians(lon2 - lon1)
    y = math.sin(dl) * math.cos(f2)
    x = math.cos(f1) * math.sin(f2) - math.sin(f1) * math.cos(f2) * math.cos(dl)
    return (math.degrees(math.atan2(y, x)) + 360) % 360


def turn_time_s(prev, mid, nxt, node_data, turn_rate=TURN_RATE_DPS):
    """Turn time (s) = turning_angle / turn_rate."""
    b_in  = bearing(node_data[prev]["lat"], node_data[prev]["lon"],
                    node_data[mid]["lat"],  node_data[mid]["lon"])
    b_out = bearing(node_data[mid]["lat"],  node_data[mid]["lon"],
                    node_data[nxt]["lat"],  node_data[nxt]["lon"])
    ang = abs((b_out - b_in + 180) % 360 - 180)
    return round(ang / turn_rate, 2)


def load_all_sumo_delays():
    """Merged map {OSM_node_id: delay_s} from all sumo_data_*.json.
    This way a loaded OSM that falls within a known zone uses the real
    delays."""
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
    """Realistic AVERAGE signal delay for a node (fallback for when a
    specific movement can't be mapped — see
    assign_movement_signal_delay):
    - from the SUMO data if available,
    - otherwise a 2-phase default if it's an OSM traffic signal,
    - otherwise 0."""
    if osm_id in SUMO_DELAYS:
        return SUMO_DELAYS[osm_id]
    if osm_id in signal_nodes:
        return DEFAULT_SIGNAL_DELAY
    return 0


def cluster_member_ids(junction_id):
    """From a SUMO junction id, derives the OSM node ids it represents
    (same logic as extract_sumo_data.py::cluster_member_ids)."""
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
    """Merged map {OSM_node_id: [movement, ...]} from all sumo_data_*.json,
    expanding the SUMO clusters. Each movement has
    delay_s/bearing_in_bucket/bearing_out_bucket/dir_label (see
    extract_sumo_data.py). Used by assign_movement_signal_delay for the
    per-movement (prev,from,to) delay instead of the per-node average."""
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
    """Rounds a bearing (0=North, clockwise) to the nearest sector among
    n_buckets equally spaced ones — same convention as
    extract_sumo_data.py."""
    if angle_deg is None:
        return None
    step = 360.0 / n_buckets
    a = angle_deg % 360.0
    return int(round(a / step)) % n_buckets * step


def circ_dist(a, b):
    """Minimum angular distance between two bearings (0-360)."""
    d = abs(a - b) % 360.0
    return min(d, 360.0 - d)


def assign_movement_signal_delay(a, b, c, node_data, movements_by_node, is_first=False):
    """Signal delay (s) for the specific movement (a,b,c) crossing node 'b'
    — same logic as
    build_problems.py::assign_movement_signal_delay. Returns None if 'b'
    has no SUMO movement data."""
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

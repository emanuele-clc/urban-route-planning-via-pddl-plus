"""
extract_sumo_data.py
=====================
Estrae da una rete SUMO (*.net.xml) i dati necessari ad arricchire i problemi
PDDL+ del progetto "urban-route-planning-via-pddl-plus":

  1. SEMAFORI + SETTAGGI  -> per ogni incrocio semaforizzato (junction
     type="traffic_light") legge il tlLogic reale di SUMO (fasi, stati,
     durate), ne ricava ciclo/verde/giallo/rosso e calcola un RITARDO
     SEMAFORICO REALISTICO (signal-delay) che sostituisce i 30 s fissi usati
     oggi in build_problems.py.

  2. TURN RATE            -> per ogni svolta (connection) calcola dalla
     geometria delle corsie l'angolo di cambio di direzione (heading in
     ingresso vs heading in uscita) e, dato un turn rate realistico in
     gradi/secondo (velocita' angolare tipica di un veicolo in svolta),
     ricava il TEMPO DI SVOLTA.

--- Nota sui valori "realistici" ------------------------------------------
I tempi nel net.xml NON sono i tempi reali di Dublino: netconvert genera i
semafori con un ciclo di default di 90 s (verde diviso equamente, giallo
calcolato dalla velocita' della strada). Fonte: SUMO docs.
A Dublino gli incroci sono controllati dal sistema adattivo SCATS con cicli
tipici fino a ~120 s (in UK/IE il ciclo massimo e' 120 s, 90 s dove ci sono
attraversamenti pedonali). Fonte: JCT Consultancy / review SCATS Dublino.
Per questo lo script preserva la struttura verde/rosso della rete SUMO ma
RISCALA il ciclo a un valore realistico (REAL_CYCLE_S, default 120 s) prima
di calcolare i ritardi.

Turn rate: lo yaw rate di un'auto in svolta urbana stretta e' ~15-20 gradi/s
(oltre ~30 gradi/s interviene l'ESC). Usiamo TURN_RATE_DPS = 20 gradi/s.
Fonte: letteratura vehicle dynamics (yaw rate).
"""

import os
import sys
import json
import math
import xml.etree.ElementTree as ET
from collections import defaultdict, Counter

# --- PARAMETRI REALISTICI (configurabili) ----------------------------------
REAL_CYCLE_S   = 120.0   # ciclo semaforico realistico Dublino (SCATS), s
TURN_RATE_DPS  = 20.0    # turn rate veicolo (velocita' angolare), gradi/s
STRAIGHT_DEG   = 20.0    # sotto questo angolo la svolta e' considerata "dritto"

GREEN_CHARS = {"G", "g"}          # stati di verde nel tlLogic SUMO
# tutto cio' che non e' verde (giallo 'y', rosso 'r', off 'o'/'O', ecc.)
# viene considerato tempo in cui il movimento NON puo' procedere.


# ---------------------------------------------------------------------------
# GEOMETRIA
# ---------------------------------------------------------------------------
def parse_shape(shape_str):
    """'x1,y1 x2,y2 ...' -> [(x1,y1), (x2,y2), ...]"""
    pts = []
    if not shape_str:
        return pts
    for tok in shape_str.strip().split():
        try:
            x, y = tok.split(",")[:2]
            pts.append((float(x), float(y)))
        except ValueError:
            continue
    return pts


def heading(p0, p1):
    """Angolo (gradi) del vettore p0->p1 rispetto all'asse x, in [-180,180]."""
    return math.degrees(math.atan2(p1[1] - p0[1], p1[0] - p0[0]))


def norm180(a):
    """Normalizza un angolo in (-180, 180]."""
    while a > 180.0:
        a -= 360.0
    while a <= -180.0:
        a += 360.0
    return a


def bearing_bucket(angle_deg, n_buckets=8):
    """Arrotonda un angolo (gradi, qualunque range) al piu' vicino tra
    n_buckets settori equidistanti (default 8 settori da 45 gradi),
    usato per confrontare heading SUMO (lane shape) con bearing PDDL
    (calcolati da lat/lon in build_problems.py/webapp) senza richiedere
    un match esatto in floating point."""
    if angle_deg is None:
        return None
    step = 360.0 / n_buckets
    a = angle_deg % 360.0
    return int(round(a / step)) % n_buckets * step


def to_compass(math_deg):
    """Converte un angolo 'matematico' (atan2(dy,dx), 0=Est, antiorario) in
    bearing 'compass' (0=Nord, orario) — stessa convenzione della funzione
    bearing() lat/lon usata in build_problems.py/webapp/app.py, cosi' i
    bucket direzionali SUMO e PDDL sono confrontabili. Assume che il net.xml
    sia proiettato con x=est, y=nord in metri (proiezione di default di
    netconvert per l'import da OSM, es. UTM)."""
    if math_deg is None:
        return None
    return (90.0 - math_deg) % 360.0


def dir_label(angle):
    """Classifica l'angolo di svolta in dritto/sinistra/destra/inversione.
    Convenzione SUMO: y cresce verso l'alto, angolo positivo = antiorario =
    svolta a SINISTRA."""
    a = norm180(angle)
    if abs(a) >= 150.0:
        return "inversione"
    if abs(a) <= STRAIGHT_DEG:
        return "dritto"
    return "sinistra" if a > 0 else "destra"


# ---------------------------------------------------------------------------
# PARSING NET.XML
# ---------------------------------------------------------------------------
def load_net(net_path):
    root = ET.parse(net_path).getroot()

    # lane_id -> lista di punti (shape)
    lane_shape = {}
    # edge_id -> {from, to, name}
    edge_info = {}
    for e in root.findall("edge"):
        eid = e.get("id")
        if eid is None or eid.startswith(":"):
            continue  # edge interno alla junction
        edge_info[eid] = {
            "from": e.get("from"),
            "to":   e.get("to"),
            "name": e.get("name", ""),
        }
        for lane in e.findall("lane"):
            lane_shape[lane.get("id")] = parse_shape(lane.get("shape"))

    # anche le lane interne (:junc_..) hanno una shape: la usiamo come
    # fallback per la curva di svolta se serve
    for e in root.findall("edge"):
        if not (e.get("id") or "").startswith(":"):
            continue
        for lane in e.findall("lane"):
            lane_shape[lane.get("id")] = parse_shape(lane.get("shape"))

    # junction_id -> {type, x, y}
    junctions = {}
    for j in root.findall("junction"):
        jid = j.get("id")
        if jid is None or jid.startswith(":"):
            continue
        junctions[jid] = {
            "type": j.get("type"),
            "x": float(j.get("x", 0)),
            "y": float(j.get("y", 0)),
        }

    # tlLogic_id -> {phases:[(dur,state)], cycle, n_links}
    tls = {}
    for t in root.findall("tlLogic"):
        phases = [(float(p.get("duration")), p.get("state"))
                  for p in t.findall("phase")]
        cycle = sum(d for d, _ in phases)
        n_links = len(phases[0][1]) if phases else 0
        tls[t.get("id")] = {
            "type": t.get("type"),
            "programID": t.get("programID"),
            "offset": float(t.get("offset", 0)),
            "phases": phases,
            "cycle": cycle,
            "n_links": n_links,
        }

    # connection: from,to,fromLane,toLane,via,tl,linkIndex,dir,state
    connections = []
    for c in root.findall("connection"):
        frm = c.get("from")
        if frm is None or frm.startswith(":"):
            continue
        connections.append({
            "from": frm,
            "to": c.get("to"),
            "fromLane": c.get("fromLane"),
            "toLane": c.get("toLane"),
            "via": c.get("via"),
            "tl": c.get("tl"),
            "linkIndex": (int(c.get("linkIndex")) if c.get("linkIndex") is not None else None),
            "dir": c.get("dir"),
            "state": c.get("state"),
        })

    return {
        "lane_shape": lane_shape,
        "edge_info": edge_info,
        "junctions": junctions,
        "tls": tls,
        "connections": connections,
    }


# ---------------------------------------------------------------------------
# 1) SEMAFORI: ciclo, verde/rosso per movimento, ritardo realistico
# ---------------------------------------------------------------------------
def green_red_by_link(tl):
    """Per ogni linkIndex del tlLogic ritorna (green_time, notgreen_time)
    sul ciclo ORIGINALE del net.xml."""
    n = tl["n_links"]
    green = [0.0] * n
    for dur, state in tl["phases"]:
        for i in range(min(n, len(state))):
            if state[i] in GREEN_CHARS:
                green[i] += dur
    cycle = tl["cycle"] or 1.0
    return [(g, cycle - g) for g in green]


def uniform_delay(red_real, cycle_real):
    """Ritardo medio uniforme (arrivi casuali, sotto-saturazione) per un
    semaforo a tempo fisso: d = r^2 / (2C)  [Webster, termine uniforme].
    red_real e cycle_real gia' riscalati al ciclo realistico."""
    if cycle_real <= 0:
        return 0.0
    return round(red_real * red_real / (2.0 * cycle_real), 2)


def green_phase_indices(tl, link_idx):
    """Indici di fase (0-based) in cui il linkIndex e' verde."""
    return [pi for pi, (_, state) in enumerate(tl["phases"])
            if link_idx < len(state) and state[link_idx] in GREEN_CHARS]


def extract_traffic_lights(net, turns_by_link=None, real_cycle=REAL_CYCLE_S):
    """turns_by_link: {(tl_id, linkIndex): turn_dict} da extract_turns(),
    usato per arricchire ogni movimento con bearing/dir_label geometrici
    (necessari a build_problems.py per associare signal-delay alle triple
    PDDL (prev,from,to) — vedi 2_traffic_signal_optimization.md, sez. 3.1)."""
    if turns_by_link is None:
        turns_by_link = {}
    out = {}
    # mappa (from,to,fromLane) -> connection con tl, per etichettare i link
    conn_by_link = defaultdict(dict)  # tl_id -> {linkIndex: conn}
    for c in net["connections"]:
        if c["tl"] is not None and c["linkIndex"] is not None:
            conn_by_link[c["tl"]][c["linkIndex"]] = c

    for tid, tl in net["tls"].items():
        jinfo = net["junctions"].get(tid, {})
        net_cycle = tl["cycle"]
        scale = (real_cycle / net_cycle) if net_cycle else 1.0
        gr = green_red_by_link(tl)

        per_move = []
        delays = []
        for idx, (g, notg) in enumerate(gr):
            green_frac = g / net_cycle if net_cycle else 0.0
            red_real = real_cycle * (1.0 - green_frac)
            d = uniform_delay(red_real, real_cycle)
            delays.append(d)
            c = conn_by_link[tid].get(idx, {})
            t = turns_by_link.get((tid, idx), {})
            phase_idxs = green_phase_indices(tl, idx)
            per_move.append({
                "linkIndex": idx,
                "phase_idx": phase_idxs[0] if phase_idxs else None,
                "green_phase_idxs": phase_idxs,
                "from": c.get("from"),
                "to": c.get("to"),
                "dir": c.get("dir"),
                "green_frac": round(green_frac, 3),
                "green_real_s": round(real_cycle * green_frac, 1),
                "red_real_s": round(red_real, 1),
                "delay_s": d,
                "bearing_in_bucket": t.get("compass_in_bucket"),
                "bearing_out_bucket": t.get("compass_out_bucket"),
                "dir_label": t.get("geo_dir_label"),
            })

        # ritardo rappresentativo del NODO = media sui movimenti controllati
        # (mantenuto per retro-compatibilita' col punto 1: node_signal_delay)
        signal_delay = round(sum(delays) / len(delays), 2) if delays else 0.0

        out[tid] = {
            "junction_type": jinfo.get("type"),
            "x": jinfo.get("x"),
            "y": jinfo.get("y"),
            "net_cycle_s": round(net_cycle, 1),
            "real_cycle_s": real_cycle,
            "n_links": tl["n_links"],
            "offset_s": tl["offset"],
            "phases": [{"duration_s": d, "state": s} for d, s in tl["phases"]],
            "signal_delay_s": signal_delay,
            "movements": per_move,
        }
    return out


# ---------------------------------------------------------------------------
# 2) TURN RATE: angolo di svolta e tempo di svolta per ogni connection
# ---------------------------------------------------------------------------
def lane_in_heading(pts):
    """Heading dell'ULTIMO segmento della corsia (avvicinamento all'incrocio)."""
    if len(pts) < 2:
        return None
    return heading(pts[-2], pts[-1])


def lane_out_heading(pts):
    """Heading del PRIMO segmento della corsia (uscita dall'incrocio)."""
    if len(pts) < 2:
        return None
    return heading(pts[0], pts[1])


def extract_turns(net, turn_rate=TURN_RATE_DPS):
    turns = []
    dir_map = {"s": "dritto", "l": "sinistra", "r": "destra",
               "t": "inversione", "L": "sinistra", "R": "destra",
               "invalid": "invalid"}
    for c in net["connections"]:
        from_lane_id = f"{c['from']}_{c['fromLane']}"
        to_lane_id = f"{c['to']}_{c['toLane']}"
        fp = net["lane_shape"].get(from_lane_id)
        tp = net["lane_shape"].get(to_lane_id)
        if not fp or not tp:
            continue
        h_in = lane_in_heading(fp)
        h_out = lane_out_heading(tp)
        if h_in is None or h_out is None:
            continue
        angle = norm180(h_out - h_in)
        turn_time = round(abs(angle) / turn_rate, 2) if turn_rate else 0.0

        # incrocio = junction di arrivo dell'edge 'from'
        junc = net["edge_info"].get(c["from"], {}).get("to")

        compass_in = to_compass(h_in)
        compass_out = to_compass(h_out)

        turns.append({
            "junction": junc,
            "from": c["from"],
            "to": c["to"],
            "fromLane": c["fromLane"],
            "tl": c["tl"],
            "linkIndex": c["linkIndex"],
            "sumo_dir": c["dir"],
            "sumo_dir_label": dir_map.get(c["dir"], c["dir"]),
            "angle_deg": round(angle, 1),
            "geo_dir_label": dir_label(angle),
            "turn_time_s": turn_time,
            # bearing assoluti (compass, 0=Nord orario) e relativo bucket a
            # 8 settori — usati per il match con le triple PDDL (prev,from,to)
            "compass_in": round(compass_in, 1) if compass_in is not None else None,
            "compass_out": round(compass_out, 1) if compass_out is not None else None,
            "compass_in_bucket": bearing_bucket(compass_in),
            "compass_out_bucket": bearing_bucket(compass_out),
        })
    return turns


# ---------------------------------------------------------------------------
# MAPPA id-nodo-OSM -> ritardo semaforico  (per integrazione in build_problems)
# ---------------------------------------------------------------------------
def cluster_member_ids(junction_id):
    """Da un id junction SUMO ricava gli id-nodo OSM che rappresenta.
    - junction normale: '12639663'            -> ['12639663']
    - cluster:  'cluster_1064_2033_659784'    -> ['1064','2033','659784']
      (i membri nascosti '_#Nmore' non sono ricostruibili dal solo id)."""
    if not junction_id.startswith("cluster_"):
        return [junction_id]
    body = junction_id[len("cluster_"):]
    ids = []
    for tok in body.split("_"):
        if tok.startswith("#"):      # es. '#2more'
            continue
        if tok.isdigit():
            ids.append(tok)
    return ids


def node_signal_delay_map(tl_data):
    """{id_nodo_OSM: ritardo_semaforico_s} espandendo i cluster.
    Se un nodo compare in piu' junction, tiene il ritardo massimo."""
    m = {}
    for jid, v in tl_data.items():
        d = v["signal_delay_s"]
        for nid in cluster_member_ids(jid):
            m[nid] = max(m.get(nid, 0.0), d)
    return m


# ---------------------------------------------------------------------------
# REPORT
# ---------------------------------------------------------------------------
def summarize(zone, tl_data, turns):
    lines = []
    lines.append(f"## Zona {zone}")
    lines.append("")
    # --- semafori ---
    n_tl = len(tl_data)
    delays = [v["signal_delay_s"] for v in tl_data.values()]
    avg_delay = sum(delays) / len(delays) if delays else 0
    clustered = sum(1 for k in tl_data if k.startswith("cluster"))
    lines.append(f"### Semafori: {n_tl} incroci semaforizzati "
                 f"({clustered} cluster di piu' incroci)")
    lines.append(f"- Ciclo net.xml (default SUMO): tutti a "
                 f"{Counter(round(v['net_cycle_s']) for v in tl_data.values()).most_common(1)[0][0]:.0f} s")
    lines.append(f"- Ciclo realistico applicato: {REAL_CYCLE_S:.0f} s (Dublino/SCATS)")
    lines.append(f"- Ritardo semaforico medio per nodo: {avg_delay:.1f} s "
                 f"(min {min(delays):.1f} / max {max(delays):.1f})  "
                 f"[oggi in PDDL: 30 s fissi]")
    lines.append("")
    # --- turn ---
    n_turns = len(turns)
    by_geo = Counter(t["geo_dir_label"] for t in turns)
    non_straight = [t for t in turns if t["geo_dir_label"] != "dritto"]
    tt = [t["turn_time_s"] for t in non_straight]
    avg_tt = sum(tt) / len(tt) if tt else 0
    # coerenza geo vs SUMO dir
    agree = sum(1 for t in turns
                if t["sumo_dir_label"] == t["geo_dir_label"])
    lines.append(f"### Turn: {n_turns} manovre (connection)")
    lines.append(f"- Per tipo (da geometria): " +
                 ", ".join(f"{k}={v}" for k, v in by_geo.most_common()))
    lines.append(f"- Turn rate usato: {TURN_RATE_DPS:.0f} gradi/s")
    lines.append(f"- Tempo di svolta medio (escluso 'dritto'): {avg_tt:.1f} s")
    lines.append(f"- Coerenza direzione geometrica vs attributo SUMO 'dir': "
                 f"{agree}/{n_turns} ({100*agree/n_turns:.0f}%)")
    lines.append("")
    return "\n".join(lines)


# ---------------------------------------------------------------------------
# MAIN
# ---------------------------------------------------------------------------
def process(zone, net_path, out_dir):
    net = load_net(net_path)
    turns = extract_turns(net)
    turns_by_link = {
        (t["tl"], t["linkIndex"]): t
        for t in turns if t["tl"] is not None and t["linkIndex"] is not None
    }
    tl_data = extract_traffic_lights(net, turns_by_link)

    result = {
        "zone": zone,
        "source_net": os.path.basename(net_path),
        "params": {
            "real_cycle_s": REAL_CYCLE_S,
            "turn_rate_dps": TURN_RATE_DPS,
            "straight_threshold_deg": STRAIGHT_DEG,
        },
        "n_traffic_lights": len(tl_data),
        "n_turns": len(turns),
        # mappa pronta per build_problems.py: id-nodo-OSM -> ritardo (s)
        "node_signal_delay": node_signal_delay_map(tl_data),
        "traffic_lights": tl_data,
        "turns": turns,
    }
    out_json = os.path.join(out_dir, f"sumo_data_{zone}.json")
    with open(out_json, "w", encoding="utf-8") as f:
        json.dump(result, f, ensure_ascii=False, indent=2)

    report = summarize(zone, tl_data, turns)
    return out_json, report


if __name__ == "__main__":
    BASE = os.path.dirname(os.path.abspath(__file__))
    # cerca la cartella net_files relativa al progetto o al file
    net_dir_candidates = [
        os.path.join(BASE, "net_files"),
        os.path.join(os.getcwd(), "net_files"),
    ]
    net_dir = next((d for d in net_dir_candidates if os.path.isdir(d)), None)

    zones = sys.argv[1:] if len(sys.argv) > 1 else ["piccola", "media", "grande"]
    out_dir = os.path.join(BASE, "sumo_extracted")
    os.makedirs(out_dir, exist_ok=True)

    all_reports = ["# Estrazione dati SUMO (semafori + turn rate)\n"]
    for zone in zones:
        net_path = os.path.join(net_dir, f"{zone}.net.xml")
        if not os.path.exists(net_path):
            print(f"[skip] net non trovata: {net_path}")
            continue
        out_json, report = process(zone, net_path, out_dir)
        print(f"[{zone}] -> {out_json}")
        all_reports.append(report)

    with open(os.path.join(out_dir, "report.md"), "w", encoding="utf-8") as f:
        f.write("\n".join(all_reports))
    print(f"\nreport: {os.path.join(out_dir, 'report.md')}")

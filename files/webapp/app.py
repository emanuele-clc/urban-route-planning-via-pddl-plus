import os
import sys
import math
import re
import json
import glob
import sysconfig
import subprocess
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict, Counter, deque

from flask import Flask, render_template, request, jsonify

app = Flask(__name__)

HIGHWAY_TYPES = {
    "motorway", "trunk", "primary", "secondary", "tertiary",
    "residential", "motorway_link", "trunk_link", "primary_link",
    "secondary_link", "tertiary_link", "unclassified", "living_street",
}

DOMAIN_PATH = os.path.join(os.path.dirname(os.path.abspath(__file__)),
                           '..', 'pddl_files', 'domain.pddl')


def haversine(lat1, lon1, lat2, lon2):
    R = 6371000
    f1 = math.radians(lat1)
    f2 = math.radians(lat2)
    df = math.radians(lat2 - lat1)
    dl = math.radians(lon2 - lon1)
    a = math.sin(df/2)**2 + math.cos(f1) * math.cos(f2) * math.sin(dl/2)**2
    return round(R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))


def slugify(name):
    if not name:
        return ""
    s = re.sub(r"[^a-z0-9]+", "_", name.lower()).strip("_")[:28]
    return s if s else ""


def build_contracted_graph(osm_path):
    with open(osm_path, encoding="utf-8") as f:
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

    junctions = set()
    for nds, _, _ in good_ways:
        junctions.add(nds[0])
        junctions.add(nds[-1])
    junctions |= {n for n, c in membership.items() if c >= 2}

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
                    if nid not in adj[seg_start] or adj[seg_start][nid][0] > seg_dist:
                        adj[seg_start][nid] = (seg_dist, spd)
                    if not oneway:
                        if seg_start not in adj[nid] or adj[nid][seg_start][0] > seg_dist:
                            adj[nid][seg_start] = (seg_dist, spd)
                seg_start = nid
                seg_dist = 0

    return node_data, adj


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


def write_pddl(zone, selected, node_data, edges, start, goal, nm):
    def n(x): return nm[x]
    se = sorted(edges.keys(), key=lambda e: (n(e[0]), n(e[1])))
    lines = []
    lines.append(f"(define (problem dublin-{zone})")
    lines.append("  (:domain dublin-navigation)")
    lines.append("")
    lines.append(f"  ; {len(selected)} nodi — zona {zone}")
    lines.append(f"  ; START: {n(start)}   GOAL: {n(goal)}")
    lines.append("")
    lines.append("  (:objects")
    for nd in selected:
        nd_name = node_data[nd]["name"]
        if nd_name:
            lines.append(f"    {n(nd)}  ; {nd_name}")
        else:
            lines.append(f"    {n(nd)}")
    lines.append("    - location")
    lines.append("  )")
    lines.append("")
    lines.append("  (:init")
    lines.append(f"    (at {n(start)})")
    lines.append("    (= (total-dist) 0)")
    lines.append("")
    lines.append("    ; Progress = 0 per ogni tratto")
    for a, b in se:
        lines.append(f"    (= (progress {n(a):<28} {n(b)}) 0)")
    lines.append("")
    lines.append("    ; Strade")
    for a, b in se:
        lines.append(f"    (road {n(a):<28} {n(b)})")
    lines.append("")
    lines.append("    ; Distanze (metri)")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (distance {n(a):<28} {n(b)}) {d})")
    lines.append("")
    lines.append("    ; Velocita (m/s)")
    for a, b in se:
        d, spd = edges[(a, b)]
        lines.append(f"    (= (speed {n(a):<28} {n(b)}) {spd})")
    lines.append("")
    lines.append("  )")
    lines.append("")
    lines.append(f"  (:goal (at {n(goal)}))")
    lines.append("")
    lines.append("  (:metric minimize (total-dist))")
    lines.append(")")
    return "\n".join(lines)


def trova_enhsp():
    cartelle = [sysconfig.get_path("purelib"), sysconfig.get_path("platlib")]
    for base in cartelle:
        if base and os.path.exists(base):
            for root, dirs, files in os.walk(base):
                for f in files:
                    if f.endswith(".jar") and "enhsp" in f.lower():
                        return os.path.join(root, f)
    for ver in ["313", "312", "311", "310", "39"]:
        for base in [
            os.path.join(os.environ.get("LOCALAPPDATA", ""), "Programs", "Python", f"Python{ver}"),
            os.path.join(os.environ.get("APPDATA", ""), "Python", f"Python{ver}"),
            f"C:\\Python{ver}",
        ]:
            for hit in glob.glob(os.path.join(base, "**", "enhsp*.jar"), recursive=True):
                return hit
    for base in ["/usr/local/lib", "/usr/lib", os.path.expanduser("~/.local/lib")]:
        for hit in glob.glob(os.path.join(base, "**", "enhsp*.jar"), recursive=True):
            return hit
    return None


def parse_plan(output, nm_inv):
    """Estrae la sequenza di nodi dal piano ENHSP."""
    plan_lines = []
    in_plan = False
    for line in output.splitlines():
        if "Found Plan:" in line:
            in_plan = True
            continue
        if in_plan:
            if line.strip() == "" or "Plan-Length" in line:
                in_plan = False
            else:
                plan_lines.append(line.strip())

    route_names = []
    for line in plan_lines:
        m = re.search(r'\(start-move\s+(\S+)\s+(\S+)\)', line, re.IGNORECASE)
        if m:
            frm = m.group(1).lower()
            to = m.group(2).lower()
            if not route_names:
                route_names.append(frm)
            route_names.append(to)

    total_dist = None
    plan_time = None
    for line in output.splitlines():
        if "Metric" in line and total_dist is None:
            m = re.search(r'([\d.]+)', line)
            if m:
                total_dist = float(m.group(1))
        if ("Planning Time" in line or "Elapsed Time" in line) and plan_time is None:
            m = re.search(r'([\d.]+)', line)
            if m:
                plan_time = float(m.group(1))

    return "\n".join(plan_lines), route_names, total_dist, plan_time


@app.route('/')
def index():
    return render_template('index.html')


@app.route('/api/process', methods=['POST'])
def process():
    osm_file = request.files.get('osm_file')
    max_nodes = int(request.form.get('max_nodes', 50))
    zone = request.form.get('zone', 'custom') or 'custom'

    if not osm_file:
        return jsonify({"error": "Nessun file caricato"}), 400

    tmp_dir = tempfile.mkdtemp()
    osm_path = os.path.join(tmp_dir, 'input.osm')
    osm_file.save(osm_path)

    try:
        node_data, adj = build_contracted_graph(osm_path)
        if not adj:
            return jsonify({"error": "Nessuna strada percorribile trovata nel file OSM"}), 400

        selected = select_connected_subgraph(node_data, adj, max_nodes)
        sel_set = set(selected)

        edges = {}
        for a in selected:
            for b, (d, spd) in adj.get(a, {}).items():
                if b in sel_set:
                    edges[(a, b)] = (d, spd)

        out_deg = Counter(a for a, _ in edges)
        start = max(selected, key=lambda nd: out_deg[nd])

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
        reachable_others = reach - {start}
        if not reachable_others:
            return jsonify({"error": "Nessun nodo raggiungibile dallo start"}), 400
        goal = max(reachable_others,
                   key=lambda nd: haversine(slat, slon, node_data[nd]["lat"], node_data[nd]["lon"]))

        nm = name_map_for(selected, node_data)
        nm_inv = {v: k for k, v in nm.items()}

        pddl_content = write_pddl(zone, selected, node_data, edges, start, goal, nm)

        pddl_path = os.path.join(tmp_dir, 'problem.pddl')
        with open(pddl_path, 'w', encoding='utf-8') as f:
            f.write(pddl_content)

        # provo a far girare ENHSP
        plan_text = None
        route = None
        total_dist = None
        plan_time = None
        enhsp_error = None

        jar = trova_enhsp()
        domain_abs = os.path.abspath(DOMAIN_PATH)

        if not jar:
            enhsp_error = "ENHSP non trovato. Installa con: pip install up-enhsp"
        elif not os.path.exists(domain_abs):
            enhsp_error = f"domain.pddl non trovato in: {domain_abs}"
        else:
            try:
                cmd = ["java", "-jar", jar, "-o", domain_abs, "-f", pddl_path, "-s", "aibr"]
                result = subprocess.run(cmd, capture_output=True, text=True, timeout=120)
                output = result.stdout + result.stderr
                if "Problem Solved" in output:
                    plan_text, route, total_dist, plan_time = parse_plan(output, nm_inv)
                else:
                    enhsp_error = "ENHSP non ha trovato una soluzione"
            except subprocess.TimeoutExpired:
                enhsp_error = "ENHSP ha superato il timeout (120s)"
            except FileNotFoundError:
                enhsp_error = "Java non trovato. Installa Java 17+."

        # preparo i dati per la mappa
        nodes_out = []
        for nd in selected:
            nodes_out.append({
                "id": nm[nd],
                "lat": node_data[nd]["lat"],
                "lon": node_data[nd]["lon"],
                "name": node_data[nd]["name"],
                "is_start": nd == start,
                "is_goal": nd == goal,
            })

        edges_out = []
        for (a, b), (d, spd) in edges.items():
            edges_out.append({
                "from": nm[a],
                "to": nm[b],
                "distance": d,
                "speed": spd,
            })

        return jsonify({
            "success": True,
            "pddl_content": pddl_content,
            "plan_text": plan_text,
            "route": route,
            "nodes": nodes_out,
            "edges": edges_out,
            "enhsp_error": enhsp_error,
            "stats": {
                "n_nodes": len(selected),
                "n_edges": len(edges),
                "n_reachable": len(reach),
                "total_dist": total_dist,
                "plan_time": plan_time,
                "start": nm[start],
                "goal": nm[goal],
            }
        })

    except ET.ParseError as e:
        return jsonify({"error": f"File OSM non valido: {e}"}), 400
    except Exception as e:
        import traceback
        return jsonify({"error": str(e), "trace": traceback.format_exc()}), 500


if __name__ == '__main__':
    print("Avvia il server su http://localhost:5000")
    app.run(debug=True, port=5000)

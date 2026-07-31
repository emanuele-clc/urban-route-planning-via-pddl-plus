"""
calibrate_sumo.py
=================
PUNTO 5 — calibrazione del modello PDDL+ contro SUMO.

Il dominio PDDL+ stima il tempo di un percorso con una formula: guida =
distanza/velocita', piu' i ritardi ai semafori (Webster) e i tempi di svolta.
Ma quella e' una stima. SUMO invece simula il veicolo nel dettaglio, con
accelerazioni, frenate e attese reali ai rossi. Questo script confronta le due
cose sullo STESSO identico percorso, per capire quanto e' realistico il modello.

Punto chiave: previsto e misurato sono calcolati sulla stessa lista di archi
SUMO, cosi' il confronto non e' falsato da percorsi diversi. Il tempo previsto
usa le stesse grandezze del dominio (lunghezza/velocita' degli archi, angoli di
svolta dalle corsie, ritardo di Webster per nodo estratto dalla rete). Il tempo
misurato viene da un veicolo isolato in SUMO (partenze molto distanziate, cosi'
non c'e' traffico e si misura il percorso in se').

Tre confronti:
  - GUIDA+SVOLTE previsto vs tempo in MOVIMENTO in SUMO (durata - attesa): la
    parte deterministica del modello contro la microsimulazione;
  - SEMAFORI (Webster) previsto vs ATTESA misurata ai rossi;
  - TOTALE previsto vs durata totale misurata.

Uso (dalla radice del progetto):
    python scripts/calibrate_sumo.py                    # tutte le zone
    python scripts/calibrate_sumo.py piccola
    python scripts/calibrate_sumo.py media --n-routes 30
"""

import os
import sys
import json
import math
import random
import argparse
import tempfile
import statistics
import subprocess
import xml.etree.ElementTree as ET

SCRIPT_DIR = os.path.dirname(os.path.abspath(__file__))
sys.path.insert(0, SCRIPT_DIR)
from sumo_common import build_sumo_graph, dijkstra, build_edge_endpoints  # noqa: E402

BASE = os.path.dirname(SCRIPT_DIR)
NET_DIR = os.path.join(BASE, "net_files")
SUMO_DIR = os.path.join(BASE, "sumo_extracted")
OUT_DIR = os.path.join(BASE, "sumo_comparison")

ZONES = ("piccola", "media", "grande")

DEPART_GAP = 250      # s fra due partenze: veicoli isolati, niente traffico
MIN_DIST = 300        # m
MAX_DIST = 3000       # m
TURN_RATE_DPS = 20.0  # gradi/s, come nel resto del progetto
SEED = 42


# --------------------------------------------------------------------------
def find_sumo_bin(name="sumo"):
    home = os.environ.get("SUMO_HOME", "")
    cands = []
    if home:
        cands += [os.path.join(home, "bin", name + ".exe"),
                  os.path.join(home, "bin", name)]
    try:
        import sumo as _s
        p = os.path.dirname(_s.__file__)
        cands += [os.path.join(p, "bin", name + ".exe"),
                  os.path.join(p, "bin", name)]
    except Exception:
        pass
    import shutil
    for c in cands:
        if os.path.exists(c):
            return c
    return shutil.which(name)


def read_edge_speed_shape(net_path):
    """{edge_id: velocita'} e {edge_id: [(x,y),...]} dalla prima corsia."""
    root = ET.parse(net_path).getroot()
    spd, shape = {}, {}
    for e in root.findall("edge"):
        eid = e.get("id")
        if not eid or eid.startswith(":") or e.get("function") == "internal":
            continue
        lane = e.find("lane")
        if lane is None:
            continue
        if lane.get("speed"):
            try:
                spd[eid] = float(lane.get("speed"))
            except ValueError:
                pass
        if lane.get("shape"):
            try:
                shape[eid] = [tuple(map(float, p.split(","))) for p in lane.get("shape").split()]
            except Exception:
                pass
    return spd, shape


def signal_delay_lookup(net_path):
    """Ritorna una funzione delay(junction_id) -> ritardo di Webster (s) dal
    file estratto (node_signal_delay). Gestisce le junction cluster cercando
    fra i membri; 0 se il nodo non e' semaforizzato."""
    path = os.path.join(SUMO_DIR, f"sumo_data_{os.path.basename(net_path).split('.')[0]}.json")
    nsd = {}
    if os.path.exists(path):
        try:
            nsd = json.load(open(path, encoding="utf-8")).get("node_signal_delay", {})
        except Exception:
            nsd = {}

    def delay(jid):
        if jid in nsd:
            return nsd[jid]
        if jid.startswith("cluster_"):
            best = 0.0
            for m in jid[len("cluster_"):].split("_"):
                m = m.lstrip("#")
                if m in nsd:
                    best = max(best, nsd[m])
            return best
        return 0.0
    return delay


def _heading(p, q):
    return math.atan2(q[1] - p[1], q[0] - p[0])


def turn_deg(shape_in, shape_out):
    """Angolo di svolta (gradi, 0-180) fra l'arco entrante e quello uscente,
    dagli ultimi/primi due punti delle rispettive corsie."""
    if not shape_in or not shape_out or len(shape_in) < 2 or len(shape_out) < 2:
        return 0.0
    a = _heading(shape_in[-2], shape_in[-1])
    b = _heading(shape_out[0], shape_out[1])
    d = math.degrees(b - a)
    while d > 180:
        d -= 360
    while d < -180:
        d += 360
    return abs(d)


def predict(edges, eid_len, eid_speed, eid_shape, endpoints, delay):
    """Tempo previsto dal modello sulla lista di archi SUMO 'edges'.
    Ritorna (drive, turn, signal) in secondi."""
    drive = 0.0
    for e in edges:
        v = eid_speed.get(e, 0)
        if v > 0:
            drive += eid_len.get(e, 0.0) / v
    turn = 0.0
    for i in range(len(edges) - 1):
        turn += turn_deg(eid_shape.get(edges[i]), eid_shape.get(edges[i + 1])) / TURN_RATE_DPS
    signal = 0.0
    for i in range(len(edges) - 1):
        j = endpoints.get(edges[i], (None, None))[1]   # junction attraversata
        if j:
            signal += delay(j)
    return drive, turn, signal


def sample_routes(zone, net_path, n_routes):
    graph, jpos, eid_len, _et, _cp = build_sumo_graph(net_path)
    eid_speed, eid_shape = read_edge_speed_shape(net_path)
    endpoints = build_edge_endpoints(graph)
    delay = signal_delay_lookup(net_path)

    nodes = [n for n in jpos if graph.get(n)]
    rng = random.Random(SEED)
    routes, tries = [], 0
    while len(routes) < n_routes and tries < n_routes * 80:
        tries += 1
        s = rng.choice(nodes)
        g = rng.choice(nodes)
        if s == g:
            continue
        edges = dijkstra(graph, s, g)
        if not edges or len(edges) < 2:
            continue
        dist = sum(eid_len.get(e, 0.0) for e in edges)
        if not (MIN_DIST <= dist <= MAX_DIST):
            continue
        # almeno un semaforo attraversato, per esercitare anche il modello dei ritardi
        n_sig = sum(1 for i in range(len(edges) - 1)
                    if delay(endpoints.get(edges[i], (None, None))[1]) > 0)
        if n_sig == 0:
            continue
        d, t, sg = predict(edges, eid_len, eid_speed, eid_shape, endpoints, delay)
        routes.append({"edges": edges, "dist": round(dist), "n_sig": n_sig,
                       "pred_drive": d, "pred_turn": t, "pred_signal": sg})
    return routes


def build_rou(routes):
    lines = ['<routes>',
             '  <vType id="car" accel="2.6" decel="4.5" sigma="0.0" '
             'length="4.5" maxSpeed="55"/>']
    for i, r in enumerate(routes):
        lines.append(f'  <vehicle id="v{i}" type="car" depart="{i * DEPART_GAP}" departSpeed="0">')
        lines.append(f'    <route edges="{" ".join(r["edges"])}"/>')
        lines.append('  </vehicle>')
    lines.append('</routes>')
    return "\n".join(lines)


def run_and_measure(zone, sumo_bin, n_routes, verbose=True):
    net_path = os.path.join(NET_DIR, f"{zone}.net.xml")
    if not os.path.exists(net_path):
        print(f"[{zone}] rete non trovata: {net_path}")
        return None

    routes = sample_routes(zone, net_path, n_routes)
    if not routes:
        print(f"[{zone}] nessun percorso campionato.")
        return None

    tmp = tempfile.mkdtemp(prefix=f"cal_{zone}_")
    rou = os.path.join(tmp, "r.rou.xml")
    trip = os.path.join(tmp, "trip.xml")
    with open(rou, "w", encoding="utf-8") as f:
        f.write(build_rou(routes))

    # --end limita la durata (un veicolo su rotta non valida non farebbe mai
    # terminare la simulazione). Il teleport resta ai valori di default: per un
    # veicolo isolato l'attesa a un rosso e' sotto la soglia, quindi non scatta,
    # ma evita che un caso patologico blocchi tutto.
    sim_end = len(routes) * DEPART_GAP + 1500
    cmd = [sumo_bin, "-n", net_path, "-r", rou, "--tripinfo-output", trip,
           "--begin", "0", "--end", str(sim_end),
           "--no-step-log", "true", "--no-warnings", "true",
           "--ignore-route-errors", "true"]
    if verbose:
        print(f"[{zone}] {len(routes)} percorsi, simulazione in corso...")
    subprocess.run(cmd, capture_output=True, text=True, timeout=1800)
    if not os.path.exists(trip):
        print(f"[{zone}] nessun tripinfo prodotto.")
        return None

    measured = {}
    for t in ET.parse(trip).getroot().findall("tripinfo"):
        measured[t.get("id")] = (float(t.get("duration", 0)), float(t.get("waitingTime", 0)))

    rows = []
    for i, r in enumerate(routes):
        m = measured.get(f"v{i}")
        if not m:
            continue
        dur, wait = m
        rows.append({
            "dist": r["dist"], "n_sig": r["n_sig"],
            "pred_drive_turn": round(r["pred_drive"] + r["pred_turn"], 1),
            "pred_signal": round(r["pred_signal"], 1),
            "pred_total": round(r["pred_drive"] + r["pred_turn"] + r["pred_signal"], 1),
            "meas_moving": round(dur - wait, 1),
            "meas_waiting": round(wait, 1),
            "meas_total": round(dur, 1),
        })
    return rows


# --------------------------------------------------------------------------
def _pearson(xs, ys):
    if len(xs) < 2:
        return None
    mx, my = statistics.mean(xs), statistics.mean(ys)
    num = sum((x - mx) * (y - my) for x, y in zip(xs, ys))
    dx = sum((x - mx) ** 2 for x in xs) ** 0.5
    dy = sum((y - my) ** 2 for y in ys) ** 0.5
    return round(num / (dx * dy), 3) if dx and dy else None


def summarize(zone, rows):
    def stats(pk, mk):
        p = [r[pk] for r in rows]
        m = [r[mk] for r in rows]
        errs = [b - a for a, b in zip(p, m)]                  # misurato - previsto
        rel = [(b - a) / a * 100 for a, b in zip(p, m) if a > 0]
        return {"pred_mean": round(statistics.mean(p), 1),
                "meas_mean": round(statistics.mean(m), 1),
                "mae": round(statistics.mean([abs(e) for e in errs]), 1),
                "bias": round(statistics.mean(errs), 1),
                "bias_pct": round(statistics.mean(rel), 1) if rel else None,
                "corr": _pearson(p, m)}
    return {"zone": zone, "n": len(rows),
            "drive_turn": stats("pred_drive_turn", "meas_moving"),
            "signal": stats("pred_signal", "meas_waiting"),
            "total": stats("pred_total", "meas_total"),
            "rows": rows}


def write_report(results):
    os.makedirs(OUT_DIR, exist_ok=True)
    with open(os.path.join(OUT_DIR, "calibration.json"), "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)

    L = ["# Calibrazione del modello PDDL+ contro SUMO", "",
         "Il dominio PDDL+ stima il tempo di un percorso con una formula (guida =",
         "distanza/velocita', piu' i ritardi di Webster ai semafori e i tempi di",
         "svolta). Qui la stessa stima viene confrontata, sullo stesso identico",
         "percorso, con il tempo che SUMO misura simulando il veicolo nel",
         "dettaglio. I veicoli sono isolati (partenze molto distanziate), quindi",
         "si misura il percorso in se', non la congestione.", "",
         "Legenda: **bias** = misurato - previsto (in secondi; positivo = il",
         "modello sottostima, negativo = sovrastima); **corr** = correlazione di",
         "Pearson fra previsto e misurato sui singoli percorsi.", "",
         "| Zona | N | Confronto | Previsto (s) | Misurato (s) | Bias | Bias % | Corr |",
         "|---|--:|---|--:|--:|--:|--:|--:|"]
    label = {"drive_turn": "guida+svolte vs movimento",
             "signal": "semafori (Webster) vs attesa",
             "total": "TOTALE vs durata"}
    for r in results:
        for key in ("drive_turn", "signal", "total"):
            s = r[key]
            bp = f"{s['bias_pct']:+.1f}%" if s['bias_pct'] is not None else "—"
            co = f"{s['corr']:.2f}" if s['corr'] is not None else "—"
            L.append(f"| **{r['zone']}** | {r['n']} | {label[key]} | "
                     f"{s['pred_mean']:.1f} | {s['meas_mean']:.1f} | "
                     f"{s['bias']:+.1f} | {bp} | {co} |")
    L += ["", "## Come leggere i numeri", "",
          "La riga **guida+svolte** e' la parte deterministica del modello. Una",
          "correlazione vicina a 1 dice che il modello ordina bene i percorsi per",
          "durata: piu' prevede lungo, piu' SUMO misura lungo. Un piccolo scarto",
          "sistematico e' normale ed e' il costo di accelerazioni e decelerazioni",
          "che un modello a velocita' costante non rappresenta.", "",
          "La riga **semafori** confronta il ritardo medio di Webster (usato dal",
          "modello) con l'attesa davvero misurata ai rossi. Qui la dispersione e'",
          "piu' alta, perche' Webster e' una media statistica mentre l'attesa di",
          "un singolo passaggio dipende da quando esattamente il veicolo arriva",
          "al semaforo: puo' trovare verde o rosso pieno. La media resta pero'",
          "confrontabile, ed e' cio' che conta quando il modello somma molti",
          "semafori lungo un percorso.", "",
          "La riga **TOTALE** mette insieme le due parti: e' la stima complessiva",
          "che il planner usa per scegliere il percorso.", "",
          "## Riproducibilita'", "",
          "```bash",
          "python scripts/calibrate_sumo.py               # tutte le zone",
          "python scripts/calibrate_sumo.py media --n-routes 30",
          "```",
          "Output: `sumo_comparison/calibration.json` (un record per percorso) e",
          "questo report."]
    path = os.path.join(OUT_DIR, "calibration.md")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    return path


def main():
    ap = argparse.ArgumentParser(
        description="Punto 5: calibra il tempo previsto dal modello PDDL+ contro "
                    "il tempo misurato in SUMO, sugli stessi percorsi.")
    ap.add_argument("zones", nargs="*", default=list(ZONES))
    ap.add_argument("--n-routes", type=int, default=25,
                    help="numero di percorsi campionati per zona (default 25)")
    args = ap.parse_args()

    sumo_bin = find_sumo_bin("sumo")
    if not sumo_bin:
        print("[ERRORE] SUMO non trovato. Installa SUMO o: pip install eclipse-sumo")
        sys.exit(1)
    print(f"SUMO: {sumo_bin}\n")

    results = []
    for zone in args.zones:
        if zone not in ZONES:
            print(f"[skip] zona sconosciuta: {zone}")
            continue
        rows = run_and_measure(zone, sumo_bin, args.n_routes)
        if rows:
            r = summarize(zone, rows)
            results.append(r)
            dt, to = r["drive_turn"], r["total"]
            print(f"[{zone}] guida+svolte: corr {dt['corr']}, bias {dt['bias']:+.1f}s "
                  f"({dt['bias_pct']:+.1f}%) | totale: corr {to['corr']}, "
                  f"bias {to['bias']:+.1f}s")
        print()

    if results:
        print(f"Report salvato in: {write_report(results)}")
    else:
        print("Nessuna calibrazione eseguita.")


if __name__ == "__main__":
    main()

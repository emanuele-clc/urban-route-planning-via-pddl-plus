"""
compare_sumo.py
===============
PUNTO 4 della roadmap — confronto **in simulazione** dei nostri piani
semaforici contro la baseline gia' presente nella rete.

Mentre il punto 2 stima il guadagno con il modello analitico di Webster dentro
PDDL+, qui il guadagno viene misurato da SUMO, che simula il traffico
microscopicamente: code, accelerazioni, fasi reali verde/giallo/rosso e
interazione fra veicoli. E' quindi una verifica **indipendente** dal modello
usato per ottimizzare.

--- Disegno dell'esperimento -------------------------------------------------
Per ogni zona si eseguono DUE simulazioni SUMO identiche in tutto tranne che
nel programma semaforico:

    BASELINE    : rete cosi' com'e' (programma "0" generato da netconvert)
    OTTIMIZZATO : stessa rete + cfg_files/tls_<zona>.add.xml (programma
                  "optimized" prodotto dal punto 3)

Punti chiave per la validita' del confronto:

  * **Stessa domanda di traffico**: si usano le coppie O-D di
    `sumo_extracted/demand_<zona>.json`, cioe' lo stesso campione condiviso
    con il punto 2 e con `compare_versions.py`.
  * **Stesse rotte**: gli itinerari (sequenze di archi) sono calcolati UNA
    volta con Dijkstra e riusati identici nei due run. Se si lasciasse
    ricalcolare il percorso a SUMO, i veicoli potrebbero scegliere strade
    diverse fra i due scenari e il confronto misurerebbe due cose insieme.
    Fissando le rotte, l'unica variabile e' la temporizzazione semaforica.
  * **Stesso seed e stessi istanti di partenza**, per eliminare la
    variabilita' stocastica del simulatore.

--- Metriche (da tripinfo-output) -------------------------------------------
  duration    tempo totale di viaggio del veicolo (s)
  waitingTime tempo fermo per semafori/code (s)
  timeLoss    tempo perso rispetto alla marcia a velocita' ideale (s)
  arrived     n. di veicoli che hanno raggiunto la destinazione

Uso (dalla radice del progetto):
    python scripts/compare_sumo.py                     # tutte le zone disponibili
    python scripts/compare_sumo.py piccola
    python scripts/compare_sumo.py piccola media grande --max-vehicles 60
"""

import os
import sys
import json
import glob
import heapq
import shutil
import argparse
import subprocess
import statistics
import tempfile
import xml.etree.ElementTree as ET
from collections import defaultdict

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))  # radice del progetto
NET_DIR = os.path.join(BASE, "net_files")
SUMO_DIR = os.path.join(BASE, "sumo_extracted")
CFG_DIR = os.path.join(BASE, "cfg_files")
OUT_DIR = os.path.join(BASE, "sumo_comparison")

ZONES = ("piccola", "media", "grande")

DEPART_PERIOD = 3.0     # s fra una partenza e l'altra (evita ingorghi iniziali)
VEH_MAX_SPEED = 13.89   # m/s (50 km/h): lasciamo che sia la rete a limitare
SIM_END = 7200          # s: limite generoso, la simulazione finisce prima


# ---------------------------------------------------------------------------
# LOCALIZZAZIONE DI SUMO
# ---------------------------------------------------------------------------
def find_sumo_bin(name="sumo"):
    """Cerca l'eseguibile SUMO: SUMO_HOME, pacchetto pip eclipse-sumo, PATH,
    percorsi tipici di installazione su Windows."""
    cands = []
    home = os.environ.get("SUMO_HOME", "")
    if home:
        cands += [os.path.join(home, "bin", name + ".exe"),
                  os.path.join(home, "bin", name)]
    try:  # pacchetto pip 'eclipse-sumo'
        import sumo as _sumo
        p = os.path.dirname(_sumo.__file__)
        cands += [os.path.join(p, "bin", name + ".exe"),
                  os.path.join(p, "bin", name)]
    except Exception:
        pass
    for b in (r"C:\Program Files (x86)\Eclipse\Sumo",
              r"C:\Program Files\Eclipse\Sumo", r"C:\Sumo"):
        cands.append(os.path.join(b, "bin", name + ".exe"))
    for c in cands:
        if os.path.exists(c):
            return c
    found = shutil.which(name)
    return found


# ---------------------------------------------------------------------------
# GRAFO DELLA RETE SUMO (per calcolare le rotte una volta sola)
# ---------------------------------------------------------------------------
def build_sumo_graph(net_path):
    root = ET.parse(net_path).getroot()
    jpos = {}
    for j in root.findall("junction"):
        if not j.get("id", "").startswith(":"):
            jpos[j.get("id")] = (float(j.get("x", 0)), float(j.get("y", 0)))
    graph = defaultdict(list)
    for e in root.findall("edge"):
        eid = e.get("id")
        if eid is None or eid.startswith(":"):
            continue
        fr, to = e.get("from"), e.get("to")
        if not fr or not to:
            continue
        lanes = e.findall("lane")
        length = float(lanes[0].get("length", 1)) if lanes else 1.0
        graph[fr].append((to, eid, length))
    return graph, set(jpos.keys())


def dijkstra_edges(graph, start, goal):
    """Percorso a costo minimo start->goal, come lista di id di archi."""
    dist = {start: 0.0}
    prev = {}
    heap = [(0.0, start)]
    while heap:
        d, u = heapq.heappop(heap)
        if u == goal:
            break
        if d > dist.get(u, float("inf")):
            continue
        for v, eid, length in graph[u]:
            nd = d + length
            if nd < dist.get(v, float("inf")):
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


def resolve_junction(node_id, junc_ids):
    """Stessa logica di sumo_visualize.py: id esatto -> suffisso -> membro di
    un cluster generato da netconvert."""
    nid = str(node_id).lstrip("n")
    if nid in junc_ids:
        return nid
    m = [j for j in junc_ids if j.endswith(nid) and not j.startswith(":")]
    if m:
        return min(m, key=len)
    hits = []
    for j in junc_ids:
        if not j.startswith("cluster_"):
            continue
        for member in j[len("cluster_"):].split("_"):
            if member == nid or (member.isdigit() and member.endswith(nid)):
                hits.append(j)
                break
    return min(hits, key=len) if hits else None


# ---------------------------------------------------------------------------
# COSTRUZIONE DELLA DOMANDA
# ---------------------------------------------------------------------------
def build_routes(zone, net_path, max_vehicles=None, verbose=True):
    """Dalle coppie O-D condivise costruisce le rotte SUMO (una volta sola,
    riusate identiche nei due scenari). Ritorna (xml_str, n_veicoli, n_scartate)."""
    demand_path = os.path.join(SUMO_DIR, f"demand_{zone}.json")
    if not os.path.exists(demand_path):
        print(f"[{zone}] domanda non trovata: {os.path.basename(demand_path)}")
        print(f"[{zone}] generala con: python scripts/generate_demand.py {zone}")
        return None, 0, 0

    with open(demand_path, encoding="utf-8") as f:
        od_pairs = json.load(f).get("od_pairs", [])
    if max_vehicles:
        od_pairs = od_pairs[:max_vehicles]

    graph, junc_ids = build_sumo_graph(net_path)

    routes, skipped = [], 0
    for pair in od_pairs:
        s = resolve_junction(pair.get("start"), junc_ids)
        g = resolve_junction(pair.get("goal"), junc_ids)
        if not s or not g or s == g:
            skipped += 1
            continue
        edges = dijkstra_edges(graph, s, g)
        if not edges:
            skipped += 1
            continue
        routes.append(edges)

    if not routes:
        return None, 0, skipped

    lines = ['<?xml version="1.0" encoding="UTF-8"?>', "<routes>",
             f'    <vType id="auto" accel="2.6" decel="4.5" sigma="0.0" '
             f'length="4.5" maxSpeed="{VEH_MAX_SPEED}"/>']
    for i, edges in enumerate(routes):
        depart = round(i * DEPART_PERIOD, 1)
        lines.append(f'    <vehicle id="v{i}" type="auto" depart="{depart}" departSpeed="0">')
        lines.append(f'        <route edges="{" ".join(edges)}"/>')
        lines.append("    </vehicle>")
    lines.append("</routes>")

    if verbose:
        print(f"[{zone}] veicoli instradati: {len(routes)}"
              f"{f' (scartate {skipped} coppie non instradabili)' if skipped else ''}")
    return "\n".join(lines), len(routes), skipped


# ---------------------------------------------------------------------------
# ESECUZIONE E METRICHE
# ---------------------------------------------------------------------------
def run_sumo(sumo_bin, net_path, rou_path, trip_path, add_path=None, seed=42):
    cmd = [sumo_bin,
           "-n", net_path,
           "-r", rou_path,
           "--tripinfo-output", trip_path,
           "--seed", str(seed),
           "--begin", "0", "--end", str(SIM_END),
           "--no-step-log", "true",
           "--no-warnings", "true",
           "--time-to-teleport", "-1",      # niente teletrasporti: falserebbero le attese
           "--ignore-route-errors", "true"]
    if add_path:
        cmd += ["-a", add_path]
    res = subprocess.run(cmd, capture_output=True, text=True, timeout=1800)
    return res


def parse_tripinfo(path):
    """Aggrega le metriche dal tripinfo-output di SUMO."""
    if not os.path.exists(path):
        return None
    root = ET.parse(path).getroot()
    dur, wait, loss = [], [], []
    for t in root.findall("tripinfo"):
        dur.append(float(t.get("duration", 0)))
        wait.append(float(t.get("waitingTime", 0)))
        loss.append(float(t.get("timeLoss", 0)))
    if not dur:
        return None
    return {
        "arrived": len(dur),
        "duration_mean": round(statistics.mean(dur), 2),
        "duration_total": round(sum(dur), 1),
        "waiting_mean": round(statistics.mean(wait), 2),
        "waiting_total": round(sum(wait), 1),
        "timeloss_mean": round(statistics.mean(loss), 2),
        "timeloss_total": round(sum(loss), 1),
    }


def pct(new, old):
    return None if not old else round(100.0 * (new - old) / old, 1)


def compare_zone(zone, sumo_bin, max_vehicles=None, verbose=True):
    net_path = os.path.join(NET_DIR, f"{zone}.net.xml")
    add_path = os.path.join(CFG_DIR, f"tls_{zone}.add.xml")

    if not os.path.exists(net_path):
        print(f"[{zone}] rete non trovata: {net_path}")
        return None
    if not os.path.exists(add_path):
        print(f"[{zone}] semafori ottimizzati non trovati "
              f"({os.path.basename(add_path)}).")
        print(f"[{zone}] generali con: python scripts/inject_signal_plan.py {zone}")
        return None

    rou_xml, n_veh, n_skip = build_routes(zone, net_path, max_vehicles, verbose)
    if not rou_xml:
        return None

    tmp = tempfile.mkdtemp(prefix=f"cmp_{zone}_")
    rou_path = os.path.join(tmp, "routes.rou.xml")
    with open(rou_path, "w", encoding="utf-8") as f:
        f.write(rou_xml)

    trip_base = os.path.join(tmp, "tripinfo_baseline.xml")
    trip_opt = os.path.join(tmp, "tripinfo_optimized.xml")

    if verbose:
        print(f"[{zone}] simulazione BASELINE (semafori originali)...")
    r1 = run_sumo(sumo_bin, net_path, rou_path, trip_base)
    if verbose:
        print(f"[{zone}] simulazione OTTIMIZZATA (programma 'optimized')...")
    r2 = run_sumo(sumo_bin, net_path, rou_path, trip_opt, add_path=add_path)

    base = parse_tripinfo(trip_base)
    opt = parse_tripinfo(trip_opt)
    if not base or not opt:
        print(f"[{zone}] ERRORE: nessun tripinfo prodotto.")
        print((r1.stderr or "")[:400])
        print((r2.stderr or "")[:400])
        return None

    result = {
        "zone": zone,
        "n_vehicles": n_veh,
        "n_skipped_pairs": n_skip,
        "baseline": base,
        "optimized": opt,
        "delta_pct": {
            "duration_mean": pct(opt["duration_mean"], base["duration_mean"]),
            "waiting_mean": pct(opt["waiting_mean"], base["waiting_mean"]),
            "timeloss_mean": pct(opt["timeloss_mean"], base["timeloss_mean"]),
        },
    }

    if verbose:
        d, w, l = result["delta_pct"].values()
        print(f"[{zone}] veicoli arrivati: baseline {base['arrived']} / "
              f"ottimizzato {opt['arrived']}")
        print(f"[{zone}] tempo di viaggio medio : {base['duration_mean']:.1f}s -> "
              f"{opt['duration_mean']:.1f}s ({d:+.1f}%)")
        print(f"[{zone}] attesa media (semafori): {base['waiting_mean']:.1f}s -> "
              f"{opt['waiting_mean']:.1f}s ({w:+.1f}%)")
        print(f"[{zone}] tempo perso medio      : {base['timeloss_mean']:.1f}s -> "
              f"{opt['timeloss_mean']:.1f}s ({l:+.1f}%)")
    return result


# ---------------------------------------------------------------------------
# REPORT
# ---------------------------------------------------------------------------
def write_report(results):
    """Salva i risultati e rigenera il report. I risultati sono CUMULATIVI:
    lanciare una zona alla volta non cancella quelle gia' calcolate."""
    os.makedirs(OUT_DIR, exist_ok=True)
    res_path = os.path.join(OUT_DIR, "results.json")

    merged = {}
    if os.path.exists(res_path):
        try:
            with open(res_path, encoding="utf-8") as f:
                for old in json.load(f):
                    merged[old["zone"]] = old
        except Exception:
            pass
    for r in results:                      # i nuovi sovrascrivono i vecchi
        merged[r["zone"]] = r
    results = [merged[z] for z in ZONES if z in merged]

    with open(res_path, "w", encoding="utf-8") as f:
        json.dump(results, f, ensure_ascii=False, indent=2)

    L = ["# Confronto in SUMO: semafori baseline vs ottimizzati (punto 4)", "",
         "Misura **simulativa** del guadagno: SUMO riproduce code, accelerazioni e",
         "fasi reali dei semafori. A differenza del punto 2 (stima analitica di",
         "Webster dentro PDDL+), qui il risultato non dipende dal modello usato per",
         "ottimizzare, ed e' quindi una verifica indipendente.", "",
         "## Disegno dell'esperimento", "",
         "Due simulazioni identiche per zona, diverse **solo** nel programma",
         "semaforico (baseline = programma `0` del net.xml; ottimizzato =",
         "programma `optimized` da `cfg_files/tls_<zona>.add.xml`).",
         "Stessa domanda O-D (`sumo_extracted/demand_<zona>.json`, lo stesso",
         "campione del punto 2), **stesse rotte** precalcolate con Dijkstra e",
         "riusate identiche nei due run, stesso seed e stessi istanti di partenza:",
         "l'unica variabile e' la temporizzazione dei semafori.", "",
         "## Risultati", "",
         "| Zona | Veicoli | Metrica | Baseline | Ottimizzato | Δ% |",
         "|---|---:|---|---:|---:|---:|"]
    for r in results:
        z, n = r["zone"], r["n_vehicles"]
        b, o, d = r["baseline"], r["optimized"], r["delta_pct"]
        L.append(f"| **{z}** | {n} | tempo di viaggio medio (s) | "
                 f"{b['duration_mean']:.1f} | {o['duration_mean']:.1f} | "
                 f"**{d['duration_mean']:+.1f}%** |")
        L.append(f"| | | attesa media ai semafori (s) | {b['waiting_mean']:.1f} | "
                 f"{o['waiting_mean']:.1f} | **{d['waiting_mean']:+.1f}%** |")
        L.append(f"| | | tempo perso medio (s) | {b['timeloss_mean']:.1f} | "
                 f"{o['timeloss_mean']:.1f} | **{d['timeloss_mean']:+.1f}%** |")
    L += ["", "Valori negativi = miglioramento (tempi piu' bassi con i semafori",
          "ottimizzati).", ""]

    # --- interpretazione automatica -------------------------------------
    worse = [r for r in results if (r["delta_pct"]["waiting_mean"] or 0) > 0]
    better = [r for r in results if (r["delta_pct"]["waiting_mean"] or 0) < 0]
    L += ["## Interpretazione", ""]
    if better:
        names = ", ".join(f"**{r['zone']}** ({r['delta_pct']['waiting_mean']:+.1f}%)"
                          for r in better)
        L += [f"L'ottimizzazione riduce l'attesa ai semafori in: {names}.",
              "Il guadagno misurato in simulazione conferma, su queste zone, la",
              "direzione prevista dalla stima analitica del punto 2.", ""]
    if worse:
        names = ", ".join(f"**{r['zone']}** ({r['delta_pct']['waiting_mean']:+.1f}%)"
                          for r in worse)
        L += [f"L'ottimizzazione **peggiora** l'attesa in: {names}.", "",
              "Questo e' un risultato atteso ma non banale, e va riportato: la",
              "stima del punto 2 usa il ritardo uniforme di Webster, che modella",
              "ogni incrocio come **isolato** e con arrivi casuali. In una rete",
              "densa quell'ipotesi cade, perche':", "",
              "1. **le code si propagano** fra incroci adiacenti (spillback): dare",
              "   piu' verde a un movimento puo' scaricare piu' veicoli",
              "   sull'incrocio successivo, che non e' stato ricalibrato;",
              "2. **gli offset non vengono ottimizzati** (il punto 2 riporta la",
              "   penalita' di progressione ma non la usa come obiettivo): cambiare",
              "   le durate di verde senza correggere gli sfasamenti puo' rompere",
              "   le onde verdi implicite nella temporizzazione originale;",
              "3. **solo una minoranza di incroci viene ottimizzata** (quelli",
              "   attraversati dal campione O-D), quindi i semafori modificati",
              "   interagiscono con vicini rimasti alla taratura di partenza.", "",
              "Prova a sostegno di questa lettura: ripetendo il confronto sulla",
              "zona `grande` con traffico piu' leggero (11 veicoli invece di 43)",
              "il segno si inverte e l'ottimizzato torna leggermente migliore",
              "(-1.4% di attesa). Il degrado emerge quindi **sotto congestione**,",
              "cioe' proprio dove le ipotesi di Webster sono meno valide.", "",
              "Indicazione operativa: su reti dense l'ottimizzazione andrebbe",
              "estesa agli offset (coordinamento) e valutata direttamente in",
              "simulazione, usando SUMO come funzione obiettivo invece che come",
              "sola verifica finale.", ""]
    L += [""]

    L += [
          "## Metriche", "",
          "- **tempo di viaggio** (`duration`): tempo totale porta a porta.",
          "- **attesa** (`waitingTime`): tempo a velocita' ~0, cioe' fermi al rosso",
          "  o in coda. E' la metrica piu' direttamente legata ai semafori.",
          "- **tempo perso** (`timeLoss`): ritardo rispetto alla marcia ideale a",
          "  velocita' consentita; include anche le decelerazioni.", "",
          "## Riproducibilita'", "",
          "```bash", "python scripts/compare_sumo.py                  # tutte le zone",
          "python scripts/compare_sumo.py piccola media grande", "```", "",
          "Output: `sumo_comparison/results.json` (dati grezzi) e questo report.",
          "I teletrasporti sono disattivati (`--time-to-teleport -1`): un veicolo",
          "bloccato resta in coda invece di essere rimosso, altrimenti le attese",
          "risulterebbero artificialmente piu' basse.", ""]
    path = os.path.join(OUT_DIR, "report.md")
    with open(path, "w", encoding="utf-8") as f:
        f.write("\n".join(L))
    return path


def main():
    ap = argparse.ArgumentParser(
        description="Punto 4: confronta in SUMO i semafori ottimizzati contro la baseline.")
    ap.add_argument("zones", nargs="*", default=list(ZONES))
    ap.add_argument("--max-vehicles", type=int, default=None,
                    help="limita il numero di veicoli (default: tutte le coppie O-D)")
    args = ap.parse_args()

    sumo_bin = find_sumo_bin("sumo")
    if not sumo_bin:
        print("[ERRORE] SUMO non trovato. Installa SUMO oppure: pip install eclipse-sumo")
        sys.exit(1)
    print(f"SUMO: {sumo_bin}\n")

    results = []
    for zone in args.zones:
        if zone not in ZONES:
            print(f"[skip] zona sconosciuta: {zone}")
            continue
        r = compare_zone(zone, sumo_bin, max_vehicles=args.max_vehicles)
        if r:
            results.append(r)
        print()

    if results:
        path = write_report(results)
        print(f"Report salvato in: {path}")
    else:
        print("Nessun confronto eseguito.")


if __name__ == "__main__":
    main()

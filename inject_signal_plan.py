"""
inject_signal_plan.py
=====================
PUNTO 3 della roadmap — iniezione del piano semaforico ottimizzato in SUMO.

Prende il piano prodotto dal punto 2 (`signal_optimization/optimize.py`) e lo
traduce in un *additional-file* SUMO contenente i `<tlLogic>` con le durate di
fase ottimizzate, pronto da caricare in sumo/sumo-gui.

    net_files/<zona>.net.xml                 (tlLogic originali)
    sumo_extracted/signal_plan_<zona>.json   (durate ottimizzate)
                    |
                    v
    cfg_files/tls_<zona>.add.xml             (<additional> con i tlLogic)

--- Semantica SUMO (documentazione ufficiale) -------------------------------
Dalla doc SUMO (Simulation/Traffic_Lights, "Defining New TLS-Programs"):

  "You can load new definitions for traffic lights as a part of an
   additional-file. When loaded, the last program will be used."

Quindi il programma definito qui diventa AUTOMATICAMENTE quello attivo, senza
bisogno di WAUT o TraCI. Requisiti imposti dalla doc e rispettati da questo
script:
  - `id` deve essere un id di semaforo gia' esistente nel .net.xml;
  - `programID` deve essere un nome NUOVO per quel semaforo (non "0", che e'
    quello originale; "off" e' riservato).

Effetto collaterale utile: siccome il programma originale "0" resta caricato,
dalla GUI di SUMO si puo' passare da uno all'altro col tasto destro sul
semaforo -> "Switch TLS program", per confrontare a occhio baseline e
ottimizzato sulla stessa simulazione.

--- Formato del piano in ingresso -------------------------------------------
    {"plan": {"<tlLogic_id>": {"<phase_idx>": durata_s, ...}, ...}}
Le fasi NON presenti nel piano mantengono la durata originale del net.xml
(cosi' come documentato nel file prodotto dal punto 2).

Uso:
    python inject_signal_plan.py                    # tutte le zone disponibili
    python inject_signal_plan.py piccola            # una zona
    python inject_signal_plan.py piccola media grande
    python inject_signal_plan.py piccola --program-id opt2
"""

import os
import sys
import json
import argparse
import xml.etree.ElementTree as ET
from xml.dom import minidom

BASE = os.path.dirname(os.path.abspath(__file__))
NET_DIR = os.path.join(BASE, "net_files")
SUMO_DIR = os.path.join(BASE, "sumo_extracted")
CFG_DIR = os.path.join(BASE, "cfg_files")

ZONES = ("piccola", "media", "grande")

# programID del programma ottimizzato: deve essere DIVERSO da quello del
# net.xml (tipicamente "0") perche' SUMO lo consideri un programma nuovo.
DEFAULT_PROGRAM_ID = "optimized"


# ---------------------------------------------------------------------------
# LETTURA
# ---------------------------------------------------------------------------
def read_net_tls(net_path):
    """Legge i tlLogic dal net.xml.
    Ritorna {tls_id: {"type","programID","offset","phases":[dict_attributi]}}.
    Di ogni fase conserviamo TUTTI gli attributi originali (state, minDur,
    maxDur, name, ...), cosi' l'iniezione sovrascrive solo la durata."""
    root = ET.parse(net_path).getroot()
    tls = {}
    for t in root.findall("tlLogic"):
        phases = [dict(p.attrib) for p in t.findall("phase")]
        tls[t.get("id")] = {
            "type": t.get("type", "static"),
            "programID": t.get("programID", "0"),
            "offset": t.get("offset", "0"),
            "phases": phases,
        }
    return tls


def read_signal_plan(plan_path):
    """Legge signal_plan_<zona>.json -> {tls_id: {phase_idx(int): durata}}."""
    with open(plan_path, encoding="utf-8") as f:
        data = json.load(f)
    raw = data.get("plan", {})
    plan = {}
    for tls_id, phases in raw.items():
        try:
            plan[tls_id] = {int(k): float(v) for k, v in phases.items()}
        except (TypeError, ValueError):
            continue
    return plan, data.get("report", {})


# ---------------------------------------------------------------------------
# COSTRUZIONE ADDITIONAL-FILE
# ---------------------------------------------------------------------------
def fmt_duration(x):
    """Durata SUMO: intero se e' intero, altrimenti 2 decimali."""
    return str(int(round(x))) if abs(x - round(x)) < 1e-9 else f"{x:.2f}"


def build_additional(net_tls, plan, program_id=DEFAULT_PROGRAM_ID):
    """Costruisce l'albero XML <additional> con i tlLogic ottimizzati.
    Ritorna (elemento_root, statistiche)."""
    root = ET.Element("additional")
    stats = {"written": 0, "changed_phases": 0, "skipped_missing": [],
             "skipped_bad_index": [], "unchanged_tls": 0}

    for tls_id in sorted(plan.keys()):
        base = net_tls.get(tls_id)
        if base is None:
            # il piano cita un semaforo che non esiste nella rete
            stats["skipped_missing"].append(tls_id)
            continue

        new_durations = plan[tls_id]
        n_phases = len(base["phases"])
        bad = [i for i in new_durations if i < 0 or i >= n_phases]
        if bad:
            stats["skipped_bad_index"].append((tls_id, bad, n_phases))
            continue

        # il programID nuovo deve differire da quello originale
        pid = program_id
        if pid == base["programID"]:
            pid = f"{program_id}_1"

        tl_el = ET.SubElement(root, "tlLogic", {
            "id": tls_id,
            "type": base["type"],
            "programID": pid,
            "offset": base["offset"],
        })

        changed_here = 0
        for idx, attrib in enumerate(base["phases"]):
            new_attrib = dict(attrib)  # conserva state e ogni altro attributo
            if idx in new_durations:
                old = float(attrib.get("duration", 0))
                new = new_durations[idx]
                new_attrib["duration"] = fmt_duration(new)
                if abs(old - new) > 1e-6:
                    changed_here += 1
            ET.SubElement(tl_el, "phase", new_attrib)

        stats["written"] += 1
        stats["changed_phases"] += changed_here
        if changed_here == 0:
            stats["unchanged_tls"] += 1

    return root, stats


def write_additional(root, out_path, zone, program_id, report):
    """Serializza con indentazione leggibile e un commento di intestazione."""
    rough = ET.tostring(root, encoding="unicode")
    pretty = minidom.parseString(rough).toprettyxml(indent="    ")
    # minidom aggiunge la sua dichiarazione: la sostituiamo con una nostra
    body = "\n".join(line for line in pretty.splitlines()[1:] if line.strip())

    bm = report.get("baseline_mean_metric")
    om = report.get("optimized_mean_metric")
    delta_txt = ""
    if isinstance(bm, (int, float)) and isinstance(om, (int, float)):
        delta_txt = (f"\n     baseline mean total-time = {bm:.2f} s"
                     f" -> ottimizzato = {om:.2f} s ({om - bm:+.2f} s)")

    header = (
        '<?xml version="1.0" encoding="UTF-8"?>\n'
        f"<!-- Programmi semaforici OTTIMIZZATI per la zona '{zone}'.\n"
        f"     Generato da inject_signal_plan.py (punto 3) a partire da\n"
        f"     sumo_extracted/signal_plan_{zone}.json (punto 2).\n"
        f"     programID='{program_id}': essendo caricato da additional-file,\n"
        f"     SUMO lo rende il programma attivo. Dal tasto destro sul semaforo\n"
        f"     in sumo-gui si puo' tornare al programma originale '0'.{delta_txt}\n"
        f"-->\n"
    )
    os.makedirs(os.path.dirname(out_path), exist_ok=True)
    with open(out_path, "w", encoding="utf-8") as f:
        f.write(header + body + "\n")


# ---------------------------------------------------------------------------
# PIPELINE
# ---------------------------------------------------------------------------
def inject_zone(zone, program_id=DEFAULT_PROGRAM_ID, verbose=True):
    net_path = os.path.join(NET_DIR, f"{zone}.net.xml")
    plan_path = os.path.join(SUMO_DIR, f"signal_plan_{zone}.json")
    out_path = os.path.join(CFG_DIR, f"tls_{zone}.add.xml")

    if not os.path.exists(net_path):
        print(f"[{zone}] ERRORE: rete non trovata: {net_path}")
        return None
    if not os.path.exists(plan_path):
        print(f"[{zone}] piano non trovato ({os.path.basename(plan_path)}).")
        print(f"[{zone}] generalo prima con: "
              f"python -m signal_optimization.optimize {zone}")
        return None

    net_tls = read_net_tls(net_path)
    plan, report = read_signal_plan(plan_path)
    root, stats = build_additional(net_tls, plan, program_id)

    if stats["written"] == 0:
        print(f"[{zone}] nessun tlLogic scritto — controlla il piano.")
        return None

    write_additional(root, out_path, zone, program_id, report)

    if verbose:
        print(f"[{zone}] semafori nella rete: {len(net_tls)} | nel piano: {len(plan)}")
        print(f"[{zone}] tlLogic scritti: {stats['written']} "
              f"(fasi con durata modificata: {stats['changed_phases']}, "
              f"semafori invariati: {stats['unchanged_tls']})")
        if stats["skipped_missing"]:
            print(f"[{zone}] ATTENZIONE: {len(stats['skipped_missing'])} id del piano "
                  f"non presenti nella rete (ignorati): {stats['skipped_missing'][:5]}")
        if stats["skipped_bad_index"]:
            print(f"[{zone}] ATTENZIONE: indici di fase fuori range (ignorati): "
                  f"{stats['skipped_bad_index'][:3]}")
        print(f"[{zone}] salvato: {out_path}")
    return out_path


def main():
    ap = argparse.ArgumentParser(
        description="Punto 3: inietta il piano semaforico ottimizzato in SUMO "
                    "generando un additional-file con i tlLogic.")
    ap.add_argument("zones", nargs="*", default=list(ZONES),
                    help="zone da elaborare (default: tutte)")
    ap.add_argument("--program-id", default=DEFAULT_PROGRAM_ID,
                    help=f"programID del programma ottimizzato (default: {DEFAULT_PROGRAM_ID})")
    args = ap.parse_args()

    done = []
    for zone in args.zones:
        if zone not in ZONES:
            print(f"[skip] zona sconosciuta: {zone}")
            continue
        out = inject_zone(zone, program_id=args.program_id)
        if out:
            done.append(out)

    print()
    if done:
        print(f"Fatti {len(done)} file. Per usarli, il .sumocfg deve contenere:")
        print('    <additional-files value="..._tls_<zona>.add.xml"/>')
        print("(sumo_visualize.py lo aggiunge automaticamente se il file esiste)")
    else:
        print("Nessun file generato.")


if __name__ == "__main__":
    main()

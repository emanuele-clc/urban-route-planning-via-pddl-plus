#!/usr/bin/env python3
"""
osm_to_pddl.py
==============
Project #2 – Map Construction in PDDL+
Automated Planning – Master CSAI, UNICAL 2026
Gruppo: Chiara, Elisa, Emanuele, Pierluigi

Pipeline:
  1. Scarica il grafo stradale da OpenStreetMap (OSMnx)
  2. Semplifica il grafo (elimina nodi intermedi, tieni solo gli incroci)
  3. Estrae un sottografo di dimensione gestibile attorno al centro città
  4. Genera:
       output/domain.pddl   – dominio PDDL+
       output/problem.pddl  – problema PDDL+ (start→goal)
       output/node_map.json – mappa nodi OSM ↔ nomi PDDL (con coordinate GPS)
       output/map.png       – visualizzazione del grafo (se matplotlib disponibile)

Dipendenze (vedi requirements.txt):
  pip install osmnx networkx matplotlib

Uso:
  python osm_to_pddl.py
  python osm_to_pddl.py --city "Rende, Calabria, Italy" --max-nodes 30
"""

import argparse
import json
import sys
from pathlib import Path

# ──────────────────────────────────────────────────────────────────────────────
# CONFIGURAZIONE DEFAULT
# ──────────────────────────────────────────────────────────────────────────────
DEFAULT_CITY     = "Cosenza, Calabria, Italy"
DEFAULT_MAX_NODES = 40          # nodi massimi nel problema PDDL (per trattabilità)
DEFAULT_SPEED_KPH = 50.0        # velocità di default se OSM non la specifica
OUTPUT_DIR        = Path("output")


# ──────────────────────────────────────────────────────────────────────────────
# STEP 1 – Estrazione e semplificazione del grafo
# ──────────────────────────────────────────────────────────────────────────────
def extract_graph(city_name: str, max_nodes: int):
    """
    Scarica la rete stradale (solo strade percorribili in auto) da OSM,
    semplifica il grafo consolidando i nodi intermedi, e se necessario
    taglia un sottografo attorno al centro città.

    Ritorna un MultiDiGraph di OSMnx con attributi 'length', 'speed_kph',
    'travel_time' su ogni arco.
    """
    try:
        import osmnx as ox
        import networkx as nx
    except ImportError:
        sys.exit("❌  Installa le dipendenze: pip install osmnx networkx matplotlib")

    print(f"\n[1/4] Download rete stradale: {city_name}")
    # OSMnx >= 2.x semplifica il grafo automaticamente durante il download
    # (simplify=True è il default), quindi non serve chiamare simplify_graph()
    G = ox.graph_from_place(city_name, network_type="drive", simplify=True)
    print(f"      Grafo semplificato:{len(G.nodes):>4} nodi  |  {len(G.edges):>6} archi")

    print("[2/4] Aggiunta dati velocità...")

    # Aggiunge velocità (usa tag maxspeed di OSM, default dove manca) e tempi
    G = ox.add_edge_speeds(G, fallback=DEFAULT_SPEED_KPH)
    G = ox.add_edge_travel_times(G)

    # Se il grafo è ancora troppo grande, prendiamo un sottografo attorno al centro
    if len(G.nodes) > max_nodes:
        print(f"[3/4] Grafo troppo grande – estraggo sottografo di {max_nodes} nodi"
              f" attorno al centro città...")
        center_lat, center_lon = ox.geocode(city_name)
        center_node = ox.nearest_nodes(G, center_lon, center_lat)

        # BFS dal centro, prendiamo i primi max_nodes nodi raggiunti
        reachable = dict(
            nx.single_source_shortest_path_length(G.to_undirected(), center_node)
        )
        # Ordina per distanza (hop count) e prendi i più vicini
        selected = sorted(reachable, key=reachable.get)[:max_nodes]
        G = G.subgraph(selected).copy()
        # Rimuovi eventuali nodi isolati rimasti
        isolated = [n for n in G.nodes() if G.degree(n) == 0]
        G.remove_nodes_from(isolated)
        print(f"      Sottografo:        {len(G.nodes):>4} nodi  |  {len(G.edges):>6} archi")
    else:
        print("[3/4] Dimensione OK – grafo semplificato usato integralmente.")

    return G


# ──────────────────────────────────────────────────────────────────────────────
# STEP 2 – Generazione dominio PDDL+
# ──────────────────────────────────────────────────────────────────────────────
def generate_domain() -> str:
    """
    Ritorna il testo del dominio PDDL+ per la navigazione veicolo A→B.

    Modello:
      - Tipi:       location (intersezioni OSM), vehicle (agente)
      - Predicati:  (at v l)         veicolo v si trova in l
                    (road from to)   esiste una strada diretta da from a to
      - Fluenti:    road-length      lunghezza arco [m]
                    speed-limit      limite di velocità [m/s]
                    total-distance   odometro [m]
                    travel-time      tempo trascorso [s]
      - Azione:     drive – azione durativa; durata = length / speed
    """
    return """\
;; DOMAIN: map-navigation  (PDDL+)
;;
;; Proper PDDL+ encoding using action / process / event.
;;
;; start-drive  (action)   : vehicle begins traversing a road segment
;; moving       (process)  : continuous motion along the road
;; arrive       (event)    : vehicle reaches next intersection

(define (domain map-navigation)

  (:requirements :typing :numeric-fluents :negative-preconditions :time)

  (:types location vehicle)

  (:predicates
    (at      ?v - vehicle ?l - location)
    (road    ?from - location ?to - location)
    (driving ?v - vehicle ?from - location ?to - location)
    (free    ?v - vehicle)
  )

  (:functions
    (road-length  ?from - location ?to - location)
    (speed-limit  ?from - location ?to - location)
    (position     ?v - vehicle)
    (total-distance ?v - vehicle)
    (travel-time  ?v - vehicle)
  )

  ;; Instantaneous action: begin driving from ?from toward ?to
  (:action start-drive
    :parameters (?v - vehicle ?from - location ?to - location)
    :precondition (and
      (at ?v ?from)
      (road ?from ?to)
      (free ?v)
    )
    :effect (and
      (not (at ?v ?from))
      (not (free ?v))
      (driving ?v ?from ?to)
      (assign (position ?v) 0)
    )
  )

  ;; Continuous process: vehicle moves along the road at speed-limit
  (:process moving
    :parameters (?v - vehicle ?from - location ?to - location)
    :precondition (and
      (driving ?v ?from ?to)
      (< (position ?v) (road-length ?from ?to))
    )
    :effect (and
      (increase (position ?v)       (* #t (speed-limit ?from ?to)))
      (increase (total-distance ?v) (* #t (speed-limit ?from ?to)))
      (increase (travel-time ?v)    #t)
    )
  )

  ;; Event: vehicle has covered the full road length -> arrives
  (:event arrive
    :parameters (?v - vehicle ?from - location ?to - location)
    :precondition (and
      (driving ?v ?from ?to)
      (>= (position ?v) (road-length ?from ?to))
    )
    :effect (and
      (not (driving ?v ?from ?to))
      (at ?v ?to)
      (free ?v)
      (assign (position ?v) 0)
    )
  )

)
"""


# ──────────────────────────────────────────────────────────────────────────────
# STEP 3 – Generazione problema PDDL+
# ──────────────────────────────────────────────────────────────────────────────
def generate_problem(G, start_node: int, goal_node: int, node_map: dict) -> str:
    """
    Genera il testo del problema PDDL+ a partire dal grafo OSMnx.

    node_map: dizionario  osm_id → nome_pddl (es. 123456 → 'loc007')
    """
    lines = []
    lines.append(";; Problema generato automaticamente da osm_to_pddl.py")
    lines.append(f";; Città: {DEFAULT_CITY}")
    lines.append(f";; Nodi: {len(G.nodes)}  |  Archi: {len(G.edges)}")
    lines.append("")
    lines.append("(define (problem navigate-cosenza)")
    lines.append("  (:domain map-navigation)")
    lines.append("")

    # ── Oggetti ──
    loc_names = " ".join(node_map[n] for n in sorted(G.nodes()))
    lines.append("  (:objects")
    # Spezza su più righe se troppo lungo
    chunk = []
    for n in sorted(G.nodes()):
        chunk.append(node_map[n])
        if len(" ".join(chunk)) > 70:
            lines.append("    " + " ".join(chunk) + " - location")
            chunk = []
    if chunk:
        lines.append("    " + " ".join(chunk) + " - location")
    lines.append("    vehicle1 - vehicle")
    lines.append("  )")
    lines.append("")

    # ── Init ──
    lines.append("  (:init")
    lines.append(f"    ;; Initial vehicle state")
    lines.append(f"    (at vehicle1 {node_map[start_node]})")
    lines.append(f"    (free vehicle1)")
    lines.append(f"    (= (position       vehicle1) 0)")
    lines.append(f"    (= (total-distance vehicle1) 0)")
    lines.append(f"    (= (travel-time    vehicle1) 0)")
    lines.append("")
    lines.append("    ;; Road segments: predicate + length [m] + speed-limit [m/s]")

    for u, v, data in sorted(G.edges(data=True)):
        if u not in node_map or v not in node_map:
            continue
        lu, lv = node_map[u], node_map[v]
        length    = float(data.get("length",    100.0))
        speed_kph = float(data.get("speed_kph", DEFAULT_SPEED_KPH))
        speed_ms  = max(round(speed_kph / 3.6, 4), 0.5)

        lines.append(f"    (road {lu} {lv})")
        lines.append(f"    (= (road-length  {lu} {lv}) {length:.1f})")
        lines.append(f"    (= (speed-limit  {lu} {lv}) {speed_ms:.4f})")

    lines.append("  )")
    lines.append("")

    # ── Goal ──
    lines.append("  (:goal")
    lines.append(f"    (at vehicle1 {node_map[goal_node]})")
    lines.append("  )")
    lines.append("")

    # ── Metrica ──
    lines.append("  (:metric minimize (travel-time vehicle1))")
    lines.append(")")

    return "\n".join(lines)


# ──────────────────────────────────────────────────────────────────────────────
# STEP 4 – Esporta mappa nodi come JSON
# ──────────────────────────────────────────────────────────────────────────────
def export_node_map(G, node_map: dict) -> dict:
    """
    Crea un dizionario  nome_pddl → {osm_id, lat, lon}
    utile per debug e per visualizzare i nodi su una mappa.
    """
    result = {}
    for osm_id, pddl_name in node_map.items():
        data = G.nodes[osm_id]
        result[pddl_name] = {
            "osm_id": osm_id,
            "lat": round(data.get("y", 0.0), 7),
            "lon": round(data.get("x", 0.0), 7),
        }
    return result


# ──────────────────────────────────────────────────────────────────────────────
# STEP 5 – Visualizzazione
# ──────────────────────────────────────────────────────────────────────────────
def save_map_image(G, output_path: Path, node_map: dict, start_node, goal_node):
    """
    Salva un'immagine PNG del grafo con start (verde) e goal (rosso) evidenziati.
    """
    try:
        import matplotlib
        matplotlib.use("Agg")
        import matplotlib.pyplot as plt
        import osmnx as ox

        node_colors = []
        for n in G.nodes():
            if n == start_node:
                node_colors.append("green")
            elif n == goal_node:
                node_colors.append("red")
            else:
                node_colors.append("steelblue")

        fig, ax = ox.plot_graph(
            G,
            node_color=node_colors,
            node_size=40,
            edge_color="#888888",
            edge_linewidth=0.8,
            bgcolor="white",
            show=False,
            close=False,
        )
        # Aggiungi etichette su start e goal
        for n, color, label in [
            (start_node, "green", f"START\n{node_map.get(start_node,'')}"),
            (goal_node,  "red",   f"GOAL\n{node_map.get(goal_node,'')}"),
        ]:
            if n in G.nodes():
                x = G.nodes[n]["x"]
                y = G.nodes[n]["y"]
                ax.annotate(label, xy=(x, y), fontsize=7, color=color,
                            fontweight="bold",
                            xytext=(5, 5), textcoords="offset points")

        ax.set_title(f"Grafo stradale – {DEFAULT_CITY}\n"
                     f"{len(G.nodes)} nodi, {len(G.edges)} archi", fontsize=9)
        fig.savefig(str(output_path), dpi=150, bbox_inches="tight")
        plt.close(fig)
        return True
    except Exception as e:
        print(f"      ⚠  Impossibile generare immagine: {e}")
        return False


# ──────────────────────────────────────────────────────────────────────────────
# MAIN
# ──────────────────────────────────────────────────────────────────────────────
def main():
    parser = argparse.ArgumentParser(
        description="Estrae una mappa OSM e genera file PDDL+ per la navigazione."
    )
    parser.add_argument("--city",      default=DEFAULT_CITY,
                        help=f"Nome città OSM (default: '{DEFAULT_CITY}')")
    parser.add_argument("--max-nodes", type=int, default=DEFAULT_MAX_NODES,
                        help=f"Max nodi nel problema PDDL (default: {DEFAULT_MAX_NODES})")
    parser.add_argument("--start",     type=int, default=None,
                        help="OSM node ID del nodo di partenza (default: automatico)")
    parser.add_argument("--goal",      type=int, default=None,
                        help="OSM node ID del nodo di arrivo (default: automatico)")
    args = parser.parse_args()

    OUTPUT_DIR.mkdir(exist_ok=True)

    # 1. Estrai grafo
    G = extract_graph(args.city, args.max_nodes)

    if len(G.nodes) == 0:
        sys.exit("❌  Il grafo estratto è vuoto. Prova ad aumentare --max-nodes.")

    # 2. Mappa nomi nodi  osm_id → loc000, loc001, …
    sorted_nodes = sorted(G.nodes())
    node_map = {n: f"loc{i:03d}" for i, n in enumerate(sorted_nodes)}

    # 3. Scegli start e goal (o usa quelli passati da CLI)
    start_node = args.start if args.start in G.nodes() else sorted_nodes[0]
    goal_node  = args.goal  if args.goal  in G.nodes() else sorted_nodes[-1]

    print(f"\n[4/4] Generazione file PDDL+")
    print(f"      Start : {node_map[start_node]}  (OSM id: {start_node})")
    print(f"      Goal  : {node_map[goal_node]}   (OSM id: {goal_node})")

    # 4. Dominio
    domain_path = OUTPUT_DIR / "domain.pddl"
    domain_path.write_text(generate_domain(), encoding="utf-8")
    print(f"      ✓  {domain_path}")

    # 5. Problema
    problem_path = OUTPUT_DIR / "problem.pddl"
    problem_path.write_text(
        generate_problem(G, start_node, goal_node, node_map),
        encoding="utf-8"
    )
    print(f"      ✓  {problem_path}")

    # 6. Mappa nodi JSON
    map_path = OUTPUT_DIR / "node_map.json"
    map_path.write_text(
        json.dumps(export_node_map(G, node_map), indent=2),
        encoding="utf-8"
    )
    print(f"      ✓  {map_path}")

    # 7. Immagine grafo
    img_path = OUTPUT_DIR / "map.png"
    saved = save_map_image(G, img_path, node_map, start_node, goal_node)
    if saved:
        print(f"      ✓  {img_path}")

    # 8. Riepilogo
    print("\n" + "─" * 60)
    print("✅  Pipeline completata!")
    print(f"   Grafo:    {len(G.nodes)} location  |  {len(G.edges)} road segments")
    print(f"   Start:    {node_map[start_node]}")
    print(f"   Goal:     {node_map[goal_node]}")
    print("─" * 60)
    print("📂  File generati in ./output/")
    print("    domain.pddl   → dominio PDDL+")
    print("    problem.pddl  → problema (start → goal)")
    print("    node_map.json → coordinate GPS di ogni location")
    if saved:
        print("    map.png       → visualizzazione del grafo")
    print("\n💡  Per usare un nodo di partenza/arrivo diverso:")
    print("    Leggi node_map.json, trova gli OSM id e passa:")
    print("    python osm_to_pddl.py --start <osm_id> --goal <osm_id>")


if __name__ == "__main__":
    main()

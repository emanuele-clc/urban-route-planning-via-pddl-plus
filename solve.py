#!/usr/bin/env python3
"""
solve.py
========
Risolve il problema PDDL+ generato da osm_to_pddl.py usando ENHSP
tramite la libreria unified-planning.

Uso:
  python solve.py
  python solve.py --domain output/domain.pddl --problem output/problem.pddl
"""

import argparse
import json
from pathlib import Path

def main():
    parser = argparse.ArgumentParser()
    parser.add_argument("--domain",  default="output/domain.pddl")
    parser.add_argument("--problem", default="output/problem.pddl")
    parser.add_argument("--nodemap", default="output/node_map.json")
    args = parser.parse_args()

    # Carica la mappa nodi per stampare nomi leggibili
    node_names = {}
    if Path(args.nodemap).exists():
        nm = json.loads(Path(args.nodemap).read_text(encoding="utf-8"))
        node_names = {k: f"OSM {v['osm_id']} (lat={v['lat']}, lon={v['lon']})"
                      for k, v in nm.items()}

    print("=" * 60)
    print("  ENHSP – Risolutore PDDL+")
    print("=" * 60)
    print(f"  Domain : {args.domain}")
    print(f"  Problem: {args.problem}")
    print()

    import up_enhsp  # registra il motore ENHSP nel framework unified-planning
    from unified_planning.io import PDDLReader
    from unified_planning.engines import PlanGenerationResultStatus
    from unified_planning.shortcuts import OneshotPlanner, get_environment

    # Silenzia i log di unified-planning
    get_environment().credits_stream = None

    print("[1/3] Lettura file PDDL+...")
    reader = PDDLReader()
    problem = reader.parse_problem(args.domain, args.problem)
    print(f"      Oggetti: {sum(1 for _ in problem.all_objects)} "
          f"| Azioni: {len(problem.actions)}")

    print("[2/3] Lancio ENHSP (planner: GBFS + hadd)...")
    print("      (potrebbe richiedere qualche secondo...)\n")

    with OneshotPlanner(name="enhsp") as planner:
        result = planner.solve(problem)

    print("[3/3] Risultato:\n")

    if result.status in (
        PlanGenerationResultStatus.SOLVED_SATISFICING,
        PlanGenerationResultStatus.SOLVED_OPTIMALLY,
    ):
        plan = result.plan
        actions = list(plan.timed_actions)  # lista di (start, action, duration)

        print(f"✅  Piano trovato! {len(actions)} azioni\n")
        print(f"{'Tempo':>8}  {'Azione':<50}  {'Durata':>8}")
        print("-" * 72)

        total_time = 0.0
        for start, action, duration in actions:
            t     = float(start)
            dur   = float(duration)
            total_time = t + dur
            # Estrai nomi parametri
            params = [str(p) for p in action.actual_parameters]
            if len(params) >= 3:
                vehicle, frm, to = params[0], params[1], params[2]
                frm_info = node_names.get(frm, frm)
                to_info  = node_names.get(to,  to)
                label = f"drive {vehicle}: {frm} → {to}"
            else:
                label = str(action)
            print(f"{t:>8.2f}  {label:<50}  [{dur:.2f}s]")

        print("-" * 72)
        print(f"{'Tempo totale:':>58} {total_time:.2f}s")
        print(f"{'(= {:.1f} minuti)':>{58}}".format(total_time / 60))

        # Salva il piano su file
        plan_path = Path("output") / "plan.txt"
        with open(plan_path, "w", encoding="utf-8") as f:
            f.write(f"Piano PDDL+ – {len(actions)} azioni\n")
            f.write(f"Tempo totale: {total_time:.2f}s ({total_time/60:.1f} min)\n\n")
            for start, action, duration in actions:
                params = [str(p) for p in action.actual_parameters]
                f.write(f"{float(start):8.2f}: {action.action.name}({', '.join(params)})  [{float(duration):.2f}]\n")
        print(f"\n💾  Piano salvato in {plan_path}")

    else:
        print(f"❌  Nessun piano trovato. Stato: {result.status}")
        print()
        print("Possibili cause:")
        print("  - Il grafo non è connesso tra start e goal (prova nodi diversi)")
        print("  - Aumenta --max-nodes in osm_to_pddl.py e rigenera")
        print()
        print("Per scegliere nodi diversi:")
        print("  1. Apri output/node_map.json e scegli due osm_id")
        print("  2. python osm_to_pddl.py --start <osm_id> --goal <osm_id>")
        print("  3. python solve.py")

if __name__ == "__main__":
    main()

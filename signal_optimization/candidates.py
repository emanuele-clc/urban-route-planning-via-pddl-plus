"""
signal_optimization/candidates.py
==================================
Generazione di configurazioni candidate {phase_idx: duration_s} per un
singolo tlLogic, vincolate a NON alterare la topologia delle fasi gia'
validata da netconvert (Criticita' #6, P0 — vedi
2_traffic_signal_optimization.md, sez. 3.2):

  - solo le fasi con almeno un carattere verde (GREEN_CHARS) sono variabili
    di decisione; le fasi di giallo/transizione mantengono la durata
    originale (scalata al ciclo realistico REAL_CYCLE_S).
  - ogni durata verde >= MIN_GREEN_S.
  - somma(durate verdi) + somma(durate gialle fisse) == REAL_CYCLE_S,
    sempre — per costruzione (mai rinormalizzato a posteriori).

Nessun linkIndex cambia gruppo/fase, quindi ogni candidato resta
iniettabile in SUMO senza rischio di conflitti tra movimenti: la
compatibilita' e' quella gia' validata da netconvert.
"""

import os
import sys
import random

BASE = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
SCRIPTS_DIR = os.path.join(BASE, "scripts")
if SCRIPTS_DIR not in sys.path:
    sys.path.insert(0, SCRIPTS_DIR)

from extract_sumo_data import GREEN_CHARS, REAL_CYCLE_S

MIN_GREEN_S = 10.0      # verde minimo per fase (soglia di sicurezza/pedonale)
DURATION_STEP_S = 10.0  # passo di discretizzazione della ricerca (riduce lo
                         # spazio e stabilizza la fitness, vedi sez. 3.4/3.6)


def real_phase_durations(tl, real_cycle=REAL_CYCLE_S):
    """Durate di fase riscalate al ciclo realistico (stessa formula usata
    da extract_sumo_data.extract_traffic_lights per green_real_s)."""
    net_cycle = tl.get("net_cycle_s") or 1.0
    scale = (real_cycle / net_cycle) if net_cycle else 1.0
    return [round(p["duration_s"] * scale, 2) for p in tl["phases"]]


def green_phase_indices(tl):
    """Indici di fase con almeno un movimento verde — le uniche variabili
    di decisione ammesse (vedi criticita' #6)."""
    return [i for i, p in enumerate(tl["phases"])
            if any(ch in GREEN_CHARS for ch in p["state"])]


def baseline_candidate(tl, real_cycle=REAL_CYCLE_S):
    """Candidato identita' = configurazione attualmente in uso (quella gia'
    prodotta da extract_sumo_data.py) — punto di partenza della ricerca."""
    return dict(enumerate(real_phase_durations(tl, real_cycle)))


def slack_budget(tl, real_cycle=REAL_CYCLE_S):
    """(green_idxs, fixed_total, slack): slack e' il budget di verde totale
    (s) da ripartire tra le fasi verdi, dopo aver sottratto le fasi fisse
    (giallo/transizione, mai variate)."""
    durs = real_phase_durations(tl, real_cycle)
    green_idxs = green_phase_indices(tl)
    fixed_total = sum(d for i, d in enumerate(durs) if i not in green_idxs)
    slack = real_cycle - fixed_total
    return green_idxs, fixed_total, slack


def is_feasible(candidate, green_idxs, slack, min_green=MIN_GREEN_S, tol=0.5):
    """Verifica il vincolo di ciclo/verde-minimo (criticita' #6)."""
    if any(candidate[i] < min_green - tol for i in green_idxs):
        return False
    return abs(sum(candidate[i] for i in green_idxs) - slack) < tol


def enumerate_candidates(tl, real_cycle=REAL_CYCLE_S, min_green=MIN_GREEN_S,
                          step=DURATION_STEP_S, max_candidates=25, seed=42):
    """Genera un pool di candidati {phase_idx: duration_s} per un tlLogic,
    sempre includendo il baseline. Per 2 fasi verdi (il caso piu' comune,
    incrocio a 2 fasi principali) enumera esaustivamente la griglia
    discreta; per >2 fasi verdi campiona sul simplesso (seed fisso,
    riproducibile) per tenere lo spazio di ricerca gestibile."""
    durs = real_phase_durations(tl, real_cycle)
    green_idxs, _fixed_total, slack = slack_budget(tl, real_cycle)
    n_green = len(green_idxs)

    base = dict(enumerate(durs))
    candidates = [dict(base)]

    if n_green < 2 or slack <= 0:
        return candidates  # un solo movimento controllato: nulla da ripartire

    if n_green == 2:
        i, j = green_idxs
        g = min_green
        while g <= slack - min_green + 1e-9:
            cand = dict(base)
            cand[i] = round(g, 1)
            cand[j] = round(slack - g, 1)
            candidates.append(cand)
            g += step
    else:
        rng = random.Random(seed)
        attempts = 0
        while len(candidates) < max_candidates and attempts < max_candidates * 20:
            attempts += 1
            cuts = sorted(rng.uniform(0, slack) for _ in range(n_green - 1))
            parts = [cuts[0]] + [cuts[k] - cuts[k - 1] for k in range(1, len(cuts))] + [slack - cuts[-1]]
            if any(p < min_green for p in parts):
                continue
            cand = dict(base)
            for idx, p in zip(green_idxs, parts):
                cand[idx] = round(p, 1)
            candidates.append(cand)

    uniq, seen = [], set()
    for c in candidates:
        key = tuple(round(c[i], 1) for i in sorted(c))
        if key not in seen:
            seen.add(key)
            uniq.append(c)
    return uniq[:max_candidates]


def neighbors(candidate, green_idxs, step=DURATION_STEP_S, min_green=MIN_GREEN_S):
    """Vicini per ricerca locale (hill-climbing/coordinate search, sez. 3.4):
    sposta 'step' secondi da una fase verde i a una fase verde j (i != j).
    La somma totale resta invariata per costruzione (mai rinormalizzata a
    posteriori) -> il vincolo di ciclo (criticita' #6) resta sempre
    rispettato. Generalizza naturalmente a >2 fasi verdi via scambi a coppie."""
    result = []
    for i in green_idxs:
        for j in green_idxs:
            if i == j:
                continue
            if candidate[i] - step >= min_green - 1e-9:
                cand = dict(candidate)
                cand[i] = round(cand[i] - step, 1)
                cand[j] = round(cand[j] + step, 1)
                result.append(cand)
    return result


def controllable_tls(tls_data):
    """id dei tlLogic con >= 2 fasi verdi — gli unici su cui ottimizzare ha
    senso (un solo movimento controllato = nessun trade-off da arbitrare,
    vedi proposta concettuale sez. 1)."""
    return [tid for tid, tl in tls_data.items() if len(green_phase_indices(tl)) >= 2]


def baseline_plan(tls_data, real_cycle=REAL_CYCLE_S):
    """{tl_id: {phase_idx: duration_s}} per tutti i tlLogic della zona —
    configurazione attuale, usata come punto di partenza/baseline di
    confronto per la ricerca."""
    return {tid: baseline_candidate(tl, real_cycle) for tid, tl in tls_data.items()}

# Confronto in SUMO: semafori baseline vs ottimizzati (punto 4)

Misura **simulativa** del guadagno: SUMO riproduce code, accelerazioni e
fasi reali dei semafori. A differenza del punto 2 (stima analitica di
Webster dentro PDDL+), qui il risultato non dipende dal modello usato per
ottimizzare, ed e' quindi una verifica indipendente.

## Disegno dell'esperimento

Due simulazioni identiche per zona, diverse **solo** nel programma
semaforico (baseline = programma `0` del net.xml; ottimizzato =
programma `optimized` da `cfg_files/tls_<zona>.add.xml`).
Stessa domanda O-D (`sumo_extracted/demand_<zona>.json`, lo stesso
campione del punto 2), **stesse rotte** precalcolate con Dijkstra e
riusate identiche nei due run, stesso seed e stessi istanti di partenza:
l'unica variabile e' la temporizzazione dei semafori.

## Risultati

| Zona | Veicoli | Metrica | Baseline | Ottimizzato | Δ% |
|---|---:|---|---:|---:|---:|
| **piccola** | 46 | tempo di viaggio medio (s) | 32.7 | 30.5 | **-6.7%** |
| | | attesa media ai semafori (s) | 3.2 | 1.0 | **-68.5%** |
| | | tempo perso medio (s) | 6.0 | 3.8 | **-36.5%** |

Valori negativi = miglioramento (tempi piu' bassi con i semafori
ottimizzati).

## Interpretazione

L'ottimizzazione riduce l'attesa ai semafori in: **piccola** (-68.5%).
Il guadagno misurato in simulazione conferma, su queste zone, la
direzione prevista dalla stima analitica del punto 2.


## Metriche

- **tempo di viaggio** (`duration`): tempo totale porta a porta.
- **attesa** (`waitingTime`): tempo a velocita' ~0, cioe' fermi al rosso
  o in coda. E' la metrica piu' direttamente legata ai semafori.
- **tempo perso** (`timeLoss`): ritardo rispetto alla marcia ideale a
  velocita' consentita; include anche le decelerazioni.

## Riproducibilita'

```bash
python scripts/compare_sumo.py                  # tutte le zone
python scripts/compare_sumo.py piccola media grande
```

Output: `sumo_comparison/results.json` (dati grezzi) e questo report.
I teletrasporti sono disattivati (`--time-to-teleport -1`): un veicolo
bloccato resta in coda invece di essere rimosso, altrimenti le attese
risulterebbero artificialmente piu' basse.

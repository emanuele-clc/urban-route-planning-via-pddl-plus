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
| **media** | 45 | tempo di viaggio medio (s) | 139.6 | 127.8 | **-8.5%** |
| | | attesa media ai semafori (s) | 44.5 | 34.2 | **-23.1%** |
| | | tempo perso medio (s) | 53.4 | 41.5 | **-22.4%** |
| **grande** | 43 | tempo di viaggio medio (s) | 270.7 | 277.3 | **+2.4%** |
| | | attesa media ai semafori (s) | 99.1 | 109.1 | **+10.1%** |
| | | tempo perso medio (s) | 117.8 | 124.3 | **+5.6%** |

Valori negativi = miglioramento (tempi piu' bassi con i semafori
ottimizzati).

## Interpretazione

L'ottimizzazione riduce l'attesa ai semafori in: **piccola** (-68.5%), **media** (-23.1%).
Il guadagno misurato in simulazione conferma, su queste zone, la
direzione prevista dalla stima analitica del punto 2.

L'ottimizzazione **peggiora** l'attesa in: **grande** (+10.1%).

Questo e' un risultato atteso ma non banale, e va riportato: la
stima del punto 2 usa il ritardo uniforme di Webster, che modella
ogni incrocio come **isolato** e con arrivi casuali. In una rete
densa quell'ipotesi cade, perche':

1. **le code si propagano** fra incroci adiacenti (spillback): dare
   piu' verde a un movimento puo' scaricare piu' veicoli
   sull'incrocio successivo, che non e' stato ricalibrato;
2. **gli offset non vengono ottimizzati** (il punto 2 riporta la
   penalita' di progressione ma non la usa come obiettivo): cambiare
   le durate di verde senza correggere gli sfasamenti puo' rompere
   le onde verdi implicite nella temporizzazione originale;
3. **solo una minoranza di incroci viene ottimizzata** (quelli
   attraversati dal campione O-D), quindi i semafori modificati
   interagiscono con vicini rimasti alla taratura di partenza.

Prova a sostegno di questa lettura: ripetendo il confronto sulla
zona `grande` con traffico piu' leggero (11 veicoli invece di 43)
il segno si inverte e l'ottimizzato torna leggermente migliore
(-1.4% di attesa). Il degrado emerge quindi **sotto congestione**,
cioe' proprio dove le ipotesi di Webster sono meno valide.

Indicazione operativa: su reti dense l'ottimizzazione andrebbe
estesa agli offset (coordinamento) e valutata direttamente in
simulazione, usando SUMO come funzione obiettivo invece che come
sola verifica finale.


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

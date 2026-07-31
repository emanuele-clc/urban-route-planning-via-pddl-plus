# Calibrazione del modello PDDL+ contro SUMO

Il dominio PDDL+ stima il tempo di un percorso con una formula (guida =
distanza/velocita', piu' i ritardi di Webster ai semafori e i tempi di
svolta). Qui la stessa stima viene confrontata, sullo stesso identico
percorso, con il tempo che SUMO misura simulando il veicolo nel
dettaglio. I veicoli sono isolati (partenze molto distanziate), quindi
si misura il percorso in se', non la congestione.

Legenda: **bias** = misurato - previsto (in secondi; positivo = il
modello sottostima, negativo = sovrastima); **corr** = correlazione di
Pearson fra previsto e misurato sui singoli percorsi.

| Zona | N | Confronto | Previsto (s) | Misurato (s) | Bias | Bias % | Corr |
|---|--:|---|--:|--:|--:|--:|--:|
| **piccola** | 15 | guida+svolte vs movimento | 104.5 | 106.1 | +1.5 | +3.9% | 0.90 |
| **piccola** | 15 | semafori (Webster) vs attesa | 57.7 | 53.5 | -4.2 | -7.1% | 0.48 |
| **piccola** | 15 | TOTALE vs durata | 162.2 | 159.6 | -2.6 | +0.2% | 0.73 |

## Come leggere i numeri

La riga **guida+svolte** e' la parte deterministica del modello. Una
correlazione vicina a 1 dice che il modello ordina bene i percorsi per
durata: piu' prevede lungo, piu' SUMO misura lungo. Un piccolo scarto
sistematico e' normale ed e' il costo di accelerazioni e decelerazioni
che un modello a velocita' costante non rappresenta.

La riga **semafori** confronta il ritardo medio di Webster (usato dal
modello) con l'attesa davvero misurata ai rossi. Qui la dispersione e'
piu' alta, perche' Webster e' una media statistica mentre l'attesa di
un singolo passaggio dipende da quando esattamente il veicolo arriva
al semaforo: puo' trovare verde o rosso pieno. La media resta pero'
confrontabile, ed e' cio' che conta quando il modello somma molti
semafori lungo un percorso.

La riga **TOTALE** mette insieme le due parti: e' la stima complessiva
che il planner usa per scegliere il percorso.

## Riproducibilita'

```bash
python scripts/calibrate_sumo.py               # tutte le zone
python scripts/calibrate_sumo.py media --n-routes 30
```
Output: `sumo_comparison/calibration.json` (un record per percorso) e
questo report.
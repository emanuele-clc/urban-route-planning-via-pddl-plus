# Confronto vecchia vs nuova versione del dominio PDDL+

> **Scope.** Questo documento riporta i risultati della suite di test
> `compare_versions.py`, che confronta la **vecchia** versione del modello
> (`signal-delay` per NODO, media dei movimenti — stato del repo prima
> della modifica descritta in `2_traffic_signal_optimization.md`) con la
> **nuova** versione (`signal-delay` per MOVIMENTO `(prev,from,to)`, sez.
> 3.1 dello stesso documento) attualmente nel working tree.

---

## 1. Vecchia vs nuova metodologia di calcolo del ritardo semaforico

### Vecchia metodologia

Il ritardo semaforico era modellato come una funzione **per nodo**,
`(signal-delay ?l - location)`: per ogni incrocio semaforizzato,
`extract_sumo_data.py` calcolava il ritardo di Webster (formula del
ritardo uniforme, `d = rosso² / (2·ciclo)`) per **ciascun movimento**
controllato dal semaforo (ogni `linkIndex` del `tlLogic` SUMO), ma poi
**mediava** questi valori in un unico numero rappresentativo del nodo
(`signal_delay_s = media(delay_i)`), scartando l'informazione su quale
movimento specifico avesse quale ritardo. `build_problems.py` iniettava
questo valore medio nel problema PDDL come `(= (signal-delay n) valore)`,
e il dominio lo addebitava nell'evento `arrive`, cioe' all'**arrivo** al
nodo `?to` — indipendentemente da quale fosse la direzione di provenienza
o di prosecuzione del veicolo in quell'incrocio.

Conseguenza: un incrocio con due fasi molto sbilanciate (es. una direzione
con verde quasi continuo, l'altra con verde minoritario) risultava
rappresentato da un **unico valore intermedio**, applicato a *tutti* i
veicoli che lo attraversano — anche a quelli che in realta' percorrono la
direzione col verde piu' lungo (per cui il ritardo reale e' vicino a 0) o
quella col verde piu' corto (per cui il ritardo reale e' molto piu' alto
della media).

### Nuova metodologia

Il ritardo semaforico e' ora modellato come una funzione **per
movimento**, `(signal-delay ?prev ?from ?to - location)` — stessa forma
gia' usata per `turn-time`. `extract_sumo_data.py` non media piu' i
ritardi: conserva il ritardo di Webster calcolato per **ciascun**
`linkIndex`/fase, insieme al bearing di ingresso e di uscita del
movimento (`bearing_in_bucket`/`bearing_out_bucket`, con classificazione
dritto/sinistra/destra/inversione riusata da `extract_turns`).
`build_problems.py` associa ogni tripla PDDL `(prev, from, to)` al
movimento SUMO reale con bearing piu' vicino
(`assign_movement_signal_delay`), e il dominio addebita il ritardo
nell'azione `start-move` — cioe' alla **partenza** dall'incrocio `?from`
verso `?to`, provenendo da `?prev`: il momento in cui il veicolo impegna
davvero quello specifico movimento e attende il verde corrispondente.

Conseguenza: due veicoli che attraversano lo stesso incrocio semaforizzato
in direzioni diverse ricevono ora ritardi diversi, ciascuno vicino al
valore reale della fase che effettivamente li riguarda.

### Confronto diretto

| | Vecchia | Nuova |
|---|---|---|
| Granularita' | per nodo (1 valore per incrocio) | per movimento (1 valore per ogni `(prev,from,to)`) |
| Fonte del valore | media dei ritardi di Webster di tutti i `linkIndex` del `tlLogic` | ritardo di Webster del `linkIndex`/fase con bearing piu' vicino al movimento reale |
| Momento di addebito nel dominio | evento `arrive`, all'arrivo al nodo `?to` | azione `start-move`, alla partenza da `?from` verso `?to` |
| Sensibilita' alla direzione di marcia | nessuna (stesso valore per tutte le direzioni) | sì (bearing di ingresso/uscita) |
| Rischio di errore sistematico | sì: sovrastima le direzioni "facili", sottostima quelle "difficili" | nessuno per costruzione (valore specifico del movimento realmente percorso), residuo solo se il match per bearing e' impreciso |
| Fallback quando i dati SUMO mancano | valore fisso (`FALLBACK_SIGNAL_DELAY`/`DEFAULT_SIGNAL_DELAY`) | stesso fallback, ma solo se non e' disponibile alcun dato di movimento per quel nodo |

Questo report quantifica l'effetto pratico di questo cambio di
metodologia sui piani effettivamente calcolati da ENHSP.

---

## 2. Metodologia della suite di confronto

Per ciascuna delle tre mappe (`piccola`, `media`, `grande`):

1. si ricostruisce il sottografo PDDL (stessa selezione nodi/archi usata
   da `build_problems.py`, invariata tra le due versioni);
2. si campionano coppie (start, goal) casuali ma **raggiungibili** nel
   sottografo diretto (verifica via Dijkstra prima di accettare la
   coppia, seed dedicato `RANDOM_SEED=123` per riproducibilità);
3. per ogni coppia si genera un `problem.pddl` con la logica VECCHIA
   (`build_problems.py` e `domain.pddl` letti da `git show HEAD:...`,
   nessuna modifica allo stato del repo) e uno con la logica NUOVA
   (working tree), e si risolvono entrambi con ENHSP (`-s aibr`);
4. dal piano trovato si ricostruisce la sequenza di nodi percorsi e se ne
   scompone il costo in: distanza totale, numero di archi, tempo di
   percorrenza (arc-time), ritardo di svolta (turn-time), ritardo
   semaforico (signal-delay) e ritardo di congestione (congestion-delay).

Campione: **12 coppie** per `piccola` e `media`, **20 coppie** per
`grande` (campione piu' ampio per compensare i casi non risolti dal
planner su questa mappa, vedi sez. 5).

### Nota metodologica importante: il campo "Metric" di ENHSP

Durante lo sviluppo della suite e' emerso che il valore stampato da ENHSP
come `Metric (Search)` **non corrisponde** al valore finale del fluent
`(total-time)` nello stato goal. Verifica diretta su un caso di 4 archi
(`n1193756 -> n659784`, zona piccola): ENHSP riportava
`Metric (Search): 13.07`, mentre la somma esatta di
arc-time + turn-time + signal-delay + congestion-delay lungo il piano,
ricalcolata dai valori letti direttamente dal `problem.pddl` generato,
risultava **110.92** — confermata anche ricostruendo il costo con uno
script indipendente che analizza il testo del `.pddl`. Per questo motivo
**tutte le statistiche di questo report sono ricostruite post-hoc dalle
formule del dominio** (stessa logica gia' usata da `webapp/app.py` per il
riepilogo del percorso), non dal campo `Metric` di ENHSP, che viene
scartato. Il campo resta salvato nei JSON grezzi come
`old_metric_enhsp_raw`/`new_metric_enhsp_raw` a solo scopo diagnostico.

---

## 3. Risultati per zona

Valori medi sulle coppie **risolte da entrambe le versioni** (i casi non
risolti da una o entrambe sono esclusi dalle medie e discussi a parte,
sez. 5).

### Piccola (12/12 coppie risolte da entrambe)

| Metrica | Vecchia (media) | Nuova (media) | Δ | Δ% |
|---|---:|---:|---:|---:|
| total-time (s) | 106.30 | 103.44 | -2.86 | -2.7% |
| distanza totale (m) | 317.5 | 317.5 | 0.0 | 0.0% |
| n. archi percorsi | 3.50 | 3.50 | 0.0 | 0.0% |
| tempo di percorrenza — arc-time (s) | vedi nota¹ | vedi nota¹ | 0.0 | 0.0% |
| ritardo di svolta — turn-time (s) | vedi nota¹ | vedi nota¹ | 0.0 | 0.0% |
| **ritardo semaforico (s)** | **8.55** | **5.69** | **-2.86** | **-33.5%** |
| ritardo di congestione (s) | 55.25 | 55.25 | 0.0 | 0.0% |

### Media (12/12 coppie risolte da entrambe)

| Metrica | Vecchia (media) | Nuova (media) | Δ | Δ% |
|---|---:|---:|---:|---:|
| total-time (s) | 189.43 | 194.43 | +5.00 | +2.6% |
| distanza totale (m) | 879.0 | 879.0 | 0.0 | 0.0% |
| n. archi percorsi | 9.25 | 9.25 | 0.0 | 0.0% |
| **ritardo semaforico (s)** | **17.49** | **22.49** | **+5.00** | **+28.6%** |
| ritardo di congestione (s) | 86.0 | 86.0 | 0.0 | 0.0% |

### Grande (15/20 coppie risolte da entrambe — vedi sez. 5)

| Metrica | Vecchia (media) | Nuova (media) | Δ | Δ% |
|---|---:|---:|---:|---:|
| total-time (s) | 201.16 | 197.56 | -3.60 | -1.8% |
| distanza totale (m) | 952.5 | 952.5 | 0.0 | 0.0% |
| n. archi percorsi | 15.0 | 15.0 | 0.0 | 0.0% |
| **ritardo semaforico (s)** | **35.89** | **32.29** | **-3.60** | **-10.0%** |
| n. semafori attraversati (media) | 2.20 | 1.93 | -0.27 | -12.3% |
| ritardo di congestione (s) | 38.53 | 38.53 | 0.0 | 0.0% |

### Aggregato sulle 3 zone (39 coppie risolte da entrambe, pooled)

| Metrica | Vecchia (media) | Nuova (media) | Δ | Δ% |
|---|---:|---:|---:|---:|
| **total-time (s)** | **168.36** | **167.64** | **-0.72** | **-0.4%** |
| distanza totale (m) | 734.51 | 734.51 | 0.0 | 0.0% |
| n. archi percorsi | 9.69 | 9.69 | 0.0 | 0.0% |
| tempo di percorrenza — arc-time (s) | 76.69 | 76.69 | 0.0 | 0.0% |
| ritardo di svolta — turn-time (s) | 11.57 | 11.57 | 0.0 | 0.0% |
| **ritardo semaforico (s)** | **21.82** | **21.09** | **-0.73** | **-3.3%** |
| n. semafori attraversati (media) | 1.38 | 1.31 | -0.07 | -5.1% |
| ritardo di congestione (s) | 58.28 | 58.28 | 0.0 | 0.0% |

¹ *arc-time, turn-time e congestion-delay non dipendono dal modello
semaforico e restano per costruzione identici tra le due versioni quando
il percorso scelto e' lo stesso (vedi sez. 4) — omessi nelle tabelle di
zona per brevita', riportati solo nell'aggregato.*

---

## 4. Il percorso scelto non cambia (in questo campione) — cambia solo la sua stima di costo

In **tutte** le 39 coppie risolte da entrambe le versioni, `distanza
totale` e `numero di archi` sono risultati **identici** tra vecchia e
nuova versione: ENHSP ha sempre scelto la stessa sequenza di strade. Cio'
che cambia e' esclusivamente la stima del ritardo semaforico lungo quel
percorso — la nuova versione lo calcola per il movimento specifico
(bearing di ingresso/uscita all'incrocio) invece che come media di tutti
i movimenti controllati dallo stesso semaforo.

Questo e' un risultato atteso in un campione di queste dimensioni: la
componente di distanza/velocita' base domina tipicamente il costo totale,
quindi un errore sulla sola componente semaforica raramente e' abbastanza
grande da rendere conveniente un percorso alternativo. Non esclude pero'
che, in altri campioni o su incroci con squilibrio di fase piu' marcato
(vedi punto 2, criticita' #1 del design doc), il ricalcolo per movimento
possa effettivamente cambiare il percorso ottimo — e' proprio questo il
meccanismo che l'ottimizzazione semaforica del punto 2 sfrutta
deliberatamente (`signal_optimization/`) per rendere alcuni percorsi/fasi
piu' o meno convenienti.

### Il ritardo semaforico cambia in entrambe le direzioni

Su 39 coppie, in **23 (59%)** il ritardo semaforico stimato e' cambiato in
modo significativo (>0.05s) passando dalla vecchia alla nuova versione:
**9 volte in diminuzione**, **14 volte in aumento**. Alcuni esempi
rappresentativi:

| Zona | Percorso | Vecchia (media/nodo) | Nuova (per movimento) | Nota |
|---|---|---:|---:|---|
| piccola | n1193756 → n659784 | 17.10 s | **0.00 s** | la media attribuiva ritardo a un nodo il cui movimento realmente attraversato ha in realta' verde quasi continuo |
| piccola | n1503255 → n1442755 | 34.20 s | **17.07 s** | media doppia rispetto al movimento realmente attraversato |
| media | n389679 → n1626144 | 38.80 s | **60.35 s** | qui la media SOTTOSTIMAVA: il movimento reale e' quello con fase minoritaria (poco verde) |
| media | n9400040 → n9100868 | 0.00 s | **20.81 s** | media a 0 mascherava un movimento minoritario con ritardo reale non trascurabile |
| grande | n0979472 → n8935624 | 15.50 s | **0.00 s** | media sovrastimava un movimento in realta' quasi sempre verde |
| grande | n2129845 → n1346905 | 49.90 s | **34.14 s** | |

Questo conferma empiricamente la diagnosi della Criticita' #1 del design
doc: la media per nodo non ha un errore sistematico in una direzione sola
— a seconda di quale specifico movimento viene realmente attraversato
puo' sia sovrastimare sia sottostimare (anche pesantemente, fino al 100%
in piu' o in meno) il ritardo reale. Il valore per movimento e' per
costruzione quello corretto per il percorso realmente pianificato.

---

## 5. Casi non risolti (solo zona "grande")

Su 20 coppie campionate in `grande`, **4** non sono state risolte da
**nessuna** delle due versioni (`Problem unsolvable` riportato da ENHSP,
con euristica iniziale `h(I)` che tende a infinito) e **1** e' stata
risolta **solo dalla nuova versione**:

| Percorso | Vecchia | Nuova |
|---|---|---|
| n4904418 → n2472084 | non risolto | non risolto |
| n8246966 → n2129829 | non risolto | non risolto |
| n8246966 → n3062012 | non risolto | non risolto |
| n3171574 → n8918445 | non risolto | non risolto |
| n8246966 → n0977317 | non risolto | **risolto** (total-time = 1193.53 s) |

I 4 casi comuni non sembrano legati al modello semaforico (falliscono
identicamente in entrambe le versioni) ma a un limite di scalabilita' del
planner: la mappa `grande` (120 nodi selezionati, ma solo ~28
raggiungibili dal nodo di partenza di default usato altrove nel progetto)
produce un grounding PDDL+ molto piu' grande (`|A|:395 |P|:206 |E|:206`
nel caso analizzato) su cui l'euristica di ENHSP con `-s aibr` a volte
dichiara erroneamente il problema irrisolvibile, pur essendo la coppia
raggiungibile secondo Dijkstra sul grafo delle distanze. Il quinto caso
(risolto solo dalla nuova versione) mostra invece che il ricalcolo del
ritardo semaforico puo' effettivamente cambiare il "paesaggio" euristico
esplorato da ENHSP abbastanza da far convergere la ricerca dove prima
falliva — un effetto collaterale interessante ma isolato (1 caso su 20)
che non altera le conclusioni aggregate.

---

## 6. Interpretazione

- **La correzione della Criticita' #1 e' confermata empiricamente**: il
  passaggio da un ritardo semaforico medio per nodo a uno specifico per
  movimento cambia la stima di `total-time` nel 59% delle coppie
  testate, in entrambe le direzioni, con variazioni individuali fino a
  ±20 secondi sul solo termine semaforico.
- **A livello aggregato l'effetto netto e' piccolo** (-0.4% sul
  total-time medio, pooled su 3 zone) perche' sovrastime e sottostime
  tendono a compensarsi nel campione — ma questo e' un artefatto
  statistico del campionamento casuale, non un segnale che la correzione
  sia irrilevante: per un singolo veicolo/coppia O-D specifica l'errore
  del vecchio modello puo' essere sostanziale (vedi tabella sez. 4).
- **Il percorso scelto (numero di archi, distanza) non e' mai cambiato**
  nel campione testato: la correzione qui misurata agisce sulla
  *precisione della stima di costo*, non (ancora, in questo campione)
  sulla *scelta del percorso*. E' pero' esattamente la leva che
  `signal_optimization/` (punto 2) usa deliberatamente, variando le
  durate di fase per rendere alcuni movimenti piu' o meno costosi.
- **La mappa "grande" mostra limiti di scalabilita' di ENHSP** non legati
  al modello semaforico, utili da tenere presente per il punto 4 (SUMO
  potrebbe risolvere/simulare O-D che ENHSP non riesce a pianificare).

---

## 7. Riproducibilita'

```bash
python compare_versions.py piccola media --n-samples 12 --max-workers 6 --timeout 60
python compare_versions.py grande --n-samples 20 --max-workers 6 --timeout 60
```

Output:
- `comparison_results/results_<zona>.json` — dati grezzi per zona (ogni
  coppia O-D, entrambi i piani, la scomposizione del costo);
- `comparison_results/results.json` — tutte le zone unite.

La versione "vecchia" e' letta da `git show HEAD:...` (nessun checkout,
nessuna modifica allo stato del repository) — il confronto resta
riproducibile anche dopo un commit delle modifiche correnti, purche' il
commit di riferimento resti raggiungibile in `git log`.

## 8. Limiti del confronto

- Campione relativamente piccolo (12-20 coppie per zona): sufficiente a
  rilevare la presenza e la direzione dell'effetto, non a stimarne con
  precisione la distribuzione completa.
- Il campionamento delle coppie O-D usa un seed dedicato
  (`RANDOM_SEED=123`, `compare_versions.py`), distinto sia da quello del
  modello di congestione (`RANDOM_SEED=42`) sia dal campione condiviso
  punto 2/punto 4 (`generate_demand.py`) — e' un campione indipendente,
  creato apposta per questo confronto.
- Le coppie non risolte da ENHSP (sez. 5) sono escluse dalle medie: la
  zona "grande" e' quindi rappresentata da un sotto-campione leggermente
  piu' piccolo (15/20) di quello nominale.

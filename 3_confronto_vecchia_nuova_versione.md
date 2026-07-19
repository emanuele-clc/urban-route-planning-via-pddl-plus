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

---

## 9. Punto 3 — iniezione del piano semaforico in SUMO

Il confronto delle sezioni precedenti si chiude sul lato PDDL+. Il passo
successivo della roadmap (punto 3) porta il risultato dell'ottimizzazione
del punto 2 dentro il simulatore, chiudendo il ciclo PDDL+ -> SUMO.
Implementato in **`inject_signal_plan.py`**.

### 9.1 Cosa fa

```
net_files/<zona>.net.xml                 (tlLogic originali: stati, offset, type)
sumo_extracted/signal_plan_<zona>.json   (durate ottimizzate, output punto 2)
                 |
                 v
cfg_files/tls_<zona>.add.xml             (<additional> con i tlLogic ottimizzati)
```

Lo script legge il piano nel formato gia' prodotto dal punto 2
(`{tlLogic_id: {phase_idx: durata_s}}`) e riscrive i `tlLogic` conservando
dal `net.xml` **tutti** gli attributi originali di ogni fase (`state`,
`minDur`, `maxDur`, `name`, ...), l'`offset` e il `type` del semaforo:
viene sovrascritta **solo** la `duration` delle fasi presenti nel piano.
Le fasi non ottimizzate mantengono quindi la durata originale, coerentemente
con quanto dichiarato nel campo `format` del file di piano.

### 9.2 Come il programma diventa attivo

Dalla documentazione SUMO (*Simulation/Traffic Lights*, sez. "Defining New
TLS-Programs"): *"You can load new definitions for traffic lights as a part
of an additional-file. **When loaded, the last program will be used**"*.
Non servono quindi ne' WAUT ne' TraCI. I due vincoli imposti dalla stessa
documentazione sono rispettati dallo script:

- l'`id` del `tlLogic` deve essere un semaforo gia' esistente nel `.net.xml`
  (gli id del piano non presenti nella rete vengono ignorati con warning);
- il `programID` deve essere **nuovo** per quel semaforo: si usa
  `optimized`, distinto dall'originale `0` (`off` e' riservato).

Poiche' il programma originale `0` resta comunque caricato, in `sumo-gui` e'
possibile passare da un programma all'altro con **tasto destro sul semaforo
-> Switch TLS program**: baseline e ottimizzato sono confrontabili a occhio
nella stessa simulazione, senza rigenerare nulla.

### 9.3 Integrazione nella pipeline esistente

`sumo_visualize.py` ricava la zona dal `net.xml` realmente usato (quindi
funziona anche in modalita' dinamica, dove la rete puo' differire da quella
richiesta) e aggiunge automaticamente al `.sumocfg`:

```xml
<additional-files value=".../cfg_files/tls_<zona>.add.xml"/>
```

**solo se il file esiste**: in assenza del piano la simulazione parte
esattamente come prima, con i semafori del `net.xml` (nessuna regressione).
E' stato aggiunto il flag `--baseline` per forzare i semafori originali anche
quando l'additional-file e' presente.

La **webapp** non ha richiesto modifiche all'endpoint `/api/sumo`: avvia SUMO
attraverso lo stesso `sumo_visualize.py`, quindi eredita l'iniezione. Verifica
effettuata generando un problema custom dalla webapp e controllando il
`.sumocfg` prodotto, che carica correttamente il `tls_<zona>.add.xml`
corrispondente alla rete realmente usata.

### 9.3.1 Correzione del mapping nodo PDDL -> junction SUMO

Il test del flusso webapp su zone diverse da `piccola` ha fatto emergere un
bug **preesistente** (indipendente dall'iniezione semaforica) che impediva
del tutto l'avvio di SUMO: la visualizzazione falliva con

```
[ERRORE] Junction per 'n9100868' o 'n2842641' non trovata in nessuna net.
```

Causa: `compute_edges_from_pddl` accettava una rete solo se **sia** lo start
**sia** il goal esistevano come junction SUMO con quell'id. Ma `netconvert`
semplifica la rete in modo diverso da come `build_problems.py` costruisce il
grafo contratto, quindi un nodo PDDL puo' legittimamente non esistere come
junction pur essendo la rete quella corretta (verificato: il nodo OSM
`9100868` non e' presente in `media.net.xml`, ne' isolato ne' dentro un
cluster). Lo stesso errore si riproduceva con il `problem_custom.pddl` gia'
committato nel repository, a conferma che il difetto precede questo lavoro.

Correzione applicata in `sumo_visualize.py`, su due livelli:

1. **`pddl_name_to_junction`** prova ora anche il match sui **membri dei
   cluster**: netconvert fonde piu' nodi OSM vicini in una junction
   `cluster_<id1>_<id2>_...`, e senza questo passo un nodo fuso risultava
   "non trovato" pur essendo presente nella rete.
2. **Selezione della rete e ripiego su start/goal**: invece di pretendere il
   match esatto di start e goal, si sceglie la rete che mappa **piu' nodi**
   del problema (start, goal e nodi del piano); se start o goal non sono
   mappabili, si usano il primo e l'ultimo nodo **mappabile del piano ENHSP**.
   Il messaggio di errore residuo, nel caso limite in cui non esista alcun
   piano di ripiego, indica esplicitamente come procedere.

Verifica sul flusso webapp per la zona `media` (problema custom risolto da
ENHSP, 19 nodi di piano):

```
(uso net: media.net.xml)
(start 'n9100868' non e' una junction SUMO: uso 9100869, primo nodo mappabile del piano)
START: n9100868 -> 9100869
GOAL : n2842641 -> 8752842641
(percorso SUMO = piano ENHSP, 19 nodi)
```

Il nodo di ripiego (`9100869`) e' adiacente a quello richiesto, quindi lo
scostamento geografico e' trascurabile. Il `.sumocfg` prodotto carica
`tls_media.add.xml`, cioe' i semafori ottimizzati della rete effettivamente
selezionata: la scelta della rete e quella del programma semaforico restano
coerenti anche quando la zona passata da riga di comando non corrisponde
(la webapp passa sempre `piccola`).

Regressioni verificate dopo la modifica: `sumo_visualize.py piccola`,
`media`, `grande` in modalita' standard caricano ciascuno il proprio
`tls_<zona>.add.xml`, e `--baseline` continua a non caricare alcun
additional-file.

### 9.4 Verifica sui file generati (tutte e tre le zone)

Controllo automatico di ogni `add.xml` contro il `net.xml` di origine:

| Controllo | piccola | media | grande |
|---|---|---|---|
| XML ben formato, root `<additional>` | ok | ok | ok |
| `tlLogic` scritti / semafori della rete | 27 / 27 | 97 / 97 | 453 / 453 |
| fasi con durata modificata | 96 | 399 | 1745 |
| `programID` diverso dall'originale | ok | ok | ok |
| numero di fasi invariato per semaforo | ok | ok | ok |
| `state` di ogni fase identico all'originale | ok | ok | ok |
| durate nulle o negative | nessuna | nessuna | nessuna |
| ciclo semaforico | 90 -> **120 s** | 90 -> **120 s** | 90 -> **120 s** |

Nessun errore rilevato su nessuna delle tre zone. Il ciclo risultante coincide
con i 120 s realistici del sistema SCATS di Dublino gia' assunti nel modello
PDDL+ (`REAL_CYCLE_S` in `extract_sumo_data.py`): l'ottimizzatore
redistribuisce il verde **entro** il ciclo realistico, senza allungarlo
arbitrariamente — proprieta' che vale su tutti i 577 semafori delle tre reti.

### 9.5 Guadagno stimato dall'ottimizzazione (punto 2)

Valori riportati dal report di `optimize.py`, su campione O-D condiviso di 60
coppie per zona (`total-time` medio PDDL+, baseline vs piano ottimizzato):

| Zona | Semafori candidati | Migliorati | Baseline | Ottimizzato | Δ | Δ% |
|---|---:|---:|---:|---:|---:|---:|
| piccola | 1 | 1 | 16.45 s | 10.33 s | -6.11 s | **-37.2%** |
| media | 4 | 3 | 72.04 s | 47.07 s | -24.98 s | **-34.7%** |
| grande | 16 | 16 | 97.80 s | 67.94 s | -29.86 s | **-30.5%** |

Da notare che i semafori *candidati* sono molti meno di quelli presenti in
rete (1, 4 e 16 contro 27, 97 e 453): la ricerca agisce solo sulle giunzioni
effettivamente attraversate dal campione O-D, mentre le altre restano alla
temporizzazione baseline. Il piano iniettato in SUMO contiene comunque
**tutti** i semafori della rete, cosi' il programma `optimized` e' completo e
autoconsistente.

Tempi di calcolo dell'ottimizzazione: 27.9 s (piccola), 234.4 s (media),
1145.0 s (grande) — la crescita e' dovuta al numero di giunzioni candidate
valutate con ENHSP.

### 9.6 Stato per zona

| Zona | `signal_plan_<zona>.json` | `tls_<zona>.add.xml` |
|---|---|---|
| piccola | presente | **generato e verificato** |
| media | presente | **generato e verificato** |
| grande | presente | **generato e verificato** |

Pipeline completa riproducibile con:

```bash
python -m signal_optimization.optimize piccola media grande
python inject_signal_plan.py
python sumo_visualize.py piccola        # oppure media, grande
```

Se un piano manca, `inject_signal_plan.py` lo segnala indicando il comando da
eseguire, senza fallire.

### 9.7 Nota di collegamento con le sezioni 4 e 6 (vedi ora anche sez. 10)

La sez. 4 osserva che, nel campione testato, il ricalcolo del ritardo
semaforico per movimento cambia la **stima di costo** ma non il **percorso
scelto**. L'iniezione in SUMO fornisce il banco di prova indipendente per
questa osservazione: la simulazione microscopica applica le fasi reali
(verde/giallo/rosso con code e accelerazioni) invece del modello analitico di
Webster usato nel PDDL+, e permette quindi di verificare se il guadagno
stimato dal punto 2 (`baseline mean total-time = 16.45 s -> ottimizzato =
10.33 s` per la zona piccola) si conferma anche in simulazione — misura che
rientra nel punto 4, ora svolta e riportata nella sezione seguente.

---

## 10. Punto 4 — confronto in SUMO contro la baseline

Implementato in **`compare_sumo.py`**. Mentre il punto 2 stima il guadagno con
il ritardo uniforme di Webster *dentro* PDDL+, qui il guadagno e' **misurato**
da SUMO in simulazione microscopica: e' quindi una verifica indipendente dal
modello usato per ottimizzare.

### 10.0 A cosa serve questo confronto

Il punto 2 produce una **previsione**: applicando la formula di Webster alle
nuove durate di verde, stima un guadagno. Ma quella formula poggia su ipotesi
forti — incrocio **isolato**, arrivi casuali, nessuna coda che si propaga,
nessuna interferenza fra semafori adiacenti. Sono ipotesi ragionevoli per un
incrocio singolo, non necessariamente valide per una rete urbana reale.

Il punto 4 serve quindi a **falsificare o confermare quella previsione con una
misura indipendente**: si mette davvero una flotta di veicoli in circolazione
nel simulatore e si cronometra il risultato. La differenza metodologica e'
sostanziale, perche' la metrica non proviene piu' dallo stesso modello che ha
prodotto l'ottimizzazione — non c'e' circolarita' fra criterio di
ottimizzazione e criterio di valutazione.

Il controllo non e' stato una formalita': su `grande` ha **smentito** la
previsione (stima -30.5%, misura +10.1%, sez. 10.3). Senza questa verifica il
progetto avrebbe riportato come acquisito un guadagno che in simulazione non
si verifica, e la causa (ipotesi di Webster non valide sotto congestione)
sarebbe rimasta invisibile.

### 10.1 Disegno dell'esperimento

Due simulazioni per zona, identiche in tutto tranne il programma semaforico:

| | Programma semaforico |
|---|---|
| BASELINE | programma `0` del `net.xml` (quello generato da netconvert) |
| OTTIMIZZATO | programma `optimized` da `cfg_files/tls_<zona>.add.xml` (punto 3) |

Accorgimenti per la validita' del confronto:

- **stessa domanda**: coppie O-D di `sumo_extracted/demand_<zona>.json`, lo
  stesso campione condiviso con il punto 2;
- **stesse rotte**: gli itinerari sono calcolati UNA volta con Dijkstra e
  riusati identici nei due run. Lasciando ricalcolare il percorso a SUMO i
  veicoli potrebbero scegliere strade diverse fra i due scenari, e il confronto
  misurerebbe due effetti sovrapposti invece del solo effetto semaforico;
- **stesso seed e stessi istanti di partenza** (partenze scaglionate ogni 3 s,
  per evitare un ingorgo artificiale all'istante 0);
- **teletrasporti disattivati** (`--time-to-teleport -1`): un veicolo bloccato
  resta in coda invece di essere rimosso dalla simulazione, altrimenti proprio
  i casi peggiori sparirebbero dalle statistiche e le attese risulterebbero
  artificialmente piu' basse.

Metriche lette dal `tripinfo-output`: `duration` (tempo di viaggio porta a
porta), `waitingTime` (tempo a velocita' ~0, la metrica piu' direttamente
legata ai semafori) e `timeLoss` (ritardo rispetto alla marcia ideale).

### 10.2 Risultati

| Zona | Veicoli | Metrica | Baseline | Ottimizzato | Δ% |
|---|---:|---|---:|---:|---:|
| **piccola** | 46 | tempo di viaggio medio (s) | 32.7 | 30.6 | **-6.6%** |
| | | attesa media ai semafori (s) | 3.2 | 1.0 | **-68.5%** |
| | | tempo perso medio (s) | 6.0 | 3.8 | **-36.5%** |
| **media** | 45 | tempo di viaggio medio (s) | 139.7 | 127.8 | **-8.5%** |
| | | attesa media ai semafori (s) | 44.5 | 34.2 | **-23.1%** |
| | | tempo perso medio (s) | 53.4 | 41.5 | **-22.3%** |
| **grande** | 43 | tempo di viaggio medio (s) | 270.7 | 277.3 | **+2.4%** |
| | | attesa media ai semafori (s) | 99.1 | 109.1 | **+10.1%** |
| | | tempo perso medio (s) | 117.8 | 124.4 | **+5.6%** |

In tutti gli scenari il numero di veicoli arrivati a destinazione e' identico
fra baseline e ottimizzato (46/46, 45/45, 43/43): le differenze riguardano il
tempo impiegato, non la quota di viaggi completati.

### 10.3 Il caso "grande": l'ottimizzazione peggiora le prestazioni

Su `piccola` e `media` la simulazione conferma la direzione prevista dal punto
2. Su `grande` no: il piano che PDDL+ stimava migliore del 30.5% risulta
**peggiore del 10.1%** sull'attesa ai semafori. Il risultato e' riportato come
tale perche' e' informativo, e la spiegazione sta nelle ipotesi del modello
analitico.

Il ritardo uniforme di Webster descrive un incrocio **isolato** con arrivi
casuali. In una rete densa questa ipotesi cade per tre motivi concomitanti:

1. **propagazione delle code (spillback)**: dare piu' verde a un movimento
   scarica piu' veicoli sull'incrocio successivo, che non e' stato
   ricalibrato e puo' andare in saturazione;
2. **offset non ottimizzati**: il punto 2 calcola la penalita' di progressione
   ma non la usa come obiettivo (sez. 3.5 del design doc). Cambiare le durate
   di verde senza correggere gli sfasamenti puo' rompere le onde verdi
   implicite nella temporizzazione di partenza;
3. **ottimizzazione parziale**: su `grande` sono stati ottimizzati 16 semafori
   su 453 (solo quelli attraversati dal campione O-D), che quindi interagiscono
   con centinaia di vicini rimasti alla taratura originale.

**Prova a sostegno.** Ripetendo il confronto su `grande` con traffico piu'
leggero (11 veicoli invece di 43) il segno si inverte e l'ottimizzato torna
leggermente migliore (**-1.4%** di attesa, -2.2% di tempo perso). Il degrado
compare quindi **sotto congestione**, cioe' esattamente dove le ipotesi di
Webster sono meno valide: e' un effetto di interazione fra incroci, non un
errore nel piano in se'.

**Indicazione operativa.** Su reti dense l'ottimizzazione andrebbe estesa agli
offset (coordinamento fra incroci) e valutata direttamente in simulazione,
usando SUMO come funzione obiettivo invece che come sola verifica finale. Il
risultato su `grande` misura quindi il limite del modello analitico, non
l'inutilita' dell'ottimizzazione: dove le ipotesi reggono (`piccola`, `media`)
il guadagno e' reale e consistente.

### 10.4 Coerenza con la stima del punto 2

| Zona | Stima PDDL+ (Webster) | Misura SUMO (attesa) | Concorde? |
|---|---:|---:|---|
| piccola | -37.2% | -68.5% | si', anzi sottostimata |
| media | -34.7% | -23.1% | si', ordine di grandezza simile |
| grande | -30.5% | +10.1% | **no** |

Le due misure non sono direttamente confrontabili come valori assoluti (la
prima e' il `total-time` di un piano PDDL+, la seconda il tempo di attesa
simulato di una flotta), ma il **segno** e' l'informazione rilevante: concorde
su due zone su tre.

### 10.5 Integrazione nella webapp

- `/api/sumo` accetta ora `variant`: `optimized` (default, carica
  `tls_<zona>.add.xml`) oppure `baseline` (aggiunge `--baseline` a
  `sumo_visualize.py`). L'interfaccia espone due pulsanti "Apri in SUMO",
  che funzionano esattamente come quello precedente.
- `/api/compare_sumo` esegue il confronto sulla zona scelta e restituisce le
  metriche, mostrate in tabella con evidenziazione verde/rossa.

Verifica effettuata avviando Flask: `/api/compare_sumo` sulla zona `piccola`
restituisce gli stessi valori della riga di comando (attesa 3.17 -> 1.0 s,
-68.5%), e `/api/sumo` produce un `.sumocfg` **con** l'`additional-files` per
`optimized` e **senza** per `baseline`.

### 10.6 Riproducibilita' e limiti

```bash
python compare_sumo.py                  # tutte le zone (risultati cumulativi)
python compare_sumo.py grande --max-vehicles 15
```

Limiti da tenere presenti:

- il campione e' di ~45 veicoli per zona (le coppie O-D non instradabili sulla
  rete SUMO vengono scartate: 14, 15 e 17 rispettivamente su 60);
- si simula un solo livello di domanda per zona, senza repliche con seed
  diversi: sufficiente a rilevare segno ed entita' dell'effetto, non a
  stimarne l'intervallo di confidenza;
- le rotte sono fissate a priori: non si modella la ri-scelta del percorso da
  parte dei conducenti in risposta alla nuova temporizzazione.

# Map Construction in PDDL+
**Progetto #2 — Automated Planning**  
UNICAL  
Gruppo: Chiara, Elisa, Emanuele, Pierluigi

---

## Descrizione

Il progetto implementa uno strumento software che:
1. Scarica mappe reali da **OpenStreetMap** (città di Dublino)
2. Le codifica in **PDDL+** come problema di navigazione
3. Trova il percorso ottimale con il planner **ENHSP**
4. Visualizza il percorso animato in **SUMO**

Sono state create tre istanze della mappa a diversa scala:

| Zona | Area | Raggio | Nodi PDDL | Archi | START → GOAL | Distanza | Tempo teorico |
|------|------|--------|-----------|-------|--------------|----------|---------------|
| **Piccola** | Temple Bar / Centro | 400m | 14 | 20 | Liffey St → Aungier St | 1.57 km | 194 s |
| **Media** | Ranelagh / Residenziale | 1200m | 50 | 93 | Leeson St → Saint Mary's Rd | 1.62 km | 150 s |
| **Grande** | Phibsborough / Nord | 3000m | 120 | 206 | Sherrard St → Botanic Ave | 1.33 km | 142 s |

---

## Come funziona — pipeline completa

Il progetto è composto da cinque fasi distinte. PDDL+ e SUMO sono sistemi separati: PDDL+ fa il planning, SUMO fa solo la visualizzazione. Il codice Python fa da ponte tra i due.

### Fase 1 — Scaricare la mappa reale

`download_dublin_map.py` usa la libreria **osmnx** per interrogare OpenStreetMap e scaricare i dati stradali reali di Dublino: coordinate GPS degli incroci, strade con nome, limiti di velocità e sensi unici. Li salva come file `.osm` nella cartella `osm_files/`.

### Fase 2 — Codificare la mappa in PDDL+ (`build_problems.py`)

Questo script è il cuore del progetto. Prende un file `.osm` grezzo (migliaia di nodi GPS e strade) e produce un file PDDL+ pulito e risolvibile. Lo fa in quattro passi interni:

#### Passo A — Lettura del file OSM

Un file `.osm` è un XML. Contiene due tipi di elementi:
- **`<node>`**: un punto sulla mappa con latitudine, longitudine e a volte un nome (es. "Dame Street")
- **`<way>`**: una strada, cioè una sequenza ordinata di nodi, con tag come `highway`, `maxspeed`, `oneway`

Lo script legge tutti i nodi e tutte le strade percorribili in auto (esclude ciclabili, sentieri, ecc.).

#### Passo B — Costruzione del grafo contratto

Un file OSM ha migliaia di nodi, ma la maggior parte sono punti intermedi di una curva — non hanno senso come "fermate" del percorso. Quello che conta sono gli **incroci**, cioè i punti dove il conducente può scegliere una direzione.

Lo script identifica come incroci:
- il primo e l'ultimo nodo di ogni strada (le estremità)
- i nodi che compaiono in 2 o più strade diverse (punti di scelta)

Poi costruisce un **grafo contratto**: collega direttamente un incrocio al successivo, saltando tutti i nodi intermedi, e accumula la distanza percorsa lungo il tratto. Questa distanza viene calcolata con la formula **Haversine**, che calcola la distanza reale in metri tra due coordinate GPS tenendo conto della curvatura della Terra:

```python
def haversine(lat1, lon1, lat2, lon2):
    R = 6371000  # raggio della Terra in metri
    # ... formula trigonometrica ...
    return round(R * 2 * math.atan2(math.sqrt(a), math.sqrt(1 - a)))
```

Per la velocità, lo script legge il tag `maxspeed` della strada in OSM (es. `"30"` km/h) e lo converte in m/s:

```python
spd = float(tags.get("maxspeed", "30").split()[0])
speed_ms = round(spd * 1000 / 3600, 2)  # → 8.33 m/s
```

Se la strada non ha `maxspeed`, assume 30 km/h come default.

#### Passo C — Selezione del sottografo

Il grafo contratto ha ancora centinaia di nodi — troppi per un problema PDDL risolvibile in tempi ragionevoli. Lo script seleziona un sottoinsieme di N nodi (50 per la zona media, 120 per la grande) con questo criterio:

1. Parte dal nodo con più connessioni in uscita (il "hub" principale)
2. Ad ogni passo aggiunge il nodo nella frontiera più lontano dal centroide geografico del gruppo già selezionato

Questo garantisce che i nodi scelti siano **geograficamente distribuiti** su tutta la zona, non ammucchiati in un quartiere.

Lo **START** viene scelto come il nodo con più archi uscenti nel sottografo finale (massimo hub). Il **GOAL** viene scelto come il nodo raggiungibile più lontano dallo start in linea d'aria.

#### Passo D — Scrittura del file PDDL e nomi dei nodi

Per ogni nodo selezionato, lo script gli assegna un nome PDDL. Il criterio è:

```python
base = slugify(node_data[n]["name"]) or f"n{n[-7:]}"
```

- Se il nodo OSM **ha un nome** (es. "Dame Street") → lo converte in un identificatore PDDL valido: `dame_st`
- Se il nodo OSM **non ha un nome** (è un incrocio anonimo) → usa le ultime 7 cifre del suo ID OSM numerico, con una `n` davanti: `n4005414`

Quindi un nome come `n4005414` nel PDDL non è inventato: è un vero incrocio di Dublino che su OpenStreetMap esiste ma non ha un cartello con il nome.

Il file PDDL risultante contiene, per ogni coppia di nodi collegati:
- `(road A B)` — la strada esiste e si può percorrere in quella direzione (rispettando i sensi unici OSM)
- `(= (distance A B) 173)` — la distanza in metri, calcolata con Haversine dai dati GPS reali
- `(= (speed A B) 8.33)` — la velocità in m/s, convertita dal `maxspeed` OSM
- `(= (progress A B) 0)` — lo stato iniziale: il veicolo non ha ancora percorso nulla su quel tratto

Risultato: un file PDDL+ che descrive fedelmente la mappa stradale reale, con tutti i numeri derivati direttamente dai dati geografici di OpenStreetMap.

### Fase 3 — ENHSP risolve il problema PDDL+

`run.py` lancia il planner **ENHSP** passandogli `domain.pddl` e il problema scelto. ENHSP cerca la sequenza di azioni che porta dalla START alla GOAL minimizzando `(total-dist)`.

Il dominio definisce tre costrutti PDDL+:

```pddl
; AZIONE istantanea: il veicolo inizia a percorrere una strada
(:action start-move
  :parameters (?from ?to - location)
  :precondition (and (at ?from) (road ?from ?to))
  :effect (and (not (at ?from)) (moving ?from ?to) (assign (progress ?from ?to) 0)))

; PROCESSO continuo: la distanza percorsa aumenta nel tempo (#t)
(:process driving
  :parameters (?from ?to - location)
  :precondition (moving ?from ?to)
  :effect (increase (progress ?from ?to) (* #t (speed ?from ?to))))

; EVENTO automatico: il veicolo arriva quando progress >= distanza
(:event arrive
  :parameters (?from ?to - location)
  :precondition (and (moving ?from ?to) (>= (progress ?from ?to) (distance ?from ?to)))
  :effect (and (not (moving ?from ?to)) (at ?to)
               (increase (total-dist) (distance ?from ?to))
               (assign (progress ?from ?to) 0)))
```

`#t` è la variabile temporale continua di PDDL+. Il processo `driving` fa avanzare `progress` in modo continuo nel tempo; l'evento `arrive` scatta automaticamente nel momento esatto in cui `progress` raggiunge `distance`. Questo modella il movimento fisico senza discretizzare il tempo.

ENHSP produce un piano come questo:
```
0:     (start-move liffey_st_upper wellington_quay_e)
10.0:  (start-move wellington_quay_e aston_quay)
...
183.0: (start-move sgeorges_m aungier_st)
194s   GOAL ✅   —   Planning Time: 44ms, Expanded Nodes: 207
```

Il piano viene salvato in `output_piccola.txt`.

> ⚠️ Il planner usa il motore `-s aibr`. Non usare `sat-hadd` con domini PDDL+ che
> contengono processi ed eventi: ritorna h(I)=0.0 e non converge mai.

### Fase 4 — Tradurre il piano ENHSP in una rotta SUMO

SUMO e PDDL+ non si parlano direttamente. Il piano ENHSP usa nomi PDDL come `liffey_st_upper`; SUMO usa ID numerici di archi stradali come `39994843`. Il collegamento viene fatto una volta sola con un BFS sul file `net.xml`:

1. Il piano ENHSP dà la sequenza di nodi PDDL: `liffey_st_upper → wellington_quay_e → aston_quay → ...`
2. Ogni nome PDDL corrisponde a un nodo OSM con un ID numerico (es. `659788`)
3. Il file `piccola.net.xml` contiene la rete stradale SUMO generata dallo stesso OSM: ogni junction ha quell'ID incorporato nel nome
4. Un BFS sul grafo del net.xml trova la sequenza di archi SUMO connessa che attraversa quei junction nell'ordine giusto
5. Il risultato è una lista di ID archi: `39994843 -1126998263#0 1478689539 ...`

Questa lista è **fissa per ogni zona** — calcolata una volta e salvata in `sumo_visualize.py`. Non cambia a ogni esecuzione perché il piano ENHSP è deterministico e la traduzione è univoca.

### Fase 5 — SUMO visualizza il percorso

`sumo_visualize.py` genera tre file XML e apre sumo-gui:

**`{zona}_piano.rou.xml`** — dice a SUMO come è fatta l'auto e dove deve andare:
```xml
<vType id="auto" maxSpeed="4.0" color="1,0,0" shape="passenger" accel="1.5" decel="3.0"/>
<route id="piano_enhsp" edges="39994843 -1126998263#0 1478689539 ..."/>
<vehicle id="veicolo_enhsp" type="auto" route="piano_enhsp" depart="1"/>
```
Il tag `edges` contiene la sequenza esatta di archi — l'auto li percorre in ordine, senza mai deviare. Il percorso non è casuale: è esattamente quello trovato da ENHSP.

**`{zona}.sumocfg`** — dice a SUMO quali file caricare (rete stradale, percorso, grafica).

**`gui_{zona}.xml`** — imposta zoom e posizione iniziale della telecamera centrata sul punto di partenza.

SUMO applica poi la **fisica realistica**: accelerazione, frenata, rispetto dei semafori. I semafori che si vedono nella simulazione vengono dai dati OSM reali di Dublino — SUMO li applica automaticamente al veicolo. Il piano PDDL+ non li modella, quindi il veicolo nella simulazione si ferma al rosso mentre il piano teorico assume velocità costante. Questo spiega la differenza tra il tempo del piano (194s) e il tempo reale (≈15 min secondo Google Maps): il piano PDDL+ è un **lower bound ottimistico**.

Il veicolo **scompare dalla mappa quando raggiunge la destinazione** — è il comportamento normale di SUMO: rimuove il veicolo al termine del suo itinerario.

---

## Struttura del progetto

```
progetto_maratea/
├── README.md
├── requirements.txt             # Dipendenze Python (up-enhsp, osmnx)
├── setup.bat                    # Installazione automatica (Windows)
├── projects.pdf                 # Specifiche del progetto
│
└── files/
    ├── osm_files/               # Mappe scaricate da OpenStreetMap
    │   ├── dublin_piccola_centro.osm
    │   ├── dublin_media_residenziale.osm
    │   └── dublin_grande_porto.osm
    │
    ├── net_files/               # Reti stradali per SUMO (da netconvert)
    │   ├── piccola.net.xml
    │   ├── media.net.xml
    │   └── grande.net.xml
    │
    ├── cfg_files/               # File generati da sumo_visualize.py
    │   ├── {zona}.sumocfg
    │   ├── {zona}_piano.rou.xml
    │   └── gui_{zona}.xml
    │
    ├── pddl_files/              # File PDDL+ del progetto
    │   ├── domain.pddl          # Dominio (uguale per tutte le mappe)
    │   ├── problem_piccola.pddl # Problema zona piccola (14 nodi)
    │   ├── problem_media.pddl   # Problema zona media (50 nodi)
    │   ├── problem_grande.pddl  # Problema zona grande (120 nodi)
    │   └── run.py               # Lancia ENHSP e mostra il piano
    │
    ├── encoder/                 # Analisi strade OSM
    │   ├── encoder.py           # Genera report .txt per ogni zona
    │   ├── strade_piccola.txt
    │   ├── strade_media.txt
    │   └── strade_grande.txt
    │
    ├── download_dublin_map.py   # Scarica le mappe OSM via osmnx
    ├── convert_to_osm.py        # Converte OSM → net.xml con netconvert
    ├── build_problems.py        # Genera i file problem_*.pddl da OSM
    └── sumo_visualize.py        # Visualizza il piano in sumo-gui (tutte le zone)
```

---

## Requisiti

| Strumento | Versione | Note |
|-----------|----------|------|
| Python | 3.9+ | https://www.python.org |
| Java | 17+ | Necessario per ENHSP |
| SUMO | 1.x | Per la visualizzazione |
| up-enhsp | 0.1.0 | Installato via pip |
| osmnx | ≥ 1.9 | Installato via pip |

---

## Setup (una volta sola)

**Windows:**
```bat
setup.bat
```

**Mac / Linux:**
```bash
pip install -r requirements.txt
```

---

## Come eseguire

### 1. Risolvere il problema PDDL+ con ENHSP

```bash
cd files/pddl_files
python run.py piccola   # oppure: media, grande
```

### 2. Visualizzare il percorso in SUMO

```bash
cd files
python sumo_visualize.py piccola   # oppure: media, grande
```

Si apre sumo-gui con la rete di Dublino e il veicolo rosso pronto a partire:
- **▶ Play** per avviare la simulazione
- **Ctrl+A** per adattare la vista all'intera rete
- Click destro sull'auto → **Track** per seguirla lungo il percorso
- Il veicolo sparisce quando raggiunge la destinazione (comportamento normale di SUMO)

### 3. (Opzionale) Rigenerare i problemi media e grande

```bash
cd files
python build_problems.py
```

### 4. (Opzionale) Analizzare le strade OSM

```bash
cd files/encoder
python encoder.py
```

---

## Risultati

### Zona Piccola ✅

**Piano trovato da ENHSP** — 44ms, 207 nodi esplorati:

| Tempo (s) | Posizione |
|-----------|-----------|
| 0 | Liffey Street Upper ← START |
| 10 | Wellington Quay Est |
| 12 | Aston Quay |
| 33 | Ormond Quay West |
| 51 | Capel Street Nord |
| 57 | Capel Street / Quay |
| 67 | Grattan Bridge Sud |
| 128 | Cork Hill |
| 132 | Cork Hill Sud |
| 155 | Dame Street Est |
| 160 | South Gt George's St Nord |
| 183 | South Gt George's St Centro |
| **194** | **Aungier Street ← GOAL ✅** |

**Distanza:** 1.57 km — **Tempo teorico:** 194 s a 30 km/h costante  
**Confronto:** ~15 min reali (Google Maps) — il piano PDDL+ è un lower bound (nessun semaforo, nessun traffico)

### Zona Media ✅

**Piano calcolato** — 50 nodi, 93 archi, 18 strade percorse:

| Tempo (s) | Azione |
|-----------|--------|
| 0.0 | Leeson Street Upper ← START |
| 5.7 – 134.5 | 17 tratti intermedi |
| **150** | **Saint Mary's Road ← GOAL ✅** |

**Distanza:** 1.62 km — **Tempo teorico:** 150 s

### Zona Grande ✅

**Piano calcolato** — 120 nodi, 206 archi, 15 strade percorse:

| Tempo (s) | Azione |
|-----------|--------|
| 0.0 | Sherrard Street Lower ← START |
| 5.6 – 138.8 | 14 tratti intermedi |
| **142** | **Botanic Avenue ← GOAL ✅** |

**Distanza:** 1.33 km — **Tempo teorico:** 142 s

### Riepilogo

| Zona | Nodi | Archi | Distanza | Tempo PDDL+ | Note |
|------|------|-------|----------|-------------|------|
| Piccola | 14 | 20 | 1.57 km | 194 s | Risolto da ENHSP in 44ms |
| Media | 50 | 93 | 1.62 km | 150 s | Lower bound ottimistico |
| Grande | 120 | 206 | 1.33 km | 142 s | Lower bound ottimistico |

Il piano PDDL+ modella velocità costante senza semafori né traffico. SUMO applica invece i semafori reali di Dublino da OSM: il veicolo si ferma al rosso, rendendo la simulazione più realistica del piano teorico.

---

## Riferimenti

- PDDL+: Fox & Long (2002), *PDDL+: Modeling continuous time dependent effects*
- ENHSP: Scala et al. (2016), *Interval-Based Relaxation for General Numeric Planning*
- OSMnx: Boeing (2017), *OSMnx: New Methods for Acquiring, Constructing, Analyzing, and Visualizing Complex Street Networks*
- SUMO: Lopez et al. (2018), *Microscopic Traffic Simulation using SUMO*

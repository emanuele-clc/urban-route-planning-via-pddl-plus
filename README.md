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
    ├── build_problems.py        # Genera problem_media.pddl e problem_grande.pddl da OSM
    └── sumo_visualize.py        # Visualizza il piano ENHSP in sumo-gui (tutte le zone)
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

`run.py` trova automaticamente ENHSP, risolve il problema e stampa il piano ottimale.

```bash
cd files/pddl_files
python run.py piccola   # oppure: media, grande
```

Output atteso per piccola:
```
Problem Solved
0: (start-move liffey_st_upper wellington_quay_e)
10.0: (start-move wellington_quay_e aston_quay)
...
183.0: (start-move sgeorges_m aungier_st)
Elapsed Time: 194s  |  Planning Time: 44ms  |  Expanded Nodes: 207
```

Ogni riga del piano è un'**azione istantanea**: il veicolo inizia a percorrere un tratto.
Tra un'azione e la successiva il **processo** `driving` avanza la posizione in modo continuo,
finché l'**evento** `arrive` scatta quando `progress >= distance`.
Il tempo finale è la durata teorica del viaggio a 30 km/h senza traffico né semafori —
un *lower bound* ottimistico rispetto ai tempi reali.

> ⚠️ Il planner usa il motore `-s aibr`. Non usare `sat-hadd` con domini PDDL+ che
> contengono processi ed eventi: ritorna h(I)=0.0 e non converge.

### 2. Visualizzare il percorso in SUMO

```bash
cd files
python sumo_visualize.py piccola   # oppure: media, grande
```

Si apre sumo-gui con la rete di Dublino e il veicolo rosso pronto a partire.
- Premi **▶ Play** per avviare la simulazione
- **Ctrl+A** per adattare la vista all'intera rete
- Click destro sull'auto → **Track** per seguirla lungo il percorso
- Il veicolo **sparisce quando raggiunge la destinazione** — è normale: SUMO rimuove
  il veicolo alla fine del suo itinerario

### 3. (Opzionale) Rigenerare i problemi media e grande

I file `problem_media.pddl` e `problem_grande.pddl` sono già inclusi nel repository.
Per rigenerarli dai file OSM originali (utile se si cambia area o numero di nodi):

```bash
cd files
python build_problems.py
```

L'algoritmo: costruisce il grafo contratto degli incroci OSM → espande un sottografo
connesso di N nodi via BFS diretto → scrive il file PDDL+ con tutte le fluenti.

### 4. Analizzare le strade OSM

```bash
cd files/encoder
python encoder.py
```

Genera tre file `.txt` con tipo di strada, velocità e senso unico per ogni zona.

---

## Modello PDDL+

Il dominio usa i tre costrutti tipici di PDDL+:

```pddl
; AZIONE istantanea: il veicolo inizia a percorrere una strada
(:action start-move
  :parameters (?from ?to - location)
  :precondition (and (at ?from) (road ?from ?to))
  :effect (and (not (at ?from))
               (moving ?from ?to)
               (assign (progress ?from ?to) 0)))

; PROCESSO continuo: la distanza percorsa aumenta nel tempo (#t)
(:process driving
  :parameters (?from ?to - location)
  :precondition (moving ?from ?to)
  :effect (increase (progress ?from ?to) (* #t (speed ?from ?to))))

; EVENTO automatico: il veicolo arriva quando progress >= distanza
(:event arrive
  :parameters (?from ?to - location)
  :precondition (and (moving ?from ?to)
                     (>= (progress ?from ?to) (distance ?from ?to)))
  :effect (and (not (moving ?from ?to))
               (at ?to)
               (increase (total-dist) (distance ?from ?to))
               (assign (progress ?from ?to) 0)))
```

`#t` è la variabile temporale continua di PDDL+.  
Le distanze sono in **metri** (calcolate con Haversine dai dati OSM).  
Le velocità sono in **m/s** (convertite dal limite OSM in km/h: 30 km/h = 8.33 m/s).

---

## Risultati

### Zona Piccola ✅

**Piano trovato da ENHSP** in 44ms, 207 nodi esplorati:

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

**Distanza totale:** ~1.57 km  
**Tempo teorico:** 194 s a 30 km/h  
**Confronto Google Maps:** ~15 min reali — il piano PDDL+ è un lower bound ottimistico (nessun traffico, nessuna frenata)

### Zona Media ✅

**Piano calcolato** (50 nodi, 93 archi, 18 strade percorse):

| Tempo (s) | Posizione |
|-----------|-----------|
| 0.0 | Leeson Street Upper ← START |
| 5.7 | incrocio successivo |
| 6.0 | → |
| 19.9 | → |
| 28.3 | → |
| 39.5 | → |
| 46.2 | → |
| 52.0 | → |
| 65.1 | → |
| 91.7 | → |
| 110.1 | → |
| 123.9 | → |
| 130.3 | → |
| **150** | **Saint Mary's Road ← GOAL ✅** |

**Distanza totale:** 1.62 km  
**Tempo teorico:** 150 s a velocità media 30 km/h

### Zona Grande ✅

**Piano calcolato** (120 nodi, 206 archi, 15 strade percorse):

| Tempo (s) | Posizione |
|-----------|-----------|
| 0.0 | Sherrard Street Lower ← START |
| 5.6 | → |
| 22.6 | → |
| 29.9 | → |
| 46.3 | → |
| 53.1 | → |
| 75.5 | → |
| 84.6 | → |
| 94.6 | → |
| 101.6 | → |
| 113.7 | → |
| 121.5 | → |
| 135.4 | → |
| **142** | **Botanic Avenue ← GOAL ✅** |

**Distanza totale:** 1.33 km  
**Tempo teorico:** 142 s a velocità media 30 km/h

### Riepilogo

| Zona | Nodi | Archi | Distanza | Tempo teorico | Confronto reale |
|------|------|-------|----------|---------------|-----------------|
| Piccola | 14 | 20 | 1.57 km | 194 s | ~15 min (Google Maps) |
| Media | 50 | 93 | 1.62 km | 150 s | lower bound ottimistico |
| Grande | 120 | 206 | 1.33 km | 142 s | lower bound ottimistico |

Il piano PDDL+ è sempre un **lower bound**: non modella semafori, traffico, accelerazioni
né decelerazioni. Il tempo reale è significativamente maggiore.

---

## Riferimenti

- PDDL+: Fox & Long (2002), *PDDL+: Modeling continuous time dependent effects*
- ENHSP: Scala et al. (2016), *Interval-Based Relaxation for General Numeric Planning*
- OSMnx: Boeing (2017), *OSMnx: New Methods for Acquiring, Constructing, Analyzing, and Visualizing Complex Street Networks*
- SUMO: Lopez et al. (2018), *Microscopic Traffic Simulation using SUMO*

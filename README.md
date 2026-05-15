# Map Construction in PDDL+
**Progetto #2 — Automated Planning**  
UNICAL  
Gruppo: Chiara, Elisa, Emanuele, Pierluigi

---

## Descrizione

Il progetto consiste nella costruzione di una mappa reale in **PDDL+** e nella risoluzione di un problema di navigazione tramite il planner **ENHSP**.

La mappa utilizzata è quella di **Dublino, Irlanda**, scaricata da OpenStreetMap. Sono state create tre versioni della mappa a diversa scala:

| Zona | Area reale | Raggio | Nodi PDDL |
|------|-----------|--------|-----------|
| **Piccola** | Temple Bar / Dublino Centro | 400m | 14 |
| **Media** | Ranelagh / zona residenziale | 1200m | ~50 |
| **Grande** | Docklands / Porto | 3000m | ~150 |

---

## Struttura del progetto

```
progetto_maratea/
├── projects.pdf                 # Specifiche del progetto
├── requirements.txt             # Dipendenze Python
├── setup.bat                    # Setup automatico (Windows)
├── README.md
│
└── files/
    ├── osm_files/               # Mappe scaricate da OpenStreetMap
    │   ├── dublin_piccola_centro.osm
    │   ├── dublin_media_residenziale.osm
    │   └── dublin_grande_porto.osm
    │
    ├── net_files/               # Reti convertite per SUMO (netconvert)
    │   ├── piccola.net.xml
    │   ├── media.net.xml
    │   └── grande.net.xml
    │
    ├── pddl_files/              # File PDDL+ del progetto
    │   ├── domain.pddl          # Dominio (uguale per tutte e tre le mappe)
    │   ├── problem_piccola.pddl # Problema zona piccola (14 nodi)
    │   ├── problem_media.pddl   # Problema zona media (~50 nodi)
    │   ├── problem_grande.pddl  # Problema zona grande (~150 nodi)
    │   └── run.py               # Script per lanciare ENHSP
    │
    ├── encoder/                 # Analisi delle strade OSM
    │   ├── encoder.py           # Legge OSM e genera report .txt
    │   ├── strade_piccola.txt
    │   ├── strade_media.txt
    │   └── strade_grande.txt
    │
    ├── download_dublin_map.py   # Scarica le mappe OSM tramite osmnx
    └── convert_to_osm.py        # Converte OSM → net.xml con netconvert
```

---

## Requisiti

| Strumento | Versione | Download |
|-----------|----------|---------|
| Python | 3.9+ | https://www.python.org/downloads/ |
| Java | qualsiasi | https://www.java.com/it/download/ |
| up-enhsp | 0.1.0 | installato con pip (vedi sotto) |
| osmnx | ≥1.9 | installato con pip (vedi sotto) |

> **Nota:** Java è necessario perché ENHSP è un programma Java (`.jar`). Senza Java non si può risolvere il problema PDDL+.

---

## Setup (una volta sola)

**Windows** — doppio click su `setup.bat`, oppure da terminale:
```bat
setup.bat
```

**Mac / Linux** — da terminale:
```bash
pip install -r requirements.txt
```

Questo installa automaticamente `up-enhsp` (il planner) e `osmnx` (per scaricare mappe).

---

## Come eseguire il progetto

### 1. Risolvere il problema PDDL+ con ENHSP

```bash
cd files/pddl_files
python run.py piccola
python run.py media
python run.py grande
```

`run.py` trova ENHSP automaticamente sul PC, lancia il planner e stampa il piano trovato. Il log completo viene salvato in `output_piccola.txt` / `output_media.txt` / `output_grande.txt`.

**Oppure**, se preferisci il comando Java diretto:
```bash
java -jar <percorso>/enhsp.jar -o domain.pddl -f problem_piccola.pddl -s aibr
```

> ⚠️ Usare sempre `-s aibr` (non `sat-hadd`): è l'unico motore che funziona correttamente con domini PDDL+ che usano processi ed eventi.

### 2. Analizzare le strade OSM

```bash
cd files/encoder
python encoder.py
```

Genera tre file `.txt` con nome, tipo, velocità e senso unico per ogni strada delle tre zone.

### 3. Visualizzare la mappa in SUMO

Aprire `sumo-gui` e caricare uno dei file in `files/net_files/`:
- `piccola.net.xml`
- `media.net.xml`
- `grande.net.xml`

---

## Modello PDDL+

Il dominio usa tre costrutti di PDDL+: **azione**, **processo** ed **evento**.

```pddl
; AZIONE istantanea: il veicolo inizia a percorrere una strada
(:action start-move
  :parameters (?from ?to - location)
  :precondition (and (at ?from) (road ?from ?to))
  :effect (and (not (at ?from)) (moving ?from ?to)
               (assign (progress ?from ?to) 0)))

; PROCESSO continuo: la distanza percorsa aumenta nel tempo (#t)
(:process driving
  :parameters (?from ?to - location)
  :precondition (moving ?from ?to)
  :effect (increase (progress ?from ?to) (* #t (speed ?from ?to))))

; EVENTO automatico: quando progress >= distanza, il veicolo arriva
(:event arrive
  :parameters (?from ?to - location)
  :precondition (and (moving ?from ?to)
                     (>= (progress ?from ?to) (distance ?from ?to)))
  :effect (and (not (moving ?from ?to)) (at ?to)
               (increase (total-dist) (distance ?from ?to))))
```

`#t` è la variabile temporale continua di PDDL+. Le distanze sono in **metri** reali (Haversine sui dati OSM), le velocità in **m/s** (convertite dai limiti OSM in km/h).

---

## Risultati (zona piccola)

Piano trovato da ENHSP in **44ms**, 207 nodi esplorati:

```
  0s → liffey_st_upper
 10s → wellington_quay_e
 12s → aston_quay
 33s → ormond_quay_w
 51s → capel_st_n
 57s → capel_st_quay
 67s → grattan_bridge_s
128s → cork_hill
132s → cork_hill_s
155s → dame_st_e
160s → sgeorges_n
183s → sgeorges_m
194s → aungier_st  ✅ GOAL
```

**Tempo totale: 194 secondi** (~3 minuti) su 12 strade reali del centro di Dublino.

---

## Riferimenti

- PDDL+: Fox & Long (2002), *PDDL+: Modeling continuous time dependent effects*
- ENHSP: Scala et al. (2016), *Interval-Based Relaxation for General Numeric Planning*
- OSMnx: Boeing (2017), *OSMnx: New Methods for Acquiring, Constructing, Analyzing, and Visualizing Complex Street Networks*
- SUMO: Lopez et al. (2018), *Microscopic Traffic Simulation using SUMO*

# Map Construction in PDDL+
**Progetto #2 — Automated Planning**  
Corso tenuto dal Prof. Marco Maratea — UNICAL  
Gruppo: Chiara, Elisa, Emanuele, Pierluigi

---

## Descrizione

Il progetto consiste nella costruzione di una mappa reale in **PDDL+** e nella risoluzione di un problema di navigazione tramite il planner **ENHSP**.

La mappa utilizzata è quella di **Dublino, Irlanda**, scaricata da OpenStreetMap. Sono state create tre versioni della mappa a diversa scala:

| Zona | Area reale | Raggio | Nodi PDDL |
|------|-----------|--------|-----------|
| **Piccola** | Temple Bar / Dublino Centro | 400m | ~14 |
| **Media** | Ranelagh / zona residenziale | 1200m | ~50 |
| **Grande** | Docklands / Porto | 3000m | ~150 |

---

## Struttura del progetto

```
progetto_maratea/
├── projects.pdf                 # Specifiche del progetto (Prof. Maratea)
├── README.md
│
└── files/                      # Tutto il lavoro del progetto
    │
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
    ├── cfg_files/               # File di configurazione SUMO
    │
    ├── pddl_files/              # File PDDL+ del progetto
    │   ├── domain.pddl          # Dominio (uguale per tutte e tre le mappe)
    │   ├── problem_piccola.pddl # Problema zona piccola
    │   ├── problem_media.pddl   # Problema zona media
    │   └── problem_grande.pddl  # Problema zona grande
    │
    ├── encoder/                 # Analisi delle strade OSM
    │   ├── encoder.py           # Script: legge OSM e genera report .txt
    │   ├── strade_piccola.txt   # Report strade zona piccola
    │   ├── strade_media.txt     # Report strade zona media
    │   └── strade_grande.txt    # Report strade zona grande
    │
    ├── download_dublin_map.py   # Scarica le mappe OSM tramite osmnx
    └── convert_to_osm.py        # Converte OSM → net.xml con netconvert
```

---

## Modello PDDL+

Il dominio modella la navigazione su strada con tre costrutti tipici di PDDL+:

```pddl
; AZIONE: il veicolo inizia a percorrere una strada
(:action start-move
  :parameters (?from ?to - location)
  :precondition (and (at ?from) (road ?from ?to))
  :effect (and (not (at ?from)) (moving ?from ?to) (assign (progress) 0)))

; PROCESSO: la distanza percorsa aumenta continuamente nel tempo
(:process driving
  :parameters (?from ?to - location)
  :precondition (moving ?from ?to)
  :effect (increase (progress) (* #t (speed ?from ?to))))

; EVENTO: quando progress >= distanza, il veicolo è arrivato
(:event arrive
  :parameters (?from ?to - location)
  :precondition (and (moving ?from ?to) (>= (progress) (distance ?from ?to)))
  :effect (and (not (moving ?from ?to)) (at ?to) (assign (progress) 0)))
```

Il simbolo `#t` è la variabile temporale di PDDL+: il processo aggiorna la posizione **continuamente** in proporzione alla velocità della strada.

Le distanze sono in **metri** reali (calcolate con la formula di Haversine dai dati OSM).  
Le velocità sono in **m/s** (convertite dal limite OSM in km/h).

---

## Strumenti utilizzati

| Strumento | Scopo |
|-----------|-------|
| `osmnx` | Download mappe da OpenStreetMap |
| `netconvert` (SUMO) | Conversione OSM → rete stradale SUMO |
| `sumo-gui` | Visualizzazione della mappa e del percorso |
| `ENHSP` | Risoluzione del problema PDDL+ |

---

## Come eseguire

**1. Scaricare le mappe** (se non già presenti):
```bash
cd files
python download_dublin_map.py
```

**2. Convertire in formato SUMO**:
```bash
python convert_to_osm.py
```

**3. Analizzare le strade**:
```bash
cd encoder
python encoder.py
```

**4. Risolvere il problema PDDL+** con ENHSP:
```bash
java -jar enhsp.jar -o pddl_files/domain.pddl -f pddl_files/problem_piccola.pddl -s sat-hadd
```

---

## Riferimenti

- PDDL+: Fox & Long (2002), *PDDL+: Modeling continuous time dependent effects*
- ENHSP: Scala et al. (2016), *Interval-Based Relaxation for General Numeric Planning*
- OSMnx: Boeing (2017), *OSMnx: New Methods for Acquiring, Constructing, Analyzing, and Visualizing Complex Street Networks*
- SUMO: Lopez et al. (2018), *Microscopic Traffic Simulation using SUMO*

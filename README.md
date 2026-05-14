# Street Maps in PDDL+

**Progetto #2 — Automated Planning** | Università della Calabria (UNICAL)  
Corso: Automated Planning — Prof. Marco Maratea  
Gruppo: Chiara, Elisa, Emanuele, Pierluigi

---

## Descrizione

Pipeline completa che:
1. Estrae la rete stradale di **Cosenza** da OpenStreetMap via OSMnx
2. La codifica in **PDDL+** (Planning Domain Definition Language Plus)
3. Trova un percorso ottimale con il planner **ENHSP**
4. Valida il piano con una simulazione realistica in **SUMO**

Il problema di pianificazione è la navigazione di un veicolo dal punto A al punto B in una rete stradale reale.

---

## Strumenti usati

| Strumento | Versione | Scopo |
|-----------|----------|-------|
| [OSMnx](https://osmnx.readthedocs.io) | ≥ 1.9 | Download e semplificazione rete OSM |
| [NetworkX](https://networkx.org) | ≥ 3.0 | Gestione grafi stradali |
| [ENHSP](https://sites.google.com/view/enhsp/) | bundled | Planner PDDL+ (modalità `sat-hadd`) |
| [SUMO](https://sumo.dlr.de) | ≥ 1.14 | Simulazione traffico per validazione |
| Python | ≥ 3.9 | Pipeline principale |

---

## Struttura del progetto

```
progetto_maratea/
│
├── osm_to_pddl.py          # Pipeline principale: OSM → PDDL+
├── solve.py                # Risolve il problema con ENHSP
├── sumo_validation.py      # Valida il piano con simulazione SUMO
├── requirements.txt        # Dipendenze Python
│
└── output/
    ├── domain.pddl         # Dominio PDDL+ generato
    ├── problem.pddl        # Problema PDDL+ (40 nodi di Cosenza)
    ├── node_map.json       # Mappa nomi PDDL ↔ coordinate GPS ↔ OSM ID
    ├── map.png             # Visualizzazione grafo stradale
    ├── plan_visualization.html  # Mappa interattiva del percorso trovato
    └── sumo/               # File generati dalla validazione SUMO
        ├── cosenza.osm
        ├── cosenza.net.xml
        ├── cosenza.sumocfg
        └── tripinfo.xml
```

---

## Installazione

### Dipendenze Python

```bash
pip install -r requirements.txt
```

oppure su Windows:

```bash
python -m pip install -r requirements.txt
```

### ENHSP

ENHSP è incluso nel pacchetto `up-enhsp`. Per usarlo direttamente da Java:

```bash
# Trova il jar
python -c "import up_enhsp; import os; print(os.path.dirname(up_enhsp.__file__))"

# Esegui (sostituisci PATH con il percorso trovato)
java -jar PATH/ENHSP/enhsp.jar -o output/domain.pddl -f output/problem.pddl -planner sat-hadd
```

### SUMO

Scarica e installa SUMO da [sumo.dlr.de/docs/Downloads.html](https://sumo.dlr.de/docs/Downloads.html).

Su Windows, dopo l'installazione imposta la variabile d'ambiente:

```
SUMO_HOME = C:\Program Files (x86)\Eclipse\Sumo
```

---

## Utilizzo

### 1. Genera la rete PDDL+

```bash
python osm_to_pddl.py
```

Scarica la rete stradale di Cosenza, estrae un sottografo di 40 nodi e genera `output/domain.pddl` e `output/problem.pddl`.

### 2. Risolvi il problema di pianificazione

```bash
python solve.py
```

Invoca ENHSP in modalità `sat-hadd` e stampa il piano trovato.

Oppure direttamente con Java:

```bash
java -jar <path_to_enhsp.jar> -o output/domain.pddl -f output/problem.pddl -planner sat-hadd
```

### 3. Valida con SUMO

```bash
python sumo_validation.py
```

Scarica i dati OSM grezzi, converte la rete con `netconvert`, esegue la simulazione e confronta il tempo ENHSP con quello simulato da SUMO.

---

## Risultati

### Piano trovato da ENHSP

Il planner ha trovato un percorso da **loc000** (nord Cosenza) a **loc039** (sud Cosenza) in **8 azioni `start-drive`**:

```
loc000 → loc021 → loc008 → loc003 → loc018 → loc010 → loc005 → loc038 → loc039
```

| Metrica | Valore |
|---------|--------|
| Tempo totale (ENHSP) | 1430 s ≈ 23.8 min |
| Numero di tappe | 8 |
| Planner | ENHSP sat-hadd |

La mappa interattiva del percorso è in `output/plan_visualization.html`.

### Confronto ENHSP vs SUMO

La simulazione SUMO valida il piano eseguendolo su una rete stradale realistica con accelerazione, decelerazione e semaforistica. I risultati del confronto vengono salvati in `output/sumo_comparison.json`.

---

## Modello PDDL+

Il dominio usa la struttura tipica di PDDL+: **azione + processo + evento**.

```pddl
; Azione istantanea: il veicolo inizia a muoversi
(:action start-drive
  :parameters (?v - vehicle ?from - location ?to - location)
  :precondition (and (at ?v ?from) (road ?from ?to) (free ?v))
  :effect (and (not (at ?v ?from)) (not (free ?v))
               (driving ?v ?from ?to) (assign (position ?v) 0)))

; Processo continuo: la posizione aumenta proporzionalmente al tempo (#t)
(:process moving
  :parameters (?v - vehicle ?from - location ?to - location)
  :precondition (and (driving ?v ?from ?to)
                     (< (position ?v) (road-length ?from ?to)))
  :effect (and
    (increase (position ?v) (* #t (speed-limit ?from ?to)))
    (increase (travel-time ?v) #t)))

; Evento automatico: quando arriva a destinazione
(:event arrive
  :parameters (?v - vehicle ?from - location ?to - location)
  :precondition (and (driving ?v ?from ?to)
                     (>= (position ?v) (road-length ?from ?to)))
  :effect (and (not (driving ?v ?from ?to))
               (at ?v ?to) (free ?v) (assign (position ?v) 0)))
```

Il simbolo `#t` rappresenta la variabile temporale di PDDL+: il processo `moving` aggiorna la posizione **continuamente** nel tempo in proporzione alla velocità limite dell'arco.

---

## Note tecniche

- **OSMnx 2.x** semplifica automaticamente il grafo al momento del download (`simplify=True`); non è necessaria una chiamata esplicita a `simplify_graph()`.
- **ENHSP** nella versione bundled in `up-enhsp 0.1.0` supporta solo PDDL+ puro (`:action`, `:process`, `:event`), **non** le `:durative-action` di PDDL 2.1.
- Il sottografo di 40 nodi viene estratto con una BFS dal nodo più vicino al centro di Cosenza, per mantenere il problema trattabile.
- I limiti di velocità sono estratti da OSM; dove assenti viene usato un default di 50 km/h.

---

## Riferimenti

- PDDL+ originale: Fox & Long (2002), *PDDL+: Modeling continuous time dependent effects*
- ENHSP: Scala et al. (2016), *Interval-Based Relaxation for General Numeric Planning*
- OSMnx: Boeing (2017), *OSMnx: New Methods for Acquiring, Constructing, Analyzing, and Visualizing Complex Street Networks*
- SUMO: Lopez et al. (2018), *Microscopic Traffic Simulation using SUMO*

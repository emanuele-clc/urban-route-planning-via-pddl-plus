# Map Construction in PDDL+

**Progetto #2 — Automated Planning**  
Università della Calabria (UNICAL)  
Gruppo: Emanuele Colecchia, Chiara Costantino, Elisa Gigliotti, Pierluigi Trocini

---

## Descrizione del progetto

Il progetto realizza una pipeline software completa per la costruzione automatica di problemi di navigazione in **PDDL+**, a partire da dati geografici reali. L'area geografica scelta è la città di **Dublino (Irlanda)**, utilizzando dati stradali scaricati da OpenStreetMap.

Il sistema integra tre strumenti distinti:

- **ENHSP** — planner numerico per PDDL+, responsabile della ricerca del percorso ottimale
- **OSMnx** — libreria Python per il download e la manipolazione di grafi stradali OSM
- **SUMO** — simulatore microscopico del traffico, utilizzato per la visualizzazione animata del piano trovato

Il dominio modella un veicolo che deve raggiungere un nodo obiettivo a partire da un nodo di partenza, minimizzando il tempo totale di percorrenza. Il tempo include la guida su ciascun arco stradale, i ritardi semaforici **realistici estratti dalla rete SUMO** e il **tempo di svolta** agli incroci (in base all'angolo di sterzata e alla velocità angolare del veicolo).

---

## Istanze del problema

Sono state costruite tre istanze a scala crescente, ciascuna corrispondente a una zona diversa di Dublino:

| Zona     | Area geografica              | Nodi PDDL | Archi | Start → Goal                         | Distanza | Tempo piano |
|----------|------------------------------|-----------|-------|--------------------------------------|----------|-------------|
| Piccola  | Temple Bar / Centro storico  | 14        | 20    | Liffey St. → Aungier St.            | 1.57 km  | ~314 s      |
| Media    | Ranelagh / Residenziale      | 50        | 93    | Leeson St. → Saint Mary's Rd.       | 1.62 km  | 160 s       |
| Grande   | Phibsborough / Nord          | 120       | 206   | St. Patrick's Rd. → Botanic Ave.    | 1.33 km  | 147 s       |

---

## Architettura del sistema

### Fase 1 — Download della mappa (`download_dublin_map.py`)

Lo script utilizza la libreria **osmnx** per scaricare i dati stradali reali di Dublino da OpenStreetMap: coordinate GPS degli incroci, topologia della rete, limiti di velocità e sensi unici. I dati vengono salvati come file `.osm` nella cartella `osm_files/`.

### Fase 2 — Costruzione del problema PDDL+ (`build_problems.py`)

Questo script costituisce il nucleo del progetto. A partire da un file `.osm` grezzo, produce un problema PDDL+ completo attraverso quattro passi:

**A. Lettura del file OSM.** Il file viene analizzato come XML. Vengono estratti i nodi (incroci con coordinate GPS) e le strade percorribili da veicoli, escludendo piste ciclabili e percorsi pedonali. I nodi con attributo `highway=traffic_signals` vengono identificati come semafori.

**B. Costruzione del grafo contratto.** I nodi OSM intermedi (punti di curvatura di una strada, privi di valore decisionale) vengono eliminati. Vengono conservati unicamente gli incroci reali, cioè i nodi che compaiono in due o più strade distinte o che costituiscono le estremità di una strada. Gli archi del grafo contratto connettono direttamente incroci adiacenti; la distanza di ciascun arco è calcolata tramite la formula di Haversine applicata alle coordinate GPS reali.

**C. Selezione del sottografo.** Per mantenere il problema PDDL+ risolvibile in tempi computazionali accettabili, viene selezionato un sottoinsieme di N nodi (parametrizzabile). La selezione parte dal nodo con il maggior grado uscente e ad ogni passo aggiunge il nodo della frontiera più distante dal centroide geografico del gruppo già selezionato, garantendo una copertura spazialmente distribuita dell'area.

**D. Scrittura del file PDDL+.** Per ogni arco del sottografo vengono scritti i predicati e le funzioni numeriche necessari: `(road A B)`, `(distance A B)`, `(speed A B)`, `(progress A B)`, `(signal-delay A)`, oltre alle funzioni `(turn-time P A B)` per ogni tripla di nodi consecutivi (vedi sezione "Semafori e tempi di svolta"). Il ritardo semaforico **non è più un valore fisso di 30 s**: viene letto dai dati realistici estratti da SUMO (`sumo_extracted/sumo_data_{zona}.json`), con fallback a 30 s solo per i semafori non mappabili. La velocità è ricavata dal tag `maxspeed` OSM (default: 30 km/h), convertita in m/s.

### Fase 3 — Risoluzione con ENHSP (`pddl_files/run.py`)

Il planner **ENHSP** viene invocato con il dominio e il problema selezionato. Viene utilizzata la configurazione `-s aibr` (Admissible Interval-Based Relaxation), adatta a domini PDDL+ con processi ed eventi. La metrica ottimizzata è `(total-time)`, che accumula il tempo di guida e i ritardi semaforici.

### Fase 4 — Traduzione del piano per SUMO (`sumo_visualize.py`)

ENHSP e SUMO utilizzano sistemi di identificazione dei nodi incompatibili: il piano PDDL+ usa identificatori testuali (es. `liffey_st_upper`), mentre SUMO richiede ID numerici di archi stradali (es. `4396046#0`). La corrispondenza viene stabilita tramite un algoritmo di Dijkstra applicato al file `net.xml` della zona corrispondente: a partire dalle junction SUMO di partenza e di arrivo (identificate tramite i medesimi ID OSM usati nella costruzione del problema PDDL+), viene calcolato il percorso a distanza minima, ottenendo la sequenza ordinata di archi da passare a SUMO.

### Fase 5 — Visualizzazione in SUMO

Lo script genera i file di configurazione necessari a SUMO (`{zona}.sumocfg`, `{zona}_piano.rou.xml`, `gui_{zona}.xml`) e apre sumo-gui con il percorso già caricato. SUMO applica la fisica del traffico (accelerazione, frenata, semafori dinamici) in modo indipendente dal piano PDDL+: i semafori vengono simulati con le loro fasi reali (verde/giallo/rosso), rendendo la simulazione più realistica rispetto al modello semplificato del dominio.

---

## Semafori e tempi di svolta (dati estratti da SUMO)

Lo script **`extract_sumo_data.py`** legge le reti SUMO (`net_files/*.net.xml`) e produce, per ogni zona, il file `sumo_extracted/sumo_data_{zona}.json` (più un `report.md` di riepilogo) con i dati usati per arricchire i problemi PDDL+. L'estrazione avviene parsando direttamente l'XML del `net.xml`, senza bisogno di eseguire SUMO.

### Da dove vengono i tempi dei semafori

Ogni incrocio semaforizzato in SUMO è un elemento `<tlLogic>` con le sue fasi:

```xml
<tlLogic id="12639663" type="static" programID="0" offset="0">
    <phase duration="42" state="GGrrrr"/>   <!-- verde direzione 1 -->
    <phase duration="3"  state="yyrrrr"/>   <!-- giallo -->
    <phase duration="42" state="rrGGGG"/>   <!-- verde direzione 2 -->
    <phase duration="3"  state="rryyyy"/>   <!-- giallo -->
</tlLogic>
```

Questi tempi, però, **non sono i tempi reali di Dublino**: sono i valori *di default* generati da `netconvert` (ciclo fisso di 90 s, verde diviso equamente tra le direzioni, giallo calcolato dalla velocità della strada — fonte: documentazione SUMO). A Dublino gli incroci sono controllati dal sistema adattivo **SCATS**, con cicli tipici fino a **~120 s** (nel Regno Unito/Irlanda il ciclo massimo è 120 s, 90 s dove sono presenti attraversamenti pedonali).

Per ottenere un ritardo realistico procediamo così:

1. **Struttura dal net.xml** — da SUMO prendiamo *quali* movimenti sono verdi insieme e la *proporzione* verde/rosso di ogni movimento. Questa informazione è significativa perché deriva dalla geometria reale dell'incrocio (numero di bracci, corsie, precedenze).
2. **Ciclo realistico** — riscaliamo il ciclo dai 90 s di SUMO al valore realistico di Dublino (`REAL_CYCLE_S = 120 s`), mantenendo il rapporto verde/rosso.
3. **Ritardo medio (formula di Webster)** — per un semaforo a tempo fisso con arrivi casuali, il ritardo medio di un veicolo è

   > **d = rosso² / (2 · ciclo)**

   dove *rosso* è il tempo di rosso del movimento sul ciclo realistico. Il `signal-delay` del nodo è la media di *d* sui suoi movimenti.

Esempio (incrocio a 2 fasi): verde ≈ 47 % del ciclo → rosso reale ≈ 64 s su 120 s → **d ≈ 64² / (2·120) ≈ 17 s**. Da qui il valore ~17 s che sostituisce i 30 s fissi usati in precedenza.

Gli ID delle *junction* SUMO coincidono con gli ID-nodo OSM; il JSON contiene quindi una mappa `node_signal_delay` (ID-nodo → ritardo, con i *cluster* espansi) direttamente riutilizzabile da `build_problems.py` e dalla webapp.

### Da dove viene il tempo di svolta (turn time)

Per **turn rate** si intende la velocità angolare con cui un veicolo cambia direzione, misurata in **gradi al secondo**. Usiamo `TURN_RATE_DPS = 20 °/s`, coerente con lo *yaw rate* reale di un'automobile in una svolta urbana stretta (~15–20 °/s; oltre ~30 °/s interviene il controllo di stabilità ESC).

Il **tempo di svolta** a un incrocio dipende dall'angolo di cui il veicolo deve ruotare:

> **turn-time = |angolo di svolta| / turn-rate**

L'*angolo di svolta* è la differenza fra la direzione (rotta) dell'arco in ingresso e quella dell'arco in uscita all'incrocio:

- in `extract_sumo_data.py` è calcolato dalla **geometria delle corsie** del `net.xml` (heading dell'ultimo segmento della corsia entrante vs. primo segmento di quella uscente); la direzione così calcolata coincide con l'attributo `dir` di SUMO nel 94–97 % dei casi (validazione dell'algoritmo);
- in `build_problems.py` e nella webapp è calcolato dalle **coordinate GPS reali dei nodi** (rotta *prev→from* vs. *from→to*), coerentemente con il grafo contratto usato nel PDDL.

Esempi: svolta a 90° → 90/20 = **4,5 s**; inversione a U (180°) → **9 s**; proseguire dritto (~0°) → **~0 s**.

---

## Dominio PDDL+

Il dominio definisce un'azione discreta, un processo continuo e un evento automatico. Rispetto alla versione base, `start-move` conosce il **nodo di provenienza** (`?prev`) per poter addebitare il tempo di svolta, e un unico fatto `(prev ...)` tiene traccia dell'ultimo arco percorso (cancellato in `start-move`, ristabilito da `arrive`):

```pddl
(:action start-move
  :parameters (?prev ?from ?to - location)
  :precondition (and (at ?from) (road ?from ?to) (prev ?prev))
  :effect (and
    (not (at ?from)) (not (prev ?prev)) (moving ?from ?to)
    (assign (progress ?from ?to) 0)
    (increase (total-time) (turn-time ?prev ?from ?to))))   ; costo di svolta

(:process driving
  :parameters (?from ?to - location)
  :precondition (moving ?from ?to)
  :effect (increase (progress ?from ?to) (* #t (effective-speed ?from ?to))))

(:event arrive
  :parameters (?from ?to - location)
  :precondition (and (moving ?from ?to) (>= (progress ?from ?to) (distance ?from ?to)))
  :effect (and
    (not (moving ?from ?to)) (at ?to) (prev ?from)
    (increase (total-dist) (distance ?from ?to))
    (increase (total-time) (arc-time ?from ?to))          ; tempo di guida
    (increase (total-time) (signal-delay ?to))            ; ritardo semaforico (SUMO)
    (increase (total-time) (congestion-delay ?to))        ; ritardo congestione
    (assign (progress ?from ?to) 0)))
```

Il processo `driving` fa avanzare `progress` in modo continuo tramite la variabile temporale `#t`; l'evento `arrive` si attiva automaticamente quando `progress >= distance` e aggiorna `total-time` in modo discreto, sommando il tempo di guida, il ritardo semaforico e quello di congestione del nodo di arrivo; l'azione `start-move` aggiunge il tempo di svolta. L'aggiornamento di `total-time` avviene esclusivamente negli eventi/azioni discreti, e non nel processo continuo, per evitare che ENHSP tratti il tempo accumulato come variabile continua da campionare ad ogni istante, con conseguente aumento del costo computazionale della ricerca.

> **Nota sulla scomposizione del tempo.** La *timeline* stampata da ENHSP (gli istanti delle azioni) riflette il solo tempo di **guida** simulato dal processo continuo; i ritardi (semaforo, congestione, svolta) sono incrementi *discreti* su `total-time` e non fanno avanzare l'orologio simulato. Il costo effettivo del piano è quindi la somma: **total-time = guida + semafori + congestione + svolte**. Esempio (zona piccola): 242,7 s = 85,6 (guida) + 17,1 (semafori) + 130 (congestione) + 10,0 (svolte).

---

## Interfaccia web (`webapp/`)

Il sistema include un'interfaccia web sviluppata con **Flask** e **Leaflet.js** che consente di:

1. Caricare un file `.osm` tramite interfaccia grafica
2. Visualizzare la rete stradale su mappa interattiva, con i nodi semaforizzati evidenziati
3. Selezionare start e goal cliccando sui nodi
4. Avviare la risoluzione con ENHSP e visualizzare il percorso ottimale sulla mappa
5. Aprire direttamente la simulazione in sumo-gui tramite apposito pulsante

Il problema PDDL+ generato dalla webapp viene salvato automaticamente come `pddl_files/problem_custom.pddl` ad ogni risoluzione, consentendo di riesaminarlo o di avviarne la visualizzazione SUMO da riga di comando.

La webapp usa lo **stesso modello** della generazione da riga di comando: emette i fatti `turn-time`, l'init `prev` e i `signal-delay` realistici (mappa unita dei ritardi SUMO delle tre zone; per un incrocio non presente nei dati SUMO usa il valore realistico di default di un incrocio a 2 fasi, ~17 s). Il parser del piano gestisce l'azione `start-move` a tre argomenti per ricostruire il percorso da passare a SUMO.

---

## Struttura del repository

```
├── README.md
├── requirements.txt
├── setup.bat
├── build_problems.py          # Genera i file problem_*.pddl da OSM
├── extract_sumo_data.py       # Estrae semafori/settaggi/turn dai net.xml SUMO
├── download_dublin_map.py     # Scarica le mappe OSM tramite osmnx
├── convert_to_osm.py          # Converte OSM in net.xml tramite netconvert
├── sumo_visualize.py          # Visualizza il piano in sumo-gui
├── dublin_map.png             # Mappa di riferimento
├── dublin_streets.graphml     # Grafo stradale in formato GraphML
│
├── sumo_extracted/            # Dati estratti da SUMO (generati da extract_sumo_data.py)
│   ├── sumo_data_piccola.json
│   ├── sumo_data_media.json
│   ├── sumo_data_grande.json
│   └── report.md
│
├── osm_files/                 # Dati OSM scaricati
│   ├── dublin_piccola_centro.osm
│   ├── dublin_media_residenziale.osm
│   ├── dublin_grande_porto.osm
│   └── dublino.osm
│
├── net_files/                 # Reti stradali per SUMO
│   ├── piccola.net.xml
│   ├── media.net.xml
│   └── grande.net.xml
│
├── cfg_files/                 # File di configurazione SUMO
│   ├── {zona}.sumocfg
│   ├── {zona}_piano.rou.xml
│   └── gui_{zona}.xml
│
├── pddl_files/                # File PDDL+ del progetto
│   ├── domain.pddl
│   ├── problem_piccola.pddl
│   ├── problem_piccola_manuale.pddl  # Versione precedente (backup, nomi via)
│   ├── problem_media.pddl
│   ├── problem_grande.pddl
│   ├── problem_custom.pddl    # Generato dalla webapp (aggiornato automaticamente)
│   ├── output_piccola.txt
│   ├── output_media.txt
│   ├── output_grande.txt
│   └── run.py
│
├── encoder/                   # Analisi delle strade OSM
│   ├── encoder.py
│   ├── strade_piccola.txt
│   ├── strade_media.txt
│   └── strade_grande.txt
│
└── webapp/                    # Interfaccia web (Flask + Leaflet)
    ├── app.py
    └── templates/
        └── index.html
```

---

## Requisiti

| Strumento / Libreria | Versione minima | Note                                      |
|----------------------|-----------------|-------------------------------------------|
| Python               | 3.9             |                                           |
| Java                 | 17              | Necessario per l'esecuzione di ENHSP      |
| SUMO                 | 1.x             | Richiesto solo per la visualizzazione     |
| flask                | 3.0             | Installato tramite pip                    |
| up-enhsp             | latest          | Include il file .jar di ENHSP             |
| osmnx                | 1.9             | Per il download e la manipolazione di OSM |

---

## Installazione ed esecuzione

### Installazione delle dipendenze Python

```bash
pip install flask osmnx up-enhsp
```

### Avvio dell'interfaccia web

```bash
cd webapp
python app.py
```

L'interfaccia è accessibile all'indirizzo `http://localhost:5000`.

### Risoluzione da riga di comando

```bash
cd pddl_files
python run.py piccola    # oppure: media, grande
```

### Visualizzazione in SUMO

```bash
# Zona predefinita
python sumo_visualize.py piccola    # oppure: media, grande

# Percorso generato dalla webapp
python sumo_visualize.py pddl pddl_files/problem_custom.pddl piccola
```

---

## Risultati

### Zona Piccola

Piano trovato da ENHSP — 14 nodi, 20 archi, 4 semafori sul percorso:

| Tempo (s) | Nodo                        | Semaforo |
|-----------|-----------------------------|----------|
| 0         | Liffey Street Upper (START) |          |
| 10        | Wellington Quay Est         | +30 s    |
| 128       | Cork Hill                   | +30 s    |
| 132       | Cork Hill Sud               | +30 s    |
| ~314      | Aungier Street (GOAL)       | +30 s    |

Distanza percorsa: 1.57 km — Ritardo semaforico totale: 120 s — Tempo totale: ~314 s

### Zona Media

Piano trovato da ENHSP — 50 nodi, 93 archi, 18 azioni `start-move`:

Distanza percorsa: 1.62 km — Durata del piano: 160 s

### Zona Grande

Piano trovato da ENHSP — 120 nodi, 206 archi, 15 azioni `start-move`:

Distanza percorsa: 1.33 km — Durata del piano: 147 s

### Riepilogo

| Zona    | Nodi | Archi | Distanza | Azioni | Durata piano | Nodi espansi | Tempo pianificazione |
|---------|------|-------|----------|--------|--------------|--------------|----------------------|
| Piccola | 14   | 20    | 1.57 km  | 13     | ~314 s       | —            | —                    |
| Media   | 50   | 93    | 1.62 km  | 18     | 160 s        | 179          | 141 ms               |
| Grande  | 120  | 206   | 1.33 km  | 15     | 147 s        | 163          | 80 ms                |

---

## Riferimenti

- Fox, M., Long, D. (2002). *PDDL+: Modelling Continuous Time Dependent Effects*. AIPS Workshop on Planning for Temporal Domains.
- Scala, E., Haslum, P., Thiébaux, S., Ramírez, M. (2016). *Interval-Based Relaxation for General Numeric Planning*. ECAI 2016.
- Boeing, G. (2017). *OSMnx: New Methods for Acquiring, Constructing, Analyzing, and Visualizing Complex Street Networks*. Computers, Environment and Urban Systems.
- Lopez, P.A. et al. (2018). *Microscopic Traffic Simulation using SUMO*. IEEE ITSC 2018.

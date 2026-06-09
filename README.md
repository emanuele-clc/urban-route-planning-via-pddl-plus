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
| **Grande** | Phibsborough / Nord | 3000m | 120 | 206 | St Patrick's Rd → Botanic Ave | 1.33 km | 147 s |

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

Lo script legge tutti i nodi e tutte le strade percorribili in auto (esclude ciclabili, sentieri, ecc.). Durante questa fase rileva anche i **nodi semaforo**: nodi con tag `highway=traffic_signals` in OSM, che vengono raccolti in un insieme separato e poi propagati nel PDDL come ritardo d'attesa.

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
- `(= (signal-delay A) 30)` oppure `0` — il ritardo semaforico del nodo A: **30 secondi** se il nodo OSM è taggato `highway=traffic_signals`, **0** altrimenti

Risultato: un file PDDL+ che descrive fedelmente la mappa stradale reale, con tutti i numeri derivati direttamente dai dati geografici di OpenStreetMap, inclusi i semafori.

### Fase 3 — ENHSP risolve il problema PDDL+

`run.py` lancia il planner **ENHSP** passandogli `domain.pddl` e il problema scelto. ENHSP cerca la sequenza di azioni che porta dalla START alla GOAL minimizzando `(total-time)` — il tempo totale incluse le attese ai semafori.

Il dominio definisce tre costrutti PDDL+ e due funzioni numeriche chiave:

```pddl
; signal-delay: 30s per i semafori OSM, 0 per tutti gli altri nodi
; total-time:   tempo cumulativo (guida + attese semafori)

; AZIONE istantanea: il veicolo inizia a percorrere una strada
(:action start-move
  :parameters (?from ?to - location)
  :precondition (and (at ?from) (road ?from ?to))
  :effect (and (not (at ?from)) (moving ?from ?to) (assign (progress ?from ?to) 0)))

; PROCESSO continuo: solo progress avanza — total-time non è continuo (performance)
(:process driving
  :parameters (?from ?to - location)
  :precondition (moving ?from ?to)
  :effect (increase (progress ?from ?to) (* #t (speed ?from ?to))))

; EVENTO automatico: arriva, calcola il tempo del tratto e aggiunge il ritardo semaforico
(:event arrive
  :parameters (?from ?to - location)
  :precondition (and (moving ?from ?to) (>= (progress ?from ?to) (distance ?from ?to)))
  :effect (and (not (moving ?from ?to)) (at ?to)
               (increase (total-dist) (distance ?from ?to))
               (increase (total-time) (/ (distance ?from ?to) (speed ?from ?to)))
               (increase (total-time) (signal-delay ?to))
               (assign (progress ?from ?to) 0)))
```

`#t` è la variabile temporale continua di PDDL+. Il processo `driving` fa avanzare `progress`; l'evento `arrive` scatta automaticamente quando `progress >= distance` e accumula il tempo di guida (`dist/speed`) più l'attesa semaforica (`signal-delay`). Il `total-time` viene aggiornato **solo negli eventi discreti** — non nel processo continuo — per mantenere la ricerca efficiente.

> ⚠️ Usare `(increase (total-time) #t)` nel processo lo renderebbe una variabile continua, costringendo ENHSP a campionarla ad ogni istante e rallentando enormemente la ricerca. Il calcolo discreto `dist/speed` nell'evento `arrive` produce lo stesso risultato numerico senza questo costo.

ENHSP produce un piano come questo:
```
0:     (start-move liffey_st_upper wellington_quay_e)
10.0:  (start-move wellington_quay_e aston_quay)   ← +30s semaforo
...
183.0: (start-move sgeorges_m aungier_st)
~250s  GOAL ✅   —   Planning Time: ~50ms
```

Il piano viene salvato in `output_piccola.txt`.

> ⚠️ Il planner usa il motore `-s aibr`. Non usare `sat-hadd` con domini PDDL+ che
> contengono processi ed eventi: ritorna h(I)=0.0 e non converge mai.

### Fase 4 — Tradurre il piano ENHSP in una rotta SUMO

SUMO e PDDL+ non si parlano direttamente. Il piano ENHSP usa nomi PDDL come `liffey_st_upper`; SUMO usa ID numerici di archi stradali come `4396046#0`. Il collegamento viene fatto con **Dijkstra** sul file `net.xml`:

1. Per ogni zona si conoscono la junction SUMO di partenza e quella di arrivo (ricavate dagli stessi ID OSM usati nel PDDL)
2. Lo script `sumo_visualize.py` analizza il `net.xml` corrispondente: costruisce un grafo diretto dove i nodi sono le junction SUMO e gli archi sono i segmenti stradali con la loro lunghezza in metri
3. Dijkstra trova il percorso a distanza minima tra le due junction
4. Il risultato è una sequenza ordinata di ID archi SUMO — ciascun arco inizia esattamente dove termina il precedente

**Esempio (zona piccola):** da junction `659788` (Liffey Street Upper) alla junction cluster di Aungier Street → 24 archi consecutivi:
```
4396046#0 4396046#1 18927706 1478689539 1062391643#0 4396056 ...
```

Questa sequenza è **salvata in `sumo_visualize.py`** per le tre zone predefinite. Per i percorsi generati dalla webapp, la stessa logica Dijkstra viene eseguita dinamicamente a ogni chiamata (vedi [Modalità dinamica](#modalità-dinamica--percorsi-dalla-webapp)).

> ⚠️ **Nomi PDDL vs ID SUMO**: I nomi PDDL usati dalla webapp seguono il pattern `n` + ultime 7 cifre dell'ID OSM (es. `n1193756` → junction SUMO il cui ID termina con `1193756`). Per i nomi espliciti come `liffey_st_upper` usati nei PDDL predefiniti, la corrispondenza è hardcoded nelle coordinate delle zone. SUMO può unire nodi OSM vicini in junction cluster (es. `cluster_11742165391_...`) — Dijkstra le trova correttamente perché l'ID OSM originale è incorporato nel nome del cluster.

### Fase 5 — SUMO visualizza il percorso

`sumo_visualize.py` genera tre file XML e apre sumo-gui:

**`{zona}_piano.rou.xml`** — dice a SUMO come è fatta l'auto e dove deve andare:
```xml
<vType id="auto" maxSpeed="4.0" color="1,0,0" shape="passenger" accel="1.5" decel="3.0"/>
<route id="piano_enhsp" edges="4396046#0 4396046#1 18927706 ..."/>
<vehicle id="veicolo_enhsp" type="auto" route="piano_enhsp" depart="1"/>
```
Il tag `edges` contiene la sequenza esatta di archi — l'auto li percorre in ordine, senza mai deviare. Il percorso non è casuale: è esattamente quello trovato da ENHSP (o calcolato da Dijkstra per la modalità dinamica).

**`{zona}.sumocfg`** — dice a SUMO quali file caricare (rete stradale, percorso, grafica).

**`gui_{zona}.xml`** — imposta zoom e posizione iniziale della telecamera. La telecamera è centrata sul **punto di partenza** del veicolo (coordinate SUMO della junction di start), così l'auto è subito visibile all'apertura senza dover fare Ctrl+A.

SUMO applica poi la **fisica realistica**: accelerazione, frenata, rispetto dei semafori. I semafori che si vedono nella simulazione vengono dai dati OSM reali di Dublino — SUMO li applica automaticamente al veicolo. Il piano PDDL+ non li modella, quindi il veicolo nella simulazione si ferma al rosso mentre il piano teorico assume velocità costante. Questo spiega la differenza tra il tempo del piano (194s) e il tempo reale (≈15 min secondo Google Maps): il piano PDDL+ è un **lower bound ottimistico**.

Il veicolo **scompare dalla mappa quando raggiunge la destinazione** — è il comportamento normale di SUMO: rimuove il veicolo al termine del suo itinerario.

---

### Modalità dinamica — percorsi dalla webapp

`sumo_visualize.py` supporta anche una **modalità dinamica** che calcola automaticamente il percorso SUMO a partire da qualsiasi file PDDL generato dalla webapp, senza dover modificare lo script a mano.

**Come funziona:**

1. La webapp genera il PDDL con i nodi scelti dall'utente (es. `n1193756` → `n5832633`) e lo salva automaticamente come `files/pddl_files/problem_custom.pddl`
2. Lo script legge il file PDDL, estrae i nomi di start e goal, li mappa alle junction SUMO corrispondenti tramite i suffix numerici (es. `n1193756` → junction il cui ID termina con `1193756`)
3. Dijkstra calcola la rotta ottimale sul `net.xml` della zona scelta
4. SUMO-gui si apre con il percorso calcolato

**Esecuzione:**

```bash
cd files
python sumo_visualize.py pddl pddl_files/problem_custom.pddl piccola
# oppure: media, grande — deve corrispondere alla zona usata nella webapp
```

**Auto-salvataggio dalla webapp:** ogni volta che si preme "Risolvi con ENHSP" nella webapp, il file PDDL generato viene salvato automaticamente come `files/pddl_files/problem_custom.pddl`. Dopo aver ottenuto il percorso nella webapp, basta eseguire il comando sopra per aprire la stessa rotta in SUMO.

> **Nota:** la corrispondenza PDDL→SUMO funziona con i nomi in formato `n` + cifre generati dalla webapp. Lo script prova automaticamente tutte e tre le reti disponibili e usa quella in cui trova i nodi — non serve specificare la zona corretta. I PDDL predefiniti (piccola, media, grande) usano nomi espliciti come `liffey_st_upper` — per quelli si usa il comando senza `pddl` (es. `python sumo_visualize.py piccola`).

---

## Struttura del progetto

```
progetto_maratea/
├── README.md
├── requirements.txt             # Dipendenze Python (up-enhsp, osmnx, flask)
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
    │   ├── problem_custom.pddl  # Ultimo problema generato dalla webapp (auto-aggiornato)
    │   └── run.py               # Lancia ENHSP e mostra il piano
    │
    ├── encoder/                 # Analisi strade OSM
    │   ├── encoder.py           # Genera report .txt per ogni zona
    │   ├── strade_piccola.txt
    │   ├── strade_media.txt
    │   └── strade_grande.txt
    │
    ├── webapp/                  # Interfaccia web interattiva
    │   ├── app.py               # Server Flask (backend)
    │   └── templates/
    │       └── index.html       # Frontend con mappa Leaflet
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
| Python | 3.9+ | Vedi istruzioni sotto |
| Java | 17+ | Necessario per ENHSP |
| SUMO | 1.x | Solo per la visualizzazione desktop |
| flask | ≥ 3.0 | Installato via pip |
| up-enhsp | latest | Installato via pip — include il .jar di ENHSP |
| osmnx | ≥ 1.9 | Installato via pip |

---

## Installazione completa (prima volta)

### 1. Python 3.9+

**Windows** — opzione A, da terminale con winget:
```powershell
winget install Python.Python.3.12
```
oppure scarica l'installer da https://www.python.org/downloads/ e durante l'installazione spunta **"Add Python to PATH"**.

**Mac:**
```bash
brew install python
```
Se non hai Homebrew: `/bin/bash -c "$(curl -fsSL https://raw.githubusercontent.com/Homebrew/install/HEAD/install.sh)"`

**Linux (Ubuntu/Debian):**
```bash
sudo apt update && sudo apt install python3 python3-pip -y
```

Verifica:
```bash
python --version        # Windows
python3 --version       # Mac / Linux
```

---

### 2. Java 17+

ENHSP è un programma Java — senza Java non parte.

**Windows** — da terminale con winget:
```powershell
winget install EclipseAdoptium.Temurin.17.JDK
```
oppure scarica l'installer da https://adoptium.net

**Mac:**
```bash
brew install temurin@17
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt install openjdk-17-jdk -y
```

Verifica:
```bash
java -version
```
Deve rispondere con una versione ≥ 17. Se il comando non viene riconosciuto su Windows, riapri il terminale dopo l'installazione.

---

### 3. SUMO (solo per la visualizzazione desktop)

SUMO serve solo per lo script `sumo_visualize.py`. Per la webapp non è necessario.

**Windows:** scarica e installa da https://sumo.dlr.de/docs/Downloads.php

**Mac:**
```bash
brew install sumo
```

**Linux (Ubuntu/Debian):**
```bash
sudo apt install sumo sumo-tools -y
```

Verifica:
```bash
sumo --version
```

---

### 4. Dipendenze Python (flask, osmnx, up-enhsp)

Entra nella cartella della webapp e installa tutto:

**Windows:**
```powershell
cd files\webapp
python -m pip install flask osmnx up-enhsp
```

**Mac / Linux:**
```bash
cd files/webapp
pip install flask osmnx up-enhsp
```

**Con uv** (se preferisci ambienti virtuali isolati, qualsiasi sistema):
```bash
cd files/webapp
uv venv
uv pip install flask osmnx up-enhsp
```

> `up-enhsp` è il pacchetto che scarica e installa il file `.jar` di ENHSP dentro Python. Dopo l'installazione, la webapp lo trova automaticamente senza configurazioni aggiuntive.

---

## Come eseguire

### Interfaccia web (modo consigliato)

L'interfaccia web permette di caricare un file `.osm`, visualizzare la mappa, scegliere start e goal cliccando sui nodi, e risolvere con ENHSP — tutto dal browser.

**Con pip:**
```bash
cd files/webapp
python app.py
```

**Con uv:**
```bash
cd files/webapp
uv run python app.py
```

Poi apri il browser su **http://localhost:5000**.

Flusso d'uso:
1. Trascina un file `.osm` nella zona di upload (o usa uno di quelli in `osm_files/`)
2. Imposta il numero massimo di nodi con lo slider, oppure spunta **"Tutti i nodi"** per caricare l'intera mappa senza limiti
3. Premi **"Visualizza Mappa"** — appare la rete stradale con i semafori evidenziati in **giallo ambra** 🟡
4. Clicca un nodo per impostarlo come **Start** 🟢 o **Goal** 🔴; i nodi semaforo mostrano nel popup il badge `🚦 +30s attesa media`
5. Premi **"Risolvi con ENHSP"** — il percorso ottimale viene tracciato in blu; i semafori sul percorso diventano arancio brillante e le statistiche mostrano quanti semafori vengono attraversati e il ritardo totale accumulato
6. Se ENHSP trova la soluzione, appare il bottone **"▶ Apri in SUMO"** — cliccandolo si apre automaticamente sumo-gui con il percorso già caricato, senza dover eseguire comandi da terminale

---

### Da riga di comando

#### Risolvere un problema PDDL+ con ENHSP

```bash
cd files/pddl_files
python run.py piccola   # oppure: media, grande
```

#### Visualizzare il percorso in SUMO

**Zona predefinita** (piccola, media o grande):
```bash
cd files
python sumo_visualize.py piccola   # oppure: media, grande
```

**Percorso generato dalla webapp** (modalità dinamica):
```bash
cd files
python sumo_visualize.py pddl pddl_files/problem_custom.pddl piccola
```

In modalità dinamica, lo script legge il PDDL, identifica start e goal e cerca automaticamente la rete giusta tra `piccola.net.xml`, `media.net.xml` e `grande.net.xml` — non è necessario che la zona passata corrisponda esattamente a quella usata nella webapp. Il file `problem_custom.pddl` viene aggiornato automaticamente dalla webapp a ogni risoluzione.

> **Nota:** dalla webapp è anche possibile usare direttamente il bottone **"▶ Apri in SUMO"** che compare dopo ogni risoluzione riuscita — equivale a eseguire questo comando ma senza aprire il terminale.

Si apre sumo-gui con la rete di Dublino e il veicolo rosso pronto a partire:
- **▶ Play** per avviare la simulazione
- La telecamera è già centrata sul punto di partenza — il veicolo è visibile immediatamente
- **Ctrl+A** per adattare la vista all'intera rete
- Click destro sull'auto → **Track** per seguirla lungo il percorso
- Il veicolo sparisce quando raggiunge la destinazione (comportamento normale di SUMO)

#### (Opzionale) Rigenerare i problemi media e grande

```bash
cd files
python build_problems.py
```

#### (Opzionale) Analizzare le strade OSM

```bash
cd files/encoder
python encoder.py
```

---

## Risultati

### Zona Piccola ✅

**Piano trovato da ENHSP** con semafori OSM integrati:

| Tempo (s) | Posizione | Semaforo |
|-----------|-----------|----------|
| 0 | Liffey Street Upper ← START | |
| 10 | Wellington Quay Est | 🚦 +30s |
| 12 | Aston Quay | |
| 33 | Ormond Quay West | |
| 51 | Capel Street Nord | |
| 57 | Capel Street / Quay | |
| 67 | Grattan Bridge Sud | |
| 128 | Cork Hill | 🚦 +30s |
| 132 | Cork Hill Sud | 🚦 +30s |
| 155 | Dame Street Est | |
| 160 | South Gt George's St Nord | |
| 183 | South Gt George's St Centro | |
| **~314** | **Aungier Street ← GOAL ✅** | 🚦 +30s |

**Distanza:** 1.57 km — **Semafori attraversati:** 4 — **Ritardo semafori:** +120s  
**Tempo totale (guida + semafori):** ~314 s — **Metrica minimizzata:** `total-time`  
**Confronto:** ~15 min reali (Google Maps) — il piano è un lower bound (traffico zero, attesa semaforo media fissa)

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
| 0.0 | St Patrick's Road ← START |
| 5.6 – 138.8 | 14 tratti intermedi |
| **147** | **Botanic Avenue ← GOAL ✅** |

**Distanza:** 1.33 km — **Tempo teorico:** 147 s

### Riepilogo

| Zona | Nodi | Archi | Distanza | Semafori | Tempo PDDL+ | Note |
|------|------|-------|----------|----------|-------------|------|
| Piccola | 14 | 20 | 1.57 km | 4 × +30s | ~314 s | Metrica: total-time |
| Media | 50 | 93 | 1.62 km | 6 rilevati | da ricalcolare | Metrica: total-time |
| Grande | 120 | 206 | 1.33 km | 6 rilevati | da ricalcolare | Metrica: total-time |

Il piano PDDL+ ora modella i semafori reali di OSM con un'attesa media di 30 secondi per ogni nodo semaforizzato sul percorso. La metrica minimizzata è `total-time`, non più `total-dist`. SUMO applica i semafori dinamicamente con fasi reali (verde/giallo/rosso), rendendo la simulazione ancora più realistica.

---

## Riferimenti

- PDDL+: Fox & Long (2002), *PDDL+: Modeling continuous time dependent effects*
- ENHSP: Scala et al. (2016), *Interval-Based Relaxation for General Numeric Planning*
- OSMnx: Boeing (2017), *OSMnx: New Methods for Acquiring, Constructing, Analyzing, and Visualizing Complex Street Networks*
- SUMO: Lopez et al. (2018), *Microscopic Traffic Simulation using SUMO*

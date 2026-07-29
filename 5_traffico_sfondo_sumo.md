# Traffico di sfondo nella visualizzazione SUMO interattiva

Bozza di valutazione e proposta implementativa. Richiesta originale: oltre al
veicolo che segue il piano ENHSP, aggiungere altri veicoli "fittizi" nelle
simulazioni SUMO, per dare l'impressione di traffico urbano quando si apre
la GUI dalla webapp ("Apri in SUMO").

## 1. Perimetro della richiesta

Questa proposta riguarda **solo** la visualizzazione interattiva
(`scripts/sumo_visualize.py`, invocata da `webapp/app.py` → `/api/sumo` →
bottone "Apri in SUMO" della webapp). È un arricchimento **visivo/dimostrativo**:

- **Non tocca la pianificazione PDDL+.** Il modello di congestione che entra
  nel problema PDDL (`vehicle-count`, `congestion-delay`,
  `congestion-factor`, vedi `webapp/osm_graph.py::compute_vehicle_counts` /
  `compute_congestion_delay` e `webapp/pddl_writer.py`) è calcolato **prima**
  di ENHSP con un Dijkstra sintetico su 10 coppie O-D casuali nel
  sottografo PDDL — è un numero statico, già fissato nel file `.pddl` prima
  che SUMO entri in gioco. Aggiungere veicoli nella GUI non lo cambia e non
  lo dovrebbe cambiare: sono due pipeline indipendenti, e non vanno
  confuse a livello di naming (vedi punto 4.i).
- **Non tocca `/api/compare_sumo`** (`scripts/compare_sumo.py`, punto 4
  della roadmap): quello scenario ha *già* traffico multi-veicolo (vedi
  sotto), perché misura l'effetto dei semafori ottimizzati su un campione
  di domanda condiviso, in modalità headless.

## 2. Stato attuale (cosa esiste già)

| Script | Veicoli | Sorgente O-D | Routing | Esecuzione |
|---|---|---|---|---|
| `sumo_visualize.py` | **1** (`veicolo_enhsp`) | piano ENHSP (via PDDL) | segmento-per-segmento su nodi PDDL | `sumo-gui` interattivo (`Popen`) |
| `compare_sumo.py` | N (tipicamente ~60) | `sumo_extracted/demand_<zona>.json` | Dijkstra puro, precalcolato una volta | `sumo` headless, 2 run (baseline/ottimizzato) |

Punti chiave emersi dall'analisi del codice:

- `sumo_visualize.py` scrive il `.rou.xml` con un'unica chiamata
  `.format()` che produce esattamente un `<vType>` e un `<vehicle>`
  (righe ~329-341). Non c'è alcun ciclo, non c'è caricamento di domanda:
  zero supporto multi-veicolo oggi.
- `compare_sumo.py::build_routes()` ha **già** tutta la logica per
  trasformare un set di coppie O-D in N `<vehicle>` instradati: legge
  `demand_<zona>.json`, risolve ogni id OSM in una giunzione SUMO
  (`resolve_junction`, quasi identica a `pddl_name_to_junction` già
  presente in `sumo_visualize.py`), calcola il percorso con Dijkstra
  (`dijkstra_edges`, quasi identica al `dijkstra` già in
  `sumo_visualize.py`), e scrive un `<vehicle>` per coppia con partenze
  scaglionate (`DEPART_PERIOD = 3.0s`) e velocità realistica
  (`VEH_MAX_SPEED = 13.89 m/s`, 50 km/h).
- `demand_<zona>.json` esiste già per le 3 zone precaricate (piccola/
  media/grande), generato da `scripts/generate_demand.py` (60 coppie O-D,
  seed 42, metodo primario `randomTrips.py` sull'intera rete + snap al
  sottografo PDDL, fallback Dijkstra uniforme).
- Nessun uso di `traci`/`sumolib` in tutto il progetto: ogni interazione
  con SUMO è o `Popen(sumo-gui, ...)` (visualizzazione) o
  `subprocess.run(sumo, ...)` headless (confronto). Niente `duarouter`,
  niente `<flow>`: ogni veicolo è sempre un `<vehicle>` con `<route>`
  esplicita, mai instradamento dinamico lasciato a SUMO.

**Conseguenza pratica**: l'infrastruttura di routing (grafo SUMO da
`net.xml`, Dijkstra, risoluzione id-OSM→giunzione) è già scritta e
duplicata due volte (in `sumo_visualize.py` e in `compare_sumo.py`).
Aggiungere traffico di sfondo alla GUI interattiva è in larga parte
"riusare quello che c'è", non costruire qualcosa di nuovo.

## 3. Sorgente O-D — DECISO: pesata sulla congestione già calcolata, non riuso di `demand_<zona>.json`

La tentazione ovvia era: prendi `demand_<zona>.json`, applica la stessa
`build_routes()` di `compare_sumo.py`, aggiungi i veicoli risultanti al
`.rou.xml` di `sumo_visualize.py`. **Non funziona bene per il caso d'uso
principale**, per due motivi:

1. **La webapp non usa mai una "zona" fissa.** `/api/sumo` viene chiamato
   sempre in modalità dinamica (`pddl <problem_custom.pddl>`), su un
   sottografo generato da un file OSM caricato dall'utente con parametri
   arbitrari (`zone` salvata come stringa `'custom'` in `app.py`). Non
   esiste — e non avrebbe senso creare — un `demand_custom.json`: cambia
   ad ogni generazione.
   - `sumo_visualize.py` in modalità dinamica sceglie comunque una delle 3
     reti fisse (`piccola/media/grande.net.xml`) per compatibilità di
     giunzioni (quella con più nodi del piano ENHSP mappabili,
     `compute_edges_from_pddl`), ma questo non rende `demand_<quella
     zona>.json` geograficamente rilevante: quel file contiene coppie O-D
     legate al sottografo PDDL *statico* di 14/50/120 nodi usato da
     `build_problems.py` per quella zona, che può essere ovunque nella
     rete e non ha alcun legame con dove si trova il percorso
     dell'utente.
2. **La telecamera della GUI è fissa vicino allo start del percorso**
   (`viewport x/y/zoom` centrato sul nodo di partenza). Se il traffico di
   sfondo è sparso genericamente nella zona (specie su `grande`, rete da
   17 MB), la stragrande maggioranza dei veicoli di sfondo non entrerebbe
   mai nel campo visivo: SUMO li simulerebbe per niente, sprecando tempo
   di calcolo senza beneficio visivo.

**Deciso**: i veicoli di sfondo per un dato arco vengono generati in
numero **casuale, ma proporzionato al grado di congestione già calcolato
per quell'arco** (vedi §4.a per il meccanismo). Questo risolve entrambi i
problemi sopra senza bisogno di `demand_<zona>.json` né di un'euristica a
raggio/bounding-box:

- **Nessuna dipendenza da zone fisse**: il `vehicle-count`/
  `congestion-factor` per arco è già calcolato per QUALUNQUE OSM caricato
  dall'utente (`webapp/osm_graph.py::compute_vehicle_counts`) e già scritto
  dentro `problem_custom.pddl` con gli stessi nomi-nodo che
  `sumo_visualize.py` sa già mappare a giunzioni SUMO
  (`pddl_name_to_junction`). Zero nuovo canale dati: basta fare un regex
  scan delle righe `(= (vehicle-count nome_a nome_b) N)` nello stesso file
  PDDL già letto per start/goal.
- **Visibilità garantita gratis**: `problem_custom.pddl` contiene solo gli
  archi del corridoio locale attorno al percorso risolto da ENHSP (vedi
  `select_local_subgraph` lato webapp), quindi il traffico di sfondo
  ricade automaticamente vicino a dove guarda la telecamera — senza
  bisogno di filtrare per raggio.

**Copertura — DECISO**: solo modalità dinamica/webapp (`pddl
<problem_custom.pddl>`), unico caso che ha motivato la richiesta. I preset
statici a CLI (`python scripts/sumo_visualize.py piccola`, senza PDDL) sono
**fuori scope**: non hanno un `problem_custom.pddl` da cui leggere la
congestione, quindi non ricevono traffico di sfondo in questo intervento
(restano com'è oggi, un solo veicolo).

## 4. Punti chiave di design da discutere

a. **Sorgente O-D e numero di veicoli — DECISO**: per ogni arco `(a, b)`
   presente in `problem_custom.pddl`, il numero di veicoli di sfondo
   instradati ad attraversarlo è **casuale, con un tetto che scala col
   `vehicle-count`/`congestion-factor` già calcolato per quell'arco** —
   non un moltiplicatore fisso e deterministico. Es. (da tarare in fase di
   test, vedi §7):
   ```python
   n_bg_for_edge = random.randint(0, max(1, round(vehicle_count * K)))
   ```
   con `K` fattore di scala. Archi con `vehicle-count = 0` possono comunque
   generare 0 o 1 veicolo (range casuale con tetto basso), invece di
   restare sempre vuoti: dà un minimo di vita anche alle strade
   "tranquille" senza forzare un numero fisso.
   **Vincolo esplicito**: il totale dei veicoli di sfondo generati (somma
   su tutti gli archi) va limitato con un **cap globale**, per non
   intaccare la fluidità della GUI — se la somma casuale supera il cap, si
   tronca/ricampiona. Il valore esatto del cap resta da tarare
   empiricamente (vedi punto aperto più sotto e §7).

b. *(assorbito dal punto a — il "numero di veicoli" non è più un parametro
   singolo ma il risultato del campionamento per-arco + cap globale)*

c. **Distinzione visiva e velocità — DECISO**: il veicolo ENHSP resta
   `sigma="0.0"` (deterministico, per restare prevedibile come oggi) a
   `maxSpeed="4.0"` (lento apposta, leggibilità demo — invariato). Il
   traffico di sfondo usa invece **`sigma` moderato (proposta 0.3-0.5)**
   con `maxSpeed` realistico (~13.89 m/s, come in `compare_sumo.py`): dà
   una guida più naturale/variabile invece che robotica, restando comunque
   riproducibile a parità di seed. Il traffico di sfondo **supererà
   visibilmente** il veicolo tracciato (accettato: è realistico, un'auto
   ferma a un incrocio viene superata). **Colore — DECISO**: `vType`
   distinto dal rosso (`color="1,0,0"`) del veicolo ENHSP — traffico di
   sfondo in giallo/ambra (`color="1,0.8,0"`), scelto perché non si
   confonde né col rosso del veicolo tracciato né con altri colori già
   usati nella webapp (verde per gli incroci aperti/start, rosso per
   chiusure/goal — vedi `webapp/templates/index.html`).

d. **Durata simulazione — DECISO**: `cfg['end']` **resta ancorato solo al
   veicolo ENHSP** (formula invariata, `dyn['total_length'] / 4.0` con
   margine) — la simulazione finisce quando l'auto rossa arriva a
   destinazione, non viene estesa per il traffico di sfondo. Conseguenza:
   il traffico di sfondo è **subordinato** a questa finestra temporale — se
   una macchina di sfondo non fa in tempo a partire o a completare il
   percorso prima che la finestra si chiuda, va bene così (è ambientazione,
   non viene misurata). Vedi discussione su cosa significa esattamente
   "finisce quando arriva" nella sezione punti aperti (§7).

e. **Semafori ottimizzati**: nessun lavoro aggiuntivo — si applicano
   automaticamente a tutti i veicoli del file di route, incluso il
   traffico di sfondo, esattamente come già succede con `--baseline` /
   `additional-files`.

f. **Copertura — DECISO**: solo modalità dinamica/webapp. Vedi §3.

g. **Esposizione in webapp — DECISO**: checkbox "include background
   traffic" nella sezione risultati vicino ai bottoni "Open in SUMO"
   (`webapp/templates/index.html`), **default ON** (la simulazione include
   il traffico di sfondo a meno che l'utente non la disattivi
   esplicitamente). Propagazione: JS (`launchSumo`) → body POST di
   `/api/sumo` (`background_traffic: true/false`) → argomento extra nel
   comando `Popen` verso `sumo_visualize.py` (`--traffic 0` per
   disattivare) → letto in testa allo script.

h. **Refactor — DECISO**: sì, farlo in questo stesso intervento. Vedi §5.4.

i. **Naming/documentazione**: chiamarlo esplicitamente "traffico di
   sfondo"/"background traffic" nella UI e nel codice, per non
   sovrapporlo concettualmente al "congestion model" statico già esistente
   (`vehicle-count`, `congestion-delay` nel PDDL) che condivide il
   vocabolario ma è tecnicamente tutt'altro (vedi §1).

j. **Fuori scope per questa proposta**: `/api/compare_sumo` (già ha
   traffico), feedback verso il modello di congestione PDDL, uso di
   `traci` per interattività runtime (non necessario: i veicoli di sfondo
   sono route statiche precalcolate come tutto il resto del progetto).

## 5. Bozza di implementazione

### 5.1 `scripts/sumo_visualize.py`

**Passo 1 — estrarre la congestione per arco dal PDDL già letto.** Vicino a
`m_start`/`m_goal` in `compute_edges_from_pddl`, aggiungere un regex scan
delle righe di congestione già presenti nel file:

```python
def parse_congestion(text):
    """{(nome_a, nome_b): vehicle_count} dalle righe
    (= (vehicle-count nome_a nome_b) N) gia' presenti nel PDDL."""
    out = {}
    for m in re.finditer(r'\(=\s*\(vehicle-count\s+(\S+)\s+(\S+)\)\s+(\d+)\)', text):
        out[(m.group(1), m.group(2))] = int(m.group(3))
    return out
```

**Passo 2 — generare i veicoli di sfondo per arco, con cap globale.**

*Fedeltà visiva vs validità della pianificazione — nota di principio.* Il
traffico di sfondo non può in nessun caso alterare il piano: viene generato
**dopo** che ENHSP ha già risolto e scritto `problem_custom.pddl` (nessun
canale torna indietro verso il solver — zero `traci`, zero run SUMO prima
del solve). Il rischio reale non è "falsare la pianificazione" ma mostrare
una scena che sembra contraddirla (arco che il modello giudica
congestionato ma appare vuoto in GUI). Per questo il conteggio per arco non
punta a un'uguaglianza numerica esatta con `vehicle-count` (che è un
punteggio sintetico da un campione di 10 viaggi, non un conteggio letterale
di auto), ma a **preservare l'ordine relativo**: un'estrazione `randint(0,
tetto)` indipendente per arco è troppo rumorosa per questo — un arco
trafficato (tetto alto) può per puro caso estrarre 0 mentre uno tranquillo
(tetto basso) estrae il massimo, invertendo visivamente l'ordine. Si usa
quindi una distribuzione di Poisson (varianza bassa quando la media è
bassa, cioè proprio nel nostro range 0-6) centrata sul valore atteso
`vehicle_count * k_scale`, campionata senza dipendenze esterne (il progetto
non usa `numpy`/`scipy` da nessuna parte — coerente con `random.Random`
usato ovunque — algoritmo di Knuth su `random` stdlib):

```python
def _poisson(rng, lam):
    """Campiona da Poisson(lam) senza numpy (algoritmo di Knuth) — coerente
    con random.Random gia' usato in tutto il progetto."""
    l_thresh = math.exp(-lam)
    k, p = 0, 1.0
    while p > l_thresh:
        k += 1
        p *= rng.random()
    return k - 1


def generate_background_traffic(congestion, graph, junc_ids, seed=42,
                                  k_scale=1.5, global_cap=40, max_hops=25,
                                  hard_cap_per_edge=6):
    """Per ogni arco PDDL (a,b) con dati di congestione, campiona da
    Poisson(vehicle_count * k_scale) il numero di veicoli da instradare
    lungo quell'arco (varianza bassa attorno al valore atteso -> preserva
    l'ordine relativo di congestione tra archi, a differenza di
    randint(0, tetto)). hard_cap_per_edge tronca la coda di Poisson per un
    singolo arco anomalo. Il percorso Dijkstra a->b viene calcolato UNA
    volta per arco e riusato per tutti i veicoli assegnati a
    quell'arco (non ricalcolato n volte identico). Tronca infine al cap
    globale per proteggere la fluidita' della GUI. Se 'congestion' e'
    vuoto (regex non ha trovato nulla nel PDDL), ritorna [] e lo script
    prosegue mostrando solo il veicolo ENHSP, come oggi."""
    rng = random.Random(seed)
    routes = []
    for (a_name, b_name), vc in congestion.items():
        a = pddl_name_to_junction(a_name, junc_ids)
        b = pddl_name_to_junction(b_name, junc_ids)
        if not a or not b:
            continue
        n = min(_poisson(rng, vc * k_scale), hard_cap_per_edge)
        if n == 0:
            continue
        edges = dijkstra(graph, a, b)          # calcolato una volta sola
        if edges and len(edges) <= max_hops:
            routes.extend([edges] * n)
    if len(routes) > global_cap:
        routes = rng.sample(routes, global_cap)
    return routes
```

`k_scale`, `global_cap`, `max_hops`, `hard_cap_per_edge` da tarare
empiricamente — vedi §7 (punti ancora aperti) e §8 (piano di verifica).

Modifica al blocco che scrive `ROU_PATH` (oggi righe ~329-341): sostituire
il singolo `.format()` con una costruzione incrementale della sezione
`<routes>`, aggiungendo un secondo `<vType id="traffic" sigma="0.4"
maxSpeed="13.89" color="1,0.8,0" .../>` (giallo/ambra, distinto dal rosso
`color="1,0,0"` del veicolo ENHSP) e un `<vehicle id="bg{i}">` per ogni
rotta di sfondo, con `depart` scaglionato (es. `round(i * 2.5 +
random.uniform(0, 1.5), 1)`).

Nuovo argomento CLI: `--traffic` (default: attivo, `--traffic 0` per
disattivare / sola visualizzazione del veicolo ENHSP come oggi), letto
insieme a `--baseline` nel blocco di parsing argv esistente in testa allo
script.

`cfg['end']` **non cambia**: resta calcolato solo sul veicolo ENHSP (vedi
§4.d). Il traffico di sfondo eredita la stessa finestra: se non fa in
tempo a partire/arrivare entro `cfg['end']`, resta semplicemente non
generato o viene rimosso a fine simulazione — nessuna logica di estensione
da aggiungere.

Ogni veicolo di sfondo percorre per intero il percorso che gli viene
assegnato (l'arco `(a, b)` per cui è stato generato, via Dijkstra) — non
si estende con hop aggiuntivi a monte/valle.

### 5.2 `webapp/app.py`

`/api/sumo` (`launch_sumo`): leggere `background_traffic` (bool, **default
`True`** se il campo manca) dal JSON body, e passare `--traffic 0` solo se
esplicitamente disattivato:

```python
if data.get('background_traffic', True) is False:
    cmd += ['--traffic', '0']
```

### 5.3 `webapp/templates/index.html`

Nuovo controllo UI vicino ai bottoni "Open in SUMO" (sezione risultati):
checkbox "Background traffic", **spuntata di default** (`checked` in
HTML). Letta in `launchSumo(variant)` e aggiunta al body della
`fetch('/api/sumo', ...)` come `background_traffic: <bool>`.

### 5.4 `scripts/sumo_common.py` — refactor (deciso, da fare in questo intervento)

Estrarre in un nuovo modulo condiviso le funzioni oggi duplicate quasi
identiche tra `sumo_visualize.py` e `compare_sumo.py`:

- `build_sumo_graph(net_path)` (parsing `net.xml` → grafo + posizioni
  giunzioni; le due versioni attuali differiscono solo in dettagli minori
  — es. `sumo_visualize.py` ritorna anche `eid_len`, `compare_sumo.py` no
  — da unificare mantenendo il superset di quello che serve a entrambi);
- `dijkstra(graph, start, goal)` (equivalente a `dijkstra_edges` di
  `compare_sumo.py`, stessa logica);
- `pddl_name_to_junction`/`resolve_junction` → un'unica funzione (stesso
  algoritmo a 3 tentativi: id esatto, suffisso, membro di cluster).

`sumo_visualize.py` e `compare_sumo.py` importano da qui invece di avere
ciascuno la propria copia. Nessun cambio di comportamento atteso — è un
refactor puro, da coprire con lo stesso test di non-regressione già
previsto in §8 (route/edge risultanti identici prima/dopo).

## 6. Stima di sforzo e rischi

- **Sforzo**: basso-medio. La parte più delicata è il campionamento
  locale (§3) e la sua integrazione nel blocco `.rou.xml`; il resto
  (Dijkstra, risoluzione giunzioni, sumocfg) è già scritto e va solo
  riusato/esteso.
- **Rischio principale**: prestazioni della GUI su `grande.net.xml`
  (rete da 17 MB) con troppi veicoli — da verificare empiricamente con i
  valori scelti per `k_scale`/`global_cap`/`max_hops` (§7) prima di
  considerarli definitivi.
- **Rischio secondario**: coppie O-D non instradabili o percorsi
  eccessivamente lunghi — già gestito con lo stesso pattern di scarto/
  conteggio usato in `compare_sumo.py` (`n_skipped_pairs`) e
  `generate_demand.py`.

## 7. Punti ancora aperti

1. **Valori esatti di `k_scale`, `global_cap`, `max_hops`,
   `hard_cap_per_edge`** (§5.1): il meccanismo — casuale ma proporzionato
   alla congestione (Poisson, non uniforme, per preservare l'ordine
   relativo tra archi — vedi §5.1), con cap globale e per-arco — è deciso
   nel principio, ma i numeri concreti vanno tarati empiricamente guardando
   la fluidità reale della GUI (specie su `grande.net.xml`, 17 MB) — non
   c'è modo di indovinarli a tavolino, vanno provati (§8).

## 7bis. Ulteriori dettagli emersi in fase di analisi (nessuna decisione richiesta, solo da tenere presenti in implementazione)

- **Nessuna nuova dipendenza**: niente `numpy`/`scipy` per il campionamento
  Poisson — implementato a mano (algoritmo di Knuth) su `random.Random`
  stdlib, coerente con l'uso di `random` già presente in tutto il progetto
  (`osm_graph.py`, `generate_demand.py`, ecc.). `requirements.txt` non
  cambia.
- **Direzionalità**: `vehicle-count` è per arco *direzionato* (il modello
  supporta strade a senso unico), quindi i veicoli di sfondo rispettano
  automaticamente il verso — nessuna gestione aggiuntiva necessaria.
- **Interazione con le chiusure stradali (replanning)**: gli archi bloccati
  dall'utente non finiscono mai in `problem_custom.pddl` (vengono esclusi
  a monte da `write_pddl` sia in `/api/solve` che in `/api/replan`), quindi
  il traffico di sfondo non può mai instradarsi su una strada chiusa —
  proprietà automatica, non richiede codice dedicato.
- **Interazione con `/api/replan`**: `problem_custom.pddl` viene
  riscritto ad ogni solve/replan; il traffico di sfondo letto da
  `sumo_visualize.py` riflette quindi sempre il corridoio **più recente**
  (l'intero percorso iniziale, oppure solo il tratto ricalcolato dopo un
  replan) — comportamento coerente con quello già esistente per il
  veicolo ENHSP stesso, nessuna differenza di trattamento.
- **Degrado controllato**: se il regex di `parse_congestion` non trova
  nulla (file malformato, o nessun dato di congestione), la funzione
  ritorna un dizionario vuoto e `generate_background_traffic` ritorna `[]`
  — lo script deve continuare a mostrare il solo veicolo ENHSP come fa
  oggi, senza errori.
- **Nessuna collisione di id**: nuovo `<vType id="traffic">` (distinto da
  `id="auto"` del veicolo ENHSP) e id veicolo `bg{i}` (distinti da
  `veicolo_enhsp`) — verificato, nessun conflitto.
- **Ottimizzazione**: il Dijkstra per un dato arco `(a,b)` viene calcolato
  una sola volta e riusato per tutti gli `n` veicoli assegnati a
  quell'arco (non ricalcolato `n` volte in modo identico) — dettaglio già
  incorporato nella bozza di codice in §5.1.
- **Seed**: `seed=42` di default, coerente con la convenzione già usata in
  tutto il progetto (`RANDOM_SEED = 42` in `osm_graph.py`,
  `generate_demand.py`, ecc.) — a parità di problema PDDL il traffico di
  sfondo generato è riproducibile.

## 8. Piano di verifica

- Caso piccolo (`dublin_piccola_centro.osm`, pochi nodi): pochi veicoli
  di sfondo, verifica visiva che appaiano vicino al percorso.
  - Caso grande (`dublin_grande_porto.osm`, "tutti i nodi"): tempo di
  generazione del `.rou.xml` + fluidità percepita della GUI con N veicoli.
- Verifica di non-regressione: il veicolo `veicolo_enhsp` (id, colore,
  rotta, `depart`) resta identico a oggi quando `--traffic 0` /
  `background_traffic: false`.

## 9. Riepilogo — checklist implementativa

Tutte le decisioni di design sono prese; resta da tarare solo il punto 1
di §7 (empiricamente, in fase di test). Da implementare, file per file:

1. `scripts/sumo_common.py` (nuovo) — refactor: `build_sumo_graph`,
   `dijkstra`, `pddl_name_to_junction`, estratti da `sumo_visualize.py` e
   `compare_sumo.py`, importati da entrambi al posto delle copie duplicate.
2. `scripts/sumo_visualize.py`:
   - `parse_congestion(text)` — regex sulle righe `vehicle-count` già nel
     PDDL letto da `compute_edges_from_pddl`;
   - `_poisson(rng, lam)` — campionamento Poisson senza dipendenze esterne;
   - `generate_background_traffic(...)` — genera le rotte di sfondo
     (Poisson per arco, Dijkstra cachato per arco, cap globale e
     per-arco);
   - blocco di scrittura `ROU_PATH` esteso: secondo `<vType id="traffic">`
     (`sigma` moderato, `maxSpeed` realistico, `color="1,0.8,0"` giallo/
     ambra) + un
     `<vehicle id="bg{i}">` per rotta di sfondo con `depart` scaglionato;
   - nuovo argomento CLI `--traffic` (default attivo, `--traffic 0`
     disattiva);
   - `cfg['end']` **non** viene toccato (resta ancorato solo al veicolo
     ENHSP, per decisione presa).
3. `webapp/app.py` — `/api/sumo` (`launch_sumo`): legge
   `background_traffic` dal body JSON (default `True`), passa
   `--traffic 0` solo se esplicitamente `false`.
4. `webapp/templates/index.html` — checkbox "Background traffic" (default
   spuntata) vicino ai bottoni "Open in SUMO"; JS `launchSumo()` aggiornato
   per includerla nel body della `fetch('/api/sumo', ...)`.
5. Verifica finale secondo il piano di §8 (non-regressione sul veicolo
   ENHSP + taratura empirica di `k_scale`/`global_cap`/`max_hops`/
   `hard_cap_per_edge` su un caso piccolo e uno grande).

## 10. Riepilogo implementazione (fatto)

Tutti i punti di §9 sono stati implementati con i valori di default proposti
in §5.1 (`k_scale=1.5`, `global_cap=40`, `max_hops=25`,
`hard_cap_per_edge=6`, seed `42`). Creato `scripts/sumo_common.py`
(`build_sumo_graph`, `dijkstra`, `pddl_name_to_junction`), importato da
`sumo_visualize.py` e `compare_sumo.py` al posto delle copie duplicate.
`sumo_visualize.py` ora legge la congestione dal PDDL, campiona il traffico
di sfondo con Poisson e lo scrive nel `.rou.xml` come `vType id="traffic"`
(giallo/ambra) distinto dal veicolo ENHSP, dietro flag `--traffic`
(webapp: checkbox default-on → `background_traffic` in `/api/sumo`).
Verifica: non-regressione byte-per-byte confermata con `--traffic 0`
(diff nullo tra output pre/post refactor su un percorso a 74 nodi);
generazione testata su caso piccolo (14 nodi, 24 veicoli di sfondo) e
grande (rete da 17 MB, 40 veicoli = cap globale, 0.6s di generazione);
`compare_sumo.py` rieseguito su `piccola` per confermare che il refactor
non altera le rotte/metriche esistenti; smoke test end-to-end della webapp
(`/api/generate` → `/api/solve` → `/api/sumo`) via Flask test client, con
verifica esplicita del flag `--traffic 0` passato solo quando
`background_traffic: false`. `k_scale`/`global_cap`/`max_hops`/
`hard_cap_per_edge` restano i valori di partenza: nessuna evidenza di
problemi di fluidità con questi numeri, ma non sono stati sottoposti a
tuning oltre la verifica funzionale sopra.

## 11. Modalità video + livello di congestione scelto dall'utente (fatto)

Problema: con pochi veicoli la simulazione dal vivo è fluida ma poco
rappresentativa di traffico reale; alzando il cap globale la GUI
interattiva (`sumo-gui` in tempo reale) diventa pesante da seguire.
Soluzione implementata: una modalità "video" che disaccoppia la cattura
dal rendering in tempo reale, usando il meccanismo nativo di `sumo-gui`
(elementi `<snapshot file=".." time=".."/>` nel gui-settings-file, letti
durante una run non interattiva `-S -Q`) invece di TraCI — nessuna nuova
dipendenza oltre `ffmpeg` (già presente via Homebrew). I fotogrammi sono
fissati a ~200 indipendentemente dalla durata simulata (l'intervallo tra
due snapshot si allarga per simulazioni più lunghe), così tempo di
generazione e lunghezza del video restano prevedibili; `ffmpeg` li
incapsula in un mp4 a 15 fps e i PNG intermedi vengono ripuliti.

`scripts/sumo_visualize.py`: nuovi flag `--video` / `--video-out <path>`
(quest'ultimo implica il primo); `--traffic` ora accetta anche un
moltiplicatore numerico (non solo `0`/off) che scala `k_scale`,
`global_cap` e `hard_cap_per_edge` di `generate_background_traffic` —
permette di scegliere il "grado di congestione" invece di solo on/off.
`webapp/app.py`: `/api/sumo` (invariato nel comportamento, ora accetta
`traffic_scale` invece del solo booleano `background_traffic`) resta
fire-and-forget per la GUI dal vivo; nuovo `/api/sumo_video` è invece
bloccante — genera il video sincronicamente e ritorna l'URL statico
(`webapp/static/videos/sumo_<variant>.mp4`, nome fisso sovrascritto ad
ogni richiesta, aggiunto a `.gitignore`). `webapp/templates/index.html`:
la checkbox on/off è stata sostituita da due `<select>` — livello di
traffico (Off/Low/Medium/High/Very high → moltiplicatori 0/0.5/1/2.5/5) e
modalità di output (GUI dal vivo / video registrato); in modalità video il
bottone chiama `/api/sumo_video` e il risultato viene mostrato con un
`<video controls>` inline.

Bug trovato e corretto durante il test: il gui-settings-file include da
sempre `<delay value="200"/>` per rendere gradevole la riproduzione dal
vivo, ma `sumo-gui` rispetta questo ritardo anche in modalità batch,
rallentando la cattura a ~5 step simulati/secondo reale (900s simulati →
180s reali) fino a superare il timeout con molti veicoli. Risolto
azzerando `delay` quando `VIDEO_MODE` è attivo (invariato a 200 in
modalità dal vivo).

Verifica: non-regressione confermata ricostruendo la versione del file
precedente a questa sessione e diffando l'output su un caso reale
(`piccola_custom`, stesso PDDL) — `.rou.xml` byte-per-byte identico con
argomenti di default; unica differenza nel gui-settings-file è una riga
vuota innocua. Livelli di congestione testati (0/0.5/1/2.5/5 → 0/20/40
(default, invariato)/100/200 veicoli). Generazione video testata con 200
veicoli di sfondo su `piccola_custom` (rete "media", 900s simulati): dopo
il fix del `delay`, 225/226 fotogrammi catturati in ~10.5s reali, video di
15s, presenza di veicoli verificata via analisi pixel su tutti i
fotogrammi decodificati. Endpoint `/api/sumo` e `/api/sumo_video` testati
via Flask test client (non mockati: sono stati eseguiti sumo-gui/ffmpeg
reali) per entrambe le varianti `optimized`/`baseline` e più valori di
`traffic_scale`, confermando file mp4 validi su disco. Non è stato
possibile un test in browser interattivo (nessun tool di automazione
browser disponibile in questa sessione): la pagina è stata verificata
via Flask test client (elementi HTML e riferimenti JS presenti e
coerenti) ma non cliccata a mano.

## 12. Bug di concorrenza, tab "Simulation" a schermo intero, tracking del veicolo (fatto)

Tre correzioni/estensioni alla modalità video del punto 11, richieste dopo
il primo utilizzo reale.

**a) Bug: non si poteva più richiedere una seconda simulazione.**
Causa trovata e riprodotta: `scripts/sumo_visualize.py` scriveva sempre
sugli stessi percorsi fissi per zona (`cfg_files/piccola_custom.sumocfg`,
`..._piano.rou.xml`, `gui_...xml`, la cartella dei fotogrammi e — lato
webapp — il file mp4 finale). Due richieste ravvicinate (dal vivo + video,
o due video) potevano sovrascrivere a vicenda gli input mentre l'altra era
ancora in corso. Riprodotto concretamente: due `POST /api/sumo_video`
concorrenti con `traffic_scale` diversi restituivano entrambi lo stesso
URL/timestamp del file finale, cioè uno dei due processi aveva vinto la
scrittura sull'altro. Nota: il sospetto iniziale di un server Flask di
sviluppo single-threaded (che serializzerebbe le richieste) è stato
escluso empiricamente — Flask 3.1/Werkzeug 3.1 gestiscono già le richieste
concorrenti; il problema era solo la condivisione dei file.
Fix: `sumo_visualize.py` accetta un nuovo flag `--run-id <id>`, incluso nel
suffisso di `zona` per l'intera durata della modalità dinamica — rende
univoci cfg/rou/gui-settings e la cartella dei fotogrammi per ogni
invocazione. `webapp/app.py` genera un `run_id` (`uuid.uuid4().hex[:8]`) a
ogni chiamata di `_sumo_cmd` e lo passa allo script; per `/api/sumo_video`
l'output di `ffmpeg` viene scritto su un percorso temporaneo univoco
(`.tmp_<variant>_<run_id>.mp4`) e pubblicato con `os.replace()` (rinomina
atomica) sul nome fisso servito dal frontend, così un client in lettura
non vede mai un file a metà scrittura. In `VIDEO_MODE`, a fine
generazione, i file di lavoro univoci (cfg/rou/gui) vengono rimossi (non
per la modalità dal vivo, dove non si può sapere quando `sumo-gui`,
lanciato con `Popen` fire-and-forget, ha finito di leggerli).
Verifica: due `/api/sumo_video` concorrenti sulla stessa variant → video
finale valido (verificato con `ffprobe`, durata coerente col numero di
fotogrammi, nessuna corruzione) e nessun file temporaneo residuo; due
concorrenti su variant diverse → entrambi completano con URL/mtime
indipendenti; `/api/sumo` (dal vivo) verificato generare cfg con suffisso
univoco (`piccola_custom_<run_id>.sumocfg`).

**b) Video troppo piccolo nella sidebar → tab dedicata "Simulation".**
`webapp/templates/index.html`: aggiunta una quinta tab nel pannello di
destra (`🎬 Simulation`, accanto a Map/PDDL/Plan/Congestion), disabilitata
(`.tab.disabled`, `pointer-events` bloccati via il controllo già presente
nel listener dei click) finché non esiste un video generato. Il piccolo
player inline nella sidebar (`sumo-video-box`) è stato rimosso: alla
generazione di un video, `showSimVideo()` abilita la tab, popola un
`<video>` a piena dimensione nel pannello destro e ci passa
automaticamente (`switchTab('simulation')`); la sidebar mostra solo un
messaggio breve ("✅ Video pronto — vedi la tab 🎬 Simulation"). La tab
torna disabilitata (`resetSimTab()`) quando si genera una nuova mappa o si
risolve un nuovo percorso, per non lasciare in giro un video ormai
riferito a un'altra route. Verificato via Flask test client (presenza ed
unicità di tutti gli id/funzioni coinvolti, sintassi JS validata con
`node --check`) e con una generazione video reale end-to-end (risposta
JSON con lo `url` atteso, file servito correttamente da `/static/videos/`).

**c) Tracking del veicolo + vista de-zoommata in modalità video.**
Il viewport del gui-settings è statico (un solo `x,y,zoom` per l'intera
run, confermato leggendo lo schema `viewsettings_file.xsd` installato — 
nessun modo nativo di far seguire un veicolo dalla telecamera senza
TraCI), e in modalità dinamica riusava lo zoom/centro del preset statico
della zona, spesso non centrato sul percorso reale (veicoli piccoli o
fuori scena). Soluzione: `_capture_frames_traci()` in
`scripts/sumo_visualize.py` guida `sumo-gui` via TraCI invece della
cattura nativa a schermate fisse — `traci.gui.trackVehicle()` fa seguire
la telecamera al veicolo ENHSP (`veicolo_enhsp`), `traci.gui.setZoom()`
imposta uno zoom più ampio (`VIDEO_ZOOM = 800`, calibrato a vista
confrontando 400/800/1500/2500/4000: 800 mostra alcuni isolati attorno
all'auto restando comunque leggibile). Si smette di catturare non a
`sim_end` ma appena il veicolo ENHSP arriva a destinazione (+`VIDEO_TAIL_S
= 15` secondi di margine): la simulazione da sola non si fermerebbe (il
traffico di sfondo continua), e fermarsi prima riduce anche il tempo di
esposizione della sessione TraCI. Nuova dipendenza: pacchetto pip
`traci` (`>=1.20.0`, aggiunto a `requirements.txt`), non serve avere
`SUMO_HOME`/`tools` sul path.
Affidabilità: nei primi test isolati, guidare `sumo-gui` via TraCI passo
per passo per l'intera durata (fino a 800 step) si è rivelato instabile
in questa sessione (`FatalTraCIError: Connection closed by SUMO` non
riproducibile in modo deterministico, tra step 113 e 559 a seconda della
run) — `sumo` headless, per confronto, completava sempre 800/800 step
senza problemi. L'utente ha fatto notare che parte dei crash erano dovuti
a lui stesso, che chiudeva manualmente le finestre `sumo-gui` viste
comparire una volta che l'auto arrivava (dato che nulla ferma la
simulazione da solo). Fermarsi subito dopo l'arrivo (+ margine) invece di
proseguire fino a `sim_end` elimina sia il problema (niente più finestre
"abbandonate" aperte a lungo) sia gran parte del tempo di esposizione: nei
test end-to-end successivi (dopo questa modifica) il tracking è riuscito
al primo tentativo in tutte le run, sia con traffico moderato (~40
veicoli, video di 6.5s in ~15s totali) sia pesante (200 veicoli, video di
6.7s in ~10-15s totali, 3/3 run ripetute).
Per robustezza resta comunque un ripiego automatico: se il tracking via
TraCI fallisce (fino a 2 tentativi), si passa alla cattura nativa a
schermate fisse del punto 11 (`_write_gui_settings(with_snapshots=True)`,
riscrive il gui-settings per includere gli elementi `<snapshot>` esclusi
di default quando il tracking è il percorso primario) — nessuna vista con
tracking, ma la generazione del video non fallisce mai del tutto. Testato
esplicitamente forzando un binario `sumo-gui` inesistente per il solo
tentativo TraCI: dopo 2 fallimenti il ripiego si attiva correttamente e
produce comunque un video valido (verificato con `ffprobe`).
Verifica visiva: fotogrammi estratti dal video con `ffmpeg` a diversi
istanti mostrano il veicolo rosso sempre centrato/visibile con più
isolati di contesto attorno, in posizioni diverse della rete tra un
fotogramma e l'altro (conferma che la telecamera segue realmente il
veicolo, non è una vista statica).
La modalità dal vivo (`sumo-gui` interattivo, senza `-Q`) resta invariata:
l'utente può già oggi fare tasto destro sul veicolo → "Track" per seguirlo
manualmente, come indicato dai messaggi stampati dallo script.

## 13. Traffico di sfondo: più denso, concentrato sul percorso dell'ego, tratte più lunghe (fatto)

Richiesta dell'utente: (a) troppe poche macchine di sfondo; (b) dato che
ormai la vista (dal vivo e video, vedi §12) segue/insegue il veicolo
ENHSP, concentrare il traffico attorno ad esso invece che sparso su tutta
la zona, per alleggerire la simulazione e renderla più realistica; (c)
migliorare il realismo delle auto di sfondo preferendo tratte più lunghe
che occupano gli stessi archi del veicolo ENHSP.

**Causa della scarsità/dispersione precedente.** `generate_background_traffic`
usava le coppie PDDL `(a, b) -> vehicle-count` scritte da
`webapp/pddl_writer.py`, a loro volta calcolate da
`osm_graph.compute_vehicle_counts`: solo `N_VEHICLES = 10` viaggi
O-D casuali instradati su un sottografo che copre l'INTERA zona
selezionata (fino a centinaia di nodi). Con soli 10 viaggi sintetici, la
maggior parte degli archi del sottografo restava a `vehicle-count = 0`
(nessun veicolo di sfondo generato lì) e quelli diversi da zero erano
sparsi ovunque nella zona — molti su strade mai inquadrate dalla
telecamera che segue l'ego (tracking via TraCI, §12), quindi veicoli
"sprecati": consumavano step di simulazione senza mai comparire nella
vista dal vivo o nel video.

**Nuovo design.** `generate_background_traffic` (in
`scripts/sumo_visualize.py`) non usa più le coppie di congestione del PDDL
né ricostruisce percorsi Dijkstra su un sottografo esterno: prende
direttamente `ego_edges`, la sequenza di edge SUMO realmente percorsi dal
veicolo ENHSP (`edges_str.split()`, già calcolata da
`compute_edges_from_pddl`/`route_to_sumo_edges`), e genera ogni veicolo di
sfondo come una **finestra contigua** di quella sequenza — lunghezza
scelta uniformemente tra il 45% e il 100% degli archi del percorso,
posizione di partenza scelta a caso lungo il percorso. Il numero di
veicoli scala con `TRAFFIC_SCALE` (`--traffic`, lo stesso selettore
"grado di traffico" già presente nella webapp: Off/Low/Medium/High/Very
high) e con la lunghezza del percorso (`base_vehicles=18` ogni 10 archi),
con un tetto assoluto `max_vehicles=120` per non ingolfare la GUI/il video
anche al livello più alto. `parse_congestion`, e i campi
`congestion`/`graph`/`junc_ids` restituiti da `compute_edges_from_pddl`,
sono stati rimossi perché non più usati da nessun altro punto dello
script (il dato di congestione PDDL resta comunque usato altrove,
nella pianificazione ENHSP e nel pannello "Congestion" della webapp:
qui si è tolto solo il suo uso come sorgente per il traffico di sfondo
SUMO).

Concentrare le rotte di sfondo sugli stessi archi dell'ego risolve (b) e
(c) con lo stesso meccanismo: i veicoli condividono letteralmente la
strada con l'auto rossa (non solo le vicinanze), per tratti lunghi invece
che un singolo arco isolato — visivamente indistinguibile da traffico
reale che si muove nello stesso corridoio.

**Verifica.** Con `pddl_files/problem_custom.pddl` (percorso di 44 archi
SUMO, ~2146 m, `sim_end` stimato 1000s): a `--traffic 1` (default,
"Medium") generati 56 veicoli di sfondo; a `--traffic 5` ("Very high")
generati 120 (tetto raggiunto). Entrambe le run in modalità `--video`
hanno completato il tracking TraCI al primo tentativo (nessun ripiego),
con l'ego arrivato a destinazione ben prima di `sim_end` (video di
~9.9s, coerente con un arrivo intorno ai 730s simulati, nessun segno di
gridlock nonostante fino a 120 veicoli concentrati sullo stesso
corridoio). Fotogrammi estratti con `ffmpeg` a istanti diversi mostrano
una fila visibile di veicoli gialli di sfondo lungo la stessa strada
del veicolo rosso tracciato, in entrambi i test.

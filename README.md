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

## Iniezione del piano semaforico in SUMO (`inject_signal_plan.py`)

L'ottimizzazione semaforica (`signal_optimization/optimize.py`) produce
`sumo_extracted/signal_plan_<zona>.json` nel formato
`{tlLogic_id: {phase_idx: durata_s}}`. Lo script **`inject_signal_plan.py`**
traduce questo piano in un *additional-file* SUMO, chiudendo il ciclo
PDDL+ → SUMO:

```
net_files/<zona>.net.xml                 (tlLogic originali: stati, offset, type)
sumo_extracted/signal_plan_<zona>.json   (durate ottimizzate)
                 |
                 v
cfg_files/tls_<zona>.add.xml             (<additional> con i tlLogic ottimizzati)
```

Lo script conserva dal `net.xml` le stringhe di stato (`GGrrrr`…), l'`offset`
e il `type` di ogni semaforo, e sovrascrive **solo** le durate delle fasi
presenti nel piano: le fasi non ottimizzate mantengono la durata originale.

### Perché il programma diventa attivo

Dalla documentazione SUMO (*Simulation/Traffic Lights → Defining New
TLS-Programs*): *"You can load new definitions for traffic lights as a part of
an additional-file. **When loaded, the last program will be used**"*. Non
servono quindi né WAUT né TraCI. I due vincoli imposti dalla doc sono
rispettati dallo script:

- l'`id` del `tlLogic` è un semaforo già esistente nel `.net.xml`;
- il `programID` è **nuovo** (`optimized`), diverso da quello originale `0`
  (`off` è riservato).

Siccome il programma originale `0` resta comunque caricato, in sumo-gui si può
passare da un programma all'altro col **tasto destro sul semaforo → Switch TLS
program**, confrontando a occhio baseline e ottimizzato nella stessa
simulazione.

### Uso

```bash
# 1. genera il piano ottimizzato (punto 2) — calcolo pesante, alcuni minuti
python -m signal_optimization.optimize piccola media grande

# 2. traduci il piano in additional-file SUMO (punto 3)
python inject_signal_plan.py                 # tutte le zone disponibili
python inject_signal_plan.py piccola         # una sola zona

# 3. visualizza: i semafori ottimizzati sono caricati automaticamente
python sumo_visualize.py piccola
python sumo_visualize.py piccola --baseline  # forza i semafori originali
```

`sumo_visualize.py` aggiunge da solo la riga
`<additional-files value=".../tls_<zona>.add.xml"/>` al `.sumocfg` **se il file
esiste**; se manca, la simulazione parte come prima con i semafori del
`net.xml`. La zona viene ricavata dalla rete realmente usata, non da quella
passata da riga di comando: la webapp avvia SUMO attraverso lo stesso script
passando sempre `piccola`, ma se il problema custom appartiene a un'altra zona
vengono caricati correttamente la rete e i semafori di quella zona.

### Mapping dei nodi PDDL sulle junction SUMO

I nomi dei nodi PDDL (`n9100868`) vanno ricondotti agli id delle *junction*
SUMO. La corrispondenza non è sempre diretta, perché `netconvert` semplifica la
rete in modo diverso da come `build_problems.py` costruisce il grafo contratto.
`pddl_name_to_junction` prova quindi, in ordine: id esatto → junction il cui id
termina con quel suffisso (i nomi PDDL conservano solo le ultime 7 cifre) →
junction **cluster** che contiene quell'id fra i propri membri (netconvert
fonde nodi vicini in `cluster_<id1>_<id2>_...`).

Se nemmeno così start o goal risultano mappabili, la rete viene scelta come
quella che mappa **più nodi** del problema, e come start/goal si usano il primo
e l'ultimo nodo mappabile del piano ENHSP. Senza questo ripiego la
visualizzazione falliva del tutto per i problemi custom delle zone media e
grande.

Verifica automatica sui file generati, **nessun errore su nessuna zona**:
stati di fase identici all'originale, numero di fasi invariato, `programID`
sempre nuovo, nessuna durata nulla o negativa, e ciclo che passa da 90 s
(default netconvert) a 120 s (valore realistico SCATS usato in tutto il
progetto) su tutti i 577 semafori delle tre reti.

| Zona | `tlLogic` scritti | Fasi modificate | `total-time` PDDL+ baseline → ottimizzato |
|---------|------------------:|----------------:|-------------------------------------------|
| piccola | 27                | 96              | 16.45 s → 10.33 s (**-37.2%**)            |
| media   | 97                | 399             | 72.04 s → 47.07 s (**-34.7%**)            |
| grande  | 453               | 1745            | 97.80 s → 67.94 s (**-30.5%**)            |

---

## Confronto in simulazione: baseline vs ottimizzato (`compare_sumo.py`)

### A cosa serve

L'ottimizzazione semaforica calcola le nuove durate di verde con una **formula
matematica** (il ritardo uniforme di Webster) applicata dentro PDDL+. Quella
formula *prevede* un guadagno, ma si basa su ipotesi semplificate: tratta ogni
incrocio come **isolato**, con arrivi casuali, senza code che si accumulano e
senza interferenza fra semafori vicini.

`compare_sumo.py` serve a **verificare quella previsione sul campo**: mette
davvero ~45 veicoli in circolazione dentro il simulatore e misura quanto tempo
impiegano, prima e dopo. È la differenza fra *affermare* che l'ottimizzazione
funziona e *dimostrarlo* con una misura indipendente dal modello usato per
ottimizzare.

Il controllo non è formale: ha effettivamente smentito la previsione su una
delle tre zone (vedi `grande` più sotto), dove il piano stimato migliore del
30% risulta invece peggiore del 10%. Senza questa verifica il progetto avrebbe
riportato un guadagno che nella realtà simulata non si verifica.

### Come funziona

SUMO riproduce il traffico microscopicamente — code, accelerazioni, frenate e
fasi reali verde/giallo/rosso — quindi cattura proprio gli effetti che la
formula analitica ignora.

### Disegno dell'esperimento

Per ogni zona vengono eseguite due simulazioni identiche, diverse **solo** nel
programma semaforico (baseline = programma `0` del `net.xml`; ottimizzato =
programma `optimized` da `tls_<zona>.add.xml`). Per rendere il confronto
pulito:

- **stessa domanda**: le coppie O-D di `sumo_extracted/demand_<zona>.json`, lo
  stesso campione usato dal punto 2;
- **stesse rotte**: gli itinerari sono calcolati una volta con Dijkstra e
  riusati identici nei due run. Se si lasciasse ricalcolare il percorso a SUMO,
  i veicoli potrebbero scegliere strade diverse e il confronto misurerebbe due
  effetti insieme;
- **stesso seed e stessi istanti di partenza**;
- **teletrasporti disattivati** (`--time-to-teleport -1`): un veicolo bloccato
  resta in coda invece di sparire, altrimenti le attese risulterebbero
  artificialmente più basse.

### Risultati misurati

| Zona | Veicoli | Tempo di viaggio | Attesa ai semafori | Tempo perso |
|---------|--------:|------------------------|------------------------|------------------------|
| piccola | 46 | 32.7 → 30.6 s (**-6.6%**) | 3.2 → 1.0 s (**-68.5%**) | 6.0 → 3.8 s (**-36.5%**) |
| media | 45 | 139.7 → 127.8 s (**-8.5%**) | 44.5 → 34.2 s (**-23.1%**) | 53.4 → 41.5 s (**-22.3%**) |
| grande | 43 | 270.7 → 277.3 s (**+2.4%**) | 99.1 → 109.1 s (**+10.1%**) | 117.8 → 124.4 s (**+5.6%**) |

Su **piccola** e **media** il guadagno previsto è confermato. Su **grande**
l'ottimizzazione **peggiora** le prestazioni, pur essendo prevista in
miglioramento dalla stima analitica: il ritardo di Webster modella ogni
incrocio come *isolato*, ipotesi che cade in una rete densa dove le code si
propagano fra incroci adiacenti, gli offset non vengono ricalibrati e solo una
minoranza di semafori viene ottimizzata. A conferma, ripetendo il confronto su
`grande` con traffico più leggero (11 veicoli invece di 43) il segno si
inverte (-1.4% di attesa): il degrado emerge **sotto congestione**, cioè dove
le ipotesi di Webster sono meno valide.

### Uso

```bash
python compare_sumo.py                        # tutte le zone
python compare_sumo.py piccola media grande
python compare_sumo.py grande --max-vehicles 15
```

I risultati sono **cumulativi**: eseguire una zona alla volta non cancella
quelle già calcolate. Output in `sumo_comparison/` (`results.json` + `report.md`,
con una sezione di interpretazione generata automaticamente).

Il confronto è disponibile anche dalla webapp, insieme ai due pulsanti per
aprire la simulazione con i semafori ottimizzati o con quelli originali.

---

## Dominio PDDL+

Il dominio definisce un'azione discreta, un processo continuo e un evento automatico. Rispetto alla versione base, `start-move` conosce il **nodo di provenienza** (`?prev`) per poter addebitare il tempo di svolta e il ritardo semaforico, e un unico fatto `(prev ...)` tiene traccia dell'ultimo arco percorso (cancellato in `start-move`, ristabilito da `arrive`):

```pddl
(:action start-move
  :parameters (?prev ?from ?to - location)
  :precondition (and (at ?from) (road ?from ?to) (prev ?prev))
  :effect (and
    (not (at ?from)) (not (prev ?prev)) (moving ?from ?to)
    (assign (progress ?from ?to) 0)
    (increase (total-time) (turn-time ?prev ?from ?to))       ; costo di svolta
    (increase (total-time) (signal-delay ?prev ?from ?to))))  ; ritardo semaforico del MOVIMENTO

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
    (increase (total-time) (congestion-delay ?to))        ; ritardo congestione
    (assign (progress ?from ?to) 0)))
```

Il processo `driving` fa avanzare `progress` in modo continuo tramite la variabile temporale `#t`; l'evento `arrive` si attiva automaticamente quando `progress >= distance` e aggiorna `total-time` in modo discreto, sommando il tempo di guida e il ritardo di congestione del nodo di arrivo; l'azione `start-move` aggiunge il tempo di svolta **e** il ritardo semaforico del movimento `(?prev, ?from, ?to)` che si sta per impegnare — addebitato alla partenza da `?from`, non all'arrivo a `?to`, perché è il momento in cui il veicolo attende davvero il verde di quello specifico movimento (vedi sezione "Semafori e tempi di svolta" e `2_traffic_signal_optimization.md`, sez. 1, per il perché del passaggio da un valore medio per nodo a uno specifico per movimento). L'aggiornamento di `total-time` avviene esclusivamente negli eventi/azioni discreti, e non nel processo continuo, per evitare che ENHSP tratti il tempo accumulato come variabile continua da campionare ad ogni istante, con conseguente aumento del costo computazionale della ricerca.

> **Nota sulla scomposizione del tempo.** La *timeline* stampata da ENHSP (gli istanti delle azioni) riflette il solo tempo di **guida** simulato dal processo continuo; i ritardi (semaforo, congestione, svolta) sono incrementi *discreti* su `total-time` e non fanno avanzare l'orologio simulato. Il costo effettivo del piano è quindi la somma: **total-time = guida + semafori + congestione + svolte**.

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

### Replanning: strade chiuse e ricalcolo del percorso

L'interfaccia permette di marcare strade e incroci come **non percorribili** e
di far ricalcolare il percorso. Il replanning non riparte dall'origine: simula
un veicolo **già in viaggio** che trova la strada chiusa, quindi ripianifica
dal **nodo immediatamente precedente** alla chiusura, cioè l'ultimo punto
raggiungibile.

**Come si usa**

1. Risolvi normalmente il problema (compare la sezione *Strade chiuse & ricalcolo*).
2. Premi **Attiva modalità chiusura**: il cursore diventa un mirino.
3. Clicca una **strada** per chiuderla o un **incrocio** per bloccarlo
   (ri-clicca per riaprire). Le chiusure appaiono in rosso tratteggiato con un
   cartello di lavori in corso.
4. Premi **Ricalcola percorso**.

Sulla mappa vengono poi disegnati contemporaneamente: il piano originale
sbiadito, il tratto **già percorso** in verde, il punto di ricalcolo con
un'icona pulsante e la **deviazione** in ambra con tratteggio animato che scorre
verso il goal. Un pannello riepiloga distanza, tempo e semafori prima e dopo,
con il costo della deviazione.

**Dettagli di modellazione**

- Una strada chiusa lo è in **entrambi i sensi** (chiusura reale, non senso unico).
- Il nodo di provenienza viene passato ad ENHSP come `prev`, così la prima
  svolta dopo il ricalcolo ha il costo reale: ripartendo da metà percorso il
  veicolo ha già un orientamento.
- Chiudere un incrocio equivale a chiudere tutte le strade che vi confluiscono.
- Se le chiusure non toccano il percorso attuale, il sistema lo rileva e non
  ricalcola nulla.

**Quali strade si possono chiudere: l'interfaccia lo dice prima del click**

Il sottografo estratto è spesso quasi un *albero*: molte strade sono l'unico
collegamento verso la destinazione, quindi chiuderle la isola e il ricalcolo
fallisce legittimamente. Per non lasciare l'utente a tentativi, appena si
attiva la modalità chiusura il percorso viene colorato in base a una BFS
calcolata **nel browser**:

- **giallo** — chiudendo quel tratto esiste una deviazione;
- **rosso scuro punteggiato** — è l'unico collegamento, chiuderlo isolerebbe il goal.

Cliccando un tratto critico l'avviso compare subito, senza attendere il server.

**Quanti nodi servono**

Più nodi = più alternative disponibili, ma anche ENHSP più lento. Tratti
chiudibili sul percorso, misurati:

| Nodi | Zona media | Zona grande |
|-----:|------------|-------------|
| 120  | 5 su 9     | 1 su 15     |
| 200  | —          | —           |
| 300  | 14 su 45   | 7 su 32     |
| 400  | —          | 11 su 29    |

Lo slider arriva a 1000 e parte da 200. Il grafo contratto completo ha 938
nodi (media) e 3756 (grande), quindi *tutti i nodi* copre per intero la zona
media.

**Il limite dipende dalla macchina.** `start-move` e `signal-delay` hanno tre
argomenti (`prev`, `from`, `to`), quindi il numero di istanze generate da
ENHSP cresce col cubo dei nodi e il consumo di memoria sale in fretta. Su un
ambiente molto modesto (3 GB di RAM, 1 core) con heap da 2 GB: 300 nodi in 8 s,
400 in 14 s, 939 in `OutOfMemory`. Su un PC normale il tetto è molto più alto,
quindi i default sono heap **6 GB** e limite **1200 nodi**, entrambi
regolabili:

```bash
set ENHSP_HEAP=8g            # heap della JVM per ENHSP
set MAX_SOLVABLE_NODES=2000  # tetto di sicurezza sui nodi
set ENHSP_TIMEOUT=300        # secondi; 0 o assente = nessun limite (default)
```

ENHSP gira **senza timeout** per impostazione predefinita: sui grafi grandi il
grounding può richiedere parecchi minuti e interrompere la ricerca a metà
sprecava lavoro già fatto. Il rovescio della medaglia è che una richiesta molto
pesante resta in attesa senza possibilità di annullarla dal browser: se serve
un tetto, si imposta `ENHSP_TIMEOUT`.

Se ENHSP esaurisce la memoria l'interfaccia lo dice esplicitamente (non più un
generico “problema irrisolvibile”) e suggerisce l'azione corretta.

> **La zona `piccola` non è adatta al replanning.** Il centro storico
> (Temple Bar) è fatto di vie strette a senso unico e resta quasi ad albero
> anche usando tutti i suoi 201 nodi: solo **1 tratto su 44** risulta
> chiudibile. Per provare il replanning usare la zona **media**.

### I controlli SUMO nell'interfaccia

Dopo che ENHSP ha trovato la soluzione compaiono tre controlli, che fanno cose
diverse fra loro:

| Controllo | Cosa fa | Cosa ottieni |
|---|---|---|
| **▶ Apri in SUMO (semafori ottimizzati)** | apre sumo-gui con **il tuo percorso** e i semafori ottimizzati | una verifica **visiva**: guardi la tua auto percorrere la strada |
| **▶ Apri in SUMO (semafori originali)** | idem, ma con i semafori di default del `net.xml` | il confronto a occhio: aprendo prima uno e poi l'altro si vede se l'auto si ferma di più o di meno ai rossi |
| **Confronto in SUMO** (selettore zona + *Confronta*) | simula **l'intera zona** con ~45 veicoli, due volte (originali e ottimizzati) | una tabella con i **numeri**: tempo di viaggio, attesa ai semafori e tempo perso, con la variazione percentuale |

La distinzione importante: i due pulsanti mostrano **una singola corsa** (la
tua), mentre il pannello di confronto misura la qualità del **piano semaforico
della zona** su una flotta di veicoli — per questo ha un selettore di zona e
non usa il percorso che hai disegnato. Il calcolo richiede qualche secondo,
perché esegue due simulazioni complete.

Nella tabella il verde indica un miglioramento e il rosso un peggioramento:
sulla zona `grande` il risultato è **rosso**, e non è un errore — vedi la
spiegazione nella sezione sul confronto in simulazione.

---

## Struttura del repository

```
├── README.md
├── requirements.txt
├── setup.bat
├── build_problems.py          # Genera i file problem_*.pddl da OSM
├── extract_sumo_data.py       # Estrae semafori/settaggi/turn dai net.xml SUMO
├── inject_signal_plan.py      # Punto 3: inietta il piano ottimizzato in SUMO
├── compare_sumo.py            # Punto 4: confronto in simulazione baseline vs ottimizzato
├── download_dublin_map.py     # Scarica le mappe OSM tramite osmnx
├── convert_to_osm.py          # Converte OSM in net.xml tramite netconvert
├── sumo_visualize.py          # Visualizza il piano in sumo-gui
├── compare_versions.py        # Confronto vecchia/nuova versione del dominio
├── generate_demand.py         # Campione O-D condiviso fra i punti 2 e 4
│
├── docs/                      # Materiale di documentazione e presentazione
│   ├── spiegazione_tecnica.pdf
│   ├── spiegazione_congestione_sumo.pdf
│   ├── screenshots/               # Schermate della webapp
│   └── slide/                     # Slide di presentazione
│
├── signal_optimization/       # Punto 2: ottimizzazione semaforica
│   ├── optimize.py            # Orchestratore della pipeline
│   ├── candidates.py          # Generazione candidati vincolati
│   ├── webster_screen.py      # Screening analitico
│   ├── enhsp_eval.py          # Valutazione con ENHSP
│   ├── search.py              # Ricerca locale
│   └── progression.py         # Penalita' di progressione
│
├── comparison_results/        # Output di compare_versions.py
├── diagnostics_out/           # Report diagnostici
│
├── sumo_comparison/           # Punto 4: risultati del confronto in simulazione
│   ├── results.json
│   └── report.md
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
│   ├── tls_{zona}.add.xml     # Semafori ottimizzati (punto 3)
│   └── gui_{zona}.xml
│
├── pddl_files/                # File PDDL+ del progetto
│   ├── domain.pddl
│   ├── problem_piccola.pddl
│   ├── problem_media.pddl
│   ├── problem_grande.pddl
│   ├── problem_custom.pddl    # Generato dalla webapp (aggiornato automaticamente)
│   ├── output_piccola.txt
│   ├── output_media.txt
│   ├── output_grande.txt
│   └── run.py
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
pip install -r requirements.txt
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

# Map Construction in PDDL+

Progetto #2 — Automated Planning
Università della Calabria (UNICAL)
Gruppo: Emanuele Colecchia, Chiara Costantino, Elisa Gigliotti, Pierluigi Trocini

---

## Descrizione del progetto

L'idea del progetto è costruire in modo automatico dei problemi di navigazione in PDDL+ partendo da dati stradali veri. Abbiamo scelto Dublino, e i dati delle strade li scarichiamo da OpenStreetMap.

Nel progetto usiamo tre strumenti che lavorano insieme. ENHSP è il planner numerico per PDDL+, ed è quello che trova il percorso migliore. OSMnx è una libreria Python che scarica e sistema i grafi stradali di OSM. SUMO è un simulatore di traffico, e lo usiamo per far vedere il piano in movimento.

Il dominio descrive un veicolo che parte da un nodo e deve arrivare a un altro nodo spendendo meno tempo possibile. Il tempo tiene conto della guida su ogni tratto di strada, del ritardo ai semafori (preso dalla rete SUMO, non inventato) e del tempo che serve per svoltare agli incroci (che dipende dall'angolo della curva e da quanto velocemente l'auto riesce a girare).

---

## Istanze del problema

Abbiamo costruito tre istanze di dimensione crescente, una per ogni zona di Dublino.

| Zona     | Area geografica              | Nodi PDDL | Archi | Start → Goal                         | Distanza | Tempo piano |
|----------|------------------------------|-----------|-------|--------------------------------------|----------|-------------|
| Piccola  | Temple Bar / Centro storico  | 14        | 20    | Liffey St. → Aungier St.            | 1.57 km  | ~314 s      |
| Media    | Ranelagh / Residenziale      | 50        | 93    | Leeson St. → Saint Mary's Rd.       | 1.62 km  | 160 s       |
| Grande   | Phibsborough / Nord          | 120       | 206   | St. Patrick's Rd. → Botanic Ave.    | 1.33 km  | 147 s       |

---

## Architettura del sistema

### Fase 1 — Download della mappa (`download_dublin_map.py`)

Lo script usa osmnx per scaricare le strade vere di Dublino da OpenStreetMap. Prende le coordinate GPS degli incroci, come sono collegati fra loro, i limiti di velocità e i sensi unici. Tutto viene salvato in file `.osm` dentro `osm_files/`.

### Fase 2 — Costruzione del problema PDDL+ (`build_problems.py`)

Questa è la parte principale del progetto. Da un file `.osm` grezzo si arriva a un problema PDDL+ completo, in quattro passaggi.

A. Lettura del file OSM. Il file è un XML, quindi lo leggiamo come tale. Da lì tiriamo fuori i nodi (gli incroci, con le loro coordinate GPS) e le strade dove possono passare le auto, buttando via piste ciclabili e percorsi pedonali. I nodi che hanno il tag `highway=traffic_signals` li segniamo come semafori.

B. Costruzione del grafo contratto. Molti nodi OSM servono solo a disegnare la curva di una strada e non contano per le decisioni, quindi li togliamo. Teniamo solo gli incroci veri, cioè i nodi che compaiono in due o più strade oppure che stanno all'inizio o alla fine di una strada. Nel grafo che ne esce ogni arco collega due incroci vicini, e la lunghezza dell'arco la calcoliamo con la formula di Haversine sulle coordinate GPS.

C. Selezione del sottografo. Se dessimo a ENHSP tutta la città non finirebbe più, quindi scegliamo un sottoinsieme di N nodi (il numero è regolabile). Partiamo dal nodo con più strade uscenti e a ogni passo aggiungiamo il nodo di frontiera più lontano dal centro di quelli già presi. Così i nodi restano sparsi bene sull'area invece di ammucchiarsi in un punto.

D. Scrittura del file PDDL+. Per ogni arco del sottografo scriviamo i predicati e le funzioni che servono: `(road A B)`, `(distance A B)`, `(speed A B)`, `(progress A B)`, `(signal-delay A)`, più le funzioni `(turn-time P A B)` per ogni tripla di nodi consecutivi (ne parliamo nella sezione sui semafori e le svolte). Il ritardo ai semafori non è più fisso a 30 secondi, ma viene letto dai dati estratti da SUMO (`sumo_extracted/sumo_data_{zona}.json`); i 30 secondi restano solo come valore di riserva per i semafori che non riusciamo a mappare. La velocità la prendiamo dal tag `maxspeed` di OSM (se manca usiamo 30 km/h) e la convertiamo in m/s.

### Fase 3 — Risoluzione con ENHSP (`pddl_files/run.py`)

Diamo a ENHSP il dominio e il problema scelto. Usiamo la configurazione `-s aibr` (Admissible Interval-Based Relaxation), che va bene per i domini PDDL+ con processi ed eventi. La cosa che ENHSP minimizza è `(total-time)`, cioè la somma del tempo di guida e dei ritardi ai semafori.

### Fase 4 — Traduzione del piano per SUMO (`sumo_visualize.py`)

ENHSP e SUMO chiamano le cose in modo diverso. Il piano di ENHSP parla di nodi (per esempio `n9100868`), mentre SUMO ragiona per archi stradali con id numerici (per esempio `4396046#0`). Quindi bisogna tradurre la sequenza di nodi del piano nella sequenza di archi SUMO corrispondente.

Il punto delicato è che netconvert (che crea la rete SUMO) semplifica la mappa in modo diverso da come la semplifica il nostro grafo contratto, perciò non tutti i nodi del piano esistono come incrocio in SUMO. Per gestire questo caso la webapp, quando risolve, salva accanto al problema un file `route_custom.json` con tre cose: la sequenza di nodi del piano, le coordinate GPS di ogni nodo e la forma reale (la lista di punti lat/lon) di ogni tratto.

Con queste informazioni la traduzione funziona così. I nodi del piano che corrispondono davvero a un incrocio SUMO diventano punti fissi, e l'auto ci deve passare. Fra due punti fissi consecutivi, se esiste un arco diretto lo usiamo perché è la strada scelta dal planner. Se invece in mezzo c'è un "buco" (nodi che SUMO ha semplificato) lo riempiamo con un Dijkstra che, fra i percorsi possibili, preferisce quello la cui forma segue la strada vera salvata dalla webapp. In questo modo, quando ci sono due strade parallele, prendiamo quella giusta e non una scorciatoia diversa. Partenza e arrivo, se non sono incroci SUMO, li agganciamo all'incrocio più vicino per posizione. Il risultato è una lista ordinata di archi collegati che passiamo a SUMO.

Abbiamo verificato la cosa sulle tre mappe confrontando punto per punto la rotta SUMO con il percorso disegnato sulla webapp: la distanza media è di 2-3 metri (più o meno la larghezza di una corsia), senza giri a vuoto.

### Fase 5 — Visualizzazione in SUMO

Lo script prepara i file di configurazione per SUMO (`{zona}.sumocfg`, `{zona}_piano.rou.xml`, `gui_{zona}.xml`) e apre sumo-gui con il percorso già caricato. SUMO poi applica la sua fisica del traffico (accelerazioni, frenate, semafori che cambiano) per conto suo, senza toccare il piano PDDL+. I semafori vengono simulati con le loro fasi vere di verde, giallo e rosso, così la simulazione è più realistica del modello semplificato del dominio.

---

## Semafori e tempi di svolta (dati estratti da SUMO)

Lo script `extract_sumo_data.py` legge le reti SUMO (`net_files/*.net.xml`) e per ogni zona produce il file `sumo_extracted/sumo_data_{zona}.json`, più un `report.md` di riepilogo, con i dati che poi usiamo nei problemi PDDL+. L'estrazione avviene leggendo direttamente l'XML del `net.xml`, senza far girare SUMO.

### Da dove vengono i tempi dei semafori

Ogni incrocio semaforizzato in SUMO è un elemento `<tlLogic>` con le sue fasi.

```xml
<tlLogic id="12639663" type="static" programID="0" offset="0">
    <phase duration="42" state="GGrrrr"/>   <!-- verde direzione 1 -->
    <phase duration="3"  state="yyrrrr"/>   <!-- giallo -->
    <phase duration="42" state="rrGGGG"/>   <!-- verde direzione 2 -->
    <phase duration="3"  state="rryyyy"/>   <!-- giallo -->
</tlLogic>
```

Questi tempi però non sono quelli veri di Dublino. Sono i valori di default che genera netconvert, con un ciclo fisso di 90 secondi, il verde diviso in parti uguali fra le direzioni e il giallo calcolato dalla velocità della strada (lo dice la documentazione di SUMO). A Dublino invece gli incroci sono gestiti dal sistema adattivo SCATS, con cicli che arrivano fino a circa 120 secondi (nel Regno Unito e in Irlanda il ciclo massimo è 120 s, e scende a 90 s dove ci sono attraversamenti pedonali).

Per avere un ritardo realistico facciamo tre cose. Prima prendiamo dal net.xml la struttura dell'incrocio, cioè quali movimenti sono verdi insieme e la proporzione di verde e rosso di ognuno; questa informazione conta perché dipende dalla forma vera dell'incrocio (numero di bracci, corsie, precedenze). Poi riscaliamo il ciclo dai 90 secondi di SUMO ai 120 secondi realistici di Dublino (`REAL_CYCLE_S = 120 s`), tenendo lo stesso rapporto verde/rosso. Infine calcoliamo il ritardo medio con la formula di Webster, che per un semaforo a tempo fisso con arrivi casuali dà

> d = rosso² / (2 · ciclo)

dove *rosso* è il tempo di rosso del movimento sul ciclo realistico. Il `signal-delay` del nodo è la media di *d* sui suoi movimenti.

Un esempio su un incrocio a 2 fasi. Se il verde è circa il 47% del ciclo, il rosso reale è circa 64 secondi su 120, quindi d ≈ 64² / (2·120) ≈ 17 secondi. Da qui viene il valore intorno ai 17 secondi che ha sostituito i 30 secondi fissi di prima.

Gli id delle junction SUMO coincidono con gli id dei nodi OSM, quindi il JSON contiene una mappa `node_signal_delay` (id-nodo → ritardo, con i cluster già espansi) che `build_problems.py` e la webapp possono riusare direttamente.

### Da dove viene il tempo di svolta (turn time)

Con turn rate intendiamo quanto velocemente un veicolo cambia direzione, misurato in gradi al secondo. Usiamo `TURN_RATE_DPS = 20 °/s`, che è in linea con lo yaw rate reale di un'auto in una svolta urbana stretta (intorno ai 15-20 °/s; sopra i 30 °/s circa interviene il controllo di stabilità ESC).

Il tempo di svolta a un incrocio dipende da quanto deve girare il veicolo.

> turn-time = |angolo di svolta| / turn-rate

L'angolo di svolta è la differenza fra la direzione dell'arco con cui si arriva all'incrocio e quella dell'arco con cui si esce. In `extract_sumo_data.py` lo calcoliamo dalla geometria delle corsie del net.xml (l'orientamento dell'ultimo pezzo della corsia in entrata contro il primo pezzo di quella in uscita); questa direzione coincide con l'attributo `dir` di SUMO nel 94-97% dei casi, e ci è servito per validare l'algoritmo. In `build_problems.py` e nella webapp invece lo calcoliamo dalle coordinate GPS dei nodi (direzione *prev→from* contro *from→to*), per restare coerenti con il grafo contratto usato nel PDDL.

Qualche esempio. Una svolta a 90° dà 90/20 = 4,5 s, un'inversione a U (180°) dà 9 s, andare dritto (circa 0°) costa circa 0 s.

---

## Iniezione del piano semaforico in SUMO (`inject_signal_plan.py`)

L'ottimizzazione dei semafori (`signal_optimization/optimize.py`) produce il file
`sumo_extracted/signal_plan_<zona>.json`, nel formato
`{tlLogic_id: {phase_idx: durata_s}}`. Lo script `inject_signal_plan.py`
prende questo piano e lo trasforma in un additional-file di SUMO, chiudendo
il giro da PDDL+ a SUMO.

```
net_files/<zona>.net.xml                 (tlLogic originali: stati, offset, type)
sumo_extracted/signal_plan_<zona>.json   (durate ottimizzate)
                 |
                 v
cfg_files/tls_<zona>.add.xml             (<additional> con i tlLogic ottimizzati)
```

Dal net.xml lo script conserva le stringhe di stato (`GGrrrr`…), l'offset e il
type di ogni semaforo, e cambia solo le durate delle fasi che compaiono nel
piano. Le fasi non ottimizzate restano com'erano.

### Perché il programma diventa attivo

La documentazione di SUMO (*Simulation/Traffic Lights → Defining New
TLS-Programs*) dice: *"You can load new definitions for traffic lights as a part
of an additional-file. When loaded, the last program will be used"*. Quindi non
servono né WAUT né TraCI. I due vincoli che pone la documentazione sono
rispettati dallo script. L'id del `tlLogic` è un semaforo che esiste già nel
`.net.xml`, e il `programID` è nuovo (`optimized`), diverso da quello originale
`0` (`off` è riservato).

Siccome il programma originale `0` resta comunque caricato, in sumo-gui si può
passare dall'uno all'altro con tasto destro sul semaforo → Switch TLS program, e
confrontare a occhio baseline e ottimizzato nella stessa simulazione.

### Uso

```bash
# 1. genera il piano ottimizzato (punto 2) — calcolo pesante, alcuni minuti
python -m signal_optimization.optimize piccola media grande

# 2. traduci il piano in additional-file SUMO (punto 3)
python scripts/inject_signal_plan.py                 # tutte le zone disponibili
python scripts/inject_signal_plan.py piccola         # una sola zona

# 3. visualizza: i semafori ottimizzati sono caricati automaticamente
python scripts/sumo_visualize.py piccola
python scripts/sumo_visualize.py piccola --baseline  # forza i semafori originali
```

`sumo_visualize.py` aggiunge da solo la riga
`<additional-files value=".../tls_<zona>.add.xml"/>` al `.sumocfg`, ma solo se il
file esiste; se manca, la simulazione parte come prima con i semafori del
net.xml. La zona viene ricavata dalla rete davvero usata, non da quella passata
da riga di comando. La webapp avvia SUMO con lo stesso script passando sempre
`piccola`, ma se il problema custom è di un'altra zona vengono caricati la rete e
i semafori giusti di quella zona.

### Mapping dei nodi PDDL sulle junction SUMO

I nomi dei nodi PDDL (`n9100868`) vanno ricollegati agli id delle junction SUMO,
e la corrispondenza non è sempre diretta perché netconvert semplifica la rete in
modo diverso da come lo fa il nostro grafo contratto. `pddl_name_to_junction`
prova allora in ordine: id esatto, poi junction il cui id finisce con quel
suffisso (i nomi PDDL tengono solo le ultime 7 cifre), poi junction cluster che
contiene quell'id fra i suoi membri (netconvert fonde i nodi vicini in
`cluster_<id1>_<id2>_...`).

Alcuni nodi però non esistono proprio come incrocio in SUMO. Per questi non ci
affidiamo più solo all'id: la webapp salva anche le coordinate e la forma delle
strade in `route_custom.json`, e la ricostruzione del percorso (vedi Fase 4) usa
come punti fissi solo i nodi che mappano con certezza, riempiendo i tratti in
mezzo con un percorso che segue la geometria reale. Start e goal, se non sono
incroci SUMO, vengono agganciati all'incrocio più vicino per posizione. Prima di
questa gestione la visualizzazione dei problemi custom di media e grande poteva
partire dal punto sbagliato o mostrare una strada diversa da quella della webapp.

Sulla parte dei semafori la verifica automatica non dà errori su nessuna zona:
stati di fase uguali all'originale, stesso numero di fasi, `programID` sempre
nuovo, nessuna durata nulla o negativa, e ciclo che passa da 90 s (default
netconvert) a 120 s (valore SCATS usato in tutto il progetto) su tutti i 577
semafori delle tre reti.

| Zona | `tlLogic` scritti | Fasi modificate | `total-time` PDDL+ baseline → ottimizzato |
|---------|------------------:|----------------:|-------------------------------------------|
| piccola | 27                | 96              | 16.45 s → 10.33 s (**-37.2%**)            |
| media   | 97                | 399             | 72.04 s → 47.07 s (**-34.7%**)            |
| grande  | 453               | 1745            | 97.80 s → 67.94 s (**-30.5%**)            |

---

## Confronto in simulazione: baseline vs ottimizzato (`compare_sumo.py`)

### A cosa serve

L'ottimizzazione dei semafori calcola le nuove durate del verde con una formula
matematica, il ritardo uniforme di Webster, applicata dentro PDDL+. Quella
formula prevede un guadagno, ma parte da ipotesi semplificate: guarda ogni
incrocio come se fosse isolato, con arrivi casuali, senza code che si accumulano
e senza interferenza fra semafori vicini.

`compare_sumo.py` serve a controllare quella previsione sul campo. Mette davvero
circa 45 veicoli in strada dentro il simulatore e misura quanto ci mettono, prima
e dopo l'ottimizzazione. Una cosa è dire che l'ottimizzazione funziona, un'altra
è misurarlo con uno strumento diverso da quello usato per ottimizzare.

E il controllo non è una formalità, perché su una delle tre zone ha smentito la
previsione (vedi `grande` più sotto): lì il piano che sulla carta migliora del
30% risulta invece peggiore del 10%. Senza questa verifica avremmo scritto un
guadagno che nella simulazione non c'è.

### Come funziona

SUMO riproduce il traffico nel dettaglio, con code, accelerazioni, frenate e fasi
vere di verde, giallo e rosso, quindi coglie proprio gli effetti che la formula
analitica ignora.

### Disegno dell'esperimento

Per ogni zona facciamo due simulazioni identiche, che cambiano solo nel programma
semaforico (baseline = programma `0` del net.xml, ottimizzato = programma
`optimized` da `tls_<zona>.add.xml`). Per tenere il confronto pulito usiamo la
stessa domanda, cioè le coppie O-D di `sumo_extracted/demand_<zona>.json` (lo
stesso campione del punto 2), e le stesse rotte, calcolate una volta con Dijkstra
e riusate uguali nei due run. Se lasciassimo ricalcolare il percorso a SUMO, i
veicoli potrebbero scegliere strade diverse e finiremmo per misurare due effetti
insieme. Usiamo anche lo stesso seed e gli stessi istanti di partenza, e
disattiviamo i teletrasporti (`--time-to-teleport -1`), altrimenti un veicolo
bloccato sparirebbe e le attese risulterebbero più basse del vero.

### Risultati misurati

| Zona | Veicoli | Tempo di viaggio | Attesa ai semafori | Tempo perso |
|---------|--------:|------------------------|------------------------|------------------------|
| piccola | 46 | 32.7 → 30.6 s (**-6.6%**) | 3.2 → 1.0 s (**-68.5%**) | 6.0 → 3.8 s (**-36.5%**) |
| media | 45 | 139.7 → 127.8 s (**-8.5%**) | 44.5 → 34.2 s (**-23.1%**) | 53.4 → 41.5 s (**-22.3%**) |
| grande | 43 | 270.7 → 277.3 s (**+2.4%**) | 99.1 → 109.1 s (**+10.1%**) | 117.8 → 124.4 s (**+5.6%**) |

Su piccola e media il guadagno previsto si conferma. Su grande invece
l'ottimizzazione peggiora le cose, anche se la stima analitica la dava in
miglioramento. Il motivo è che il ritardo di Webster tratta ogni incrocio come
isolato, e questa ipotesi cade in una rete densa dove le code passano da un
incrocio all'altro, gli offset non vengono ricalibrati e solo una parte dei
semafori è ottimizzata. A conferma, se rifacciamo il confronto su grande con
meno traffico (11 veicoli invece di 43) il segno si inverte (-1,4% di attesa): il
peggioramento salta fuori sotto congestione, cioè proprio dove le ipotesi di
Webster valgono meno.

### Uso

```bash
python scripts/compare_sumo.py                        # tutte le zone
python scripts/compare_sumo.py piccola media grande
python scripts/compare_sumo.py grande --max-vehicles 15
```

I risultati si accumulano, quindi calcolare una zona alla volta non cancella
quelle già fatte. L'output finisce in `sumo_comparison/` (`results.json` +
`report.md`, con una sezione di interpretazione generata in automatico).

Il confronto si può lanciare anche dalla webapp, insieme ai due pulsanti per
aprire la simulazione con i semafori ottimizzati o con quelli originali.

---

## Dominio PDDL+

Il dominio ha un'azione discreta, un processo continuo e un evento automatico.
Rispetto alla versione base, `start-move` conosce anche il nodo da cui si arriva
(`?prev`), così può addebitare il tempo di svolta e il ritardo del semaforo, e un
unico fatto `(prev ...)` tiene traccia dell'ultimo arco percorso (viene tolto in
`start-move` e rimesso da `arrive`).

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

Il processo `driving` fa crescere `progress` con continuità grazie alla variabile
temporale `#t`. L'evento `arrive` scatta da solo quando `progress >= distance` e
aggiorna `total-time` in un colpo, sommando il tempo di guida e il ritardo di
congestione del nodo di arrivo. L'azione `start-move` aggiunge invece il tempo di
svolta e il ritardo del semaforo per il movimento `(?prev, ?from, ?to)` che sta
per iniziare. Lo addebitiamo alla partenza da `?from` e non all'arrivo a `?to`
perché è quello il momento in cui il veicolo aspetta davvero il verde di quel
movimento (vedi la sezione sui semafori e `2_traffic_signal_optimization.md`, sez.
1, per il passaggio da un valore medio per nodo a uno specifico per movimento).
Aggiorniamo `total-time` solo nelle azioni ed eventi discreti, mai nel processo
continuo, altrimenti ENHSP tratterebbe il tempo accumulato come una variabile da
ricampionare a ogni istante e la ricerca diventerebbe molto più lenta.

> Nota sulla scomposizione del tempo. La timeline che stampa ENHSP (gli istanti
> delle azioni) riflette solo il tempo di guida del processo continuo; i ritardi
> di semaforo, congestione e svolta sono incrementi discreti su `total-time` e non
> fanno avanzare l'orologio simulato. Il costo vero del piano è quindi la somma:
> total-time = guida + semafori + congestione + svolte.

---

## Interfaccia web (`webapp/`)

Il progetto ha anche un'interfaccia web fatta con Flask e Leaflet.js. Permette di
caricare un file `.osm`, vedere la rete stradale sulla mappa con i nodi
semaforizzati evidenziati, scegliere start e goal cliccando sui nodi, far
risolvere il problema a ENHSP e vedere il percorso sulla mappa, e infine aprire la
simulazione in sumo-gui con un pulsante.

Ogni volta che risolve, la webapp salva il problema in
`pddl_files/problem_custom.pddl`, così lo si può riguardare o aprire in SUMO da
riga di comando.

La webapp usa lo stesso modello della generazione da riga di comando: scrive i
fatti `turn-time`, l'init `prev` e i `signal-delay` realistici (con la mappa unita
dei ritardi SUMO delle tre zone; per un incrocio che non compare nei dati SUMO usa
il valore di default di un incrocio a 2 fasi, circa 17 s). Il parser del piano
gestisce l'azione `start-move` a tre argomenti per ricostruire il percorso da
passare a SUMO.

### Replanning: strade chiuse e ricalcolo del percorso

L'interfaccia permette di segnare strade e incroci come non percorribili e di far
ricalcolare il percorso. Il replanning non riparte dall'inizio: fa finta che il
veicolo sia già in viaggio e trovi la strada chiusa, quindi ripianifica dal nodo
appena prima della chiusura, che è l'ultimo punto ancora raggiungibile.

**Come funziona, dall'inizio**

Prima due parole sui termini, per chi parte da zero. La mappa della città è un
grafo, dove gli incroci sono i nodi e le strade che li collegano sono gli archi.
ENHSP è il programma che, dati un nodo di partenza e uno di arrivo, calcola il
percorso migliore, quello che costa meno tempo tenendo conto di guida, semafori e
svolte. Un percorso è solo una sequenza di nodi da attraversare.

Il replanning serve quando, dopo aver già trovato un percorso, alcune strade
diventano impraticabili (lavori, incidenti) e ne serve uno nuovo che le eviti.
L'idea è quella che verrebbe in mente al volante: se trovi una strada chiusa non
torni indietro alla partenza, prosegui da dove sei e aggiri l'ostacolo. Di seguito
tutti i passaggi in ordine.

Passo 0, la situazione di partenza. Hai già risolto un problema, quindi esiste un
percorso calcolato da ENHSP, cioè una lista di nodi dallo start al goal, disegnato
sulla mappa.

Passo 1, segnali cosa è chiuso. Clicchi sulla mappa le strade e gli incroci da
rendere impraticabili. Due regole di modellazione: una strada chiusa lo è nei due
sensi di marcia, e chiudere un incrocio vale come chiudere tutte le strade che vi
entrano ed escono. Dentro il programma diventano due insiemi, `blocked_edges` per
le strade chiuse e `blocked_nodes` per gli incroci chiusi.

Passo 2, trovi dove il percorso incontra il primo blocco. Il programma scorre il
percorso attuale nodo per nodo e si ferma alla prima strada chiusa. Il nodo appena
prima è il punto da cui ripartire (`replan_from`), cioè l'ultimo posto che l'auto
raggiunge prima dell'ostacolo. Il nodo ancora prima serve a ricordare da che
direzione arrivava l'auto (`prev`, utile al passo 5). Se nessuna strada del
percorso è chiusa non c'è niente da ricalcolare, e il programma lo dice subito.

Passo 3, costruisci una mappa tagliata. Si fa una copia del grafo dove spariscono
le strade e gli incroci chiusi (`open_edges` sono solo gli archi non bloccati).
ENHSP userà questa mappa, e non vedendo affatto le strade chiuse è impossibile che
le riproponga. È qui che i punti vengono esclusi.

Passo 4, controlli che l'arrivo sia ancora raggiungibile. Prima di far lavorare il
planner, una visita del grafo (una BFS, cioè un'esplorazione a cerchi concentrici
da `replan_from`) controlla che dal punto di ripartenza si possa ancora arrivare
al goal usando solo le strade aperte. Se le chiusure isolano la destinazione, il
programma lo dice con un messaggio chiaro invece di far girare ENHSP per niente.

Passo 5, rifai girare ENHSP. Questa è la parte centrale. Si costruisce un problema
PDDL nuovo, con partenza `replan_from`, arrivo lo stesso goal e mappa quella
tagliata del passo 3, e lo si dà a ENHSP come per il primo calcolo. Il planner non
aggiusta il vecchio piano, ne calcola uno nuovo da zero, che per forza aggira il
blocco. Al problema aggiungiamo anche il nodo `prev`, così la prima svolta dopo la
ripartenza ha un costo realistico (l'auto sta già andando in una direzione, non è
ferma, quindi girare le costa tempo). Piccola ottimizzazione: a ENHSP non diamo
tutta la città ma solo la porzione di mappa attorno al percorso, per farlo
rispondere in fretta anche sulle mappe grandi.

Passo 6, mostri il risultato. Il percorso finale è fatto di due pezzi, il tratto
già percorso (che resta com'era, fino al blocco) e la deviazione appena calcolata.
La mappa li disegna con colori diversi, insieme al piano originale sbiadito, e un
pannello riassume quanto costa in più la deviazione in distanza, tempo e semafori
attraversati.

In breve, alle due domande tipiche la risposta è sì e sì. Sì, i punti chiusi
vengono esclusi (si costruisce una mappa senza le strade chiuse), e sì, ENHSP
viene rifatto girare (un nuovo problema dal punto di blocco all'arrivo). Il codice
sta nella funzione `replan()` di `webapp/app.py` (endpoint `/api/replan`), che usa
`write_pddl` (`webapp/pddl_writer.py`) per scrivere il problema e `run_enhsp`
(`webapp/enhsp_runner.py`) per risolverlo.

**Come si usa**

1. Risolvi normalmente il problema (compare la sezione *Strade chiuse & ricalcolo*).
2. Premi *Attiva modalità chiusura*, il cursore diventa un mirino.
3. Clicca una strada per chiuderla o un incrocio per bloccarlo (ri-clicca per
   riaprire). Le chiusure appaiono in rosso tratteggiato con un cartello di lavori.
4. Premi *Ricalcola percorso*.

Poi sulla mappa vedi insieme il piano originale sbiadito, il tratto già percorso,
il punto di ricalcolo con un'icona che pulsa e la deviazione con il tratteggio
animato che scorre verso il goal. Un pannello riepiloga distanza, tempo e semafori
prima e dopo, con il costo della deviazione.

**Dettagli di modellazione**

Una strada chiusa lo è in entrambi i sensi (è una chiusura vera, non un senso
unico). Il nodo di provenienza viene passato a ENHSP come `prev`, così la prima
svolta dopo il ricalcolo ha il costo reale, perché ripartendo da metà percorso il
veicolo è già orientato. Chiudere un incrocio vale come chiudere tutte le strade
che ci arrivano. Se le chiusure non toccano il percorso attuale, il sistema se ne
accorge e non ricalcola nulla.

**Quali strade si possono chiudere: l'interfaccia lo dice prima del click**

Il sottografo estratto è spesso quasi un albero, cioè molte strade sono l'unico
collegamento verso la destinazione, quindi chiuderle la isola e il ricalcolo
fallisce, giustamente. Per non lasciare l'utente a tentativi, appena si attiva la
modalità chiusura il percorso viene colorato in base a una BFS calcolata nel
browser. Il giallo vuol dire che chiudendo quel tratto esiste una deviazione, il
rosso scuro punteggiato vuol dire che è l'unico collegamento e chiuderlo
isolerebbe il goal. Cliccando un tratto critico l'avviso compare subito, senza
aspettare il server.

**Quanti nodi servono**

Più nodi vuol dire più alternative disponibili, ma anche ENHSP più lento. Ecco i
tratti chiudibili sul percorso, misurati.

| Nodi | Zona media | Zona grande |
|-----:|------------|-------------|
| 120  | 5 su 9     | 1 su 15     |
| 200  | —          | —           |
| 300  | 14 su 45   | 7 su 32     |
| 400  | —          | 11 su 29    |

Lo slider arriva a 1000 e parte da 200. Il grafo contratto completo ha 938 nodi
(media) e 3756 (grande), quindi *tutti i nodi* copre per intero la zona media.

**Il limite dipende dalla macchina.** `start-move` e `signal-delay` hanno tre
argomenti (`prev`, `from`, `to`), quindi il numero di istanze che ENHSP genera
cresce con il cubo dei nodi e la memoria si riempie in fretta. Su un ambiente
molto modesto (3 GB di RAM, 1 core) con heap da 2 GB: 300 nodi in 8 s, 400 in 14
s, 939 va in OutOfMemory. Su un PC normale il tetto è molto più alto, perciò i
default sono heap 6 GB e limite 1200 nodi, entrambi regolabili.

```bash
set ENHSP_HEAP=8g            # heap della JVM per ENHSP
set MAX_SOLVABLE_NODES=2000  # tetto di sicurezza sui nodi
set ENHSP_TIMEOUT=300        # secondi; 0 o assente = nessun limite (default)
```

Di default ENHSP gira senza timeout, perché sui grafi grandi il grounding può
volerci qualche minuto e fermare la ricerca a metà buttava via il lavoro già
fatto. Lo svantaggio è che una richiesta molto pesante resta in attesa senza poterla
annullare dal browser; se serve un tetto, si imposta `ENHSP_TIMEOUT`.

Se ENHSP finisce la memoria, l'interfaccia lo dice chiaramente (non più un generico
"problema irrisolvibile") e suggerisce cosa fare.

> La zona `piccola` non è adatta al replanning. Il centro storico (Temple Bar) è
> fatto di vie strette a senso unico e resta quasi ad albero anche usando tutti i
> suoi 201 nodi: solo 1 tratto su 44 è chiudibile. Per provare il replanning
> conviene usare la zona media.

### I controlli SUMO nell'interfaccia

Quando ENHSP ha trovato la soluzione compaiono tre controlli, che fanno cose
diverse.

| Controllo | Cosa fa | Cosa ottieni |
|---|---|---|
| ▶ Apri in SUMO (semafori ottimizzati) | apre sumo-gui con il tuo percorso e i semafori ottimizzati | una verifica visiva, guardi la tua auto che percorre la strada |
| ▶ Apri in SUMO (semafori originali) | uguale, ma con i semafori di default del net.xml | il confronto a occhio, aprendo prima uno e poi l'altro vedi se l'auto si ferma di più o di meno ai rossi |
| Confronto in SUMO (selettore zona + Confronta) | simula tutta la zona con circa 45 veicoli, due volte (originali e ottimizzati) | una tabella con i numeri: tempo di viaggio, attesa ai semafori e tempo perso, con la variazione percentuale |

La differenza importante è questa: i due pulsanti mostrano una singola corsa, la
tua, mentre il pannello di confronto misura la qualità del piano semaforico della
zona su tanti veicoli, e per questo ha un selettore di zona e non usa il percorso
che hai disegnato. Il calcolo ci mette qualche secondo perché fa due simulazioni
complete.

Nella tabella il verde è un miglioramento e il rosso un peggioramento. Sulla zona
grande il risultato è rosso, e non è un errore, come spiegato nella sezione sul
confronto in simulazione.

---

## Struttura del repository

```
├── README.md
├── requirements.txt
├── setup.bat
├── setup_macos.py
│
├── scripts/                   # Pipeline dati e strumenti di confronto/ottimizzazione
│   ├── build_problems.py          # Genera i file problem_*.pddl da OSM
│   ├── extract_sumo_data.py       # Estrae semafori/settaggi/turn dai net.xml SUMO
│   ├── inject_signal_plan.py      # Punto 3: inietta il piano ottimizzato in SUMO
│   ├── compare_sumo.py            # Punto 4: confronto in simulazione baseline vs ottimizzato
│   ├── download_dublin_map.py     # Scarica le mappe OSM tramite osmnx
│   ├── convert_to_osm.py          # Converte OSM in net.xml tramite netconvert
│   ├── sumo_visualize.py          # Visualizza il piano in sumo-gui
│   ├── compare_versions.py        # Confronto vecchia/nuova versione del dominio
│   └── generate_demand.py         # Campione O-D condiviso fra i punti 2 e 4
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
├── comparison_results/        # Output di scripts/compare_versions.py
├── diagnostics_out/           # Report diagnostici
│
├── sumo_comparison/           # Punto 4: risultati del confronto in simulazione
│   ├── results.json
│   └── report.md
│
├── sumo_extracted/            # Dati estratti da SUMO (generati da scripts/extract_sumo_data.py)
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
    ├── app.py                 # Route Flask (generate/solve/replan/sumo)
    ├── osm_graph.py           # Parsing OSM, grafo contratto, congestione
    ├── sumo_signals.py        # Bearing, tempo di svolta, ritardo semaforico per movimento
    ├── pddl_writer.py         # Generazione del problema PDDL+ e route_metrics
    ├── enhsp_runner.py        # Discovery/esecuzione ENHSP, parsing del piano
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
python scripts/sumo_visualize.py piccola    # oppure: media, grande

# Percorso generato dalla webapp
python scripts/sumo_visualize.py pddl pddl_files/problem_custom.pddl piccola
```

---

## Risultati

### Zona Piccola

Piano trovato da ENHSP, 14 nodi, 20 archi, 4 semafori sul percorso.

| Tempo (s) | Nodo                        | Semaforo |
|-----------|-----------------------------|----------|
| 0         | Liffey Street Upper (START) |          |
| 10        | Wellington Quay Est         | +30 s    |
| 128       | Cork Hill                   | +30 s    |
| 132       | Cork Hill Sud               | +30 s    |
| ~314      | Aungier Street (GOAL)       | +30 s    |

Distanza percorsa 1.57 km, ritardo semaforico totale 120 s, tempo totale circa 314 s.

### Zona Media

Piano trovato da ENHSP, 50 nodi, 93 archi, 18 azioni `start-move`.

Distanza percorsa 1.62 km, durata del piano 160 s.

### Zona Grande

Piano trovato da ENHSP, 120 nodi, 206 archi, 15 azioni `start-move`.

Distanza percorsa 1.33 km, durata del piano 147 s.

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

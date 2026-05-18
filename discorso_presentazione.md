# Discorso di presentazione — Map Construction in PDDL+
**Progetto #2 — Automated Planning | UNICAL**
Gruppo: Chiara, Elisa, Emanuele, Pierluigi

---

> **Come usare questo documento:**
> Ogni sezione è uno "blocco" da assegnare a un membro del gruppo o da leggere in sequenza.
> Le parti tra parentesi quadre `[così]` sono note interne — non vanno lette ad alta voce.

---

## APERTURA — Cos'è il progetto (1-2 min)

"Il nostro progetto si chiama *Map Construction in PDDL+* e l'obiettivo è modellare un problema di navigazione stradale reale usando il linguaggio PDDL+, e risolverlo con un planner automatico.

In concreto: abbiamo preso la mappa reale di Dublino da OpenStreetMap, l'abbiamo trasformata in un problema PDDL+, e abbiamo usato il planner ENHSP per trovare il percorso ottimale tra un punto di partenza e una destinazione. Il tutto è poi visualizzato in SUMO, un simulatore di traffico.

La scelta di Dublino non è casuale: è una città con una rete stradale densa, molti sensi unici, e dati OSM ben mantenuti — un buon banco di prova per il planning."

---

## FASE 1 — Raccolta dati da OpenStreetMap (2 min)

"Il primo passo è scaricare la mappa. Abbiamo usato la libreria Python **osmnx**, che interroga le API di OpenStreetMap e scarica i dati stradali di una zona geografica specificando coordinate GPS e raggio.

Abbiamo creato tre istanze di dimensione crescente:
- **Piccola**: zona Temple Bar, raggio 400 metri — 14 nodi, 20 archi
- **Media**: zona Ranelagh, raggio 1.2 km — 50 nodi, 93 archi
- **Grande**: zona Docklands/Porto, raggio 3 km — 120 nodi, 206 archi

Il risultato è un file `.osm`, che è un XML. Contiene due tipi di elementi: i **nodi**, cioè punti con coordinate GPS, e le **way**, cioè le strade — sequenze di nodi con tag come il tipo di strada, il limite di velocità e il senso unico.

[Se il prof chiede: il flag `all_oneway=True` serve per preservare i sensi unici originali di OSM così come sono, senza che osmnx li 'sistemi'.]"

---

## FASE 2 — Costruzione del file PDDL (parte più importante, 4-5 min)

"Il cuore del progetto è lo script `build_problems.py`, che trasforma il file OSM grezzo in un problema PDDL+ risolvibile. Fa quattro cose distinte.

**Prima cosa: identifica gli incroci.**
Un file OSM può avere migliaia di nodi, ma la maggior parte sono solo punti intermedi di una curva — non hanno senso come 'fermate' del percorso. Quello che conta sono gli *incroci*, cioè i punti dove il conducente può scegliere una direzione. Lo script identifica come incrocio un nodo che si trova all'inizio o alla fine di una strada, oppure che compare in due o più strade diverse.

**Seconda cosa: costruisce il grafo contratto.**
Tra due incroci consecutivi, lo script salta tutti i nodi intermedi e calcola la distanza totale del tratto con la formula **Haversine**. Haversine calcola la distanza reale in metri tra due coordinate GPS tenendo conto della curvatura della Terra — è la formula standard per questo tipo di calcolo geografico. Per la velocità, legge il tag `maxspeed` della strada in OSM e lo converte da km/h a m/s: ad esempio 30 km/h diventano 8.33 m/s, 50 km/h diventano 13.89 m/s. Se la strada non ha un `maxspeed`, assume 30 km/h come default.

**Terza cosa: seleziona il sottografo.**
Il grafo contratto ha ancora troppi nodi per essere risolto in tempi ragionevoli. Lo script seleziona un sottoinsieme di N nodi — 50 per la media, 120 per la grande — con un criterio geografico: parte dal nodo più connesso, poi espande aggiungendo ogni volta il nodo più lontano dal centroide geografico del gruppo già selezionato. In questo modo i nodi scelti sono distribuiti su tutta la zona e non ammucchiati in un quartiere.
Lo **start** viene scelto come il nodo con più archi uscenti nel sottografo — il 'hub' principale. Il **goal** viene scelto come il nodo raggiungibile più lontano dallo start in linea d'aria.

**Quarta cosa: assegna i nomi e scrive il PDDL.**
Per ogni nodo, lo script cerca il nome della strada su OSM e lo converte in un identificatore PDDL valido — ad esempio 'Dame Street' diventa `dame_st`. Se il nodo non ha un nome su OSM — cioè è un incrocio anonimo — usa le ultime 7 cifre del suo ID OSM con una 'n' davanti: `n4005414`. Quindi tutti quei nomi del tipo `n4005414` nel file `problem_grande.pddl` non sono inventati: sono veri incroci di Dublino che su OpenStreetMap esistono ma non hanno un cartello con il nome.

Il file PDDL risultante contiene, per ogni coppia di nodi collegati: la strada `(road A B)`, la distanza in metri `(= (distance A B) 173)`, la velocità in m/s `(= (speed A B) 8.33)`, e il progress inizializzato a zero `(= (progress A B) 0)`.

[Nota: la zona piccola è scritta a mano con nomi leggibili — è il caso base/test. Le zone media e grande sono generate automaticamente da OSM.]"

---

## FASE 3 — Il dominio PDDL+ (3 min)

"Il dominio è scritto in PDDL+, che è un'estensione di PDDL classico che supporta il **tempo continuo**. Questo è fondamentale per modellare il movimento fisico di un veicolo: non si tratta di un problema a stati discreti, ma di un processo che evolve nel tempo.

Il dominio definisce tre costrutti:

**L'azione `start-move`** è un'azione istantanea: il veicolo decide di percorrere una strada da A a B. Le precondizioni sono che il veicolo sia in A e che la strada A→B esista. L'effetto è che il veicolo non è più in A e inizia a muoversi verso B, con il progress azzerato.

**Il processo `driving`** è un processo continuo: finché il veicolo è in movimento da A a B, il progress aumenta nel tempo secondo la formula `progress += speed × Δt`. Il simbolo `#t` in PDDL+ rappresenta proprio la variabile temporale continua — è questo che rende il dominio PDDL+ e non PDDL classico.

**L'evento `arrive`** è un evento che scatta automaticamente nel momento esatto in cui il progress raggiunge la distanza del tratto. Non è un'azione che si sceglie — scatta da solo. Il suo effetto è che il veicolo arriva in B, il total-dist viene aggiornato, e il progress viene azzerato.

Insieme, questi tre costrutti modellano fedelmente la fisica del movimento: il veicolo parte, percorre la strada in tempo reale proporzionale alla distanza e alla velocità, e arriva a destinazione automaticamente.

La metrica del problema è `minimize (total-dist)`, quindi ENHSP non cerca una soluzione qualsiasi — cerca quella che minimizza la distanza totale percorsa."

---

## FASE 4 — ENHSP risolve il problema (1-2 min)

"Per risolvere il problema usiamo **ENHSP**, un planner che supporta PDDL+ con variabili numeriche e tempo continuo. Lo lanciamo con lo script `run.py` passandogli domain e problem.

L'algoritmo usato è `-s aibr` — Anytime Interval-Based Relaxation. Abbiamo testato altri solver ma non convergevano su domini PDDL+ con processi ed eventi.

I risultati sono:
- Zona piccola: percorso da Liffey Street a Aungier Street, 1.57 km, trovato in 44 millisecondi, 207 nodi esplorati
- Zona media: 1.62 km, 18 tratti
- Zona grande: 1.33 km, 15 tratti

Il piano prodotto è una sequenza di `start-move` con i timestamp esatti di esecuzione — il momento in cui il veicolo inizia ogni tratto."

---

## FASE 5 — Visualizzazione in SUMO (1-2 min)

"L'ultimo pezzo è la visualizzazione. SUMO è un simulatore di traffico microscopico che usa la stessa rete stradale OSM, ma è completamente separato da PDDL+: i due sistemi non si parlano direttamente.

Il collegamento viene fatto una volta sola: ogni nome PDDL corrisponde a un nodo OSM con un ID numerico, e il file `net.xml` di SUMO contiene quegli stessi ID. Con un BFS sul grafo del net.xml, troviamo la sequenza di archi SUMO che corrisponde al piano PDDL+ trovato da ENHSP.

In SUMO si vede il veicolo rosso che percorre esattamente il percorso ottimale trovato dal planner. Una cosa importante: SUMO applica la fisica realistica — accelerazione, frenata, e soprattutto i **semafori reali di Dublino** presi da OSM. Questo spiega perché il tempo del piano PDDL+ — ad esempio 194 secondi per la zona piccola — è inferiore al tempo reale di Google Maps: il piano PDDL+ è un *lower bound* ottimistico che assume velocità costante senza semafori né traffico. SUMO rende la simulazione più realistica."

---

## CHIUSURA — Riepilogo e possibili domande (1 min)

"In sintesi, il progetto dimostra come PDDL+ sia adatto a modellare problemi di navigazione con tempo continuo, e come sia possibile costruire automaticamente istanze di dimensione realistica a partire da dati geografici reali.

Le tre istanze — piccola, media, grande — mostrano la scalabilità: da 14 nodi risolti in 44ms a 120 nodi risolti comunque in tempi pratici.

Siamo pronti per qualsiasi domanda."

---

## Possibili domande del prof — risposte pronte

**"Perché PDDL+ e non PDDL classico?"**
"Perché il movimento fisico è un processo continuo: la velocità è costante lungo ogni tratto e il tempo di percorrenza dipende dalla distanza. Con PDDL classico dovremmo discretizzare il tempo, perdendo fedeltà e precisione. PDDL+ permette di modellare questo direttamente con il processo `driving` e la variabile `#t`."

**"Come garantite che il percorso sia ottimale?"**
"ENHSP cerca il piano che minimizza `total-dist`. La metrica è dichiarata nel problema PDDL e il planner la usa come funzione obiettivo — non si ferma alla prima soluzione trovata."

**"Cosa succede se un nodo non è raggiungibile?"**
"`build_problems.py` verifica la raggiungibilità con un BFS prima di scegliere il goal: il goal viene scelto solo tra i nodi raggiungibili dallo start. Se il grafo fosse sconnesso, il problema non verrebbe scritto."

**"Perché il tempo del piano è molto inferiore al tempo reale?"**
"Il piano PDDL+ assume velocità costante, nessun semaforo, nessun traffico. È un lower bound teorico — il percorso ottimale in condizioni ideali. SUMO applica invece i semafori reali di Dublino e la fisica del veicolo, avvicinandosi più alla realtà."

**"Da dove vengono i numeri nel file PDDL?"**
"Tutti dai dati reali di OpenStreetMap. Le distanze vengono calcolate con la formula Haversine dalle coordinate GPS dei nodi. Le velocità vengono lette dal tag `maxspeed` di ogni strada in OSM e convertite da km/h a m/s."

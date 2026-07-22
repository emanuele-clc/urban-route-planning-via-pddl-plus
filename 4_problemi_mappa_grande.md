# Problemi riscontrati sulla mappa "grande" e relative soluzioni

> **Scope.** Questo documento riporta l'audit condotto sulla webapp usando
> `osm_files/dublin_grande_porto.osm` (3756 incroci, la zona più estesa tra
> le tre disponibili) dopo la separazione di `webapp/app.py` in moduli.
> Sono stati identificati e risolti due problemi indipendenti, entrambi
> riproducibili con dati reali ed entrambi mascherati, prima della modifica,
> dal vecchio tetto `MAX_SOLVABLE_NODES` (che bloccava a monte molte delle
> istanze che li avrebbero esposti).

---

## Problema 1 — il sottografo passato a ENHSP dipende dalla mappa caricata, non dal tragitto richiesto

### Sintomo

Caricando `dublin_grande_porto.osm` con l'opzione "tutti i nodi" e
risolvendo la coppia `n4904418 → n6472876`:

```
problema troppo grande per ENHSP: 3727 nodi (massimo 1000). Rigenera il
grafo con meno nodi, oppure aumenta la heap con ENHSP_HEAP e alza
MAX_SOLVABLE_NODES.
```

### Causa

`/api/generate` seleziona un sottografo (`selected`) con
`select_connected_subgraph()`, che cresce da un singolo nodo "seed" (il
più connesso) aggiungendo di volta in volta il nodo del fronte più
lontano dal baricentro della selezione corrente — una crescita che non
tiene conto di dove l'utente vuole effettivamente andare. `/api/solve`
passava poi **l'intero** `selected` a `write_pddl()` come oggetti PDDL,
indipendentemente dalla distanza reale tra `start` e `goal`.

Verificato sui dati reali:

- la coppia `n4904418 → n6472876` è a **337 m di strada, 5 hop** — un
  tragitto banale;
- eppure non compariva in nessun sottografo generato con `max_nodes` ≤
  600: serviva "tutti i nodi" perché quei due incroci venissero anche
  solo *inclusi* nella mappa mostrata;
- anche quando il sottografo da 1000 nodi la includeva, ENHSP **non
  convergeva in 180 s** per un tragitto di 5 hop, perché il problema
  veniva comunque grounded su tutti i 1000 nodi;
- una seconda coppia, agli estremi di un sottografo da soli 400 nodi (ben
  sotto il vecchio tetto di 1000), andava **in timeout dopo 150 s**.

| Test | Nodi nel problema | Esito |
|---|---|---|
| estremi mappa | 100 | risolto in 0.9 s |
| estremi mappa | 400 | **timeout dopo 150 s** |
| coppia utente (5 hop reali) | 1000 | **timeout dopo 180 s** |

Il costo del solve dipendeva quindi dalla dimensione della mappa
*caricata*, non dalla distanza reale tra i due punti richiesti.

### Soluzione

Nuova funzione `select_local_subgraph(start, goal, edges, node_data)` in
[`webapp/osm_graph.py`](webapp/osm_graph.py#L244): calcola il percorso più
breve start→goal con Dijkstra (sempre incluso) più un margine di
deviazioni plausibili — un corridoio ellittico `dist_da_start +
dist_da_goal ≤ ottimo × 1.6` — troncato a 150 nodi prendendo prima quelli
più vicini al percorso ottimo.

`/api/solve` e `/api/replan` in `webapp/app.py` ora costruiscono questo
sottografo locale **dopo** aver individuato start/goal (o il punto di
ricalcolo, per il replanning) e lo passano a `write_pddl()` al posto
dell'intero grafo caricato. Il controllo `MAX_SOLVABLE_NODES` valuta ora
la dimensione del sottografo locale, non della mappa intera — quindi
riflette la vera complessità della richiesta invece che la copertura
della mappa mostrata sullo schermo. `MAX_NODES`/timeout ENHSP e l'opzione
"tutti i nodi" non sono stati toccati: restano scelte dell'utente per la
sola *visualizzazione*.

### Verifica

| Caso | Prima del fix | Dopo il fix |
|---|---|---|
| coppia del bug report ("tutti i nodi", 3727 nodi) | errore immediato "3727 nodi (massimo 1000)" | sottografo locale: **21 nodi → risolto in 0.3 s** (5 hop) |
| coppia lontana, max_nodes=400 | timeout dopo 150 s | sottografo locale: **112 nodi → risolto in 0.9 s** (29 hop) |

Smoke test end-to-end sui veri endpoint Flask (`test_client`): `/api/generate`
+ `/api/solve` su zona piccola, e `/api/replan` su zona media con una
strada chiusa che forza una deviazione reale — entrambi confermati
funzionanti senza regressioni.

---

## Problema 2 — overflow numerico dell'euristica `aibr` di ENHSP sui percorsi lunghi

### Sintomo

Dopo la soluzione del Problema 1, una nuova istanza (`n4904418 →
n2238142`, un tragitto reale di 74 hop) falliva con:

```
ENHSP dichiara il problema irrisolvibile: la destinazione non e'
raggiungibile dal punto di partenza con i vincoli attuali.
```

nonostante il sottografo locale contenesse un percorso completo e
verificato (BFS su `local_edges`: goal raggiungibile).

### Causa

Non è un bug della webapp, ma un limite numerico dell'euristica `aibr`
(Additive Interval-Based Relaxation) di ENHSP, invocata con `-s aibr` in
[`webapp/enhsp_runner.py`](webapp/enhsp_runner.py#L69). Isolando il
problema e rieseguendo ENHSP a mano con logging dettagliato:

```
h(I):3.4028235E38
Problem unsolvable
```

`3.4028235E38` è **esattamente `Float.MAX_VALUE`**: non è "infinito"
semantico, è un **overflow** del valore euristico. Tagliando il percorso
reale a lunghezze crescenti si osserva una crescita esponenziale di
`h(I)`:

| Hop nel percorso | h(I) |
|---|---|
| 5 | 2 943 |
| 20 | 3.16 × 10¹² |
| 40 | 2.35 × 10²⁴ |
| 60 | 2.58 × 10³⁶ |
| **74** | **3.40 × 10³⁸ → overflow → "unsolvable"** |

Oltre ~65-70 hop l'euristica satura il range di un float a 32 bit ed
ENHSP interpreta la saturazione come irraggiungibilità: un **falso
negativo**, non una vera assenza di soluzione. Prima del Problema 1,
istanze così lunghe venivano quasi sempre bloccate a monte dal vecchio
tetto sui nodi; il fix del sottografo locale, sbloccandole, ha esposto
questo secondo bug.

> **Nota — perché non "basta" cambiare tipo numerico.** `h(I)` è una stima
> di costo (relaxed-plan cost estimate) e per costruzione non può essere
> negativa: il fallimento osservato è una saturazione verso l'alto, non un
> overflow con cambio di segno. Passare da `float` a `double` internamente
> ad ENHSP sposterebbe la soglia di overflow molto più in là (range fino a
> ~1.8×10³⁰⁸ contro ~3.4×10³⁸), ma non è percorribile qui — ENHSP è
> consumato come dipendenza binaria (`up-enhsp` via pip), non ricompilata —
> e comunque non risolverebbe la causa: la crescita di `h(I)` è
> **esponenziale** col numero di hop, quindi rinvierebbe il sintomo a
> percorsi ancora più lunghi invece di eliminarlo. Da qui la scelta del
> fallback mirato invece di un fix "sui bit".

### Soluzione

Provato sostituendo l'euristica con `blind` (`-h blind`, nessuna guida
euristica — equivalente a ricerca a costo uniforme): lo stesso identico
problema si risolve correttamente, ed è risultato anche più veloce
dell'euristica di default su un caso normale già funzionante:

| Caso | `aibr` (default) | `blind` |
|---|---|---|
| 74 hop, 150 nodi locali | "unsolvable" (falso negativo) | risolto in 23 ms |
| 112 nodi locali, 29 hop | risolto in 170 ms | risolto in 48 ms |

Non essendoci però garanzia che `blind` regga bene su problemi con molta
più diramazione (dove l'euristica `aibr`, quando non satura, guida la
ricerca in modo più efficiente della ricerca non informata), non si è
sostituita l'euristica di default globalmente. È stato invece implementato
un **fallback mirato** in `run_enhsp_output()`
([`webapp/enhsp_runner.py`](webapp/enhsp_runner.py#L87)): si tenta prima
con `aibr` (veloce nel caso comune); solo se ENHSP dichiara "unsolvable"
**con la firma esatta dell'overflow** (costante `AIBR_OVERFLOW_SIGNATURE
= "3.4028235E38"`), si ritenta automaticamente con `-h blind` prima di
restituire l'errore. `/api/solve` e `run_enhsp()` (usata da
`/api/replan`) condividono ora questa stessa funzione, eliminando anche
una duplicazione di codice preesistente tra i due endpoint.

### Verifica

Ripetuta l'istanza esatta segnalata attraverso il vero endpoint Flask
(`/api/generate` + `/api/solve`, zona grande, "tutti i nodi"):

```
success: True   enhsp_error: None
route hops: 74   plan_time_ms: 51.0
```

Confermato anche che un caso semplice (zona piccola, che non attiva mai
il fallback) continua a risolversi normalmente, senza regressioni di
comportamento o prestazioni.

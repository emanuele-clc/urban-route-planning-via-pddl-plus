;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; DOMAIN: map-navigation
;;
;; Dominio PDDL+ per la navigazione di un veicolo su una rete stradale
;; estratta da OpenStreetMap (Project #2 – Automated Planning, UNICAL 2026).
;;
;; ┌─────────────────────────────────────────────────────────────────────┐
;; │  MODELLO CONCETTUALE                                                │
;; │                                                                     │
;; │  Il grafo stradale è rappresentato come:                           │
;; │    • LOCATION  →  nodi del grafo (intersezioni stradali)           │
;; │    • ROAD      →  archi diretti (corsia di marcia)                 │
;; │    • VEHICLE   →  l'agente che naviga                              │
;; │                                                                     │
;; │  Ogni arco ha due proprietà numeriche:                             │
;; │    • road-length   : lunghezza in metri                            │
;; │    • speed-limit   : limite di velocità in m/s                     │
;; │                                                                     │
;; │  Il veicolo percorre un arco con un'azione durativa (drive).       │
;; │  Durata = road-length / speed-limit  [secondi]                     │
;; │                                                                     │
;; │  Fluenti tracciati durante la navigazione:                         │
;; │    • total-distance  : distanza totale percorsa [m]                │
;; │    • travel-time     : tempo totale di percorrenza [s]             │
;; │                                                                     │
;; │  Metrica tipica:  minimize (travel-time vehicle1)                  │
;; └─────────────────────────────────────────────────────────────────────┘
;;
;; Requirements PDDL+ usati:
;;   :typing                – tipi di oggetti
;;   :numeric-fluents       – fluenti numerici
;;   :durative-actions      – azioni con durata
;;   :duration-inequalities – durata come espressione (= ?duration ...)
;;
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

(define (domain map-navigation)

  (:requirements
    :typing
    :numeric-fluents
    :durative-actions
    :duration-inequalities
  )

  ;; ── Tipi ────────────────────────────────────────────────────────────────
  (:types
    location  ; nodo del grafo stradale (intersezione)
    vehicle   ; agente che percorre la rete
  )

  ;; ── Predicati ───────────────────────────────────────────────────────────
  (:predicates
    (at   ?v - vehicle  ?l - location)          ; veicolo v è alla location l
    (road ?from - location  ?to - location)     ; arco diretto da from a to
  )

  ;; ── Fluenti numerici ────────────────────────────────────────────────────
  (:functions
    (road-length    ?from - location  ?to - location)
    ;; ↑ lunghezza dell'arco in metri [m]

    (speed-limit    ?from - location  ?to - location)
    ;; ↑ velocità consentita sull'arco in m/s
    ;;   Nota: convertire da km/h → m/s dividendo per 3.6
    ;;   (il generatore osm_to_pddl.py lo fa automaticamente)

    (total-distance ?v - vehicle)
    ;; ↑ distanza totale percorsa dal veicolo [m], inizializzata a 0

    (travel-time    ?v - vehicle)
    ;; ↑ tempo totale di percorrenza [s], inizializzato a 0
  )

  ;; ── Azione: drive ───────────────────────────────────────────────────────
  ;;
  ;; Percorre un arco (road) del grafo da una location alla successiva.
  ;;
  ;; Parametri:
  ;;   ?v    – il veicolo che si sposta
  ;;   ?from – location di partenza
  ;;   ?to   – location di arrivo
  ;;
  ;; Durata:
  ;;   ?duration = road-length(?from,?to) / speed-limit(?from,?to)
  ;;   (in secondi; calcolato automaticamente dal planner)
  ;;
  ;; Precondizioni:
  ;;   [at start]  il veicolo è in ?from
  ;;   [at start]  esiste un arco diretto da ?from a ?to
  ;;
  ;; Effetti:
  ;;   [at start]  il veicolo lascia ?from
  ;;   [at end]    il veicolo arriva in ?to
  ;;   [at end]    aggiorna total-distance  (+= road-length)
  ;;   [at end]    aggiorna travel-time     (+= durata del percorso)
  ;;
  (:durative-action drive
    :parameters (
      ?v    - vehicle
      ?from - location
      ?to   - location
    )

    :duration (= ?duration
      (/ (road-length ?from ?to)
         (speed-limit ?from ?to))
    )

    :condition (and
      (at start (at   ?v ?from))
      (at start (road ?from ?to))
    )

    :effect (and
      (at start (not (at ?v ?from)))
      (at end   (at  ?v ?to))
      (at end   (increase (total-distance ?v)
                          (road-length ?from ?to)))
      (at end   (increase (travel-time ?v)
                          (/ (road-length ?from ?to)
                             (speed-limit ?from ?to))))
    )
  )

)

;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;
;; ESTENSIONI POSSIBILI (per sviluppi futuri del progetto)
;;
;; 1. Traffico / congestione
;;    Aggiungere un fluente  (congestion-factor ?from ?to)  che scala la
;;    velocità effettiva:   speed_eff = speed-limit * (1 - congestion-factor)
;;    Modellabile con un :process PDDL+ che aumenta congestion nel tempo.
;;
;; 2. Carburante
;;    Fluente  (fuel ?v)  decrementato di  (road-length ?from ?to) * consumption
;;    Precondizione:  (>= (fuel ?v) (road-length ?from ?to) * consumption)
;;
;; 3. Semafori / traffico a tempo
;;    :event triggered quando travel-time raggiunge certi valori,
;;    che modifica la disponibilità di certi archi (apertura/chiusura corsie).
;;
;; 4. Veicoli multipli
;;    Il predicato (at) e i fluenti già supportano più veicoli.
;;    Aggiungere vincoli di non-collisione se necessario.
;;
;; 5. Integrazione SUMO
;;    Usare SUMO come simulatore di validazione: esportare il piano PDDL+
;;    in route files SUMO (.rou.xml) e verificare la fattibilità sul simulatore.
;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;;

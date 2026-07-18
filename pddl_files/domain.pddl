(define (domain dublin-navigation)

  (:requirements :typing :fluents :time :continuous-effects)

  (:types location)

  (:predicates
    (at ?l - location)
    (road ?from ?to - location)
    (moving ?from ?to - location)
    (peripheral ?l - location)
    (prev ?p - location)                     ; nodo da cui si e' arrivati al nodo corrente
  )

  (:functions
    (distance ?from ?to - location)          ; distanza in metri
    (speed ?from ?to - location)             ; velocita' base in m/s
    (effective-speed ?from ?to - location)   ; speed / congestion-factor (precalcolata)
    (arc-time ?from ?to - location)          ; dist / effective-speed (precalcolata)
    (progress ?from ?to - location)          ; metri percorsi sul tratto
    (congestion-delay ?l - location)         ; ritardo statico congestione (s)
    (vehicle-count ?from ?to - location)     ; veicoli random sull'arco
    (congestion-factor ?from ?to - location) ; 1 + vehicle-count/10
    (intersection-density ?l - location)     ; incroci entro 200m
    (turn-time ?prev ?from ?to - location)   ; tempo di svolta (s) a ?from da ?prev verso ?to
    (signal-delay ?prev ?from ?to - location) ; ritardo semaforico (s) del MOVIMENTO (prev,from,to) a ?from
    (total-dist)
    (total-time)
  )

  ; start-move ora conosce il nodo di provenienza (?prev) e paga il tempo di
  ; svolta all'incrocio ?from: turn-time = angolo_di_svolta / turn-rate.
  ; signal-delay e' addebitato qui (non in 'arrive') perche' rappresenta
  ; l'attesa al semaforo di ?from PRIMA di impegnare il movimento verso ?to,
  ; ed e' specifico della coppia di strade (prev->from->to), non del nodo:
  ; incroci con piu' fasi hanno ritardi diversi per movimenti diversi.
  ; Il fatto (prev ?prev) viene cancellato qui e ristabilito da 'arrive'
  ; (?prev diventa ?from), cosi' resta sempre un solo fatto prev attivo.
  (:action start-move
    :parameters (?prev ?from ?to - location)
    :precondition (and (at ?from) (road ?from ?to) (prev ?prev))
    :effect (and
      (not (at ?from))
      (not (prev ?prev))
      (moving ?from ?to)
      (assign (progress ?from ?to) 0)
      (increase (total-time) (turn-time ?prev ?from ?to))
      (increase (total-time) (signal-delay ?prev ?from ?to))
    )
  )

  ; Il processo usa effective-speed precalcolata — nessuna divisione runtime
  (:process driving
    :parameters (?from ?to - location)
    :precondition (moving ?from ?to)
    :effect (and
      (increase (progress ?from ?to) (* #t (effective-speed ?from ?to)))
    )
  )

  ; L'evento usa arc-time precalcolata — nessuna divisione runtime
  (:event arrive
    :parameters (?from ?to - location)
    :precondition (and
      (moving ?from ?to)
      (>= (progress ?from ?to) (distance ?from ?to))
    )
    :effect (and
      (not (moving ?from ?to))
      (at ?to)
      (prev ?from)
      (increase (total-dist) (distance ?from ?to))
      (increase (total-time) (arc-time ?from ?to))
      (increase (total-time) (congestion-delay ?to))
      (assign (progress ?from ?to) 0)
    )
  )

)

(define (domain dublin-navigation)

  (:requirements :typing :fluents :time :continuous-effects)

  (:types location)

  (:predicates
    (at ?l - location)
    (road ?from ?to - location)
    (moving ?from ?to - location)
    (peripheral ?l - location)
  )

  (:functions
    (distance ?from ?to - location)          ; distanza in metri
    (speed ?from ?to - location)             ; velocita' base in m/s
    (effective-speed ?from ?to - location)   ; speed / congestion-factor (precalcolata)
    (arc-time ?from ?to - location)          ; dist / effective-speed (precalcolata)
    (progress ?from ?to - location)          ; metri percorsi sul tratto
    (signal-delay ?l - location)             ; 30 se semaforo OSM, 0 altrimenti
    (congestion-delay ?l - location)         ; ritardo statico congestione (s)
    (vehicle-count ?from ?to - location)     ; veicoli random sull'arco
    (congestion-factor ?from ?to - location) ; 1 + vehicle-count/10
    (intersection-density ?l - location)     ; incroci entro 200m
    (total-dist)
    (total-time)
  )

  (:action start-move
    :parameters (?from ?to - location)
    :precondition (and (at ?from) (road ?from ?to))
    :effect (and
      (not (at ?from))
      (moving ?from ?to)
      (assign (progress ?from ?to) 0)
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
      (increase (total-dist) (distance ?from ?to))
      (increase (total-time) (arc-time ?from ?to))
      (increase (total-time) (signal-delay ?to))
      (increase (total-time) (congestion-delay ?to))
      (assign (progress ?from ?to) 0)
    )
  )

)

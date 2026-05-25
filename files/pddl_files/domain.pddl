(define (domain dublin-navigation)

  (:requirements :typing :fluents :time :continuous-effects)

  (:types location)

  (:predicates
    (at ?l - location)
    (road ?from ?to - location)
    (moving ?from ?to - location)
  )

  (:functions
    (distance ?from ?to - location)   ; distanza in metri
    (speed ?from ?to - location)      ; velocita' in m/s
    (progress ?from ?to - location)   ; metri percorsi su questo tratto
    (signal-delay ?l - location)      ; 30 se semaforo OSM, 0 altrimenti
    (total-dist)                      ; distanza totale percorsa (metri)
    (total-time)                      ; tempo totale (secondi) — solo aggiornato negli eventi
  )

  ;; AZIONE: inizia a percorrere la strada da ?from a ?to
  (:action start-move
    :parameters (?from ?to - location)
    :precondition (and
      (at ?from)
      (road ?from ?to)
    )
    :effect (and
      (not (at ?from))
      (moving ?from ?to)
      (assign (progress ?from ?to) 0)
    )
  )

  ;; PROCESSO: solo progress avanza continuamente — total-time NON e' continuo
  (:process driving
    :parameters (?from ?to - location)
    :precondition (moving ?from ?to)
    :effect (and
      (increase (progress ?from ?to) (* #t (speed ?from ?to)))
    )
  )

  ;; EVENTO: arrivo — aggiunge tempo di guida (dist/speed) + ritardo semaforo
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
      (increase (total-time) (/ (distance ?from ?to) (speed ?from ?to)))
      (increase (total-time) (signal-delay ?to))
      (assign (progress ?from ?to) 0)
    )
  )

)

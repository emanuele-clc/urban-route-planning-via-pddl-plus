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
    (total-dist)                      ; distanza totale percorsa
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

  ;; PROCESSO: la distanza percorsa su questo tratto aumenta nel tempo
  (:process driving
    :parameters (?from ?to - location)
    :precondition (moving ?from ?to)
    :effect (and
      (increase (progress ?from ?to) (* #t (speed ?from ?to)))
    )
  )

  ;; EVENTO: quando si raggiunge la destinazione
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
      (assign (progress ?from ?to) 0)
    )
  )

)

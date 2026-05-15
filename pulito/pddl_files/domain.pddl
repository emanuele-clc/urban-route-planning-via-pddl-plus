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
    (speed ?from ?to - location)      ; velocità in m/s
    (progress)                        ; metri percorsi sul tratto corrente
    (total-time)                      ; tempo totale accumulato
  )

  ;; AZIONE: il veicolo inizia a muoversi da ?from verso ?to
  (:action start-move
    :parameters (?from ?to - location)
    :precondition (and
      (at ?from)
      (road ?from ?to)
    )
    :effect (and
      (not (at ?from))
      (moving ?from ?to)
      (assign (progress) 0)
    )
  )

  ;; PROCESSO: mentre è in movimento, il progresso aumenta continuamente
  (:process driving
    :parameters (?from ?to - location)
    :precondition (moving ?from ?to)
    :effect (and
      (increase (progress) (* #t (speed ?from ?to)))
      (increase (total-time) (* #t 1))
    )
  )

  ;; EVENTO: quando progress >= distanza, il veicolo è arrivato
  (:event arrive
    :parameters (?from ?to - location)
    :precondition (and
      (moving ?from ?to)
      (>= (progress) (distance ?from ?to))
    )
    :effect (and
      (not (moving ?from ?to))
      (at ?to)
      (assign (progress) 0)
    )
  )

)
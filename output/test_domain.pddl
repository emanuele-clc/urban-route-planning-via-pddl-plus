(define (domain test)
  (:requirements :typing :numeric-fluents :durative-actions)
  (:types location vehicle)
  (:predicates
    (at ?v - vehicle ?l - location)
    (road ?from - location ?to - location)
  )
  (:functions
    (travel-time ?v - vehicle)
    (road-time ?from - location ?to - location)
  )
  (:durative-action drive
    :parameters (?v - vehicle ?from - location ?to - location)
    :duration (= ?duration 10)
    :precondition (and
      (at start (at ?v ?from))
      (at start (road ?from ?to))
    )
    :effect (and
      (at start (not (at ?v ?from)))
      (at end (at ?v ?to))
      (at end (increase (travel-time ?v) 10))
    )
  )
)

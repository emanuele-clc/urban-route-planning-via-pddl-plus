(define (domain map-navigation)
  (:requirements :typing :numeric-fluents :durative-actions :duration-inequalities)
  (:types location vehicle)
  (:predicates
    (at   ?v - vehicle  ?l - location)
    (road ?from - location  ?to - location)
  )
  (:functions
    (road-length  ?from - location  ?to - location)
    (speed-limit  ?from - location  ?to - location)
    (total-distance ?v - vehicle)
    (travel-time    ?v - vehicle)
  )
  (:durative-action drive
    :parameters (?v - vehicle  ?from - location  ?to - location)
    :duration (= ?duration (/ (road-length ?from ?to) (speed-limit ?from ?to)))
    :condition (and
      (at start (at   ?v ?from))
      (at start (road ?from ?to))
    )
    :effect (and
      (at start (not (at ?v ?from)))
      (at end   (at  ?v ?to))
      (at end   (increase (total-distance ?v) (road-length ?from ?to)))
      (at end   (increase (travel-time ?v) (/ (road-length ?from ?to) (speed-limit ?from ?to))))
    )
  )
)
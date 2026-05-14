;; DOMAIN: map-navigation  (PDDL+)
;;
;; Proper PDDL+ encoding using action / process / event.
;;
;; start-drive  (action)   : vehicle begins traversing a road segment
;; moving       (process)  : continuous motion along the road
;; arrive       (event)    : vehicle reaches next intersection

(define (domain map-navigation)

  (:requirements :typing :numeric-fluents :negative-preconditions :time)

  (:types location vehicle)

  (:predicates
    (at      ?v - vehicle ?l - location)
    (road    ?from - location ?to - location)
    (driving ?v - vehicle ?from - location ?to - location)
    (free    ?v - vehicle)
  )

  (:functions
    (road-length  ?from - location ?to - location)
    (speed-limit  ?from - location ?to - location)
    (position     ?v - vehicle)
    (total-distance ?v - vehicle)
    (travel-time  ?v - vehicle)
  )

  ;; Instantaneous action: begin driving from ?from toward ?to
  (:action start-drive
    :parameters (?v - vehicle ?from - location ?to - location)
    :precondition (and
      (at ?v ?from)
      (road ?from ?to)
      (free ?v)
    )
    :effect (and
      (not (at ?v ?from))
      (not (free ?v))
      (driving ?v ?from ?to)
      (assign (position ?v) 0)
    )
  )

  ;; Continuous process: vehicle moves along the road at speed-limit
  (:process moving
    :parameters (?v - vehicle ?from - location ?to - location)
    :precondition (and
      (driving ?v ?from ?to)
      (< (position ?v) (road-length ?from ?to))
    )
    :effect (and
      (increase (position ?v)       (* #t (speed-limit ?from ?to)))
      (increase (total-distance ?v) (* #t (speed-limit ?from ?to)))
      (increase (travel-time ?v)    #t)
    )
  )

  ;; Event: vehicle has covered the full road length -> arrives
  (:event arrive
    :parameters (?v - vehicle ?from - location ?to - location)
    :precondition (and
      (driving ?v ?from ?to)
      (>= (position ?v) (road-length ?from ?to))
    )
    :effect (and
      (not (driving ?v ?from ?to))
      (at ?v ?to)
      (free ?v)
      (assign (position ?v) 0)
    )
  )

)

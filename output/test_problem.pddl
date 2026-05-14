(define (problem test-nav)
  (:domain test)
  (:objects
    locA locB - location
    car1 - vehicle
  )
  (:init
    (at car1 locA)
    (road locA locB)
    (= (travel-time car1) 0)
    (= (road-time locA locB) 10)
  )
  (:goal (at car1 locB))
  (:metric minimize (travel-time car1))
)

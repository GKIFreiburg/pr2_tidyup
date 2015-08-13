(define (problem p01)
  (:domain tidyup_grounding)
  (:moduleoptions (navstack_init@libplanner_modules_pr2.so /map 0.05 1) (liftTorsoInit@libplanner_modules_pr2.so) (drivePoseInit@libplanner_modules_pr2.so) )
  (:moduleexitoptions )
  (:objects
    /map - frameid
    table1 table2 - table
  )
  (:init
    (robot-x 5.0)
    (robot-y 5.0)
    (robot-theta 0.0)
    (= (torso-position) 0.0117837)
    (= (qw table1) 1)
    (= (qx table1) 0)
    (= (qy table1) 0)
    (= (qz table1) 0)
    (= (timestamp table1) 0)
    (= (x table1) 4.81)
    (= (y table1) 5.28)
    (= (z table1) 0.9)
    (= (qw table2) 1)
    (= (qx table2) 0)
    (= (qy table2) 0)
    (= (qz table2) 0)
    (= (timestamp table2) 0)
    (= (x table2) 0.5)
    (= (y table2) 6.44)
    (= (z table2) 0.73)
    (= (arm-state left_arm) arm_unknown)
    (= (arm-state right_arm) arm_unknown)
    (= (frame-id table1) /map)
    (= (frame-id table2) /map)
  )
  (:goal (and
    (forall (?o - arm) (hand-free ?o))
    (arms-drive-pose)
    (sensor-data-stale)
    ;(table-inspected table1)
    ;(forall (?t - table) (table-inspected ?t))
    ;(forall (?o - movable_object) (object-inspected ?o))
  ))
)

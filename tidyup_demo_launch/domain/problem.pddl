(define (problem p01)
  (:domain hybris_demo)
  ;(:moduleoptions (navstack_init@libplanner_modules_pr2.so /map 0.05 1) )
  (:objects
    /map - frameid
    loc1 loc2 loc3 - manipulation_location
    table1 - table
    cup1 bowl1 bowl2 - movable_object
  )
  (:init
    (robot-at robot_location)
    (location-near-table loc1 table1)
    (location-near-table loc2 table1)
    (location-near-table loc3 table1)
    (object-on cup1 table1)
    (object-on bowl1 table1)
    (object-on bowl2 table1)
    (= (arm-state left_arm) arm_unknown)
    (= (arm-state right_arm) arm_unknown)
  )
  (:goal (and
    (forall (?o - movable_object) (object-inspected ?o))
    (forall (?o - movable_object) (object-on ?o table1))
    ;(object-inspected cup1)
    ;(= (arm-state right_arm) arm_at_front)
    ;(object-grasped cup1 right_arm)
    ;(inspected loc1)
    ;(robot-at loc1)
    (arms-drive-pose)
  ))
)

(define (domain hybris_demo)
    (:requirements :strips :typing :durative-actions :modules :fluents :derived-predicates :equality)

    (:types
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose, ideally a fixed frame

        location - pose                 ; a pose for the robot base
        manipulation_location - location       ; a location that grasp actions can be applied from

        table - pose                    ; something static like a table
        movable_object - pose           ; an object with pose that can be grasped

        arm                             ; use left or right arm, see constants
        arm_state                       ; specific arm positions, see constants
    )

    (:modules
         (path-condition ?s - location ?g - location conditionchecker navigation_cost@libplanner_modules_pr2.so)
         (update-robot-pose ?s - location ?g - location
            (x robot_location) (y robot_location) (z robot_location) (qx robot_location) (qy robot_location) (qz robot_location) (qw robot_location)
            effect navigation_effect@libplanner_modules_pr2.so)

         (can-pickup ?o - movable_object ?a - arm ?t - table ?l - manipulation_location conditionchecker can_pickup@libplanner_modules_pr2.so)
         (apply-pickup ?o - movable_object ?a - arm ?t - table ?l - manipulation_location
            (x ?o) (y ?o) (z ?o) (qx ?o) (qy ?o) (qz ?o) (qw ?o)
            effect pickup_effect@libplanner_modules_pr2.so)

         (can-putdown ?o - movable_object ?a - arm ?t - table ?l - manipulation_location conditionchecker can_putdown@libplanner_modules_pr2.so)
         (apply-putdown ?o - movable_object ?a - arm ?t - table ?l - manipulation_location
            (x ?o) (y ?o) (z ?o) (qx ?o) (qy ?o) (qz ?o) (qw ?o)
            effect putdown_effect@libplanner_modules_pr2.so)

;        (path-cost ?start ?goal - location cost path_cost@libplanner_modules_pr2.so)
;        (lift-cost ?t - table cost lift_torso_cost@libplanner_modules_pr2.so)
;        (need-lift-torso ?t - table conditionchecker need_to_lift_torso@libplanner_modules_pr2.so)
;        (torso-lifted ?t - table conditionchecker torso_lifted@libplanner_modules_pr2.so)
;        (update-torso-position ?t - table
;            (torso-position)
;            effect update_torso_position@libplanner_modules_pr2.so)
    )

    (:constants
        left_arm right_arm - arm
        arm_at_side arm_unknown arm_at_front - arm_state
        robot_location - location
    )

    (:predicates
        ; location was scanned at least once
        (location-inspected ?l - manipulation_location)
        ; location is updated with sensor data
        (location-inspected-recently ?l - manipulation_location) 
        (object-inspected ?o - movable_object)              ; object is inspected

        (robot-at ?l - location)
        (location-near-table ?l - manipulation_location ?t - table)

        (object-grasped ?o - movable_object ?a - arm)
        (object-on ?o - movable_object ?t - table)
   )

    (:functions
        (arm-state ?a - arm) - arm_state

        (torso-position) - number

        (x ?p - pose) - number
        (y ?p - pose) - number
        (z ?p - pose) - number
        ; quaternion orientation
        (qx ?p - pose) - number
        (qy ?p - pose) - number
        (qz ?p - pose) - number
        (qw ?p - pose) - number
        (timestamp ?p - pose) - number
        (inspection-timestamp ?l - manipulation_location) - number
        (frame-id ?p - pose) - frameid
    )

	; aquire sensor data from this location
    (:durative-action inspect-location
        :parameters (?l - manipulation_location ?t - table)
        :duration (= ?duration 1.0)
        :condition
        (and
            (at start (robot-at ?l))
            (at start (location-near-table ?l ?t))
            (at start (not (location-inspected-recently ?l)))
            (at start (arms-drive-pose))
;            (at start ([torso-lifted ?t]))
        )
        :effect
        (and
            (at end (location-inspected-recently ?l))
            (at end (location-inspected ?l))
        )
    )

	; aquire sensor data of object in hand
    (:durative-action inspect-object
        :parameters (?o - movable_object ?a - arm)
        :duration (= ?duration 5.0)
        :condition
        (and
            (at start (not (object-inspected ?o)))
            (at start (= (arm-state ?a) arm_at_front))
            (at start (object-grasped ?o ?a))
        )
        :effect
        (and
            (at end (object-inspected ?o))
        )
    )

    (:durative-action move-robot
        :parameters (?s - location ?g - location)
;        :duration (= ?duration [path-cost ?s ?g])
        :duration (= ?duration 1000)
        :condition
        (and
            (at start (robot-at ?s))
            (at start (not (= ?s ?g)))
            (at start ([path-condition ?s ?g]))
            (at start (arms-drive-pose))
        )
        :effect
        (and
            (at start (not (location-inspected-recently ?s)))
            (at start (not (location-inspected-recently ?g)))
            (at start (not (robot-at ?s)))
            (at end (robot-at ?g))
            (at end ([update-robot-pose ?s ?g]))
        )
    )

    (:durative-action pickup-object
        :parameters (?o - movable_object ?a - arm ?t - table ?l - manipulation_location)
        :duration (= ?duration 20.0)
        :condition
        (and
            (at start (robot-at ?l))
            (at start (location-near-table ?l ?t))
            (at start (location-inspected-recently ?l))
            (at start (arms-drive-pose))
            (at start (object-on ?o ?t))
            (at start (hand-free ?a))
            (at start (not (object-inspected ?o)))
            (at start ([can-pickup ?o ?a ?t ?l]))
        )
        :effect
        (and
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (not (location-inspected-recently ?l)))
            (at end (not (object-on ?o ?t)))
            (at end (object-grasped ?o ?a))
            (at end ([apply-pickup ?o ?a ?t ?l]))
        )
    )

    (:durative-action putdown-object
        :parameters (?o - movable_object ?a - arm ?t - table ?l - manipulation_location)
        :duration (= ?duration 20.0)
        :condition
        (and
            (at start (robot-at ?l))
            (at start (location-near-table ?l ?t))
            (at start (location-inspected-recently ?l))
            (at start (arms-drive-pose))
            (at start (object-grasped ?o ?a))
            (at start (object-inspected ?o))
            (at start ([can-putdown ?o ?a ?t ?l]))
        )
        :effect
        (and
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (not (location-inspected-recently ?l)))
            (at end (object-on ?o ?t))
            (at end (not (object-grasped ?o ?a)))
            (at end ([apply-putdown ?o ?a ?t ?l]))
        )
    )

;    (:durative-action lift-torso
;        :parameters (?t - table ?l - manipulation_location)
;        ;:duration (= ?duration 15.0)
;        :duration (= ?duration [lift-cost ?t])
;        :condition
;        (and
;            (at start (robot-at ?l))
;            (at start (location-near-table ?l ?t))
;            (at start (arms-drive-pose))
;            ;(at start (not (torso-lifted ?t)))
;            (at start ([need-lift-torso ?t]))
;       )
;        :effect
;        (and
;            (at start (not (location-inspected-recently ?l)))
;            ;(at end ([torso-lifted ?t]))
;            (at end ([update-torso-position ?t]))
;        )
;    )

    (:durative-action arm-to-side
        :parameters (?a - arm)
        :duration (= ?duration 15.0)
        :condition
        (and
            (at start (not (= (arm-state ?a) arm_at_side)))
        )
        :effect
        (and
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (assign (arm-state ?a) arm_at_side))
        )
    )

    (:durative-action arm-to-front
        :parameters (?a - arm)
        :duration (= ?duration 15.0)
        :condition
        (and
            ;(at start (arms-drive-pose))
            (at start (not (= (arm-state ?a) arm_at_front)))
        )
        :effect
        (and
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (assign (arm-state ?a) arm_at_front))
        )
    )

    (:derived
        (arms-drive-pose)
        (and
            ; make sure arms are in drive position
            ; arms holding no objects should moved to side
            (forall (?_a - arm)
                (= (arm-state ?_a) arm_at_side)
            )
        )
    )

    (:derived
        (hand-free ?a - arm)
        (and
            (forall (?_o - movable_object)
                (not (object-grasped ?_o ?a))
            )
        )
    )

)


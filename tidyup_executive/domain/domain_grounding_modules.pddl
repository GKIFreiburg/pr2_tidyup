(define (domain tidyup_grounding)
    (:requirements :strips :typing :durative-actions :modules :fluents :derived-predicates :equality :grounding-modules)

    (:types
        pose                            ; any pose in space
        frameid                         ; the coordinate frame of a pose, ideally a fixed frame

        location - pose                 ; a pose for the robot base

        table - pose                    ; something static like a table
        movable_object - pose           ; an object with pose that can be grasped

        arm                             ; use left or right arm, see constants
        arm_state                       ; specific arm positions, see constants
    )

    (:modules
        (determine-drive-pose grounding determine_drive_pose@libplanner_modules_pr2.so)
        (robot-near-table ?t - table conditionchecker robot_near_table@libplanner_modules_pr2.so)
        
        (path-cost ?t - table cost navigation_cost_grounding@libplanner_modules_pr2.so)
        (path-condition ?t - table conditionchecker navigation_cost_grounding@libplanner_modules_pr2.so)
        (update-robot-pose ?t - table
            (robot-x)
            (robot-y)
            (robot-theta)
            (robot-torso-position)
            effect navigation_effect_grounding@libplanner_modules_pr2.so)
         
        (can-pickup ?o - movable_object ?a - arm ?t - table conditionchecker can_pickup@libplanner_modules_pr2.so)
        (apply-pickup ?o - movable_object ?a - arm ?t - table
            (x ?o) (y ?o) (z ?o) (qx ?o) (qy ?o) (qz ?o) (qw ?o)
            effect pickup_effect@libplanner_modules_pr2.so)

        (can-putdown ?o - movable_object ?a - arm ?t - table conditionchecker can_putdown@libplanner_modules_pr2.so)
        (apply-putdown ?o - movable_object ?a - arm ?t - table
            (x ?o) (y ?o) (z ?o) (qx ?o) (qy ?o) (qz ?o) (qw ?o)
            effect putdown_effect@libplanner_modules_pr2.so)
    )

    (:constants
        left_arm right_arm - arm
        arm_at_side arm_unknown arm_at_front - arm_state
    )

    (:predicates
        (table-inspected ?t - table)
        (table-inspected-recently ?t - table)
        (object-inspected ?o - movable_object)

        (object-grasped ?o - movable_object ?a - arm)
        (object-on ?o - movable_object ?t - table)
   )

    (:functions
        (arm-state ?a - arm) - arm_state

        (x ?p - pose) - number
        (y ?p - pose) - number
        (z ?p - pose) - number
        (qx ?p - pose) - number
        (qy ?p - pose) - number
        (qz ?p - pose) - number
        (qw ?p - pose) - number
        (timestamp ?p - pose) - number
        (frame-id ?p - pose) - frameid

        (robot-x) - number
        (robot-y) - number
        (robot-theta) - number
        (robot-torso-position) - number
    )

    (:durative-action inspect-table
        :parameters (?t - table)
        :duration (= ?duration 10.0)
        :condition
        (and
            (at start ([robot-near-table ?t]))
            (at start (not (table-inspected-recently ?t)))
            (at start (arms-drive-pose))
        )
        :effect
        (and
            (at end (table-inspected-recently ?t))
            (at end (table-inspected ?t))
        )
    )

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

    (:durative-action move-robot-to-table
        :parameters (?t - table)
        :grounding ([determine-drive-pose])
        :duration (= ?duration [path-cost ?t])
        ;:duration (= ?duration 20.0)
        :condition
        (and
            (at start ([path-condition ?t]))
            (at start (arms-drive-pose))
        )
        :effect
        (and
            (at start (not (table-inspected-recently ?t)))
            (at end ([update-robot-pose ?t]))
        )
    )

    (:durative-action pickup-object
        :parameters (?o - movable_object ?a - arm ?t - table)
        :duration (= ?duration 20.0)
        :condition
        (and
            (at start ([robot-near-table ?t]))
            (at start (table-inspected-recently ?t))
            (at start (arms-drive-pose))
            (at start (object-on ?o ?t))
            (at start (hand-free ?a))
            (at start (not (object-inspected ?o)))
            (at start ([can-pickup ?o ?a ?t]))
        )
        :effect
        (and
            (at start (not (table-inspected-recently ?t)))
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (not (object-on ?o ?t)))
            (at end (object-grasped ?o ?a))
            (at end ([apply-pickup ?o ?a ?t]))
        )
    )

    (:durative-action putdown-object
        :parameters (?o - movable_object ?a - arm ?t - table)
        :duration (= ?duration 20.0)
        :condition
        (and
            (at start ([robot-near-table ?t]))
            (at start (table-inspected-recently ?t))
            (at start (arms-drive-pose))
            (at start (object-inspected ?o))
            (at start (object-grasped ?o ?a))
            (at start ([can-putdown ?o ?a ?t]))
        )
        :effect
        (and
            (at start (not (table-inspected-recently ?t)))
            (at start (assign (arm-state ?a) arm_unknown))
            (at end (object-on ?o ?t))
            (at end (not (object-grasped ?o ?a)))
            (at end ([apply-putdown ?o ?a ?t]))
        )
    )

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


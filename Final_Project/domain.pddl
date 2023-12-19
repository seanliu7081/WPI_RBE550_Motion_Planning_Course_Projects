(define (domain sussman-anomaly)
  (:requirements :strips :equality)
  (:predicates
    (block ?b)
    (clear ?b)
    (on-table ?b)
    (on ?b1 ?b2)
    (holding ?b)
    (hand-empty))

  (:action pick-up
    :parameters (?b)
    :precondition (and (block ?b) (clear ?b) (on-table ?b) (hand-empty))
    :effect (and (not (on-table ?b)) (not (hand-empty)) (holding ?b)))

  (:action put-down
    :parameters (?b)
    :precondition (and (block ?b) (holding ?b))
    :effect (and (not (holding ?b)) (hand-empty) (on-table ?b)))

  (:action stack
    :parameters (?b1 ?b2)
    :precondition (and (block ?b1) (block ?b2) (holding ?b1) (clear ?b2))
    :effect (and (not (holding ?b1)) (hand-empty) (on ?b1 ?b2) (not (clear ?b2))))

  (:action unstack
    :parameters (?b1 ?b2)
    :precondition (and (block ?b1) (block ?b2) (on ?b1 ?b2) (clear ?b1) (hand-empty))
    :effect (and (holding ?b1) (clear ?b2) (not (on ?b1 ?b2)) (not (hand-empty))))

  ; Additional streams and derived predicates can be added here if needed
)

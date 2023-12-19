(define (problem sussman-anomaly-problem)
  (:domain sussman-anomaly)
  (:objects
    a b c - block
    robot - robot
    table - room
    grasp1 grasp2 grasp3 - grasp
    pose1 pose2 pose3 pose4 pose5 pose6 - pose
    conf1 conf2 conf3 conf4 conf5 conf6 - conf
    traj1 traj2 traj3 traj4 traj5 traj6 - traj)

  (:init
    ; Initial block poses
    (Pose a pose1)
    (Pose b pose2)
    (Pose c pose3)
    (Supported a pose1 table)
    (Supported b pose2 table)
    (Supported c pose3 table)
    (On-table a)
    (On-table b)
    (On c a)
    (Clear b)
    (Clear c)

    ; Robot configuration
    (AtConf robot conf1)
    (HandEmpty robot)

    ; Sampled grasps for blocks
    (Grasp a grasp1)
    (Grasp b grasp2)
    (Grasp c grasp3)

    ; Kinematic constraints (simplified for the example)
    (Kin a pose1 grasp1 conf2 traj1)
    (Kin b pose2 grasp2 conf3 traj2)
    (Kin c pose3 grasp3 conf4 traj3)

    ; Motion planning streams (simplified for the example)
    (FreeMotion conf1 traj1 conf2)
    (FreeMotion conf2 traj2 conf3)
    (HoldingMotion conf3 traj3 conf4 a grasp1)
    (HoldingMotion conf4 traj4 conf5 b grasp2)
    (HoldingMotion conf5 traj5 conf6 c grasp3)

    ; Collision-free motion (simplified for the example)
    (CFreeTrajPose traj1 a pose1)
    (CFreeTrajPose traj2 b pose2)
    (CFreeTrajPose traj3 c pose3)
  )

  (:goal
    (and
      (On a b)
      (On b c)
    )
  )
)

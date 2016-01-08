#move_group_test


This package is for testing the hand programmatically.

After running the ur5_barrett_bringup and ur5_barrett_moveit package to test the hand:

rosrun move_group_test move_random

rosrun move_group_test move_up

rosrun move_group_test move_joint

rosrun move_group_test move_cartesian

rosrun move_group_test move_constraint

rosrun move_group_test move_pose

rosrun move_group_test move_pose_attached

rosrun move_group_test move_grasp_simple

The move_grasp_simple is for mimicking a grasp planning motion.(IT is not a grasping test)

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup group("manipulator");
  group.setPlannerId("RRTkConfigDefault");
  
  
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.192;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 0.7;
  group.setPoseTarget(target_pose1);
   
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
  //group.move();
  sleep(2.0);


  moveit_msgs::OrientationConstraint ocm;
  ocm.link_name = "ee_link";
  ocm.header.frame_id = "world";
  ocm.orientation.w = 1.0;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  ocm.weight = 1.0;

  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  group.setPathConstraints(test_constraints);

  robot_state::RobotState start_state(*group.getCurrentState());
 
  geometry_msgs::Pose start_pose2;
  start_pose2.orientation.w = 1.0;
  start_pose2.position.x = 0.25;
  start_pose2.position.y = 0;
  start_pose2.position.z = 1;
  group.setPoseTarget(start_pose2);
  group.move();

 const robot_state::JointModelGroup *joint_model_group =
                start_state.getJointModelGroup(group.getName());
  start_state.setFromIK(joint_model_group, start_pose2);
  group.setStartState(start_state);
  group.setPoseTarget(target_pose1);
  success = group.plan(my_plan);

  group.move();
}

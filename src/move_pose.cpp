#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");
  ros::NodeHandle node_handle;  
 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup group("manipulator");
  //group.setPlannerId("RRTkConfigDefault");
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.25;
  target_pose1.position.y = 0;
  target_pose1.position.z = 1;
  group.setPoseTarget(target_pose1);
  
   ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
  group.move();
  sleep(2.0);
}

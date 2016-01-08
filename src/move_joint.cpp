#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

 
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("manipulator");
  
  std::map<std::string, double> joints;
  joints["shoulder_pan_joint"] = -0.40;
  joints["shoulder_lift_joint"] =  0.27;
  joints["elbow_joint"] =  0.48;
  joints["wrist_1_joint"] =  0.07;
  joints["wrist_2_joint"] =  0.75;
  joints["wrist_3_joint"] = -0.19;

  group.setJointValueTarget(joints);
  
  group.move();

  
}

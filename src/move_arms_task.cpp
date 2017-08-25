#include <ros/ros.h>
#include<moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_joint");
  //ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("arms");
  std::map<std::string, double> joints;
  joints["r_shoulder_pan_joint"] =-1.50;
  joints["r_shoulder_lift_joint"] = 0.0;
  joints["r_upper_arm_roll_joint"] = 0.0;
  joints["r_forearm_roll_joint"] = 0.0;
  joints["r_elbow_flex_joint"] = -1.14;
  joints["r_wrist_flex_joint"] = -1.06;
  joints["r_wrist_roll_joint"] = 0.0;

  joints["l_shoulder_pan_joint"] =1.50;
  joints["l_shoulder_lift_joint"] = 0.0;
  joints["l_upper_arm_roll_joint"] = 0.0;
  joints["l_forearm_roll_joint"] = 0.0;
  joints["l_elbow_flex_joint"] = -1.14;
  joints["l_wrist_flex_joint"] = -1.06;
  joints["l_wrist_roll_joint"] = 0.0;


  group.setJointValueTarget(joints);
  group.move();
  ROS_INFO("Moving By given joint values!");

}

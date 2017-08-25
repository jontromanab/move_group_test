#include <ros/ros.h>
#include<moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_joint");
  //ros::NodeHandle nh;

  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("head");
  std::map<std::string, double> joints;
  joints["head_tilt_joint"] =0.78;
  joints["head_pan_joint"] = 0.0;


  group.setJointValueTarget(joints);
  group.move();
  ROS_INFO("Moving By given joint values!");

}

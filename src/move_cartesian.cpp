#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup group("manipulator");
  group.setPlannerId("RRTkConfigDefault");
  
 Eigen::Affine3d pose = Eigen::Translation3d(0.028, 0.793, 0.390)
                         * Eigen::Quaterniond(-0.014, 0.733, 0.680, -0.010);
  group.setPoseTarget(pose);
  group.move();

  sleep(2.0);
  Eigen::Affine3d pose2 = Eigen::Translation3d(-0.506, 0.000, 0.532)
                         * Eigen::Quaterniond(0.000,-0.644, 0.000, 0.765);
  group.setPoseTarget(pose2);
  group.move();
}

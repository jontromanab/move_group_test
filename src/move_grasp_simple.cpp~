#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup group("manipulator");
  group.setPlannerId("RRTkConfigDefault");
/******************home********************************/
  group.setNamedTarget("home");
  ROS_INFO("Going to home position");
  group.move();

  group.setNamedTarget("up");
  ROS_INFO("Going to UP position");
  group.move();
  sleep(1.0);
/*******************approach********************************/

/*Eigen::Affine3d pose2 = Eigen::Translation3d(-0.506, 0.000, 0.532)
                         * Eigen::Quaterniond(0.000,-0.644, 0.000, 0.765);
  group.setPoseTarget(pose2);
  ROS_INFO("Going to Approach position");
  group.move();*/
 Eigen::Affine3d approach = Eigen::Translation3d(0.606, 0.000, 0.285)
                         * Eigen::AngleAxisd(M_PI, Eigen::Vector3d::UnitX());
  

  ROS_INFO("Going to Approach position");
  group.setPoseTarget(approach);
  group.move() ;
  

/**************pick and retreat *****************************/

Eigen::Affine3d pick = approach.translate(0.25*Eigen::Vector3d::UnitX());
  ROS_INFO("Going to Pick");
  group.setPoseTarget(pick);
  group.move();

Eigen::Affine3d retreat = pick.translate(-0.1*Eigen::Vector3d::UnitX());
  ROS_INFO("Going to Retreat");
  group.setPoseTarget(retreat);
  group.move();

/*****************Inspect**********************************/
std::map<std::string, double> joints;
  joints["shoulder_pan_joint"] = 0.1363963462366371;
  joints["shoulder_lift_joint"] =  0.2109966536389436;
  joints["elbow_joint"] =  -0.9980383708427638;
  joints["wrist_1_joint"] =  -2.3544856100677274;
  joints["wrist_2_joint"] =  -0.13646883251496789;
  joints["wrist_3_joint"] = -1.4810632157125827;
  ROS_INFO("Going to Inspect position");
  group.setJointValueTarget(joints);
  group.move();

/******************Back to home********************************/
  group.setNamedTarget("up");
  ROS_INFO("Going back to UP position");
  group.move();
  sleep(1.0);
}

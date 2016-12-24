#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");

 
  ros::AsyncSpinner spinner(1);
  spinner.start();
  
  moveit::planning_interface::MoveGroup group("manipulator");
  moveit::planning_interface::MoveGroup group2("gripper");
  group.setPlannerId("RRTkConfigDefault");
/******************home********************************/
  

  group.setNamedTarget("up");
  ROS_INFO("Going to UP position");
  group.move();
  sleep(1.0);
/*******************approach********************************/

  
  Eigen::Affine3d approach = Eigen::Translation3d(-0.27, -0.287, 1.1)
                         * Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
  

  ROS_INFO("Going to Approach position");
  group.setPoseTarget(approach);
  group.move() ;
  
  ROS_INFO("opening gripper");
  group2.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.01);
  group2.move();

/**************pick and retreat *****************************/

  Eigen::Affine3d pick = approach.translate(0.1*Eigen::Vector3d::UnitX());
  ROS_INFO("Going to Pick");
  group.setPoseTarget(pick);
  group.move();


/*******************Closing gripper ************************/
  
  /*ROS_INFO("Closing gripper");
  group2.setNamedTarget("close");
  group2.move();*/

  ROS_INFO("grasping");
  group2.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.79);
  group2.move();


/**********************Retreat *****************************/
  Eigen::Affine3d retreat = pick.translate(-0.7*Eigen::Vector3d::UnitX());
  ROS_INFO("Going to Retreat");
  group.setPoseTarget(retreat);
  group.move();

/**********************Place it *****************************/
  Eigen::Affine3d place = Eigen::Translation3d(0.95, -0.486, 1.12)
                         *  Eigen::AngleAxisd(M_PI/2, Eigen::Vector3d::UnitY());
  ROS_INFO("Going to Place");
  group.setPoseTarget(place);
  group.move() ;

/********************Placing ********************************/
  Eigen::Affine3d placing = place.translate(0.06*Eigen::Vector3d::UnitX());
  ROS_INFO("Placing");
  group.setPoseTarget(placing);
  group.move();

/*******************Opening gripper ************************/
  
  //group2.setStartStateToCurrentState ();
  ROS_INFO("Opening gripper");
  group2.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.01);
  group2.move();
  
/**********************Retreat Again*****************************/
  Eigen::Affine3d retreat2 = placing.translate(-0.15*Eigen::Vector3d::UnitX());
  ROS_INFO("Going to Retreat");
  group.setPoseTarget(retreat2);
  group.move();  

/*****************Inspect**********************************/
/*std::map<std::string, double> joints;
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
 
  group2.setJointValueTarget("robotiq_85_left_knuckle_joint", 0.8);
  group2.move();
  
  sleep(1.0);
}

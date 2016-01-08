#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
int main(int argc, char **argv)
{
  ros::init(argc, argv, "lesson_move_group");
  ros::NodeHandle nh;  
  
  ros::AsyncSpinner spinner(1);
  spinner.start();
  

  ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
  ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);
  moveit::planning_interface::MoveGroup group("manipulator");
  //moveit::planning_interface::PlanningSceneInterface planning_scene_interface;
  group.setPlannerId("RRTkConfigDefault");


  //Creating the collision object
  moveit_msgs::CollisionObject collision_object;
  //collision_object.header.frame_id = group.getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "box1";
  collision_object.header.stamp = ros::Time::now();
  collision_object.header.frame_id = "world";
  collision_object.operation = moveit_msgs::CollisionObject::REMOVE;
  pub_co.publish(collision_object);

  /* Define a box to add to the world. */
  collision_object.operation = collision_object.ADD;
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.05;
  primitive.dimensions[1] = 0.05;
  primitive.dimensions[2] = 0.3;

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;
  box_pose.orientation.w = 1.0;
  box_pose.position.x =  0.4;
  box_pose.position.y = -0.829;
  box_pose.position.z =  0.370;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
  

  pub_co.publish(collision_object);

  ROS_INFO("Add an object into the world");
  //planning_scene_interface.addCollisionObjects(collision_objects);

  moveit_msgs::AttachedCollisionObject aco;
  aco.object = collision_object;
  pub_aco.publish(aco);
  
  ROS_INFO("Attach the object to the robot");
  group.attachObject(collision_object.id);
  /* Sleep to give Rviz time to show the object attached (different color). */
  sleep(4.0);

  /* Sleep so we have time to see the object in RViz */
  //sleep(2.0);
  group.setPlanningTime(10.0);
  
  group.setStartState(*group.getCurrentState());


  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.45;
  target_pose1.position.y = 0;
  target_pose1.position.z = 1;
  group.setPoseTarget(target_pose1);
  
   ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
  
  
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);
  group.move();
  ROS_INFO("Detach the object from the robot");
  group.detachObject(collision_object.id);
  /* Sleep to give Rviz time to show the object detached. */
  //sleep(4.0);
  sleep(2.0);

  
}

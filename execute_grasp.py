#!/usr/bin/env python
# -*- coding: utf-8 -*-

import sys
import rospy
import copy
import numpy
import moveit_commander
from moveit_commander import RobotCommander, roscpp_initialize, roscpp_shutdown
from moveit_msgs.msg import *
from trajectory_msgs.msg import JointTrajectoryPoint
from visualization_msgs.msg import Marker, MarkerArray
from geometry_msgs.msg import Pose, Vector3, Point

def quittableInput(s):
  s = raw_input(s + "(q to quit) ")
  if s == 'q': exit(0)


def generateWaypointsGrasp(pose, approach):
  num_waypoints = 3
  dist = [0.12, 0.06, 0.0] #parameter of robotiq gripper.we have to set this
  waypoints = [None] * num_waypoints
  for i in range(0, num_waypoints):
    waypoints[i] = Pose()
    waypoints[i].position.x = pose.position.x - dist[i] * approach.x
    waypoints[i].position.y = pose.position.y - dist[i] * approach.y
    waypoints[i].position.z = pose.position.z - dist[i] * approach.z
    waypoints[i].orientation = pose.orientation
    #print "--------------waypoint", i, "----------"
    #print waypoints[i].position

  return waypoints

def createExtraWaypoint(pose, waypoint):
  way0 = waypoint.position
  way0 = numpy.array ([way0.x, way0.y, way0.z])
  pose_curr = pose.position
  pose_curr = numpy.array([pose_curr.x, pose_curr.y, pose_curr.z])
  way_add = pose_curr + 0.5*(way0 - pose_curr)
  pose = Pose()
  pose.orientation = waypoint.orientation
  pose.position = Point(way_add[0], way_add[1], way_add[2])
  return pose

def generateWayPointsLift(pose, dist):
  waypoints = []
  waypoints.append(pose)
  wpose = geometry_msgs.msg.Pose()
  wpose.orientation.w = 1.0
  wpose.position.x = waypoints[0].position.x
  wpose.position.y = waypoints[0].position.y
  wpose.position.z = waypoints[0].position.z + dist
  waypoints.append(copy.deepcopy(wpose))
  return waypoints

def generateWayPointsVertical(pose, z_target, num_waypoints = 5):
  waypoints = []
  waypoints.append(pose)
  wpose = copy.deepcopy(waypoints[0])
  dist_z = z_target - pose.position.z
  for i in range(0, num_waypoints):
    wpose.position.z +=dist_z / num_waypoints
    print wpose.position.x, wpose.position.y, wpose.position.z
    waypoints.append(copy.deepcopy(wpose))

  return waypoints

def planAndExecuteWaypoints(group, waypoints, speeds = [0.6]):
  (plan, fraction) = group.compute_cartesian_path(waypoints, 0.005, 0.0)
  print "fraction:", fraction
  print "=================moving the arm======================"
  group.execute(plan)

def planAndExecuteJointTarget(group, joints, speed = 3.0):
  group.set_joint_value_target(joints)
  plan = group.plan()
  new_traj = changeTrajectorySpeed(plan, speed)
  group.execute(new_traj)
  return True

def changeTrajectorySpeed(traj, speed):
  traj_new = RobotTrajectory()
  traj_new = traj
  for p in traj_new.joint_trajectory.points:
    p.time_from_start /= speed

  return traj_new


def createTargetMarker(pose, axis):
  diam = 0.01
  alpha = 1.0
  marker = Marker()
  marker.type = Marker.ARROW
  marker.id = 0
  marker.header.frame_id = "/world"
  marker.header.stamp = rospy.get_rostime()
  marker.lifetime = rospy.Duration.from_sec(60.0)
  marker.action = Marker.ADD
  marker.scale.x = diam #Shaft diameter
  marker.scale.y = 0.02 # head diameter
  marker.scale.z = 0.03 # head length
  marker.color.r = 0.0
  marker.color.g = 0.0
  marker.color.b = 1.0
  marker.color.a = alpha
  p = Point()
  q = Point()
  p.x = pose.position.x
  p.y = pose.position.y
  p.z = pose.position.z
  q.x = p.x - 0.15*axis.x
  q.y = p.y - 0.15*axis.y
  q.z = p.z - 0.15*axis.z
  marker.points.append(q)
  marker.points.append(p)
  return marker

def createPointMarker(waypoint, id):
  marker = Marker()
  marker.type = Marker.SPHERE
  marker.id = id
  marker.ns = "waypoints"
  marker.header.frame_id = "/world"
  marker.header.stamp = rospy.get_rostime()
  marker.lifetime = rospy.Duration.from_sec(30.0)
  marker.action = Marker.ADD
  marker.scale.x = 0.015
  marker.scale.y = 0.015
  marker.scale.z = 0.015
  marker.color.r = 1.0
  marker.color.g = 1.0
  marker.color.b = 0.0
  marker.color.a = 1.0
  marker.pose.position = waypoint.position
  marker.pose.orientation = waypoint.orientation
  return marker

def createWayPointMarkers(waypoints):
  marker_array = MarkerArray()
  for i in range(0, len(waypoints)):
    marker_array.markers.append(createPointMarker(waypoints[i], i))
  return marker_array


def grasping():
    #Initialize MoveIT and ROS
    roscpp_initialize(sys.argv)
    rospy.init_node('grasp_execution', anonymous=True)

    # create publisher to visualize the grasp target
    grasp_pub = rospy.Publisher('target', Marker, queue_size=1)

    # create publisher to visualize the waypoints
    waypoints_pub = rospy.Publisher('waypoints', MarkerArray, queue_size=1)

    robot = RobotCommander()

    print robot.get_group_names()

    arm = robot.manipulator
    hand = robot.gripper

    arm.set_planner_id("RRTkConfigDefault")
    print "============ Reference frame: %s" % arm.get_planning_frame()
    print "============ Reference frame: %s" % arm.get_end_effector_link()
    print "============ Printing joint positions"
    print arm.get_joints()
    print arm.get_current_joint_values()
    #print "============ Printing current robot hand pose"
    pose_curr = arm.get_current_pose().pose
    #print pose_curr.position.x, pose_curr.position.y, pose_curr.position.z, pose_curr.orientation.x, pose_curr.orientation.y, pose_curr.orientation.z, pose_curr.orientation.w

    print "============Closing the hand"
    hand.set_named_target("close")
    hand.go()
    rospy.sleep(0.1)

    #if EE is too low, take it up to 1.20

    if pose_curr.position.z < 0.20:
      planAndExecuteWaypoints(arm, generateWayPointsVertical(arm.get_current_pose().pose, 1.20), [5.0])

    joints_nominal = [0.8970585591545106, -1.7025583906566055, 1.92330208699848631, -0.221234222444048, 1.982689962060214, 3.1406743707619498]
    joints_place = [-1.4602422404003672, -1.6093748327068855, 1.9663897429378636, -1.9869793485660718, -1.531336697160997, -1.6658099841334861]
    #print arm.get_current_joint_values()
    #quittableInput("Hit Enter to start grasp process (q to quit) ")


    #Defining manual grasp msg with Pose and Approach vector. We have to take it from a callable service that will send single/multiple grasp msg
    grasp_pose = Pose()
    grasp_axis = Vector3()
    grasp_pose.position.x = 0.95
    grasp_pose.position.y = -0.2
    grasp_pose.position.z = 1.0
    grasp_pose.orientation.w = 1.0

    grasp_axis.x = 0.0
    grasp_axis.y = 0.0
    grasp_axis.z = -1.0

    while not rospy.is_shutdown():
      planAndExecuteJointTarget(arm, joints_nominal, 1.6)
      print "=====moving the hand to joints_nominal position"
      rospy.sleep(0.1)

      hand.set_named_target("open")
      print "===opening the hand====="
      hand.go()
      rospy.sleep(0.1)

      quittableInput("Hit Enter to start grasp process (q to quit) ")
      print "starting........................................................."

      print "Publishing grasp target as arrow"
      grasp_pub.publish(createTargetMarker(grasp_pose, grasp_axis))
      rospy.sleep(0.1)

      print "Creating grasp waypoints"
      wayPoints = generateWaypointsGrasp(grasp_pose, grasp_axis)
      print "Generated waypoints"

      print "Creating extra waypoint"
      wayPoints.insert(0, createExtraWaypoint(arm.get_current_pose().pose, wayPoints[0]))

      print "Publishing waypoints"
      waypoints_pub.publish(createWayPointMarkers(wayPoints))
      rospy.sleep(0.1)

      print "adding current robot pose to waypoints"
      wayPoints.insert(0, arm.get_current_pose().pose)

      print "executing grasp waypoints"
      #print wayPoints
      planAndExecuteWaypoints(arm, wayPoints, [10.0])

      hand.set_named_target("close")
      print "===Grasping the object====="
      hand.go()
      rospy.sleep(0.1)

      print "===Lifting object=============="
      #planAndExecuteWaypoints(arm, generateWayPointsVertical(arm.get_current_pose().pose, 0.9), [5.0])
      #rospy.sleep(0.1)
      planAndExecuteWaypoints(arm, generateWayPointsLift(arm.get_current_pose().pose, 0.1), [5.0])
      rospy.sleep(0.1)

      planAndExecuteJointTarget(arm,joints_place, 1.6)
      print "=====moving the hand to place ======"
      rospy.sleep(0.1)

      hand.set_named_target("open")
      print "===Releasing the object====="
      hand.go()
      rospy.sleep(0.1)

      #print "Hello"


if __name__=='__main__':
  try:
    grasping()
  except rospy.ROSInterruptException:
    pass


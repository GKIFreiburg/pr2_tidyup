#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_tools')
import rospy
import sys, traceback
from tidyup_msgs import msg
from tidyup_msgs import srv
from pr2_python.planning_scene_interface import *
from arm_navigation_msgs.msg import CollisionObject, Shape, RobotState
from geometry_msgs.msg import Point, Pose, PoseStamped
from pr2_tasks.pickplace import PickPlace
from pr2_tasks.pickplace_definitions import PlaceGoal, PlaceResult
from visualization_msgs.msg import Marker, MarkerArray
from pr2_python import visualization_tools as vt
from pr2_python.exceptions import ManipulationError
import numpy as np

if __name__ == '__main__':
  try:
    rospy.init_node('test_putdown_feasablity', anonymous=True)
    vispub = rospy.Publisher('/tidyup/visualization', MarkerArray)
    rospy.loginfo('setting up planning scene...')
    psi = get_planning_scene_interface()
    
    # create cup
#    object = CollisionObject()
#    object.id = 'cup_test'
#    shape = Shape()
#    shape.type = shape.BOX
#    shape.dimensions.append(0.05)
#    shape.dimensions.append(0.05)
#    shape.dimensions.append(0.12)
#    object.shapes.append(shape)
#    pose = Pose()
#    pose.position.x = 0.032
#    pose.position.y = 0.015
#    pose.position.z = 0.0
#    pose.orientation.x = 0.707
#    pose.orientation.y = -0.106
#    pose.orientation.z = -0.690
#    pose.orientation.w = 0.105
#    object.poses.append(pose)
#    object.header.frame_id = 'l_gripper_r_finger_tip_link'
#    psi.add_object(object)
#    if not psi.attach_object_to_gripper('left_arm', 'cup_test'):
#      rospy.logerr('attach object failed.')

    #create table
#    object = CollisionObject()
#    object.id = 'table_test'
#    shape = Shape()
#    shape.type = shape.BOX
#    shape.dimensions.append(0.5)
#    shape.dimensions.append(1)
#    shape.dimensions.append(0.1)
#    object.shapes.append(shape)
#    pose = Pose()
#    pose.position.x = 0.5
#    pose.position.y = 0.0
#    pose.position.z = 0.55
#    pose.orientation.x = 0
#    pose.orientation.y = 0
#    pose.orientation.z = 0
#    pose.orientation.w = 1
#    object.poses.append(pose)
#    object.header.frame_id = '/base_link'
#    psi.add_object(object)
    
    # set robot state
    left_arm_joints = [2.1, 1.26, 1.8, -1.9, -3.5, -1.8, np.pi/2.0]
    right_arm_joints = [-2.1, 1.26, -1.8, -1.9, 3.5, -1.8, np.pi/2.0]
    pose = Pose()
    pose.position.x = -0.6
    pose.position.y = 0.0
    pose.position.z = 0.0510018
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    robot_state = psi.get_robot_state()
    robot_state.multi_dof_joint_state.poses[0] = pose
    r = robot_state.joint_state.name.index('r_shoulder_pan_joint')
    l = robot_state.joint_state.name.index('l_shoulder_pan_joint')
    position = list(robot_state.joint_state.position)
    for offset in range(len(right_arm_joints)):
      position[r + offset] = right_arm_joints[offset]
      position[l + offset] = left_arm_joints[offset]
    robot_state.joint_state.position = position
    rospy.loginfo('setting robot state...')
    #psi.set_robot_state(robot_state)
    
    pickplace = PickPlace()
    rospy.loginfo('pickplace initialized.')
    # create place goal
    place_locations = list()
    poseStamped = PoseStamped()
    poseStamped.header.frame_id = '/base_link'
    pose = poseStamped.pose
    pose.position.x = 0.5
    pose.position.y = 0.0
    pose.position.z = 0.752
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    #place_locations.append(poseStamped)
    for y in range(-5, 5):
      pose.position.y = y * 0.1
      place_locations.append(copy.deepcopy(poseStamped))
    place_goal = PlaceGoal('left_arm', place_locations, collision_object_name='cup_2', collision_support_surface_name= 'table1_loc1')
    print(place_goal.place_locations)
    place_goal.only_perform_feasibility_test = True
    place_markers = MarkerArray()
    for i, pose in enumerate(place_goal.place_locations):
      place_markers.markers.append(vt.marker_at(pose_stamped=pose, ns="putdown_test", mid=i, mtype=Marker.ARROW))
    vispub.publish(place_markers)
    try:
      place_result = pickplace.place(place_goal)
    except ManipulationError, err:
      rospy.loginfo(err)
    rospy.loginfo('done.')
  except:
    traceback.print_exc(file=sys.stdout)

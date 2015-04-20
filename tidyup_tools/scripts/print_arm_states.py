#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_tools')
import rospy
import sys
import copy
import numpy as np
from arm_navigation_msgs.msg import RobotState, PlanningScene
from geometry_msgs.msg import Point, Pose, PoseStamped
from tidyup_arm_services.planning_scene_sync import PlanningSceneSync
from pr2_python.planning_scene_interface import get_planning_scene_interface

if __name__ == '__main__':
  rospy.init_node('print_arm_angles')
  psi = get_planning_scene_interface()
  pss = PlanningSceneSync()
  pss._latest_planning_scene.robot_state = psi.get_robot_state()

  left_arm_joints = [2.1, 1.26, 1.8, -1.9, -3.5, -1.8, np.pi/2.0]
  right_arm_joints = [-2.1, 1.26, -1.8, -1.9, 3.5, -1.8, np.pi/2.0]
  loop_rate = rospy.Rate(0.2)
#  rospy.loginfo('checking current robot state for collisions...')
  while not rospy.is_shutdown():
    robot_state = pss._latest_planning_scene.robot_state
    r = robot_state.joint_state.name.index('r_shoulder_pan_joint')
    l = robot_state.joint_state.name.index('l_shoulder_pan_joint')
    for offset in range(len(right_arm_joints)):
      right_arm_joints[offset] = robot_state.joint_state.position[r + offset]
      left_arm_joints[offset] = robot_state.joint_state.position[l + offset]
    rospy.loginfo('right_arm: \n{0}'.format(right_arm_joints))
    rospy.loginfo('left_arm: \n{0}'.format(left_arm_joints))
    loop_rate.sleep()

  

#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_tools')
import rospy
import sys
import copy
import numpy as np

from tidyup_utils.geometry_pose_loader import read_pose_stamped_file
from pr2_python.base import Base
from pr2_python.arm_planner import ArmPlanner

def check_robot_locations(location_file_name):
  # get locations
  named_poses = read_pose_stamped_file(location_file_name)
  # get robot state
  base = Base()
  while not rospy.is_shutdown():
      with base._js_lock:
          if base._last_state is not None:
              robot_state = copy.deepcopy(base._last_state)
              break
      rospy.loginfo("Waiting for joint state")
      rospy.sleep(1.0)
      
  # load arm at side config
  side_joint_positions = {}
  arm_planners = {}
  for arm in ['left_arm', 'right_arm']:
    arm_planners[arm] = ArmPlanner(arm)
    side_joint_positions[arm] = rospy.get_param('/arm_configurations/side_tuck/position/'+arm)
    # replace in state
    arm_planners[arm].set_joint_positions_in_robot_state(side_joint_positions[arm], robot_state)
  robot_states = []
  for name, pose_stamped in named_poses.items():
    robot_state.multi_dof_joint_state.poses[0] = copy.deepcopy(pose_stamped.pose)
    robot_states.append(copy.deepcopy(robot_state))
    
  # call collision service
  error_codes = base._check_pose_srv(robot_states=robot_states).error_codes
  
  # evaluate response
  collision_free = True
  for i in range(len(named_poses)):
    error_code = error_codes[i]
    named_pose = named_poses[i]
  #for error_code, named_pose in error_codes, named_poses:
    if error_code.val != error_code.SUCCESS:
      collision_free = False
      rospy.logwarn('location {0} has returned collision error: {1}'.format(named_pose['name'], error_code.val))
  if collision_free: rospy.loginfo('all locations are collision free.')
  return collision_free

if __name__ == '__main__':
  rospy.init_node('check_robot_locations', anonymous=True)
  if len(sys.argv) > 1:
    check_robot_locations(sys.argv[1])
  else:
    rospy.loginfo('checking current robot state for collisions...')
    base = Base()
    if base.check_base_pose(base.get_current_pose_stamped()):
      rospy.loginfo('current state is collision free.')
    else:
      rospy.logwarn('current state is in collision.')

  

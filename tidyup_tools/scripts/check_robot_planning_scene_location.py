#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_tools')
import rospy
import sys
import copy

from pr2_python.base import Base
from tidyup_arm_services.planning_scene_sync import PlanningSceneSync

if __name__ == '__main__':
  rospy.init_node('check_robot_locations')
  loop_rate = rospy.Rate(0.2)
  pss = PlanningSceneSync()
  rospy.loginfo('checking current robot state for collisions...')
  base = Base()
  while not rospy.is_shutdown():
    error_codes = base._check_pose_srv(robot_states=[copy.deepcopy(pss._latest_planning_scene.robot_state)]).error_codes
    if not len(error_codes) > 0:
        raise ex.AllYourBasePosesAreBelongToUs((x,y,z))
    if error_codes[0].val == error_codes[0].SUCCESS:
      rospy.loginfo('current state is collision free.')
    else:
      rospy.logwarn('current state is in collision.')
    loop_rate.sleep()

  

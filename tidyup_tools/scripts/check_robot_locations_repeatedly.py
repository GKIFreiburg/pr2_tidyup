#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_tools')
import rospy
import sys
import copy

from pr2_python.base import Base

if __name__ == '__main__':
  rospy.init_node('check_robot_locations', anonymous=True)
  loop_rate = rospy.Rate(1.0)
  rospy.loginfo('checking current robot state for collisions...')
  base = Base()
  while not rospy.is_shutdown():
    if base.check_base_pose(base.get_current_pose_stamped()):
      rospy.loginfo('current state is collision free.')
    else:
      rospy.logwarn('current state is in collision.')
    loop_rate.sleep()

  

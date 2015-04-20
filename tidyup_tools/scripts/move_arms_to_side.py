#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_tools')
import rospy
import sys, traceback
from tidyup_msgs import msg
import actionlib

if __name__ == '__main__':
  try:
    rospy.init_node('arms_to_side', anonymous=True)
    rospy.loginfo('moving arms to side.')
    arm_service = actionlib.SimpleActionClient('/tidyup/side_position_action', msg.ArmToSideAction)
    arm_service.wait_for_server()
    rospy.loginfo('action server found.')
    side_goal = msg.ArmToSideGoal()
    side_goal.right_arm = True
    arm_service.send_goal_and_wait(side_goal)
    side_goal.right_arm = False
    side_goal.left_arm = True
    arm_service.send_goal_and_wait(side_goal)
    rospy.loginfo('done.')
  except:
    traceback.print_exc(file=sys.stdout)

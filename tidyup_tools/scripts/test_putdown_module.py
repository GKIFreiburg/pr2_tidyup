#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_tools')
import rospy
import sys, traceback
from tidyup_msgs import msg
from tidyup_msgs import srv
from pr2_python.planning_scene_interface import *
from geometry_msgs.msg import Point, Pose, PoseStamped

if __name__ == '__main__':
  try:
    rospy.init_node('test_putdown_module', anonymous=True)
    rospy.loginfo('setting up planning scene...')
    psi = get_planning_scene_interface()
    psi.reset()
    # attach object to gripper
    object_name = 'bowl_1'
#    #object = psi.collision_object(object_name)
#    pose = Pose()
#    pose.position.x = 0.1
#    pose.position.y = 0.05
#    pose.position.z = 0.0
#    pose.orientation.x = 0.707
#    pose.orientation.y = -0.106
#    pose.orientation.z = -0.690
#    pose.orientation.w = 0.105
#    #object.poses[0] = pose
#    #object.header.frame_id = 'l_gripper_r_finger_tip_link'
#    #psi.add_object(object)
#    #if not psi.attach_object_to_gripper('left_arm', object_name):
#    #  rospy.logerr('attach object failed.')
#
#    # set robot pose
#    pose = Pose()
#    #table1_loc1_room1  /map 0.120862 0.698891 0.0510018 0 0 0.251132 0.967953
#    #table1_loc1_room1  /map 15.84 18.4088 0.0531112 0 0 0.999299 0.037442
##    pose.position.x = 15.84
##    pose.position.y = 18.40
##    pose.position.z = 0.0510018
##    pose.orientation.x = 0
##    pose.orientation.y = 0
##    pose.orientation.z = 1
##    pose.orientation.w = 0
#    #table2_loc4_room2 1348583312 347842222 /map 20.0308 17.9577 0.0500652 0 0 -0.723473 0.690353
#    pose.position.x = 20.0308
#    pose.position.y = 17.9577
#    pose.position.z = 0.0510018
#    pose.orientation.x = 0
#    pose.orientation.y = 0
#    pose.orientation.z = -0.707
#    pose.orientation.w = 0.707
#    robot_state = psi.get_robot_state()
#    robot_state.multi_dof_joint_state.poses[0] = pose
#    converted = list(robot_state.joint_state.position)
#    converted[0] = pose.position.x
#    converted[1] = pose.position.y;
#    converted[2] = pose.position.z;
#    converted[3] = pose.orientation.x;
#    converted[4] = pose.orientation.y;
#    converted[5] = pose.orientation.z;
#    converted[6] = pose.orientation.w;
#    robot_state.joint_state.position = converted
#    rospy.loginfo('setting robot state...')
#    #psi.set_robot_state(robot_state)
#    rospy.loginfo(psi.get_robot_state())
#    
    putdwon_service = rospy.ServiceProxy('/tidyup/request_putdown_pose', srv.GetPutdownPose)
    putdwon_service.wait_for_service()
    rospy.loginfo('connected to service.')
    request = srv.GetPutdownPoseRequest()
    request.arm = 'right_arm'
    request.static_object = 'table1_loc1'
    request.putdown_object = object_name
    response = putdwon_service.call(request)
    #psi.reset()
    rospy.loginfo('done.')
  except:
    traceback.print_exc(file=sys.stdout)

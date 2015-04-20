#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_tools')
import rospy
import sys, traceback
from tidyup_msgs import msg
from tidyup_msgs import srv
from pr2_python.planning_scene_interface import *
from pr2_python.world_interface import WorldInterface 
from geometry_msgs.msg import Point, Pose, PoseStamped
from arm_navigation_msgs.msg import AttachedCollisionObject, CollisionObject, Shape

if __name__ == '__main__':
  try:
    rospy.init_node('attaching sponge', anonymous=True)
    psi = get_planning_scene_interface()
    psi.reset()
    wi = WorldInterface()
    
    rospy.loginfo('creating object')
    object_name = 'sponge'
    stamped = PoseStamped()
    pose = Pose()
    pose.position.x = 0.25
    pose.position.y = 0.0
    pose.position.z = 0.33
    pose.orientation.x = 0
    pose.orientation.y = 0
    pose.orientation.z = 0
    pose.orientation.w = 1
    stamped.pose = pose
    stamped.header.frame_id = 'base_link'
    wi.add_collision_box(stamped, [0.07, 0.15, 0.07], object_name)
    
    rospy.loginfo('attaching object')
    attached = AttachedCollisionObject()
    attached.object = wi.collision_object(object_name)
    attached.link_name = 'base_link'
    attached.touch_links.append('base_link')
    attached.object.operation.operation = attached.object.operation.ATTACH_AND_REMOVE_AS_OBJECT
    wi._publish(attached, wi._attached_object_pub)
    
    psi.reset()
    rospy.loginfo('done.')
  except:
    traceback.print_exc(file=sys.stdout)

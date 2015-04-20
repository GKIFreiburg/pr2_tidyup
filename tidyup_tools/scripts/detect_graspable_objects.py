#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_tools')
import rospy
import sys, traceback
from tidyup_msgs import msg
from tidyup_msgs import srv

if __name__ == '__main__':
  try:
    rospy.init_node('detect_graspable_objects', anonymous=True)
    rospy.loginfo('detecting graspable objects.')
    detect_service = rospy.ServiceProxy('/tidyup/detect_objects', srv.DetectGraspableObjects)
    detect_service.wait_for_service()
    rospy.loginfo('connected to service.')
    request = srv.DetectGraspableObjectsRequest()
    request.static_object = 'test_table'
    response = detect_service.call(request)
    if not response.objects:
      rospy.loginfo('no objects in field of view.')
    else:
      rospy.loginfo('objects in field of view:')
      for object in response.objects:
        rospy.loginfo(' {0}'.format(object.name))
    rospy.loginfo('done.')
  except:
    traceback.print_exc(file=sys.stdout)

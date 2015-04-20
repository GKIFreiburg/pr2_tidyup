#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_tools')
import rospy
import sys, traceback
from tidyup_msgs import msg
from tidyup_msgs import srv
from pr2_python.planning_scene_interface import *
from pr2_python import head
from geometry_msgs.msg import Point, Pose, PoseStamped
from pr2_tasks.tableware_detection import TablewareDetection, TablewareDetectionResult, DetectionError

def _detect_table_callback(request):
  # run table detection 
  rospy.loginfo('request: detect table at current pose.')
  psi.reset()
  head.look_at_relative_point(0.8, 0.0, 0.7)
  rospy.loginfo("Calling table detection.")
  try:
    # det is a tableware_detection.TablewareDetectionResult
    det = detector.detect_objects(add_objects_to_map=False, add_table_to_map=True, reset_collision_objects=True)
  except DetectionError, last_ex:
    rospy.logerr('object detection FAILED')
    return False #{ 'objects': list()}

  # get table bounding box
  z_max = 0
  for point in det.table.convex_hull.vertices:
    if point.z > z_max:
      z_max = point.z
  
  # send results
  print det.table
  det.table.pose.pose.position.x += 0.5 * (det.table.x_max + det.table.x_min)
  det.table.pose.pose.position.y += 0.5 * (det.table.y_max + det.table.y_min)
  #det.table.pose.pose.position.z += 0.5 * (det.table.z_max + det.table.z_min)
  pose_stamped = psi.transform_pose_stamped('/map', det.table.pose)   
  size = Point()
  size.x = det.table.x_max - det.table.x_min
  size.y = det.table.y_max - det.table.y_min
  size.z = 0.02
  #pose_stamped.pose.position.z = z_max - 0.5 * size.z 
  psi.reset()
  return {'pose': pose_stamped, 'bounding_box_size': size} 


if __name__ == '__main__':
  try:
    rospy.init_node('record_table_service')
    detector = TablewareDetection()
    head = head.Head()
    psi = get_planning_scene_interface()
    # table detection service 
    tableDetectionService = rospy.Service('/tidyup/detect_table', srv.DetectTable, _detect_table_callback)
    rospy.spin()
  except:
    traceback.print_exc(file=sys.stdout)
 

#! /usr/bin/env python

import roslib; roslib.load_manifest('tidyup_utils')
import rospy

from geometry_msgs import msg as geometry

def read_pose_stamped_file(file_name):
  try:
    data_file = open(file_name, 'r')
  except:
    return {}
  poses_data = data_file.readlines()
  data_file.close()
  named_poses = dict()
  for object in poses_data:
    object = object.strip()
    if len(object) == 0 or object.startswith('#'):
      continue
    print object
    pose = geometry.PoseStamped()
    [name, secs, nsecs, frame_id, x, y, z, qx, qy, qz, qw] = object.split()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
    named_poses[name]=pose
  return named_poses

def read_table_file(file_name):
  try:
    data_file = open(file_name, 'r')
  except:
    return []
  poses_data = data_file.readlines()
  data_file.close()
  tables = list()
  for object in poses_data:
    object = object.strip()
    if len(object) == 0 or object.startswith('#'):
      continue
    print object
    pose = geometry.PoseStamped()
    [name, secs, nsecs, frame_id, x, y, z, qx, qy, qz, qw, size_x, size_y, size_z] = object.split()
    pose.header.frame_id = frame_id
    pose.pose.position.x = float(x)
    pose.pose.position.y = float(y)
    pose.pose.position.z = float(z)
    pose.pose.orientation.x = float(qx)
    pose.pose.orientation.y = float(qy)
    pose.pose.orientation.z = float(qz)
    pose.pose.orientation.w = float(qw)
    tables.append([pose, [float(size_x), float(size_y), float(size_z)], name])
  return tables




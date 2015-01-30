cmake_minimum_required(VERSION 2.8)
project(tidyup_msgs)
 
find_package(catkin REQUIRED COMPONENTS
roscpp
rospy
tf
genmsg
actionlib
actionlib_msgs
geometry_msgs
moveit_msgs
)

add_message_files(
	FILES
    GraspableObject.msg
    WipeGoal.msg
)

add_service_files(
	FILES
    ArmsAtSide.srv
    DetectDoorState.srv
    DetectExpectedGraspableObjects.srv
    DetectGraspableObjects.srv
    DetectTable.srv
    DoorState.srv
#    GetInspectPose.srv
#    GetPutdownPose.srv
    RequestObjectsGraspability.srv
)

add_action_files(
   FILES
#   ArmToInspectObject.action
   ArmToSide.action
#   GraspObject.action
#   OpenDoor.action
#   PlaceObject.action
#   PostGraspPosition.action
)

generate_messages(
   DEPENDENCIES
   actionlib_msgs
   geometry_msgs
   std_msgs  # Or other packages containing msgs
)

catkin_package(
)


cmake_minimum_required(VERSION 2.8)
project(tidyup_utils)
 
find_package(catkin REQUIRED COMPONENTS
  roscpp
  rospy
  tidyup_msgs
  moveit_core
  moveit_msgs
  moveit_ros_planning
  moveit_ros_planning_interface
  geometry_msgs
  tf
  tf_conversions
)

catkin_package(
	INCLUDE_DIRS include
	LIBRARIES tidyup_utils
)

find_package(Qt4 REQUIRED)
set( QT_USE_QTGUI TRUE )
#set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

QT4_WRAP_UI(uis_h src/geometryPosesCreatorGui.ui)
# include this for ui_h
include_directories(
  ${catkin_INCLUDE_DIRS}
  ${CMAKE_CURRENT_BINARY_DIR}
  include
)

set(qt_srcs
      src/geometryPosesCreatorGui.cpp)
set(qt_hdrs
      src/geometryPosesCreatorGui.h)

qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})

add_library(tidyup_utils 
    src/stringutil.cpp
    src/planning_scene_interface.cpp
#    src/planning_scene_msg_interface.cpp
    src/transformer.cpp
    src/hand_description.cpp
    src/arm_state.cpp
    src/geometryPoses.cpp
    src/get_pose_stamped_from_param.cpp
)

add_dependencies(tidyup_utils tidyup_msgs_generate_messages_cpp)

target_link_libraries(tidyup_utils
  ${catkin_LIBRARIES}
)

add_executable(test_geometryPoses test/test_geometryPoses.cpp)
target_link_libraries(test_geometryPoses ${PROJECT_NAME} ${catkin_LIBRARIES})

#add_executable(test_planning_scene_msg_interface test/test_planning_scene_msg_interface.cpp)
#target_link_libraries(test_planning_scene_msg_interface ${PROJECT_NAME} ${catkin_LIBRARIES})

add_executable(geometryPosesCreatorGui src/geometryPosesCreatorGuiMain.cpp
    ${uis_h} ${qt_srcs} ${qt_moc_srcs})
target_link_libraries(geometryPosesCreatorGui ${QT_LIBRARIES} ${catkin_LIBRARIES})

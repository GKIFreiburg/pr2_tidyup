cmake_minimum_required(VERSION 2.4.6)
include($ENV{ROS_ROOT}/core/rosbuild/rosbuild.cmake)

# Set the build type.  Options are:
#  Coverage       : w/ debug symbols, w/o optimization, w/ code-coverage
#  Debug          : w/ debug symbols, w/o optimization
#  Release        : w/o debug symbols, w/ optimization
#  RelWithDebInfo : w/ debug symbols, w/ optimization
#  MinSizeRel     : w/o debug symbols, w/ optimization, stripped binaries
#set(ROS_BUILD_TYPE RelWithDebInfo)

rosbuild_init()

#set the default path for built executables to the "bin" directory
set(EXECUTABLE_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/bin)
#set the default path for built libraries to the "lib" directory
set(LIBRARY_OUTPUT_PATH ${PROJECT_SOURCE_DIR}/lib)

find_package(Qt4 REQUIRED)
set( QT_USE_QTGUI TRUE )
set( QT_USE_QTOPENGL TRUE )
set( QT_USE_QTXML TRUE )
include(${QT_USE_FILE})

ADD_DEFINITIONS(-DQT_NO_KEYWORDS)

QT4_WRAP_UI(uis_h src/tidyupTeachingGui.ui)
# include this for ui_h
include_directories(${CMAKE_CURRENT_BINARY_DIR})

set(qt_srcs
      src/tidyupTeachingGui.cpp)
set(qt_hdrs
      src/tidyupTeachingGui.h)

qt4_automoc(${qt_srcs})
QT4_WRAP_CPP(qt_moc_srcs ${qt_hdrs})

rosbuild_add_executable(plan_sbpl_to_location 
    src/test_nav_sbpl.cpp)
rosbuild_add_executable(plan_to_location_planning_scene 
    src/test_nav_planning_scene.cpp)
rosbuild_add_executable(test_planning_scene_interface 
    src/test_planning_scene_interface.cpp)

target_link_libraries(plan_to_location_planning_scene planner_modules_pr2)

rosbuild_add_executable(tidyupTeachingGui src/tidyupTeachingGuiMain.cpp
    ${uis_h} ${qt_srcs} ${qt_moc_srcs})
target_link_libraries(tidyupTeachingGui ${QT_LIBRARIES})

rosbuild_add_executable(octree_to_collision_space src/octree_to_collision_space.cpp)
rosbuild_add_executable(sbpl_config_writer src/sbplConfigWriter.cpp)

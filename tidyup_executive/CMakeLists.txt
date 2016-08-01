cmake_minimum_required(VERSION 2.8.3)
project(tidyup_executive)

## Find catkin macros and libraries
## if COMPONENTS list like find_package(catkin REQUIRED COMPONENTS xyz)
## is used, also find other catkin packages
find_package(catkin REQUIRED
roscpp
geometry_msgs
sensor_msgs
object_manipulation_actions
tidyup_state_creators
#planner_modules_pr2
#continual_planning_server
continual_planning_executive
continual_planning_monitor
tfd_modules
gki_3dnav_planner
gki_global_localization
screenrun
)

###################################
## catkin specific configuration ##
###################################
## The catkin_package macro generates cmake config files for your package
## Declare things to be passed to dependent projects
## INCLUDE_DIRS: uncomment this if you package contains header files
## LIBRARIES: libraries you create in this project that dependent projects also need
## CATKIN_DEPENDS: catkin_packages dependent projects also need
## DEPENDS: system dependencies of this project that dependent projects also need
catkin_package(
#  INCLUDE_DIRS include
#  LIBRARIES pick
#  CATKIN_DEPENDS other_catkin_pkg
#  DEPENDS system_lib
)


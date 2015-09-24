#!/bin/bash

### 4 steps are taken out in order to execute an experiment
# 1.) start simulation: gazebo, rviz, localization, ...
# 2.) load location poses and objects
# 3.) execute arms_to_side to verify that everything works fine
# 4.) run planner
###

if [ -z "$1" ]; then
	echo "Usage: ./script.sh <config_pkg> <num_locations> <num_objects>"
	exit 1
fi

if [ -z "$2" ]; then
	echo "Usage: ./script.sh <config_pkg> <num_locations> <num_objects>"
	exit 1
fi

if [ -z "$3" ]; then
	echo "Usage: ./script.sh <config_pkg> <num_locations> <num_objects>"
	exit 1
fi

config_pkg=$1
num_locations=$2
num_objects=$3

### Step 1
export ROBOT=sim
#nohup roslaunch single_room_objects start_simulation.launch output:="log" &
roslaunch $config_pkg start_simulation.launch output:="log" &
PID_simulation=$!
echo $PID_simulation

# wait 90 seconds to start gazebo, etc and print countdown
seconds=90; date1=$((`date +%s` + $seconds)); 
while [ "$date1" -ne `date +%s` ]; do 
  echo -ne "$(date -u --date @$(($date1 - `date +%s` )) +%H:%M:%S)\r"; 
done

echo "90 seconds are over"

### Step 2
rosrun experiments_evaluation setLocationsAndObjects $config_pkg $num_locations $num_objects
if [ $? != 0 ]; then
	echo "ERROR WHILE SETTING EXPERIMENT ENVIRONMENT UP"
	kill -INT $PID_simulation
	exit 1
else 
	echo "EXPERIMENT ENVIRONMENT SET UP"
fi

### Step 3
rosrun arms_to_side arms_to_side
if [ $? != 0 ]; then
	echo "ERROR IN MOVE_GROUP"
	kill -INT $PID_simulation
	exit 1
else 
	echo "EVERYTHING WORKS - start planner"
fi

### Step 4
roslaunch tidyup_executive continual_planning.launch config_pkg:=$(rospack find $config_pkg) start_paused:=false gui:=true
if [ $? != 0 ]; then
	echo "ERROR IN PLANNER"
else 
	echo "SUCCESSFUL EXECUTED TASK"
fi

# WAY TO GO
kill -INT $PID_simulation


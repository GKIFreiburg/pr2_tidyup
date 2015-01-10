/*
 * test_grasp_block.cpp
 *
 *  Created on: Dec 15, 2014
 *      Author: luc
 */

//#include <moveit_simple_grasps/moveit_blocks.h>
#include <grasping/moveit_blocks.h> // in pkg grasping

int main(int argc, char **argv)
{
  ros::init(argc, argv, "test_grasp_block");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Load grasp data specific to our robot
  ros::NodeHandle nh("~"); // private namespace

  nh.setParam("ee_group_name", "left_gripper");
  nh.setParam("planning_group_name", "left_arm");

  moveit_simple_grasps::MoveItBlocks move_it_blocks;

}



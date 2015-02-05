#include <ros/ros.h>
#include "tidyup/arms_at_side.h"
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "tidyup");
  ros::NodeHandle nhPriv("~");

  		moveit::planning_interface::MoveGroup right_arm_group("right_arm");
  		moveit::planning_interface::MoveGroup left_arm_group("left_arm");

  ros::ServiceServer service = nhPriv.advertiseService("arms_at_side", tidyup::armsAtSide::checkIfArmsAtSide);
  ros::spin();

  return 0;
}

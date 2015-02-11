#include <ros/ros.h>
#include "tidyup/arms_at_side.h"
#include <moveit/move_group_interface/move_group.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tidyup");
	ros::NodeHandle nhPriv("~");

	moveit::planning_interface::MoveGroup right_arm_group("right_arm");
	moveit::planning_interface::MoveGroup left_arm_group("left_arm");
	moveit::planning_interface::MoveGroup arms_group("arms");

	// Needed to initialize/update moveGroups to the current state!
	right_arm_group.getCurrentState();
	left_arm_group.getCurrentState();
	arms_group.getCurrentState();

	// Initialize objects.
	tidyup::armsAtSide armsAtSide(&right_arm_group, &left_arm_group);

	// Advertise the different services and action servers.
	ros::ServiceServer service = nhPriv.advertiseService(
			"arms_at_side", &tidyup::armsAtSide::checkIfArmsAtSide, &armsAtSide);
	ros::spin();

	return 0;
}
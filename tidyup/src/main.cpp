#include <ros/ros.h>
#include <actionlib/server/simple_action_server.h>
#include <moveit/move_group_interface/move_group.h>
#include <tidyup_utils/arms_at_side.h>
#include <symbolic_planning_utils/moveGroupInterface.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "tidyup");
	ros::NodeHandle nhPriv("~");

	moveit::planning_interface::MoveGroup* right_arm_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
	moveit::planning_interface::MoveGroup* left_arm_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
	moveit::planning_interface::MoveGroup* arms_group = symbolic_planning_utils::MoveGroupInterface::getInstance()->getArmsGroup();

	// Needed to initialize/update moveGroups to the current state!
	right_arm_group->getCurrentState();
	left_arm_group->getCurrentState();
	arms_group->getCurrentState();

	// Initialize objects.

	// Advertise the "tidyup/arms_at_side" service server
	tidyup::ArmsAtSide ArmsAtSide(right_arm_group, left_arm_group);

	ros::spin();

	return 0;
}

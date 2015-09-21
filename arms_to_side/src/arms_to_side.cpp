#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <symbolic_planning_utils/moveGroupInterface.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arms_to_side");

	ros::NodeHandle node_handle;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	moveit::planning_interface::MoveGroup* left_arm;
	moveit::planning_interface::MoveGroup* right_arm;
	moveit::planning_interface::MoveItErrorCode error_code;

	left_arm = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
	right_arm = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();

	std::string target_left = "left_arm_to_side";
	std::string target_right = "right_arm_to_side";

	moveit::planning_interface::MoveGroup* arms;
	arms = symbolic_planning_utils::MoveGroupInterface::getInstance()->getArmsGroup();

	if (!left_arm->setNamedTarget(target_left))
	{
		ROS_ERROR("Could not find named target: %s", target_left.c_str());
		return  moveit::planning_interface::MoveItErrorCode::FAILURE;
	}

	if (!right_arm->setNamedTarget(target_right))
	{
		ROS_ERROR("Could not find named target: %s", target_right.c_str());
		return  moveit::planning_interface::MoveItErrorCode::FAILURE;
	}

	error_code = left_arm->move();
	if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR("Could not move %s.", left_arm->getName().c_str());
		return error_code;
	}

	error_code = right_arm->move();
	if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
	{
		ROS_ERROR("Could not move %s.", right_arm->getName().c_str());
		return error_code;
	}

	return 0;
}


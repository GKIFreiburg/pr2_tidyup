#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <control_robot/control_head.h>
#include <control_robot/control_arms.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_robot");
	ros::NodeHandle nhPriv("~");

	moveit::planning_interface::MoveGroup right_arm_group("right_arm");
	moveit::planning_interface::MoveGroup left_arm_group("left_arm");
	moveit::planning_interface::MoveGroup arms_group("arms");
	moveit::planning_interface::MoveGroup head_group("head");

	// Needed to initialize/update moveGroups to the current state!
	right_arm_group.getCurrentState();
	left_arm_group.getCurrentState();
	arms_group.getCurrentState();
	head_group.getCurrentState();

	control_robot::ControlHead controlHead(&head_group);
	control_robot::ControlArms controlArms(&right_arm_group, &left_arm_group, &arms_group);

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
	return 0;
}


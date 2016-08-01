#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <control_robot/control_head.h>
#include <control_robot/control_arms.h>
#include <control_robot/control_torso.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "control_robot");
	ros::NodeHandle nhPriv("~");

	moveit::planning_interface::MoveGroup right_arm_group("right_arm");
	moveit::planning_interface::MoveGroup left_arm_group("left_arm");
	moveit::planning_interface::MoveGroup arms_group("arms");
	moveit::planning_interface::MoveGroup head_group("head");
	moveit::planning_interface::MoveGroup torso_group("torso");

	control_robot::ControlHead controlHead(&head_group);
	control_robot::ControlArms controlArms(&right_arm_group, &left_arm_group, &arms_group);
	control_robot::ControlTorso controlTorso(&torso_group);

	ros::AsyncSpinner spinner(4); // Use 4 threads
	spinner.start();
	ros::waitForShutdown();
	return 0;
}


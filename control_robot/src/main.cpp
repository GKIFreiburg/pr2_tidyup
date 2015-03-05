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

	ros::ServiceServer srvHeadPitchDown = nhPriv.advertiseService(
			"head_pitch_down", &control_robot::ControlHead::headPitchDown, &controlHead);
	ros::ServiceServer srvHeadPitchUp = nhPriv.advertiseService(
			"head_pitch_up", &control_robot::ControlHead::headPitchUp, &controlHead);
	ros::ServiceServer srvHeadPitchStraight = nhPriv.advertiseService(
			"head_pitch_straight", &control_robot::ControlHead::headPitchStraight, &controlHead);
	ros::ServiceServer srvHeadPitchDegrees = nhPriv.advertiseService(
			"head_pitch_degrees", &control_robot::ControlHead::headPitchDegrees, &controlHead);

	ros::ServiceServer srvHeadYawLeft = nhPriv.advertiseService(
			"head_yaw_left", &control_robot::ControlHead::headYawLeft, &controlHead);
	ros::ServiceServer srvHeadYawRight = nhPriv.advertiseService(
			"head_yaw_right", &control_robot::ControlHead::headYawRight, &controlHead);
	ros::ServiceServer srvHeadYawStraight = nhPriv.advertiseService(
			"head_yaw_straight", &control_robot::ControlHead::headYawStraight, &controlHead);
	ros::ServiceServer srvHeadYawDegrees = nhPriv.advertiseService(
			"head_yaw_degrees", &control_robot::ControlHead::headYawDegrees, &controlHead);

	ros::ServiceServer srvHeadInitialPosition = nhPriv.advertiseService(
			"head_initial_position", &control_robot::ControlHead::headInitialPosition, &controlHead);

	ros::ServiceServer srvRightArmToSide = nhPriv.advertiseService(
			"right_arm_to_side", &control_robot::ControlArms::rightArmToSide, &controlArms);
	ros::ServiceServer srvRightArmToFront = nhPriv.advertiseService(
			"right_arm_to_front", &control_robot::ControlArms::rightArmToFront, &controlArms);
	ros::ServiceServer srvRightArmToFrontBent = nhPriv.advertiseService(
			"right_arm_to_front_bent", &control_robot::ControlArms::rightArmToFrontBent, &controlArms);

	ros::ServiceServer srvLeftArmToSide = nhPriv.advertiseService(
			"left_arm_to_side", &control_robot::ControlArms::leftArmToSide, &controlArms);
	ros::ServiceServer srvLeftArmToFront = nhPriv.advertiseService(
			"left_arm_to_front", &control_robot::ControlArms::leftArmToFront, &controlArms);
	ros::ServiceServer srvLeftArmToFrontBent = nhPriv.advertiseService(
			"left_arm_to_front_bent", &control_robot::ControlArms::leftArmToFrontBent, &controlArms);

	ros::ServiceServer srvArmsToSide = nhPriv.advertiseService(
			"arms_to_side", &control_robot::ControlArms::armsToSide, &controlArms);
	ros::ServiceServer srvArmsToFront = nhPriv.advertiseService(
			"arms_to_front", &control_robot::ControlArms::armsToFront, &controlArms);
	ros::ServiceServer srvArmsToFrontBent = nhPriv.advertiseService(
			"arms_to_front_bent", &control_robot::ControlArms::armsToFrontBent, &controlArms);


	//std::string ns = ros::this_node::getNamespace();

    ROS_INFO("Waiting for head services.");
    ros::Duration timeout = ros::Duration(0.5);
    ros::service::waitForService("control_robot/head_pitch_down", timeout);
    ros::service::waitForService("control_robot/head_pitch_up", timeout);
    ros::service::waitForService("control_robot/head_pitch_straight", timeout);
    ros::service::waitForService("control_robot/head_pitch_degrees", timeout);

    ros::service::waitForService("control_robot/head_yaw_left", timeout);
    ros::service::waitForService("control_robot/head_yaw_right", timeout);
    ros::service::waitForService("control_robot/head_yaw_straight", timeout);
    ros::service::waitForService("control_robot/head_yaw_degrees", timeout);

    ros::service::waitForService("control_robot/head_initial_position", timeout);

    ros::service::waitForService("control_robot/right_arm_to_side", timeout);
    ros::service::waitForService("control_robot/right_arm_to_front", timeout);
    ros::service::waitForService("control_robot/right_arm_to_front_bent", timeout);

    ros::service::waitForService("control_robot/left_arm_to_side", timeout);
    ros::service::waitForService("control_robot/left_arm_to_front", timeout);
    ros::service::waitForService("control_robot/left_arm_to_front_bent", timeout);

    ros::service::waitForService("control_robot/arms_to_side", timeout);
    ros::service::waitForService("control_robot/arms_to_front", timeout);
    ros::service::waitForService("control_robot/arms_to_front_bent", timeout);

    ROS_INFO("Ready!");

    ros::AsyncSpinner spinner(4); // Use 4 threads
    spinner.start();
    ros::waitForShutdown();
	return 0;
}


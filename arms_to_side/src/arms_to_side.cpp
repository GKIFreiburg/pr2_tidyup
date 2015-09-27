#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <control_robot_msgs/MoveIt.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arms_to_side");

	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::ServiceClient torso_client = nh.serviceClient<control_robot_msgs::MoveIt>("/control_robot/torso_lift_max");
	control_robot_msgs::MoveIt srv;

	ROS_INFO("Wait for existence of service: %s", torso_client.getService().c_str());
	torso_client.waitForExistence();

	if (!torso_client.exists())
		ROS_ERROR("Client %s does not exist.", torso_client.getService().c_str());

	ROS_INFO("Lifting torso...");

	if (!torso_client.call(srv))
	{
		ROS_ERROR("Could not send service message to client: %s", torso_client.getService().c_str());
		return 1;
	}
	ROS_INFO("Service call for torso was successful.");


	// move arms to side
	ros::ServiceClient client = nh.serviceClient<control_robot_msgs::MoveIt>("/control_robot/arms_to_side");

	ROS_INFO("Wait for existence of service: %s", client.getService().c_str());
	client.waitForExistence();

	if(!client.exists())
		ROS_ERROR("Client %s does not exist.", client.getService().c_str());

	if (!client.call(srv))
	{
		ROS_ERROR("Could not send service message to client: %s", client.getService().c_str());
		return 1;
	}

	ROS_INFO("Service call for arms was successful.");
	return 0;


	// using service calls of control_robot -> allows to move both arms simultaneously
//	// lifting torso
//	moveit::planning_interface::MoveGroup* torso;
//	torso = symbolic_planning_utils::MoveGroupInterface::getInstance()->getTorsoGroup();
//	std::vector<std::string> torso_joints = torso->getJoints();
//	ROS_ASSERT(torso_joints.size() > 0);
//	double max_position = torso->getCurrentState().get()->getJointModel(torso_joints[0])->getVariableBounds()[0].max_position_;
//	// ROS_INFO("Max Position: %lf", max_position);
//	torso->setJointValueTarget(torso_joints[0], max_position);
//	moveit::planning_interface::MoveItErrorCode error_code;
//	ROS_INFO("Lifting torso to max position");
//	error_code = torso->move();
//	if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
//	{
//		ROS_ERROR("Could not lift torso!");
//		return 1;
//	}

//	moveit::planning_interface::MoveGroup* left_arm;
//	moveit::planning_interface::MoveGroup* right_arm;
//	moveit::planning_interface::MoveItErrorCode error_code;
//
//	left_arm = symbolic_planning_utils::MoveGroupInterface::getInstance()->getLeftArmGroup();
//	right_arm = symbolic_planning_utils::MoveGroupInterface::getInstance()->getRightArmGroup();
//
//	std::string target_left = "left_arm_to_side";
//	std::string target_right = "right_arm_to_side";
//
//	moveit::planning_interface::MoveGroup* arms;
//	arms = symbolic_planning_utils::MoveGroupInterface::getInstance()->getArmsGroup();
//
//	if (!left_arm->setNamedTarget(target_left))
//	{
//		ROS_ERROR("Could not find named target: %s", target_left.c_str());
//		return  moveit::planning_interface::MoveItErrorCode::FAILURE;
//	}
//
//	if (!right_arm->setNamedTarget(target_right))
//	{
//		ROS_ERROR("Could not find named target: %s", target_right.c_str());
//		return  moveit::planning_interface::MoveItErrorCode::FAILURE;
//	}
//
//	error_code = left_arm->move();
//	if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
//	{
//		ROS_ERROR("Could not move %s.", left_arm->getName().c_str());
//		return error_code;
//	}
//
//	error_code = right_arm->move();
//	if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
//	{
//		ROS_ERROR("Could not move %s.", right_arm->getName().c_str());
//		return error_code;
//	}
//
//	return 0;
}


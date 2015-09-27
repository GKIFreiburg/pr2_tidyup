#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <control_robot_msgs/MoveItPosition.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "lift_torso");

	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	double position;
	if (argc != 2)
	{
		ROS_ERROR("Usage: rosrun control_robot lift_torso <torso_position>");
		return 1;
	}
	position = atof(argv[1]);
	ROS_INFO("Set torso position to %lf", position);

	moveit::planning_interface::MoveGroup torso_group("torso");
	std::vector<std::string> torso_joints = torso_group.getJoints();
	ROS_ASSERT(torso_joints.size() > 0);
	if (!torso_group.setJointValueTarget(torso_joints[0], position))
	{
		ROS_ERROR("Position out of bounds - not moving torso");
		return 1;
	}
	moveit::planning_interface::MoveItErrorCode error_code;
	ROS_INFO("Lifting torso...");
	error_code = torso_group.move();
	ros::shutdown();
	return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;

//	//ros::ServiceClient torso_client = nh.serviceClient<control_robot_msgs::MoveIt>("/control_robot/torso_lift_max");
//	ros::ServiceClient torso_client = nh.serviceClient<control_robot_msgs::MoveItPosition>("/control_robot/torso_lift");
//	control_robot_msgs::MoveItPosition srv;
//	srv.request.position.data = position;
//
//	ROS_INFO("Wait for existence of service: %s", torso_client.getService().c_str());
//	torso_client.waitForExistence();
//
//	if (!torso_client.exists())
//		ROS_ERROR("Client %s does not exist.", torso_client.getService().c_str());
//
//	ROS_INFO("Lifting torso...");
//
//	if (!torso_client.call(srv))
//	{
//		ROS_ERROR("Could not send service message to client: %s", torso_client.getService().c_str());
//		return 1;
//	}
//	ROS_INFO("Service call for torso was successful.");

//	ros::shutdown();
//	return 0;

}

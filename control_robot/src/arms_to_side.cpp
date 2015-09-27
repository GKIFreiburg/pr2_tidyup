#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <control_robot_msgs/MoveIt.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "arms_to_side");

	ros::NodeHandle nh;
	ros::AsyncSpinner spinner(1);
	spinner.start();

	// move arms to side
	ros::ServiceClient client = nh.serviceClient<control_robot_msgs::MoveIt>("/control_robot/arms_to_side");
	control_robot_msgs::MoveIt srv;

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

}

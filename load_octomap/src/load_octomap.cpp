#include <ros/ros.h>
#include <moveit_msgs/LoadMap.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "load_octomap");
	ros::NodeHandle nh;

	if (argc != 2)
	{
		ROS_ERROR("No octomap found!\n"
				"load_octomap <path-to-octomap>");
		return 1;
	}
	std::string octomap_path = argv[1];

	// send recorded octomap to move_group
    ros::ServiceClient client = nh.serviceClient<moveit_msgs::LoadMap>("move_group/load_map");
    while(! client.waitForExistence(ros::Duration(3)))
    {
    	ROS_INFO_STREAM("service "<<client.getService()<<" has not been advertized yet...");
    }
    moveit_msgs::LoadMap srv;
    srv.request.filename = octomap_path;
    if (!client.call(srv))
    {
    	ROS_ERROR("Could not send '%s' to move_group", octomap_path.c_str());
    	return 1;
    }

    ROS_INFO("Octomap successfully loaded into move_group");
	return 0;
}

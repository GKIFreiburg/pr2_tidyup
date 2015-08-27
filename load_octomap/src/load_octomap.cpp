#include <ros/ros.h>
#include <moveit_msgs/LoadMap.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "load_octomap");
	ros::NodeHandle nh;

	ROS_ASSERT(argc == 2);
	std::string octomap_path = argv[1];

	// send recorded octomap to move_group
    ros::ServiceClient client = nh.serviceClient<moveit_msgs::LoadMap>("/move_group/load_map");
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

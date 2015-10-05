#include "ros/ros.h"
#include <ros/package.h>
#include <shape_msgs/Mesh.h>
#include <fstream>
#include <tidyup_utils/stringutil.h>
#include <sstream>
#include <fstream>
#include <rosbag/bag.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "export_table_mesh");

	ros::NodeHandle nh;
	ros::ServiceClient client = nh.serviceClient<moveit_msgs::GetPlanningScene>("get_planning_scene");

	moveit_msgs::GetPlanningScene srv;
	srv.request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY;

	if (!client.call(srv))
	{
		ROS_ERROR("Could not fetch planning scene!");
		return EXIT_FAILURE;
	}

	const std::vector<moveit_msgs::CollisionObject>& cos = srv.response.scene.world.collision_objects;
	for (int i = 0; i < cos.size(); i++)
	{
		if (!StringUtil::startsWith(cos[i].id, "table"))
			continue;


	    rosbag::Bag bag;
	    std::string fileName = ros::package::getPath("export_table_mesh");
	    fileName += "/exports/";
	    fileName += cos[i].id;
	    fileName += ".bag";
	    bag.open(fileName, rosbag::bagmode::Write);

	    moveit_msgs::CollisionObject co = cos[i];

	    bag.write("export_table", ros::TIME_MAX , co);

	    bag.close();


	}

}

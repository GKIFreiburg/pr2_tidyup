#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <symbolic_planning_utils/load_tables.h>
#include <shape_msgs/SolidPrimitive.h>
#include <shape_tools/solid_primitive_dims.h>
#include <manipulation_location_generator_msgs/CreateManipulationLocation.h>
#include <manipulation_location_generator/manipulation_location_generator.h>

#include <boost/foreach.hpp>
#ifdef __CDT_PARSER__
#define forEach(a, b) for(a : b)
#else
#define forEach BOOST_FOREACH
#endif

moveit_msgs::CollisionObject getTable(const std::string& id)
{
	moveit_msgs::CollisionObject table;
	std::vector<symbolic_planning_utils::LoadTables::TableLocation> tables;
	if (!symbolic_planning_utils::LoadTables::getTables(tables))
	{
		ROS_ERROR("Could not load tables from file");
		return table;
	}

	bool found = false;
	forEach(const symbolic_planning_utils::LoadTables::TableLocation& tl, tables)
	{
		if (tl.name != id)
			continue;

		found = true;
		table.id = tl.name;
		table.header = tl.pose.header;
		table.primitives.resize(1);
		table.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
		table.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
		table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = tl.sizex;
		table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = tl.sizey;
		table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = tl.sizez;
		table.primitive_poses.push_back(tl.pose.pose);
	}

	if (!found)
		ROS_WARN("No collision object with id: %s was found.", id.c_str());
	return table;
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_manipulation_location_generator");
    ros::NodeHandle nh;

    ros::ServiceClient client = nh.serviceClient
    		<manipulation_location_generator_msgs::CreateManipulationLocation>
    		("generate_manipulation_locations");

    manipulation_location_generator_msgs::CreateManipulationLocation srv;
    srv.request.sampling_method = manipulation_location_generator::UNIFORM_SAMPLING;
    srv.request.planning_scene_topic = "get_planning_scene";
    moveit_msgs::CollisionObject table = getTable("table1");
    srv.request.table = table;
    srv.request.max_samples = 100;
    srv.request.attempts = 1000;

    if (!client.call(srv))
    	ROS_ERROR("service call: %s failed", client.getService().c_str());
	std::vector<geometry_msgs::PoseStamped> mani_locs = srv.response.manipulation_locations;
    ROS_INFO("Found %lu manipulation locations.", mani_locs.size());

    ros::shutdown();
}


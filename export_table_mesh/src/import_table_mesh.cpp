#include "ros/ros.h"
#include <shape_msgs/Mesh.h>
#include <fstream>
#include <tidyup_utils/stringutil.h>
#include <sstream>
#include <fstream>
#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <ros/package.h>

#include <boost/foreach.hpp>
#define foreach BOOST_FOREACH

int main(int argc, char **argv)
{
	ros::init(argc, argv, "import_table_mesh");

	ros::NodeHandle nh;
	ros::Publisher pub = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 1);

    rosbag::Bag bag;
    std::string fileName = ros::package::getPath("export_table_mesh");
    fileName += "/exports/table1.bag";
    bag.open(fileName, rosbag::bagmode::Read);


    std::string topic = "export_table";

    rosbag::View view(bag, rosbag::TopicQuery(topic));
    moveit_msgs::CollisionObject obj;
    foreach(rosbag::MessageInstance const m, view)
    {
        moveit_msgs::CollisionObject::ConstPtr co = m.instantiate<moveit_msgs::CollisionObject>();
        if (co != NULL)
        {
        	obj = *co;
        	ROS_INFO("Publishing Collision Object");
        	obj.id = "haha";
        	obj.operation = moveit_msgs::CollisionObject::ADD;
        	ROS_WARN_STREAM(obj);
            pub.publish(obj);
        }
    }

	moveit_msgs::CollisionObject co;
	co.id = "test";
	co.header.frame_id = "map";
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(3);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.3;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;
	geometry_msgs::Pose pose;
	pose.position.x = 8.0;
	pose.position.y = 3.0;
	pose.position.z = 1.0;
	co.primitive_poses.push_back(pose);
	co.operation = co.ADD;
	pub.publish(co);


    ros::Publisher pu = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1, true);
    moveit_msgs::PlanningScene ps;
    ps.is_diff = true;
    ps.world.collision_objects.push_back(obj);


    ros::spin();
    bag.close();

}

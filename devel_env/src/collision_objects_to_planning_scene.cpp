#include <ros/ros.h>

// Moveit
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_tools/solid_primitive_dims.h>

#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Pose.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "collision_objects_to_planning_scene");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
//	ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

	ros::WallDuration(1.0).sleep();
	ros::spinOnce();

	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = "base_link";

	co.id = "coke";
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.067;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.067;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.12;
	co.primitive_poses.resize(1);
	co.primitive_poses[0].position.x = 0.5;
	co.primitive_poses[0].position.y = -0.75;
	co.primitive_poses[0].position.z = 0.56;
	co.primitive_poses[0].orientation.w = 1;

	co.operation = co.ADD;
	ROS_INFO("Publishing collision object: %s to planning scene", co.id.c_str());
	pub_co.publish(co);

	co.id = "bottle";
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.08;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.08;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
	co.primitive_poses.resize(1);
	co.primitive_poses[0].position.x = 0.5;
	co.primitive_poses[0].position.y = 0;
	co.primitive_poses[0].position.z = 0.67;
	co.primitive_poses[0].orientation.w = 1;

	co.operation = co.ADD;
	ROS_INFO("Publishing collision object: %s to planning scene", co.id.c_str());
	pub_co.publish(co);

	co.id = "bowl";
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.12;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.12;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.07;
	co.primitive_poses.resize(1);
	co.primitive_poses[0].position.x = 0.5;
	co.primitive_poses[0].position.y = 0.5;
	co.primitive_poses[0].position.z = 0.56;
	co.primitive_poses[0].orientation.w = 1;

	co.operation = co.ADD;
	ROS_INFO("Publishing collision object: %s to planning scene", co.id.c_str());
	pub_co.publish(co);

	co.id = "table1";
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.2;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.2;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
	co.primitive_poses.resize(1);
	co.primitive_poses[0].position.x = 1.0;
	co.primitive_poses[0].position.y = 0;
	co.primitive_poses[0].position.z = 0.5;
	co.primitive_poses[0].orientation.w = 1;

	co.operation = co.ADD;
	ROS_INFO("Publishing collision object: %s to planning scene", co.id.c_str());
	pub_co.publish(co);

	// wait a bit for ros things to initialize
	ros::WallDuration(1.0).sleep();

	ROS_INFO("Collision objects have been published, shutting down node");
	ros::shutdown();
	return 0;
}

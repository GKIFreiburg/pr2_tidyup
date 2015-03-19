#include <ros/ros.h>
#include "clear_planning_scene/ClearAllObjects.h"
#include "clear_planning_scene/ClearAttachedCollisionObjects.h"
#include "clear_planning_scene/ClearCollisionObjects.h"
#include "clear_planning_scene/DeleteAttachedCollisionObject.h"
#include "clear_planning_scene/DeleteCollisionObject.h"

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

ros::Publisher *planning_scene_diff_publisher;

// Clear collision objects as well as attached collision objects from planning scene.
bool clearAllObjects(clear_planning_scene::ClearAllObjects::Request  &req,
		clear_planning_scene::ClearAllObjects::Response &res)
{
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.clear();
	planning_scene.robot_state.attached_collision_objects.clear();
	//planning_scene.is_diff = true;
	planning_scene_diff_publisher->publish(planning_scene);
	ROS_INFO("All collision and attached objects have been removed.");
	return true;
}

// Clear all attached collision objects from planning scene.
bool clearAttachedCollisionObjects(clear_planning_scene::ClearAttachedCollisionObjects::Request  &req,
		clear_planning_scene::ClearAttachedCollisionObjects::Response &res)
{
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.robot_state.attached_collision_objects.clear();
	planning_scene_diff_publisher->publish(planning_scene);
	ROS_INFO("All attached objects have been removed.");
	return true;
}

// Clear all collision objects from planning scene.
bool clearCollisionObjects(clear_planning_scene::ClearCollisionObjects::Request  &req,
		clear_planning_scene::ClearCollisionObjects::Response &res)
{
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.clear();
	planning_scene_diff_publisher->publish(planning_scene);
	ROS_INFO("All collision objects have been removed.");
	return true;
}

// Clear a specific attached collision object from the planning scene.
bool deleteAttachedCollisionObject(clear_planning_scene::DeleteAttachedCollisionObject::Request  &req,
		clear_planning_scene::DeleteAttachedCollisionObject::Response &res)
{
	moveit_msgs::PlanningScene planning_scene;
	std::vector<moveit_msgs::AttachedCollisionObject>::iterator it;
	for (it = planning_scene.robot_state.attached_collision_objects.begin();
			it != planning_scene.robot_state.attached_collision_objects.end(); it++)
	{
		if (it->object.id == req.attachedCollisionObjectId)
		{
			planning_scene.robot_state.attached_collision_objects.erase(it);
			planning_scene_diff_publisher->publish(planning_scene);
			res.result = true;
			res.errorCode = "Success: Attached collision object with id " + it->object.id + "has been removed";
			return true;
		}
	}
	res.result = false;
	res.errorCode = "Warning: Could not find attached collision object with id " +
			req.attachedCollisionObjectId;
	return true;
}

// Clear a specific collision object from the planning scene.
bool deleteCollisionObject(clear_planning_scene::DeleteCollisionObject::Request  &req,
		clear_planning_scene::DeleteCollisionObject::Response &res)
{
	moveit_msgs::PlanningScene planning_scene;
	std::vector<moveit_msgs::CollisionObject>::iterator it;
	for (it = planning_scene.world.collision_objects.begin();
			it != planning_scene.world.collision_objects.end(); it++)
	{
		if (it->id == req.collisionObjectId)
		{
			planning_scene.world.collision_objects.erase(it);
			planning_scene_diff_publisher->publish(planning_scene);
			res.result = true;
			res.errorCode = "Success: Collision object with id " + it->id + "has been removed";
			return true;
		}
	}
	res.result = false;
	res.errorCode = "Warning: Could not find collision object with id " + req.collisionObjectId;
	return false;
}

int main(int argc, char **argv)
{
	ros::init(argc, argv, "clear_planning_scene_server");
	ros::NodeHandle nh;
	// private node handle
	ros::NodeHandle pnh("~");

	planning_scene_diff_publisher = new ros::Publisher(
			nh.advertise<moveit_msgs::PlanningScene>(
			"planning_scene", 1));
	ROS_INFO("Initialize %s", ros::this_node::getName().c_str());
	while(planning_scene_diff_publisher->getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
	}

	ros::ServiceServer serviceClearAllObjects = pnh.advertiseService(
			"clear_all_objects", clearAllObjects);
	ros::ServiceServer serviceClearAttachedCollisionObjects = pnh.advertiseService(
			"clear_attached_collision_objects", clearAttachedCollisionObjects);
	ros::ServiceServer serviceClearCollisionObjects = pnh.advertiseService(
			"clear_collision_objects", clearCollisionObjects);
	ros::ServiceServer serviceDeleteAttachedObject = pnh.advertiseService(
			"delete_attached_collision_object", deleteAttachedCollisionObject);
	ros::ServiceServer serviceDeleteCollisionObject = pnh.advertiseService(
			"delete_collision_object", deleteCollisionObject);
	ROS_INFO("Ready to delete collision objects.");
	ros::spin();

	return 0;
}

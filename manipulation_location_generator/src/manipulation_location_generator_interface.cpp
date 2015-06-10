#include "manipulation_location_generator/manipulation_location_generator_interface.h"
#include <ros/ros.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <shape_tools/solid_primitive_dims.h>

#include <moveit/planning_scene/planning_scene.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>
#include <geometry_msgs/PoseArray.h>

#include <costmap_2d/costmap_2d.h>
#include <costmap_2d/cost_values.h>
#include <nav_msgs/GetMap.h>
#include <nav_msgs/GetPlan.h>

#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

namespace manipulation_location_generator
{

ManipulationLocationGeneratorInterface::ManipulationLocationGeneratorInterface()
{

	planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_ = symbolic_planning_utils::PlanningSceneMonitorSingleton::getInstance()->getPlanningSceneMonitorPtr();
	planning_scene_.reset(new planning_scene::PlanningScene(planning_scene_monitor_->getRobotModel()));

	ros::NodeHandle nh;
	plan_path_client_ = nh.serviceClient<nav_msgs::GetPlan>("/move_base/make_plan", false);
	if (!plan_path_client_.exists())
		plan_path_client_ = nh.serviceClient<nav_msgs::GetPlan>("/move_base_node/make_plan", false);

	get_map_client_ = nh.serviceClient<nav_msgs::GetMap>("/static_map", false);

	// Publishers to publish the different kinds of samples
	pub_samples_ = ros::NodeHandle("~").advertise<geometry_msgs::PoseArray>("samples", 1, true);
	pub_not_in_collision_samples_ = ros::NodeHandle("~").advertise<geometry_msgs::PoseArray>("notInCollisionSamples", 1, true);
	pub_in_map_ = ros::NodeHandle("~").advertise<geometry_msgs::PoseArray>("inMapSamples", 1, true);
	pub_reachable_samples_ = ros::NodeHandle("~").advertise<geometry_msgs::PoseArray>("reachableSamples", 1, true);
}

ManipulationLocationGeneratorInterface::~ManipulationLocationGeneratorInterface()
{
}

bool ManipulationLocationGeneratorInterface::initialize(const std::string& planning_scene_topic,
		const moveit_msgs::CollisionObject& table,
		const int max_samples, const int attempts)
{
	if (!setPlanningScene(planning_scene_topic))
		return false;
	if (!extractTableInfo(table, table_width_, table_height_, table_pose_))
		return false;
	max_samples_ = max_samples;
	attempts_ = attempts;

	return true;
}


ManipulationLocations ManipulationLocationGeneratorInterface::generateSamples()
{
	ManipulationLocations mani_locs;
	geometry_msgs::PoseStamped table_pose = table_pose_;
	// project samples on ground
	table_pose.pose.position.z = 0;

	geometry_msgs::PoseArray samples;
	geometry_msgs::PoseArray not_in_collision_samples;
	geometry_msgs::PoseArray in_map_samples;
	geometry_msgs::PoseArray reachable_samples;
	samples.header 					= table_pose_.header;
	not_in_collision_samples.header = table_pose_.header;
	in_map_samples.header			= table_pose_.header;
	reachable_samples.header 		= table_pose_.header;

	unsigned i;
	for (i = 0; i < attempts_; i++)
	{
		// found enough good samples
		if (mani_locs.size() >= max_samples_)
			break;

		geometry_msgs::Pose sample = generateSample();

		// transform sample into world frame
		Eigen::Affine3d table_p;
		tf::poseMsgToEigen(table_pose.pose, table_p);

		Eigen::Affine3d sample_pose;
		tf::poseMsgToEigen(sample, sample_pose);
		sample_pose = table_p * sample_pose;

		geometry_msgs::PoseStamped robot_pose;
		robot_pose.header = table_pose_.header;
		tf::poseEigenToMsg(sample_pose, robot_pose.pose);

		samples.poses.push_back(robot_pose.pose);

		// check if sampled robot pose would be in collision
		if (inCollision(robot_pose))
			continue;
		not_in_collision_samples.poses.push_back(robot_pose.pose);

		// check if sample is in map
		if (!inMap(robot_pose))
			continue;
		in_map_samples.poses.push_back(robot_pose.pose);

//		// check if sampled robot pose is reachable
//		if (!isReachable(robot_pose))
//			continue;
//		reachable_samples.poses.push_back(robot_pose.pose);

		mani_locs.push_back(robot_pose);
	}

	pub_samples_.publish(samples);
	pub_not_in_collision_samples_.publish(not_in_collision_samples);
	pub_in_map_.publish(in_map_samples);
	pub_reachable_samples_.publish(reachable_samples);

	ROS_INFO("ManipulationLocationGeneratorInterface::%s: attempts: %u, possible robot poses: %lu",
			__func__, i, mani_locs.size());
	return mani_locs;
}

bool ManipulationLocationGeneratorInterface::setPlanningScene(const std::string& planning_scene_topic)
{
	ros::ServiceClient srv_planning_scene = nh_.serviceClient<moveit_msgs::GetPlanningScene>(planning_scene_topic);
	bool existence = srv_planning_scene.waitForExistence(ros::Duration(0.5));
	if (!existence)
	{
		ROS_ERROR("ManipulationLocationGeneratorInterface::%s: Could not subscribe to service: %s",
				__func__, planning_scene_topic.c_str());
		return false;
	}

	// get planning scene msg from service
    moveit_msgs::GetPlanningScene::Request request;
    request.components.components = moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
    		moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
    		moveit_msgs::PlanningSceneComponents::OBJECT_COLORS |
    		moveit_msgs::PlanningSceneComponents::OCTOMAP |
    		moveit_msgs::PlanningSceneComponents::ROBOT_STATE |
    		moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
    		moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS |
    		moveit_msgs::PlanningSceneComponents::TRANSFORMS |
    		moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
    		moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;
    moveit_msgs::GetPlanningScene::Response response;
	if (!srv_planning_scene.call(request, response))
	{
		ROS_ERROR("ManipulationLocationGeneratorInterface::%s: %s request failed.",
				__func__, srv_planning_scene.getService().c_str());
		return false;
	}
	// Verify that planning scene is not empty, should have at least some collision objects
	//ROS_ASSERT(response.scene.world.collision_objects.size() != 0);

	moveit_msgs::PlanningScene msg = response.scene;
	planning_scene_->setPlanningSceneMsg(msg);
	return true;
}

bool ManipulationLocationGeneratorInterface::extractTableInfo(const moveit_msgs::CollisionObject& table,
		double& width, double& height, geometry_msgs::PoseStamped& pose)
{
	ROS_ASSERT(table.primitives.size() == 1);
	if (table.primitives[0].type != shape_msgs::SolidPrimitive::BOX)
	{
		ROS_ERROR("ManipulationLocationGeneratorInterface::%s: Object %s does not have a solid primitive box",
				__func__, table.id.c_str());
		return false;
	}

	ROS_ASSERT(table.primitive_poses.size() == 1);
	pose.header = table.header;
	pose.pose   = table.primitive_poses[0];

	width  = table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];
	height = table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
	return true;
}

bool ManipulationLocationGeneratorInterface::inCollision(const geometry_msgs::PoseStamped& sample)
{
    collision_detection::CollisionRequest request;
    collision_detection::CollisionResult result;

	robot_state::RobotState robot_state = planning_scene_->getCurrentStateNonConst();
	//  std::vector<std::string> a = robot_state.getVariableNames();
	//  for (int i =0; i < a.size(); i++)
	//	  ROS_WARN("variable Name: %s", a[i].c_str());

	double yaw = tf::getYaw(sample.pose.orientation);
	robot_state.setVariablePosition("world_joint/x", sample.pose.position.x);
	robot_state.setVariablePosition("world_joint/y", sample.pose.position.y);
	robot_state.setVariablePosition("world_joint/theta", yaw);
	planning_scene_->setCurrentState(robot_state);

    planning_scene_->checkCollision(request, result);
    if (result.collision)
    	return true;
    else
    	return false;
}

bool ManipulationLocationGeneratorInterface::isReachable(const geometry_msgs::PoseStamped& sample)
{
	bool result = false;
	if (!plan_path_client_.exists())
	{
		ROS_ERROR("ManipulationLocationGeneratorInterface::%s: Could not subscribe to service: %s",
				__func__, plan_path_client_.getService().c_str());
		return false;
	}

	const robot_state::RobotState robot_state = planning_scene_->getCurrentState();
	geometry_msgs::PoseStamped start = getRobotPose(robot_state);

	nav_msgs::GetPlan srv;
	srv.request.start = start;
	srv.request.goal = sample;

	if (!plan_path_client_.call(srv))
	{
		ROS_ERROR("ManipulationLocationGeneratorInterface::%s: %s request failed.",
				__func__, plan_path_client_.getService().c_str());
		return false;
	}
	// if plan is empty, means that no path could be found, return false else return true
	if (srv.response.plan.poses.empty())
		result = false;
	else
		result = true;

	return result;
}

geometry_msgs::PoseStamped ManipulationLocationGeneratorInterface::getRobotPose(const robot_state::RobotState& robot_state)
{
	geometry_msgs::PoseStamped robot_pose;

	double x = robot_state.getVariablePosition("world_joint/x");
	double y = robot_state.getVariablePosition("world_joint/y");
	double theta = robot_state.getVariablePosition("world_joint/theta");

	tf::Quaternion q = tf::createQuaternionFromYaw(theta);
	geometry_msgs::Quaternion orientation;
	tf::quaternionTFToMsg(q, orientation);

	robot_pose.pose.position.x = x;
	robot_pose.pose.position.y = y;
	robot_pose.pose.position.z = 0;
	robot_pose.pose.orientation = orientation;

	robot_pose.header.frame_id = "/map";

	return robot_pose;
}

bool ManipulationLocationGeneratorInterface::inMap(const geometry_msgs::PoseStamped& sample)
{
	if (!get_map_client_.exists())
	{
		ROS_ERROR("ManipulationLocationGeneratorInterface::%s: Could not subscribe to service: %s",
				__func__, get_map_client_.getService().c_str());
		return false;
	}

	nav_msgs::GetMap srv;
	if (!get_map_client_.call(srv))
	{
		ROS_ERROR("ManipulationLocationGeneratorInterface::%s: %s request failed.",
				__func__, get_map_client_.getService().c_str());
		return false;
	}

	nav_msgs::OccupancyGrid map = srv.response.map;
	nav_msgs::MapMetaData info = map.info;
    costmap_2d::Costmap2D cost_map(info.width, info.height, info.resolution,
    		info.origin.position.x, info.origin.position.y);

    unsigned int mx, my;

    // Check if sample is in map, if not, return false
    if (!cost_map.worldToMap(sample.pose.position.x, sample.pose.position.y, mx, my))
    	return false;

    unsigned int index = cost_map.getIndex(mx, my);

    uint8_t value = map.data[index];

    // check if sample is on an free spot
    if (!value == costmap_2d::FREE_SPACE)
    	return false;

	return true;
}

};

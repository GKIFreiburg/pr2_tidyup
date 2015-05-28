/*********************************************************************
* Software License Agreement (BSD License)
*
*  Copyright (c) 2012, Willow Garage, Inc.
*  All rights reserved.
*
*  Redistribution and use in source and binary forms, with or without
*  modification, are permitted provided that the following conditions
*  are met:
*
*   * Redistributions of source code must retain the above copyright
*     notice, this list of conditions and the following disclaimer.
*   * Redistributions in binary form must reproduce the above
*     copyright notice, this list of conditions and the following
*     disclaimer in the documentation and/or other materials provided
*     with the distribution.
*   * Neither the name of Willow Garage nor the names of its
*     contributors may be used to endorse or promote products derived
*     from this software without specific prior written permission.
*
*  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
*  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
*  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
*  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
*  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
*  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
*  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
*  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
*  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
*  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
*  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
*  POSSIBILITY OF SUCH DAMAGE.
*********************************************************************/

/* Author: Sachin Chitta */

#include <pluginlib/class_loader.h>
#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/planning_pipeline/planning_pipeline.h>
#include <moveit/planning_interface/planning_interface.h>
#include <moveit/kinematic_constraints/utils.h>
#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/PlanningScene.h>

#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <actionlib/client/simple_action_client.h>
#include <grasp_provider_msgs/GenerateGraspsAction.h>
#include <eigen_conversions/eigen_msg.h>
#include <symbolic_planning_utils/moveGroupInterface.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "move_group_tutorial");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle node_handle("~");
	ros::NodeHandle nh;
	ros::Publisher pubPlanningScene = nh.advertise<moveit_msgs::PlanningScene>("virtual_planning_scene", 1, true);
	ros::Publisher pubPS = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	ros::Publisher visualization_publisher = nh.advertise<visualization_msgs::MarkerArray>("vis_debug", 1, true);
	ros::Publisher pubPoses = nh.advertise<geometry_msgs::PoseStamped>("poses", 1, true);
	ros::Publisher pubPoses2 = nh.advertise<geometry_msgs::PoseStamped>("poses2", 1, true);
	ros::ServiceClient srvPlanningScene = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
	ROS_INFO("Waiting for %s service.", move_group::GET_PLANNING_SCENE_SERVICE_NAME.c_str());
	srvPlanningScene.waitForExistence();
	actionlib::SimpleActionClient<grasp_provider_msgs::GenerateGraspsAction> actionGenerateGrasps_("generate_grasps", true);
	ROS_INFO("ActionExecutorPickupObject::%s: Waiting for generate_grasps "
			"action server to start.", __func__);
	actionGenerateGrasps_.waitForServer(); // will wait for infinite time

	// (Optional) Create a publisher for visualizing plans in Rviz.
	ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
	moveit_msgs::DisplayTrajectory display_trajectory;

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_model::RobotModelPtr robot_model = robot_model_loader.getModel();

	// create a planningScene object
	planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    psm->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRO locked_ps(psm);
    // Create a copy of planningScene
    planning_scene::PlanningScenePtr planning_scene = locked_ps->diff();

	ROS_INFO("PLANNING FRAME: %s", planning_scene->getPlanningFrame().c_str());

	// Change Position of Robot
	geometry_msgs::Pose robot_pose;
	// table1_loc3_room1 /map 4.54824 6.04086 0.000101075 0 0 -0.744561 0.667555
	// table1_loc4_room1 /map 4.85523 6.01968 -0.000637379 0 0 -0.739216 0.673468
	robot_pose.position.x = 4.85523;
	robot_pose.position.y = 6.02086;
	robot_pose.position.z = 0.0;
	robot_pose.orientation.x = 0;
	robot_pose.orientation.y = 0;
	robot_pose.orientation.z = -0.739216;
	robot_pose.orientation.w = 0.673468;
	double yaw = tf::getYaw(robot_pose.orientation);
	robot_state::RobotState robot_state = planning_scene->getCurrentStateNonConst();
	//  std::vector<std::string> a = robot_state.getVariableNames();
	//  for (int i =0; i < a.size(); i++)
	//	  ROS_WARN("variable Name: %s", a[i].c_str());

	robot_state.setVariablePosition("world_joint/x", robot_pose.position.x);
	robot_state.setVariablePosition("world_joint/y", robot_pose.position.y);
	robot_state.setVariablePosition("world_joint/theta", yaw);
	planning_scene->setCurrentState(robot_state);

//	visualization_msgs::MarkerArray visual_array;
//	std::vector<std::string> link_names = robot_state.getRobotModel()->getLinkModelNames();
//	std_msgs::ColorRGBA yellow;
//	yellow.a = 1;
//	yellow.r = 1;
//	yellow.g = 1;
//	yellow.b = 0;
//	robot_state.getRobotMarkers(visual_array, link_names, yellow, "initial", ros::Duration(10), true);
//	visualization_publisher.publish(visual_array);


	moveit_msgs::PlanningScene ps_msg;
	planning_scene->getPlanningSceneMsg(ps_msg);
	ps_msg.is_diff = true;
	pubPlanningScene.publish(ps_msg);
//	pubPS.publish(ps_msg);


	grasp_provider_msgs::GenerateGraspsGoal goal;
	moveit_msgs::CollisionObject collObj;
	std::vector<moveit_msgs::CollisionObject> objects = ps_msg.world.collision_objects;
	for (size_t i = 0; i < objects.size(); i++)
	{
	  if (objects[i].id == "coke_1")
	  {
		  ROS_WARN("FOUND!!!");
		  collObj = objects[i];
		  break;
	  }
	}
	goal.collision_object = collObj;
	std::string eef_name;
	eef_name = "right_gripper";
	goal.eef_group_name = eef_name;
	actionGenerateGrasps_.sendGoal(goal);
	// wait for the action to return
	bool finished_before_timeout = actionGenerateGrasps_.waitForResult(ros::Duration(30.0));
	std::vector<moveit_msgs::Grasp> grasps;
	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionGenerateGrasps_.getState();
		ROS_INFO("planning_pipeline::%s: Generate grasp action %s.", __func__, state.toString().c_str());
		if(state != actionlib::SimpleClientGoalState::SUCCEEDED)
		{
			ROS_ERROR("planning_pipeline::%s: Generate grasp action failed.", __func__);
			return false;
		}
		grasps = actionGenerateGrasps_.getResult()->grasps;
		ROS_INFO("planning_pipeline::%s: %lu grasps were found.", __func__, grasps.size());
	}


	// We can now setup the
	// `PlanningPipeline`_
	// object, which will use the ROS param server
	// to determine the set of request adapters and the
	// planning plugin to use
	planning_pipeline::PlanningPipelinePtr planning_pipeline(new planning_pipeline::PlanningPipeline(robot_model, node_handle, "planning_plugin", "request_adapters"));
	// Pose Goal
	// ^^^^^^^^^
	// We will now create a motion plan request for the right arm of the PR2
	// specifying the desired pose of the end-effector as input.
	planning_interface::MotionPlanRequest req;
	planning_interface::MotionPlanResponse res;
	req.allowed_planning_time = 25;

	// A tolerance of 0.01 m is specified in position
	// and 0.01 radians in orientation
	std::vector<double> tolerance_pose(3, 0.05);
	std::vector<double> tolerance_angle(3, 0.05);

	req.group_name = "right_arm";
	planning_pipeline->displayComputedMotionPlans(false);
	planning_pipeline->publishReceivedRequests(false);
	planning_pipeline->checkSolutionPaths(false);


	ros::Time start = ros::Time::now();

	for (size_t i = 0; i < grasps.size(); i++)
	{

		moveit_msgs::Grasp grasp = grasps[i];
		if (grasp.grasp_quality < 0.90)
			continue;

		geometry_msgs::PoseStamped target_pose = grasp.grasp_pose;
		// ROS_WARN("Target Frame: %s, Approach Frame: %s", target_pose.header.frame_id.c_str(),
		// 		grasp.pre_grasp_approach.direction.header.frame_id.c_str());

		// Compute Pre-grasp pose
		Eigen::Affine3d a;
		tf::poseMsgToEigen(target_pose.pose, a);
		Eigen::Vector3d approach;
		tf::vectorMsgToEigen(grasp.pre_grasp_approach.direction.vector, approach);
		a = a.translate(approach * grasp.pre_grasp_approach.min_distance * -1);
		// ROS_INFO_STREAM(target_pose);
		// ROS_INFO_STREAM("Affine3d: Translation " << a.translation());
		// ROS_INFO_STREAM("Affine3d: Rotation " << a.rotation());
		geometry_msgs::Pose p;
		tf::poseEigenToMsg(a, p);
		target_pose.pose = p;
		//pubPoses2.publish(target_pose);

		moveit_msgs::Constraints pose_goal =
				kinematic_constraints::constructGoalConstraints("r_wrist_roll_link", target_pose, tolerance_pose, tolerance_angle);
		req.goal_constraints.push_back(pose_goal);

		ROS_WARN("Grasp quality[%lu]: %lf", i, grasp.grasp_quality);

	}

	ROS_WARN("Number of constraints: %lu", req.goal_constraints.size());
	planning_pipeline->generatePlan(planning_scene, req, res);

	if(res.error_code_.val != res.error_code_.SUCCESS)
	{
		ROS_ERROR("Could not compute plan successfully");
	}
	else
	{
		ROS_INFO("Found possible plan!");
	}

	ros::Time end = ros::Time::now();
	ROS_WARN("Operation took %lf seconds!", end.toSec() - start.toSec());


	ROS_INFO("Visualizing the trajectory");
	moveit_msgs::MotionPlanResponse response;
	res.getMessage(response);

	display_trajectory.trajectory_start = response.trajectory_start;
	display_trajectory.trajectory.push_back(response.trajectory);
	display_publisher.publish(display_trajectory);

//	ros::WallDuration(180).sleep();
	ROS_INFO("Done");
	//  return 0;
}

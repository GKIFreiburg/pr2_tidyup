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

#include <ros/ros.h>
#include <cstdlib>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <shape_tools/solid_primitive_dims.h>
#include <actionlib/client/simple_action_client.h>

#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Pose.h>

#include <object_recognition_msgs/GetObjectInformation.h>
#include <object_recognition_msgs/ObjectInformation.h>
#include <object_recognition_msgs/ObjectType.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <tidyup_utils/stringutil.h>
#include <ork_to_planning_scene_msgs/UpdatePlanningSceneFromOrkAction.h>
#include <grasp_provider_msgs/GenerateGraspsAction.h>

#include <tf/transform_listener.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

void place(moveit::planning_interface::MoveGroup &group)
{
	std::vector<moveit_msgs::PlaceLocation> loc;

	geometry_msgs::PoseStamped p;
	p.header.frame_id = "base_link";
	p.pose.position.x = 0.5;
	p.pose.position.y = -0.75;
	p.pose.position.z = 0.58;
	p.pose.orientation.x = 0;
	p.pose.orientation.y = 0;
	p.pose.orientation.z = 0;
	p.pose.orientation.w = 1;
	moveit_msgs::PlaceLocation g;
	g.place_pose = p;

	g.pre_place_approach.direction.vector.z = -1.0;
	g.post_place_retreat.direction.vector.x = -1.0;
	g.post_place_retreat.direction.header.frame_id = "base_link";
	g.pre_place_approach.direction.header.frame_id = "r_wrist_roll_link";
	g.pre_place_approach.min_distance = 0.1;
	g.pre_place_approach.desired_distance = 0.2;
	g.post_place_retreat.min_distance = 0.1;
	g.post_place_retreat.desired_distance = 0.25;

	g.post_place_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
	g.post_place_posture.points.resize(1);
	g.post_place_posture.points[0].positions.resize(1);
	g.post_place_posture.points[0].positions[0] = 1;

	loc.push_back(g);
	//group.setSupportSurfaceName("table");


	// add path constraints
	moveit_msgs::Constraints constr;
	constr.orientation_constraints.resize(1);
	moveit_msgs::OrientationConstraint &ocm = constr.orientation_constraints[0];
	ocm.link_name = "r_wrist_roll_link";
	ocm.header.frame_id = p.header.frame_id;
	ocm.orientation.x = 0.0;
	ocm.orientation.y = 0.0;
	ocm.orientation.z = 0.0;
	ocm.orientation.w = 1.0;
	ocm.absolute_x_axis_tolerance = 0.2;
	ocm.absolute_y_axis_tolerance = 0.2;
	ocm.absolute_z_axis_tolerance = M_PI;
	ocm.weight = 1.0;
	group.setPathConstraints(constr);
	group.setPlannerId("RRTConnectkConfigDefault");

//	group.place(objectName, loc);
}

void dumpGrasps(const std::vector<moveit_msgs::Grasp>& grasps)
{
	ROS_WARN("pick_grasp_provider: Dumping grasps into ~/dumpGrasps/");
	std::ofstream outputFile;
	for (size_t i = 0; i < grasps.size(); i++)
	{
		std::string fileName = "grasp";
		fileName.append(boost::lexical_cast<std::string>(i) + ".txt");
		ROS_WARN("pick_grasp_provider: FileName: %s", fileName.c_str());
		outputFile.open(fileName.c_str());
		outputFile << grasps[i];
		outputFile.close();
	}
}

// Publishing a coke, estimated by a block to the planning scene.
// Pose w.r.t. base_link
void publishCokeBlockToPlanningScene(const ros::Publisher &pub_co, const geometry_msgs::Pose poseCokeBlock, const std::string& objName)
{
	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = "base_link";

	co.id = objName;
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.5 * 0.067;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.12;
	co.primitive_poses.push_back(poseCokeBlock);
//	co.primitive_poses.resize(1);
//	co.primitive_poses[0].position.x = 0.5;
//	co.primitive_poses[0].position.y = -0.75;
//	co.primitive_poses[0].position.z = 0.56;
//	co.primitive_poses[0].orientation.w = 1;

	co.operation = co.ADD;
	pub_co.publish(co);
}

void publishTableToPlanningScene(const ros::Publisher& pub_co)
{
	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = "odom_combined";

	co.id = "table";
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	// values from gazebo_worlds/objects/table/urdf.xacro
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 1.0;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 2.0;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.05;
	co.primitive_poses.resize(1);
	co.primitive_poses[0].position.x = 0.98;
	co.primitive_poses[0].position.y = 0;
	co.primitive_poses[0].position.z = 0.525;
	co.primitive_poses[0].orientation.w = 1;

	co.operation = co.ADD;
	pub_co.publish(co);
}


int main(int argc, char **argv)
{
	ros::init (argc, argv, "pick_grasp_provider");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);

	ROS_INFO("Waiting for GetPlanningScene Service to start.");
	ros::ServiceClient getPlanningSceneClient_;
	getPlanningSceneClient_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
	getPlanningSceneClient_.waitForExistence();

	actionlib::SimpleActionClient<grasp_provider_msgs::GenerateGraspsAction> actionGenerateGrasps_("generate_grasps", true);
	ROS_INFO("Waiting for action server \"generate_grasps\" to start.");
	// wait for the action server to start
	actionGenerateGrasps_.waitForServer(); // will wait for infinite time

	ros::NodeHandle pnh("~");
	std::string object_detection_;
	std::string grasp_object_;
	std::string eef_name_;
	// ros param: /pick_grasp_provider/object_detection
	pnh.param<std::string>("object_detection", object_detection_, "true");
	pnh.param<std::string>("grasp_object", grasp_object_, "coke");
	pnh.param<std::string>("eef_name", eef_name_, "right_gripper");
	ROS_WARN("object_detection is %s", object_detection_.c_str());

//	publishTableToPlanningScene(pub_co);


	if (!(object_detection_.compare("true") == 0))
	{
		geometry_msgs::PoseStamped p;
//		p.header.frame_id = "base_link";
		p.header.frame_id = "odom_combined";
		p.header.stamp = ros::Time::now();
		p.pose.position.x = 0.70;
		p.pose.position.y = -0.25;
//		p.pose.position.z = 0.61;
		p.pose.position.z = 0.55;
		p.pose.orientation.x = 0;
		p.pose.orientation.y = 0;
		p.pose.orientation.z = 0;
		p.pose.orientation.w = 1;
		publishCokeBlockToPlanningScene(pub_co, p.pose, "coke");

	} else { // using object detection

		actionlib::SimpleActionClient<ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkAction> actionOrkToPs_("ork_to_planning_scene", true);
		ROS_INFO("Waiting for action server \"ork_to_planning_scene\" to start.");
		// wait for the action server to start
		actionOrkToPs_.waitForServer(); // will wait for infinite time

		// execute action ork-to-planning-scene
		ROS_WARN("pick_grasp_provider: Sending UpdatePlanningSceneFromORK request.");
		ork_to_planning_scene_msgs::UpdatePlanningSceneFromOrkGoal updatePSGoal;
		updatePSGoal.verify_planning_scene_update = false;
		updatePSGoal.add_tables = false;
		actionOrkToPs_.sendGoal(updatePSGoal);
		bool finished_before_timeout = actionOrkToPs_.waitForResult(ros::Duration(30.0));
		if (finished_before_timeout)
		{
			actionlib::SimpleClientGoalState state = actionOrkToPs_.getState();
			ROS_INFO("pick_grasp_provider: ORK to Planning Scene Action finished: %s", state.toString().c_str());
			if (state != actionlib::SimpleClientGoalState::SUCCEEDED)
			{
				ROS_ERROR("Update ORK to Planning Scene action failed.");
				return 1;
			}
		} else {
			ROS_ERROR("pick_grasp_provider: Update ORK to Planning Scene action did not finish before the time out.");
			return 1;
		}
	}

	ROS_INFO("Wait 2 sec after publishing cos");
	ros::Duration(2.0).sleep();
	moveit_msgs::GetPlanningScene::Request request;
	moveit_msgs::GetPlanningScene::Response response;
	request.components.components = moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
		moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS;
	if (!getPlanningSceneClient_.call(request, response))
	{
		ROS_ERROR("pick_grasp_provider: planning scene request failed.");
		return 1;
	}

	moveit_msgs::CollisionObject collObj;
	bool found = false;
	ROS_ASSERT(response.scene.world.collision_objects.size() > 0);
	forEach(const moveit_msgs::CollisionObject & co, response.scene.world.collision_objects)
	{
		ROS_INFO("CO Name: %s", co.id.c_str());
		if (StringUtil::startsWith(co.id, grasp_object_))
		{
			found = true;
			collObj = co;
			break;
		}
	}

	if (!found)
	{
		ROS_ERROR("pick_grasp_provider: No collision object found with name %s", grasp_object_.c_str());
		return 1;
	}

	grasp_provider_msgs::GenerateGraspsGoal goal;
	goal.collision_object = collObj;
	goal.eef_group_name = eef_name_;
	actionGenerateGrasps_.sendGoal(goal);

	moveit::planning_interface::MoveGroup group("right_arm");
	group.setPlanningTime(45.0);

	//wait for the action to return
	bool finished_before_timeout = actionGenerateGrasps_.waitForResult(ros::Duration(30.0));

	if (finished_before_timeout)
	{
		actionlib::SimpleClientGoalState state = actionGenerateGrasps_.getState();
		ROS_INFO("Action finished: %s",state.toString().c_str());
		if(state != actionlib::SimpleClientGoalState::SUCCEEDED) {
			ROS_ERROR("Generate grasp action failed.");
			return 1;
		}
		std::vector<moveit_msgs::Grasp> grasps = actionGenerateGrasps_.getResult()->grasps;
		ROS_INFO("pick_grasp_provider: %lu grasps were found.", grasps.size());
		//dumpGrasps(grasps);
		group.pick(collObj.id, grasps);
	} else {
		ROS_ERROR("pick_grasp_provider: Generate grasp action did not finish before the time out.");
		return 1;
	}

	ros::shutdown();
	return 0;
}

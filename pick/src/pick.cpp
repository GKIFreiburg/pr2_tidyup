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

/* Author: Ioan Sucan */

#include <ros/ros.h>
#include <cstdlib>

// MoveIt!
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit_msgs/CollisionObject.h>
#include <shape_tools/solid_primitive_dims.h>

#include <shape_msgs/Mesh.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PoseWithCovarianceStamped.h>

#include <object_recognition_msgs/GetObjectInformation.h>
#include <object_recognition_msgs/ObjectInformation.h>
#include <object_recognition_msgs/ObjectType.h>
#include <object_recognition_msgs/RecognizedObjectArray.h>

#include <tf/transform_listener.h>

#include <rosbag/bag.h>
#include <rosbag/view.h>
#include <rosbag/message_instance.h>
#include <boost/foreach.hpp>
// #include <assert.h>
// #include <gtest/gtest.h> // only needed by ASSERT_EQ


const std::string bagTopic = "grasp";
const std::string bagFile = "grasp.txt";
const std::string objectName = "coke";

void pick(moveit::planning_interface::MoveGroup &group)
{
	std::vector<moveit_msgs::Grasp> grasps;

	moveit_msgs::Grasp g;

	geometry_msgs::PoseStamped p;
	p.header.frame_id = "base_link";
	p.header.stamp = ros::Time::now();
	//p.pose.position.x = 0.25;
	p.pose.position.x = 0.32; // 0.5-0.18 very important!
	p.pose.position.y = -0.75;
	p.pose.position.z = 0.58;
	p.pose.orientation.x = 0;
	p.pose.orientation.y = 0;
	p.pose.orientation.z = 0;
	p.pose.orientation.w = 1;
	g.grasp_pose = p;
	g.grasp_quality = 0.9;
	//g.id = "Grasp" + boost::lexical_cast<std::string>(k);
	//g.id = "Grasp" + boost::lexical_cast<std::string>(i) + " - " + boost::lexical_cast<std::string>(j);

	g.pre_grasp_approach.direction.vector.x = 1.0;
	g.pre_grasp_approach.direction.header.frame_id = "r_wrist_roll_link";
	g.pre_grasp_approach.min_distance = 0.12;
	g.pre_grasp_approach.desired_distance = 0.2;

	g.post_grasp_retreat.direction.header.frame_id = "base_footprint";
	g.post_grasp_retreat.direction.vector.z = 1.0;
	g.post_grasp_retreat.min_distance = 0.12;
	g.post_grasp_retreat.desired_distance = 0.2;

	g.pre_grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
	//g.pre_grasp_posture.joint_names.resize(1, "r_gripper_joint");
	//g.pre_grasp_posture.joint_names.resize(1, "r_gripper_l_finger_joint");
	g.pre_grasp_posture.points.resize(1);
	g.pre_grasp_posture.points[0].positions.resize(1);
	g.pre_grasp_posture.points[0].positions[0] = 1.0;
	g.pre_grasp_posture.points[0].time_from_start = ros::Duration(45.0);

	g.grasp_posture.joint_names.resize(1, "r_gripper_motor_screw_joint");
	//g.grasp_posture.joint_names.resize(1, "r_gripper_joint");
	g.grasp_posture.points.resize(1);
	g.grasp_posture.points[0].positions.resize(1);
	g.grasp_posture.points[0].positions[0] = 0;
	g.grasp_posture.points[0].time_from_start = ros::Duration(45.0);


	grasps.push_back(g);

	//group.setSupportSurfaceName("table");

	// Using rosbag to record msg.
//	rosbag::Bag bag;
//	bag.open(bagFile, rosbag::bagmode::Write);
//	bag.write(bagTopic, ros::Time::now(), g);
//	bag.close();

	// Dump a MSG into a file.
//	std::ofstream outputFile;
//	std::string fileName = "loaded_pick_grasp.txt";
//	outputFile.open(fileName.c_str());
//	outputFile << g;
//	outputFile.close();

	group.pick(objectName, grasps);
}

void loadGraspFromFileAndPick(moveit::planning_interface::MoveGroup &group)
{
	ROS_INFO("Loading Grasp from file");
	std::vector<moveit_msgs::Grasp> grasps;

	rosbag::Bag bag;
	bag.open(bagFile, rosbag::bagmode::Read);

	std::vector<std::string> topics;
	topics.push_back(bagTopic);

	rosbag::View view(bag, rosbag::TopicQuery(topics));

	BOOST_FOREACH(rosbag::MessageInstance const m, view)
	{
		moveit_msgs::Grasp::ConstPtr g = m.instantiate<moveit_msgs::Grasp>();
		if (g != NULL) {

			grasps.push_back(*g);

			//ASSERT_EQ(s->data, std::string("foo"));
			//std::cout << g << std::endl;

//			std::ofstream outputFile;
//			std::string fileName = "loaded_pick_grasp1.txt";
//			outputFile.open(fileName.c_str());
//			outputFile << *g;
//			outputFile.close();

		}
	}

	ROS_INFO("Size of grasps list %lu", grasps.size());

	bag.close();

	group.pick(objectName, grasps);
}

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

	group.place(objectName, loc);
}

// No const pointer because otherwise it must be declared here
object_recognition_msgs::RecognizedObjectArrayPtr recognizedObjects;

void recognizedObjectsReceived(
  const object_recognition_msgs::RecognizedObjectArrayPtr &msgIn
) {
	//ROS_INFO("Received recognized objects");
	recognizedObjects = msgIn;
}

bool publishDetectedObjectsToPlanningScene(ros::ServiceClient &client, const ros::Publisher &pub_co){

	if (recognizedObjects->objects.empty()) {
		ROS_WARN("RecognizedObjectArray is empty! No objects are added to the planning scene");
		return false;
	}

	for (std::size_t i = 0; i < recognizedObjects->objects.size(); i++)
	{
		// Creating an object an rewrite for each object the address, so get the current object.
		const object_recognition_msgs::RecognizedObject &object = recognizedObjects->objects[i];

		object_recognition_msgs::GetObjectInformation srv;
		srv.request.type = object.type;

		object_recognition_msgs::ObjectInformation information;
		if (client.call(srv))
		{
			information = srv.response.information;
			ROS_INFO("Receiving information of object #: %lu", i);
		} else {
			ROS_WARN("Failed to call service get_object_info");
			return false;
		}

		// Creating collision object
		moveit_msgs::CollisionObject co;
		co.header = object.header;

		co.id = information.name;

		co.meshes.resize(1);
		co.meshes.push_back(information.ground_truth_mesh);
		co.mesh_poses.resize(1);
		co.mesh_poses.push_back(object.pose.pose.pose);
		co.type = object.type;
		co.operation = co.ADD;

		// publish object to planning scene
		pub_co.publish(co);
	}

	return true;
}

// Publishing a coke, estimated by a block to the planning scene.
// Pose w.r.t. base_link
bool publishCokeBlockToPlanningScene(const ros::Publisher &pub_co, const geometry_msgs::Pose poseCokeBlock)
{
	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = "base_link";

	co.id = objectName;
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.067;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.067;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.12;
	co.primitive_poses.push_back(poseCokeBlock);
//	co.primitive_poses.resize(1);
//	co.primitive_poses[0].position.x = 0.5;
//	co.primitive_poses[0].position.y = -0.75;
//	co.primitive_poses[0].position.z = 0.56;
//	co.primitive_poses[0].orientation.w = 1;

	co.operation = co.ADD;
	pub_co.publish(co);

	return true;
}

int main(int argc, char **argv)
{
	ros::init (argc, argv, "right_arm_pick_place");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle nh;
	ros::Publisher pub_co = nh.advertise<moveit_msgs::CollisionObject>("collision_object", 10);
	ros::Publisher pub_aco = nh.advertise<moveit_msgs::AttachedCollisionObject>("attached_collision_object", 10);

	ros::Subscriber sub = nh.subscribe("/recognized_object_array", 10,	&recognizedObjectsReceived);

	ros::ServiceClient client = nh.serviceClient<object_recognition_msgs::GetObjectInformation>("get_object_info");

	ros::WallDuration(1.0).sleep();
	ros::spinOnce();

	moveit::planning_interface::MoveGroup group("right_arm");
	group.setPlanningTime(45.0);

/*
	moveit_msgs::CollisionObject remove_object;
	remove_object.id = "box";
	remove_object.header.frame_id = "odom_combined";
	remove_object.operation = remove_object.REMOVE;
	collision_object_publisher.publish(remove_object);

	// remove pole
	co.id = "pole";
	co.operation = moveit_msgs::CollisionObject::REMOVE;
	pub_co.publish(co);

	// add pole
	co.operation = moveit_msgs::CollisionObject::ADD;
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.3;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 1.0;
	co.primitive_poses.resize(1);
	co.primitive_poses[0].position.x = 0.7;
	co.primitive_poses[0].position.y = -0.4;
	co.primitive_poses[0].position.z = 0.85;
	co.primitive_poses[0].orientation.w = 1.0;
	pub_co.publish(co);



	// remove table
	co.id = "table";
	co.operation = moveit_msgs::CollisionObject::REMOVE;
	pub_co.publish(co);

	// add table
	co.operation = moveit_msgs::CollisionObject::ADD;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.5;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 1.5;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.35;
	co.primitive_poses[0].position.x = 0.7;
	co.primitive_poses[0].position.y = -0.2;
	co.primitive_poses[0].position.z = 0.175;
	pub_co.publish(co);



	co.id = "part";
	co.operation = moveit_msgs::CollisionObject::REMOVE;
	pub_co.publish(co);

	moveit_msgs::AttachedCollisionObject aco;
	aco.object = co;
	pub_aco.publish(aco);

	co.operation = moveit_msgs::CollisionObject::ADD;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.15;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.1;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.3;

	co.primitive_poses[0].position.x = 0.6;
	co.primitive_poses[0].position.y = -0.7;
	co.primitive_poses[0].position.z = 0.5;
	pub_co.publish(co);
	*/
	// wait a bit for ros things to initialize
	ros::WallDuration(1.0).sleep();

//	if (!publishDetectedObjectsToPlanningScene(client, pub_co))
//		ROS_WARN("Could not add detected objects to planning scene");
    geometry_msgs::PoseStamped p;
    p.header.frame_id = "base_link";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = 0.50;
    p.pose.position.y = -0.75;
    p.pose.position.z = 0.58;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
	publishCokeBlockToPlanningScene(pub_co, p.pose);

	pick(group);

	ros::WallDuration(1.0).sleep();

	place(group);

	ros::shutdown();
	return 0;
}

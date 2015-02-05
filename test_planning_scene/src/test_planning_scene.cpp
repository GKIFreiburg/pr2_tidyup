#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/PlanningSceneComponents.h>
#include <moveit_msgs/DisplayRobotState.h>
#include <shape_tools/solid_primitive_dims.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>

#include <fstream>

int main(int argc, char **argv)
{
	ros::init (argc, argv, "planning_scene_ros_api_tutorial");
	ros::AsyncSpinner spinner(1);
	spinner.start();

	ros::NodeHandle node_handle;
	ros::Duration sleep_time(15.0);
	sleep_time.sleep();

	ros::ServiceClient getPlanningSceneClient;
	getPlanningSceneClient = ros::service::createClient<moveit_msgs::GetPlanningScene>("get_planning_scene", true);

	moveit_msgs::GetPlanningScene srv;
	moveit_msgs::PlanningSceneComponents components;
	components.SCENE_SETTINGS;
	components.ROBOT_STATE;
	//srv.request.components = components;
	srv.request.components.ROBOT_STATE;
//    std::ofstream outputFile;
//    std::string fileName = "moveit_msgs::PlanningSceneComponents.msg";
//	outputFile.open(fileName.c_str());
//	outputFile << components;
//	outputFile.close();


	if (getPlanningSceneClient.call(srv))
	{
		ROS_INFO("Service calling planning scene was successful.");
	}

	robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
	robot_state::RobotState robot_state(robot_model_loader.getModel());

	std::string robotModelName = srv.response.scene.robot_model_name;
	ROS_WARN("RobotModelName = %s\n"
			 "Name = %s", robotModelName.c_str(), srv.response.scene.name.c_str());

//  // Test to visualize the robot using markers.
//	visualization_msgs::MarkerArray robotMarker;
//	std::vector<std::string> link_names = robot_state.getRobotModel()->getLinkModelNames();
//	for (int i = 0; i < link_names.size(); i++)
//		ROS_INFO("link_name[%d] = %s", i, link_names[i].c_str());
//
//	robot_state.getRobotMarkers(robotMarker, link_names, false);
//	std::ofstream outputFile;
//	std::string fileName = "robotMarker";
//	outputFile.open(fileName.c_str());
//	outputFile << robotMarker;
//	outputFile.close();
//	ros::NodeHandle nh("~");
//	ros::Publisher robotMarkerPub = nh.advertise<visualization_msgs::MarkerArray>("markers", 5, 1);
//	robotMarkerPub.publish(robotMarker);
//	ROS_INFO("Robot Marker Array published to 'robot_Marker' topic");











	// BEGIN_TUTORIAL
	//
	// ROS API
	// ^^^^^^^
	// The ROS API to the planning scene publisher is through a topic interface
	// using "diffs". A planning scene diff is the difference between the current
	// planning scene (maintained by the move_group node) and the new planning
	// scene desired by the user.

	// Advertise the required topic
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Note that this topic may need to be remapped in the launch file
	ros::Publisher planning_scene_diff_publisher = node_handle.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	while(planning_scene_diff_publisher.getNumSubscribers() < 1)
	{
		ros::WallDuration sleep_t(0.5);
		sleep_t.sleep();
	}

	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = "base_link";

	co.id = "cokeBox";
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


	// Add an object into the environment
	// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
	// Add the object into the environment by adding it to
	// the set of collision objects in the "world" part of the
	// planning scene. Note that we are using only the "object".
	ROS_INFO("Adding the object into the world at a fixed location.");
	moveit_msgs::PlanningScene planning_scene;
	planning_scene.world.collision_objects.push_back(co);
	planning_scene.is_diff = true;
	planning_scene_diff_publisher.publish(planning_scene);
	sleep_time.sleep();



	// Set Robot to a different state. All joints are set to random positions.
	ROS_INFO("Setting robot to another state");
	moveit::core::robotStateMsgToRobotState(srv.response.scene.robot_state,
											robot_state);
	robot_state.setToRandomPositions();
	moveit_msgs::RobotState newRobotState;
	moveit::core::robotStateToRobotStateMsg(robot_state, newRobotState);
	planning_scene.robot_state = newRobotState;
	planning_scene.is_diff = true;
	planning_scene_diff_publisher.publish(planning_scene);
	sleep_time.sleep();

	std::string frame_id = newRobotState.multi_dof_joint_state.header.frame_id;
	std::vector<std::string> joint_Names = newRobotState.multi_dof_joint_state.joint_names;
	std::vector<geometry_msgs::Transform> transforms = newRobotState.multi_dof_joint_state.transforms;

	ROS_WARN_STREAM("\nframe_id: " << frame_id);
	for(std::vector<std::string>::iterator Iter = joint_Names.begin(); Iter != joint_Names.end(); Iter++)
	{
		ROS_WARN_STREAM("\nJointName: " << *Iter);
	}

	for(std::vector<geometry_msgs::Transform>::iterator Iter = transforms.begin(); Iter != transforms.end(); Iter++)
	{
		ROS_WARN_STREAM("\nTransform: " << *Iter);
	}

	ROS_WARN_STREAM("Get Model Frame: " << robot_state.getRobotModel()->getModelFrame());





/*
// Attach an object to the robot
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// When the robot picks up an object from the environment, we need to
// "attach" the object to the robot so that any component dealing with
// the robot model knows to account for the attached object, e.g. for
// collision checking.

// Attaching an object requires two operations
//  * Removing the original object from the environment
//  * Attaching the object to the robot

  // First, define the REMOVE object message
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "odom_combined";
  remove_object.operation = remove_object.REMOVE;

// Note how we make sure that the diff message contains no other
// attached objects or collisions objects by clearing those fields
// first.
  // Carry out the REMOVE + ATTACH operation
  ROS_INFO("Attaching the object to the right wrist and removing it from the world.");
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_diff_publisher.publish(planning_scene);

  sleep_time.sleep();
*/

/*
// Detach an object from the robot
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Detaching an object from the robot requires two operations
//  * Detaching the object from the robot
//  * Re-introducing the object into the environment

  // First, define the DETACH object message
  moveit_msgs::AttachedCollisionObject detach_object;
  detach_object.object.id = "box";
  detach_object.link_name = "r_wrist_roll_link";
  detach_object.object.operation = attached_object.object.REMOVE;

// Note how we make sure that the diff message contains no other
// attached objects or collisions objects by clearing those fields
// first.
  // Carry out the DETACH + ADD operation
  ROS_INFO("Detaching the object from the robot and returning it to the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.robot_state.attached_collision_objects.push_back(detach_object);
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene_diff_publisher.publish(planning_scene);

  sleep_time.sleep();

// REMOVE THE OBJECT FROM THE COLLISION WORLD
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Removing the object from the collision world just requires
// using the remove object message defined earlier.
// Note, also how we make sure that the diff message contains no other
// attached objects or collisions objects by clearing those fields
// first.
  ROS_INFO("Removing the object from the world.");
  planning_scene.robot_state.attached_collision_objects.clear();
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene_diff_publisher.publish(planning_scene);
// END_TUTORIAL
*/
  ros::shutdown();
  return 0;
}

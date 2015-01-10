#include <ros/ros.h>
#include <geometry_msgs/Pose.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

// MoveIt!
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/GetStateValidity.h>
#include <moveit_msgs/DisplayRobotState.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_state/robot_state.h>
#include <moveit/robot_state/conversions.h>


int main(int argc, char **argv)
{
  ros::init (argc, argv, "planning_scene_ros_api_tutorial");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  ros::NodeHandle node_handle;
  ros::Duration sleep_time(10.0);
  sleep_time.sleep();
  sleep_time.sleep();


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

// Define the attached object message
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// We will use this message to add or
// subtract the object from the world
// and to attach the object to the robot
  moveit_msgs::AttachedCollisionObject attached_object;
  attached_object.link_name = "r_wrist_roll_link";
  /* The header must contain a valid TF frame*/
  attached_object.object.header.frame_id = "r_wrist_roll_link";
  /* The id of the object */
  attached_object.object.id = "box";

  /* A default pose */
  geometry_msgs::Pose pose;
  pose.orientation.w = 1.0;

  /* Define a box to be attached */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.1;
  primitive.dimensions[1] = 0.1;
  primitive.dimensions[2] = 0.1;

  attached_object.object.primitives.push_back(primitive);
  attached_object.object.primitive_poses.push_back(pose);

// Note that attaching an object to the robot requires
// the corresponding operation to be specified as an ADD operation
  attached_object.object.operation = attached_object.object.ADD;


// Add an object into the environment
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// Add the object into the environment by adding it to
// the set of collision objects in the "world" part of the
// planning scene. Note that we are using only the "object"
// field of the attached_object message here.
  ROS_INFO("Adding the object into the world at the location of the right wrist.");
  moveit_msgs::PlanningScene planning_scene;
  planning_scene.world.collision_objects.push_back(attached_object.object);
  planning_scene.is_diff = true;
  planning_scene_diff_publisher.publish(planning_scene);
  sleep_time.sleep();

// Attach an object to the robot
// ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
// When the robot picks up an object from the environment, we need to
// "attach" the object to the robot so that any component dealing with
// the robot model knows to account for the attached object, e.g. for
// collision checking.

// Attaching an object requires two operations
//  * Removing the original object from the environment
//  * Attaching the object to the robot

  /* First, define the REMOVE object message*/
  moveit_msgs::CollisionObject remove_object;
  remove_object.id = "box";
  remove_object.header.frame_id = "odom_combined";
  remove_object.operation = remove_object.REMOVE;

// Note how we make sure that the diff message contains no other
// attached objects or collisions objects by clearing those fields
// first.
  /* Carry out the REMOVE + ATTACH operation */
  ROS_INFO("Attaching the object to the right wrist and removing it from the world.");
  planning_scene.world.collision_objects.clear();
  planning_scene.world.collision_objects.push_back(remove_object);
  planning_scene.robot_state.attached_collision_objects.push_back(attached_object);
  planning_scene_diff_publisher.publish(planning_scene);

  sleep_time.sleep();







  // The :move_group_interface:`MoveGroup` class can be easily
    // setup using just the name
    // of the group you would like to control and plan for.
    moveit::planning_interface::MoveGroup group("right_arm");

    // We will use the :planning_scene_interface:`PlanningSceneInterface`
    // class to deal directly with the world.
    moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

    // Getting Basic Information
    // ^^^^^^^^^^^^^^^^^^^^^^^^^
    //
    // We can print the name of the reference frame for this robot.
    ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

    // We can also print the name of the end-effector link for this group.
    ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

    // Planning to a Pose goal
    // ^^^^^^^^^^^^^^^^^^^^^^^
    // We can plan a motion for this group to a desired pose for the
    // end-effector.
    geometry_msgs::Pose target_pose1;
    target_pose1.orientation.w = 1.0;
    target_pose1.position.x = 0.28;
    target_pose1.position.y = -0.7;
    target_pose1.position.z = 1.0;
    group.setPoseTarget(target_pose1);


    // Now, we call the planner to compute the plan
    // and visualize it.
    // Note that we are just planning, not asking move_group
    // to actually move the robot.
    moveit::planning_interface::MoveGroup::Plan my_plan;
    bool success = group.plan(my_plan);

    ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
    /* Sleep to give Rviz time to visualize the plan. */
    sleep(5.0);





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










































































//#include <ros/ros.h>
//
//#include <moveit/move_group_interface/move_group.h>
//#include <moveit/planning_scene_interface/planning_scene_interface.h>
//
//#include <moveit_msgs/DisplayTrajectory.h>
//#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
//
//int main(int argc, char **argv)
//{
//  ros::init(argc, argv, "test");
//  ros::NodeHandle node_handle;
//  ros::AsyncSpinner spinner(1);
//  spinner.start();
//
//  // The :move_group_interface:`MoveGroup` class can be easily
//  // setup using just the name
//  // of the group you would like to control and plan for.
//  moveit::planning_interface::MoveGroup group("right_arm");
//
//  // Getting Basic Information
//  // ^^^^^^^^^^^^^^^^^^^^^^^^^
//  //
//  // We can print the name of the reference frame for this robot.
//  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());
//
//  // We can also print the name of the end-effector link for this group.
//  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());
//
//  // Get the name of the group this instance operates on
//  ROS_INFO("Get the name of the group this instance operates on: %s", group.getName().c_str());
//
//  // Setup
//  // ^^^^^
//  //
//
//
//
//
//  robot_model::RobotModelConstPtr robot_model_const_ptr = group.getCurrentState()->getRobotModel();
//
//
//
//  //robot_model::RobotModelPtr robot_model_ptr = robot_model_const_ptr.
// // planning_scene::PlanningScene planning_scene(robot_model_const_ptr);
//
////  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
////  robot_model::RobotModelPtr kinematic_model = robot_model_loader.getModel();
////  planning_scene::PlanningScene planning_scene(kinematic_model);
//
//
//  collision_detection::CollisionRequest collision_request;
//  collision_detection::CollisionResult collision_result;
//  planning_scene.checkSelfCollision(collision_request, collision_result);
//  ROS_INFO_STREAM("Test 1 (Self-collision checking): Current state is "
//                  << (collision_result.collision ? "in" : "not in")
//                  << " self collision");
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
//
////  std::filebuf fb;
////  fb.open("test.txt",std::ios::out);
////  std::ostream os(&fb);
////  robot_model_const_ptr->printModelInfo(os);
////  fb.close();
//
//
//
////
////  // Self-collision checking
////  // ^^^^^^^^^^^^^^^^^^^^^^^
////  //
////  collision_detection::CollisionRequest collision_request;
////  collision_detection::CollisionResult collision_result;
////  planning_scene_ptr->checkSelfCollision(collision_request, collision_result);
////  ROS_INFO_STREAM("Test 1: Current state is "
////                  << (collision_result.collision ? "in" : "not in")
////                  << " self collision");
////
////
////
////  // Planning to a Pose goal
////  // ^^^^^^^^^^^^^^^^^^^^^^^
////  // We can plan a motion for this group to a desired pose for the
////  // end-effector.
////  geometry_msgs::Pose target_pose1;
////  target_pose1.orientation.w = 1;
////  target_pose1.position.x = 0.28;
////  target_pose1.position.y = -0.7;
////  target_pose1.position.z = 1.0;
////  group.setPoseTarget(target_pose1);
////
////
////  // Now, we call the planner to compute the plan
////  // and visualize it.
////  // Note that we are just planning, not asking move_group
////  // to actually move the robot.
////  moveit::planning_interface::MoveGroup::Plan my_plan;
////  bool success = group.plan(my_plan);
////
////  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
////  /* Sleep to give Rviz time to visualize the plan. */
////  sleep(5.0);
////
////  // Visualizing plans
////  // ^^^^^^^^^^^^^^^^^
////  // Now that we have a plan we can visualize it in Rviz.  This is not
////  // necessary because the group.plan() call we made above did this
////  // automatically.  But explicitly publishing plans is useful in cases that we
////  // want to visualize a previously created plan.
//////  if (1)
//////  {
//////    ROS_INFO("Visualizing plan 1 (again)");
//////    display_trajectory.trajectory_start = my_plan.start_state_;
//////    display_trajectory.trajectory.push_back(my_plan.trajectory_);
//////    display_publisher.publish(display_trajectory);
//////    /* Sleep to give Rviz time to visualize the plan. */
//////    sleep(5.0);
//////  }
////
////  // Moving to a pose goal
////  // ^^^^^^^^^^^^^^^^^^^^^
////  //
////  // Moving to a pose goal is similar to the step above
////  // except we now use the move() function. Note that
////  // the pose goal we had set earlier is still active
////  // and so the robot will try to move to that goal. We will
////  // not use that function in this tutorial since it is
////  // a blocking function and requires a controller to be active
////  // and report success on execution of a trajectory.
////
////  /* Uncomment below line when working with a real robot*/
////  /* group.move() */
//
//  ros::shutdown();
//  return 0;
//}

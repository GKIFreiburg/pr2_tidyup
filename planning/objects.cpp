#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  //
  // Setup
  // ^^^^^
  //
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("right_arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group.getEndEffectorLink().c_str());

  // Import, otherwise no object is shown in rviz.
  sleep(1.0);


  // Adding/Removing Objects and Attaching/Detaching Objects
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First, we will define the collision object message.
  moveit_msgs::CollisionObject collision_object;
  collision_object.header.frame_id = group.getPlanningFrame();

  /* The id of the object is used to identify it. */
  collision_object.id = "box1";

  /* Define a box to add to the world. */
  shape_msgs::SolidPrimitive primitive;
  primitive.type = primitive.BOX;
  primitive.dimensions.resize(3);
  primitive.dimensions[0] = 0.4;  // x
  primitive.dimensions[1] = 0.2;  // y
  primitive.dimensions[2] = 0.4;  // z

  /* A pose for the box (specified relative to frame_id) */
  geometry_msgs::Pose box_pose;


  double roll, pitch, yaw;
  geometry_msgs::Quaternion geo_q;
  roll = M_PI/2;
  pitch = 0;
  yaw = 0;

  tf::Quaternion tf_q;
  tf_q.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(tf_q, geo_q);

  box_pose.orientation.w = 1;
/*  box_pose.orientation.w = geo_q.w;
  box_pose.orientation.x = geo_q.x;
  box_pose.orientation.y = geo_q.y;
  box_pose.orientation.z = geo_q.z; */
  box_pose.position.x =  0.6;
  box_pose.position.y = -0.4;
  box_pose.position.z =  1.2;

  collision_object.primitives.push_back(primitive);
  collision_object.primitive_poses.push_back(box_pose);
/*
  // Define a box to add to the world.
  shape_msgs::SolidPrimitive box2;
  box2 = primitive;
  geometry_msgs::Pose box2_pose;

  box2_pose.orientation.w = 1;
  box2_pose.position.x =  0.6;
  box2_pose.position.y =  0.4;
  box2_pose.position.z =  1.2;

  collision_object.primitives.push_back(box2);
  collision_object.primitive_poses.push_back(box2_pose);
*/
  collision_object.operation = collision_object.ADD;

  std::vector<moveit_msgs::CollisionObject> collision_objects;
  collision_objects.push_back(collision_object);

  // Now, let's add the collision object into the world
  ROS_INFO("Add an object into the world");
  planning_scene_interface.addCollisionObjects(collision_objects);

  /* Sleep so we have time to see the object in RViz */
  sleep(2.0);

  // Planning with collision detection can be slow.  Lets set the planning time
  // to be sure the planner has enough time to plan around the box.  10 seconds
  // should be plenty.
  group.setPlanningTime(10.0);


  // Now when we plan a trajectory it will avoid the obstacle
  group.setStartState(*group.getCurrentState());

  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 5 (pose goal move around box) %s",
    success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  sleep(10.0);

  // Now, let's attach the collision object to the robot.
  ROS_INFO("Attach the object to the robot");
  group.attachObject(collision_object.id);
  // Sleep to give Rviz time to show the object attached (different color).
 /* target_pose1.position.x = -0.2;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);
  success = group.plan(my_plan); // Object does not move! */

  ROS_INFO("Visualizing plan 6 (move arm with box) %s",
    success?"":"FAILED");

  sleep(4.0);


  // Now, let's detach the collision object from the robot.
  ROS_INFO("Detach the object from the robot");
  group.detachObject(collision_object.id);
  /* Sleep to give Rviz time to show the object detached. */
  sleep(4.0);


  // Now, let's remove the collision object from the world.
  ROS_INFO("Remove the object from the world");
  std::vector<std::string> object_ids;
  object_ids.push_back(collision_object.id);
  // removeCollisionObjects needs a vector of strings as input.
  planning_scene_interface.removeCollisionObjects(object_ids);
  /* Sleep to give Rviz time to show the object is no longer there. */
  sleep(4.0);

  //END_TUTORIAL

  ros::shutdown();
  return 0;
}

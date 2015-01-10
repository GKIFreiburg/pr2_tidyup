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

  ROS_INFO("Cartesian reference frame %s", group.getPoseReferenceFrame().c_str());

  bool success;

  std::vector<geometry_msgs::Pose> waypoints;


  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = 1.0;
  start_pose.position.x = 0.55;
  start_pose.position.y = -0.05;
  start_pose.position.z = 0.8;

  geometry_msgs::Pose target_pose3 = start_pose;
  target_pose3.position.x = 0.28;
  target_pose3.position.y = -0.7;
  target_pose3.position.z = 1;
  waypoints.push_back(target_pose3);  // up and out

  target_pose3.position.x = -0.3;
  target_pose3.position.y = -0.7;
  target_pose3.position.z = 0.6;
  waypoints.push_back(target_pose3);  // left

  target_pose3.position.x = 0.28;
  target_pose3.position.y = -0.7;
  target_pose3.position.z = 1;
  waypoints.push_back(target_pose3);  // left

  start_pose.position.x = 0.55;
  start_pose.position.y = -0.05;
  start_pose.position.z = 0.8;
  waypoints.push_back(start_pose);  // down and right (back to start)

  // We want the cartesian path to be interpolated at a resolution of 1 cm
  // which is why we will specify 0.01 as the max step in cartesian
  // translation.  We will specify the jump threshold as 0.0, effectively
  // disabling it.
  moveit_msgs::RobotTrajectory trajectory;

  // Compute a Cartesian path that follows specified waypoints with a step size of at most \e eef_step meters
  // between end effector configurations of consecutive points in the result \e trajectory. The reference frame for the
  // waypoints is that specified by setPoseReferenceFrame(). No more than \e jump_threshold
  // is allowed as change in distance in the configuration space of the robot (this is to prevent 'jumps' in IK solutions).
  // Collisions are avoided if \e avoid_collisions is set to true. If collisions cannot be avoided, the function fails.
  // Return a value that is between 0.0 and 1.0 indicating the fraction of the path achieved as described by the waypoints.
  // Return -1.0 in case of error.
  double fraction = group.computeCartesianPath(waypoints,
                                               0.05,  // eef_step
                                               0.0,   // jump_threshold
                                               trajectory);

  ROS_INFO("Visualizing plan 4 (cartesian path) (%.2f%% acheived)",
        fraction * 100.0);

  //END_TUTORIAL

  ros::shutdown();
  return 0;
}

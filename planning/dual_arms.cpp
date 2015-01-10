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

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;



  // Dual-arm pose goals
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  // First define a new group for addressing the two arms. Then define
  // two separate pose goals, one for each end-effector. Note that
  // we are reusing the goal for the right arm above
  moveit::planning_interface::MoveGroup two_arms_group("arms");

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", two_arms_group.getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", two_arms_group.getEndEffectorLink().c_str());

  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;


  two_arms_group.setPoseTarget(target_pose1, "r_wrist_roll_link");

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.7;
  target_pose2.position.y = 0.45;
  target_pose2.position.z = 1.0;

  two_arms_group.setPoseTarget(target_pose2, "l_wrist_roll_link");

  // Now, we can plan and visualize
  moveit::planning_interface::MoveGroup::Plan two_arms_plan;
  two_arms_group.plan(two_arms_plan);
  sleep(4.0);

  //END_TUTORIAL

  ros::shutdown();
  return 0;
}

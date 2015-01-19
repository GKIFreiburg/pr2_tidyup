#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>
#include <moveit_msgs/RobotState.h>
#include <moveit/robot_state/robot_state.h>

double degreesToRadians(const double& angle)
{
  double result = M_PI/180 * angle;
  return result;
}

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_body_parts_for_detection");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // This sleep is ONLY to allow Rviz to come up
  //sleep(15.0);

  // Setup
  // ^^^^^
  //
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup two_arms_group("arms");
  moveit::planning_interface::MoveGroup head_group("head");

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;

  // Getting Basic Information
  // ^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // We can print the name of the reference frame for this robot.
  ROS_INFO("Reference frame: %s", two_arms_group.getPlanningFrame().c_str());

  robot_state::RobotState rstate = *two_arms_group.getCurrentState();
  rstate.setToDefaultValues();

  // Moving to a pose goal
  // ^^^^^^^^^^^^^^^^^^^^^
  //
  // Moving to a pose goal is similar to the step above
  // except we now use the move() function. Note that
  // the pose goal we had set earlier is still active
  // and so the robot will try to move to that goal. We will
  // not use that function in this tutorial since it is
  // a blocking function and requires a controller to be active
  // and report success on execution of a trajectory.
  // Uncomment below line when working with a real robot
  two_arms_group.move();
  head_group.move();

  ros::shutdown();
  return 0;
}


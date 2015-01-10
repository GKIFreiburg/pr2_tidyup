#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>

double degreesToRadians(const double& angle)
{
  double result = M_PI/180 * angle;
  return result;
}



int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  //sleep(15.0);

  // Setup
  // ^^^^^
  //
  // The :move_group_interface:`MoveGroup` class can be easily
  // setup using just the name
  // of the group you would like to control and plan for.
  moveit::planning_interface::MoveGroup group("whole_body");

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

  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = 1.0;
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;

  group.setPoseTarget(target_pose1, "r_wrist_roll_link");

  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = 1.0;
  target_pose2.position.x = 0.28;
  target_pose2.position.y = 0.70;
  target_pose2.position.z = 1.0;

  group.setPoseTarget(target_pose2, "l_wrist_roll_link");

  // First get the current set of joint values for the group.
  std::vector<double> group_variable_values;
  //group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group.getCurrentState()->copyJointGroupPositions(group.getName(), group_variable_values);

  std::vector<std::string> jointNames = group.getJoints();
  int i;
  for (i = 0; i < jointNames.size(); i++)
  {
    ROS_INFO("jointNames[%d]: %s, jointvalue before change: %f", i, jointNames[i].c_str(), group_variable_values[i]);
    if (jointNames[i] == "head_tilt_joint")
      break;
  }

  // Now, let's modify one of the joints, plan to the new joint
  // space goal and visualize the plan.
  // group_variable_Values[1] corresponds to head_tilt_joint
  ROS_INFO("i = %d", i);
  group_variable_values[i] = 0.6;
  group.setJointValueTarget(group_variable_values);


  // Now, we can plan and visualize
  bool success;
  moveit::planning_interface::MoveGroup::Plan whole_body_plan;
  success = group.plan(whole_body_plan);


  ROS_INFO("Visualizing plan %s",success?"":"FAILED");

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

  /* Uncomment below line when working with a real robot*/
  //group.move();
  //sleep(5.0);

  ros::shutdown();
  return 0;
}


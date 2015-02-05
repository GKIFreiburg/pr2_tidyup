#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <moveit_msgs/AttachedCollisionObject.h>
#include <moveit_msgs/CollisionObject.h>

int main(int argc, char **argv)
{

  if (argc != 1 && argc != 3)
  {
    ROS_ERROR("Usage: rosrun planning joint_Space <jointNr> <jointValue>");
    exit(EXIT_FAILURE);
  }

  int jointNr;
  float jointValue;
  if (argc == 1)
  {
    jointNr = 0;
    jointValue = -1.0;
  } else {
    jointNr = atoi(argv[1]);
    jointValue = atof(argv[2]);
  }


  ros::init(argc, argv, "move_group_interface_tutorial");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();


  /* This sleep is ONLY to allow Rviz to come up */
  // sleep(20.0);

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

  // Planning to a joint-space goal
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Let's set a joint space goal and move towards it.  This will replace the
  // pose target we set above.
  //
  // First get the current set of joint values for the group.
  std::vector<double> group_variable_values;
  //group.getCurrentState()->copyJointGroupPositions(group.getCurrentState()->getRobotModel()->getJointModelGroup(group.getName()), group_variable_values);
  group.getCurrentState()->copyJointGroupPositions(group.getName(), group_variable_values);
  // gives the same values as group_variable_values
  std::vector<double> jointValues = group.getCurrentJointValues();

  // Verify that getCurrentJointValues gives the same values as copyJointGroupPositions.
  if (jointValues == group_variable_values)
	  ROS_INFO("Both vectors have the same values for the joint positions.");
  else
	  ROS_WARN("Both vectors have different values for the joint positions.");

  std::vector<std::string> jointNames = group.getJoints();
  for (int i = 0; i < jointNames.size(); i++)
    ROS_INFO("jointNames[%d]: %s, jointvalue before change: %f - %f", i, jointNames[i].c_str(), group_variable_values[i], jointValues[i]);

  // Now, let's modify one of the joints, plan to the new joint
  // space goal and visualize the plan.
  // group_variable_values[jointNr] = jointValue;
  //group.setJointValueTarget(group_variable_values);

  // Set the joint position for the entire right arm. Arm tucked
  double values[] = {-2.110, 1.230, -2.06, -1.69, 0.3, -1.32, 1.57};
  std::vector<double> right_arm_at_side(values, values + sizeof(values) / sizeof(double));
  group.setJointValueTarget(right_arm_at_side);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan (joint space goal) %s",success?"":"FAILED");
  if (!success)
    ROS_WARN("Probably the joint value is out of limits!");

  // Uncomment below line when working with a real robot
  group.move();

  /* Sleep to give Rviz time to visualize the plan. */
  sleep(5.0);

  ros::shutdown();
  return 0;
}



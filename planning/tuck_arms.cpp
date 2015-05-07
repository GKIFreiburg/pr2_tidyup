#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayTrajectory.h>

typedef std::map<std::string, std::vector<double> > Map;

int main(int argc, char **argv)
{

  ros::init(argc, argv, "tuck_arms");
  ros::NodeHandle node_handle;
  ros::AsyncSpinner spinner(1);
  spinner.start();

  moveit::planning_interface::MoveGroup group("left_arm");

  // We will use the :planning_scene_interface:`PlanningSceneInterface`
  // class to deal directly with the world.
  moveit::planning_interface::PlanningSceneInterface planning_scene_interface;

  // (Optional) Create a publisher for visualizing plans in Rviz.
  ros::Publisher display_publisher = node_handle.advertise<moveit_msgs::DisplayTrajectory>("/move_group/display_planned_path", 1, true);
  moveit_msgs::DisplayTrajectory display_trajectory;


  std::vector<double> right_arm_values = group.getCurrentJointValues();
  std::vector<std::string> right_arm_joints = group.getJoints();

  for (int i = 0; i < right_arm_values.size(); i++)
	  ROS_WARN("%s - %lf", right_arm_joints[i].c_str(), right_arm_values[i]);

  std::string target = "left_arm_to_front";
  if (!group.setNamedTarget(target))
	  ROS_ERROR("tuck_arms: Could not set named target: %s", target.c_str());
  else
	 ROS_INFO("tuck_arms: working!");

  std::vector<double> group_variable_values;
  const robot_state::RobotState& rs = group.getJointValueTarget();
  const robot_state::JointModelGroup* joint_model_group = rs.getJointModelGroup("right_arm");
  const std::vector<std::string> &group_variable_joints = joint_model_group->getJointModelNames();
  rs.copyJointGroupPositions(joint_model_group, group_variable_values);

	for (int i = 0; i < group_variable_values.size(); i++)
	{
		 ROS_INFO("%s - %lf", group_variable_joints[i].c_str(), group_variable_values[i]);
	}

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = group.plan(my_plan);

  ROS_INFO("Visualizing plan 1 (pose goal) %s",success?"":"FAILED");
  /* Sleep to give Rviz time to visualize the plan. */
  //sleep(5.0);

  // Visualizing plans
  // ^^^^^^^^^^^^^^^^^
  // Now that we have a plan we can visualize it in Rviz.  This is not
  // necessary because the group.plan() call we made above did this
  // automatically.  But explicitly publishing plans is useful in cases that we
  // want to visualize a previously created plan.
//  if (1)
//  {
//    ROS_INFO("Visualizing plan 1 (again)");
//    display_trajectory.trajectory_start = my_plan.start_state_;
//    display_trajectory.trajectory.push_back(my_plan.trajectory_);
//    display_publisher.publish(display_trajectory);
//    /* Sleep to give Rviz time to visualize the plan. */
//    sleep(5.0);
//  }

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
  group.move();

  ros::shutdown();
  return 0;
}


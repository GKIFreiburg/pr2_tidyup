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

  if (argc != 1 && argc != 7)
  {
    ROS_ERROR("Usage: rosrun planning pose_goal x y z <roll in °> <pitch in °> <yaw in °>");
    exit(EXIT_FAILURE);
  }

  double x, y, z, roll, pitch, yaw;

  if (argc == 1)
  {
    x = 0.28;
    y = -0.7;
    z = 1.0;
    roll = 0.0;
    pitch = 0.0;
    yaw = 0.0;
  } else {
    x = atof(argv[1]);
    y = atof(argv[2]);
    z = atof(argv[3]);
    roll = degreesToRadians(atof(argv[4]));
    pitch = degreesToRadians(atof(argv[5]));
    yaw = degreesToRadians(atof(argv[6]));
  }

  tf::Quaternion tf_q;
  geometry_msgs::Quaternion geo_q;
  tf_q.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(tf_q, geo_q);


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

  // Planning to a Pose goal
  // ^^^^^^^^^^^^^^^^^^^^^^^
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  target_pose1.orientation.w = geo_q.w;
  target_pose1.orientation.x = geo_q.x;
  target_pose1.orientation.y = geo_q.y;
  target_pose1.orientation.z = geo_q.z;
  target_pose1.position.x = x;
  target_pose1.position.y = y;
  target_pose1.position.z = z;
  group.setPoseTarget(target_pose1);

  ROS_INFO("Position: (%f, %f, %f) and Orientation: (%f, %f, %f, %f)", x, y, z, geo_q.w, geo_q.x, geo_q.y, geo_q.z);


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

  /* Uncomment below line when working with a real robot*/
  /* group.move() */

  ros::shutdown();
  return 0;
}


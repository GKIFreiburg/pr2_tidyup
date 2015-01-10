#include <ros/ros.h>

#include <moveit/move_group_interface/move_group.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>

#include <moveit_msgs/DisplayRobotState.h>
#include <moveit_msgs/DisplayTrajectory.h>

#include <math.h>
#include <geometry_msgs/Point.h>


class PathConstraint
{
  public:
    PathConstraint(moveit::planning_interface::MoveGroup* group, std::string header_frame,
        std::string link_name);
    ~PathConstraint();

    void computePathConstraint(const geometry_msgs::Point& start_position, const geometry_msgs::Point& end_position,
        const geometry_msgs::Quaternion& orientation, double tolerance = 0.1, double weight = 1) const;

  private:
    moveit::planning_interface::MoveGroup* _group;
    std::string _header_frame;
    std::string _link_name;

};

PathConstraint::PathConstraint(moveit::planning_interface::MoveGroup* group, std::string header_frame,
        std::string link_name)
{
  _group = group;
  _header_frame = header_frame;
  _link_name = link_name;
}

PathConstraint::~PathConstraint()
{
  delete _group;
}

void PathConstraint::computePathConstraint(const geometry_msgs::Point& start_position,
    const geometry_msgs::Point& end_position, const geometry_msgs::Quaternion& orientation, double tolerance, double weight) const
{
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose start_pose;
  start_pose.orientation.w = orientation.w;
  start_pose.orientation.x = orientation.x;
  start_pose.orientation.y = orientation.y;
  start_pose.orientation.z = orientation.z;
  start_pose.position.x = start_position.x;
  start_pose.position.y = start_position.y;
  start_pose.position.z = start_position.z;
  _group->setPoseTarget(start_pose);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  bool success = _group->plan(my_plan);

  ROS_INFO("Visualizing plan (pose start) %s",success?"":"FAILED");

  moveit_msgs::OrientationConstraint ocm;
  // This message contains the definition of an orientation constraint.
  ocm.header.frame_id = _header_frame;
  // The robot link this constraint refers to
  ocm.link_name = _link_name;
  // The desired orientation of the robot link specified as a quaternion
  ocm.orientation.w = orientation.w;
  ocm.orientation.x = orientation.x;
  ocm.orientation.y = orientation.y;
  ocm.orientation.z = orientation.z;

  ocm.absolute_x_axis_tolerance = tolerance;
  ocm.absolute_y_axis_tolerance = tolerance;
  ocm.absolute_z_axis_tolerance = tolerance;
  // A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
  ocm.weight = weight;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  _group->setPathConstraints(test_constraints);

  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose;
  target_pose.orientation.w = orientation.w;
  target_pose.orientation.x = orientation.x;
  target_pose.orientation.y = orientation.y;
  target_pose.orientation.z = orientation.z;
  target_pose.position.x = end_position.x;
  target_pose.position.y = end_position.y;
  target_pose.position.z = end_position.z;

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  _group->setPoseTarget(target_pose);

  success = _group->plan(my_plan);

  ROS_INFO("Visualizing plan (constraints) %s",success?"":"FAILED");

}

bool getValueFromParamServer(std::string& path, std::string& result) {

  ROS_INFO("Rosparam: %s", path.c_str());

  if (ros::param::get(path, result))
    return true;
  else
    return false;
}

double strToDouble(std::string& input) {
  double result;

  if (input == "M_PI")
    result = M_PI;
  else if (input == "-M_PI")
    result = -M_PI;
  else if (input == "M_PI/2")
    result = M_PI/2;
  else if (input == "-M_PI/2")
    result = -M_PI/2;
  else
    result = atof(input.c_str());

  return result;
}


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
  //moveit::planning_interface::MoveGroup group("right_arm");
  moveit::planning_interface::MoveGroup* group = new moveit::planning_interface::MoveGroup("right_arm");

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
  ROS_INFO("Reference frame: %s", group->getPlanningFrame().c_str());

  // We can also print the name of the end-effector link for this group.
  ROS_INFO("Reference frame: %s", group->getEndEffectorLink().c_str());

  // Planning with Path Constraints
  // ^^^^^^^^^^^^^^^^^^^^^^^^^^^^^^
  //
  // Path constraints can easily be specified for a link on the robot.
  // Let's specify a path constraint and a pose goal for our group.
  // First define the path constraint.

  // path_constraints These are optional constraints that are to be imposed along the solution path.
  // All the states along the solution path must satisfy these constraints.
  // This includes the specified start state, so the caller should make sure that the robot is currently
  // at a state that satisfies the path constraints.
  // This can be easily achieved for example by executing a plan
  // that has the desired path constraints as goal constraints.


  double roll, pitch, yaw;
  geometry_msgs::Quaternion geo_q;
  geometry_msgs::Point start_position;
  geometry_msgs::Point end_position;

  std::string path = "/pathConstraints/orientation";
  if (ros::param::has(path))
  {
    std::string result;
    std::string tmp = path;

    tmp = path + "/roll";
    if (getValueFromParamServer(tmp, result))
      roll = strToDouble(result);

    tmp = path + "/pitch";
    if (getValueFromParamServer(tmp, result))
      pitch = strToDouble(result);

    tmp = path + "/yaw";
    if (getValueFromParamServer(tmp, result))
      yaw = strToDouble(result);

    path = "/pathConstraints/start_position";
    tmp = path + "/x";
    if (getValueFromParamServer(tmp, result))
      start_position.x = strToDouble(result);

    tmp = path + "/y";
    if (getValueFromParamServer(tmp, result))
      start_position.y = strToDouble(result);

    tmp = path + "/z";
    if (getValueFromParamServer(tmp, result))
      start_position.z = strToDouble(result);

    path = "/pathConstraints/end_position";
    tmp = path + "/x";
    if (getValueFromParamServer(tmp, result))
      end_position.x = strToDouble(result);

    tmp = path + "/y";
    if (getValueFromParamServer(tmp, result))
      end_position.y = strToDouble(result);

    tmp = path + "/z";
    if (getValueFromParamServer(tmp, result))
      end_position.z = strToDouble(result);

  } else {

    roll = 0;
    pitch = -M_PI/2;
    yaw = 0;

    start_position.x = 0.28;
    start_position.y = -0.7;
    start_position.z = 1.0;

    end_position.x = 0;
    end_position.y = -0.7;
    end_position.z = 0.5;
  }

  tf::Quaternion tf_q;
  tf_q.setRPY(roll, pitch, yaw);
  tf::quaternionTFToMsg(tf_q, geo_q);

  tf::Matrix3x3(tf_q).getRPY(roll, pitch, yaw);
  ROS_INFO("roll = %f", roll);
  ROS_INFO("pitch = %f", pitch);
  ROS_INFO("yaw = %f", yaw);

  ROS_INFO("start_position (x, y, z) = (%f, %f, %f)", start_position.x, start_position.y, start_position.z);
  ROS_INFO("end_position (x, y, z) = (%f, %f, %f)", end_position.x, end_position.y, end_position.z);

  PathConstraint pc(group, "base_link", "r_wrist_roll_link");
  pc.computePathConstraint(start_position, end_position, geo_q);

  // When done with the path constraint be sure to clear it.
  group->clearPathConstraints();
  //END_TUTORIAL

  ros::shutdown();
  return 0;
}

/*
  bool success;
  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose1;
  ROS_INFO("orientation w = %f", q.getW());
  ROS_INFO("orientation x = %f", q.getX());
  ROS_INFO("orientation y = %f", q.getY());
  ROS_INFO("orientation z = %f", q.getZ());
  target_pose1.orientation.w = q.getW();
  target_pose1.orientation.x = q.getX();
  target_pose1.orientation.y = q.getY();
  target_pose1.orientation.z = q.getZ();
  target_pose1.position.x = 0.28;
  target_pose1.position.y = -0.7;
  target_pose1.position.z = 1.0;
  group.setPoseTarget(target_pose1);


  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan (pose start) %s",success?"":"FAILED");


  moveit_msgs::OrientationConstraint ocm;
  // This message contains the definition of an orientation constraint.
  ocm.header.frame_id = "base_link";
  // The robot link this constraint refers to
  ocm.link_name = "r_wrist_roll_link";
  // The desired orientation of the robot link specified as a quaternion
  ocm.orientation.w = q.getW();
  ocm.orientation.x = q.getX();
  ocm.orientation.y = q.getY();
  ocm.orientation.z = q.getZ();

//  ocm.orientation.w = 0.5;
  ocm.absolute_x_axis_tolerance = 0.1;
  ocm.absolute_y_axis_tolerance = 0.1;
  ocm.absolute_z_axis_tolerance = 0.1;
  // A weighting factor for this constraint (denotes relative importance to other constraints. Closer to zero means less important)
  ocm.weight = 1.0;

  // Now, set it as the path constraint for the group.
  moveit_msgs::Constraints test_constraints;
  test_constraints.orientation_constraints.push_back(ocm);
  group.setPathConstraints(test_constraints);



//  // We will reuse the old goal that we had and plan to it.
//  // Note that this will only work if the current state already
//  // satisfies the path constraints. So, we need to set the start
//  // state to a new pose.
//  robot_state::RobotState start_state(*group.getCurrentState());
//  geometry_msgs::Pose start_pose2;
//  start_pose2.orientation.w = 1.0;
//  start_pose2.position.x = 0.55;
//  start_pose2.position.y = -0.05;
//  start_pose2.position.z = 0.8;
//  const robot_state::JointModelGroup *joint_model_group =
//                  start_state.getJointModelGroup(group.getName());
//  start_state.setFromIK(joint_model_group, start_pose2);
//  group.setStartState(start_state);


  // We can plan a motion for this group to a desired pose for the
  // end-effector.
  geometry_msgs::Pose target_pose2;
  target_pose2.orientation.w = q.getW();
  target_pose2.orientation.x = q.getX();
  target_pose2.orientation.y = q.getY();
  target_pose2.orientation.z = q.getZ();
  target_pose2.position.x = 0;
  target_pose2.position.y = -0.7;
  target_pose2.position.z = 0.5;

  // Now we will plan to the earlier pose target from the new
  // start state that we have just created.
  group.setPoseTarget(target_pose2);

  // Now, we call the planner to compute the plan
  // and visualize it.
  // Note that we are just planning, not asking move_group
  // to actually move the robot.
  moveit::planning_interface::MoveGroup::Plan my_plan;
  success = group.plan(my_plan);

  ROS_INFO("Visualizing plan (constraints) %s",success?"":"FAILED");
  // Sleep to give Rviz time to visualize the plan.
  sleep(10.0);

  // When done with the path constraint be sure to clear it.
  group.clearPathConstraints();
*/

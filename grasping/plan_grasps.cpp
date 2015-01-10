// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Grasp generation and visualization
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_visual_tools/visual_tools.h>

// Grasp generator
moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

// class for publishing stuff to rviz
moveit_visual_tools::VisualToolsPtr visual_tools_;

// robot-specific data for generating grasps
moveit_simple_grasps::GraspData grasp_data_;

int main(int argc, char **argv)
{
  ros::init(argc, argv, "plan_grasps");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // Load the Robot Viz Tools for publishing to Rviz
  visual_tools_.reset(new moveit_visual_tools::VisualTools("base_link"));

  // Load grasp data specific to our robot
  ros::NodeHandle nh("~"); // private namespace
  if (!grasp_data_.loadRobotGraspData(nh, "/left_gripper"))
    ros::shutdown();
  // Load grasp generator
  simple_grasps_.reset( new moveit_simple_grasps::SimpleGrasps(visual_tools_) );

  // Let everything load
  ros::Duration(1.0).sleep();

  geometry_msgs::Pose object_pose;
  object_pose.position.x = 0.4;
  object_pose.position.y = -0.4;
  object_pose.position.z = 0.55;

  // Orientation
  double angle = M_PI / 1.5;
  Eigen::Quaterniond quat(Eigen::AngleAxis<double>(double(angle), Eigen::Vector3d::UnitZ()));
  object_pose.orientation.x = quat.x();
  object_pose.orientation.y = quat.y();
  object_pose.orientation.z = quat.z();
  object_pose.orientation.w = quat.w();

  // visualize block in rviz
  double size = 0.1; // height=width=length
  visual_tools_->publishBlock(object_pose, moveit_visual_tools::BLUE, size);

  std::vector<moveit_msgs::Grasp> possible_grasps;
  simple_grasps_->generateBlockGrasps( object_pose, grasp_data_, possible_grasps);

  ROS_INFO("Number of possible grasps: %lu", possible_grasps.size());

  ROS_INFO("EE parent link: %s", grasp_data_.ee_parent_link_.c_str());

  grasp_data_.print();
  //visual_tools_->publishAnimatedGrasps(possible_grasps, grasp_data_.ee_parent_link_);
  visual_tools_->publishGrasps(possible_grasps, grasp_data_.ee_parent_link_);


  ros::shutdown();
  return 0;
/*
  // filter grasps
  // Filter the grasp for only the ones that are reachable
  bool filter_pregrasps = true;
  std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization
  grasp_filter_->filterGrasps(possible_grasps, ik_solutions, filter_pregrasps, grasp_data_.ee_parent_link_, planning_group_name_);
  
  visual_tools_->publishIKSolutions(ik_solutions, planning_group_name_, 0.25);
*/
}




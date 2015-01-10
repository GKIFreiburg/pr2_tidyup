#ifndef MOVEOBJECT_H_
#define MOVEOBJECT_H_

// ROS
#include <ros/ros.h>

// MoveIt!
#include <moveit/move_group_interface/move_group.h>

// Grasp generation
#include <moveit_simple_grasps/simple_grasps.h>
#include <moveit_simple_grasps/grasp_data.h>
#include <moveit_visual_tools/visual_tools.h> // simple tool for showing grasps

#include <fstream> // for dumping (debugging)
#include <sys/stat.h>
#include <sys/types.h>
#include <boost/filesystem.hpp>

struct MetaBlock
{
  std::string name;
  geometry_msgs::Pose pose;
};


class MoveObject
{
public:
  // Constructor
  MoveObject();

  // Create block and visualize it in Rviz
  MetaBlock createBlock(const std::string &name, double x, double y, double z,
                          double roll = 0, double pitch = 0, double yaw = 0, double size = 0.04);

  // Create can (approximated by a box) and visualize it in Rviz
  // x, y, z represent the center of the can!
  MetaBlock createCan(const std::string &name, double x, double y, double z,
                          double roll = 0, double pitch = 0, double yaw = 0,
                          double length = 0.034, double width = 0.034, double height = 0.06);

  // Compute grasps for the object and use move group to pick it up
  bool pick(const geometry_msgs::Pose& block_pose, const std::string& block_name);

private:

  // dump all the grasps into a seperate file, named after its grasp id.
  void dumpGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps);

  // grasp generator
  moveit_simple_grasps::SimpleGraspsPtr simple_grasps_;

  moveit_visual_tools::VisualToolsPtr visual_tools_;

  // data for generating grasps
  moveit_simple_grasps::GraspData grasp_data_;

  // our interface with MoveIt
  boost::scoped_ptr<move_group_interface::MoveGroup> move_group_;

  // which arm are we using
  std::string ee_group_name_;
  std::string planning_group_name_;

  // counts how many pick_places have run
  int pick_place_count_;

  // private nodehandle
  ros::NodeHandle nh_;

};


#endif /* MOVEOBJECT_H_ */

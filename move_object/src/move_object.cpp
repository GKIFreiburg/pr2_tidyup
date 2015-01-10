#include <move_object/move_object.h>

MoveObject::MoveObject()
  : nh_("~"),
    ee_group_name_("left_gripper"),
    planning_group_name_("left_arm"),
    pick_place_count_(0)
{
  ROS_INFO("End Effector: %s", ee_group_name_.c_str());
  ROS_INFO("Planning Group: %s", planning_group_name_.c_str());

  // Create MoveGroup for one of the planning groups
  move_group_.reset(new move_group_interface::MoveGroup(planning_group_name_));
  move_group_->setPlanningTime(30.0);

  // Load grasp generator
  if (!grasp_data_.loadRobotGraspData(nh_, ee_group_name_))
    ros::shutdown();

  grasp_data_.print();

  // Load the Robot Viz Tools for publishing to rviz
  visual_tools_.reset(new moveit_visual_tools::VisualTools( grasp_data_.base_link_));
  //visual_tools_->setFloorToBaseHeight(-0.6);
  visual_tools_->print();
  visual_tools_->loadEEMarker(grasp_data_.ee_group_, planning_group_name_);

  // Load grasp generator
  //simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_));
  simple_grasps_.reset(new moveit_simple_grasps::SimpleGrasps(visual_tools_, true));

  // Let everything load
  ros::Duration(1.0).sleep();
}

MetaBlock MoveObject::createBlock(const std::string &name, double x, double y, double z,
    double roll, double pitch, double yaw, double size)
{
  MetaBlock block;
  block.name = name;

  // Create the pose of block
  geometry_msgs::Pose pose;

  // Position
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  // Orientation
  tf::Quaternion tf_q;
  geometry_msgs::Quaternion geo_q;

  tf_q.setRPY(roll, pitch, yaw);
  // Convert tf quaternion into geometry_msgs quaternion
  tf::quaternionTFToMsg(tf_q, geo_q);

  pose.orientation.w = geo_q.w;
  pose.orientation.x = geo_q.x;
  pose.orientation.y = geo_q.y;
  pose.orientation.z = geo_q.z;

  block.pose = pose;

  // visualize block in rviz
  visual_tools_->publishBlock(pose, moveit_visual_tools::BLUE, size);

  return block;
}

MetaBlock MoveObject::createCan(const std::string &name, double x, double y, double z,
                          double roll, double pitch, double yaw,
                          double length, double width, double height)
{
  MetaBlock block;
  block.name = name;

  // Create the pose of block
  geometry_msgs::Pose pose;

  // Position
  pose.position.x = x;
  pose.position.y = y;
  pose.position.z = z;

  // Orientation
  tf::Quaternion tf_q;
  geometry_msgs::Quaternion geo_q;

  tf_q.setRPY(roll, pitch, yaw);
  // Convert tf quaternion into geometry_msgs quaternion
  tf::quaternionTFToMsg(tf_q, geo_q);

  pose.orientation.w = geo_q.w;
  pose.orientation.x = geo_q.x;
  pose.orientation.y = geo_q.y;
  pose.orientation.z = geo_q.z;

  block.pose = pose;

  // Create rectangle for display.
  // Point1 is top corner of can
  // Point2 is bottom opposite corner of can

  geometry_msgs::Point point1;
  point1.x = x + length / 2;
  point1.y = y + width / 2;
  point1.z = z + height / 2;

  geometry_msgs::Point point2;
  point2.x = x - length / 2;
  point2.y = y - width / 2;
  point2.z = z - height / 2;

  ROS_INFO_STREAM("Point1 \n" << point1 << "\n" << "Point2 \n" << point2);

  // visualize block in rviz
  visual_tools_->publishRectangle(point1, point2, moveit_visual_tools::BLUE);

  return block;
}

bool MoveObject::pick(const geometry_msgs::Pose& block_pose, const std::string& block_name)
{
  std::vector<moveit_msgs::Grasp> possible_grasps;

  // Pick grasp
  simple_grasps_->generateBlockGrasps( block_pose, grasp_data_, possible_grasps );

  // Visualize them
  visual_tools_->publishGrasps(possible_grasps, grasp_data_.ee_parent_link_);

  // dumpGrasps(possible_grasps);

/*
  robot_state::RobotState robot_state(move_group_->getCurrentState()); // selwer geschriwwen
  // Filter the grasp for only the ones that are reachable
  bool filter_pregrasps = true;
  std::vector<trajectory_msgs::JointTrajectoryPoint> ik_solutions; // save each grasps ik solution for visualization
  grasp_filter_->filterGrasps(possible_grasps, ik_solutions, filter_pregrasps, grasp_data_.ee_parent_link_, planning_group_name_);
*/
  return move_group_->pick(block_name, possible_grasps);
  //return move_group_->pick(block_name, possible_grasps[0]);
}

void MoveObject::dumpGrasps(const std::vector<moveit_msgs::Grasp>& possible_grasps)
{

  std::string fileName;
  boost::filesystem::path path("/home/luc/drump_grasp");

  if (boost::filesystem::exists(path))
  {
    ROS_INFO("Folder already exists. Do nothing.");
    return;
  }
  else
  {
    ROS_INFO("Create Folder and start the dump.");
    boost::filesystem::create_directory(path);

    std::ofstream outputFile;
    for (size_t i = 0; i < possible_grasps.size(); i++)
    {
      fileName = path.string() + "/" + possible_grasps[i].id;
      ROS_INFO("Dumping into: %s", fileName.c_str());
      outputFile.open(fileName.c_str());
      outputFile << possible_grasps[i];
      outputFile.close();
    }
    ROS_INFO("Dumping done.");
  }
}



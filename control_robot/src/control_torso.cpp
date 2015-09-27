#include "control_robot/control_torso.h"
#include <stdlib.h>

namespace control_robot
{
	ControlTorso::ControlTorso(moveit::planning_interface::MoveGroup* torso_group) :
		torso_group_(torso_group),
		torso_("torso_lift_joint")
	{
		ros::NodeHandle nhPriv("~");
		srvTorsoLiftMax_ = nhPriv.advertiseService(
				"torso_lift_max", &ControlTorso::torsoLiftMax, this);
		srvTorsoLiftMin_ = nhPriv.advertiseService(
				"torso_lift_min", &ControlTorso::torsoLiftMin, this);
		srvTorsoLift_ = nhPriv.advertiseService(
						"torso_lift", &ControlTorso::torsoLift, this);

	    ROS_INFO("Waiting for %s services.", ros::this_node::getName().c_str());
	    ros::Duration timeout = ros::Duration(0.5);
	    // If one service is not online, we get an info message
	    ros::service::waitForService("control_robot/torso_lift_max", timeout);
	    ros::service::waitForService("control_robot/torso_lift_min", timeout);
	    ros::service::waitForService("control_robot/torso_lift", timeout);

	    ROS_INFO("Torso control services are Ready!");
	}

	ControlTorso::~ControlTorso()
	{
	}

	bool ControlTorso::torsoLiftMax(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		std::vector<std::string> torso_joints = torso_group_->getJoints();
		ROS_ASSERT(torso_joints.size() > 0);
//		double max_position = torso_group_->getCurrentState().get()->getJointModel(torso_joints[0])->getVariableBounds()[0].max_position_;
		double max_position = 0.3;
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveTorsoJointPosition(max_position);
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlTorso::torsoLiftMin(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		std::vector<std::string> torso_joints = torso_group_->getJoints();
		ROS_ASSERT(torso_joints.size() > 0);
		double min_position = torso_group_->getCurrentState().get()->getJointModel(torso_joints[0])->getVariableBounds()[0].min_position_;
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveTorsoJointPosition(min_position);
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlTorso::torsoLift(control_robot_msgs::MoveItPosition::Request &req,
			control_robot_msgs::MoveItPosition::Response &res)
	{
		double position = req.position.data;
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveTorsoJointPosition(position);
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	moveit::planning_interface::MoveItErrorCode ControlTorso::moveTorsoJointPosition(double position)
	{
		std::vector<std::string> torso_joints = torso_group_->getJoints();
		ROS_ASSERT(torso_joints.size() > 0);
		if (!torso_group_->setJointValueTarget(torso_joints[0], position))
		{
			ROS_ERROR("Position out of bounds - not moving torso");
			return moveit::planning_interface::MoveItErrorCode::FAILURE;
		}
		return torso_group_->move();
	}

};

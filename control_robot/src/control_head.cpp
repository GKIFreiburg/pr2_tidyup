#include "control_robot/control_head.h"
#include <stdlib.h>

namespace control_robot
{
	ControlHead::ControlHead(moveit::planning_interface::MoveGroup* head_group) :
		head_group_(head_group),
		head_pitch_("head_tilt_joint"),
		head_yaw_("head_pan_joint"),
		head_step_(0.15)
	{
	}

	ControlHead::~ControlHead()
	{
	}

	bool ControlHead::headPitchDown(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveHeadJointByStep(head_pitch_, +head_step_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlHead::headPitchUp(control_robot_msgs::MoveIt::Request &req,
					control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveHeadJointByStep(head_pitch_, -head_step_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlHead::headPitchStraight(control_robot_msgs::MoveIt::Request &req,
					control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveHeadJointToJointValue(head_pitch_, 0);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlHead::headPitchDegrees(control_robot_msgs::MoveItDegrees::Request &req,
			control_robot_msgs::MoveItDegrees::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		double rad = degreesToRadians( (double) req.degrees.data );
		error_code = moveHeadJointToJointValue(head_pitch_, rad);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlHead::headYawLeft(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveHeadJointByStep(head_yaw_, +head_step_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlHead::headYawRight(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveHeadJointByStep(head_yaw_, -head_step_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlHead::headYawStraight(control_robot_msgs::MoveIt::Request &req,
					control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveHeadJointToJointValue(head_yaw_, 0);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlHead::headYawDegrees(control_robot_msgs::MoveItDegrees::Request &req,
			control_robot_msgs::MoveItDegrees::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		double rad = degreesToRadians( (double) req.degrees.data );
		error_code = moveHeadJointToJointValue(head_yaw_, rad);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlHead::headInitialPosition(control_robot_msgs::MoveIt::Request &req,
					control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;

		std::map<std::string, double> jointValues;
		jointValues.insert( std::make_pair<std::string, double>(head_pitch_, 0) );
		jointValues.insert( std::make_pair<std::string, double>(head_yaw_, 0) );
		error_code = moveHeadJoints( jointValues );

		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	moveit::planning_interface::MoveItErrorCode ControlHead::moveHeadJointByStep(
			const std::string joint, const double step)
	{
		double value;
		if (!getCurrentJointValueOfJoint(joint, value))
			return false;

		value += step;
		return moveHeadJointToJointValue(joint, value);
	}

	bool ControlHead::getCurrentJointValueOfJoint(const std::string jointName, double &value)
	{
		// First get the current set of joint values for the group.
		std::vector<double> group_variable_values;
		group_variable_values = head_group_->getCurrentJointValues();

		std::vector<std::string> jointNames = head_group_->getJoints();
		for (std::size_t i = 0; i < jointNames.size(); i++)
		{
			std::string joint = jointNames[i];
			if (joint.compare(jointName) == 0)
			{
				value = group_variable_values[i];
				return true;
			}
		}

		ROS_ERROR("ControlHead::%s: Could not load corresponding joint name for %s",
				__func__, jointName.c_str());
		return false;
	}

	moveit::planning_interface::MoveItErrorCode ControlHead::moveHeadJointToJointValue(
			const std::string jointName, const double value)
	{
		std::map<std::string, double> jointValues;
		std::pair<std::string, double> jointValue = std::make_pair(jointName, value);
		jointValues.insert( jointValue );

		head_group_->setJointValueTarget( jointValues );
		return head_group_->move();
	}

	moveit::planning_interface::MoveItErrorCode ControlHead::moveHeadJoints(
			const std::map<std::string, double> jointValues)
	{
		head_group_->setJointValueTarget( jointValues );
		return head_group_->move();
	}
};

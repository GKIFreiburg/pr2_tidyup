#include "control_robot/control_arms.h"
#include <stdlib.h>
#include <boost/assign/list_of.hpp>

namespace control_robot
{
	ControlArms::ControlArms(moveit::planning_interface::MoveGroup* right_arm_group,
			moveit::planning_interface::MoveGroup* left_arm_group,
			moveit::planning_interface::MoveGroup* arms_group)	:
		right_arm_group_(right_arm_group),
		left_arm_group_(left_arm_group),
		arms_group_(arms_group)
	{
		// hard coded values which were measured.
		right_arm_at_side_values_ = boost::assign::list_of(-2.110)(1.230)(-2.06)(-1.69)(0.3)(-1.32)(1.57);
		right_arm_at_front_values_ = boost::assign::list_of(0.0)(0.065)(0.0)(-0.15)(0.0)(-0.13)(0.0);
		right_arm_at_front_bent_values_ = boost::assign::list_of(0.0)(0.0)(0.0)(-1.350)(0.0)(-1.050)(0.0);

		left_arm_at_side_values_ = boost::assign::list_of(2.110)(1.230)(2.06)(-1.69)(-0.3)(-1.32)(1.57);
		left_arm_at_front_values_ = boost::assign::list_of(0.0)(0.065)(0.0)(-0.15)(0.0)(-0.13)(0.0);
		left_arm_at_front_bent_values_ = boost::assign::list_of(0.0)(0.0)(0.0)(-1.35)(0.0)(-1.05)(0.0);

		// Fill the arms vector with the values of the particular arms (first left, then right arm)
		std::vector<double>::iterator it;
		arms_at_side_values_.insert(arms_at_side_values_.begin(), left_arm_at_side_values_.begin(), left_arm_at_side_values_.end());
		arms_at_side_values_.insert(arms_at_side_values_.end(), right_arm_at_side_values_.begin(), right_arm_at_side_values_.end());
		arms_at_front_values_.insert(arms_at_front_values_.begin(), left_arm_at_front_values_.begin(), left_arm_at_front_values_.end());
		arms_at_front_values_.insert(arms_at_front_values_.end(), right_arm_at_front_values_.begin(), right_arm_at_front_values_.end());
		arms_at_front_bent_values_.insert(arms_at_front_bent_values_.begin(), left_arm_at_front_bent_values_.begin(), left_arm_at_front_bent_values_.end());
		arms_at_front_bent_values_.insert(arms_at_front_bent_values_.end(), right_arm_at_front_bent_values_.begin(), right_arm_at_front_bent_values_.end());
	}

	ControlArms::~ControlArms()
	{
	}

	bool ControlArms::armsToSide(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveArms(arms_group_, arms_at_side_values_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlArms::armsToFront(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveArms(arms_group_, arms_at_front_values_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlArms::armsToFrontBent(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveArms(arms_group_, arms_at_front_bent_values_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlArms::rightArmToSide(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveArms(right_arm_group_, right_arm_at_side_values_);
		//printValues(right_arm_at_side_values_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlArms::rightArmToFront(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveArms(right_arm_group_, right_arm_at_front_values_);
		//printValues(right_arm_at_front_values_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlArms::rightArmToFrontBent(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveArms(right_arm_group_, right_arm_at_front_bent_values_);
		//printValues(right_arm_at_front_bent_values_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlArms::leftArmToSide(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveArms(left_arm_group_, left_arm_at_side_values_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlArms::leftArmToFront(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveArms(left_arm_group_, left_arm_at_front_values_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	bool ControlArms::leftArmToFrontBent(control_robot_msgs::MoveIt::Request &req,
			control_robot_msgs::MoveIt::Response &res)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		error_code = moveArms(left_arm_group_, left_arm_at_front_bent_values_);
		res.error_code = error_code;
		return error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS;
	}

	moveit::planning_interface::MoveItErrorCode ControlArms::moveArms(
		moveit::planning_interface::MoveGroup* group,
		const std::map<std::string, double> jointValues)
	{
		group->setJointValueTarget( jointValues );
		return group->move();
	}

	moveit::planning_interface::MoveItErrorCode ControlArms::moveArms(
		moveit::planning_interface::MoveGroup* group,
		const std::vector<double> jointValues)
	{
		group->setJointValueTarget( jointValues );
		return group->move();
	}

	void ControlArms::printValues(std::vector<double> values)
	{
		for (std::size_t i = 0; i < values.size(); i++)
			ROS_INFO("[%lu] [%lf]", i, values[i]);
	}
};




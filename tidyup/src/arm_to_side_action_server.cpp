#include "tidyup/arm_to_side_action_server.h"
#include "tidyup/arms_at_side_service_server.h"
#include <boost/lexical_cast.hpp>

#include <ros/ros.h>

namespace tidyup
{
	ArmToSideActionServer::ArmToSideActionServer(ros::NodeHandle nh, std::string name, moveit::planning_interface::MoveGroup* right_arm_group,
			moveit::planning_interface::MoveGroup* left_arm_group)	:
			nhPriv_(nh),
			as_(nhPriv_, name, boost::bind(&ArmToSideActionServer::executeArmToSide, this, _1), false),
			action_name_(name),
			right_arm_group_(right_arm_group),
			left_arm_group_(left_arm_group)
	{
		as_.start();
	}

	ArmToSideActionServer::~ArmToSideActionServer()
	{
	}

	void ArmToSideActionServer::executeArmToSide(const tidyup_msgs::ArmToSideGoalConstPtr &goal)
	{
		moveit::planning_interface::MoveItErrorCode error_code;
		if (goal->left_arm == true)
			error_code = armToSide(left_arm_group_);
		else if (goal->right_arm == true)
			error_code = armToSide(right_arm_group_);

		// execution was successful
		if (error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			result_.result = "Execution of arm movement was successful, MoveItErrorCode: " + boost::lexical_cast<std::string>(error_code.val);
			as_.setSucceeded(result_);
		}

		else // something went wrong
		{
			result_.result = "Execution of arm movement failed, MoveitErrorCode: " + boost::lexical_cast<std::string>(error_code.val);
			as_.setAborted(result_);
		}
	}

	moveit::planning_interface::MoveItErrorCode ArmToSideActionServer::armToSide(moveit::planning_interface::MoveGroup* group)
	{
		// get the joint values for arm side position from param server
		std::vector<double> jointValues;
		tidyup::ArmsAtSideServiceServer::loadJointValues(group->getName(), jointValues);

		ROS_ASSERT(jointValues.size() > 0);

        ROS_INFO("setting joint values");
		group->setJointValueTarget(jointValues);

		moveit::planning_interface::MoveItErrorCode error_code;

		// Call the planner to compute a plan.
		// Note that we are just planning, not asking move_group
		// to actually move the robot.
		moveit::planning_interface::MoveGroup::Plan my_plan;
        ROS_INFO("planning arm motion...");

        error_code = group->plan(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ArmToSideActionServer::%s: Ups, something with arm motion planning went wrong.", __func__);
			return error_code;
		}

		// planning was successful
		error_code = group->execute(my_plan);
		if (error_code != moveit::planning_interface::MoveItErrorCode::SUCCESS)
		{
			ROS_WARN("ArmToSideActionServer::%s: Ups, something with arm motion execution went wrong.", __func__);
			return error_code;
		}

		// error_code == moveit::planning_interface::MoveItErrorCode::SUCCESS
		return error_code;
	}

};




#include "tidyup/arm_to_side_action_server.h"
#include "tidyup/arms_at_side_service_server.h"

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
		if (goal->left_arm == true){
			// get the joint values for arm side position from param server
			std::vector<double> jointValues;
			tidyup::ArmsAtSideServiceServer::loadJointValues(left_arm_group_->getName(), jointValues);

			ROS_ASSERT(jointValues.size() > 0);

            ROS_INFO("setting joint values");
			left_arm_group_->setJointValueTarget(jointValues);

			// Call the planner to compute a plan.
			// Note that we are just planning, not asking move_group
			// to actually move the robot.
			moveit::planning_interface::MoveGroup::Plan my_plan;
            ROS_INFO("planning arm motion...");
            moveit::planning_interface::MoveItErrorCode error_code = left_arm_group_->plan(my_plan);
			if (!moveit::planning_interface::MoveItErrorCode::SUCCESS == error_code)
				ROS_WARN("ArmToSide::%s:Ups, something with planning went wrong.", __func__);

			left_arm_group_->move();

		} else if (goal->right_arm == true){
			// get the joint values for arm side position from param server
			std::vector<double> jointValues;
			tidyup::ArmsAtSideServiceServer::loadJointValues(right_arm_group_->getName(), jointValues);
			ROS_ASSERT(jointValues.size() > 0);

            ROS_INFO("setting joint values");
			right_arm_group_->setJointValueTarget(jointValues);

			// Call the planner to compute a plan.
			// Note that we are just planning, not asking move_group
			// to actually move the robot.
			moveit::planning_interface::MoveGroup::Plan my_plan;
            ROS_INFO("planning arm motion...");
            moveit::planning_interface::MoveItErrorCode error_code = right_arm_group_->plan(my_plan);
			if (!moveit::planning_interface::MoveItErrorCode::SUCCESS == error_code)
				ROS_WARN("ArmToSide::%s:Ups, something with planning went wrong.", __func__);

			right_arm_group_->move();
		}

		result_.result = "This works";
		// set the action state to succeeded
		as_.setSucceeded(result_);
	}

};




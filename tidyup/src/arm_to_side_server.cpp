#include "tidyup/arm_to_side_server.h"
#include "tidyup/arms_at_side.h"

#include <ros/ros.h>

namespace tidyup
{
	ArmToSideServer::ArmToSideServer(ros::NodeHandle nh, std::string name, moveit::planning_interface::MoveGroup* right_arm_group,
			moveit::planning_interface::MoveGroup* left_arm_group)	:
			nhPriv_(nh),
			as_(nhPriv_, name, boost::bind(&ArmToSideServer::executeArmToSide, this, _1), false),
			action_name_(name),
			right_arm_group_(right_arm_group),
			left_arm_group_(left_arm_group)
	{
		as_.start();
	}

	ArmToSideServer::~ArmToSideServer()
	{
	}

	void ArmToSideServer::executeArmToSide(const tidyup_msgs::ArmToSideGoalConstPtr &goal)
	{
		if (goal->left_arm == true){
			// get the joint values for arm side position from param server
			std::vector<double> jointValues;
			tidyup::ArmsAtSide::loadJointValues(left_arm_group_->getName(), jointValues);

			for (int i = 0; i < jointValues.size(); i++){
				ROS_WARN("Joint[%d] = %f", i , jointValues[i]);
			}

			ROS_ASSERT(jointValues.size() > 0);

			left_arm_group_->setJointValueTarget(jointValues);
			sleep(1.0);
			left_arm_group_->move();
			sleep(5.0);
			ROS_ERROR(":O");

		} else if (goal->right_arm == true){
			// get the joint values for arm side position from param server
			std::vector<double> jointValues;
			tidyup::ArmsAtSide::loadJointValues(right_arm_group_->getName(), jointValues);
			ROS_ASSERT(jointValues.size() > 0);

			right_arm_group_->setJointValueTarget(jointValues);
			right_arm_group_->move();
		}

		result_.result = "This works";
		// set the action state to succeeded
		as_.setSucceeded(result_);
	}

};




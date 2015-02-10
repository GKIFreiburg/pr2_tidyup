#include "tidyup/arms_at_side.h"
#include <stdlib.h>

namespace tidyup
{
	armsAtSide::armsAtSide(moveit::planning_interface::MoveGroup* right_arm_group,
					moveit::planning_interface::MoveGroup* left_arm_group) :
		right_arm_group_(right_arm_group),
		left_arm_group_(left_arm_group)
	{
	}

	armsAtSide::~armsAtSide()
	{
	}

	bool armsAtSide::loadJointValues(const std::string &arm_group_name, std::vector<double> &arm_joints) const
	{
		ros::NodeHandle nhPriv("~");
		if (!nhPriv.hasParam(arm_group_name))
			return false;
		//ROS_ASSERT(nhPriv.hasParam(group_name));

		XmlRpc::XmlRpcValue joint_list;
		nhPriv.getParam(arm_group_name, joint_list);
		if (joint_list.getType() != XmlRpc::XmlRpcValue::TypeArray)
			return false;
		//ROS_ASSERT(joint_list.getType() == XmlRpc::XmlRpcValue::TypeArray);

		for (int32_t i = 0; i < joint_list.size(); ++i)
		{
			if (joint_list[i].getType() != XmlRpc::XmlRpcValue::TypeDouble)
				return false;
			//ROS_ASSERT(joint_list[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
			arm_joints.push_back(static_cast<double>(joint_list[i]));
		}

		return true;
	}

	bool armsAtSide::checkIfArmsAtSide(tidyup_msgs::ArmsAtSide::Request &req,
									tidyup_msgs::ArmsAtSide::Response &res)
	{
		// First get the current set of joint values for the groups.
		std::vector<double> right_arm_values = right_arm_group_->getCurrentJointValues();
		std::vector<double> left_arm_values = left_arm_group_->getCurrentJointValues();


		std::vector<double> right_arm_at_side;
		std::vector<double> left_arm_at_side;

		if (!loadJointValues(left_arm_group_->getName(), left_arm_at_side) &
				!loadJointValues(right_arm_group_->getName(), right_arm_at_side))
		{
			ROS_WARN("armsAtSide::%s: Failed to load joint values from param server, take hard coded values.", __func__);
			double values[] = {-2.110, 1.230, -2.06, -1.69, 0.3, -1.32, 1.57};
			std::vector<double> tmp(values, values + sizeof(values) / sizeof(double));
			right_arm_at_side = tmp;

			double lvalues[] = {2.110, 1.230, 2.06, -1.69, -0.3, -1.32, 1.57};
			std::vector<double> tmp2(lvalues, lvalues + sizeof(lvalues) / sizeof(double));
			left_arm_at_side = tmp2;
		}

		bool right_arm = true;
		bool left_arm = true;

		ROS_ASSERT(right_arm_at_side.size() > 0);
		ROS_ASSERT(left_arm_at_side.size() > 0);

		// Error needed since joints never reach the exactly given values.
		double error = 0.01;
		for (int i = 0; i < right_arm_at_side.size(); i++)
		{
			// ROS_INFO("armsAtSide::%s: right arm value: %f - %f", __func__ ,right_arm_values[i], right_arm_at_side[i]);
			if (std::abs((double)right_arm_values[i] - (double)right_arm_at_side[i]) > error)
				right_arm = false;
		}

		for (int i = 0; i < left_arm_at_side.size(); i++)
		{
			// ROS_INFO("armsAtSide::%s: left arm value: %f - %f", __func__, left_arm_values[i], left_arm_at_side[i]);
			if (std::abs((double)left_arm_values[i] - (double)left_arm_at_side[i]) > error)
				left_arm = false;
		}

		res.right_arm = right_arm;
		res.left_arm = left_arm;

		return true;
	}
};



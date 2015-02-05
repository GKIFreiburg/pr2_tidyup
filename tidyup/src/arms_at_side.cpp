#include "tidyup/arms_at_side.h"
#include <moveit/move_group_interface/move_group.h>
#include <stdlib.h>

namespace tidyup
{
	armsAtSide::armsAtSide()
	{
	}

	armsAtSide::~armsAtSide()
	{
	}

	bool armsAtSide::checkIfArmsAtSide(tidyup_msgs::ArmsAtSide::Request &req,
									tidyup_msgs::ArmsAtSide::Response &res)
	{
		moveit::planning_interface::MoveGroup right_arm_group("right_arm");
		moveit::planning_interface::MoveGroup left_arm_group("left_arm");

		// First get the current set of joint values for the groups.
		std::vector<double> right_arm_values = right_arm_group.getCurrentJointValues();
		std::vector<double> left_arm_values = left_arm_group.getCurrentJointValues();

		// TODO: read from param
		double values[] = {-2.110, 1.230, -2.06, -1.69, 0.3, -1.32, 1.57};
		std::vector<double> right_arm_at_side(values, values + sizeof(values) / sizeof(double));

		double lvalues[] = {2.110, 1.230, 2.06, -1.69, -0.3, -1.32, 1.57};
		std::vector<double> left_arm_at_side(lvalues, lvalues + sizeof(lvalues) / sizeof(double));

		bool right_arm = true;
		bool left_arm = true;

		double error = 0.01;
		for (int i = 0; i < right_arm_at_side.size(); i++)
		{
			ROS_WARN("right arm value: %f - %f", right_arm_values[i], right_arm_at_side[i]);
			if (std::abs((double)right_arm_values[i] - (double)right_arm_at_side[i]) > error)
				right_arm = false;
		}

		for (int i = 0; i < left_arm_at_side.size(); i++)
		{
			ROS_WARN("left arm value: %f - %f", left_arm_values[i], left_arm_at_side[i]);
			if (std::abs((double)left_arm_values[i] - (double)left_arm_at_side[i]) > error)
				left_arm = false;
		}

//		if (left_arm_values != left_arm_at_side)
//			left_arm = false;

		res.right_arm = right_arm;
		res.left_arm = left_arm;

		return true;
	}


};



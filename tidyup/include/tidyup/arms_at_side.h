#ifndef ARMS_AT_SIDE_H_
#define ARMS_AT_SIDE_H_

#include <ros/ros.h>
#include <tidyup_msgs/ArmsAtSide.h>
#include <moveit/move_group_interface/move_group.h>

namespace tidyup
{

	class armsAtSide
	{
		public:
		armsAtSide(moveit::planning_interface::MoveGroup* right_arm_group,
				moveit::planning_interface::MoveGroup* left_arm_group);
		~armsAtSide();

		bool checkIfArmsAtSide(tidyup_msgs::ArmsAtSide::Request &req,
								tidyup_msgs::ArmsAtSide::Response &res);

		// Load the joint values for the arm at the side into the given vector.
		bool loadJointValues(const std::string &group_name, std::vector<double> &arm_joints) const;
		private:

		moveit::planning_interface::MoveGroup* right_arm_group_;
		moveit::planning_interface::MoveGroup* left_arm_group_;

	};


};

#endif /* ARMS_AT_SIDE_H_ */

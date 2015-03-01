#ifndef ARM_TO_SIDE_ACTION_SERVER_H_
#define ARM_TO_SIDE_ACTION_SERVER_H_

#include <actionlib/server/simple_action_server.h>
#include <tidyup_msgs/ArmToSideAction.h>
#include <moveit/move_group_interface/move_group.h>

namespace tidyup
{

	class ArmToSideActionServer
	{
		public:

		ArmToSideActionServer(ros::NodeHandle nh, std::string name, moveit::planning_interface::MoveGroup* right_arm_group,
				moveit::planning_interface::MoveGroup* left_arm_group);
		~ArmToSideActionServer();

		void executeArmToSide(const tidyup_msgs::ArmToSideGoalConstPtr &goal);

		private:

		ros::NodeHandle nhPriv_;
		actionlib::SimpleActionServer<tidyup_msgs::ArmToSideAction> as_;
		std::string action_name_;

		// create messages that are used to publish feedback/result
		tidyup_msgs::ArmToSideFeedback feedback_;
		tidyup_msgs::ArmToSideResult result_;

		moveit::planning_interface::MoveGroup* right_arm_group_;
		moveit::planning_interface::MoveGroup* left_arm_group_;
	};
};

#endif /* ARM_TO_SIDE_ACTION_SERVER_H_ */

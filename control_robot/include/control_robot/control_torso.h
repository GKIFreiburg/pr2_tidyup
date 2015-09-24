#ifndef CONTROL_TORSO_H_
#define CONTROL_TORSO_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <control_robot_msgs/MoveIt.h>
#include <control_robot_msgs/MoveItDegrees.h>

namespace control_robot
{
	class ControlTorso
	{
		public:
		// Declaring all member variables
		ControlTorso(moveit::planning_interface::MoveGroup* torso_group);
		~ControlTorso();

		// Service callback for lifting torso of robot
		bool torsoLiftMax(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for lifting torso of robot
		bool torsoLiftMin(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);

		private:

		moveit::planning_interface::MoveItErrorCode moveTorsoJointPosition(double position);

		moveit::planning_interface::MoveGroup* torso_group_;
		std::string torso_;
		ros::ServiceServer srvTorsoLiftMax_;
		ros::ServiceServer srvTorsoLiftMin_;
		ros::ServiceServer srvTorsoLift_;

	};

};

#endif /* CONTROL_HEAD_H_ */

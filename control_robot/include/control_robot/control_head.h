#ifndef CONTROL_HEAD_H_
#define CONTROL_HEAD_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <control_robot_msgs/MoveIt.h>
#include <control_robot_msgs/MoveItDegrees.h>

namespace control_robot
{

	class ControlHead
	{
		public:
		// Declaring all member variables
		ControlHead(moveit::planning_interface::MoveGroup* head_group);
		~ControlHead();

		// Service callback for lowering head of robot
		bool headPitchDown(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for raising head of robot
		bool headPitchUp(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for setting head straight (horizontal)
		bool headPitchStraight(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback to lower/raise by the given degrees
		bool headPitchDegrees(control_robot_msgs::MoveItDegrees::Request &req,
				control_robot_msgs::MoveItDegrees::Response &res);

		// Service callback for turning head to left
		bool headYawLeft(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for turning head to right
		bool headYawRight(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for setting head straight (vertical)
		bool headYawStraight(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);

		bool headYawDegrees(control_robot_msgs::MoveItDegrees::Request &req,
				control_robot_msgs::MoveItDegrees::Response &res);
		// Service callback to turn head left/right by the given degrees
		bool headInitialPosition(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);

		inline static double degreesToRadians(const double degrees) {return degrees * M_PI / 180;}

		private:

		moveit::planning_interface::MoveGroup* head_group_;
		std::string head_pitch_;
		std::string head_yaw_;
		double head_step_;

		// Receive the current joint value of the entered joint
		bool getCurrentJointValueOfJoint(const std::string jointName, double &value);
		// Move the specified joint to the entered value
		moveit::planning_interface::MoveItErrorCode moveHeadJointToJointValue(
				const std::string jointName, const double value);
		// Move the specified joint by the entered step, regarding the current position
		moveit::planning_interface::MoveItErrorCode moveHeadJointByStep(const std::string joint, const double step);
		// Move all joints defined in the map to their corresponding value in the map
		moveit::planning_interface::MoveItErrorCode moveHeadJoints(const std::map<std::string, double> jointValues);

	};

};

#endif /* CONTROL_HEAD_H_ */

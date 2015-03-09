#ifndef CONTROL_ARMS_H_
#define CONTROL_ARMS_H_

#include <ros/ros.h>
#include <moveit/move_group_interface/move_group.h>
#include <control_robot_msgs/MoveIt.h>
#include <control_robot_msgs/MoveItDegrees.h>

namespace control_robot
{

	class ControlArms
	{
		public:
		// Declaring all member variables
		ControlArms(moveit::planning_interface::MoveGroup* right_arm_group,
				moveit::planning_interface::MoveGroup* left_arm_group,
				moveit::planning_interface::MoveGroup* arms_group);
		~ControlArms();

		// Service callback for moving both arms to side
		bool armsToSide(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for moving both arms to front
		bool armsToFront(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for moving both arms to front and bent
		bool armsToFrontBent(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);

		// Service callback for moving right arm to side
		bool rightArmToSide(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for moving right arm to front
		bool rightArmToFront(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for moving right arm to front and bent
		bool rightArmToFrontBent(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);

		// Service callback for moving left arm to side
		bool leftArmToSide(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for moving right arm to front
		bool leftArmToFront(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);
		// Service callback for moving right arm to front and bent
		bool leftArmToFrontBent(control_robot_msgs::MoveIt::Request &req,
				control_robot_msgs::MoveIt::Response &res);

		private:

		moveit::planning_interface::MoveGroup* right_arm_group_;
		moveit::planning_interface::MoveGroup* left_arm_group_;
		moveit::planning_interface::MoveGroup* arms_group_;

		std::vector<double> right_arm_at_side_values_;
		std::vector<double> right_arm_at_front_values_;
		std::vector<double> right_arm_at_front_bent_values_;

		std::vector<double> left_arm_at_side_values_;
		std::vector<double> left_arm_at_front_values_;
		std::vector<double> left_arm_at_front_bent_values_;

		std::vector<double> arms_at_side_values_;
		std::vector<double> arms_at_front_values_;
		std::vector<double> arms_at_front_bent_values_;

		// Move all joints defined in the map of the specified group
		// to their corresponding value in the map
		moveit::planning_interface::MoveItErrorCode moveArms(
				moveit::planning_interface::MoveGroup* group,
				const std::map<std::string, double> jointValues);

		// Move all joints of the specified group to the values in the vector
		moveit::planning_interface::MoveItErrorCode moveArms(
				moveit::planning_interface::MoveGroup* group,
				const std::vector<double> jointValues);

		void printValues(std::vector<double> values);

	};

};


#endif /* CONTROL_ARMS_H_ */

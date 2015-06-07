#ifndef MANIPULATION_LOCATION_GENERATOR_INTERFACE_H_
#define MANIPULATION_LOCATION_GENERATOR_INTERFACE_H_

#include <geometry_msgs/PoseStamped.h>
#include <moveit_msgs/CollisionObject.h>
#include <ros/ros.h>
#include <moveit/planning_interface/planning_interface.h>

#include <symbolic_planning_utils/planning_scene_monitor_singleton.h>

namespace manipulation_location_generator
{
	typedef std::vector<geometry_msgs::PoseStamped> ManipulationLocations;

	class ManipulationLocationGeneratorInterface
	{
		public:
			ManipulationLocationGeneratorInterface();
			virtual ~ManipulationLocationGeneratorInterface();

			bool initialize(const std::string& planning_scene_topic,
					const moveit_msgs::CollisionObject& table,
					const int max_samples,
					const int attempts);

			ManipulationLocations generateSamples();

		protected:
			virtual geometry_msgs::Pose generateSample() = 0;

		protected:
			ros::NodeHandle nh_;
			ros::Publisher pub_samples_;
			ros::Publisher pub_not_in_collision_samples_;
			ros::Publisher pub_reachable_samples_;

			ros::ServiceClient plan_path_client_;
			ros::ServiceClient get_map_client_;

			std::string planning_scene_topic_;
			moveit_msgs::CollisionObject table_;
			int max_samples_;
			int attempts_;

			planning_scene::PlanningScenePtr planning_scene_;
			// planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
			bool setPlanningScene(const std::string& planning_scene_topic);

			double table_width_, table_height_;
			geometry_msgs::PoseStamped table_pose_;
			bool extractTableInfo(const moveit_msgs::CollisionObject& table,
					double& width, double& height, geometry_msgs::PoseStamped& pose);

			bool inCollision(const geometry_msgs::PoseStamped& sample);

			bool isReachable(const geometry_msgs::PoseStamped& sample);

			bool inMap(const geometry_msgs::PoseStamped& sample);

			geometry_msgs::PoseStamped getRobotPose(const robot_state::RobotState& robot_state);



	};

};

#endif /* MANIPULATION_LOCATION_GENERATOR_INTERFACE_H_ */

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

			// Setting member variables, needed by sampling classes
			bool initialize(const std::string& planning_scene_topic,
					const moveit_msgs::CollisionObject& table,
					const int max_samples,
					const int attempts);

			/* Generate valid Samples
			 * Check 1: collision checking with planning scene
			 * Check 2: sample in map and in free space
			 * (Check 3: sample reachable by move_base)
			 * */
			ManipulationLocations generateSamples();

		protected:
			// virtual function to create a sample - is implemented in sampling classes
			virtual geometry_msgs::Pose generateSample() = 0;

		protected:
			ros::NodeHandle nh_;
			ros::Publisher pub_samples_;
			ros::Publisher pub_not_in_collision_samples_;
			ros::Publisher pub_in_map_;
			ros::Publisher pub_reachable_samples_;

			ros::ServiceClient plan_path_client_;
			ros::ServiceClient get_map_client_;

			std::string planning_scene_topic_;
			moveit_msgs::CollisionObject table_;
			int max_samples_;
			int attempts_;

			planning_scene::PlanningScenePtr planning_scene_;
			// planning_scene_monitor::PlanningSceneMonitorPtr planning_scene_monitor_;
			// Setting planning_scene pointer, needed by collision checking
			bool setPlanningScene(const std::string& planning_scene_topic);

			double table_width_, table_height_;
			geometry_msgs::PoseStamped table_pose_;
			// Extract table info from collision object
			bool extractTableInfo(const moveit_msgs::CollisionObject& table,
					double& width, double& height, geometry_msgs::PoseStamped& pose);

			// Verify that sample is not in collision with planning scene
			bool inCollision(const geometry_msgs::PoseStamped& sample);

			// Verify that sample is empty grid cell (= free space) of map
			bool inMap(const geometry_msgs::PoseStamped& sample);

			// Verify that sample can be reached by move_base
			bool isReachable(const geometry_msgs::PoseStamped& sample);

			// Extract poseStamped from robot state
			geometry_msgs::PoseStamped getRobotPose(const robot_state::RobotState& robot_state);



	};

};

#endif /* MANIPULATION_LOCATION_GENERATOR_INTERFACE_H_ */

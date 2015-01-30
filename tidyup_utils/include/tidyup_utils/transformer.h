#ifndef TRANSFORMER_H_
#define TRANSFORMER_H_

#include <ros/ros.h>
//#include <state_transformer/GetTransform.h>
#include <geometry_msgs/Point.h>
#include <geometry_msgs/Quaternion.h>
#include <geometry_msgs/Pose.h>
#include <geometry_msgs/PointStamped.h>
#include <geometry_msgs/QuaternionStamped.h>
#include <geometry_msgs/PoseStamped.h>
#include <arm_navigation_msgs/Constraints.h>

#include <planning_environment/models/model_utils.h>

#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>

class Transformer
{
public:
    Transformer();
    virtual ~Transformer();

    static bool transform(const std::string& new_frame,
            const std::string& old_frame,
            const arm_navigation_msgs::RobotState& robotState,
            geometry_msgs::Point& point);
    static bool transform(const std::string& new_frame,
            const std::string& old_frame,
            const arm_navigation_msgs::RobotState& robotState,
            geometry_msgs::Quaternion& quaternion);
    static bool transform(const std::string& new_frame,
            const std::string& old_frame,
            const arm_navigation_msgs::RobotState& robotState,
            geometry_msgs::Pose& pose);

    static bool transform(const std::string& new_frame,
            const std::string& old_frame,
            const arm_navigation_msgs::RobotState& robotState,
            geometry_msgs::PointStamped& point);
    static bool transform(const std::string& new_frame,
            const std::string& old_frame,
            const arm_navigation_msgs::RobotState& robotState,
            geometry_msgs::QuaternionStamped& quaternion);
    static bool transform(const std::string& new_frame,
            const std::string& old_frame,
            const arm_navigation_msgs::RobotState& robotState,
            geometry_msgs::PoseStamped& pose);

    static bool transform(const std::string& new_frame,
            const arm_navigation_msgs::RobotState& robotState,
            arm_navigation_msgs::Constraints& constraint);

private:
    static Transformer* instance;
    bool getTransform_(const std::string& to_frame,
            const std::string& from_frame,
            const moveit_msgs::RobotState& robotState,
            tf::Transform& transform);
    bool getWorldTransform(const std::string& frame_id,
            const robot_state::RobotState &state,
            tf::Transform &transform);
    std::string relative_frame(const std::string& frame_id) const;
    //planning_environment::RobotModels robot_model;
    robot_model_loader::RobotModelLoader robot_model_loader_;
};

#endif /* TRANSFORMER_H_ */

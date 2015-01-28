#ifndef PLANNING_SCENE_INTERFACE_H_
#define PLANNING_SCENE_INTERFACE_H_

#include <ros/ros.h>
#include <ros/service.h>
#include <ros/publisher.h>
#include <sensor_msgs/JointState.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <moveit_msgs/AllowedCollisionEntry.h>
#include "tidyup_utils/hand_description.h"     // TODO: not used in cpp
#include <map>

class PlanningSceneInterface
{
public:
    static PlanningSceneInterface* instance();
    //bool setPlanningSceneDiff(const arm_navigation_msgs::PlanningScene& scene);

    bool sendDiff();
    bool resetPlanningScene();

    //const arm_navigation_msgs::PlanningScene& getPlanningScene() const {return spsdService.response.planning_scene;}
    const moveit_msgs::PlanningScene& getCurrentScene() const {return planning_scene_diff_;}
    const moveit_msgs::RobotState& getRobotState(){return planning_scene_diff_.robot_state;}
    const std::vector<moveit_msgs::CollisionObject>& getCollisionObjects(){return planning_scene_diff_.world.collision_objects;}
    const std::vector<moveit_msgs::AttachedCollisionObject>& getAttachedCollisionObjects() const {return planning_scene_diff_.robot_state.attached_collision_objects;}
    const moveit_msgs::CollisionObject* getCollisionObject(const std::string& id);
    const moveit_msgs::AttachedCollisionObject* getAttachedCollisionObject(const std::string& id);

    void setRobotState(const moveit_msgs::RobotState& state);
    void addObject(const moveit_msgs::CollisionObject& object);
    void removeObject(const std::string& id);
    void updateObject(const std::string& id, const geometry_msgs::Pose& pose);
    // TODO: Still useful? c.f. pick and place
    void attachObjectToGripper(const std::string& id, const std::string& arm);
    void detachObjectAndAdd(const std::string& id);

    const std::string& getGlobalFrame() const {return globalFrame_;}

    void printDiffToCurrent(const moveit_msgs::PlanningScene& other) const;
    static void printDiff(const moveit_msgs::PlanningScene& scene, const moveit_msgs::PlanningScene& other);
    static void printDiff(const moveit_msgs::RobotState& state, const moveit_msgs::RobotState& other);
    static void printDiff(const std::vector<moveit_msgs::CollisionObject>& objectList,
            const std::vector<moveit_msgs::CollisionObject>& other);
    static void printDiff(const std::vector<moveit_msgs::AttachedCollisionObject>& objectList,
            const std::vector<moveit_msgs::AttachedCollisionObject>& other);
    // TODO: only comparing Transforms
    static bool isDifferent(const geometry_msgs::Transform& transform, const geometry_msgs::Transform& other);
    static bool isDifferent(const geometry_msgs::Pose& pose, const geometry_msgs::Pose& other);
    static void printAttachedObject(const moveit_msgs::AttachedCollisionObject& object);
    static void printObjects(const moveit_msgs::PlanningScene& scene);
    void test();


private:
    PlanningSceneInterface();

    moveit_msgs::RobotState& getRobotState_() {return planning_scene_diff_.robot_state;}
    std::vector<moveit_msgs::CollisionObject>& getCollisionObjects_() {return planning_scene_diff_.world.collision_objects;}
    std::vector<moveit_msgs::AttachedCollisionObject>& getAttachedCollisionObjects_() {return planning_scene_diff_.robot_state.attached_collision_objects;}
    moveit_msgs::CollisionObject* getCollisionObject_(const std::string& id);
    moveit_msgs::AttachedCollisionObject* getAttachedCollisionObject_(const std::string& id);

    static PlanningSceneInterface* singleton_instance_;
    ros::ServiceClient getPlanningSceneClient_;
	moveit_msgs::GetPlanningScene getPlanningSceneService_;
    moveit_msgs::PlanningScene planning_scene_diff_;
    ros::Publisher planning_scene_diff_publisher_;

    // TODO: not used in cpp
    std::map<std::string, HandDescription> handDescriptions_;
    std::string globalFrame_;

    std::string logName_;
 /*
    static PlanningSceneInterface* singleton_instance;
    arm_navigation_msgs::SetPlanningSceneDiff spsdService;
    ros::ServiceClient setPlanningSceneService;

    std::map<std::string, HandDescription> handDescriptions;
    std::string globalFrame;

    std::string logName;
  */
};


#endif /* PLANNING_SCENE_INTERFACE_H_ */

/*
 * planning_scene_interface.h
 *
 *  Created on: 26 Jul 2012
 *      Author: andreas
 */

#ifndef PLANNING_SCENE_INTERFACE_H_
#define PLANNING_SCENE_INTERFACE_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <arm_navigation_msgs/SetPlanningSceneDiff.h>
#include "tidyup_utils/hand_description.h"
#include <map>

class PlanningSceneInterface
{
public:
    static PlanningSceneInterface* instance();
//    bool setPlanningSceneDiff(const arm_navigation_msgs::PlanningScene& scene);

    bool sendDiff();
    bool resetPlanningScene();

//    const arm_navigation_msgs::PlanningScene& getPlanningScene() const {return spsdService.response.planning_scene;}
    const arm_navigation_msgs::PlanningScene& getCurrentScene() const {return spsdService.request.planning_scene_diff;}
    const arm_navigation_msgs::RobotState& getRobotState(){return spsdService.request.planning_scene_diff.robot_state;}
    const std::vector <arm_navigation_msgs::CollisionObject>& getCollisionObjects(){return spsdService.request.planning_scene_diff.collision_objects;}
    const std::vector <arm_navigation_msgs::AttachedCollisionObject>& getAttachedCollisionObjects() const {return spsdService.request.planning_scene_diff.attached_collision_objects;}
    const arm_navigation_msgs::CollisionObject* getCollisionObject(const std::string& id);
    const arm_navigation_msgs::AttachedCollisionObject* getAttachedCollisionObject(const std::string& id);

    void setRobotState(const arm_navigation_msgs::RobotState& state);
    void addObject(const arm_navigation_msgs::CollisionObject& object);
    void removeObject(const std::string& id);
    void updateObject(const std::string& id, const geometry_msgs::Pose& pose);
    void attachObjectToGripper(const std::string& id, const std::string& arm);
    void detachObjectAndAdd(const std::string& id);

    const std::string& getGlobalFrame() const {return globalFrame;}

    void printDiffToCurrent(const arm_navigation_msgs::PlanningScene& other) const;
    static void printDiff(const arm_navigation_msgs::PlanningScene& scene, const arm_navigation_msgs::PlanningScene& other);
    static void printDiff(const arm_navigation_msgs::RobotState& state, const arm_navigation_msgs::RobotState& other);
    static void printDiff(const std::vector<arm_navigation_msgs::CollisionObject>& objectList,
            const std::vector<arm_navigation_msgs::CollisionObject>& other);
    static void printDiff(const std::vector<arm_navigation_msgs::AttachedCollisionObject>& objectList,
            const std::vector<arm_navigation_msgs::AttachedCollisionObject>& other);
    static bool isDifferent(const geometry_msgs::Pose& pose, const geometry_msgs::Pose& other);
    static void printAttachedObject(const arm_navigation_msgs::AttachedCollisionObject& object);
    static void printObjects(const arm_navigation_msgs::PlanningScene& scene);
    void test();


private:
    PlanningSceneInterface();

    arm_navigation_msgs::RobotState& getRobotState_() {return spsdService.request.planning_scene_diff.robot_state;}
    std::vector <arm_navigation_msgs::CollisionObject>& getCollisionObjects_() {return spsdService.request.planning_scene_diff.collision_objects;}
    std::vector <arm_navigation_msgs::AttachedCollisionObject>& getAttachedCollisionObjects_() {return spsdService.request.planning_scene_diff.attached_collision_objects;}
    arm_navigation_msgs::CollisionObject* getCollisionObject_(const std::string& id);
    arm_navigation_msgs::AttachedCollisionObject* getAttachedCollisionObject_(const std::string& id);

    static PlanningSceneInterface* singleton_instance;
    arm_navigation_msgs::SetPlanningSceneDiff spsdService;
    ros::ServiceClient setPlanningSceneService;

    std::map<std::string, HandDescription> handDescriptions;
    std::string globalFrame;

    std::string logName;
};


#endif /* PLANNING_SCENE_INTERFACE_H_ */

#ifndef PLANNING_SCENE_INTERFACE_H_
#define PLANNING_SCENE_INTERFACE_H_

#include <ros/ros.h>
#include <ros/service.h>
#include <ros/publisher.h>
#include <moveit_msgs/RobotState.h>
#include <moveit_msgs/PlanningScene.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <moveit_msgs/CollisionObject.h>
#include <map>

class PlanningSceneMsgInterface
{
public:

	static const std::string DEFAULT_PLANNING_SCENE_TOPIC; // "virtual_planning_scene"

    static PlanningSceneMsgInterface* getInstance(const std::string& planning_scene_topic = DEFAULT_PLANNING_SCENE_TOPIC);

    void setPlanningScene(const moveit_msgs::PlanningScene& scene);
    void publishPlanningScene();
    void resetPlanningScene();

    const moveit_msgs::PlanningScene& getPlanningScene() const { return planning_scene_; }
    const moveit_msgs::RobotState& getRobotState() { return planning_scene_.robot_state; }
    // get all collision objects, even those that have operation = remove set
    const std::vector<moveit_msgs::CollisionObject>& getCollisionObjects()
    		{ return planning_scene_.world.collision_objects; }
    const std::vector<moveit_msgs::AttachedCollisionObject>& getAttachedCollisionObjects() const
    		{ return planning_scene_.robot_state.attached_collision_objects; }
    const moveit_msgs::CollisionObject* getCollisionObject(const std::string& id);
    const moveit_msgs::AttachedCollisionObject* getAttachedCollisionObject(const std::string& id);

    void setRobotState(const moveit_msgs::RobotState& state);
    void addObject(const moveit_msgs::CollisionObject& object);
    void removeObject(const std::string& id);
    void updateObject(const std::string& id, const geometry_msgs::Pose& pose);
    void attachObjectToGripper(const std::string& id, const std::string& arm);
    void detachObjectAndAdd(const std::string& id);

    const std::string& getGlobalFrame() const { return globalFrame_; }

    void printDiffToCurrent(const moveit_msgs::PlanningScene& other) const;
    static void printDiff(const moveit_msgs::PlanningScene& scene, const moveit_msgs::PlanningScene& other);
    static void printDiff(const std::vector<moveit_msgs::CollisionObject>& objectList,
                const std::vector<moveit_msgs::CollisionObject>& other);
    static void printDiff(const std::vector<moveit_msgs::AttachedCollisionObject>& objectList,
                const std::vector<moveit_msgs::AttachedCollisionObject>& other);
    static bool isDifferent(const geometry_msgs::Pose& pose, const geometry_msgs::Pose& other);


private:
    PlanningSceneMsgInterface(const std::string& planning_scene_topic);
    ~PlanningSceneMsgInterface();

    static PlanningSceneMsgInterface* instance_;

    std::string globalFrame_;
    std::string get_planning_scene_service_name_;
    std::string planning_scene_topic_;
    ros::ServiceClient get_planning_scene_client_;

    moveit_msgs::GetPlanningScene get_planning_scene_service_;
    moveit_msgs::PlanningScene planning_scene_;
    ros::Publisher planning_scene_publisher_;

    moveit_msgs::RobotState& getRobotState_() { return planning_scene_.robot_state; }
    std::vector<moveit_msgs::CollisionObject>& getCollisionObjects_()
    		{ return planning_scene_.world.collision_objects; }
    std::vector<moveit_msgs::AttachedCollisionObject>& getAttachedCollisionObjects_()
    		{ return planning_scene_.robot_state.attached_collision_objects; }
    moveit_msgs::CollisionObject* getCollisionObject_(const std::string& id);
    moveit_msgs::AttachedCollisionObject* getAttachedCollisionObject_(const std::string& id);


//    //bool setPlanningSceneDiff(const arm_navigation_msgs::PlanningScene& scene);
//
//    bool sendDiff();
//    bool resetPlanningScene();
//
//    //const arm_navigation_msgs::PlanningScene& getPlanningScene() const {return spsdService.response.planning_scene;}
//    const moveit_msgs::PlanningScene& getCurrentScene() const {return planning_scene_;}
//    const moveit_msgs::RobotState& getRobotState(){return planning_scene_.robot_state;}
//    const std::vector<moveit_msgs::CollisionObject>& getCollisionObjects(){return planning_scene_.world.collision_objects;}
//    const std::vector<moveit_msgs::AttachedCollisionObject>& getAttachedCollisionObjects() const {return planning_scene_.robot_state.attached_collision_objects;}
//    const moveit_msgs::CollisionObject* getCollisionObject(const std::string& id);
//    const moveit_msgs::AttachedCollisionObject* getAttachedCollisionObject(const std::string& id);
//
//    void setRobotState(const moveit_msgs::RobotState& state);
//    void addObject(const moveit_msgs::CollisionObject& object);
//    void removeObject(const std::string& id);
//    void updateObject(const std::string& id, const geometry_msgs::Pose& pose);
//    // TODO: Still useful? c.f. pick and place
//    void attachObjectToGripper(const std::string& id, const std::string& arm);
//    void detachObjectAndAdd(const std::string& id);
//
//    const std::string& getGlobalFrame() const {return globalFrame_;}
//
//    void printDiffToCurrent(const moveit_msgs::PlanningScene& other) const;
//    static void printDiff(const moveit_msgs::PlanningScene& scene, const moveit_msgs::PlanningScene& other);
//    static void printDiff(const moveit_msgs::RobotState& state, const moveit_msgs::RobotState& other);
//    static void printDiff(const std::vector<moveit_msgs::CollisionObject>& objectList,
//            const std::vector<moveit_msgs::CollisionObject>& other);
//    static void printDiff(const std::vector<moveit_msgs::AttachedCollisionObject>& objectList,
//            const std::vector<moveit_msgs::AttachedCollisionObject>& other);
//    // TODO: only comparing Transforms
//    static bool isDifferent(const geometry_msgs::Transform& transform, const geometry_msgs::Transform& other);
//    static bool isDifferent(const geometry_msgs::Pose& pose, const geometry_msgs::Pose& other);
//    static void printAttachedObject(const moveit_msgs::AttachedCollisionObject& object);
//    static void printObjects(const moveit_msgs::PlanningScene& scene);
//    void test();
//
//
//private:
//    PlanningSceneInterface();
//
//    moveit_msgs::RobotState& getRobotState_() {return planning_scene_.robot_state;}
//    std::vector<moveit_msgs::CollisionObject>& getCollisionObjects_() {return planning_scene_.world.collision_objects;}
//    std::vector<moveit_msgs::AttachedCollisionObject>& getAttachedCollisionObjects_() {return planning_scene_.robot_state.attached_collision_objects;}
//    moveit_msgs::CollisionObject* getCollisionObject_(const std::string& id);
//    moveit_msgs::AttachedCollisionObject* getAttachedCollisionObject_(const std::string& id);
//
//    static PlanningSceneInterface* singleton_instance_;
//    ros::ServiceClient getPlanningSceneClient_;
//	moveit_msgs::GetPlanningScene getPlanningSceneService_;
//    moveit_msgs::PlanningScene planning_scene_;
//    ros::Publisher planning_scene_publisher_;
//
//    // TODO: not used in cpp
//    std::map<std::string, HandDescription> handDescriptions_;
//    std::string globalFrame_;
//
//    std::string logName_;


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

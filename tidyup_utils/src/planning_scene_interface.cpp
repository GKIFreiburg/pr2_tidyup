/*
 * planning_scene_interface.cpp
 *
 *  Created on: 26 Jul 2012
 *      Author: andreas
 */

#include "tidyup_utils/planning_scene_interface.h"
#include "tidyup_utils/transformer.h"

PlanningSceneInterface* PlanningSceneInterface::singleton_instance = NULL;

PlanningSceneInterface* PlanningSceneInterface::instance()
{
    if (singleton_instance == NULL)
    {
        singleton_instance = new PlanningSceneInterface();
    }
    return singleton_instance;
}

PlanningSceneInterface::PlanningSceneInterface() : logName("[psi]")
{
    // init service for planning scene
    std::string service_name = "/environment_server/set_planning_scene_diff";
    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
    {
        ROS_ERROR_NAMED(logName, "Service %s not available - waiting.", service_name.c_str());
    }
    setPlanningSceneService = ros::service::createClient<arm_navigation_msgs::SetPlanningSceneDiff>(service_name, true);
    if (!setPlanningSceneService)
    {
        ROS_FATAL_NAMED(logName, "Could not initialize get plan service from %s (client name: %s)", service_name.c_str(), setPlanningSceneService.getService().c_str());
        return;
    }
    if (setPlanningSceneService.call(spsdService))
    {
        spsdService.request.planning_scene_diff = spsdService.response.planning_scene;
    }
    else
    {
        ROS_ERROR_NAMED(logName, "%s Could not initialize planning scene.", __PRETTY_FUNCTION__);
        return;
    }
    globalFrame = getRobotState_().multi_dof_joint_state.frame_ids[0];
    ROS_INFO_NAMED(logName, "planning scene interface initialized.");
}

//bool PlanningSceneInterface::setPlanningSceneDiff(const arm_navigation_msgs::PlanningScene& scene)
//{
//    spsdService.request.planning_scene_diff = scene;
//    return sendDiff();
//}
bool PlanningSceneInterface::sendDiff()
{
    if (setPlanningSceneService.call(spsdService))
    {
        ROS_INFO_NAMED(logName, "sending planning scene diff.");
        spsdService.request.planning_scene_diff = spsdService.response.planning_scene;
        return true;
    }
    else
    {
        ROS_ERROR_NAMED(logName, "send planning scene diff FAILED.");
        return false;
    }
}

bool PlanningSceneInterface::resetPlanningScene()
{
    spsdService.request.planning_scene_diff = arm_navigation_msgs::PlanningScene();
    return sendDiff();
}

const arm_navigation_msgs::CollisionObject* PlanningSceneInterface::getCollisionObject(const std::string& id)
{
    return getCollisionObject_(id);
}

const arm_navigation_msgs::AttachedCollisionObject* PlanningSceneInterface::getAttachedCollisionObject(const std::string& id)
{
    return getAttachedCollisionObject_(id);
}

arm_navigation_msgs::CollisionObject* PlanningSceneInterface::getCollisionObject_(const std::string& id)
{
    std::vector<arm_navigation_msgs::CollisionObject>& objects = getCollisionObjects_();
    for(std::vector<arm_navigation_msgs::CollisionObject>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        if (it->id == id && it->operation.operation != it->operation.REMOVE)
            return &*it;
    }
    return NULL;
}

arm_navigation_msgs::AttachedCollisionObject* PlanningSceneInterface::getAttachedCollisionObject_(const std::string& id)
{
    std::vector<arm_navigation_msgs::AttachedCollisionObject>& objects = getAttachedCollisionObjects_();
    for(std::vector<arm_navigation_msgs::AttachedCollisionObject>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        if (it->object.id == id && it->object.operation.operation != it->object.operation.REMOVE)
            return &*it;
    }
    return NULL;
}

void PlanningSceneInterface::setRobotState(const arm_navigation_msgs::RobotState& state)
{
    spsdService.request.planning_scene_diff.robot_state = state;
    // set floating links according to robot pose... (some kind of pr2_python hack?)
//    if (state.joint_state.name[0] == "floating_trans_x")
//    {
//        arm_navigation_msgs::RobotState& state = spsdService.request.planning_scene_diff.robot_state;
//        state.joint_state.position[0] = state.multi_dof_joint_state.poses[0].position.x;
//        state.joint_state.position[1] = state.multi_dof_joint_state.poses[0].position.y;
//        state.joint_state.position[2] = state.multi_dof_joint_state.poses[0].position.z;
//        state.joint_state.position[3] = state.multi_dof_joint_state.poses[0].orientation.x;
//        state.joint_state.position[4] = state.multi_dof_joint_state.poses[0].orientation.y;
//        state.joint_state.position[5] = state.multi_dof_joint_state.poses[0].orientation.z;
//        state.joint_state.position[6] = state.multi_dof_joint_state.poses[0].orientation.w;
//    }
}

void PlanningSceneInterface::addObject(const arm_navigation_msgs::CollisionObject& object)
{
    std::vector<arm_navigation_msgs::CollisionObject>& objects = getCollisionObjects_();
    for(std::vector<arm_navigation_msgs::CollisionObject>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        if (it->id == object.id)
        {
            objects.erase(it);
            break;
        }
    }
    std::vector<arm_navigation_msgs::AttachedCollisionObject>& attachedObjects = getAttachedCollisionObjects_();
    for(std::vector<arm_navigation_msgs::AttachedCollisionObject>::iterator it = attachedObjects.begin(); it != attachedObjects.end(); it++)
    {
        if (it->object.id == object.id)
        {
            attachedObjects.erase(it);
            break;
        }
    }
    getCollisionObjects_().push_back(object);
    getCollisionObjects_().back().operation.operation = object.operation.ADD;
}

void PlanningSceneInterface::removeObject(const std::string& id)
{
    std::vector<arm_navigation_msgs::CollisionObject>& objects = getCollisionObjects_();
    for(std::vector<arm_navigation_msgs::CollisionObject>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        if (it->id == id)
        {
            it->operation.operation = it->operation.REMOVE;
            it->header.stamp = ros::Time::now();
            return;
        }
    }
    std::vector<arm_navigation_msgs::AttachedCollisionObject>& attachedObjects = getAttachedCollisionObjects_();
    for(std::vector<arm_navigation_msgs::AttachedCollisionObject>::iterator it = attachedObjects.begin(); it != attachedObjects.end(); it++)
    {
        if (it->object.id == id)
        {
            it->object.operation.operation = it->object.operation.REMOVE;
            it->object.header.stamp = ros::Time::now();
            return;
        }
    }
}

void PlanningSceneInterface::updateObject(const std::string& id, const geometry_msgs::Pose& pose)
{
    arm_navigation_msgs::CollisionObject* object = getCollisionObject_(id);
    if (object == NULL)
    {
        arm_navigation_msgs::AttachedCollisionObject* attached = getAttachedCollisionObject_(id);
        if (attached == NULL)
        {
            ROS_ERROR_NAMED(logName, "no object %s exists in planning scene.", id.c_str());
            return;
        }
        object = &attached->object;
    }
    object->poses[0] = pose;
    object->operation.operation = object->operation.ADD;
    object->header.stamp = ros::Time::now();
}

void PlanningSceneInterface::attachObjectToGripper(const std::string& id, const std::string& arm)
{
    arm_navigation_msgs::CollisionObject* object = getCollisionObject_(id);
    const HandDescription& hand = HandDescription::get(arm);
    if (object == NULL)
    {
        arm_navigation_msgs::AttachedCollisionObject* attached = getAttachedCollisionObject_(id);
        if (attached == NULL)
        {
            ROS_ERROR("%s no object %s exists in planning scene.", logName.c_str(), id.c_str());
            return;
        }
        if (attached->link_name == hand.getAttachLink())
        {
            ROS_INFO("%s object %s already attached at the correct arm.", logName.c_str(), id.c_str());
            return;
        }
        ROS_INFO("%s object %s attached at wrong arm. detaching.", logName.c_str(), id.c_str());
        detachObjectAndAdd(id);
        object = getCollisionObject_(id);
    }
    getAttachedCollisionObjects_().push_back(arm_navigation_msgs::AttachedCollisionObject());
    arm_navigation_msgs::AttachedCollisionObject& attached = getAttachedCollisionObjects_().back();
    attached.object = *object;
    attached.link_name = hand.getAttachLink();
    attached.touch_links = hand.getTouchLinks();
    attached.object.operation.operation = attached.object.operation.ADD;
    attached.object.header.stamp = ros::Time::now();
    Transformer::transform(hand.getAttachLink(), object->header.frame_id, getRobotState(), attached.object.poses[0]);
    attached.object.header.frame_id = hand.getAttachLink();
    object->operation.operation = object->operation.REMOVE;
//    eraseObject(object->id);
}

void PlanningSceneInterface::detachObjectAndAdd(const std::string& id)
{
    arm_navigation_msgs::AttachedCollisionObject* attached = getAttachedCollisionObject_(id);
    if (attached == NULL)
    {
        arm_navigation_msgs::CollisionObject* object = getCollisionObject_(id);
        if (object == NULL)
        {
            ROS_ERROR_NAMED(logName, "no object %s exists in planning scene.", id.c_str());
            return;
        }
        ROS_INFO_NAMED(logName, "object %s is not attached in planning scene.", id.c_str());
        return;
    }
    getCollisionObjects_().push_back(attached->object);
    arm_navigation_msgs::CollisionObject& object = getCollisionObjects_().back();
    Transformer::transform(globalFrame, object.header.frame_id, getRobotState_(), object.poses[0]);
    object.operation.operation = object.operation.ADD;
    object.header.frame_id = globalFrame;
    object.header.stamp = ros::Time::now();
    attached->object.operation.operation = attached->object.operation.REMOVE;
//    eraseAttachedObject(attached->object.id);
}

void PlanningSceneInterface::printDiffToCurrent(const arm_navigation_msgs::PlanningScene& other) const
{
    printDiff(spsdService.request.planning_scene_diff, other);
}

void PlanningSceneInterface::printDiff(const arm_navigation_msgs::PlanningScene& scene, const arm_navigation_msgs::PlanningScene& other)
{
    // robot joints
    ROS_INFO("---------------------");
    printDiff(scene.robot_state, other.robot_state);
//    // attached objects
//    printDiff(scene.attached_collision_objects, other.attached_collision_objects);
//    // objects
//    printDiff(scene.collision_objects, other.collision_objects);
    ROS_INFO("--------------------- objects");
    printObjects(scene);
    ROS_INFO("--------------------- other objects");
    printObjects(other);
    ROS_INFO("---------------------");
}

void PlanningSceneInterface::printDiff(const arm_navigation_msgs::RobotState& state, const arm_navigation_msgs::RobotState& other)
{
    ROS_INFO("RobotState diff:");
    if (isDifferent(state.multi_dof_joint_state.poses[0], other.multi_dof_joint_state.poses[0]))
    {
        ROS_INFO_STREAM("robot pose: " << state.multi_dof_joint_state.poses[0] << " vs " << other.multi_dof_joint_state.poses[0]);
    }
    double epsilon = 0.01; // rad
    for (size_t i = 0; i < state.joint_state.position.size(); i++)
    {
        if (fabs(state.joint_state.position[i] - other.joint_state.position[i]) > epsilon)
        {
            // different angle: print joint name
            ROS_INFO_STREAM("joint " << state.joint_state.name[i] << " " << state.joint_state.position[i] << " vs " << other.joint_state.position[i]);
        }
    }
}

bool PlanningSceneInterface::isDifferent(const geometry_msgs::Pose& pose, const geometry_msgs::Pose& other)
{
    double epsilon = 0.001; // 1mm
    bool different = false;
    if (fabs(pose.position.x - other.position.x) > epsilon)
        different = true;
    if (fabs(pose.position.y - other.position.y) > epsilon)
        different = true;
    if (fabs(pose.position.z - other.position.z) > epsilon)
        different = true;
    if (fabs(pose.orientation.x - other.orientation.x) > epsilon)
        different = true;
    if (fabs(pose.orientation.y - other.orientation.y) > epsilon)
        different = true;
    if (fabs(pose.orientation.z - other.orientation.z) > epsilon)
        different = true;
    if (fabs(pose.orientation.w - other.orientation.w) > epsilon)
        different = true;
    return different;
}

void PlanningSceneInterface::printDiff(
        const std::vector<arm_navigation_msgs::CollisionObject>& objectList,
        const std::vector<arm_navigation_msgs::CollisionObject>& other)
{
    // object ordering might be different
    for (std::vector<arm_navigation_msgs::CollisionObject>::const_iterator objectIt = objectList.begin();
            objectIt != objectList.end(); objectIt++)
    {
        for (std::vector<arm_navigation_msgs::CollisionObject>::const_iterator otherIt = other.begin();
                otherIt != other.end(); otherIt++)
        {
            if (objectIt->id == otherIt->id)
            {
                if (objectIt->operation.operation != otherIt->operation.operation)
                {
                    ROS_INFO_STREAM("object: " << objectIt->id << " op: " << objectIt->operation.operation << " vs " << otherIt->operation.operation);
                    continue;
                }
                if (isDifferent(objectIt->poses[0], otherIt->poses[0]))
                {
                    ROS_INFO_STREAM("object: " << objectIt->id << " pose: " << objectIt->poses[0] << " vs " << otherIt->poses[0]);
                    continue;
                }
            }
        }
        ROS_INFO_STREAM("object: " << objectIt->id << " removed in other");
    }
}

void PlanningSceneInterface::printDiff(const std::vector<arm_navigation_msgs::AttachedCollisionObject>& objectList,
            const std::vector<arm_navigation_msgs::AttachedCollisionObject>& other)
{
    // object ordering might be different
    for (std::vector<arm_navigation_msgs::AttachedCollisionObject>::const_iterator objectIt = objectList.begin();
            objectIt != objectList.end(); objectIt++)
    {
        for (std::vector<arm_navigation_msgs::AttachedCollisionObject>::const_iterator otherIt = other.begin();
                otherIt != other.end(); otherIt++)
        {
            if (objectIt->object.id == otherIt->object.id)
            {
                if (objectIt->object.operation.operation != otherIt->object.operation.operation)
                {
                    ROS_INFO_STREAM("object: " << objectIt->object.id << " op: " << objectIt->object.operation << " vs " << otherIt->object.operation);
                    continue;
                }
                if (objectIt->link_name == otherIt->link_name)
                {
                    ROS_INFO_STREAM("object: " << objectIt->object.id << " link_name: " << objectIt->link_name << " vs " << otherIt->link_name);
                    continue;
                }
            }
        }
        ROS_INFO_STREAM("object: " << objectIt->object.id << " removed in other");
    }
}

void PlanningSceneInterface::printAttachedObject(const arm_navigation_msgs::AttachedCollisionObject& object)
{
    ROS_INFO_STREAM("attached: " << object.object.id << " at " << object.link_name);
}

void PlanningSceneInterface::printObjects(const arm_navigation_msgs::PlanningScene& scene)
{
    const std::vector <arm_navigation_msgs::AttachedCollisionObject>& attachedObjects = scene.attached_collision_objects;
    for (std::vector <arm_navigation_msgs::AttachedCollisionObject>::const_iterator attachedIt = attachedObjects.begin(); attachedIt != attachedObjects.end(); attachedIt++)
    {
        ROS_INFO_STREAM("attached: " << attachedIt->object.id << " op: " << attachedIt->object.operation << " at " << attachedIt->link_name);
    }
    const std::vector <arm_navigation_msgs::CollisionObject>& objects = scene.collision_objects;
    for (std::vector <arm_navigation_msgs::CollisionObject>::const_iterator objectIt = objects.begin(); objectIt != objects.end(); objectIt++)
    {
        ROS_INFO_STREAM("object:   " << objectIt->id << " op: " << objectIt->operation);
    }
}

void PlanningSceneInterface::test()
{
    resetPlanningScene();
    printObjects(spsdService.request.planning_scene_diff);

    // attach cup_0
    attachObjectToGripper("cup_0", "right_arm");
//    removeObject("cup_0");
    sendDiff();
    printObjects(spsdService.request.planning_scene_diff);

    // attach cup_0
//    attachObjectToGripper("cup_0", "right_arm");
//    sendDiff();
//    printObjects();

    // detach cup_0
//    detachObjectAndAdd("cup_0");
//    sendDiff();
//    printObjects();
}


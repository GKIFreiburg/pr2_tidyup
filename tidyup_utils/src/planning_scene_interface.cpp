#include "tidyup_utils/planning_scene_interface.h"
#include "tidyup_utils/transformer.h"

PlanningSceneInterface* PlanningSceneInterface::singleton_instance_ = NULL;

PlanningSceneInterface* PlanningSceneInterface::instance()
{
    if (singleton_instance_ == NULL)
    {
        singleton_instance_ = new PlanningSceneInterface();
    }
    return singleton_instance_;
}

PlanningSceneInterface::PlanningSceneInterface() : logName_("[psi]")
{
    // init service for planning scene
    std::string service_name = "/get_planning_scene";
    while (!ros::service::waitForService(service_name, ros::Duration(3.0)))
    {
        ROS_ERROR_NAMED(logName_, "Service %s not available - waiting.", service_name.c_str());
    }

    // Create Client to fetch planning scene
    getPlanningSceneClient_ = ros::service::createClient<moveit_msgs::GetPlanningScene>(service_name, true);
    if (!getPlanningSceneClient_)
    {
        ROS_FATAL_NAMED(logName_, "Could not initialize get planning scene service from %s (client name: %s)", service_name.c_str(), getPlanningSceneClient_.getService().c_str());
        return;
    }

    // Get entire planning scene
    moveit_msgs::PlanningSceneComponents components;
    components.SCENE_SETTINGS;
    components.ROBOT_STATE;
    components.ROBOT_STATE_ATTACHED_OBJECTS;
    components.WORLD_OBJECT_NAMES;
    components.WORLD_OBJECT_GEOMETRY;
    components.OCTOMAP;
    components.TRANSFORMS;
    components.ALLOWED_COLLISION_MATRIX;
    components.LINK_PADDING_AND_SCALING;
    components.OBJECT_COLORS;

    if (getPlanningSceneClient_.call(getPlanningSceneService_))
    {
    	planning_scene_diff_ = getPlanningSceneService_.response.scene;
    }
    else
    {
        ROS_ERROR_NAMED(logName_, "%s Could not initialize planning scene.", __PRETTY_FUNCTION__);
        return;
    }

	ros::NodeHandle nh;
	if (!nh.ok())
	{
		ROS_FATAL_NAMED(logName_, "Could not initialize a nodehandle.");
		return;
	}
	// Publisher advertise(const std::string& topic, uint32_t queue_size, bool latch = false)
    // \param latch (optional) If true, the last message published on
    // this topic will be saved and sent to new subscribers when they
    // connect
	planning_scene_diff_publisher_ = nh.advertise<moveit_msgs::PlanningScene>("planning_scene" ,1, true);

    // TODO: check globalFrame
    globalFrame_ = getRobotState_().multi_dof_joint_state.header.frame_id;
    // TODO: globalFrame_ = 	getRobotState_().multi_dof_joint_state.frame_ids[0];
    ROS_INFO_NAMED(logName_, "planning scene interface initialized.");
}

//bool PlanningSceneInterface::setPlanningSceneDiff(const moveit_msgs::PlanningScene& scene)
//{
//    spsdService.request.planning_scene_diff = scene;
//    return sendDiff();
//}

bool PlanningSceneInterface::sendDiff()
{
	ROS_INFO_NAMED(logName_, "sending planning scene diff.");
	planning_scene_diff_publisher_.publish(planning_scene_diff_);
	return true;
// TODO:
//    if (setPlanningSceneService.call(spsdService))
//    {
//        ROS_INFO_NAMED(logName_, "sending planning scene diff.");
//        planning_scene_diff_ = getPlanningSceneService_.response.scene;
//        return true;
//    }
//    else
//    {
//        ROS_ERROR_NAMED(logName_, "send planning scene diff FAILED.");
//        return false;
//    }
}

bool PlanningSceneInterface::resetPlanningScene()
{
	planning_scene_diff_ = moveit_msgs::PlanningScene();
	return sendDiff();

	// TODO
//    spsdService.request.planning_scene_diff = moveit_msgs::PlanningScene();
//    return sendDiff();
}

moveit_msgs::CollisionObject* PlanningSceneInterface::getCollisionObject_(const std::string& id)
{
    std::vector<moveit_msgs::CollisionObject>& objects = getCollisionObjects_();
    for(std::vector<moveit_msgs::CollisionObject>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        if (it->id == id && it->operation != it->REMOVE)
            return &*it;
    }
    return NULL;
}

const moveit_msgs::CollisionObject* PlanningSceneInterface::getCollisionObject(const std::string& id)
{
    return getCollisionObject_(id);
}

moveit_msgs::AttachedCollisionObject* PlanningSceneInterface::getAttachedCollisionObject_(const std::string& id)
{
    std::vector<moveit_msgs::AttachedCollisionObject>& objects = getAttachedCollisionObjects_();
    for(std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        if (it->object.id == id && it->object.operation != it->object.REMOVE)
            return &*it;
    }
    return NULL;
}

const moveit_msgs::AttachedCollisionObject* PlanningSceneInterface::getAttachedCollisionObject(const std::string& id)
{
    return getAttachedCollisionObject_(id);
}

void PlanningSceneInterface::setRobotState(const moveit_msgs::RobotState& state)
{
	planning_scene_diff_.robot_state = state;

	//spsdService.request.planning_scene_diff.robot_state = state;
    // set floating links according to robot pose... (some kind of pr2_python hack?)
//    if (state.joint_state.name[0] == "floating_trans_x")
//    {
//        moveit_msgs::RobotState& state = spsdService.request.planning_scene_diff.robot_state;
//        state.joint_state.position[0] = state.multi_dof_joint_state.poses[0].position.x;
//        state.joint_state.position[1] = state.multi_dof_joint_state.poses[0].position.y;
//        state.joint_state.position[2] = state.multi_dof_joint_state.poses[0].position.z;
//        state.joint_state.position[3] = state.multi_dof_joint_state.poses[0].orientation.x;
//        state.joint_state.position[4] = state.multi_dof_joint_state.poses[0].orientation.y;
//        state.joint_state.position[5] = state.multi_dof_joint_state.poses[0].orientation.z;
//        state.joint_state.position[6] = state.multi_dof_joint_state.poses[0].orientation.w;
//    }
}

void PlanningSceneInterface::addObject(const moveit_msgs::CollisionObject& object)
{
    std::vector<moveit_msgs::CollisionObject>& objects = getCollisionObjects_();
    for(std::vector<moveit_msgs::CollisionObject>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        if (it->id == object.id)
        {
            objects.erase(it);
            break;
        }
    }
    std::vector<moveit_msgs::AttachedCollisionObject>& attachedObjects = getAttachedCollisionObjects_();
    for(std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = attachedObjects.begin(); it != attachedObjects.end(); it++)
    {
        if (it->object.id == object.id)
        {
            attachedObjects.erase(it);
            break;
        }
    }
    getCollisionObjects_().push_back(object);
    getCollisionObjects_().back().operation = object.ADD;
}

void PlanningSceneInterface::removeObject(const std::string& id)
{
    std::vector<moveit_msgs::CollisionObject>& objects = getCollisionObjects_();
    for(std::vector<moveit_msgs::CollisionObject>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        if (it->id == id)
        {
            it->operation = it->REMOVE;
            it->header.stamp = ros::Time::now();
            return;
        }
    }
    std::vector<moveit_msgs::AttachedCollisionObject>& attachedObjects = getAttachedCollisionObjects_();
    for(std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = attachedObjects.begin(); it != attachedObjects.end(); it++)
    {
        if (it->object.id == id)
        {
            it->object.operation = it->object.REMOVE;
            it->object.header.stamp = ros::Time::now();
            return;
        }
    }
}

void PlanningSceneInterface::updateObject(const std::string& id, const geometry_msgs::Pose& pose)
{
    moveit_msgs::CollisionObject* object = getCollisionObject_(id);
    if (object == NULL)
    {
        moveit_msgs::AttachedCollisionObject* attached = getAttachedCollisionObject_(id);
        if (attached == NULL)
        {
            ROS_ERROR_NAMED(logName_, "no object %s exists in planning scene.", id.c_str());
            return;
        }
        object = &attached->object;
    }
    object->primitive_poses[0] = pose;
    object->operation = object->ADD;
    object->header.stamp = ros::Time::now();
}

void PlanningSceneInterface::attachObjectToGripper(const std::string& id, const std::string& arm)
{
    moveit_msgs::CollisionObject* object = getCollisionObject_(id);
    const HandDescription& hand = HandDescription::get(arm);
    if (object == NULL)
    {
        moveit_msgs::AttachedCollisionObject* attached = getAttachedCollisionObject_(id);
        if (attached == NULL)
        {
            ROS_ERROR("%s no object %s exists in planning scene.", logName_.c_str(), id.c_str());
            return;
        }
        if (attached->link_name == hand.getAttachLink())
        {
            ROS_INFO("%s object %s already attached at the correct arm.", logName_.c_str(), id.c_str());
            return;
        }
        ROS_INFO("%s object %s attached at wrong arm. detaching.", logName_.c_str(), id.c_str());
        detachObjectAndAdd(id);
        object = getCollisionObject_(id);
    }
    //TODO: Rearrange. First create attachedCollisionObject and afterwards push_back
    getAttachedCollisionObjects_().push_back(moveit_msgs::AttachedCollisionObject());
    moveit_msgs::AttachedCollisionObject& attached = getAttachedCollisionObjects_().back();
    attached.object = *object;
    attached.link_name = hand.getAttachLink();
    attached.touch_links = hand.getTouchLinks();
    attached.object.operation = attached.object.ADD;
    attached.object.header.stamp = ros::Time::now();
    // TODO: attached.object.poses[0]);
    Transformer::transform(hand.getAttachLink(), object->header.frame_id, getRobotState(), attached.object.primitive_poses[0]);
    attached.object.header.frame_id = hand.getAttachLink();
    object->operation = object->REMOVE;
//    eraseObject(object->id);
}

void PlanningSceneInterface::detachObjectAndAdd(const std::string& id)
{
    moveit_msgs::AttachedCollisionObject* attached = getAttachedCollisionObject_(id);
    if (attached == NULL)
    {
        moveit_msgs::CollisionObject* object = getCollisionObject_(id);
        if (object == NULL)
        {
            ROS_ERROR_NAMED(logName_, "no object %s exists in planning scene.", id.c_str());
            return;
        }
        ROS_INFO_NAMED(logName_, "object %s is not attached in planning scene.", id.c_str());
        return;
    }
    getCollisionObjects_().push_back(attached->object);
    moveit_msgs::CollisionObject& object = getCollisionObjects_().back();
    // TODO: attached.object.poses[0]);
    Transformer::transform(globalFrame_, object.header.frame_id, getRobotState_(), object.primitive_poses[0]);
    object.operation = object.ADD;
    object.header.frame_id = globalFrame_;
    object.header.stamp = ros::Time::now();
    attached->object.operation = attached->object.REMOVE;
//    eraseAttachedObject(attached->object.id);
}

void PlanningSceneInterface::printDiffToCurrent(const moveit_msgs::PlanningScene& other) const
{
    printDiff(planning_scene_diff_, other);
}

void PlanningSceneInterface::printDiff(const moveit_msgs::PlanningScene& scene, const moveit_msgs::PlanningScene& other)
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

void PlanningSceneInterface::printDiff(const moveit_msgs::RobotState& state, const moveit_msgs::RobotState& other)
{
    ROS_INFO("RobotState diff:");
    if (isDifferent((const geometry_msgs::Transform)state.multi_dof_joint_state.transforms[0],
    		(const geometry_msgs::Transform)other.multi_dof_joint_state.transforms[0]))
    {
        ROS_INFO_STREAM("robot transform: " << state.multi_dof_joint_state.transforms[0] << " vs " << other.multi_dof_joint_state.transforms[0]);
    }
    double epsilon = 0.01; // rad
    for (std::size_t i = 0; i < state.joint_state.position.size(); i++)
    {
        if (fabs((float)state.joint_state.position[i] - (float)other.joint_state.position[i]) > epsilon)
        {
            // different angle: print joint name
            ROS_INFO_STREAM("joint " << state.joint_state.name[i] << " " << state.joint_state.position[i] << " vs " << other.joint_state.position[i]);
        }
    }
}

// TODO:
bool PlanningSceneInterface::isDifferent(const geometry_msgs::Transform& transform, const geometry_msgs::Transform& other)
{
    double epsilon = 0.001; // 1mm
    bool different = false;
    if (fabs(transform.translation.x - other.translation.x) > epsilon)
        different = true;
    if (fabs(transform.translation.y - other.translation.y) > epsilon)
        different = true;
    if (fabs(transform.translation.z - other.translation.z) > epsilon)
        different = true;
    if (fabs(transform.rotation.x - other.rotation.x) > epsilon)
        different = true;
    if (fabs(transform.rotation.y - other.rotation.y) > epsilon)
        different = true;
    if (fabs(transform.rotation.z - other.rotation.z) > epsilon)
        different = true;
    if (fabs(transform.rotation.w - other.rotation.w) > epsilon)
        different = true;
    return different;
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
        const std::vector<moveit_msgs::CollisionObject>& objectList,
        const std::vector<moveit_msgs::CollisionObject>& other)
{
    // object ordering might be different
    for (std::vector<moveit_msgs::CollisionObject>::const_iterator objectIt = objectList.begin();
            objectIt != objectList.end(); objectIt++)
    {
        for (std::vector<moveit_msgs::CollisionObject>::const_iterator otherIt = other.begin();
                otherIt != other.end(); otherIt++)
        {
            if (objectIt->id == otherIt->id)
            {
                if (objectIt->operation != otherIt->operation)
                {
                    ROS_INFO_STREAM("object: " << objectIt->id << " operation: " << objectIt->operation << " vs " << otherIt->operation);
                    continue;
                }
                if (isDifferent((const geometry_msgs::Pose)objectIt->primitive_poses[0], (const geometry_msgs::Pose)otherIt->primitive_poses[0]))
                {
                    ROS_INFO_STREAM("object: " << objectIt->id << " pose: " << objectIt->primitive_poses[0] << " vs " << otherIt->primitive_poses[0]);
                    continue;
                }
            }
        }
        ROS_INFO_STREAM("object: " << objectIt->id << " not found in other");
    }
}

void PlanningSceneInterface::printDiff(const std::vector<moveit_msgs::AttachedCollisionObject>& objectList,
            const std::vector<moveit_msgs::AttachedCollisionObject>& other)
{
    // object ordering might be different
    for (std::vector<moveit_msgs::AttachedCollisionObject>::const_iterator objectIt = objectList.begin();
            objectIt != objectList.end(); objectIt++)
    {
        for (std::vector<moveit_msgs::AttachedCollisionObject>::const_iterator otherIt = other.begin();
                otherIt != other.end(); otherIt++)
        {
            if (objectIt->object.id == otherIt->object.id)
            {
                if (objectIt->object.operation != otherIt->object.operation)
                {
                    ROS_INFO_STREAM("object: " << objectIt->object.id << " operation: " << objectIt->object.operation << " vs " << otherIt->object.operation);
                    continue;
                }
                if (objectIt->link_name == otherIt->link_name)
                {
                    ROS_INFO_STREAM("object: " << objectIt->object.id << " link_name: " << objectIt->link_name << " vs " << otherIt->link_name);
                    continue;
                }
            }
        }
        ROS_INFO_STREAM("object: " << objectIt->object.id << " not found in other");
    }
}

void PlanningSceneInterface::printAttachedObject(const moveit_msgs::AttachedCollisionObject& object)
{
    ROS_INFO_STREAM("attached: " << object.object.id << " at " << object.link_name);
}

void PlanningSceneInterface::printObjects(const moveit_msgs::PlanningScene& scene)
{
    const std::vector <moveit_msgs::AttachedCollisionObject>& attachedObjects = scene.robot_state.attached_collision_objects;
    for (std::vector <moveit_msgs::AttachedCollisionObject>::const_iterator attachedIt = attachedObjects.begin(); attachedIt != attachedObjects.end(); attachedIt++)
    {
        ROS_INFO_STREAM("attached: " << attachedIt->object.id << " operation: " << attachedIt->object.operation << " at " << attachedIt->link_name);
    }
    const std::vector <moveit_msgs::CollisionObject>& objects = scene.world.collision_objects;
    for (std::vector <moveit_msgs::CollisionObject>::const_iterator objectIt = objects.begin(); objectIt != objects.end(); objectIt++)
    {
        ROS_INFO_STREAM("object:   " << objectIt->id << " operation: " << objectIt->operation);
    }
}

void PlanningSceneInterface::test()
{
    resetPlanningScene();
    printObjects(planning_scene_diff_);

    // attach cup_0
    attachObjectToGripper("cup_0", "right_arm");
//    removeObject("cup_0");
    sendDiff();
    printObjects(planning_scene_diff_);

    // attach cup_0
//    attachObjectToGripper("cup_0", "right_arm");
//    sendDiff();
//    printObjects();

    // detach cup_0
//    detachObjectAndAdd("cup_0");
//    sendDiff();
//    printObjects();
}


#include "tidyup_utils/planning_scene_msg_interface.h"
#include "tidyup_utils/hand_description.h"
#include "tidyup_utils/transformer.h"

PlanningSceneMsgInterface* PlanningSceneMsgInterface::instance_ = NULL;
const std::string PlanningSceneMsgInterface::DEFAULT_PLANNING_SCENE_TOPIC = "virtual_planning_scene";

PlanningSceneMsgInterface* PlanningSceneMsgInterface::getInstance(const std::string& planning_scene_topic)
{
	if (instance_ == NULL)
		instance_ = new PlanningSceneMsgInterface(planning_scene_topic);

	return instance_;
}


PlanningSceneMsgInterface::PlanningSceneMsgInterface(const std::string& planning_scene_topic)
	: get_planning_scene_service_name_("/get_planning_scene"), planning_scene_topic_(planning_scene_topic)
{
	ROS_INFO_STREAM(planning_scene_topic);

    while (!ros::service::waitForService(get_planning_scene_service_name_, ros::Duration(3.0)))
    {
        ROS_ERROR_THROTTLE(1, "PlanningSceneMsgInterface::%s: Service %s not available - waiting.",
        	__func__, get_planning_scene_service_name_.c_str());
    }

    get_planning_scene_client_ = ros::service::createClient<moveit_msgs::GetPlanningScene>(get_planning_scene_service_name_, true);
    if (!get_planning_scene_client_)
	{
		ROS_FATAL("PlanningSceneMsgInterface::%s: Could not initialize %s service (client name: %s)",
				__func__, get_planning_scene_service_name_.c_str(), get_planning_scene_client_.getService().c_str());
		return;
	}

    // Get entire planning scene
    moveit_msgs::PlanningSceneComponents components;
    components.components =
		//moveit_msgs::PlanningSceneComponents::ALLOWED_COLLISION_MATRIX |
		//moveit_msgs::PlanningSceneComponents::LINK_PADDING_AND_SCALING |
		moveit_msgs::PlanningSceneComponents::OBJECT_COLORS |
		//moveit_msgs::PlanningSceneComponents::OCTOMAP |
		moveit_msgs::PlanningSceneComponents::ROBOT_STATE |
		moveit_msgs::PlanningSceneComponents::ROBOT_STATE_ATTACHED_OBJECTS |
		moveit_msgs::PlanningSceneComponents::SCENE_SETTINGS |
		moveit_msgs::PlanningSceneComponents::TRANSFORMS |
		moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_GEOMETRY |
		moveit_msgs::PlanningSceneComponents::WORLD_OBJECT_NAMES;

    get_planning_scene_service_.request.components = components;

    if (!get_planning_scene_client_.call(get_planning_scene_service_))
    {
        ROS_ERROR("PlanningSceneMsgInterface::%s: Could not set planning scene.", __func__);
        return;
    }
	planning_scene_ = get_planning_scene_service_.response.scene;

	ros::NodeHandle nh;
	if (!nh.ok())
	{
		ROS_FATAL("Could not initialize a nodehandle.");
		return;
	}

	planning_scene_publisher_ = nh.advertise<moveit_msgs::PlanningScene>(planning_scene_topic_ ,1, true);

	globalFrame_ = getRobotState_().multi_dof_joint_state.header.frame_id;
	ROS_ASSERT(globalFrame_ == "/map");
    ROS_INFO("PlanningSceneMsgInterface initialized.");
}

PlanningSceneMsgInterface::~PlanningSceneMsgInterface()
{

}

void PlanningSceneMsgInterface::setPlanningScene(const moveit_msgs::PlanningScene& scene)
{
	planning_scene_ = scene;
}

void PlanningSceneMsgInterface::publishPlanningScene()
{
	ROS_INFO("PlanningSceneMsgInterface::%s: sending planning scene diff on topic: %s", __func__, planning_scene_topic_.c_str());
//	planning_scene_.is = true;
//	ROS_WARN_STREAM(planning_scene_.robot_state);
	planning_scene_publisher_.publish(planning_scene_);
}

void PlanningSceneMsgInterface::resetPlanningScene()
{
	planning_scene_ = moveit_msgs::PlanningScene();
}

const moveit_msgs::CollisionObject* PlanningSceneMsgInterface::getCollisionObject(const std::string& id)
{
    return getCollisionObject_(id);
}

moveit_msgs::CollisionObject* PlanningSceneMsgInterface::getCollisionObject_(const std::string& id)
{
    std::vector<moveit_msgs::CollisionObject>& objects = getCollisionObjects_();
    for(std::vector<moveit_msgs::CollisionObject>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        if (it->id == id && it->operation != it->REMOVE)
            return &*it;
    }
    return NULL;
}

const moveit_msgs::AttachedCollisionObject* PlanningSceneMsgInterface::getAttachedCollisionObject(const std::string& id)
{
    return getAttachedCollisionObject_(id);
}

moveit_msgs::AttachedCollisionObject* PlanningSceneMsgInterface::getAttachedCollisionObject_(const std::string& id)
{
    std::vector<moveit_msgs::AttachedCollisionObject>& objects = getAttachedCollisionObjects_();
    for(std::vector<moveit_msgs::AttachedCollisionObject>::iterator it = objects.begin(); it != objects.end(); it++)
    {
        if (it->object.id == id && it->object.operation != it->object.REMOVE)
            return &*it;
    }
    return NULL;
}

void PlanningSceneMsgInterface::setRobotState(const moveit_msgs::RobotState& state)
{
	planning_scene_.robot_state = state;
	// deviation from old planning_scene_interface
}
void PlanningSceneMsgInterface::addObject(const moveit_msgs::CollisionObject& object)
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

void PlanningSceneMsgInterface::removeObject(const std::string& id)
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

void PlanningSceneMsgInterface::updateObject(const std::string& id, const geometry_msgs::Pose& pose)
{
    moveit_msgs::CollisionObject* object = getCollisionObject_(id);
    if (object == NULL)
    {
        moveit_msgs::AttachedCollisionObject* attached = getAttachedCollisionObject_(id);
        if (attached == NULL)
        {
            ROS_ERROR("PlanningSceneMsgInterface::%s: No object %s exists in planning scene.", __func__, id.c_str());
            return;
        }
        object = &attached->object;
    }
    object->primitive_poses[0] = pose;
    object->operation = object->ADD;
    object->header.stamp = ros::Time::now();
}

void PlanningSceneMsgInterface::attachObjectToGripper(const std::string& id, const std::string& arm)
{
    moveit_msgs::CollisionObject* object = getCollisionObject_(id);
    const HandDescription& hand = HandDescription::get(arm);
    if (object == NULL)
    {
        moveit_msgs::AttachedCollisionObject* attached = getAttachedCollisionObject_(id);
        if (attached == NULL)
        {
            ROS_ERROR("PlanningSceneMsgInterface::%s: no object %s exists in planning scene.", __func__, id.c_str());
            return;
        }
        if (attached->link_name == hand.getAttachLink())
        {
            ROS_INFO("PlanningSceneMsgInterface::%s: object %s already attached at the correct arm.", __func__, id.c_str());
            return;
        }
        ROS_INFO("PlanningSceneMsgInterface::%s: object %s attached at wrong arm. detaching.", __func__, id.c_str());
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
    if (!attached.object.mesh_poses.empty())
    	Transformer::transform(hand.getAttachLink(), object->header.frame_id, getRobotState(), attached.object.mesh_poses[0]);
    else if (!attached.object.primitive_poses.empty())
    	Transformer::transform(hand.getAttachLink(), object->header.frame_id, getRobotState(), attached.object.primitive_poses[0]);
    else
    {
    	ROS_ERROR("PlanningSceneMsgInterface::%s: object %s does not have a pose", __func__, attached.object.id.c_str());
    }
    attached.object.header.frame_id = hand.getAttachLink();
//    attached.object.header.frame_id = "base_footprint";
//    ROS_ERROR("attached frame id: %s", hand.getAttachLink().c_str());
    object->operation = object->REMOVE;
//    eraseObject(object->id);
}

void PlanningSceneMsgInterface::detachObjectAndAdd(const std::string& id)
{
    moveit_msgs::AttachedCollisionObject* attached = getAttachedCollisionObject_(id);
    if (attached == NULL)
    {
        moveit_msgs::CollisionObject* object = getCollisionObject_(id);
        if (object == NULL)
        {
            ROS_ERROR("PlanningSceneMsgInterface::%s: no object %s exists in planning scene.", __func__, id.c_str());
            return;
        }
        ROS_INFO("PlanningSceneMsgInterface::%s: object %s is not attached in planning scene.", __func__, id.c_str());
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

void PlanningSceneMsgInterface::printDiffToCurrent(const moveit_msgs::PlanningScene& other) const
{
	printDiff(planning_scene_, other);
}

void PlanningSceneMsgInterface::printDiff(const moveit_msgs::PlanningScene& scene, const moveit_msgs::PlanningScene& other)
{
    // robot joints
//    ROS_INFO("---------------------");
//    printDiff(scene.robot_state, other.robot_state);
//    // attached objects
//    printDiff(scene.attached_collision_objects, other.attached_collision_objects);
//    // objects
    printDiff(scene.world.collision_objects, other.world.collision_objects);
//    ROS_INFO("--------------------- objects");
//    printObjects(scene);
//    ROS_INFO("--------------------- other objects");
//    printObjects(other);
//    ROS_INFO("---------------------");
}

void PlanningSceneMsgInterface::printDiff(const std::vector<moveit_msgs::CollisionObject>& objectList,
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
        ROS_INFO_STREAM("object: " << objectIt->id <<  " removed in other");
    }
}

void PlanningSceneMsgInterface::printDiff(const std::vector<moveit_msgs::AttachedCollisionObject>& objectList,
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
        ROS_INFO_STREAM("object: " << objectIt->object.id << " removed in other");
    }
}

bool PlanningSceneMsgInterface::isDifferent(const geometry_msgs::Pose& pose, const geometry_msgs::Pose& other)
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


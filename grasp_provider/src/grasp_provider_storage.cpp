/*
 * Adapted from: moveit_ros_warehouse/warehouse/src/state_storage.cpp
 */

#include "grasp_provider/grasp_provider_storage.h"

namespace grasp_provider
{

const std::string GraspProviderStorage::DATABASE_NAME = "moveit_grasp_providers";

const std::string GraspProviderStorage::OBJECT_KEY_NAME = "key";
const std::string GraspProviderStorage::OBJECT_DB_NAME = "db";

GraspProviderStorage::GraspProviderStorage(const std::string & host, unsigned int port, double wait_seconds) :
    moveit_warehouse::MoveItMessageStorage(host, port, wait_seconds)
{
    createCollections();
    ROS_DEBUG("GraspProviderStorage connected to db %s on host %s port %d.",
            DATABASE_NAME.c_str(), db_host_.c_str(), db_port_);
}


void GraspProviderStorage::addGraspProvider(const grasp_provider_msgs::GraspProvider & msg,
        const object_recognition_msgs::ObjectType & type)
{
    bool replace = false;
    if(hasGraspProvider(type)) {
        removeGraspProvider(type);
        replace = true;
    }
    mongo_ros::Metadata metadata(OBJECT_KEY_NAME, type.key,
            OBJECT_DB_NAME, type.db);
    grasp_provider_collection_->insert(msg, metadata);
    ROS_INFO("%s GraspProvider (%s, %s) -> %s, %d, %d", replace ? "Replaced to " : "Added", type.key.c_str(), type.db.c_str(), msg.provider_name.c_str(), msg.shape_type, msg.grasp_types);
}

bool GraspProviderStorage::hasGraspProvider(const object_recognition_msgs::ObjectType & type) const
{
    mongo_ros::Query q(OBJECT_KEY_NAME, type.key);
    q.append(OBJECT_DB_NAME, type.db);
    std::vector<GraspProviderWithMetadata> ret = grasp_provider_collection_->pullAllResults(q, true);
    return !ret.empty();
}

std::vector<object_recognition_msgs::ObjectType> GraspProviderStorage::getKnownGraspProviders() const
{
    std::vector<object_recognition_msgs::ObjectType> ret;

    mongo_ros::Query q;
    std::vector<GraspProviderWithMetadata> gwm = grasp_provider_collection_->pullAllResults(q, true,
            OBJECT_DB_NAME, true);

    for(std::vector<GraspProviderWithMetadata>::iterator it = gwm.begin(); it != gwm.end(); ++it) {
        object_recognition_msgs::ObjectType ot;
        if((*it)->metadata.hasField(OBJECT_KEY_NAME.c_str()))
            ot.key = (*it)->lookupString(OBJECT_KEY_NAME);
        if((*it)->metadata.hasField(OBJECT_DB_NAME.c_str()))
            ot.db = (*it)->lookupString(OBJECT_DB_NAME);
        ret.push_back(ot);
    }
    return ret;
}

std::vector<GraspProviderStorageEntry> GraspProviderStorage::getAllGraspProviders() const
{
    std::vector<GraspProviderStorageEntry> ret;

    mongo_ros::Query q;
    std::vector<GraspProviderWithMetadata> gwm = grasp_provider_collection_->pullAllResults(q, false,
            OBJECT_DB_NAME, true);

    for(std::vector<GraspProviderWithMetadata>::iterator it = gwm.begin(); it != gwm.end(); ++it) {
        object_recognition_msgs::ObjectType ot;
        if((*it)->metadata.hasField(OBJECT_KEY_NAME.c_str()))
            ot.key = (*it)->lookupString(OBJECT_KEY_NAME);
        if((*it)->metadata.hasField(OBJECT_DB_NAME.c_str()))
            ot.db = (*it)->lookupString(OBJECT_DB_NAME);
        ret.push_back(std::make_pair(ot, *(*it)));
    }

    return ret;
}

bool GraspProviderStorage::getGraspProvider(GraspProviderWithMetadata & msg,
        const object_recognition_msgs::ObjectType & type) const
{
    mongo_ros::Query q(OBJECT_KEY_NAME, type.key);
    q.append(OBJECT_DB_NAME, type.db);
    std::vector<GraspProviderWithMetadata> gwm = grasp_provider_collection_->pullAllResults(q, false);
    if(gwm.empty())
        return false;

    if(gwm.size() > 1)
        ROS_WARN("Found %zu GraspProviders for (%s, %s)", gwm.size(), type.key.c_str(), type.db.c_str());
    msg = gwm.front();
    return true;
}

void GraspProviderStorage::removeGraspProvider(const object_recognition_msgs::ObjectType & type)
{
    mongo_ros::Query q(OBJECT_KEY_NAME, type.key);
    q.append(OBJECT_DB_NAME, type.db);
    unsigned int removed = grasp_provider_collection_->removeMessages(q);
    ROS_INFO("Removed %d GraspProvider messages for (%s, %s)", removed, type.key.c_str(), type.db.c_str());
}

void GraspProviderStorage::reset()
{
    grasp_provider_collection_.reset();
    MoveItMessageStorage::drop(DATABASE_NAME);
    createCollections();
}

void GraspProviderStorage::createCollections()
{
    grasp_provider_collection_.reset(new GraspProviderCollection::element_type(DATABASE_NAME, "grasp_providers",
                db_host_, db_port_, timeout_));
}

}


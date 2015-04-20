#ifndef GRASP_PROVIDER_STORAGE_H
#define GRASP_PROVIDER_STORAGE_H

#include <moveit/warehouse/moveit_message_storage.h>
#include <grasp_provider_msgs/GraspProvider.h>
#include <object_recognition_msgs/ObjectType.h>

namespace grasp_provider
{

typedef mongo_ros::MessageWithMetadata<grasp_provider_msgs::GraspProvider>::ConstPtr GraspProviderWithMetadata;
typedef boost::shared_ptr<mongo_ros::MessageCollection<grasp_provider_msgs::GraspProvider> > GraspProviderCollection;

typedef std::pair<object_recognition_msgs::ObjectType,
        mongo_ros::MessageWithMetadata<grasp_provider_msgs::GraspProvider> > GraspProviderStorageEntry;

class GraspProviderStorage : public moveit_warehouse::MoveItMessageStorage
{
    public:
        static const std::string DATABASE_NAME;

        static const std::string OBJECT_KEY_NAME;   ///< object_recognition_msgs/ObjectType
        static const std::string OBJECT_DB_NAME;    ///< The db name from ORK, not this one

        GraspProviderStorage(const std::string & host = "", unsigned int port = 0, double wait_seconds = 5.0);

        void addGraspProvider(const grasp_provider_msgs::GraspProvider & msg,
                const object_recognition_msgs::ObjectType & type);

        bool hasGraspProvider(const object_recognition_msgs::ObjectType & type) const;

        std::vector<object_recognition_msgs::ObjectType> getKnownGraspProviders() const;
        std::vector<GraspProviderStorageEntry> getAllGraspProviders() const;

        bool getGraspProvider(GraspProviderWithMetadata & msg, const object_recognition_msgs::ObjectType & type) const;

        void removeGraspProvider(const object_recognition_msgs::ObjectType & type);

        void reset();

    protected:
        void createCollections();

        GraspProviderCollection grasp_provider_collection_;
};

}

#endif


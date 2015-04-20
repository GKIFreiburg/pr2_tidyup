#ifndef GRASP_PROVIDER_H
#define GRASP_PROVIDER_H

#include "grasp_provider/grasp_provider_storage.h"
#include <moveit_msgs/CollisionObject.h>
#include "moveit_simple_grasps/moveit_simple_grasps.h"
#include "moveit_simple_grasps/grasp_data.h"
#include <map>
#include <string>
#include <actionlib/server/simple_action_server.h>
#include "grasp_provider_msgs/GenerateGraspsAction.h"

namespace grasp_provider
{
    /// Interface to generate grasps for a CollisionObject based on available methods.
    class GraspProvider
    {
        public:
            GraspProvider();

            std::vector<moveit_msgs::Grasp> generateGrasps(const moveit_msgs::CollisionObject & co,
                    const std::string & eef_group_name);

        protected:
            void executeCallback(const grasp_provider_msgs::GenerateGraspsGoalConstPtr & goal);

            bool loadGraspData(const std::string & eef_group_name);

        protected:
            moveit_simple_grasps::MoveitSimpleGrasps simple_grasps_;

            std::map<std::string, moveit_simple_grasps::GraspData> grasp_data_; ///< grasp data for each eef

            ros::NodeHandle nh_;
            actionlib::SimpleActionServer<grasp_provider_msgs::GenerateGraspsAction> as_;

            GraspProviderStorage grasp_provider_db_;
    };
}

#endif


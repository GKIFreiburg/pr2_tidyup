#include "grasp_provider/grasp_provider.h"
#include <shape_msgs/SolidPrimitive.h>

namespace grasp_provider
{

GraspProvider::GraspProvider() : as_(nh_, "generate_grasps",
        boost::bind(&GraspProvider::executeCallback, this, _1), false)
{
    as_.start();
}

void GraspProvider::executeCallback(const grasp_provider_msgs::GenerateGraspsGoalConstPtr & goal)
{
    if(goal->eef_group_name.empty()) {
        ROS_ERROR("GenerateGraspsGoal had not eef_group_name");
        as_.setAborted();
        return;
    }
    grasp_provider_msgs::GenerateGraspsResult result;
    result.grasps = generateGrasps(goal->collision_object, goal->eef_group_name);
    if(result.grasps.empty())
        as_.setAborted();
    else
        as_.setSucceeded(result);
}

std::vector<moveit_msgs::Grasp> GraspProvider::generateGrasps(const moveit_msgs::CollisionObject & co,
        const std::string & eef_group_name)
{
    std::vector<moveit_msgs::Grasp> possible_grasps;
    std::map<std::string, moveit_simple_grasps::GraspData>::iterator grasp_data = grasp_data_.find(eef_group_name);
    if(grasp_data == grasp_data_.end()) {
        if(!loadGraspData(eef_group_name)) {
            ROS_ERROR("Failed to load grasp data for %s.", eef_group_name.c_str());
            return possible_grasps;
        }
        grasp_data = grasp_data_.find(eef_group_name);
        ROS_ASSERT(grasp_data != grasp_data_.end());
    }

    std::vector<int> mesh_shape_types;
    // Get mesh_shape_types from db
    GraspProviderWithMetadata gpwm;
    if(!grasp_provider_db_.getGraspProvider(gpwm, co.type)) {
        if(!co.meshes.empty()) {
            ROS_ERROR("%s: Object %s had meshes defined, but could not find GraspProvider in db for (%s, %s)",
                    __func__, co.id.c_str(), co.type.key.c_str(), co.type.db.c_str());
            return possible_grasps;
        } else {
            // no meshes, no info -> use the shapes
            ROS_WARN("%s: No grasp provider found for object %s (%s, %s). Falling back to shape based with enclosure",
                    __func__, co.id.c_str(), co.type.key.c_str(), co.type.db.c_str());
            simple_grasps_.generateShapeGrasps(co, true, false,
                    grasp_data->second, mesh_shape_types, possible_grasps);
            return possible_grasps;
        }
    }

    // we have a GraspProvider
    if(gpwm->provider_name != "moveit_simple_grasps") {      // only one that we know about
        ROS_ERROR("%s: GraspProvider type %s for Object %s unsupported.",
                __func__, gpwm->provider_name.c_str(), co.id.c_str());
        return possible_grasps;
    }
    mesh_shape_types.resize(co.meshes.size(), gpwm->shape_type);
    bool enclosure = gpwm->grasp_types & grasp_provider_msgs::GraspProvider::ENCLOSURE_GRASPS;
    bool edge = gpwm->grasp_types & grasp_provider_msgs::GraspProvider::EDGE_GRASPS;

    simple_grasps_.generateShapeGrasps(co, enclosure, edge, grasp_data->second, mesh_shape_types, possible_grasps);

    return possible_grasps;
}

bool GraspProvider::loadGraspData(const std::string & eef_group_name)
{
    moveit_simple_grasps::GraspData grasp_data;
    ros::NodeHandle nh("~");
    if(!grasp_data.loadRobotGraspData(nh, eef_group_name))
        return false;

    grasp_data_[eef_group_name] = grasp_data;
    return true;
}

}


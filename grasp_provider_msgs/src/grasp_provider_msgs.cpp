#include "grasp_provider_msgs/grasp_provider_msgs.h"
#include "grasp_provider_msgs/GraspProvider.h"
#include <shape_msgs/SolidPrimitive.h>
#include <sstream>

namespace grasp_provider_msgs
{

std::string shapeTypeToString(int shape_type)
{
    switch(shape_type) {
        case shape_msgs::SolidPrimitive::BOX:
            return "Box";
            break;
        case shape_msgs::SolidPrimitive::SPHERE:
            return "Sphere";
            break;
        case shape_msgs::SolidPrimitive::CYLINDER:
            return "Cylinder";
            break;
        case shape_msgs::SolidPrimitive::CONE:
            return "Cone";
            break;
        case grasp_provider_msgs::GraspProvider::BOWL:
            return "Bowl";
            break;
    }

    std::stringstream ss;
    ss << "<Unknown Shape Type " << shape_type << ">";
    return ss.str();
}

std::string graspTypesToString(unsigned char grasp_types)
{
    switch(grasp_types) {
        case grasp_provider_msgs::GraspProvider::ENCLOSURE_GRASPS:
            return "Enclosure Grasps";
            break;
        case grasp_provider_msgs::GraspProvider::EDGE_GRASPS:
            return "Edge Grasps";
            break;
        case grasp_provider_msgs::GraspProvider::ALL_GRASPS:
            return "All Grasps";
            break;
    }

    std::stringstream ss;
    ss << "<Unknown Grasp Types " << static_cast<int>(grasp_types) << ">";
    return ss.str();
}

std::vector<int> getAllShapeTypes()
{
    std::vector<int> ret;
    ret.push_back(shape_msgs::SolidPrimitive::BOX);
    ret.push_back(shape_msgs::SolidPrimitive::SPHERE);
    ret.push_back(shape_msgs::SolidPrimitive::CYLINDER);
    ret.push_back(shape_msgs::SolidPrimitive::CONE);
    ret.push_back(grasp_provider_msgs::GraspProvider::BOWL);
    return ret;
}

std::vector<unsigned char> getAllGraspTypes()
{
    std::vector<unsigned char> ret;
    ret.push_back(grasp_provider_msgs::GraspProvider::ENCLOSURE_GRASPS);
    ret.push_back(grasp_provider_msgs::GraspProvider::EDGE_GRASPS);
    ret.push_back(grasp_provider_msgs::GraspProvider::ALL_GRASPS);
    return ret;
}

}


#ifndef GRASP_PROVIDER_MSGS_H
#define GRASP_PROVIDER_MSGS_H

#include <string>
#include <vector>

namespace grasp_provider_msgs
{
    std::string shapeTypeToString(int shape_type);

    std::string graspTypesToString(unsigned char grasp_types);

    std::vector<int> getAllShapeTypes();
    std::vector<unsigned char> getAllGraspTypes();
}

#endif


#ifndef GET_POSE_STAMPED_FROM_PARAM_H_
#define GET_POSE_STAMPED_FROM_PARAM_H_

#include <string>
#include <geometry_msgs/PoseStamped.h>

namespace tidyup_utils
{
	bool getPoseStampedFromParam(const std::string& name, geometry_msgs::PoseStamped& pose);
};

#endif /* GET_POSE_STAMPED_FROM_PARAM_H_ */

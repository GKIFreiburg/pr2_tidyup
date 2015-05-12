#include "tidyup_utils/get_pose_stamped_from_param.h"
#include <ros/ros.h>

namespace tidyup_utils
{
	bool getPoseStampedFromParam(const std::string& name, geometry_msgs::PoseStamped& pose)
	{
		std::string parameter = name + "/header/frame_id";
		if (!ros::param::get(parameter, pose.header.frame_id))
		{
			ROS_ERROR("%s: Could not fetch %s from param", __func__, parameter.c_str());
			return false;
		}

		parameter = name + "/header/stamp";
		double stamp;
		if (!ros::param::get(parameter, stamp))
		{
			ROS_WARN("%s: Could not fetch %s from param", __func__, parameter.c_str());
		} else
			pose.header.stamp = ros::Time(stamp);

		parameter = name + "/pose/position/x";
		if (!ros::param::get(parameter, pose.pose.position.x))
		{
			ROS_ERROR("%s: Could not fetch %s from param", __func__, parameter.c_str());
			return false;
		}

		parameter = name + "/pose/position/y";
		if (!ros::param::get(parameter, pose.pose.position.y))
		{
			ROS_ERROR("%s: Could not fetch %s from param", __func__, parameter.c_str());
			return false;
		}

		parameter = name + "/pose/position/z";
		if (!ros::param::get(parameter, pose.pose.position.z))
		{
			ROS_ERROR("%s: Could not fetch %s from param", __func__, parameter.c_str());
			return false;
		}

		parameter = name + "/pose/orientation/x";
		if (!ros::param::get(parameter, pose.pose.orientation.x))
		{
			ROS_ERROR("%s: Could not fetch %s from param", __func__, parameter.c_str());
			return false;
		}

		parameter = name + "/pose/orientation/y";
		if (!ros::param::get(parameter, pose.pose.orientation.y))
		{
			ROS_ERROR("%s: Could not fetch %s from param", __func__, parameter.c_str());
			return false;
		}

		parameter = name + "/pose/orientation/z";
		if (!ros::param::get(parameter, pose.pose.orientation.z))
		{
			ROS_ERROR("%s: Could not fetch %s from param", __func__, parameter.c_str());
			return false;
		}

		parameter = name + "/pose/orientation/w";
		if (!ros::param::get(parameter, pose.pose.orientation.w))
		{
			ROS_ERROR("%s: Could not fetch %s from param", __func__, parameter.c_str());
			return false;
		}

		return true;
	}

};

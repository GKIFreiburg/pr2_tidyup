#ifndef ARMS_AT_SIDE_H_
#define ARMS_AT_SIDE_H_

#include <ros/ros.h>
#include <tidyup_msgs/ArmsAtSide.h>

namespace tidyup
{

	class armsAtSide
	{
		public:
		armsAtSide();
		~armsAtSide();

		static bool checkIfArmsAtSide(tidyup_msgs::ArmsAtSide::Request &req,
								tidyup_msgs::ArmsAtSide::Response &res);

	};


};

#endif /* ARMS_AT_SIDE_H_ */

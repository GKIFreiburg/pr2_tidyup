#ifndef UNIFORM_SAMPLING_H_
#define UNIFORM_SAMPLING_H_

#include "manipulation_location_generator/manipulation_location_generator_interface.h"
#include <geometry_msgs/PoseStamped.h>

namespace manipulation_location_generator
{

	class UniformSampling : public ManipulationLocationGeneratorInterface
	{
		public:
			UniformSampling(double sample_dist_table, double sample_angle_table, long int seed = 0);
			~UniformSampling();

			// Generate sample around table
			virtual geometry_msgs::Pose generateSample();

		private:
			double sample_dist_table_;
			double sample_angle_table_;

			ros::Publisher pub_samples_;

			// check that position (x, y) is not inside table
			bool isSampleValid(const double& x, const double& y);
	};

};


#endif /* UNIFORM_SAMPLING_H_ */

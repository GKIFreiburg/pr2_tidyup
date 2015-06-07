#ifndef MANIPULATION_LOCATION_GENERATOR_H_
#define MANIPULATION_LOCATION_GENERATOR_H_

#include <ros/ros.h>
#include <manipulation_location_generator_msgs/CreateManipulationLocation.h>
#include <manipulation_location_generator/manipulation_location_generator_interface.h>

namespace manipulation_location_generator
{
	enum SamplingMethod
	{
		UNIFORM_SAMPLING = 0,
		CUSTOM_SAMPLING = 1
	};

	class ManipulationLocationGenerator
	{

		public:
			ManipulationLocationGenerator();
			~ManipulationLocationGenerator();

		protected:
			ros::NodeHandle nh_;
			ros::ServiceServer srv_;

//			ManipulationLocationGeneratorInterface* mlgi_;
			boost::shared_ptr<ManipulationLocationGeneratorInterface> mlgi_;

			bool executeCallBack(manipulation_location_generator_msgs::CreateManipulationLocationRequest &req,
					manipulation_location_generator_msgs::CreateManipulationLocationResponse &res);

			void selectSamplingMethod(unsigned sampling_method);

			std::string getSamplingMethod(unsigned sampling_method);
	};

};

#endif /* MANIPULATION_LOCATION_GENERATOR_H_ */

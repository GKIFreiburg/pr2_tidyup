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
			ros::ServiceServer mani_loc_gen_server_;

//			ManipulationLocationGeneratorInterface* mlgi_;
			boost::shared_ptr<ManipulationLocationGeneratorInterface> mlgi_;

			// Service Callback, parse input and generate output
			bool executeCallBack(manipulation_location_generator_msgs::CreateManipulationLocationRequest &req,
					manipulation_location_generator_msgs::CreateManipulationLocationResponse &res);

			// Initialize sampling method and sets mlgi_
			void selectSamplingMethod(unsigned sampling_method);

			// Get the name of the chosen sampling_method
			std::string getSamplingMethod(unsigned sampling_method);
	};

};

#endif /* MANIPULATION_LOCATION_GENERATOR_H_ */

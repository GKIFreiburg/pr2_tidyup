#include "manipulation_location_generator/manipulation_location_generator.h"
#include <manipulation_location_generator/uniform_sampling.h>


namespace manipulation_location_generator
{

	ManipulationLocationGenerator::ManipulationLocationGenerator()
	{
		srv_ = nh_.advertiseService("generate_manipulation_locations",
				&ManipulationLocationGenerator::executeCallBack, this);


	}

	ManipulationLocationGenerator::~ManipulationLocationGenerator()
	{

	}

	bool ManipulationLocationGenerator::executeCallBack(manipulation_location_generator_msgs::CreateManipulationLocationRequest &req,
						manipulation_location_generator_msgs::CreateManipulationLocationResponse &res)
	{
		selectSamplingMethod(req.sampling_method);
		ROS_ASSERT(mlgi_ != NULL);

		mlgi_->initialize(req.planning_scene_topic, req.table, req.max_samples, req.attempts);
		ManipulationLocations mani_locs = mlgi_->generateSamples();

		res.manipulation_locations = mani_locs;

		return true;
	}

	void ManipulationLocationGenerator::selectSamplingMethod(unsigned sampling_method)
	{
		ros::NodeHandle nhPriv("~");
		switch (sampling_method) {
			case UNIFORM_SAMPLING:
				double sample_dist_table;
				nhPriv.param("sample_dist_table", sample_dist_table, 0.5);

				double sample_angle_table;
				nhPriv.param("sample_angle_table", sample_angle_table, M_PI);

				ROS_INFO("ManipulationLocationGenerator::%s:"
						"Sampling method: %s with parameters:\n"
						"sample_dist_table: %lf \n"
						"sample_angle_table: %lf", __func__, getSamplingMethod(sampling_method).c_str(),
						sample_dist_table, sample_angle_table);

//				mlgi_ = new UniformSampling(dist_table_edge);
				mlgi_.reset(new UniformSampling(sample_dist_table, sample_angle_table));
				break;
			case CUSTOM_SAMPLING:
				break;
			default:
				break;
		}
	}

	std::string ManipulationLocationGenerator::getSamplingMethod(unsigned sampling_method)
	{
		std::string method;
		switch (sampling_method)
		{
			case UNIFORM_SAMPLING:
				method = "UNIFORM_SAMPLING";
				break;
			case CUSTOM_SAMPLING:
				method = "CUSTOM_SAMPLING";
				break;
			default:
				method = "INVALID_METHOD";
				break;
		}

		return method;
	}

};

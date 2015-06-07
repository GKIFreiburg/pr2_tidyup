#include "manipulation_location_generator/manipulation_location_generator.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "manipulation_location_generator");
    ros::NodeHandle nh;

    manipulation_location_generator::ManipulationLocationGenerator mlg;

    ros::spin();
}


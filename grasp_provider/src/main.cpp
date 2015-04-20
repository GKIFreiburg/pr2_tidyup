#include <ros/ros.h>
#include "grasp_provider/grasp_provider.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_provider");
    ros::NodeHandle nh;

    grasp_provider::GraspProvider grasp_provider;

    ros::spin();
}


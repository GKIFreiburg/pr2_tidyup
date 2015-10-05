#include <stdio.h>
#include <ros/ros.h>
#include <ros/package.h>
#include <vector>
#include <deque>
#include <string>
#include <sstream>
#include <fstream>

#include <tidyup_utils/geometryPoses.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "setLocationsAndObjects");
    // needed by wait command
    ros::NodeHandle nh;

    if (argc != 4)
    {
    	ROS_ERROR("Usage: rosrun experiments_evaluation setCansAndCubes <config_pkg> <num Cans> <num Cubes>");
    	EXIT_FAILURE;
    }

    std::string config_pkg = argv[1];
    int num_cans = atoi(argv[2]);
    int num_cubes = atoi(argv[3]);

    std::stringstream eval;
    std::string path = ros::package::getPath("experiments_evaluation");
    path += "/eval/" + config_pkg;
    std::string cmd = "mkdir " + path;
    ROS_INFO("PATH: %s", cmd.c_str());
    system(cmd.c_str());
    eval << config_pkg << "/can_" << num_cans << "_cube_" << num_cubes << ".eval";
    ros::param::set("evaluation", eval.str());

    // read cans to set
    // create 2nd GeometryPoses Objects, otherwise object poses are added to location poses
    // these objects only live for a short time period
    GeometryPoses cans = GeometryPoses();
    std::string cansFile = ros::package::getPath("experiments_evaluation");
    cansFile += "/config/cans.dat";
    if (!cans.load(cansFile))
    {
        ROS_ERROR("setCansAndCubes::%s: Could not load cans from \"%s\".", __func__, cansFile.c_str());
        return EXIT_FAILURE;
    }

    // prepare ros commands and launch them via system call
    int count = 0;
    std::map<std::string, geometry_msgs::PoseStamped>::const_iterator it;
    for (it = cans.getPoses().begin(); it != cans.getPoses().end(); it++)
    {
    	if (count == num_cans)
    		break;
    	else
    	{
    		std::stringstream command;
    		command << "rosrun gazebo_ros spawn_model -urdf -file $(rospack find object_detection)/objects/coke/coke.urdf -model ";
    		command << it->first << " -x " << it->second.pose.position.x << " -y " << it->second.pose.position.y << " -z " << it->second.pose.position.z;

    		ROS_INFO_STREAM("Executing the following command: \n" << command.str());
    		int ret = system(command.str().c_str());
    		if (ret != 0)
    			EXIT_FAILURE;
    		int seconds = 1;
    		ROS_INFO("Wait for %d seconds, until object is spawned", seconds);
    		ros::Duration(seconds).sleep();
    	}
    	count++;
    }

    GeometryPoses cubes = GeometryPoses();
    std::string cubesFile = ros::package::getPath("experiments_evaluation");
    cubesFile += "/config/cubes.dat";
    if (!cubes.load(cubesFile))
    {
        ROS_ERROR("setCansAndCubes::%s: Could not load cubes from \"%s\".", __func__, cansFile.c_str());
        return EXIT_FAILURE;
    }

    // prepare ros commands and launch them via system call
    count = 0;
    for (it = cubes.getPoses().begin(); it != cubes.getPoses().end(); it++)
    {
    	if (count == num_cubes)
    		break;
    	else
    	{
    		std::stringstream command;
    		command << "rosrun gazebo_ros spawn_model -urdf -file $(rospack find object_detection)/objects/cube/cube.urdf -model ";
    		command << it->first << " -x " << it->second.pose.position.x << " -y " << it->second.pose.position.y << " -z " << it->second.pose.position.z;

    		ROS_INFO_STREAM("Executing the following command: \n" << command.str());
    		int ret = system(command.str().c_str());
    		if (ret != 0)
    			EXIT_FAILURE;
    		int seconds = 1;
    		ROS_INFO("Wait for %d seconds, until object is spawned", seconds);
    		ros::Duration(seconds).sleep();
    	}
    	count++;
    }

    ros::shutdown();
    return 0;
}

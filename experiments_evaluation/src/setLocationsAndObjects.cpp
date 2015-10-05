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
    	ROS_ERROR("Usage: rosrun experiments_evaluation setLocationsAndObjects <config_pkg> <num Locs> <num Objs>");
    	EXIT_FAILURE;
    }

    std::string config_pkg = argv[1];
    int num_locations = atoi(argv[2]);
    int num_objects = atoi(argv[3]);

    std::stringstream eval;
    std::string path = ros::package::getPath("experiments_evaluation");
    path += "/eval/" + config_pkg;
    std::string cmd = "mkdir " + path;
    ROS_INFO("PATH: %s", cmd.c_str());
    system(cmd.c_str());
    eval << config_pkg << "/loc_" << num_locations << "_obj_" << num_objects << ".eval";
    ros::param::set("evaluation", eval.str());

    // read locations to set
    GeometryPoses locations = GeometryPoses();
    std::string locationsFile = ros::package::getPath("experiments_evaluation");
    locationsFile += "/config/locations.dat";
    if(!locations.load(locationsFile)) {
        ROS_ERROR("setLocationsAndObjects::%s: Could not load locations from \"%s\".", __func__, locationsFile.c_str());
        return EXIT_FAILURE;
    }

    // open target file
    std::string targetLocationsFile = ros::package::getPath(config_pkg);
    targetLocationsFile += "/config/locations.dat";
    std::ofstream file(targetLocationsFile.c_str());
    if (!file.is_open())
    {
    	ROS_ERROR("Could not open target locations file!");
    	return EXIT_FAILURE;
    }

    // write target file
    int count = 0;
    std::map<std::string, geometry_msgs::PoseStamped>::const_iterator it;
    for (it = locations.getPoses().begin(); it != locations.getPoses().end(); it++)
    {
    	if (count == num_locations)
    		break;
    	else
    	{
    		std::string loc = locations.getPoseWriteString(*it);
			file << loc << "\n";
    	}
    	count++;
    }
    file << "\n";
    file.close();


    // read objects to set
    // create 2nd GeometryPoses Objects, otherwise object poses are added to location poses
    // these objects only live for a short time period
    GeometryPoses objects = GeometryPoses();
    std::string objectsFile = ros::package::getPath("experiments_evaluation");
    objectsFile += "/config/objects.dat";
    if (!objects.load(objectsFile))
    {
        ROS_ERROR("setLocationsAndObjects::%s: Could not load objects from \"%s\".", __func__, objectsFile.c_str());
        return EXIT_FAILURE;
    }

    // prepare ros commands and launch them via system call
    count = 0;
    for (it = objects.getPoses().begin(); it != objects.getPoses().end(); it++)
    {
    	if (count == num_objects)
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

    ros::shutdown();
    return 0;
}

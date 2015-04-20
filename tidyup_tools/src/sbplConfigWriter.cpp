#include <string>
#include <sstream>
#include <iomanip>
#include <fstream>
#include <ros/ros.h>
#include <ros/package.h>

int main(int argc, char** argv)
{
    ros::init(argc, argv, "sbpl_config_writer");
    if (argc < 2)
    {
        ROS_INFO("usage: sbpl_config_writer 'config_package_name'");
        return 0;
    }
    std::string packageDir(ros::package::getPath(argv[1]));

    ROS_DEBUG("Chosen package dir: %s", packageDir.c_str());

    std::string template_dir = ros::package::getPath("tidyup_tools");
    if (template_dir.empty())
    {
        ROS_FATAL("Error: Could not find package tidyup_tools.");
        return -1;
    }

    std::string mkdir_cmd = std::string("mkdir -p ") + packageDir + "/config/planning";
    int ret = 0;
    ret = system(mkdir_cmd.c_str());
    if (ret != 0)
    {
        ROS_FATAL("Could not create config dir using \"%s\". Error: %d", mkdir_cmd.c_str(), ret);
    }
    mkdir_cmd = std::string("mkdir -p ") + packageDir + "/config/mapping";
    ret = system(mkdir_cmd.c_str());
    if (ret != 0)
    {
        ROS_FATAL("Could not create config dir using \"%s\". Error: %d", mkdir_cmd.c_str(), ret);
    }

    // Read map name from target config dir
    // create collision space config
    std::string map_file = packageDir + "/maps/octomap-tidyup.bt";
    std::string sbplTemplate1 = std::string(template_dir.c_str()) + "/tidyup_config_template/mapping/pr2_both_arms_tidyup.yaml.1";
    std::string sbplTemplate2 = std::string(template_dir.c_str()) + "/tidyup_config_template/mapping/pr2_both_arms_tidyup.yaml.2";
    std::string sbplFile = packageDir + "/config/mapping/pr2_both_arms_tidyup.yaml";

    // first create the config snippet
    std::string get_collision_map_cmd = std::string("rosrun tidyup_tools octree_to_collision_space ") + map_file + " > /tmp/col_space_template";
    ret = system(get_collision_map_cmd.c_str());
    if (ret != 0)
    {
        ROS_FATAL("Determine Collision Space for %s to /tmp/col_space_template failed.", map_file.c_str());
    }

    // next put it between the two parts of the config
    std::string set_collision_map_cmd = "cat " + sbplTemplate1 + " /tmp/col_space_template " + sbplTemplate2 + " > " + sbplFile;
    ret = system(set_collision_map_cmd.c_str());
    if (ret != 0)
    {
        ROS_FATAL("Write Collision Space from %s using %s to %s failed.", "/tmp/col_space_template", sbplTemplate1.c_str(), sbplFile.c_str());
    }

    ROS_INFO("writing Collision Space config file to %s", sbplFile.c_str());
    return 0;
}


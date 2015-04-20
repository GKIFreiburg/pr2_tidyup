/*
 * test_nav_sbpl.cpp
 *
 *  Created on: 2 Jul 2012
 *      Author: andreas
 */

#include <planner_modules_pr2/navstack_module_full_body.h>
#include <planner_modules_pr2/navstack_module.h>
#include <nav_msgs/GetPlan.h>
#include <geometry_msgs/PoseStamped.h>
#include <map>
#include <utility>
using std::pair; using std::make_pair;
#include <boost/foreach.hpp>
#ifdef __CDT_PARSER__
    #define forEach(a, b) for(a : b)
#else
    #define forEach BOOST_FOREACH
#endif
#include <sys/times.h>
#include <ios>
#include <string>
#include "tidyup_utils/geometryPoses.h"

using std::map;
using namespace std;

int main(int argc, char* argv[])
{
    if(argc != 3)
    {
        ROS_WARN("usage: plan_sbpl_to_location <location_file> <location_name>");
        exit(0);
    }

    ros::init(argc, argv, "test_nav_sbpl", ros::init_options::AnonymousName);

    // init module code and services
    char* fake_argv[4];
    fake_argv[0] = "bla";
    fake_argv[1] = "/map";
    fake_argv[2] = "0.05";
    fake_argv[3] = "1";
    fullbody_navstack_init(4, fake_argv);
    ros::Rate wait = 1;
    for (int i = 0; i < 5; i++)
    {
        ros::spinOnce();
        wait.sleep();
    }

    // load location file
    string locationsFile = argv[1];
    string goalLocationName = argv[2];
//    ROS_INFO("file_name: %s", locationsFile.c_str());
    GeometryPoses locations = GeometryPoses();
    if (!locations.load(locationsFile))
    {
        ROS_ERROR("Could not load locations from \"%s\".", locationsFile.c_str());
        return -1;
    }
    const geometry_msgs::PoseStamped* poseStampedPtr = NULL;
    forEach(const GeometryPoses::NamedPose & np, locations.getPoses())
    {
        if(np.first == goalLocationName)
        {
            poseStampedPtr = &(np.second);
            break;
        }
    }
    if (poseStampedPtr == NULL)
    {
        ROS_ERROR("specified location \"%s\" not found in location file.", goalLocationName.c_str());
        return -1;
    }

    // set arm state
    publishPlanningArmState();
    // send make_plan request
    nav_msgs::GetPlan srv;
    srv.request.goal = *poseStampedPtr;
    bool serviceCallSuccessfull = false;
    double cost = callPlanningService(srv, "current_location", goalLocationName, serviceCallSuccessfull);
    switchToExecutionTopic();
    if (cost < INFINITE_COST)
        ROS_INFO("Test SUCCESSFUL: found plan to location \"%s\". length: %0.2f", goalLocationName.c_str(), cost);
    else
        ROS_ERROR("Test FAILED: no plan to location \"%s\".", goalLocationName.c_str());
//    ros::spinOnce();
}


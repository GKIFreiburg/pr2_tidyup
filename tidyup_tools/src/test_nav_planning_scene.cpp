/*
 * test_nav_sbpl.cpp
 *
 *  Created on: 2 Jul 2012
 *      Author: andreas
 */

#include <planner_modules_pr2/navstack_planning_scene_module.h>
#include <planner_modules_pr2/navstack_module.h>
#include <planner_modules_pr2/module_param_cache.h>
#include "tidyup_utils/planning_scene_interface.h"
#include "tidyup_utils/stringutil.h"
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
        ROS_WARN("usage: plan_to_location_planning_scene <location_file> <location_name>");
        exit(0);
    }

    // init module code and services
    ros::init(argc, argv, "test_nav_planning_scene", ros::init_options::AnonymousName);

    char* fake_argv[4];
    fake_argv[0] = "bla";
    fake_argv[1] = "/map";
    fake_argv[2] = "0.05";
    fake_argv[3] = "1";
    ROS_ASSERT_MSG(ros::param::has("/continual_planning_executive/door_location_file"), "Door location parameter missing: /continual_planning_executive/door_location_file");

    planning_scene_navstack_init(4, fake_argv);

    // load location file
    string locationsFile = argv[1];
    string goalLocationName = argv[2];
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

    // set planning scene
    PlanningSceneInterface::instance()->resetPlanningScene();
//    const vector<arm_navigation_msgs::CollisionObject>& objectList = PlanningSceneInterface::instance()->getCollisionObjects();
//    vector<arm_navigation_msgs::CollisionObject>::const_iterator objectIterator = objectList.begin();
//    for ( ; objectIterator != objectList.end(); objectIterator++)
//    {
//        // move door +2m in z direction (aka hack open)
//        if (StringUtil::startsWith(objectIterator->id, "door"))
//        {
//            geometry_msgs::Pose pose = objectIterator->poses[0];
//            pose.position.z += 2;
//            PlanningSceneInterface::instance()->updateObject(objectIterator->id, pose);
//        }
//    }
//    PlanningSceneInterface::instance()->sendDiff();

    double cost = 0;
//    string cacheKey = computePathCacheKey("current_location", goalLocationName);
//    if (!g_PathCostCache.get(cacheKey, cost))
    {
        // send make_plan request
        nav_msgs::GetPlan srv;
        srv.request.goal = *poseStampedPtr;
        bool serviceCallSuccessful = false;
        cost = callPlanningService(srv, "current_location", goalLocationName, serviceCallSuccessful);
        if (serviceCallSuccessful)
        { // only cache real computed paths (including INFINITE_COST)
//            g_PathCostCache.set(cacheKey, cost);
        }
    }
    if (cost < INFINITE_COST)
        ROS_INFO("Test SUCCESSFUL: found plan to location \"%s\". length: %0.2f", goalLocationName.c_str(), cost);
    else
        ROS_ERROR("Test FAILED: no plan to location \"%s\".", goalLocationName.c_str());
}


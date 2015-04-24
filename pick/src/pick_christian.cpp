#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <shape_tools/solid_primitive_dims.h>
#include <actionlib/client/simple_action_client.h>
#include <grasp_provider_msgs/GenerateGraspsAction.h>
#include <visualization_msgs/MarkerArray.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

ros::ServiceClient srvPlanningScene_;
ros::Publisher pubVis_;

std::string g_table;


boost::shared_ptr<planning_scene_monitor::PlanningSceneMonitor> g_psm;

double z_above_table = 0.01;

std::vector<moveit_msgs::CollisionObject> getCollisionObjectsFromPlanningScene()
{
    planning_scene_monitor::LockedPlanningSceneRO ps(g_psm);

    moveit_msgs::PlanningScene psMsg;
    ps->getPlanningSceneMsg(psMsg);

    return psMsg.world.collision_objects;
}

std::vector<moveit_msgs::CollisionObject> getAttachedCollisionObjectsFromPlanningScene()
{
    planning_scene_monitor::LockedPlanningSceneRO ps(g_psm);

    //const robot_model::RobotModelConstPtr & robot_model = ps->getRobotModel();
    //const robot_state::RobotState & robot_state = ps->getCurrentState();

    moveit_msgs::PlanningScene psMsg;
    ps->getPlanningSceneMsg(psMsg);

    std::vector<moveit_msgs::CollisionObject> ret;
    forEach(const moveit_msgs::AttachedCollisionObject & aco, psMsg.robot_state.attached_collision_objects) {
        ret.push_back(aco.object);
    }
    return ret;
}


bool getObjectFromPlanningScene(const std::string & name, moveit_msgs::CollisionObject & collision_object)
{
    std::vector<moveit_msgs::CollisionObject> cos = getCollisionObjectsFromPlanningScene();
    forEach(const moveit_msgs::CollisionObject & co, cos) {
        if(co.id == name) {
            collision_object = co;
            return true;
        }
    }
    return false;
}

bool pick(const std::string & object, const std::string & arm, moveit::planning_interface::MoveGroup &group,
        actionlib::SimpleActionClient<grasp_provider_msgs::GenerateGraspsAction> & generate_grasps)
{
    moveit_msgs::CollisionObject co;
    if(!getObjectFromPlanningScene(object, co)) {
        ROS_ERROR("Could not get object %s from planning scene.", object.c_str());
        return false;
    }
    grasp_provider_msgs::GenerateGraspsGoal grasps;
    grasps.collision_object = co;
    grasps.eef_group_name = arm + "_gripper";

    generate_grasps.sendGoal(grasps);
    ROS_INFO("Waiting for grasps");

    if(generate_grasps.waitForResult(ros::Duration(30.0))) {
        ROS_INFO("Got grasps - performing pick");
        return group.pick(object, generate_grasps.getResult()->grasps);
    } else {
        ROS_ERROR("Could not generate grasps for %s", co.id.c_str());
        return false;
    }
}

int main(int argc, char **argv)
{
    ros::init (argc, argv, "pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;

    srvPlanningScene_ = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    pubVis_ = nh.advertise<visualization_msgs::MarkerArray>("pick_place", 1);

    actionlib::SimpleActionClient<grasp_provider_msgs::GenerateGraspsAction> ac("generate_grasps", true);
    ROS_INFO("Waiting for generate_grasps action.");
    ac.waitForServer();

    g_psm.reset(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    // update manually, don't start monitors that continuously update
    g_psm->requestPlanningSceneState();

    ros::WallDuration(1.0).sleep();

    if(argc != 5) {
        ROS_FATAL("Usage: %s <table> <arm> pick|place|pickplace <object>", argv[0]);
        return 1;
    }
    std::string table = argv[1];
    std::string arm = argv[2];
    std::string command = argv[3];
    std::string object = argv[4];
    if(command != "pick" && command != "place" && command != "pickplace" && command != "debug_place") {
        ROS_FATAL("Usage: %s <table> <arm (right/left)> pick|place|pickplace|debug_place <object>", argv[0]);
        return 1;
    }

    moveit::planning_interface::MoveGroup group(arm + "_arm");
    group.setPlanningTime(45.0);
    if(table != "-") {
        g_table = table;
        group.setSupportSurfaceName(table);
    }

    bool pick_success = false;
    if(command == "pick" || command == "pickplace") {
        ROS_INFO("picking");
        pick_success = pick(object, arm, group, ac);
        ROS_INFO("pick done");
    }

    return 0;
}


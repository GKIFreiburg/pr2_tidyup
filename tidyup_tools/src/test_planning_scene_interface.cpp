/*
 * test_nav_sbpl.cpp
 *
 *  Created on: 2 Jul 2012
 *      Author: andreas
 */

#include "tidyup_utils/planning_scene_interface.h"
#include "tidyup_utils/transformer.h"
#include "tidyup_msgs/GetPutdownPose.h"
#include "tidyup_utils/arm_state.h"
#include <sstream>
using std::map;
using namespace std;

geometry_msgs::Pose defaultAttachPose;
geometry_msgs::Pose robotPose; // 2.90178 -3.04816 0.0509599 0 0 0.848432 0.529304
string separator = " ";

void attachObjectToArm(const string& putdown_object, const string& arm)
{
    PlanningSceneInterface* psi = PlanningSceneInterface::instance();
    psi->resetPlanningScene();
    arm_navigation_msgs::PlanningScene original = psi->getCurrentScene();

    ROS_INFO("attaching object %s to arm %s", putdown_object.c_str(), arm.c_str());
    psi->attachObjectToGripper(putdown_object, arm);
    geometry_msgs::Pose pose = defaultAttachPose;
    psi->updateObject(putdown_object, pose);

    arm_navigation_msgs::RobotState state = psi->getRobotState();
    state.multi_dof_joint_state.poses[0] = robotPose;
    ArmState::get("/arm_configurations/side_tuck/position/", "left_arm").replaceJointPositions(state.joint_state);
    ArmState::get("/arm_configurations/side_tuck/position/", "right_arm").replaceJointPositions(state.joint_state);
    ArmState::get("/arm_configurations/side_carry/position/", arm).replaceJointPositions(state.joint_state);
    psi->setRobotState(state);

//    ROS_INFO_STREAM("before send: " << *psi->getAttachedCollisionObject(putdown_object));
    psi->sendDiff();
//    ROS_INFO_STREAM("after send: " << *psi->getAttachedCollisionObject(putdown_object));
    psi->printDiffToCurrent(original);
//    psi->printObjects();

    tidyup_msgs::GetPutdownPose srv;
    srv.request.arm = arm;
    srv.request.putdown_object = putdown_object;
    srv.request.static_object = "table2_loc4";
    if (ros::service::call("/tidyup/request_putdown_pose", srv))
    {
        if (srv.response.error_code.val == arm_navigation_msgs::ArmNavigationErrorCodes::SUCCESS)
            ROS_INFO_STREAM("response: " << srv.response.putdown_pose);
        else
            ROS_ERROR_STREAM("failed: " << srv.response.error_code);
    }
    else
    {
        ROS_ERROR_STREAM("service call failed.");
    }
}

void writePoseToString(std::stringstream& stream, const geometry_msgs::Pose& pose)
{
    stream << pose.position.x << separator;
    stream << pose.position.y << separator;
    stream << pose.position.z << separator;
    stream << pose.orientation.x << separator;
    stream << pose.orientation.y << separator;
    stream << pose.orientation.z << separator;
    stream << pose.orientation.w << separator;
}

bool readPoseFromString(std::stringstream& stream, geometry_msgs::Pose& pose)
{
    vector<double> coordinates;
    coordinates.resize(7);
    for(size_t i = 0; ! stream.eof() && i < coordinates.size(); i++)
    {
        stream >> coordinates[i];
    }
    if (!stream.good())
        return false;
    pose.position.x = coordinates[0];
    pose.position.y = coordinates[1];
    pose.position.z = coordinates[2];
    pose.orientation.x = coordinates[3];
    pose.orientation.y = coordinates[4];
    pose.orientation.z = coordinates[5];
    pose.orientation.w = coordinates[6];
    return true;
}

int main(int argc, char* argv[])
{
    // init module code and services
    ros::init(argc, argv, "test_planning_scene_interface", ros::init_options::AnonymousName);
    ros::NodeHandle nh;
    ros::spinOnce();

    defaultAttachPose.position.x = 0.032;
    defaultAttachPose.position.y = 0.015;
    defaultAttachPose.position.z = 0.0;
    defaultAttachPose.orientation.x = 0.707;
    defaultAttachPose.orientation.y = -0.106;
    defaultAttachPose.orientation.z = -0.690;
    defaultAttachPose.orientation.w = 0.105;

    robotPose.position.x = 2.90178;
    robotPose.position.y = -3.04816;
    robotPose.position.z = 0.0509599;
    robotPose.orientation.x = 0.0;
    robotPose.orientation.y = 0.0;
    robotPose.orientation.z = 0.848432;
    robotPose.orientation.w = 0.529304;

//    std::stringstream formatter;
//    formatter.precision(3);
//    formatter << std::fixed;
//
//    writePoseToString(formatter, defaultAttachPose);
////    writePoseToString(formatter, robotPose);
//
//    string serialized = formatter.str();
//    ROS_INFO("serialized: %s", serialized.c_str());
//
////    std::stringstream formatter2;
////    formatter2.precision(3);
////    formatter2 << std::fixed;
////    formatter2 << serialized;
//
//    formatter.flush();
////    formatter << serialized;

//    geometry_msgs::Pose newPose1;
//    geometry_msgs::Pose newPose2;
//
//    if (! readPoseFromString(formatter, newPose1))
//    {
//        ROS_ERROR("pose decode error");
//    }
//    else
//    {
//        ROS_INFO_STREAM("pose1: "<< newPose1);
//    }
//    if (! readPoseFromString(formatter, newPose2))
//    {
//        ROS_ERROR("pose2 decode error");
//    }
//    else
//    {
//        ROS_INFO_STREAM("pose2: "<< newPose2);
//    }
//
//    return 0;

//    PlanningSceneInterface::instance()->test();
    attachObjectToArm("cup_0", "left_arm");
    attachObjectToArm("cup_0", "right_arm");
    attachObjectToArm("cup_1", "left_arm");
    attachObjectToArm("cup_1", "right_arm");

//    ros::spin();
}


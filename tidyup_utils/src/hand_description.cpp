/*
 * hand_description.cpp
 *
 *  Created on: 2 Aug 2012
 *      Author: andreas
 */

#include "tidyup_utils/hand_description.h"
using std::string;
using std::map;

map<string, HandDescription> HandDescription::descriptions;

const HandDescription& HandDescription::get(const std::string& arm_name)
{
    map<string, HandDescription>::iterator it = descriptions.find(arm_name);
    if (it == descriptions.end())
    {
        descriptions.insert(make_pair(arm_name, HandDescription(arm_name)));
        it = descriptions.find(arm_name);
    }
    return it->second;
}


HandDescription::HandDescription(const std::string& arm) : armName(arm)
{
    string prefix = armName.substr(0, 1);
    if( ! ros::param::get("/hand_description/"+armName+"/hand_group_name", handGroup))
    {
        handGroup = prefix + "_end_effector";
    }

    if( ! ros::param::get("/hand_description/"+armName+"/attach_link", attachLink))
    {
        attachLink = prefix + "_gripper_r_finger_tip_link";
    }

    if( ! ros::param::get("/hand_description/"+armName+"/hand_frame", handFrame))
    {
        handFrame = prefix + "_wrist_roll_link";
    }

    if( ! ros::param::get("/hand_description/"+armName+"/gripper_joint", gripperJoint))
    {
        gripperJoint = prefix + "_gripper_joint";
    }

    if( ! ros::param::get("/hand_description/"+armName+"/end_effector_length", endEffectorLength))
    {
        endEffectorLength = 0.18;
    }

    string paramPath = "/hand_description/"+armName+"/hand_links";
    if (ros::param::has(paramPath))
    {
        XmlRpc::XmlRpcValue paramList;
        ros::param::get(paramPath, paramList);
        ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < paramList.size(); ++i)
        {
            ROS_ASSERT(paramList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            handLinks.push_back(static_cast<string>(paramList[i]));
        }
    }
    else
    {
        handLinks.push_back(prefix+"_gripper_palm_link");
        handLinks.push_back(prefix+"_gripper_r_finger_tip_link");
        handLinks.push_back(prefix+"_gripper_l_finger_tip_link");
        handLinks.push_back(prefix+"_gripper_l_finger_link");
        handLinks.push_back(prefix+"_gripper_r_finger_link");
    }

    paramPath = "/hand_description/"+armName+"/hand_touch_links";
    if (ros::param::has(paramPath))
    {
        XmlRpc::XmlRpcValue paramList;
        ros::param::get(paramPath, paramList);
        ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < paramList.size(); ++i)
        {
            ROS_ASSERT(paramList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            touchLinks.push_back(static_cast<string>(paramList[i]));
        }
    }
    else
    {
        touchLinks = handLinks;
    }

    paramPath = "/hand_description/"+armName+"/hand_fingertip_links";
    if (ros::param::has(paramPath))
    {
        XmlRpc::XmlRpcValue paramList;
        ros::param::get(paramPath, paramList);
        ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < paramList.size(); ++i)
        {
            ROS_ASSERT(paramList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            fingerTipLinks.push_back(static_cast<string>(paramList[i]));
        }
    }
    else
    {
        fingerTipLinks.push_back(prefix+"_gripper_r_finger_tip_link");
        fingerTipLinks.push_back(prefix+"_gripper_l_finger_tip_link");
    }

    paramPath = "/hand_description/"+armName+"/hand_finger_links";
    if (ros::param::has(paramPath))
    {
        XmlRpc::XmlRpcValue paramList;
        ros::param::get(paramPath, paramList);
        ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < paramList.size(); ++i)
        {
            ROS_ASSERT(paramList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            fingerLinks.push_back(static_cast<string>(paramList[i]));
        }
    }
    else
    {
        fingerLinks.push_back(prefix+"_gripper_r_finger_tip_link");
        fingerLinks.push_back(prefix+"_gripper_l_finger_tip_link");
        fingerLinks.push_back(prefix+"_gripper_r_finger_link");
        fingerLinks.push_back(prefix+"_gripper_l_finger_link");
    }

    paramPath = "/hand_description/"+armName+"/hand_approach_direction";
    if (ros::param::has(paramPath))
    {
        XmlRpc::XmlRpcValue paramList;
        ros::param::get(paramPath, paramList);
        ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        ROS_ASSERT(paramList.size() == 3);
        ROS_ASSERT(paramList[0].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        approachDirection.x = static_cast<double>(paramList[0]);
        ROS_ASSERT(paramList[1].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        approachDirection.y = static_cast<double>(paramList[1]);
        ROS_ASSERT(paramList[2].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        approachDirection.z = static_cast<double>(paramList[2]);
    }
    else
    {
        approachDirection.x = 1.0;
        approachDirection.y = 0.0;
        approachDirection.z = 0.0;
    }
}

HandDescription::~HandDescription()
{
}


/*
 * arm_state.cpp
 *
 *  Created on: 17 Jul 2012
 *      Author: andreas
 */

#include "tidyup_utils/arm_state.h"
#include <string>

using std::string;
using std::map;

map<string, ArmState> ArmState::states;

const ArmState& ArmState::get(const std::string& armStateParameter, const std::string& armName)
{
    string key = armStateParameter + armName;
    map<string, ArmState>::iterator it = states.find(key);
    if (it == states.end())
    {
        states.insert(make_pair(key, ArmState(armStateParameter, armName)));
        it = states.find(key);
    }
    return it->second;
}


ArmState::ArmState(const std::string& armStateParameter, const std::string& armName)
: armName(armName)
{
    // load joint names from param server
    string linkNames = "/hand_description/"+armName+"/arm_joints/";
    if (ros::param::has(linkNames))
    {
        XmlRpc::XmlRpcValue paramList;
        ros::param::get(linkNames, paramList);
        ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
        for (int32_t i = 0; i < paramList.size(); ++i)
        {
            ROS_ASSERT(paramList[i].getType() == XmlRpc::XmlRpcValue::TypeString);
            armState.name.push_back(static_cast<string>(paramList[i]));
        }
    }
    else
    {
        string prefix = armName.substr(0, 1);
        armState.name.push_back(prefix+"_shoulder_pan_joint");
        armState.name.push_back(prefix+"_shoulder_lift_joint");
        armState.name.push_back(prefix+"_upper_arm_roll_joint");
        armState.name.push_back(prefix+"_elbow_flex_joint");
        armState.name.push_back(prefix+"_forearm_roll_joint");
        armState.name.push_back(prefix+"_wrist_flex_joint");
        armState.name.push_back(prefix+"_wrist_roll_joint");
    }

    // load joint positions from param server
    string param = armStateParameter + armName;
    ROS_ASSERT(ros::param::has(param));
    XmlRpc::XmlRpcValue paramList;
    ros::param::get(param, paramList);
    ROS_ASSERT(paramList.getType() == XmlRpc::XmlRpcValue::TypeArray);
    for (int32_t i = 0; i < paramList.size(); ++i)
    {
        ROS_ASSERT(paramList[i].getType() == XmlRpc::XmlRpcValue::TypeDouble);
        armState.position.push_back(static_cast<double>(paramList[i]));
    }
}

void ArmState::replaceJointPositions(sensor_msgs::JointState& state) const
{
    ArmState::replaceJointPositions(state, armState);
}

void ArmState::replaceJointPositions(sensor_msgs::JointState& state, const sensor_msgs::JointState& joints)
{
    for (unsigned int i = 0; i < joints.name.size(); i++)
    {
        string name = joints.name[i];
        int index = -1;
        for (unsigned int j = 0; j < state.name.size(); j++)
        {
            if (name.compare(state.name[j]) == 0)
            {
                index = j;
            }
        }
        if (index != -1)
        {
            state.position[index] = joints.position[i];
        }
    }
}





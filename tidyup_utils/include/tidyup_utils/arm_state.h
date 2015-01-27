/*
 * arm_state.h
 *
 *  Created on: 17 Jul 2012
 *      Author: andreas
 */

#ifndef ARM_STATE_H_
#define ARM_STATE_H_

#include <ros/ros.h>
#include <sensor_msgs/JointState.h>
#include <map>


class ArmState
{
public:
    static const ArmState& get(const std::string& armStateParameter, const std::string& armName);
    static void replaceJointPositions(sensor_msgs::JointState& state, const sensor_msgs::JointState& joints);

    const std::string& getArmName() const {return armName;}
    const sensor_msgs::JointState& getJointStates() const {return armState;}
    void replaceJointPositions(sensor_msgs::JointState& state) const;

private:
    static std::map<std::string, ArmState> states;

    std::string armName;
    sensor_msgs::JointState armState;
    ArmState(const std::string& armStateParameter, const std::string& armName);
};



#endif /* ARM_STATE_H_ */

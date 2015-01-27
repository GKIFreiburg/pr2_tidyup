/*
 * hand_description.h
 *
 *  Created on: 2 Aug 2012
 *      Author: andreas
 */

#ifndef HAND_DESCRIPTION_H_
#define HAND_DESCRIPTION_H_

#include <ros/ros.h>
#include <geometry_msgs/Vector3.h>
#include <map>

class HandDescription
{
public:
    static const HandDescription& get(const std::string& arm_name);

    virtual ~HandDescription();

    const geometry_msgs::Vector3& getApproachDirection() const
    {
        return approachDirection;
    }

    const std::string& getArmName() const
    {
        return armName;
    }

    const std::string& getAttachLink() const
    {
        return attachLink;
    }

    double getEndEffectorLength() const
    {
        return endEffectorLength;
    }

    const std::vector<std::string>& getFingerLinks() const
    {
        return fingerLinks;
    }

    const std::vector<std::string>& getFingerTipLinks() const
    {
        return fingerTipLinks;
    }

    const std::string& getGripperJoint() const
    {
        return gripperJoint;
    }

    const std::string& getHandFrame() const
    {
        return handFrame;
    }

    const std::string& getHandGroup() const
    {
        return handGroup;
    }

    const std::vector<std::string>& getHandLinks() const
    {
        return handLinks;
    }

    const std::vector<std::string>& getTouchLinks() const
    {
        return touchLinks;
    }

private:
    HandDescription(const std::string& arm);

    std::string armName;
    std::string handGroup;
    std::string attachLink;
    std::string handFrame;
    std::string gripperJoint;

    std::vector<std::string> handLinks;
    std::vector<std::string> touchLinks;
    std::vector<std::string> fingerTipLinks;
    std::vector<std::string> fingerLinks;

    double endEffectorLength;
    geometry_msgs::Vector3 approachDirection;

    static std::map<std::string, HandDescription> descriptions;
};

#endif /* HAND_DESCRIPTION_H_ */

/*
 * transformer.cpp
 *
 *  Created on: 1 Aug 2012
 *      Author: andreas
 */

#include "tidyup_utils/transformer.h"

Transformer* Transformer::instance = NULL;

Transformer::Transformer() :
    robot_model("/robot_description")
{
}

Transformer::~Transformer()
{
}

std::string Transformer::relative_frame(const std::string& frame_id) const
{
    if (frame_id[0] != '/')
    {
        return frame_id;
    }
    return frame_id.substr(1, frame_id.size());
}

bool Transformer::getTransform(const std::string& to_frame,
        const std::string& from_frame,
        const arm_navigation_msgs::RobotState& robotState,
        tf::Transform& transform)
{
    planning_models::KinematicState kstate(robot_model.getKinematicModel());
    if (!planning_environment::setRobotStateAndComputeTransforms(robotState, kstate))
    {
        ROS_ERROR("Unable to transform robot state to kinematic state");
        return false;
    }

    ROS_INFO("transforming from %s to %s", from_frame.c_str(), to_frame.c_str());

    std::vector<geometry_msgs::TransformStamped> transforms;
    tf::Transform global_to_from;
    if (!getWorldTransform(from_frame, kstate, global_to_from))
    {
        ROS_ERROR("Unable to find frame_id %s in kinematic state", from_frame.c_str());
        return false;
    }
    tf::Transform global_to_to;
    if (!getWorldTransform(to_frame, kstate, global_to_to))
    {
        ROS_ERROR("Unable to find frame_id %s in kinematic state", to_frame.c_str());
        return false;
    }

    //This is the transform that will take the origin of to
    //to the origin of from.  This is how TF works so we are sticking
    //with that convention although it is confusing.  The transform
    //with from=robot frame to=wrist frame will give you the position of
    //the wrist in the robot frame for example.
    //
    //HOWEVER, the transform that takes a pose expressed in from and transforms
    //it to a pose expressed in to is the INVERSE of this transform
    transform = (global_to_to.inverse()) * global_to_from;
    return true;
}

bool Transformer::getWorldTransform(const std::string& frame_id,
        const planning_models::KinematicState& state,
        tf::Transform &transform)
{
    if (!frame_id.compare(state.getKinematicModel()->getRoot()->getParentFrameId()))
    {
        //identity transform
        transform.setIdentity();
        return true;
    }

    if (!frame_id.compare(state.getKinematicModel()->getRoot()->getChildFrameId()))
    {
        transform = state.getRootTransform();
        return true;
    }

    const planning_models::KinematicState::LinkState *link = state.getLinkState(frame_id);
    if (!link)
    {
        ROS_ERROR("Unable to find link %s in kinematic state", frame_id.c_str());
        return false;
    }

    transform = link->getGlobalLinkTransform();
    return true;
}

bool Transformer::transform(const std::string& new_frame,
        const std::string& old_frame,
        const arm_navigation_msgs::RobotState& robotState,
        geometry_msgs::Point& point)
{
    tf::Vector3 btPoint(point.x, point.y, point.z);
    tf::Transform trans;
    if (instance == NULL)
        instance = new Transformer();
    if (instance->getTransform(new_frame, old_frame, robotState, trans))
    {
        btPoint = trans * btPoint;
        point.x = btPoint.x();
        point.y = btPoint.y();
        point.z = btPoint.z();
        return true;
    }
    return false;
}

bool Transformer::transform(const std::string& new_frame,
                const std::string& old_frame,
                const arm_navigation_msgs::RobotState& robotState,
                geometry_msgs::Quaternion& quaternion)
{
    tf::Quaternion bt_Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf::Transform trans;
    if (instance == NULL)
        instance = new Transformer();
    if (instance->getTransform(new_frame, old_frame, robotState, trans))
    {
        bt_Quaternion = trans * bt_Quaternion;
        quaternion.x = bt_Quaternion.x();
        quaternion.y = bt_Quaternion.y();
        quaternion.z = bt_Quaternion.z();
        quaternion.w = bt_Quaternion.w();
        return true;
    }
    return false;
}

bool Transformer::transform(const std::string& new_frame,
            const std::string& old_frame,
            const arm_navigation_msgs::RobotState& robotState,
            geometry_msgs::Pose& pose)
{
    geometry_msgs::Point& point = pose.position;
    geometry_msgs::Quaternion& quaternion = pose.orientation;
    tf::Vector3 btPoint(point.x, point.y, point.z);
    tf::Quaternion bt_Quaternion(quaternion.x, quaternion.y, quaternion.z, quaternion.w);
    tf::Transform trans;
    if (instance == NULL)
        instance = new Transformer();
    if (instance->getTransform(new_frame, old_frame, robotState, trans))
    {
        btPoint = trans * btPoint;
        point.x = btPoint.x();
        point.y = btPoint.y();
        point.z = btPoint.z();
        bt_Quaternion = trans * bt_Quaternion;
        quaternion.x = bt_Quaternion.x();
        quaternion.y = bt_Quaternion.y();
        quaternion.z = bt_Quaternion.z();
        quaternion.w = bt_Quaternion.w();
        return true;
    }
    return false;
}

bool Transformer::transform(const std::string& new_frame,
        const std::string& old_frame,
        const arm_navigation_msgs::RobotState& robotState,
        geometry_msgs::PointStamped& point)
{
    if (transform(new_frame, old_frame, robotState, point.point))
    {
        point.header.frame_id = new_frame;
        return true;
    }
    return false;
}

bool Transformer::transform(const std::string& new_frame,
        const std::string& old_frame,
        const arm_navigation_msgs::RobotState& robotState,
        geometry_msgs::QuaternionStamped& quaternion)
{
    if (transform(new_frame, old_frame, robotState, quaternion.quaternion))
    {
        quaternion.header.frame_id = new_frame;
        return true;
    }
    return false;
}

bool Transformer::transform(const std::string& new_frame,
        const std::string& old_frame,
        const arm_navigation_msgs::RobotState& robotState,
        geometry_msgs::PoseStamped& pose)
{
    if (transform(new_frame, old_frame, robotState, pose.pose))
    {
        pose.header.frame_id = new_frame;
        return true;
    }
    return false;
}

bool Transformer::transform(const std::string& new_frame,
            const arm_navigation_msgs::RobotState& robotState,
            arm_navigation_msgs::Constraints& constraint)
{
    bool ok = true;
    for(std::vector <arm_navigation_msgs::PositionConstraint>::iterator it = constraint.position_constraints.begin();
            it != constraint.position_constraints.end(); it++)
    {
        ok &= transform(new_frame, it->header.frame_id, robotState, it->position);
        ok &= transform(new_frame, it->header.frame_id, robotState, it->constraint_region_orientation);
        it->header.frame_id = new_frame;
    }
    for(std::vector <arm_navigation_msgs::OrientationConstraint>::iterator it = constraint.orientation_constraints.begin();
            it != constraint.orientation_constraints.end(); it++)
    {
        ok &= transform(new_frame, it->header.frame_id, robotState, it->orientation);
        it->header.frame_id = new_frame;
    }
    for(std::vector <arm_navigation_msgs::VisibilityConstraint>::iterator it = constraint.visibility_constraints.begin();
            it != constraint.visibility_constraints.end(); it++)
    {
        ok &= transform(new_frame, it->header.frame_id, robotState, it->target);
    }
    return ok;
}

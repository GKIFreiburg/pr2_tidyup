/*********************************************************************
 * Software License Agreement (BSD License)
 *
 *  Copyright (c) 2012, Willow Garage, Inc.
 *  All rights reserved.
 *
 *  Redistribution and use in source and binary forms, with or without
 *  modification, are permitted provided that the following conditions
 *  are met:
 *
 *   * Redistributions of source code must retain the above copyright
 *     notice, this list of conditions and the following disclaimer.
 *   * Redistributions in binary form must reproduce the above
 *     copyright notice, this list of conditions and the following
 *     disclaimer in the documentation and/or other materials provided
 *     with the distribution.
 *   * Neither the name of Willow Garage nor the names of its
 *     contributors may be used to endorse or promote products derived
 *     from this software without specific prior written permission.
 *
 *  THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
 *  "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
 *  LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS
 *  FOR A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE
 *  COPYRIGHT OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT,
 *  INCIDENTAL, SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING,
 *  BUT NOT LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES;
 *  LOSS OF USE, DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER
 *  CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT
 *  LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN
 *  ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
 *  POSSIBILITY OF SUCH DAMAGE.
 *********************************************************************/

/* Author: Sachin Chitta */

#include <ros/ros.h>

// MoveIt!
#include <moveit/robot_model_loader/robot_model_loader.h>
#include <moveit/robot_model/robot_model.h>
#include <moveit/robot_state/robot_state.h>

int main(int argc, char **argv)
{
  ros::init (argc, argv, "right_arm_kinematics");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  // BEGIN_TUTORIAL
  // Start
  // ^^^^^
  // Setting up to start using the RobotModel class is very easy. In
  // general, you will find that most higher-level components will
  // return a shared pointer to the RobotModel. You should always use
  // that when possible. In this example, we will start with such a
  // shared pointer and discuss only the basic API. You can have a
  // look at the actual code API for these classes to get more
  // information about how to use more features provided by these
  // classes.
  //
  // We will start by instantiating a
  // `RobotModelLoader`_
  // object, which will look up
  // the robot description on the ROS parameter server and construct a
  // :moveit_core:`RobotModel` for us to use.


  robot_model_loader::RobotModelLoader robot_model_loader("robot_description");
  robot_model::RobotModelPtr robot_model_ptr = robot_model_loader.getModel();
  ROS_INFO("Model frame: %s", robot_model_ptr->getModelFrame().c_str());

  // Using the :moveit_core:`RobotModel`, we can construct a
  // :moveit_core:`RobotState` that maintains the configuration
  // of the robot. We will set all joints in the state to their
  // default values. We can then get a
  // :moveit_core:`JointModelGroup`, which represents the robot
  // model for a particular group, e.g. the "right_arm" of the PR2
  // robot.
  robot_state::RobotStatePtr robot_state_ptr(new robot_state::RobotState(robot_model_ptr));
  robot_state_ptr->setToDefaultValues();
  const robot_state::JointModelGroup* joint_model_group = robot_model_ptr->getJointModelGroup("right_arm");

  const std::vector<std::string> &joint_names = joint_model_group->getJointModelNames();

  // Get Joint Values
  // ^^^^^^^^^^^^^^^^
  // We can retreive the current set of joint values stored in the state for the right arm.
  std::vector<double> joint_values;
  robot_state_ptr->copyJointGroupPositions(joint_model_group, joint_values);
  for(std::size_t i = 0; i < joint_names.size(); ++i)
  {
    ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
  }

  // Joint Limits
  // ^^^^^^^^^^^^
  // setJointGroupPositions() does not enforce joint limits by itself, but a call to enforceBounds() will do it.
  /* Set one joint in the right arm outside its joint limit */
  joint_values[0] = 1.57;
  robot_state_ptr->setJointGroupPositions(joint_model_group, joint_values);

  /* Check whether any joint is outside its joint limits */
  ROS_INFO_STREAM("Current state is " << (robot_state_ptr->satisfiesBounds() ? "valid" : "not valid"));

  /* Enforce the joint limits for this state and check again*/
  robot_state_ptr->enforceBounds();
  ROS_INFO_STREAM("Current state is " << (robot_state_ptr->satisfiesBounds() ? "valid" : "not valid"));

  // Forward Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // Now, we can compute forward kinematics for a set of random joint
  // values. Note that we would like to find the pose of the
  // "r_wrist_roll_link" which is the most distal link in the
  // "right_arm" of the robot.
  robot_state_ptr->setToRandomPositions(joint_model_group);
  const Eigen::Affine3d &end_effector_state = robot_state_ptr->getGlobalLinkTransform("r_wrist_roll_link");

  /* Print end-effector pose. Remember that this is in the model frame */
  ROS_INFO_STREAM("Translation: " << end_effector_state.translation());
  ROS_INFO_STREAM("Rotation: " << end_effector_state.rotation());

  // Inverse Kinematics
  // ^^^^^^^^^^^^^^^^^^
  // We can now solve inverse kinematics (IK) for the right arm of the
  // PR2 robot. To solve IK, we will need the following:
  //  * The desired pose of the end-effector (by default, this is the last link in the "right_arm" chain): end_effector_state that we computed in the step above.
  //  * The number of attempts to be made at solving IK: 10
  //  * The timeout for each attempt: 0.1 s

  // If the group this state corresponds to is a chain and a solver is available, then the joint values can be set by computing inverse kinematics.
  //      The pose is assumed to be in the reference frame of the kinematic model. Returns true on success.
  //      @param pose The pose the last link in the chain needs to achieve
  //      @param tip The name of the link the pose is specified for
  //      @param attempts The number of times IK is attempted
  //      @param timeout The timeout passed to the kinematics solver on each attempt
  bool found_ik = robot_state_ptr->setFromIK(joint_model_group, end_effector_state, 10, 0.1);

  // Now, we can print out the IK solution (if found):
  if (found_ik)
  {
    robot_state_ptr->copyJointGroupPositions(joint_model_group, joint_values);
    for(std::size_t i=0; i < joint_names.size(); ++i)
    {
      ROS_INFO("Joint %s: %f", joint_names[i].c_str(), joint_values[i]);
    }
  }
  else
  {
    ROS_INFO("Did not find IK solution");
  }

  // Get the Jacobian
  // ^^^^^^^^^^^^^^^^
  // We can also get the Jacobian from the :moveit_core:`RobotState`.
  Eigen::Vector3d reference_point_position(0.0,0.0,0.0);
  Eigen::MatrixXd jacobian;
  robot_state_ptr->getJacobian(joint_model_group, robot_state_ptr->getLinkModel(joint_model_group->getLinkModelNames().back()),
                               reference_point_position,
                               jacobian);
  ROS_INFO_STREAM("Jacobian: " << jacobian);
  // END_TUTORIAL

  ros::shutdown();
  return 0;
}

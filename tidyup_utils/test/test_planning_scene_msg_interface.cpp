#include <tidyup_utils/planning_scene_msg_interface.h>
#include <ros/ros.h>
#include <moveit_msgs/CollisionObject.h>
#include <eigen_conversions/eigen_msg.h>
#include <tf/tf.h>
#include <tf_conversions/tf_eigen.h>
/*
 *     name[0]: bl_caster_rotation_joint
    name[1]: bl_caster_l_wheel_joint
    name[2]: bl_caster_r_wheel_joint
    name[3]: br_caster_rotation_joint
    name[4]: br_caster_l_wheel_joint
    name[5]: br_caster_r_wheel_joint
    name[6]: fl_caster_rotation_joint
    name[7]: fl_caster_l_wheel_joint
    name[8]: fl_caster_r_wheel_joint
    name[9]: fr_caster_rotation_joint
    name[10]: fr_caster_l_wheel_joint
    name[11]: fr_caster_r_wheel_joint
    name[12]: torso_lift_joint
    name[13]: head_pan_joint
    name[14]: head_tilt_joint
    name[15]: l_shoulder_pan_joint
    name[16]: l_shoulder_lift_joint
    name[17]: l_upper_arm_roll_joint
    name[18]: l_elbow_flex_joint
    name[19]: l_forearm_roll_joint
    name[20]: l_wrist_flex_joint
    name[21]: l_wrist_roll_joint
    name[22]: l_gripper_l_finger_joint
    name[23]: l_gripper_l_finger_tip_joint
    name[24]: l_gripper_motor_slider_joint
    name[25]: l_gripper_motor_screw_joint
    name[26]: l_gripper_r_finger_joint
    name[27]: l_gripper_r_finger_tip_joint
    name[28]: l_gripper_joint
    name[29]: laser_tilt_mount_joint
    name[30]: r_shoulder_pan_joint
    name[31]: r_shoulder_lift_joint
    name[32]: r_upper_arm_roll_joint
    name[33]: r_elbow_flex_joint
    name[34]: r_forearm_roll_joint
    name[35]: r_wrist_flex_joint
    name[36]: r_wrist_roll_joint
    name[37]: r_gripper_l_finger_joint
    name[38]: r_gripper_l_finger_tip_joint
    name[39]: r_gripper_motor_slider_joint
    name[40]: r_gripper_motor_screw_joint
    name[41]: r_gripper_r_finger_joint
    name[42]: r_gripper_r_finger_tip_joint
    name[43]: r_gripper_joint
    name[44]: torso_lift_motor_screw_joint
 *
 */

void wait(double sec = 1.0)
{
	ros::Duration(sec).sleep();
	ROS_INFO("wait for %lf seconds", sec);
}

int main(int argc, char** argv)
{
    ros::init(argc, argv, "test_planning_scene_msg_interface");
    ros::NodeHandle nh;

    PlanningSceneMsgInterface* psi = PlanningSceneMsgInterface::getInstance();
    const std::vector<moveit_msgs::CollisionObject>& cos = psi->getCollisionObjects();
    ROS_INFO("Number of collision objects: %lu - address: %p", cos.size(), &cos);
    psi->publishPlanningScene();
//    ROS_WARN_STREAM(psi->getRobotState());


//    std::string id = "coke_1";
//    const moveit_msgs::CollisionObject* obj = psi->getCollisionObject(id);
//    if (obj == NULL)
//    {
//    	ROS_ERROR("Could not find object %s", id.c_str());
//    }
//
//    ROS_ASSERT(obj != NULL);
//    moveit_msgs::CollisionObject can = *obj;
//	can.id = "fanta";
//	can.header.frame_id = "r_wrist_roll_link";
//	can.mesh_poses[0].position.x = 0.18;
//	can.mesh_poses[0].position.y = 0;
//	can.mesh_poses[0].position.z = -0.07;
//	can.mesh_poses[0].orientation.x = 0;
//	can.mesh_poses[0].orientation.y = 0;
//	can.mesh_poses[0].orientation.z = 0;
//	can.mesh_poses[0].orientation.w = 1;
//
//	psi->addObject(can);
//	psi->attachObjectToGripper(can.id, "right_arm");
//	psi->publishPlanningScene();
//	const moveit_msgs::AttachedCollisionObject* aco = psi->getAttachedCollisionObject(can.id);
//	ROS_INFO_STREAM(*aco);



//    moveit_msgs::RobotState robot = psi->getRobotState();
//    robot.joint_state.position[30] = -1;
//    psi->setRobotState(robot);
////
//    const std::vector<moveit_msgs::AttachedCollisionObject> acos = psi->getAttachedCollisionObjects();
//    ROS_INFO("Number of attached collision objects: %lu", acos.size());
//    psi->publishPlanningScene();



    ros::shutdown();
}


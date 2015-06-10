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
//    ROS_WARN_STREAM(psi->getRobotState());


    const moveit_msgs::CollisionObject* obj = NULL;
    ROS_ERROR("first address %p", obj);
    if (psi->getCollisionObject("coke_1", obj))
    {
    	ROS_INFO("address: %p", obj);
    	moveit_msgs::CollisionObject test = *obj;
    	ROS_INFO("address copy: %p", &test);
    }





//    moveit_msgs::CollisionObject obj = *(psi->getCollisionObject("coke_1"));
//    obj.id = "asd";



//    moveit_msgs::CollisionObject obj;
//    ROS_WARN("1");
//    ROS_ERROR("ADDRESS: %p", &obj);
//    if (psi->getCollisionObject("coke_1", &obj))
//    {
//    	ROS_ERROR("ADDRESS: %p", &obj);
//    	ROS_WARN_STREAM(obj);
//    	ROS_WARN("2");
//		obj.id = "fanta";
//		obj.header.frame_id = "r_wrist_roll_link";
//		obj.mesh_poses[0].position.x = 0.18;
//		obj.mesh_poses[0].position.y = 0;
//		obj.mesh_poses[0].position.z = 0;
//		obj.mesh_poses[0].orientation.x = 0;
//		obj.mesh_poses[0].orientation.y = 0;
//		obj.mesh_poses[0].orientation.z = 0;
//		obj.mesh_poses[0].orientation.w = 1;
//
//		psi->addObject(obj);
//
//		psi->attachObjectToGripper("fanta", "right_arm");
//    }


//    moveit_msgs::RobotState robot = psi->getRobotState();
//    robot.joint_state.position[30] = -1;
//    psi->setRobotState(robot);
//
//    const std::vector<moveit_msgs::AttachedCollisionObject> acos = psi->getAttachedCollisionObjects();
//    ROS_INFO("Number of attached collision objects: %lu", acos.size());
//    psi->publishPlanningScene();






//    ros::ServiceClient client = nh.serviceClient
//    		<manipulation_location_generator_msgs::CreateManipulationLocation>
//    		("generate_manipulation_locations");
//
//    manipulation_location_generator_msgs::CreateManipulationLocation srv;
//    srv.request.sampling_method = manipulation_location_generator::UNIFORM_SAMPLING;
//    srv.request.planning_scene_topic = "get_planning_scene";
//    moveit_msgs::CollisionObject table = getTable("table1");
//    srv.request.table = table;
//    srv.request.max_samples = 100;
//    srv.request.attempts = 1000;
//
//    if (!client.call(srv))
//    	ROS_ERROR("service call: %s failed", client.getService().c_str());
//	std::vector<geometry_msgs::PoseStamped> mani_locs = srv.response.manipulation_locations;
//    ROS_INFO("Found %lu manipulation locations.", mani_locs.size());

    ros::shutdown();
}


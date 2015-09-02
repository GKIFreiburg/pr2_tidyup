#include <ros/ros.h>
#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/move_group_interface/move_group.h>
#include <moveit/move_group/capability_names.h>
#include <moveit_msgs/GetPlanningScene.h>
#include <shape_tools/solid_primitive_dims.h>
#include <actionlib/client/simple_action_client.h>
#include <grasp_provider_msgs/GenerateGraspsAction.h>
#include <visualization_msgs/MarkerArray.h>
#include <geometry_msgs/PoseArray.h>

#include <boost/foreach.hpp>
#define forEach BOOST_FOREACH

int main(int argc, char **argv)
{
    ros::init (argc, argv, "pick_place");
    ros::AsyncSpinner spinner(1);
    spinner.start();

    ros::NodeHandle nh;
    std::string grasps_topic = "grasps";
    ros::Publisher pub_grasps = nh.advertise<geometry_msgs::PoseArray>(grasps_topic, 1, true);
    ros::Publisher pub_ps = nh.advertise<moveit_msgs::PlanningScene>("planning_scene", 1);
	ros::ServiceClient srvPlanningScene;
    srvPlanningScene = nh.serviceClient<moveit_msgs::GetPlanningScene>(move_group::GET_PLANNING_SCENE_SERVICE_NAME);
    moveit_msgs::GetPlanningSceneRequest req;
    moveit_msgs::GetPlanningSceneResponse res;
    req.components.components = 1023; // fetch all info
    if (!srvPlanningScene.call(req, res))
    {
    	ROS_ERROR("Could not fetch planning scene!");
    	return 1;
    }

    actionlib::SimpleActionClient<grasp_provider_msgs::GenerateGraspsAction> generate_grasps("generate_grasps", true);
    ROS_INFO("Waiting for generate_grasps action.");
    generate_grasps.waitForServer();

	moveit_msgs::CollisionObject co;
	co.header.stamp = ros::Time::now();
	co.header.frame_id = "base_link";

	co.id = "coke";
	co.primitives.resize(1);
//	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
//	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.067;
//	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.067;
//	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.12;

//	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
//	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
//	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X] = 0.23;
//	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y] = 0.07;
//	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Z] = 0.30;

	co.primitives[0].type = shape_msgs::SolidPrimitive::CYLINDER;
	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::CYLINDER>::value);
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_HEIGHT] = 0.12;
	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::CYLINDER_RADIUS] = 0.067 / 2;

//	co.primitives[0].type = shape_msgs::SolidPrimitive::SPHERE;
//	co.primitives[0].dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::SPHERE>::value);
//	co.primitives[0].dimensions[shape_msgs::SolidPrimitive::SPHERE_RADIUS] = 0.05;


    geometry_msgs::PoseStamped p;
    p.header.frame_id = "base_link";
    p.header.stamp = ros::Time::now();
    p.pose.position.x = 0.50;
    p.pose.position.y = -0.75;
    p.pose.position.z = 0.58;
    p.pose.orientation.x = 0;
    p.pose.orientation.y = 0;
    p.pose.orientation.z = 0;
    p.pose.orientation.w = 1;
	co.primitive_poses.push_back(p.pose);
	co.operation = co.ADD;
	res.scene.world.collision_objects.push_back(co);
	pub_ps.publish(res.scene);

    grasp_provider_msgs::GenerateGraspsGoal grasps;
    grasps.collision_object = co;
    //grasps.eef_group_name = arm + "_gripper";
    grasps.eef_group_name = "right_gripper";

    generate_grasps.sendGoal(grasps);
    ROS_INFO("Waiting for grasps");

    geometry_msgs::PoseArray grasps_array;


    if(generate_grasps.waitForResult(ros::Duration(30.0)))
    {
        ROS_INFO("Got grasps - performing pick");
        ROS_ASSERT(generate_grasps.getResult()->grasps.size() > 0);
        grasps_array.header = generate_grasps.getResult()->grasps[0].grasp_pose.header;
        ROS_INFO("Grasp poses frame_id: %s", grasps_array.header.frame_id.c_str());

        for (size_t i = 0; i < generate_grasps.getResult()->grasps.size(); i++)
        {
        	// for filtering poses
        	// if (generate_grasps.getResult()->grasps[i].grasp_pose.pose.position.z == p.pose.position.z)
        		grasps_array.poses.push_back(generate_grasps.getResult()->grasps[i].grasp_pose.pose);
        }

        ROS_INFO("Publishing grasps array containing %d grasps to: %s", grasps_array.poses.size(), grasps_topic.c_str());
        pub_grasps.publish(grasps_array);
    }


	ros::spin();















    return 0;
}


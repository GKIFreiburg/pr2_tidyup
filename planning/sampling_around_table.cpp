#include <ros/ros.h>

#include <moveit/planning_scene_monitor/planning_scene_monitor.h>
#include <moveit/planning_scene_interface/planning_scene_interface.h>
#include <shape_tools/solid_primitive_dims.h>
#include <geometry_msgs/PoseArray.h>
#include <tf/transform_listener.h>
#include <tf_conversions/tf_eigen.h>
#include <eigen_conversions/eigen_msg.h>





// given the table pose is inside the mesh, else receiving wrong primitives
moveit_msgs::CollisionObject extractTableDimensions(const moveit_msgs::CollisionObject& table)
{
	tf::TransformListener tf_;

	shape_msgs::SolidPrimitive table_primitive;
	table_primitive.type = shape_msgs::SolidPrimitive::BOX;

	ROS_ASSERT(table.meshes.size() != 0);
	shape_msgs::Mesh table_mesh = table.meshes[0];
	double min_x = HUGE_VAL;
	double min_y = HUGE_VAL;
	double min_z = HUGE_VAL;
	double max_x = -HUGE_VAL;
	double max_y = -HUGE_VAL;
	double max_z = -HUGE_VAL;

	Eigen::Affine3d a;
	tf::poseMsgToEigen(table.mesh_poses[0], a);

	geometry_msgs::Point vertice;
	for (size_t i = 0; i < table_mesh.vertices.size(); i++)
	{
		vertice = table_mesh.vertices[i];

		Eigen::Vector3d vertex;
		tf::pointMsgToEigen(vertice, vertex);
		Eigen::Affine3d v = a;
		v.translate(vertex);

		geometry_msgs::PoseStamped pose_stamped;
	    geometry_msgs::Pose pose;
		tf::poseEigenToMsg(v, pose);
		pose_stamped.header = table.header;
		pose_stamped.pose = pose;

	    geometry_msgs::PoseStamped pose_transformed;

	    try {
	        tf_.waitForTransform("/map", table.header.frame_id, table.header.stamp,
	                ros::Duration(0.5));
	        tf_.transformPose("/map", pose_stamped, pose_transformed);
	    } catch (tf::TransformException& ex) {
	        ROS_ERROR("%s", ex.what());
	        moveit_msgs::CollisionObject t;
	        return t;
	    }

		if (pose_transformed.pose.position.x < min_x)
		{
			min_x = pose_transformed.pose.position.x;
		}
		if (pose_transformed.pose.position.x > max_x)
		{
			max_x = pose_transformed.pose.position.x;
		}
		if (pose_transformed.pose.position.y < min_y)
		{
			min_y = pose_transformed.pose.position.y;
		}
		if (pose_transformed.pose.position.y > max_y)
		{
			max_y = pose_transformed.pose.position.y;
		}
		if (pose_transformed.pose.position.z < min_z)
			min_z = pose_transformed.pose.position.z;
		if (pose_transformed.pose.position.z > max_z)
			max_z = pose_transformed.pose.position.z;
	}


	table_primitive.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	table_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = max_x - min_x;
	table_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = max_y - min_y;
	table_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = max_z - min_z;

	geometry_msgs::Pose new_table_pose;
	new_table_pose.position.x = min_x + table_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] / 2;
	new_table_pose.position.y = min_y + table_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] / 2;
	new_table_pose.position.z = table.mesh_poses[0].position.z;
//	new_table_pose.orientation.w = 1.0;
    tf::Quaternion q = tf::createQuaternionFromYaw(0.79/2);
    geometry_msgs::Quaternion orientation;
    tf::quaternionTFToMsg(q, orientation);
    new_table_pose.orientation = orientation;





	// Create collision object for each table
	moveit_msgs::CollisionObject co;
	co.id = table.id + "_extracted";
	co.header = table.header;
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0] = table_primitive;
	co.primitive_poses.push_back(new_table_pose);

	ROS_ASSERT(co.primitives.size() == 1);
	ROS_ASSERT(co.primitives[0].dimensions.size() == 3);
	ROS_INFO_STREAM(__func__ << ": " << co);

	return co;
}

double dist(const geometry_msgs::Point& p1, const geometry_msgs::Point& p2)
{
	return sqrt( (p1.x - p2.x)*(p1.x - p2.x) + (p1.y - p2.y)*(p1.y - p2.y) );
}


// given the table pose is inside the mesh, else receiving wrong primitives
moveit_msgs::CollisionObject extractTableDimensionsGeneral(const moveit_msgs::CollisionObject& table,
		ros::Publisher* debug = NULL)
{
	tf::TransformListener tf_;

	shape_msgs::SolidPrimitive table_primitive;
	table_primitive.type = shape_msgs::SolidPrimitive::BOX;

	ROS_ASSERT(table.meshes.size() != 0);
	shape_msgs::Mesh table_mesh = table.meshes[0];
	double min_x = HUGE_VAL;
	double min_y = HUGE_VAL;
	double min_z = HUGE_VAL;
	double max_x = -HUGE_VAL;
	double max_y = -HUGE_VAL;
	double max_z = -HUGE_VAL;

	geometry_msgs::Point p_min_x, p_max_x, p_min_y, p_max_y;


	Eigen::Affine3d a;
	tf::poseMsgToEigen(table.mesh_poses[0], a);
	geometry_msgs::PoseStamped pose_transformed;

	double table_height = table_mesh.vertices[0].z;

	geometry_msgs::Point vertice;
	for (size_t i = 0; i < table_mesh.vertices.size(); i++)
	{
		vertice = table_mesh.vertices[i];
		if (vertice.z != table_height)
			continue;

		Eigen::Vector3d vertex;
		tf::pointMsgToEigen(vertice, vertex);
		Eigen::Affine3d v = a;
		v.translate(vertex);

		geometry_msgs::PoseStamped pose_stamped;
	    geometry_msgs::Pose pose;
		tf::poseEigenToMsg(v, pose);
		pose_stamped.header = table.header;
		pose_stamped.pose = pose;


	    try {
	        tf_.waitForTransform("/map", table.header.frame_id, table.header.stamp,
	                ros::Duration(0.5));
	        tf_.transformPose("/map", pose_stamped, pose_transformed);
	    } catch (tf::TransformException& ex) {
	        ROS_ERROR("%s", ex.what());
	        moveit_msgs::CollisionObject t;
	        return t;
	    }

		if (pose_transformed.pose.position.x < min_x)
		{
			min_x = pose_transformed.pose.position.x;
			p_min_x = pose_transformed.pose.position;
		}
		if (pose_transformed.pose.position.x > max_x)
		{
			max_x = pose_transformed.pose.position.x;
			p_max_x = pose_transformed.pose.position;
		}
		if (pose_transformed.pose.position.y < min_y)
		{
			min_y = pose_transformed.pose.position.y;
			p_min_y = pose_transformed.pose.position;
		}
		if (pose_transformed.pose.position.y > max_y)
		{
			max_y = pose_transformed.pose.position.y;
			p_max_y = pose_transformed.pose.position;
		}
		if (pose_transformed.pose.position.z < min_z)
			min_z = pose_transformed.pose.position.z;
		if (pose_transformed.pose.position.z > max_z)
			max_z = pose_transformed.pose.position.z;
	}

	ROS_WARN_STREAM("p_min_x" << p_min_x);
	ROS_WARN_STREAM("p_min_y" << p_min_y);
	ROS_WARN_STREAM("p_max_x" << p_max_x);
	ROS_WARN_STREAM("p_max_y" << p_max_y);


	double width, height;
	width  = dist(p_max_y, p_min_x);
	height = dist(p_max_y, p_max_x);

	ROS_ERROR("width = %lf, height = %lf", width, height);

	// compute center
	geometry_msgs::PoseStamped center;
	center = pose_transformed;
	center.pose.position.x = p_max_x.x - (p_max_x.x - p_min_x.x) / 2;
	center.pose.position.y = p_max_y.y - (p_max_y.y - p_min_y.y) / 2;


	//
	geometry_msgs::Point tmp;
	tmp.x = p_max_x.x - fabs(p_max_x.x - p_max_y.x) / 2;
	tmp.y = p_max_y.y - fabs(p_max_y.y - p_max_x.y) / 2;

	double dist_x, dist_y;
	dist_x = tmp.x - center.pose.position.x;
	dist_y = tmp.y - center.pose.position.y;

	double radians = atan2(dist_y, dist_x);
	tf::Quaternion q = tf::createQuaternionFromYaw(radians);
	geometry_msgs::Quaternion orientation;
	tf::quaternionTFToMsg(q, orientation);
	center.pose.orientation = orientation;

	if (debug != NULL)
		debug->publish(center);

	table_primitive.dimensions.resize(shape_tools::SolidPrimitiveDimCount<shape_msgs::SolidPrimitive::BOX>::value);
	table_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_X] = height;
	table_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Y] = width;
	table_primitive.dimensions[shape_msgs::SolidPrimitive::BOX_Z] = max_z - min_z;


	// Create collision object for each table
	moveit_msgs::CollisionObject co;
	co.id = table.id + "_extracted";
	co.header = table.header;
	co.primitives.resize(1);
	co.primitives[0].type = shape_msgs::SolidPrimitive::BOX;
	co.primitives[0] = table_primitive;
	co.primitive_poses.push_back(center.pose);

	ROS_ASSERT(co.primitives.size() == 1);
	ROS_ASSERT(co.primitives[0].dimensions.size() == 3);
	ROS_INFO_STREAM(__func__ << ": " << co);

	return co;

}




geometry_msgs::PoseStamped createSample(const moveit_msgs::CollisionObject& table, double x, double y)
{
	geometry_msgs::PoseStamped p;
	p.header = table.header;
	p.pose.position.x = x;
	p.pose.position.y = y;
	// p.pose.position.z = table.primitive_poses[0].position.z;
	p.pose.position.z = 0.0;

	double dist_x, dist_y;
	ROS_ASSERT(table.primitive_poses.size() == 1);
	dist_x = table.primitive_poses[0].position.x - x;
	dist_y = table.primitive_poses[0].position.y - y;

	double radians = atan2(dist_y, dist_x);
	tf::Quaternion q = tf::createQuaternionFromYaw(radians);
	geometry_msgs::Quaternion orientation;
	tf::quaternionTFToMsg(q, orientation);
	p.pose.orientation = orientation;
//	p.pose.orientation.w = 1;

	return p;
}

std::vector<geometry_msgs::PoseStamped> sampleBetweenPoints(const moveit_msgs::CollisionObject& table,
		int number_samples,
		geometry_msgs::PoseStamped start_point, geometry_msgs::PoseStamped end_point)
{
	std::vector<geometry_msgs::PoseStamped> result;

	double slope_x, slope_y;
	if (start_point.pose.position.x == end_point.pose.position.x)
		slope_x = 0;
	else if (start_point.pose.position.x < end_point.pose.position.x)
		slope_x = 1;
	else
		slope_x = -1;

	if (start_point.pose.position.y == end_point.pose.position.y)
		slope_y = 0;
	else if (start_point.pose.position.y < end_point.pose.position.y)
		slope_y = 1;
	else
		slope_y = -1;

	double seg_x, seg_y;
	seg_x = fabs(start_point.pose.position.x - end_point.pose.position.x) / number_samples;
	seg_y = fabs(start_point.pose.position.y - end_point.pose.position.y) / number_samples;

	for (int i = 0; i < number_samples; i++)
	{
		geometry_msgs::PoseStamped p = createSample(table,
				start_point.pose.position.x + slope_x * i * seg_x,
				start_point.pose.position.y + slope_y * i * seg_y);
		result.push_back(p);
	}

	return result;






















//	double start_x, start_y;
//	if (start_point.pose.position.x > end_point.pose.position.x)
//		start_x = end_point.pose.position.x;
//	else
//		start_x = start_point.pose.position.x;
//
//	if (start_point.pose.position.y > end_point.pose.position.y)
//		start_y = end_point.pose.position.y;
//	else
//		start_y = start_point.pose.position.y;
//
//	double seg_x, seg_y;
//
//	if (start_point.pose.position.x == end_point.pose.position.y)
//		seg_x = 0;
//	else
//		seg_x = sqrt((start_point.pose.position.x - end_point.pose.position.x)*
//			(start_point.pose.position.x - end_point.pose.position.x)) / number_samples;
//
//	if (start_point.pose.position.y == end_point.pose.position.y)
//		seg_y = 0;
//	else
//		seg_y = sqrt((start_point.pose.position.y - end_point.pose.position.y)*
//			(start_point.pose.position.y - end_point.pose.position.y)) / number_samples;
//
//
//	ROS_WARN("start_x: %lf, end_x: %lf, seg_x: %lf", start_x, start_y, seg_x);
//
//	for (int i = 0; i < number_samples; i++)
//	{
//		geometry_msgs::PoseStamped p = createSample(table, start_x + i * seg_x, start_y + i * seg_y);
//		result.push_back(p);
//	}
//
//	return result;
}


std::vector<geometry_msgs::PoseStamped> computeSamples(const moveit_msgs::CollisionObject& table,
		int number_samples, double distance_table, ros::Publisher* pubPoints = NULL)
{
	ROS_ASSERT(table.primitive_poses.size() != 0);
	double table_x = table.primitive_poses[0].position.x;
	double table_y = table.primitive_poses[0].position.y;
	std::vector<geometry_msgs::PoseStamped> result;


	ROS_ASSERT(table.primitives.size() == 1);
	ROS_ASSERT(table.primitives[0].dimensions.size() == 3);
	ROS_ASSERT(table.primitives[0].type == shape_msgs::SolidPrimitive::BOX);
	double table_width = table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_X];
	double table_height = table.primitives[0].dimensions[shape_msgs::SolidPrimitive::BOX_Y];



	geometry_msgs::PoseStamped p1, p2, p3, p4;
	p1.header = table.header;
	p2.header = table.header;
	p3.header = table.header;
	p4.header = table.header;

	Eigen::Affine3d table_pose;
	tf::poseMsgToEigen(table.primitive_poses[0], table_pose);
	Eigen::Vector3d translation;

	translation = Eigen::Vector3d(table_width / 2 + distance_table, - table_height / 2 - distance_table, 0 );
	Eigen::Affine3d p1_eigen = table_pose;
	p1_eigen.translate(translation);
	tf::poseEigenToMsg(p1_eigen, p1.pose);

	translation = Eigen::Vector3d(table_width / 2 + distance_table, table_height / 2 + distance_table, 0 );
	Eigen::Affine3d p2_eigen = table_pose;
	p2_eigen.translate(translation);
	tf::poseEigenToMsg(p2_eigen, p2.pose);

	translation = Eigen::Vector3d(- table_width / 2 - distance_table, table_height / 2 + distance_table, 0 );
	Eigen::Affine3d p3_eigen = table_pose;
	p3_eigen.translate(translation);
	tf::poseEigenToMsg(p3_eigen, p3.pose);

	translation = Eigen::Vector3d(- table_width / 2 - distance_table, - table_height / 2 - distance_table, 0 );
	Eigen::Affine3d p4_eigen = table_pose;
	p4_eigen.translate(translation);
	tf::poseEigenToMsg(p4_eigen, p4.pose);









//	geometry_msgs::PoseStamped p1, p2, p3, p4;
//	p1.header = table.header;
//	p1.pose.position.x = table_x + table_height / 2 + distance_table;
//	p1.pose.position.y = table_y - table_width / 2 - distance_table;
//
//	p2.header = table.header;
//	p2.pose.position.x = table_x +  table_height / 2 + distance_table;
//	p2.pose.position.y = table_y + table_width / 2 + distance_table;
//
//	p3.header = table.header;
//	p3.pose.position.x = table_x -  table_height / 2 - distance_table;
//	p3.pose.position.y = table_y + table_width / 2 + distance_table;
//
//	p4.header = table.header;
//	p4.pose.position.x = table_x -  table_height / 2 - distance_table;
//	p4.pose.position.y = table_y - table_width / 2 - distance_table;

	if (pubPoints != NULL)
	{
		geometry_msgs::PoseArray points;
		points.header = table.header;
		p1.pose.position.z = 0;
		p2.pose.position.z = 0;
		p3.pose.position.z = 0;
		p4.pose.position.z = 0;
		points.poses.push_back(p1.pose);
		points.poses.push_back(p2.pose);
		points.poses.push_back(p3.pose);
		points.poses.push_back(p4.pose);
		pubPoints->publish(points);
	}

	std::vector<geometry_msgs::PoseStamped> p1p2 = sampleBetweenPoints(table, number_samples, p1, p2);
	result.insert(result.end(), p1p2.begin(), p1p2.end());

	std::vector<geometry_msgs::PoseStamped> p2p3 = sampleBetweenPoints(table, number_samples, p2, p3);
	result.insert(result.end(), p2p3.begin(), p2p3.end());

	std::vector<geometry_msgs::PoseStamped> p3p4 = sampleBetweenPoints(table, number_samples, p3, p4);
	result.insert(result.end(), p3p4.begin(), p3p4.end());

	std::vector<geometry_msgs::PoseStamped> p4p1 = sampleBetweenPoints(table, number_samples, p4, p1);
	result.insert(result.end(), p4p1.begin(), p4p1.end());

	return result;
}

void publishPoses(ros::Publisher pubSamples, const std::vector<geometry_msgs::PoseStamped>& samples)
{
	geometry_msgs::PoseArray poses;
	if (samples.size() == 0)
		return;
	poses.header = samples[0].header;
	for (size_t i = 0; i < samples.size(); i++)
	{
		poses.poses.push_back(samples[i].pose);
	}

	pubSamples.publish(poses);
}



int main(int argc, char **argv)
{
	ros::init (argc, argv, "samplingAroundTable");
	ros::AsyncSpinner spinner(1);
	spinner.start();
	ros::NodeHandle nh;

	ros::Publisher pubObjects = nh.advertise<moveit_msgs::CollisionObject>("/collision_object", 1, true);

	ros::Publisher pubPoses = nh.advertise<geometry_msgs::PoseArray>("poses", 1, true);
	ros::Publisher pubSamples = nh.advertise<geometry_msgs::PoseArray>("samples", 1, true);
	ros::Publisher* pubPoints = new ros::Publisher(nh.advertise<geometry_msgs::PoseArray>("points", 1, true));

	ros::Publisher* debug = new ros::Publisher(nh.advertise<geometry_msgs::PoseStamped>("debug", 1, true));

	geometry_msgs::PoseArray poses;
	poses.header.frame_id = "/map";

	moveit_msgs::CollisionObject test;

	// create a planningScene object
	planning_scene_monitor::PlanningSceneMonitorPtr psm(new planning_scene_monitor::PlanningSceneMonitor("robot_description"));
    psm->requestPlanningSceneState();
    planning_scene_monitor::LockedPlanningSceneRO locked_ps(psm);
    // Create a copy of planningScene
    planning_scene::PlanningScenePtr planning_scene = locked_ps->diff();

    moveit_msgs::PlanningScene ps_msg;
    planning_scene->getPlanningSceneMsg(ps_msg);

    // look for collision object table1
    moveit_msgs::CollisionObject table_object;
    std::string co_id = "table1";
    for (size_t i = 0; i < ps_msg.world.collision_objects.size(); i++)
    {
    	if (ps_msg.world.collision_objects[i].id == co_id)
    		table_object = ps_msg.world.collision_objects[i];
    }
    test = table_object;
    std::vector<geometry_msgs::PoseStamped> samples;

    ROS_ASSERT(table_object.id == co_id);
    poses.poses.push_back(table_object.mesh_poses[0]);
    moveit_msgs::CollisionObject co = extractTableDimensionsGeneral(table_object);
    poses.poses.push_back(co.primitive_poses[0]);
    samples = computeSamples(co, 20, 0.5, pubPoints);

    co_id = "table2";
    for (size_t i = 0; i < ps_msg.world.collision_objects.size(); i++)
    {
    	if (ps_msg.world.collision_objects[i].id == co_id)
    		table_object = ps_msg.world.collision_objects[i];
    }

    ROS_ASSERT(table_object.id == co_id);
    poses.poses.push_back(table_object.mesh_poses[0]);
    co = extractTableDimensionsGeneral(table_object);
    poses.poses.push_back(co.primitive_poses[0]);

    std::vector<geometry_msgs::PoseStamped> samples2 = computeSamples(co, 20, 0.5, pubPoints);
    samples.insert(samples.end(), samples2.begin(), samples2.end());



//    test.mesh_poses[0].position.x = -4;
//    test.mesh_poses[0].position.y = 4;
//    tf::Quaternion q1 = tf::createQuaternionFromYaw(-2.6);
//    geometry_msgs::Quaternion orientation1;
//    tf::quaternionTFToMsg(q1, orientation1);
//    test.mesh_poses[0].orientation = orientation1;
//    co = extractTableDimensionsGeneral(test);
//    poses.poses.push_back(co.primitive_poses[0]);
//    std::vector<geometry_msgs::PoseStamped> samples3 = computeSamples(co, 100, 0.5);
//    samples.insert(samples.end(), samples3.begin(), samples3.end());
//
//    test.mesh_poses[0].position.x = 4;
//    test.mesh_poses[0].position.y = -4;
//    tf::Quaternion q2 = tf::createQuaternionFromYaw(1.8);
//    geometry_msgs::Quaternion orientation2;
//    tf::quaternionTFToMsg(q2, orientation2);
//    test.mesh_poses[0].orientation = orientation2;
//    co = extractTableDimensionsGeneral(test);
//    poses.poses.push_back(co.primitive_poses[0]);
//    std::vector<geometry_msgs::PoseStamped> samples4 = computeSamples(co, 100, 0.5);
//    samples.insert(samples.end(), samples4.begin(), samples4.end());
////
//    test.mesh_poses[0].position.x = -4;
//    test.mesh_poses[0].position.y = -4;
//    tf::Quaternion q3 = tf::createQuaternionFromYaw(0);
//    geometry_msgs::Quaternion orientation3;
//    tf::quaternionTFToMsg(q3, orientation3);
//    test.mesh_poses[0].orientation = orientation3;
//    co = extractTableDimensionsGeneral(test);
//    poses.poses.push_back(co.primitive_poses[0]);
//    std::vector<geometry_msgs::PoseStamped> samples5 = computeSamples(co, 100, 0.5);
//    samples.insert(samples.end(), samples5.begin(), samples5.end());
//



//    test.mesh_poses[0].position.x = -6;
//    test.mesh_poses[0].position.y = -5;
//    tf::Quaternion q = tf::createQuaternionFromYaw(0);
//    geometry_msgs::Quaternion orientation;
//    tf::quaternionTFToMsg(q, orientation);
//    test.mesh_poses[0].orientation = orientation;
//    pubObjects.publish(test);
//
////    co = extractTableDimensions(test);
//    co = extractTableDimensionsGeneral(test, debug);
//    pubObjects.publish(co);
//    ROS_ERROR_STREAM(co);
//    poses.poses.push_back(co.primitive_poses[0]);
////    std::vector<geometry_msgs::PoseStamped> samples6 = computeSamples(co, 20, 0.5, pubPoints);
//    std::vector<geometry_msgs::PoseStamped> samples6 = computeSamples(co, 20, 0.5);
//    samples.insert(samples.end(), samples6.begin(), samples6.end());











    publishPoses(pubSamples, samples);

    pubPoses.publish(poses);

	ros::shutdown();
	return 0;
}



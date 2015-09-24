#include <ros/ros.h>
#include <geometry_msgs/Pose2D.h>
#include <geometry_msgs/Pose.h>
#include <tf/transform_datatypes.h>

int main(int argc, char **argv)
{
	ros::init(argc, argv, "move_group_interface_tutorial");
	ros::NodeHandle node_handle;

	if (argc != 4)
	{
		ROS_ERROR("Usage: rosrun planning convertPose2DToPose x y theta");
		exit(EXIT_FAILURE);
	}

	geometry_msgs::Pose2D in;
	in.x = atof(argv[1]);
	in.y = atof(argv[2]);
	in.theta = atof(argv[3]);


	geometry_msgs::Pose pose;
	tf::Quaternion q = tf::createQuaternionFromYaw(in.theta);
	tf::quaternionTFToMsg(q, pose.orientation);

	pose.position.x = in.x;
	pose.position.y = in.y;

	ROS_INFO("%lf %lf %lf %lf %lf %lf %lf",
			pose.position.x,
			pose.position.y,
			pose.position.z,
			pose.orientation.x,
			pose.orientation.y,
			pose.orientation.z,
			pose.orientation.w);

  ros::shutdown();
  return 0;
}

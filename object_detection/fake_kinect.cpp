// This program subscribes to turtle1/cmd_vel and
// republishes on turtle1/cmd_vel_reversed,
// with the signs inverted.
#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>

ros::Publisher *pubPtr;

void cameraInfoReceived(
  const sensor_msgs::CameraInfo& msgIn
) {
  ROS_INFO("Pushing MSG to TOPIC");
  pubPtr->publish(msgIn);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "fake_kinect");
  ros::NodeHandle nh;

  pubPtr = new ros::Publisher(
    nh.advertise<sensor_msgs::CameraInfo>(
      "fake_kinect/depth/camera_info",
      1000));

  ros::Subscriber sub = nh.subscribe(
    "head_mount_kinect/depth/camera_info", 1000,
    &cameraInfoReceived);

  ros::spin();

  delete pubPtr;
}

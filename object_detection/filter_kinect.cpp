#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

ros::Publisher *pubImgPtr;
int* counterPtr;

void cameraImageReceived(
  const sensor_msgs::Image& msgIn
) {
  if (msgIn.header.frame_id != "head_mount_kinect_rgb_optical_frame")
    return;

// if (msgIn.encoding != "mono8")
//    return;

  *counterPtr = *counterPtr + 1;

  ROS_INFO("Pushing MSG Nr. %d to TOPIC", *counterPtr);
  pubImgPtr->publish(msgIn);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter_kinect");
  ros::NodeHandle nh;

  pubImgPtr = new ros::Publisher(
    nh.advertise<sensor_msgs::Image>(
      "head_mount_kinect/depth/image_fixed",
      1000));

  ros::Subscriber sub = nh.subscribe(
    "head_mount_kinect/depth/image_raw", 1000,
    &cameraImageReceived);

  int counter = 0;
  counterPtr = &counter;

  ros::spin();

  delete pubImgPtr;
}

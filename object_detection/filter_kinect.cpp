#include <ros/ros.h>
#include <sensor_msgs/CameraInfo.h>
#include <sensor_msgs/Image.h>

ros::Publisher *pubImgPtr;
ros::Publisher *pubCameraInfoPtr;
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

void cameraInfoReceived(
  const sensor_msgs::CameraInfo& msgIn
) {
  if (msgIn.header.frame_id == "head_mount_kinect_rgb_optical_frame")
    return;

  pubCameraInfoPtr->publish(msgIn);
}

int main(int argc, char **argv) {
  ros::init(argc, argv, "filter_kinect");
  ros::NodeHandle nh;

  pubImgPtr = new ros::Publisher(
    nh.advertise<sensor_msgs::Image>(
      "filtered_kinect/depth/image_raw",
      1000));

  pubCameraInfoPtr = new ros::Publisher(
    nh.advertise<sensor_msgs::CameraInfo>(
      "filtered_kinect/depth/camera_info",
      1000));

  ros::Subscriber sub = nh.subscribe(
    "head_mount_kinect/depth/image_raw", 1000,
    &cameraImageReceived);

  ros::Subscriber subCameraInfo = nh.subscribe(
    "head_mount_kinect/depth/camera_info", 1000,
    &cameraInfoReceived);

  int counter = 0;
  counterPtr = &counter;

  ros::spin();

  delete pubImgPtr;
}

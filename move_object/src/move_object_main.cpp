#include <ros/ros.h>

#include <move_object/move_object.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "move_object_main");
  ros::AsyncSpinner spinner(1);
  spinner.start();

  MoveObject move_object;
  // create a small cube
  //MetaBlock block = move_object.createBlock("block", 0.55, 0.2, 0.6);

  // create a rectangle which should be an approximation of a can.
  MetaBlock block = move_object.createCan("block", 0.55, 0.2, 0.6);

  move_object.pick(block.pose, "block");

  return 0;

}

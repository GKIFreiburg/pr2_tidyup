#include <ros/ros.h>
#include <tf/transform_listener.h>
#include "tidyupTeachingGui.h"
#include <string>
#include <QApplication>

int main(int argc, char** argv)
{
   ros::init(argc, argv, "tidyup_teaching_gui");

   if(argc != 3 && argc != 1) {
       ROS_FATAL("Usage: %s [fixed_frame target_frame]", argv[0]);
       return 1;
   }

   std::string fixed_frame = "/map";
   std::string target_frame = "/base_link";
   if(argc == 3) {
       fixed_frame = argv[1];
       target_frame = argv[2];
   }

   ros::NodeHandle nh;

   QApplication app(argc, argv);
   TidyupTeachingGui gui(fixed_frame, target_frame);
   gui.show();

   ros::WallRate rate(10);
   while(ros::ok() && gui.isVisible()) {
      ros::spinOnce();

      app.processEvents();

      rate.sleep();
   }

   return 0;
}

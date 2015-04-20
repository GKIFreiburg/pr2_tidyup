#include <ros/ros.h>
#include <QApplication>
#include "grasp_provider_gui.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "grasp_provider_gui");
    ros::NodeHandle nh;

    QApplication app(argc, argv);
    grasp_provider::GraspProviderGui gui;
    gui.show();

    ros::WallRate loop(20.0);
    while(ros::ok() && gui.isVisible()) {
        ros::spinOnce();

        app.processEvents();

        loop.sleep();
    }

    return 0;
}

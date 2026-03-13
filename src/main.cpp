#include <ros/ros.h>
#include <QApplication>

#include "robot_monitor/main_window.h"

int main(int argc, char** argv)
{
    ros::init(argc, argv, "robot_monitor");
    ros::NodeHandle nh;

    QApplication app(argc, argv);

    robot_monitor::MainWindow window(nh);
    window.show();

    return app.exec();
}
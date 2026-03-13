#ifndef ROBOT_MONITOR_ROS_INTERFACE_H
#define ROBOT_MONITOR_ROS_INTERFACE_H

#include <mutex>
#include <string>

#include <ros/ros.h>
#include <nav_msgs/Odometry.h>

namespace robot_monitor
{

struct OdomData
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
    bool received = false;
};

class RosInterface
{
public:
    RosInterface();
    ~RosInterface();

    bool init(ros::NodeHandle& nh, const std::string& odom_topic = "/robot/robotnik_base_control/odom");
    OdomData getOdomData() const;

private:
    void odomCallback(const nav_msgs::Odometry::ConstPtr& msg);

private:
    ros::Subscriber odom_sub_;

    mutable std::mutex odom_mutex_;
    OdomData odom_data_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_ROS_INTERFACE_H
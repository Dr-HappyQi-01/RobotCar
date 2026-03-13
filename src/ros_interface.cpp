#include "robot_monitor/ros_interface.h"

#include <tf/tf.h>

namespace robot_monitor
{

RosInterface::RosInterface()
{
}

RosInterface::~RosInterface()
{
}

bool RosInterface::init(ros::NodeHandle& nh, const std::string& odom_topic)
{
    odom_sub_ = nh.subscribe(odom_topic, 10, &RosInterface::odomCallback, this);
    return true;
}

OdomData RosInterface::getOdomData() const
{
    std::lock_guard<std::mutex> lock(odom_mutex_);
    return odom_data_;
}

void RosInterface::odomCallback(const nav_msgs::Odometry::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(odom_mutex_);

    odom_data_.x = msg->pose.pose.position.x;
    odom_data_.y = msg->pose.pose.position.y;
    odom_data_.linear_velocity = msg->twist.twist.linear.x;
    odom_data_.angular_velocity = msg->twist.twist.angular.z;

    tf::Quaternion q;
    tf::quaternionMsgToTF(msg->pose.pose.orientation, q);

    double roll = 0.0;
    double pitch = 0.0;
    double yaw = 0.0;
    tf::Matrix3x3(q).getRPY(roll, pitch, yaw);

    odom_data_.yaw = yaw;
    odom_data_.received = true;
}

}  // namespace robot_monitor
#include "robot_monitor/ros_interface.h"
#include "robot_monitor/map_types.h"

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
    map_sub_ = nh.subscribe("/map", 1, &RosInterface::mapCallback, this);
    return true;
}

GridMapData RosInterface::getMapData() const
{
    std::lock_guard<std::mutex> lock(map_mutex_);
    return map_data_;
}

void RosInterface::mapCallback(const nav_msgs::OccupancyGrid::ConstPtr& msg)
{
    std::lock_guard<std::mutex> lock(map_mutex_);

    map_data_.width = static_cast<int>(msg->info.width);
    map_data_.height = static_cast<int>(msg->info.height);
    map_data_.resolution = msg->info.resolution;
    map_data_.origin_x = msg->info.origin.position.x;
    map_data_.origin_y = msg->info.origin.position.y;
    map_data_.data = msg->data;
    map_data_.valid = (!map_data_.data.empty() &&
                       map_data_.width > 0 &&
                       map_data_.height > 0);
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
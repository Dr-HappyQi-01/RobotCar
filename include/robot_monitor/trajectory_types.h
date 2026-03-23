#ifndef ROBOT_MONITOR_TRAJECTORY_TYPES_H
#define ROBOT_MONITOR_TRAJECTORY_TYPES_H

#include <string>
#include <vector>

namespace robot_monitor
{

struct TrajectoryPoint
{
    double x = 0.0;
    double y = 0.0;
    double yaw = 0.0;
    double linear_velocity = 0.0;
    double angular_velocity = 0.0;
    double timestamp = 0.0;
};

struct TrajectoryRecord
{
    int id = -1;
    std::string name;
    std::string method_name;
    int episode_index = 0;

    double start_time = 0.0;
    double end_time = 0.0;

    std::vector<TrajectoryPoint> points;
};

struct EpisodeRewardPoint
{
    int episode = 0;
    double reward = 0.0;
    double timestamp = 0.0;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_TRAJECTORY_TYPES_H
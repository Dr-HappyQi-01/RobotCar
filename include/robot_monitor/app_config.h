#ifndef ROBOT_MONITOR_APP_CONFIG_H
#define ROBOT_MONITOR_APP_CONFIG_H

#include <string>

namespace robot_monitor
{

struct AppConfig
{
    std::string odom_topic;
    std::string map_topic;
    std::string camera_topic;
    std::string episode_event_topic;

    std::string slam_start_command;
    std::string slam_save_map_command;
    std::string slam_stop_command;

    std::string default_map_dir;
};

class AppConfigLoader
{
public:
    AppConfigLoader();
    ~AppConfigLoader();

    bool loadFromFile(const std::string& file_path,
                      AppConfig& config,
                      std::string& error_message);
};

}  // namespace robot_monitor

#endif
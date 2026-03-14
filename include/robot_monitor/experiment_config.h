#ifndef ROBOT_MONITOR_EXPERIMENT_CONFIG_H
#define ROBOT_MONITOR_EXPERIMENT_CONFIG_H

#include <string>

namespace robot_monitor
{

struct ExperimentConfig
{
    std::string method_name;
};

class ExperimentConfigLoader
{
public:
    ExperimentConfigLoader();
    ~ExperimentConfigLoader();

    bool loadFromFile(const std::string& file_path, ExperimentConfig& config, std::string& error_message);
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_EXPERIMENT_CONFIG_H
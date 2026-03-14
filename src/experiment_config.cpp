#include "robot_monitor/experiment_config.h"

#include <fstream>
#include <sstream>

#include <ros/ros.h>
#include <jsoncpp/json/json.h>

namespace robot_monitor
{

ExperimentConfigLoader::ExperimentConfigLoader()
{
}

ExperimentConfigLoader::~ExperimentConfigLoader()
{
}

bool ExperimentConfigLoader::loadFromFile(const std::string& file_path,
                                          ExperimentConfig& config,
                                          std::string& error_message)
{
    std::ifstream ifs(file_path.c_str());
    if (!ifs.is_open())
    {
        error_message = "Failed to open config file: " + file_path;
        return false;
    }

    std::stringstream buffer;
    buffer << ifs.rdbuf();
    const std::string content = buffer.str();

    Json::CharReaderBuilder reader_builder;
    Json::Value root;
    std::string parse_errors;
    std::istringstream iss(content);

    if (!Json::parseFromStream(reader_builder, iss, &root, &parse_errors))
    {
        error_message = "Failed to parse JSON: " + parse_errors;
        return false;
    }

    if (!root.isMember("method_name"))
    {
        error_message = "Missing required field: method_name";
        return false;
    }

    if (!root["method_name"].isString())
    {
        error_message = "Field method_name must be a string";
        return false;
    }

    config.method_name = root["method_name"].asString();

    if (config.method_name.empty())
    {
        error_message = "Field method_name is empty";
        return false;
    }

    return true;
}

}  // namespace robot_monitor
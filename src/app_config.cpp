#include "robot_monitor/app_config.h"

#include <fstream>
#include <sstream>

#include <jsoncpp/json/json.h>

namespace robot_monitor
{

AppConfigLoader::AppConfigLoader()
{
}

AppConfigLoader::~AppConfigLoader()
{
}

bool AppConfigLoader::loadFromFile(const std::string& file_path,
                                   AppConfig& config,
                                   std::string& error_message)
{
    std::ifstream ifs(file_path.c_str());
    if (!ifs.is_open())
    {
        error_message = "Failed to open app config file: " + file_path;
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
        error_message = "Failed to parse app config JSON: " + parse_errors;
        return false;
    }

    if (!root.isMember("topics") || !root["topics"].isObject())
    {
        error_message = "Missing or invalid 'topics' object";
        return false;
    }

    if (!root.isMember("slam") || !root["slam"].isObject())
    {
        error_message = "Missing or invalid 'slam' object";
        return false;
    }

    if (!root.isMember("paths") || !root["paths"].isObject())
    {
        error_message = "Missing or invalid 'paths' object";
        return false;
    }

    const Json::Value& topics = root["topics"];
    const Json::Value& slam = root["slam"];
    const Json::Value& paths = root["paths"];

    if (!topics.isMember("odom") || !topics["odom"].isString())
    {
        error_message = "Missing or invalid topics.odom";
        return false;
    }
    if (!topics.isMember("map") || !topics["map"].isString())
    {
        error_message = "Missing or invalid topics.map";
        return false;
    }
    if (!topics.isMember("camera") || !topics["camera"].isString())
    {
        error_message = "Missing or invalid topics.camera";
        return false;
    }
    if (!topics.isMember("episode_event") || !topics["episode_event"].isString())
    {
        error_message = "Missing or invalid topics.episode_event";
        return false;
    }

    if (!slam.isMember("start_command") || !slam["start_command"].isString())
    {
        error_message = "Missing or invalid slam.start_command";
        return false;
    }
    if (!slam.isMember("save_map_command") || !slam["save_map_command"].isString())
    {
        error_message = "Missing or invalid slam.save_map_command";
        return false;
    }
    if (!slam.isMember("stop_command") || !slam["stop_command"].isString())
    {
        error_message = "Missing or invalid slam.stop_command";
        return false;
    }

    if (!paths.isMember("default_map_dir") || !paths["default_map_dir"].isString())
    {
        error_message = "Missing or invalid paths.default_map_dir";
        return false;
    }

    config.odom_topic = topics["odom"].asString();
    config.map_topic = topics["map"].asString();
    config.camera_topic = topics["camera"].asString();
    config.episode_event_topic = topics["episode_event"].asString();

    config.slam_start_command = slam["start_command"].asString();
    config.slam_save_map_command = slam["save_map_command"].asString();
    config.slam_stop_command = slam["stop_command"].asString();

    config.default_map_dir = paths["default_map_dir"].asString();

    return true;
}

}  // namespace robot_monitor
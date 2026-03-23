#ifndef ROBOT_MONITOR_MAP_FILE_LOADER_H
#define ROBOT_MONITOR_MAP_FILE_LOADER_H

#include <string>

#include "robot_monitor/map_types.h"

namespace robot_monitor
{

class MapFileLoader
{
public:
    MapFileLoader();
    ~MapFileLoader();

    bool loadRosMapFromYaml(const std::string& yaml_path,
                            GridMapData& map_data,
                            std::string& error_message);

    bool loadImageMapFromPng(const std::string& image_path,
                             GridMapData& map_data,
                             std::string& error_message);

private:
    std::string resolveImagePath(const std::string& yaml_path,
                                 const std::string& image_field) const;
};

}  // namespace robot_monitor

#endif
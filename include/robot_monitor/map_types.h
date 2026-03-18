#ifndef ROBOT_MONITOR_MAP_TYPES_H
#define ROBOT_MONITOR_MAP_TYPES_H

#include <cstdint>
#include <vector>

namespace robot_monitor
{

struct GridMapData
{
    int width = 0;
    int height = 0;

    double resolution = 0.0;
    double origin_x = 0.0;
    double origin_y = 0.0;

    std::vector<int8_t> data;
    bool valid = false;
};

}  // namespace robot_monitor

#endif
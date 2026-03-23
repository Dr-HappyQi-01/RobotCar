#ifndef ROBOT_MONITOR_MAP_TYPES_H
#define ROBOT_MONITOR_MAP_TYPES_H

#include <cstdint>
#include <vector>
#include <QImage>
#include <QPointF>

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

struct ImageOverlayData
{
    QImage image;

    QPointF world_top_left;
    QPointF world_top_right;
    QPointF world_bottom_right;
    QPointF world_bottom_left;

    bool valid = false;
};

}  // namespace robot_monitor

#endif
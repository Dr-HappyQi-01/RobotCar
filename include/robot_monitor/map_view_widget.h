#ifndef ROBOT_MONITOR_MAP_VIEW_WIDGET_H
#define ROBOT_MONITOR_MAP_VIEW_WIDGET_H

#include <QPoint>
#include <QPointF>
#include <QWidget>
#include <vector>
#include <QImage>
#include "robot_monitor/map_types.h"

#include "robot_monitor/trajectory_types.h"

namespace robot_monitor
{

class MapViewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MapViewWidget(QWidget* parent = nullptr);
    ~MapViewWidget();

    void setRobotPose(double x, double y, double yaw, bool valid);
    void clearTrajectory();
    void setSelectedTrajectory(const TrajectoryRecord& record);
    void clearSelectedTrajectory();
    void setMapData(const GridMapData& map_data);
    void clearMapData();
    

protected:
    void paintEvent(QPaintEvent* event) override;
    void wheelEvent(QWheelEvent* event) override;
    void mousePressEvent(QMouseEvent* event) override;
    void mouseMoveEvent(QMouseEvent* event) override;
    void mouseReleaseEvent(QMouseEvent* event) override;

private:
    void drawBackground(QPainter& painter);
    void drawGrid(QPainter& painter);
    void drawAxes(QPainter& painter);
    void drawTrajectory(QPainter& painter);
    void drawRobot(QPainter& painter);
    void drawOverlayInfo(QPainter& painter);
    void drawSelectedTrajectory(QPainter& painter);
    void drawStartEndMarkers(QPainter& painter, const std::vector<QPointF>& points);

    QPointF worldToScreen(double wx, double wy) const;
    void appendTrajectoryPoint(double x, double y);
    void drawMap(QPainter& painter);
    void rebuildMapImage();



private:
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    bool pose_valid_;



    double pixels_per_meter_;
    double view_scale_;
    QPointF view_offset_;

    std::vector<QPointF> trajectory_points_;
    double min_point_distance_;

    bool is_panning_;
    QPoint last_mouse_pos_;

    TrajectoryRecord selected_trajectory_record_;
    bool has_selected_trajectory_;

    GridMapData map_data_;
    QImage map_image_;
    bool has_map_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_MAP_VIEW_WIDGET_H
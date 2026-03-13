#ifndef ROBOT_MONITOR_MAP_VIEW_WIDGET_H
#define ROBOT_MONITOR_MAP_VIEW_WIDGET_H

#include <QWidget>

namespace robot_monitor
{

class MapViewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MapViewWidget(QWidget* parent = nullptr);
    ~MapViewWidget();

    void setRobotPose(double x, double y, double yaw, bool valid);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawBackground(QPainter& painter);
    void drawGrid(QPainter& painter);
    void drawAxes(QPainter& painter);
    void drawRobot(QPainter& painter);

    QPointF worldToScreen(double wx, double wy) const;

private:
    double robot_x_;
    double robot_y_;
    double robot_yaw_;
    bool pose_valid_;

    double pixels_per_meter_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_MAP_VIEW_WIDGET_H
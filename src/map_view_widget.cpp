#include "robot_monitor/map_view_widget.h"

#include <QPainter>
#include <QPaintEvent>
#include <QPolygonF>
#include <QtMath>

namespace robot_monitor
{

MapViewWidget::MapViewWidget(QWidget* parent)
    : QWidget(parent),
      robot_x_(0.0),
      robot_y_(0.0),
      robot_yaw_(0.0),
      pose_valid_(false),
      pixels_per_meter_(50.0)
{
    setMinimumSize(800, 500);
    setAutoFillBackground(true);
}

MapViewWidget::~MapViewWidget()
{
}

void MapViewWidget::setRobotPose(double x, double y, double yaw, bool valid)
{
    robot_x_ = x;
    robot_y_ = y;
    robot_yaw_ = yaw;
    pose_valid_ = valid;
    update();
}

void MapViewWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    drawBackground(painter);
    drawGrid(painter);
    drawAxes(painter);
    drawRobot(painter);
}

void MapViewWidget::drawBackground(QPainter& painter)
{
    painter.fillRect(rect(), QColor(43, 49, 60));
}

void MapViewWidget::drawGrid(QPainter& painter)
{
    painter.save();

    QPen grid_pen(QColor(70, 82, 100));
    grid_pen.setWidth(1);
    painter.setPen(grid_pen);

    const int w = width();
    const int h = height();
    const int cx = w / 2;
    const int cy = h / 2;
    const int spacing = static_cast<int>(pixels_per_meter_);

    for (int x = cx; x < w; x += spacing)
        painter.drawLine(x, 0, x, h);
    for (int x = cx; x >= 0; x -= spacing)
        painter.drawLine(x, 0, x, h);

    for (int y = cy; y < h; y += spacing)
        painter.drawLine(0, y, w, y);
    for (int y = cy; y >= 0; y -= spacing)
        painter.drawLine(0, y, w, y);

    painter.restore();
}

void MapViewWidget::drawAxes(QPainter& painter)
{
    painter.save();

    const int w = width();
    const int h = height();
    const int cx = w / 2;
    const int cy = h / 2;

    QPen axis_pen_x(QColor(220, 80, 80));
    axis_pen_x.setWidth(2);
    painter.setPen(axis_pen_x);
    painter.drawLine(cx, cy, w, cy);

    QPen axis_pen_y(QColor(80, 220, 120));
    axis_pen_y.setWidth(2);
    painter.setPen(axis_pen_y);
    painter.drawLine(cx, cy, cx, 0);

    painter.setPen(Qt::white);
    painter.drawText(w - 20, cy - 6, "X");
    painter.drawText(cx + 6, 16, "Y");

    painter.restore();
}

QPointF MapViewWidget::worldToScreen(double wx, double wy) const
{
    const double cx = width() * 0.5;
    const double cy = height() * 0.5;

    const double sx = cx + wx * pixels_per_meter_;
    const double sy = cy - wy * pixels_per_meter_;

    return QPointF(sx, sy);
}

void MapViewWidget::drawRobot(QPainter& painter)
{
    if (!pose_valid_)
    {
        painter.save();
        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, "Waiting for odom...");
        painter.restore();
        return;
    }

    painter.save();

    const QPointF center = worldToScreen(robot_x_, robot_y_);

    const double robot_length = 0.6 * pixels_per_meter_;
    const double robot_width  = 0.4 * pixels_per_meter_;

    QPolygonF robot_shape;
    robot_shape << QPointF(robot_length * 0.5, 0.0)
                << QPointF(-robot_length * 0.5,  robot_width * 0.5)
                << QPointF(-robot_length * 0.2,  0.0)
                << QPointF(-robot_length * 0.5, -robot_width * 0.5);

    painter.translate(center);

    // Qt 屏幕坐标 y 朝下，因此这里取负号来匹配 ROS 逆时针为正的 yaw
    painter.rotate(-robot_yaw_ * 180.0 / M_PI);

    painter.setPen(QPen(QColor(20, 20, 20), 2));
    painter.setBrush(QBrush(QColor(80, 170, 255)));
    painter.drawPolygon(robot_shape);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QBrush(QColor(255, 220, 80)));
    painter.drawEllipse(QPointF(0.0, 0.0), 4.0, 4.0);

    painter.restore();

    painter.save();
    painter.setPen(Qt::white);
    painter.drawText(12, 24,
                     QString("Robot: x=%1 m, y=%2 m, yaw=%3 rad")
                         .arg(robot_x_, 0, 'f', 2)
                         .arg(robot_y_, 0, 'f', 2)
                         .arg(robot_yaw_, 0, 'f', 2));
    painter.restore();
}

}  // namespace robot_monitor
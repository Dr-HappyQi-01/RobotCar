#include "robot_monitor/map_view_widget.h"

#include <QMouseEvent>
#include <QPainter>
#include <QPaintEvent>
#include <QPolygonF>
#include <QWheelEvent>
#include <cmath>

namespace robot_monitor
{

MapViewWidget::MapViewWidget(QWidget* parent)
    : QWidget(parent),
      robot_x_(0.0),
      robot_y_(0.0),
      robot_yaw_(0.0),
      pose_valid_(false),
      pixels_per_meter_(50.0),
      view_scale_(1.0),
      view_offset_(0.0, 0.0),
      min_point_distance_(0.02),
      is_panning_(false),
      last_mouse_pos_(0, 0),
      has_selected_trajectory_(false)
{
    setMinimumSize(800, 500);
    setAutoFillBackground(true);
    setMouseTracking(true);
    setFocusPolicy(Qt::StrongFocus);
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

    if (pose_valid_)
    {
        appendTrajectoryPoint(robot_x_, robot_y_);
    }

    update();
}

void MapViewWidget::clearTrajectory()
{
    trajectory_points_.clear();
    update();
}

void MapViewWidget::appendTrajectoryPoint(double x, double y)
{
    const QPointF new_point(x, y);

    if (trajectory_points_.empty())
    {
        trajectory_points_.push_back(new_point);
        return;
    }

    const QPointF& last_point = trajectory_points_.back();
    const double dx = new_point.x() - last_point.x();
    const double dy = new_point.y() - last_point.y();
    const double dist = std::sqrt(dx * dx + dy * dy);

    if (dist >= min_point_distance_)
    {
        trajectory_points_.push_back(new_point);
    }
}

void MapViewWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);

    drawBackground(painter);
    drawGrid(painter);
    drawAxes(painter);
    drawTrajectory(painter);
    drawSelectedTrajectory(painter);
    drawRobot(painter);
    drawOverlayInfo(painter);
}

void MapViewWidget::wheelEvent(QWheelEvent* event)
{
    const QPoint angle_delta = event->angleDelta();

    if (angle_delta.y() == 0)
    {
        event->accept();
        return;
    }

    const double zoom_factor = (angle_delta.y() > 0) ? 1.15 : 1.0 / 1.15;
    view_scale_ *= zoom_factor;

    if (view_scale_ < 0.1)
    {
        view_scale_ = 0.1;
    }
    if (view_scale_ > 20.0)
    {
        view_scale_ = 20.0;
    }

    update();
    event->accept();
}

void MapViewWidget::mousePressEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        is_panning_ = true;
        last_mouse_pos_ = event->pos();
        setCursor(Qt::ClosedHandCursor);
        event->accept();
        return;
    }

    QWidget::mousePressEvent(event);
}

void MapViewWidget::mouseMoveEvent(QMouseEvent* event)
{
    if (is_panning_)
    {
        const QPoint delta = event->pos() - last_mouse_pos_;
        view_offset_ += QPointF(delta.x(), delta.y());
        last_mouse_pos_ = event->pos();
        update();
        event->accept();
        return;
    }

    QWidget::mouseMoveEvent(event);
}

void MapViewWidget::mouseReleaseEvent(QMouseEvent* event)
{
    if (event->button() == Qt::LeftButton)
    {
        is_panning_ = false;
        unsetCursor();
        event->accept();
        return;
    }

    QWidget::mouseReleaseEvent(event);
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
    const double cx = w * 0.5 + view_offset_.x();
    const double cy = h * 0.5 + view_offset_.y();
    const double spacing = pixels_per_meter_ * view_scale_;

    if (spacing < 8.0)
    {
        painter.restore();
        return;
    }

    for (double x = cx; x < w; x += spacing)
        painter.drawLine(QPointF(x, 0.0), QPointF(x, h));
    for (double x = cx; x >= 0.0; x -= spacing)
        painter.drawLine(QPointF(x, 0.0), QPointF(x, h));

    for (double y = cy; y < h; y += spacing)
        painter.drawLine(QPointF(0.0, y), QPointF(w, y));
    for (double y = cy; y >= 0.0; y -= spacing)
        painter.drawLine(QPointF(0.0, y), QPointF(w, y));

    painter.restore();
}

void MapViewWidget::drawAxes(QPainter& painter)
{
    painter.save();

    const int w = width();
    const int h = height();
    const double cx = w * 0.5 + view_offset_.x();
    const double cy = h * 0.5 + view_offset_.y();

    QPen axis_pen_x(QColor(220, 80, 80));
    axis_pen_x.setWidth(2);
    painter.setPen(axis_pen_x);
    painter.drawLine(QPointF(cx, cy), QPointF(w, cy));

    QPen axis_pen_y(QColor(80, 220, 120));
    axis_pen_y.setWidth(2);
    painter.setPen(axis_pen_y);
    painter.drawLine(QPointF(cx, cy), QPointF(cx, 0.0));

    painter.setPen(Qt::white);
    painter.drawText(QPointF(w - 20.0, cy - 6.0), "X");
    painter.drawText(QPointF(cx + 6.0, 16.0), "Y");

    painter.restore();
}

QPointF MapViewWidget::worldToScreen(double wx, double wy) const
{
    const double cx = width() * 0.5 + view_offset_.x();
    const double cy = height() * 0.5 + view_offset_.y();

    const double scale = pixels_per_meter_ * view_scale_;

    const double sx = cx + wx * scale;
    const double sy = cy - wy * scale;

    return QPointF(sx, sy);
}

void MapViewWidget::drawTrajectory(QPainter& painter)
{
    if (trajectory_points_.size() < 2)
    {
        return;
    }

    painter.save();

    QPen traj_pen(QColor(255, 200, 80));
    traj_pen.setWidth(2);
    painter.setPen(traj_pen);
    painter.setBrush(Qt::NoBrush);

    QPolygonF screen_path;
    screen_path.reserve(static_cast<int>(trajectory_points_.size()));

    for (const QPointF& pt : trajectory_points_)
    {
        screen_path << worldToScreen(pt.x(), pt.y());
    }

    painter.drawPolyline(screen_path);

    painter.restore();
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
    const double scale = pixels_per_meter_ * view_scale_;

    const double robot_length = 0.6 * scale;
    const double robot_width  = 0.4 * scale;

    QPolygonF robot_shape;
    robot_shape << QPointF(robot_length * 0.5, 0.0)
                << QPointF(-robot_length * 0.5,  robot_width * 0.5)
                << QPointF(-robot_length * 0.2,  0.0)
                << QPointF(-robot_length * 0.5, -robot_width * 0.5);

    painter.translate(center);
    painter.rotate(-robot_yaw_ * 180.0 / M_PI);

    painter.setPen(QPen(QColor(20, 20, 20), 2));
    painter.setBrush(QBrush(QColor(80, 170, 255)));
    painter.drawPolygon(robot_shape);

    painter.setPen(Qt::NoPen);
    painter.setBrush(QBrush(QColor(255, 220, 80)));
    painter.drawEllipse(QPointF(0.0, 0.0), 4.0, 4.0);

    painter.restore();
}

void MapViewWidget::drawOverlayInfo(QPainter& painter)
{
    painter.save();

    painter.setPen(Qt::white);
    painter.drawText(12, 24,
                     QString("Robot: x=%1 m, y=%2 m, yaw=%3 rad")
                         .arg(robot_x_, 0, 'f', 2)
                         .arg(robot_y_, 0, 'f', 2)
                         .arg(robot_yaw_, 0, 'f', 2));

    painter.drawText(12, 44,
                     QString("Trajectory points: %1")
                         .arg(static_cast<int>(trajectory_points_.size())));

    painter.drawText(12, 64,
                     QString("Zoom: x%1")
                         .arg(view_scale_, 0, 'f', 2));

    painter.drawText(12, 84,
                     QString("Pan: dx=%1, dy=%2 px")
                         .arg(view_offset_.x(), 0, 'f', 1)
                         .arg(view_offset_.y(), 0, 'f', 1));

    painter.drawText(12, 104,
                 QString("Selected history: %1")
                     .arg(has_selected_trajectory_
                              ? QString::fromStdString(selected_trajectory_record_.name)
                              : QString("none")));

    painter.restore();
}


void MapViewWidget::setSelectedTrajectory(const TrajectoryRecord& record)
{
    selected_trajectory_record_ = record;
    has_selected_trajectory_ = true;
    update();
}

void MapViewWidget::clearSelectedTrajectory()
{
    selected_trajectory_record_ = TrajectoryRecord();
    has_selected_trajectory_ = false;
    update();
}


void MapViewWidget::drawSelectedTrajectory(QPainter& painter)
{
    if (!has_selected_trajectory_ || selected_trajectory_record_.points.size() < 2)
    {
        return;
    }

    painter.save();

    QPen selected_pen(QColor(120, 255, 160));
    selected_pen.setWidth(3);
    painter.setPen(selected_pen);
    painter.setBrush(Qt::NoBrush);

    QPolygonF screen_path;
    screen_path.reserve(static_cast<int>(selected_trajectory_record_.points.size()));

    std::vector<QPointF> world_points;
    world_points.reserve(selected_trajectory_record_.points.size());

    for (const auto& pt : selected_trajectory_record_.points)
    {
        screen_path << worldToScreen(pt.x, pt.y);
        world_points.emplace_back(pt.x, pt.y);
    }

    painter.drawPolyline(screen_path);
    painter.restore();

    drawStartEndMarkers(painter, world_points);
}

void MapViewWidget::drawStartEndMarkers(QPainter& painter, const std::vector<QPointF>& points)
{
    if (points.empty())
    {
        return;
    }

    const QPointF start_screen = worldToScreen(points.front().x(), points.front().y());
    const QPointF end_screen   = worldToScreen(points.back().x(), points.back().y());

    painter.save();

    // Start marker
    painter.setPen(QPen(QColor(20, 20, 20), 1));
    painter.setBrush(QBrush(QColor(80, 220, 120)));
    painter.drawEllipse(start_screen, 7.0, 7.0);
    painter.setPen(Qt::white);
    painter.drawText(start_screen + QPointF(10.0, -10.0), "S");

    // End marker
    painter.setPen(QPen(QColor(20, 20, 20), 1));
    painter.setBrush(QBrush(QColor(255, 90, 90)));
    painter.drawEllipse(end_screen, 7.0, 7.0);
    painter.setPen(Qt::white);
    painter.drawText(end_screen + QPointF(10.0, -10.0), "E");

    painter.restore();
}

}  // namespace robot_monitor
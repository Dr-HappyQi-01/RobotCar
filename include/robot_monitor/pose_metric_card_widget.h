#ifndef ROBOT_MONITOR_POSE_METRIC_CARD_WIDGET_H
#define ROBOT_MONITOR_POSE_METRIC_CARD_WIDGET_H

#include <QWidget>

namespace robot_monitor
{

class PoseMetricCardWidget : public QWidget
{
    Q_OBJECT

public:
    explicit PoseMetricCardWidget(QWidget* parent = nullptr);

    void setPose(double x, double y, double yaw);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    void drawMetricBox(QPainter& p,
                       const QRectF& rect,
                       const QString& title,
                       const QString& value,
                       const QColor& accent_color,
                       bool draw_yaw_indicator = false);

private:
    double x_;
    double y_;
    double yaw_;
};

}  // namespace robot_monitor

#endif
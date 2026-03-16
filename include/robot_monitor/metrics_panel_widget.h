#ifndef ROBOT_MONITOR_METRICS_PANEL_WIDGET_H
#define ROBOT_MONITOR_METRICS_PANEL_WIDGET_H



#include <QWidget>
#include <QLabel>
namespace robot_monitor
{

class PoseMetricCardWidget;
class SparklineWidget;
class ProgressPillWidget;

class MetricsPanelWidget : public QWidget
{
    Q_OBJECT

public:
    explicit MetricsPanelWidget(QWidget* parent = nullptr);

    void setPose(double x, double y, double yaw);
    void setSystemText(const QString& text);

    void addVelocitySamples(double linear_v, double angular_v);

    void setSystemCpuUsage(double cpu_percent);
    void setAppCpuUsage(double cpu_percent);

    void setBatteryPercent(double battery_percent);
    void setBatteryUnavailable();

private:
    PoseMetricCardWidget* pose_card_;
    SparklineWidget* linear_chart_;
    SparklineWidget* angular_chart_;

    QWidget* system_state_container_;
    QLabel* system_state_label_;

    ProgressPillWidget* system_cpu_widget_;
    ProgressPillWidget* app_cpu_widget_;
    ProgressPillWidget* battery_widget_;
};

}  // namespace robot_monitor

#endif
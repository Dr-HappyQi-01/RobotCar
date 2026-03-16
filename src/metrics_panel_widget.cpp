#include "robot_monitor/metrics_panel_widget.h"

#include "robot_monitor/pose_metric_card_widget.h"
#include "robot_monitor/progress_pill_widget.h"
#include "robot_monitor/sparkline_widget.h"

#include <QFrame>
#include <QLabel>
#include <QVBoxLayout>
#include <QHBoxLayout>

namespace robot_monitor
{

MetricsPanelWidget::MetricsPanelWidget(QWidget* parent)
    : QWidget(parent)
{
    setObjectName("MetricsPanelWidget");

    QVBoxLayout* root = new QVBoxLayout(this);
    root->setContentsMargins(5, 5, 5, 5);
    root->setSpacing(5);

    // QLabel* title = new QLabel("Real-time Metrics", this);
    // title->setObjectName("MetricsPanelTitle");

    pose_card_ = new PoseMetricCardWidget(this);

    QFrame* system_card = new QFrame(this);
    system_card->setObjectName("MetricsCard");
    QHBoxLayout* system_layout = new QHBoxLayout(system_card);
    system_layout->setContentsMargins(14, 10, 14, 10);

    QLabel* system_tag = new QLabel("System", system_card);
    system_tag->setObjectName("SystemTag");

    system_state_label_ = new QLabel("odom online", system_card);
    system_state_label_->setObjectName("SystemValue");

    system_layout->addWidget(system_tag);
    system_layout->addStretch();
    system_layout->addWidget(system_state_label_);

    QFrame* chart_card = new QFrame(this);
    chart_card->setObjectName("MetricsCard");
    QHBoxLayout* chart_layout = new QHBoxLayout(chart_card);
    chart_layout->setContentsMargins(10, 10, 10, 10);
    chart_layout->setSpacing(10);

    linear_chart_ = new SparklineWidget(chart_card);
    angular_chart_ = new SparklineWidget(chart_card);

    linear_chart_->setTitle("V");
    linear_chart_->setUnit("m/s");
    linear_chart_->setLineColor(QColor(135, 235, 110));
    linear_chart_->setYRange(-1.0, 1.0);

    angular_chart_->setTitle("W");
    angular_chart_->setUnit("rad/s");
    angular_chart_->setLineColor(QColor(110, 200, 255));
    angular_chart_->setYRange(-1.5, 1.5);

    chart_layout->addWidget(linear_chart_);
    chart_layout->addWidget(angular_chart_);

    QFrame* bottom_card = new QFrame(this);
    bottom_card->setObjectName("MetricsCard");
    QVBoxLayout* bottom_layout = new QVBoxLayout(bottom_card);
    bottom_layout->setContentsMargins(10, 10, 10, 10);
    bottom_layout->setSpacing(10);

    system_cpu_widget_ = new ProgressPillWidget(bottom_card);
    system_cpu_widget_->setTitle("Sys CPU");
    system_cpu_widget_->setAccentColor(QColor(130, 220, 90));

    app_cpu_widget_ = new ProgressPillWidget(bottom_card);
    app_cpu_widget_->setTitle("App CPU");
    app_cpu_widget_->setAccentColor(QColor(100, 190, 255));

    battery_widget_ = new ProgressPillWidget(bottom_card);
    battery_widget_->setTitle("Battery");
    battery_widget_->setAccentColor(QColor(100, 190, 255));
    battery_widget_->setAvailable(false);
    battery_widget_->setDisplayText("N/A");

    bottom_layout->addWidget(system_cpu_widget_);
    bottom_layout->addWidget(app_cpu_widget_);
    bottom_layout->addWidget(battery_widget_);

    // root->addWidget(title);
    root->addWidget(pose_card_);
    root->addWidget(system_card);
    root->addWidget(chart_card);
    root->addWidget(bottom_card);
    root->addStretch();

    setStyleSheet(
        // "#MetricsPanelTitle {"
        // "  color: #eef3f8;"
        // "  font-size: 20px;"
        // "  font-weight: 700;"
        // "  padding-left: 2px;"
        // "}"
        "#MetricsCard {"
        "  background-color: #1e2430;"
        "  border: 1px solid #313949;"
        "  border-radius: 14px;"
        "}"
        "#SystemTag {"
        "  color: #94a0b2;"
        "  font-size: 13px;"
        "  font-weight: 600;"
        "}"
        "#SystemValue {"
        "  color: #eef3f8;"
        "  font-size: 14px;"
        "  font-weight: 600;"
        "}"
        "QLabel {"
        "  color: #e3e8ef;"
        "}"
    );
}

void MetricsPanelWidget::setPose(double x, double y, double yaw)
{
    pose_card_->setPose(x, y, yaw);
}

void MetricsPanelWidget::setSystemText(const QString& text)
{
    system_state_label_->setText(text);
}

void MetricsPanelWidget::addVelocitySamples(double linear_v, double angular_v)
{
    linear_chart_->addSample(linear_v);
    angular_chart_->addSample(angular_v);
}

void MetricsPanelWidget::setSystemCpuUsage(double cpu_percent)
{
    system_cpu_widget_->setAvailable(true);
    system_cpu_widget_->setValue(cpu_percent);
}

void MetricsPanelWidget::setAppCpuUsage(double cpu_percent)
{
    app_cpu_widget_->setAvailable(true);
    app_cpu_widget_->setValue(cpu_percent);
}

void MetricsPanelWidget::setBatteryPercent(double battery_percent)
{
    battery_widget_->setAvailable(true);
    battery_widget_->setValue(battery_percent);
}

void MetricsPanelWidget::setBatteryUnavailable()
{
    battery_widget_->setAvailable(false);
    battery_widget_->setDisplayText("N/A");
}

}  // namespace robot_monitor
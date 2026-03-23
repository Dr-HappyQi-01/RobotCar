#ifndef ROBOT_MONITOR_REWARD_CURVE_WIDGET_H
#define ROBOT_MONITOR_REWARD_CURVE_WIDGET_H

#include <QWidget>
#include <vector>
#include "robot_monitor/trajectory_types.h"

namespace robot_monitor
{



class RewardCurveWidget : public QWidget
{
    Q_OBJECT

public:
    explicit RewardCurveWidget(QWidget* parent = nullptr);

    void setMethodName(const QString& method_name);
    void setRewardData(const std::vector<EpisodeRewardPoint>& data);
    void clearData();
    void setStatusText(const QString& text);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QString method_name_;
    QString status_text_;
    std::vector<EpisodeRewardPoint> reward_data_;
};

}  // namespace robot_monitor

#endif
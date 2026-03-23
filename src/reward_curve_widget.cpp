#include "robot_monitor/reward_curve_widget.h"

#include <QPainter>
#include <QPaintEvent>
#include <QPainterPath>
#include <algorithm>
#include  <iostream>
#include <cmath>

namespace robot_monitor
{

RewardCurveWidget::RewardCurveWidget(QWidget* parent)
    : QWidget(parent),
      method_name_("None"),
      status_text_("No reward data")
{
    setMinimumSize(360, 220);
    setAutoFillBackground(true);
}

void RewardCurveWidget::setMethodName(const QString& method_name)
{
    method_name_ = method_name;
    update();
}

void RewardCurveWidget::setRewardData(const std::vector<EpisodeRewardPoint>& data)
{
    reward_data_ = data;
    if (reward_data_.empty())
    {
        status_text_ = "No reward data";
    }
    else
    {
        status_text_ = QString("Episodes: %1").arg(static_cast<int>(reward_data_.size()));
    }
    update();
}

void RewardCurveWidget::clearData()
{
    reward_data_.clear();
    status_text_ = "No reward data";
    update();
}

void RewardCurveWidget::setStatusText(const QString& text)
{
    status_text_ = text;
    update();
}

void RewardCurveWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // p.fillRect(rect(), QColor(24, 28, 35));

    QRectF outer = rect().adjusted(6, 6, -6, -6);

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(30, 36, 48));
    p.drawRoundedRect(outer, 16, 16);

    p.setPen(QPen(QColor(55, 65, 82), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(outer, 16, 16);

    QFont title_font = font();
    title_font.setPointSize(12);
    title_font.setBold(true);
    p.setFont(title_font);
    p.setPen(QColor(238, 243, 248));
    p.drawText(QRectF(outer.left() + 14, outer.top() + 10, 220, 22),
               Qt::AlignLeft | Qt::AlignVCenter,
               "Reward Curve");

    QFont sub_font = font();
    sub_font.setPointSize(10);
    p.setFont(sub_font);
    p.setPen(QColor(150, 165, 185));
    p.drawText(QRectF(outer.left() + 14, outer.top() + 34, 260, 18),
               Qt::AlignLeft | Qt::AlignVCenter,
               QString("Method: %1").arg(method_name_));

    QRectF plot = outer.adjusted(48, 62, -18, -42);

    p.setPen(QPen(QColor(52, 60, 74), 1, Qt::DotLine));
    for (int i = 0; i <= 4; ++i)
    {
        const qreal y = plot.top() + i * plot.height() / 4.0;
        p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    }

    p.setPen(QPen(QColor(90, 100, 118), 1.2));
    p.drawLine(plot.bottomLeft(), plot.bottomRight());
    p.drawLine(plot.bottomLeft(), plot.topLeft());

    if (reward_data_.empty())
    {
        p.setPen(QColor(180, 190, 205));
        p.drawText(plot, Qt::AlignCenter, status_text_);
        return;
    }

    double min_reward = reward_data_.front().reward;
    double max_reward = reward_data_.front().reward;
    int min_episode = reward_data_.front().episode;
    int max_episode = reward_data_.front().episode;

    for (const auto& pt : reward_data_)
    {
        min_reward = std::min(min_reward, pt.reward);
        max_reward = std::max(max_reward, pt.reward);
        min_episode = std::min(min_episode, pt.episode);
        max_episode = std::max(max_episode, pt.episode);
    }

    if (std::abs(max_reward - min_reward) < 1e-6)
    {
        max_reward += 1.0;
        min_reward -= 1.0;
    }

    if (max_episode == min_episode)
    {
        max_episode += 1;
    }

    p.setPen(QColor(190, 198, 210));
    p.setFont(sub_font);
    p.drawText(QRectF(outer.left() + 2, plot.top() - 8, 40, 16),
               Qt::AlignRight | Qt::AlignVCenter,
               QString::number(max_reward, 'f', 1));
    p.drawText(QRectF(outer.left() + 2, plot.center().y() - 8, 40, 16),
               Qt::AlignRight | Qt::AlignVCenter,
               QString::number((min_reward + max_reward) * 0.5, 'f', 1));
    p.drawText(QRectF(outer.left() + 2, plot.bottom() - 8, 40, 16),
               Qt::AlignRight | Qt::AlignVCenter,
               QString::number(min_reward, 'f', 1));

    // x-axis episode ticks: first / middle / last
        // adaptive x-axis ticks
    int max_tick_count = std::max(3, static_cast<int>(plot.width() / 80.0));
    max_tick_count = std::min(max_tick_count, static_cast<int>(reward_data_.size()));
    max_tick_count = std::max(max_tick_count, 2);

    std::vector<int> tick_indices;
    tick_indices.reserve(max_tick_count);

    if (reward_data_.size() <= static_cast<size_t>(max_tick_count))
    {
        for (size_t i = 0; i < reward_data_.size(); ++i)
        {
            tick_indices.push_back(static_cast<int>(i));
        }
    }
    else
    {
        for (int i = 0; i < max_tick_count; ++i)
        {
            int idx = static_cast<int>(
                std::round(i * (reward_data_.size() - 1.0) / (max_tick_count - 1.0)));
            if (tick_indices.empty() || tick_indices.back() != idx)
            {
                tick_indices.push_back(idx);
            }
        }
    }

    p.setPen(QPen(QColor(52, 60, 74), 1, Qt::DotLine));
    for (int idx : tick_indices)
    {
        const double tx = static_cast<double>(reward_data_[idx].episode - min_episode) /
                          static_cast<double>(max_episode - min_episode);
        const double x = plot.left() + tx * plot.width();
        p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
    }

    p.setPen(QColor(190, 198, 210));
    for (int idx : tick_indices)
    {
        const double tx = static_cast<double>(reward_data_[idx].episode - min_episode) /
                          static_cast<double>(max_episode - min_episode);
        const double x = plot.left() + tx * plot.width();

        p.drawText(QRectF(x - 28, plot.bottom() + 6, 56, 16),
                   Qt::AlignHCenter | Qt::AlignTop,
                   QString::number(reward_data_[idx].episode));
    }


    QPainterPath path;
    for (size_t i = 0; i < reward_data_.size(); ++i)
    {
        const double tx = static_cast<double>(reward_data_[i].episode - min_episode) /
                          static_cast<double>(max_episode - min_episode);
        const double x = plot.left() + tx * plot.width();

        const double ty = (reward_data_[i].reward - min_reward) /
                          (max_reward - min_reward);
        const double y = plot.bottom() - ty * plot.height();

        if (i == 0)
            path.moveTo(x, y);
        else
            path.lineTo(x, y);
    }

    p.setPen(QPen(QColor(110, 200, 255), 2.4));
    p.drawPath(path);


        // draw points only when data count is not too large
    const bool draw_points = reward_data_.size() <= 80;

    if (draw_points)
    {
        p.setPen(Qt::NoPen);
        p.setBrush(QColor(110, 200, 255));

        for (const auto& pt : reward_data_)
        {
            const double tx = static_cast<double>(pt.episode - min_episode) /
                              static_cast<double>(max_episode - min_episode);
            const double x = plot.left() + tx * plot.width();

            const double ty = (pt.reward - min_reward) /
                              (max_reward - min_reward);
            const double y = plot.bottom() - ty * plot.height();

            p.drawEllipse(QPointF(x, y), 2.5, 2.5);
        }
    }



    p.setPen(QColor(150, 165, 185));
    p.drawText(QRectF(plot.right() - 80, plot.bottom() + 22, 80, 16),
               Qt::AlignRight | Qt::AlignVCenter,
               "episode");
}

}  // namespace robot_monitor
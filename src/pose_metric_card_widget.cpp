#include "robot_monitor/pose_metric_card_widget.h"

#include <QPainter>
#include <QPaintEvent>
#include <QtMath>

namespace robot_monitor
{

PoseMetricCardWidget::PoseMetricCardWidget(QWidget* parent)
    : QWidget(parent),
      x_(0.0),
      y_(0.0),
      yaw_(0.0)
{
    setMinimumHeight(100);
    setAttribute(Qt::WA_TranslucentBackground);
}

void PoseMetricCardWidget::setPose(double x, double y, double yaw)
{
    x_ = x;
    y_ = y;
    yaw_ = yaw;
    update();
}

void PoseMetricCardWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    // p.fillRect(rect(), QColor(24, 28, 35));

    QRectF outer = rect().adjusted(4, 4, -4, -4);

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(30, 36, 48));
    p.drawRoundedRect(outer, 16, 16); 

    p.setPen(QPen(QColor(55, 65, 82), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(outer, 16, 16);

    const qreal gap = 10.0;
    const qreal box_w = (outer.width() - 4 * gap) / 3.0;
    const qreal box_h = outer.height() - 2 * gap;

    QRectF box_x(outer.left() + gap, outer.top() + gap, box_w, box_h);
    QRectF box_y(box_x.right() + gap, outer.top() + gap, box_w, box_h);
    QRectF box_yaw(box_y.right() + gap, outer.top() + gap, box_w, box_h);

    drawMetricBox(p, box_x, "X", QString::number(x_, 'f', 3), QColor(255, 85, 85), false);
    drawMetricBox(p, box_y, "Y", QString::number(y_, 'f', 3), QColor(120, 255, 160), false);
    drawMetricBox(p, box_yaw, "Yaw", QString::number(yaw_, 'f', 3), QColor(255, 200, 90), true);
}

void PoseMetricCardWidget::drawMetricBox(QPainter& p,
                                         const QRectF& rect,
                                         const QString& title,
                                         const QString& value,
                                         const QColor& accent_color,
                                         bool draw_yaw_indicator)
{
    p.save();

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(22, 27, 36));
    p.drawRoundedRect(rect, 12, 12);

    p.setPen(QPen(QColor(48, 58, 72), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(rect, 12, 12);

    QRectF accent_rect(rect.left() + 8, rect.top() + 8, rect.width() - 16, 5);
    p.setPen(Qt::NoPen);
    p.setBrush(accent_color);
    p.drawRoundedRect(accent_rect, 2.5, 2.5);

    QFont title_font = font();
    title_font.setPointSize(10);
    title_font.setBold(true);
    p.setFont(title_font);
    p.setPen(QColor(170, 180, 195));
    p.drawText(QRectF(rect.left() + 12, rect.top() + 18, rect.width() - 24, 20),
               Qt::AlignLeft | Qt::AlignVCenter,
               title);

    QFont value_font = font();
    value_font.setPointSize(10);
    value_font.setBold(true);
    p.setFont(value_font);
    p.setPen(QColor(238, 243, 248));
    p.drawText(QRectF(rect.left() + 12, rect.top() + 42, rect.width() - 24, 26),
               Qt::AlignLeft | Qt::AlignVCenter,
               value);

    if (draw_yaw_indicator)
    {
        const QPointF center(rect.right() - 28, rect.bottom() - 28);
        const qreal radius = 16.0;

        p.setPen(QPen(QColor(70, 78, 92), 1.2));
        p.setBrush(Qt::NoBrush);
        p.drawEllipse(center, radius, radius);

        const qreal angle_deg = -yaw_ * 180.0 / M_PI;
        const qreal angle_rad = qDegreesToRadians(angle_deg);

        QPointF arrow_tip(center.x() + radius * 0.75 * qCos(angle_rad),
                          center.y() + radius * 0.75 * qSin(angle_rad));
        QPointF arrow_back(center.x() - radius * 0.20 * qCos(angle_rad),
                           center.y() - radius * 0.20 * qSin(angle_rad));

        p.setPen(QPen(accent_color, 2.2));
        p.drawLine(arrow_back, arrow_tip);

        QPointF left_wing(
            arrow_tip.x() - 5 * qCos(angle_rad - M_PI / 6.0),
            arrow_tip.y() - 5 * qSin(angle_rad - M_PI / 6.0));
        QPointF right_wing(
            arrow_tip.x() - 5 * qCos(angle_rad + M_PI / 6.0),
            arrow_tip.y() - 5 * qSin(angle_rad + M_PI / 6.0));

        p.drawLine(arrow_tip, left_wing);
        p.drawLine(arrow_tip, right_wing);
    }

    p.restore();
}

}  // namespace robot_monitor
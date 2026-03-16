#include "robot_monitor/progress_pill_widget.h"

#include <QPainter>
#include <QPaintEvent>

namespace robot_monitor
{

ProgressPillWidget::ProgressPillWidget(QWidget* parent)
    : QWidget(parent),
      title_("CPU"),
      display_text_("0%"),
      accent_color_(QColor(130, 220, 90)),
      value_percent_(0.0),
      available_(true)
{
    setMinimumHeight(56);
    setAttribute(Qt::WA_TranslucentBackground);
}

void ProgressPillWidget::setTitle(const QString& title)
{
    title_ = title;
    update();
}

void ProgressPillWidget::setValue(double value_percent)
{
    value_percent_ = qBound(0.0, value_percent, 100.0);
    if (available_)
    {
        display_text_ = QString("%1%").arg(static_cast<int>(value_percent_));
    }
    update();
}

void ProgressPillWidget::setAccentColor(const QColor& color)
{
    accent_color_ = color;
    update();
}

void ProgressPillWidget::setDisplayText(const QString& text)
{
    display_text_ = text;
    update();
}

void ProgressPillWidget::setAvailable(bool available)
{
    available_ = available;
    update();
}

void ProgressPillWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    QRectF r = rect().adjusted(4, 4, -4, -4);
    // p.fillRect(rect(), QColor(20, 24, 30));

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(28, 33, 42));
    p.drawRoundedRect(r, 12, 12);

    p.setPen(QColor(230, 235, 242));
    QFont label_font = font();
    label_font.setPointSize(10);
    p.setFont(label_font);
    p.drawText(QRectF(r.left() + 12, r.top() + 8, 70, 20),
               Qt::AlignLeft | Qt::AlignVCenter, title_);

    QFont value_font = font();
    value_font.setPointSize(16);
    value_font.setBold(true);
    p.setFont(value_font);

    if (available_)
        p.setPen(accent_color_);
    else
        p.setPen(QColor(150, 155, 165));

    p.drawText(QRectF(r.left() + 12, r.top() + 24, 70, 24),
               Qt::AlignLeft | Qt::AlignVCenter, display_text_);

    QRectF bar_rect(r.left() + 88, r.center().y() - 8, r.width() - 100, 16);
    p.setPen(QPen(QColor(52, 60, 72), 1));
    p.setBrush(QColor(18, 22, 28));
    p.drawRoundedRect(bar_rect, 8, 8);

    if (available_)
    {
        QRectF fill_rect = bar_rect;
        fill_rect.setWidth(bar_rect.width() * (value_percent_ / 100.0));
        p.setPen(Qt::NoPen);
        p.setBrush(accent_color_);
        p.drawRoundedRect(fill_rect, 8, 8);
    }
}

}  // namespace robot_monitor
#include "robot_monitor/sparkline_widget.h"

#include <QPainter>
#include <QPaintEvent>
#include <QPainterPath>
#include <QFontMetrics> // 引入字体测量工具，用于计算文字宽度居中

namespace robot_monitor
{

SparklineWidget::SparklineWidget(QWidget* parent)
    : QWidget(parent),
      line_color_(QColor(120, 255, 160)),
      max_samples_(60),
      min_value_(-0.5),
      max_value_(0.5),
      title_("V"),
      unit_("m/s")
{
    setMinimumSize(170, 120);
    setAttribute(Qt::WA_TranslucentBackground);
}

void SparklineWidget::addSample(double value)
{
    samples_.push_back(value);
    while (samples_.size() > max_samples_)
    {
        samples_.pop_front();
    }
    update();
}

void SparklineWidget::clearSamples()
{
    samples_.clear();
    update();
}

void SparklineWidget::setLineColor(const QColor& color)
{
    line_color_ = color;
    update();
}

void SparklineWidget::setMaxSamples(int max_samples)
{
    max_samples_ = qMax(10, max_samples);
}

void SparklineWidget::setYRange(double min_value, double max_value)
{
    min_value_ = min_value;
    max_value_ = max_value;
    if (qFuzzyCompare(min_value_, max_value_))
    {
        max_value_ = min_value_ + 1.0;
    }
    update();
}

void SparklineWidget::setTitle(const QString& title)
{
    title_ = title;
    update();
}

void SparklineWidget::setUnit(const QString& unit)
{
    unit_ = unit;
    update();
}

void SparklineWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter p(this);
    p.setRenderHint(QPainter::Antialiasing, true);

    QRectF outer = rect().adjusted(4, 4, -4, -4);

    p.setPen(Qt::NoPen);
    p.setBrush(QColor(30, 36, 48));
    p.drawRoundedRect(outer, 14, 14);

    p.setPen(QPen(QColor(55, 65, 82), 1));
    p.setBrush(Qt::NoBrush);
    p.drawRoundedRect(outer, 14, 14);

    // --- 开始居中绘制标题和单位 ---
    QFont title_font = font();
    title_font.setPointSize(10);
    title_font.setBold(true);
    p.setFont(title_font);

    QString unit_str = QString("(%1)").arg(unit_);
    QFontMetrics fm(title_font);
    
    // 计算文字的具体像素宽度 (兼容 Qt5)
    #if QT_VERSION >= QT_VERSION_CHECK(5, 11, 0)
        int title_w = fm.horizontalAdvance(title_);
        int space_w = fm.horizontalAdvance("  "); 
        int unit_w = fm.horizontalAdvance(unit_str);
    #else
        int title_w = fm.width(title_);
        int space_w = fm.width("  "); 
        int unit_w = fm.width(unit_str);
    #endif
    
    int total_w = title_w + space_w + unit_w;

    // 动态计算居中的起始 X 坐标
    qreal start_x = outer.left() + (outer.width() - total_w) / 2.0;
    qreal text_y = outer.top() + 8; // 稍微下移一点点让重心更稳
    qreal text_h = 20;

    // 画高亮的字母 (V 或 W)
    p.setPen(line_color_);
    p.drawText(QRectF(start_x, text_y, title_w, text_h),
               Qt::AlignLeft | Qt::AlignVCenter,
               title_);

    // 画灰色的单位
    p.setPen(QColor(190, 198, 210));
    p.drawText(QRectF(start_x + title_w + space_w, text_y, unit_w, text_h),
               Qt::AlignLeft | Qt::AlignVCenter,
               unit_str);
    // --- 标题绘制结束 ---

    // 将波形图的顶部坐标下压到 45，避开顶部的文字和 1.0 刻度
    QRectF plot = outer.adjusted(36, 35, -12, -16);

    // grid
    p.setPen(QPen(QColor(52, 60, 74), 1, Qt::DotLine));
    for (int i = 0; i <= 4; ++i)
    {
        const qreal y = plot.top() + i * plot.height() / 4.0;
        p.drawLine(QPointF(plot.left(), y), QPointF(plot.right(), y));
    }
    for (int i = 0; i <= 3; ++i)
    {
        const qreal x = plot.left() + i * plot.width() / 3.0;
        p.drawLine(QPointF(x, plot.top()), QPointF(x, plot.bottom()));
    }

    // axis labels
    p.setPen(QColor(190, 198, 210));
    QFont axis_font = font();
    axis_font.setPointSize(9);
    p.setFont(axis_font);

    p.drawText(QRectF(outer.left() + 4, plot.top() - 8, 28, 16),
               Qt::AlignRight | Qt::AlignVCenter,
               QString::number(max_value_, 'f', 1));

    p.drawText(QRectF(outer.left() + 4, plot.center().y() - 8, 28, 16),
               Qt::AlignRight | Qt::AlignVCenter,
               QString::number((min_value_ + max_value_) * 0.5, 'f', 1));

    p.drawText(QRectF(outer.left() + 4, plot.bottom() - 8, 28, 16),
               Qt::AlignRight | Qt::AlignVCenter,
               QString::number(min_value_, 'f', 1));

    p.setPen(QPen(QColor(90, 100, 118), 1.2));
    p.drawLine(plot.bottomLeft(), plot.bottomRight());
    p.drawLine(plot.bottomLeft(), plot.topLeft());

    if (samples_.size() < 2)
    {
        return;
    }

    QPainterPath path;
    for (int i = 0; i < samples_.size(); ++i)
    {
        const double t = static_cast<double>(i) / static_cast<double>(samples_.size() - 1);
        const double x = plot.left() + t * plot.width();

        const double normalized = (samples_[i] - min_value_) / (max_value_ - min_value_);
        const double y = plot.bottom() - normalized * plot.height();

        if (i == 0)
        {
            path.moveTo(x, y);
        }
        else
        {
            path.lineTo(x, y);
        }
    }

    p.setPen(QPen(line_color_, 2.4));
    p.drawPath(path);
}

}  // namespace robot_monitor
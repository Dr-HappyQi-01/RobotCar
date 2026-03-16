#ifndef ROBOT_MONITOR_SPARKLINE_WIDGET_H
#define ROBOT_MONITOR_SPARKLINE_WIDGET_H

#include <QColor>
#include <QVector>
#include <QWidget>

namespace robot_monitor
{

class SparklineWidget : public QWidget
{
    Q_OBJECT

public:
    explicit SparklineWidget(QWidget* parent = nullptr);

    void addSample(double value);
    void clearSamples();

    void setLineColor(const QColor& color);
    void setMaxSamples(int max_samples);
    void setYRange(double min_value, double max_value);
    void setTitle(const QString& title);
    void setUnit(const QString& unit);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QVector<double> samples_;
    QColor line_color_;
    int max_samples_;
    double min_value_;
    double max_value_;
    QString title_;
    QString unit_;
};

}  // namespace robot_monitor

#endif
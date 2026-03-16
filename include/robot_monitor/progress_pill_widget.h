#ifndef ROBOT_MONITOR_PROGRESS_PILL_WIDGET_H
#define ROBOT_MONITOR_PROGRESS_PILL_WIDGET_H

#include <QColor>
#include <QString>
#include <QWidget>

namespace robot_monitor
{

class ProgressPillWidget : public QWidget
{
    Q_OBJECT

public:
    explicit ProgressPillWidget(QWidget* parent = nullptr);

    void setTitle(const QString& title);
    void setValue(double value_percent);
    void setAccentColor(const QColor& color);
    void setDisplayText(const QString& text);
    void setAvailable(bool available);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QString title_;
    QString display_text_;
    QColor accent_color_;
    double value_percent_;
    bool available_;
};

}  // namespace robot_monitor

#endif
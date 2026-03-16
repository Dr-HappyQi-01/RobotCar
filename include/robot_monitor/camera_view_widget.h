#ifndef ROBOT_MONITOR_CAMERA_VIEW_WIDGET_H
#define ROBOT_MONITOR_CAMERA_VIEW_WIDGET_H

#include <QImage>
#include <QMutex>
#include <QWidget>

namespace robot_monitor
{

class CameraViewWidget : public QWidget
{
    Q_OBJECT

public:
    explicit CameraViewWidget(QWidget* parent = nullptr);
    ~CameraViewWidget();

public slots:
    void setImage(const QImage& image);
    void setStatus(bool online, const QString& status_text);

protected:
    void paintEvent(QPaintEvent* event) override;

private:
    QImage current_image_;
    bool has_image_;
    bool camera_online_;
    QString status_text_;
    mutable QMutex image_mutex_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_CAMERA_VIEW_WIDGET_H
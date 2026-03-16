#ifndef ROBOT_MONITOR_CAMERA_WORKER_H
#define ROBOT_MONITOR_CAMERA_WORKER_H

#include <atomic>
#include <string>

#include <QObject>
#include <QImage>

#include <ros/ros.h>
#include <image_transport/image_transport.h>
#include <sensor_msgs/Image.h>

namespace robot_monitor
{

class CameraWorker : public QObject
{
    Q_OBJECT

public:
    explicit CameraWorker(QObject* parent = nullptr);
    ~CameraWorker();

    void configure(const std::string& topic_name);

public slots:
    void start();
    void stop();

signals:
    void imageReady(const QImage& image);
    void cameraStatusChanged(bool online, const QString& status_text);
    void rosLogMessage(const QString& message);

private:
    void imageCallback(const sensor_msgs::ImageConstPtr& msg);

private:
    std::string topic_name_;
    std::atomic<bool> running_;

    ros::NodeHandle* nh_;
    image_transport::ImageTransport* image_transport_;
    image_transport::Subscriber image_sub_;

    ros::WallTime last_emit_time_;
    double min_emit_interval_sec_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_CAMERA_WORKER_H
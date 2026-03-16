#include "robot_monitor/camera_worker.h"

#include <cv_bridge/cv_bridge.h>
#include <sensor_msgs/image_encodings.h>

#include <QDateTime>

namespace robot_monitor
{

CameraWorker::CameraWorker(QObject* parent)
    : QObject(parent),
      running_(false),
      nh_(nullptr),
      image_transport_(nullptr),
      last_emit_time_(ros::WallTime(0)),
      min_emit_interval_sec_(1.0 / 12.0)
{
}

CameraWorker::~CameraWorker()
{
    stop();
}

void CameraWorker::configure(const std::string& topic_name)
{
    topic_name_ = topic_name;
}

void CameraWorker::start()
{
    if (running_)
    {
        return;
    }

    running_ = true;

    if (nh_ == nullptr)
    {
        nh_ = new ros::NodeHandle();
    }

    if (image_transport_ == nullptr)
    {
        image_transport_ = new image_transport::ImageTransport(*nh_);
    }

    image_sub_ = image_transport_->subscribe(
        topic_name_, 1, &CameraWorker::imageCallback, this);

    emit cameraStatusChanged(false, QString("Waiting for %1 ...").arg(QString::fromStdString(topic_name_)));
    emit rosLogMessage(QString("[INFO] Camera worker started, subscribing to %1")
                           .arg(QString::fromStdString(topic_name_)));
}

void CameraWorker::stop()
{
    if (!running_)
    {
        return;
    }

    running_ = false;

    image_sub_.shutdown();

    if (image_transport_ != nullptr)
    {
        delete image_transport_;
        image_transport_ = nullptr;
    }

    if (nh_ != nullptr)
    {
        delete nh_;
        nh_ = nullptr;
    }

    emit cameraStatusChanged(false, "Camera stopped");
    emit rosLogMessage("[INFO] Camera worker stopped.");
}

void CameraWorker::imageCallback(const sensor_msgs::ImageConstPtr& msg)
{
    if (!running_)
    {
        return;
    }

    const ros::WallTime now = ros::WallTime::now();
    if ((now - last_emit_time_).toSec() < min_emit_interval_sec_)
    {
        return;
    }

    try
    {
        cv_bridge::CvImageConstPtr cv_ptr;

        if (msg->encoding == sensor_msgs::image_encodings::BGR8)
        {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::BGR8);
            const cv::Mat& bgr = cv_ptr->image;

            QImage image(bgr.data,
                         bgr.cols,
                         bgr.rows,
                         static_cast<int>(bgr.step),
                         QImage::Format_RGB888);

            emit imageReady(image.rgbSwapped().copy());
        }
        else if (msg->encoding == sensor_msgs::image_encodings::RGB8)
        {
            cv_ptr = cv_bridge::toCvShare(msg, sensor_msgs::image_encodings::RGB8);
            const cv::Mat& rgb = cv_ptr->image;

            QImage image(rgb.data,
                         rgb.cols,
                         rgb.rows,
                         static_cast<int>(rgb.step),
                         QImage::Format_RGB888);

            emit imageReady(image.copy());
        }
        else
        {
            cv_ptr = cv_bridge::toCvCopy(msg, sensor_msgs::image_encodings::BGR8);
            const cv::Mat& bgr = cv_ptr->image;

            QImage image(bgr.data,
                         bgr.cols,
                         bgr.rows,
                         static_cast<int>(bgr.step),
                         QImage::Format_RGB888);

            emit imageReady(image.rgbSwapped().copy());
        }

        emit cameraStatusChanged(true, QString("Camera Online: %1").arg(QString::fromStdString(topic_name_)));
        last_emit_time_ = now;
    }
    catch (const cv_bridge::Exception& e)
    {
        emit cameraStatusChanged(false, QString("cv_bridge error"));
        emit rosLogMessage(QString("[WARN] cv_bridge exception: %1").arg(e.what()));
    }
    catch (const std::exception& e)
    {
        emit cameraStatusChanged(false, QString("Camera decode error"));
        emit rosLogMessage(QString("[WARN] Camera exception: %1").arg(e.what()));
    }
}

}  // namespace robot_monitor
#include "robot_monitor/camera_view_widget.h"

#include <QMutexLocker>
#include <QPainter>
#include <QPaintEvent>

namespace robot_monitor
{

CameraViewWidget::CameraViewWidget(QWidget* parent)
    : QWidget(parent),
      has_image_(false),
      camera_online_(false),
      status_text_("No Signal")
{
    setMinimumSize(320, 240);
    setAutoFillBackground(true);
}

CameraViewWidget::~CameraViewWidget()
{
}

void CameraViewWidget::setImage(const QImage& image)
{
    {
        QMutexLocker locker(&image_mutex_);
        current_image_ = image.copy();
        has_image_ = !current_image_.isNull();
        if (has_image_)
        {
            camera_online_ = true;
            status_text_ = "Camera Online";
        }
    }

    update();
}

void CameraViewWidget::setStatus(bool online, const QString& status_text)
{
    {
        QMutexLocker locker(&image_mutex_);
        camera_online_ = online;
        status_text_ = status_text;
    }

    update();
}

void CameraViewWidget::paintEvent(QPaintEvent* event)
{
    Q_UNUSED(event);

    QPainter painter(this);
    painter.setRenderHint(QPainter::Antialiasing, true);
    painter.fillRect(rect(), QColor(20, 24, 30));

    QImage image_copy;
    bool has_image = false;
    bool online = false;
    QString status;

    {
        QMutexLocker locker(&image_mutex_);
        image_copy = current_image_;
        has_image = has_image_;
        online = camera_online_;
        status = status_text_;
    }

    if (has_image && !image_copy.isNull())
    {
        QImage scaled = image_copy.scaled(size(), Qt::KeepAspectRatio, Qt::SmoothTransformation);

        const int x = (width() - scaled.width()) / 2;
        const int y = (height() - scaled.height()) / 2;
        painter.drawImage(QPoint(x, y), scaled);
    }
    else
    {
        painter.setPen(QPen(QColor(255, 90, 90), 3));
        painter.drawRect(rect().adjusted(10, 10, -10, -10));
        painter.drawLine(20, 20, width() - 20, height() - 20);
        painter.drawLine(width() - 20, 20, 20, height() - 20);

        painter.setPen(Qt::white);
        painter.drawText(rect(), Qt::AlignCenter, "No Signal");
    }

    painter.setPen(Qt::white);
    painter.drawText(10, 20, status);

    painter.setPen(online ? QColor(80, 220, 120) : QColor(255, 90, 90));
    painter.setBrush(online ? QColor(80, 220, 120) : QColor(255, 90, 90));
    painter.drawEllipse(QPointF(width() - 18, 18), 6, 6);
}

}  // namespace robot_monitor
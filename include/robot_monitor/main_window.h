#ifndef ROBOT_MONITOR_MAIN_WINDOW_H
#define ROBOT_MONITOR_MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>

#include "robot_monitor/ros_interface.h"
#include "robot_monitor/map_view_widget.h"

class QLabel;
class QTextEdit;

namespace robot_monitor
{

class MainWindow : public QMainWindow
{
    Q_OBJECT

public:
    explicit MainWindow(ros::NodeHandle& nh, QWidget* parent = nullptr);
    ~MainWindow();

private slots:
    void onRosSpinOnce();
    void onUpdateUi();

private:
    void setupUi();
    void setupMenuBar();
    void setupToolBar();
    void setupCentralView();
    void setupDockWidgets();
    void setupStatusBar();

private:
    ros::NodeHandle nh_;
    RosInterface ros_interface_;
    std::string odom_topic_;

    QTimer* ros_timer_;
    QTimer* ui_timer_;

    QWidget* central_widget_;
    MapViewWidget* map_view_widget_;

    QLabel* label_pose_;
    QLabel* label_velocity_;
    QLabel* label_battery_;
    QLabel* label_system_;

    QTextEdit* log_text_edit_;
    bool odom_logged_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_MAIN_WINDOW_H
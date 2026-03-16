#ifndef ROBOT_MONITOR_MAIN_WINDOW_H
#define ROBOT_MONITOR_MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QPoint>

#include <ros/subscriber.h>
#include <std_msgs/String.h>

#include "robot_monitor/ros_interface.h"
#include "robot_monitor/map_view_widget.h"
#include "robot_monitor/trajectory_recorder.h"
#include "robot_monitor/experiment_config.h"
#include "robot_monitor/trajectory_storage.h"

#include <QThread>
#include "robot_monitor/camera_view_widget.h"
#include "robot_monitor/camera_worker.h"
#include "robot_monitor/metrics_panel_widget.h"

class QLabel;
class QTextEdit;
class QListWidget;
class QListWidgetItem;
class QPushButton;
class QMenu;
class QToolBar;
class QDockWidget;

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
    void onMethodSelectionChanged();
    void onTrajectorySelectionChanged();

private:
    void setupUi();
    void setupMenuBar();
    void setupToolBar();
    void setupCentralView();
    void setupDockWidgets();
    void setupStatusBar();
    void handleEpisodeStart();
    void handleEpisodeEnd();
    void episodeEventCallback(const std_msgs::String::ConstPtr& msg);
    void refreshTrajectoryPanel();
    void updateTrajectoryListForSelectedMethod(int preferred_trajectory_id = -1);
    void loadSelectedTrajectoryMeta();
    void setupCameraThread();
    void stopCameraThread();    
    void showMethodContextMenu(const QPoint& pos);
    void showTrajectoryContextMenu(const QPoint& pos);
    void deleteSelectedMethod();
    void deleteSelectedTrajectory();

private:
    ros::NodeHandle nh_;
    RosInterface ros_interface_;
    std::string odom_topic_;

    ros::Subscriber episode_event_sub_;
    std::string episode_event_topic_;

    ExperimentConfig experiment_config_;
    TrajectoryRecorder trajectory_recorder_;
    TrajectoryStorage trajectory_storage_;
    std::string database_path_;

    QMenu* view_menu_;

    // int episode_counter_;

    QTimer* ros_timer_;
    QTimer* ui_timer_;

    QWidget* central_widget_;
    MapViewWidget* map_view_widget_;

    // QLabel* label_pose_;
    // QLabel* label_velocity_;
    // QLabel* label_battery_;
    // QLabel* label_system_;
    MetricsPanelWidget* metrics_panel_widget_;

    QTextEdit* log_text_edit_;

    QListWidget* method_list_widget_;
    QListWidget* trajectory_list_widget_;
    QLabel* trajectory_panel_status_label_;
    QPushButton* button_refresh_trajectories_;
    QPushButton* button_load_trajectory_;

    std::vector<TrajectoryRecord> cached_trajectory_records_;
    std::vector<std::string> cached_method_names_;

    bool odom_logged_;

    QToolBar* main_tool_bar_;

    QDockWidget* metrics_dock_;
    QDockWidget* log_dock_;
    QDockWidget* playback_dock_;
    QDockWidget* trajectory_dock_;

    std::string camera_topic_;

    QThread* camera_thread_;
    CameraWorker* camera_worker_;
    CameraViewWidget* camera_view_widget_;
    QDockWidget* camera_dock_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_MAIN_WINDOW_H
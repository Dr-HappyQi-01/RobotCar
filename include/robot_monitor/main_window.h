#ifndef ROBOT_MONITOR_MAIN_WINDOW_H
#define ROBOT_MONITOR_MAIN_WINDOW_H

#include <QMainWindow>
#include <QTimer>
#include <QPoint>
#include <QComboBox>
#include <QPushButton>
#include <QVBoxLayout>
#include <ros/subscriber.h>
#include <std_msgs/String.h>
#include <QProcess>

#include "robot_monitor/ros_interface.h"
#include "robot_monitor/map_view_widget.h"
#include "robot_monitor/trajectory_recorder.h"
#include "robot_monitor/experiment_config.h"
#include "robot_monitor/trajectory_storage.h"
#include "robot_monitor/app_config.h"

#include <QThread>
#include "robot_monitor/camera_view_widget.h"
#include "robot_monitor/camera_worker.h"
#include "robot_monitor/metrics_panel_widget.h"
#include "robot_monitor/map_file_loader.h"
#include "robot_monitor/reward_curve_widget.h"


class QLabel;
class QTextEdit;
class QListWidget;
class QListWidgetItem;
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
    void onMapModeChanged(int index);
    void onStartSlamClicked();
    void onFinishSlamClicked();
    void onImportMapClicked();
    void onClearMapClicked();
    void onSlamProcessStarted();
    void onSlamProcessFinished(int exit_code, QProcess::ExitStatus exit_status);
    void onSlamProcessErrorOccurred(QProcess::ProcessError error);

private:
    enum  MapMode
{
    LiveSlam = 0,
    ImportMap = 1,
    RawGrid = 2
};
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
    void setupMapControlBar(QVBoxLayout* parent_layout);
    void updateMapModeUi();
    void setMapMode(MapMode mode);
    void startSlamProcess();
    void runSaveMapProcess();
    void stopSlamProcess();
    void appendProcessOutput(QProcess* process, const QString& prefix);
    bool promptOverlayCornerCoordinates(ImageOverlayData& overlay_data);
    void appendLogMessage(const QString& level, const QString& message);
    void appendAutoLogMessage(const QString& text);



private:
    ros::NodeHandle nh_;
    RosInterface ros_interface_;
    std::string odom_topic_;

    ros::Subscriber episode_event_sub_;
    std::string episode_event_topic_;

    ExperimentConfig experiment_config_;
    AppConfig app_config_;
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
    QDockWidget* reward_dock_;
    RewardCurveWidget* reward_curve_widget_;
    QDockWidget* trajectory_dock_;

    std::string camera_topic_;

    QThread* camera_thread_;
    CameraWorker* camera_worker_;
    CameraViewWidget* camera_view_widget_;
    QDockWidget* camera_dock_;


    MapMode current_map_mode_;
    bool slam_running_;

    QWidget* map_control_bar_widget_;
    QComboBox* combo_map_mode_;
    QPushButton* button_start_slam_;
    QPushButton* button_finish_slam_;
    QPushButton* button_import_map_;
    QPushButton* button_clear_map_;

    QProcess* slam_process_;
    QProcess* save_map_process_;
    QProcess* stop_slam_process_;


    MapFileLoader map_file_loader_;
    GridMapData imported_map_data_;
    bool imported_map_valid_;
    ImageOverlayData imported_overlay_data_;
    bool imported_overlay_valid_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_MAIN_WINDOW_H
#include "robot_monitor/main_window.h"
#include "robot_monitor/experiment_config.h"

#include <ros/ros.h>

#include <QAction>
#include <QDockWidget>
#include <QFrame>
#include <QLabel>
#include <QMenu>
#include <QMenuBar>
#include <QStatusBar>
#include <QTextEdit>
#include <QToolBar>
#include <QVBoxLayout>
#include <QWidget>
#include <QListWidget>
#include <QListWidgetItem>
#include <QPushButton>
#include <QHBoxLayout>
#include <QMap>
#include <QSet>

namespace robot_monitor
{

MainWindow::MainWindow(ros::NodeHandle& nh, QWidget* parent)
    : QMainWindow(parent),
      nh_(nh),
      ros_timer_(new QTimer(this)),
      ui_timer_(new QTimer(this)),
      central_widget_(nullptr),
      map_view_widget_(nullptr),
      label_pose_(nullptr),
      label_velocity_(nullptr),
      label_battery_(nullptr),
      label_system_(nullptr),
      log_text_edit_(nullptr),
      method_list_widget_(nullptr),
      trajectory_list_widget_(nullptr),
      trajectory_panel_status_label_(nullptr),
      button_refresh_trajectories_(nullptr),
      button_load_trajectory_(nullptr),
      odom_logged_(false),
      view_menu_(nullptr),
      main_tool_bar_(nullptr),
      metrics_dock_(nullptr),
      log_dock_(nullptr),
      playback_dock_(nullptr),
      trajectory_dock_(nullptr)
      
{
    ros_interface_.init(nh_, "/robot/robotnik_base_control/odom");
    nh_.param<std::string>("episode_event_topic", episode_event_topic_, "/experiment/episode_event");
    ROS_INFO_STREAM("robot_monitor subscribing to episode event topic: " << episode_event_topic_);

    episode_event_sub_ = nh_.subscribe(
        episode_event_topic_, 10, &MainWindow::episodeEventCallback, this);

    ExperimentConfig config;
    ExperimentConfigLoader config_loader;
    std::string error_message;

    const std::string config_path = "/home/s/catkin_ws/src/robot_monitor/config/experiment.json";

    if (config_loader.loadFromFile(config_path, config, error_message))
    {
        ROS_INFO_STREAM("Loaded experiment config, method_name = " << config.method_name);
    }
    else
    {
        ROS_WARN_STREAM("Failed to load experiment config: " << error_message);
    }


    if (config_loader.loadFromFile(config_path, experiment_config_, error_message))
    {
        ROS_INFO_STREAM("Loaded experiment config, method_name = " << experiment_config_.method_name);
    }
    else
    {
        ROS_WARN_STREAM("Failed to load experiment config: " << error_message);
        experiment_config_.method_name = "未命名方法";
    }

    database_path_ = "/home/s/catkin_ws/src/robot_monitor/data/trajectory.db";

    std::string db_error;
    if (trajectory_storage_.open(database_path_, db_error))
    {
        if (trajectory_storage_.initTables(db_error))
        {
            ROS_INFO_STREAM("Trajectory database ready: " << database_path_);
        }
        else
        {
            ROS_WARN_STREAM("Failed to init trajectory tables: " << db_error);
        }
    }
    else
    {
        ROS_WARN_STREAM("Failed to open trajectory database: " << db_error);
    }

    setupUi();
    refreshTrajectoryPanel();

    connect(ros_timer_, &QTimer::timeout, this, &MainWindow::onRosSpinOnce);
    connect(ui_timer_, &QTimer::timeout, this, &MainWindow::onUpdateUi);

    ros_timer_->start(20);
    ui_timer_->start(100);
}

MainWindow::~MainWindow()
{
}

void MainWindow::episodeEventCallback(const std_msgs::String::ConstPtr& msg)
{
    const std::string event = msg->data;

    ROS_INFO_STREAM("Received episode event: " << event);

    if (event == "start")
    {
        handleEpisodeStart();
    }
    else if (event == "end")
    {
        handleEpisodeEnd();
    }
    else
    {
        ROS_WARN_STREAM("Unknown episode event: " << event);

        if (log_text_edit_)
        {
            log_text_edit_->append(
                QString("[WARN] Unknown episode event: %1")
                    .arg(QString::fromStdString(event)));
        }
    }
}

void MainWindow::handleEpisodeStart()
{
    if (trajectory_recorder_.isRecording())
    {
        if (log_text_edit_)
        {
            log_text_edit_->append("[WARN] Episode already recording.");
        }
        return;
    }

    std::string error_message;
    const int next_index = trajectory_storage_.getNextEpisodeIndex(
        experiment_config_.method_name, error_message);

    if (next_index <= 0)
    {
        if (log_text_edit_)
        {
            log_text_edit_->append(
                QString("[WARN] Failed to get next episode index: %1")
                    .arg(QString::fromStdString(error_message)));
        }
        return;
    }

    const bool ok = trajectory_recorder_.startRecording(
        experiment_config_.method_name, next_index);

    if (ok)
    {
        if (map_view_widget_)
        {
            map_view_widget_->clearTrajectory();
            map_view_widget_->clearSelectedTrajectory();
        }

        if (log_text_edit_)
        {
            log_text_edit_->append(
                QString("[INFO] Episode started: %1")
                    .arg(QString::fromStdString(trajectory_recorder_.currentTrajectoryName())));
        }
    }
    else
    {
        if (log_text_edit_)
        {
            log_text_edit_->append("[WARN] Failed to start episode.");
        }
    }
}

void MainWindow::handleEpisodeEnd()
{
    TrajectoryRecord finished_record;
    const bool ok = trajectory_recorder_.stopRecording(finished_record);

    if (!ok)
    {
        if (log_text_edit_)
        {
            log_text_edit_->append("[WARN] No active episode to stop.");
        }
        return;
    }

    std::string error_message;
    if (trajectory_storage_.saveTrajectory(finished_record, error_message))
    {
        if (log_text_edit_)
        {
            log_text_edit_->append(
                QString("[INFO] Episode saved: %1, points=%2")
                    .arg(QString::fromStdString(finished_record.name))
                    .arg(static_cast<int>(finished_record.points.size())));
        }
        refreshTrajectoryPanel();
    }
    else
    {
        if (log_text_edit_)
        {
            log_text_edit_->append(
                QString("[WARN] Failed to save trajectory: %1")
                    .arg(QString::fromStdString(error_message)));
        }
    }
}

void MainWindow::setupUi()
{
    resize(1400, 900);
    setWindowTitle("Robot Monitor");

    setupMenuBar();
    setupToolBar();
    setupCentralView();
    setupDockWidgets();
    setupStatusBar();
}

void MainWindow::setupMenuBar()
{
    QMenu* file_menu = menuBar()->addMenu("File");
    view_menu_ = menuBar()->addMenu("View");
    QMenu* help_menu = menuBar()->addMenu("Help");

    QAction* action_exit = new QAction("Exit", this);
    connect(action_exit, &QAction::triggered, this, &QWidget::close);
    file_menu->addAction(action_exit);

    QAction* action_about = new QAction("About", this);
    help_menu->addAction(action_about);
}

void MainWindow::setupToolBar()
{
    main_tool_bar_ = addToolBar("MainToolBar");
    main_tool_bar_->setMovable(false);

    QAction* action_start = new QAction("Start", this);
    QAction* action_stop = new QAction("Stop", this);
    QAction* action_estop = new QAction("E-Stop", this);
    QAction* action_reset = new QAction("Reset", this);
    QAction* action_clear_traj = new QAction("Clear Traj", this);
    QAction* action_episode_start = new QAction("Episode Start", this);
    QAction* action_episode_end = new QAction("Episode End", this);

    main_tool_bar_->addAction(action_start);
    main_tool_bar_->addAction(action_stop);
    main_tool_bar_->addSeparator();
    main_tool_bar_->addAction(action_estop);
    main_tool_bar_->addAction(action_reset);
    main_tool_bar_->addSeparator();
    main_tool_bar_->addAction(action_clear_traj);
    main_tool_bar_->addSeparator();
    main_tool_bar_->addAction(action_episode_start);
    main_tool_bar_->addAction(action_episode_end);

    connect(action_clear_traj, &QAction::triggered, this, [this]() {
        if (map_view_widget_)
        {
            map_view_widget_->clearTrajectory();
        }

        if (log_text_edit_)
        {
            log_text_edit_->append("[INFO] Trajectory cleared.");
        }
    });

    connect(action_episode_start, &QAction::triggered, this, [this]() {
        handleEpisodeStart();
    });

    connect(action_episode_end, &QAction::triggered, this, [this]() {
        handleEpisodeEnd();
    });
}

void MainWindow::setupCentralView()
{
    central_widget_ = new QWidget(this);
    setCentralWidget(central_widget_);

    QVBoxLayout* layout = new QVBoxLayout(central_widget_);
    layout->setContentsMargins(8, 8, 8, 8);
    layout->setSpacing(8);

    map_view_widget_ = new MapViewWidget(this);
    layout->addWidget(map_view_widget_);
}

void MainWindow::setupDockWidgets()
{
    // =========================
    // Real-time Metrics
    // =========================
    metrics_dock_ = new QDockWidget("Real-time Metrics", this);
    metrics_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    metrics_dock_->setFeatures(QDockWidget::DockWidgetClosable |
                               QDockWidget::DockWidgetMovable |
                               QDockWidget::DockWidgetFloatable);

    QWidget* metrics_widget = new QWidget(metrics_dock_);
    QVBoxLayout* metrics_layout = new QVBoxLayout(metrics_widget);

    label_pose_ = new QLabel("Pose: waiting for odom ...", metrics_widget);
    label_velocity_ = new QLabel("Velocity: waiting for odom ...", metrics_widget);
    label_battery_ = new QLabel("Battery: N/A", metrics_widget);
    label_system_ = new QLabel("System: waiting for odom / episode event", metrics_widget);

    metrics_layout->addWidget(label_pose_);
    metrics_layout->addWidget(label_velocity_);
    metrics_layout->addWidget(label_battery_);
    metrics_layout->addWidget(label_system_);
    metrics_layout->addStretch();

    metrics_dock_->setWidget(metrics_widget);
    addDockWidget(Qt::RightDockWidgetArea, metrics_dock_);

    // =========================
    // Data Log
    // =========================
    log_dock_ = new QDockWidget("Data Log", this);
    log_dock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    log_dock_->setFeatures(QDockWidget::DockWidgetClosable |
                           QDockWidget::DockWidgetMovable |
                           QDockWidget::DockWidgetFloatable);

    log_text_edit_ = new QTextEdit(log_dock_);
    log_text_edit_->setReadOnly(true);
    log_text_edit_->append("[INFO] Robot monitor started.");
    log_text_edit_->append("[INFO] Subscribing to odom topic ...");
    log_text_edit_->append("[INFO] Waiting for episode event topic ...");

    log_dock_->setWidget(log_text_edit_);
    addDockWidget(Qt::BottomDockWidgetArea, log_dock_);

    // =========================
    // Trajectory Playback
    // =========================
    playback_dock_ = new QDockWidget("Trajectory Playback", this);
    playback_dock_->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);
    playback_dock_->setFeatures(QDockWidget::DockWidgetClosable |
                                QDockWidget::DockWidgetMovable |
                                QDockWidget::DockWidgetFloatable);

    QWidget* playback_widget = new QWidget(playback_dock_);
    QVBoxLayout* playback_layout = new QVBoxLayout(playback_widget);

    QLabel* playback_label = new QLabel("Playback controls placeholder", playback_widget);
    playback_label->setAlignment(Qt::AlignCenter);

    playback_layout->addWidget(playback_label);
    playback_dock_->setWidget(playback_widget);

    addDockWidget(Qt::BottomDockWidgetArea, playback_dock_);
    splitDockWidget(log_dock_, playback_dock_, Qt::Horizontal);

    // =========================
    // Trajectory Records
    // =========================
    trajectory_dock_ = new QDockWidget("Trajectory Records", this);
    trajectory_dock_->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);
    trajectory_dock_->setFeatures(QDockWidget::DockWidgetClosable |
                                  QDockWidget::DockWidgetMovable |
                                  QDockWidget::DockWidgetFloatable);

    QWidget* trajectory_panel = new QWidget(trajectory_dock_);
    QVBoxLayout* trajectory_layout = new QVBoxLayout(trajectory_panel);
    trajectory_layout->setContentsMargins(10, 10, 10, 10);
    trajectory_layout->setSpacing(8);

    QLabel* methods_title = new QLabel("Methods", trajectory_panel);
    methods_title->setStyleSheet("font-weight: bold; font-size: 14px;");

    method_list_widget_ = new QListWidget(trajectory_panel);
    method_list_widget_->setMinimumHeight(140);

    QLabel* traj_title = new QLabel("Trajectories", trajectory_panel);
    traj_title->setStyleSheet("font-weight: bold; font-size: 14px;");

    trajectory_list_widget_ = new QListWidget(trajectory_panel);
    trajectory_list_widget_->setMinimumHeight(220);

    trajectory_panel_status_label_ = new QLabel("No trajectory data loaded.", trajectory_panel);
    trajectory_panel_status_label_->setWordWrap(true);

    QHBoxLayout* button_layout = new QHBoxLayout();
    button_refresh_trajectories_ = new QPushButton("Refresh", trajectory_panel);
    button_load_trajectory_ = new QPushButton("Load", trajectory_panel);

    button_layout->addWidget(button_refresh_trajectories_);
    button_layout->addWidget(button_load_trajectory_);

    trajectory_layout->addWidget(methods_title);
    trajectory_layout->addWidget(method_list_widget_);
    trajectory_layout->addWidget(traj_title);
    trajectory_layout->addWidget(trajectory_list_widget_);
    trajectory_layout->addWidget(trajectory_panel_status_label_);
    trajectory_layout->addLayout(button_layout);

    trajectory_dock_->setWidget(trajectory_panel);
    addDockWidget(Qt::LeftDockWidgetArea, trajectory_dock_);

    // =========================
    // Signals / slots
    // =========================
    connect(method_list_widget_, &QListWidget::itemSelectionChanged,
            this, &MainWindow::onMethodSelectionChanged);

    connect(trajectory_list_widget_, &QListWidget::itemSelectionChanged,
            this, &MainWindow::onTrajectorySelectionChanged);

    connect(button_refresh_trajectories_, &QPushButton::clicked, this, [this]() {
        refreshTrajectoryPanel();
    });

    connect(button_load_trajectory_, &QPushButton::clicked, this, [this]() {
        loadSelectedTrajectoryMeta();
    });

    // =========================
    // View menu toggle actions
    // =========================
    if (view_menu_)
    {
        view_menu_->clear();

        if (main_tool_bar_)
        {
            QAction* toolbar_action = main_tool_bar_->toggleViewAction();
            toolbar_action->setText("Main Toolbar");
            view_menu_->addAction(toolbar_action);
        }

        view_menu_->addSeparator();

        if (metrics_dock_)
        {
            QAction* action = metrics_dock_->toggleViewAction();
            action->setText("Real-time Metrics");
            view_menu_->addAction(action);
        }

        if (log_dock_)
        {
            QAction* action = log_dock_->toggleViewAction();
            action->setText("Data Log");
            view_menu_->addAction(action);
        }

        if (playback_dock_)
        {
            QAction* action = playback_dock_->toggleViewAction();
            action->setText("Trajectory Playback");
            view_menu_->addAction(action);
        }

        if (trajectory_dock_)
        {
            QAction* action = trajectory_dock_->toggleViewAction();
            action->setText("Trajectory Records");
            view_menu_->addAction(action);
        }
    }
}

void MainWindow::setupStatusBar()
{
    statusBar()->showMessage("Waiting for /odom ...");
}

void MainWindow::onRosSpinOnce()
{
    if (ros::ok())
    {
        ros::spinOnce();
    }
}

void MainWindow::onUpdateUi()
{
    const OdomData odom = ros_interface_.getOdomData();

    map_view_widget_->setRobotPose(odom.x, odom.y, odom.yaw, odom.received);

    if (!odom.received)
    {
        label_pose_->setText("Pose: waiting for odom ...");
        label_velocity_->setText("Velocity: waiting for odom ...");
        label_system_->setText("System: no odom received");
        statusBar()->showMessage(QString("Waiting for %1 ...").arg(QString::fromStdString(odom_topic_)));
        return;
    }

    if (trajectory_recorder_.isRecording())
    {
        TrajectoryPoint point;
        point.x = odom.x;
        point.y = odom.y;
        point.yaw = odom.yaw;
        point.linear_velocity = odom.linear_velocity;
        point.angular_velocity = odom.angular_velocity;
        point.timestamp = ros::Time::now().toSec();

        trajectory_recorder_.appendPoint(point);
    }

    label_pose_->setText(
        QString("Pose: x=%1, y=%2, yaw=%3 rad")
            .arg(odom.x, 0, 'f', 3)
            .arg(odom.y, 0, 'f', 3)
            .arg(odom.yaw, 0, 'f', 3));

    label_velocity_->setText(
        QString("Velocity: v=%1 m/s, w=%2 rad/s")
            .arg(odom.linear_velocity, 0, 'f', 3)
            .arg(odom.angular_velocity, 0, 'f', 3));

    if (trajectory_recorder_.isRecording())
    {
        label_system_->setText(
            QString("System: recording [%1]")
                .arg(QString::fromStdString(trajectory_recorder_.currentTrajectoryName())));
    }
    else
    {
        label_system_->setText("System: odom online");
    }

    statusBar()->showMessage("ROS running: odom connected");

    if (!odom_logged_)
    {
        log_text_edit_->append("[INFO] First odom message received.");
        odom_logged_ = true;
    }
}
void MainWindow::refreshTrajectoryPanel()
{
    if (!method_list_widget_ || !trajectory_list_widget_ || !trajectory_panel_status_label_)
    {
        return;
    }

    // 先记住当前选择
    QString previous_method;
    int previous_trajectory_id = -1;

    if (method_list_widget_->currentItem())
    {
        previous_method = method_list_widget_->currentItem()->text();
    }

    if (trajectory_list_widget_->currentItem())
    {
        previous_trajectory_id = trajectory_list_widget_->currentItem()->data(Qt::UserRole).toInt();
    }

    cached_trajectory_records_.clear();
    cached_method_names_.clear();

    std::string error_message;
    if (!trajectory_storage_.listTrajectories(cached_trajectory_records_, error_message))
    {
        trajectory_panel_status_label_->setText(
            QString("Failed to load trajectories: %1")
                .arg(QString::fromStdString(error_message)));

        if (log_text_edit_)
        {
            log_text_edit_->append(
                QString("[WARN] Failed to load trajectory list: %1")
                    .arg(QString::fromStdString(error_message)));
        }
        return;
    }

    method_list_widget_->clear();
    trajectory_list_widget_->clear();

    QSet<QString> unique_methods;
    for (const auto& record : cached_trajectory_records_)
    {
        unique_methods.insert(QString::fromStdString(record.method_name));
    }

    QList<QString> methods = unique_methods.values();
    std::sort(methods.begin(), methods.end());

    int preferred_method_row = -1;

    for (int i = 0; i < methods.size(); ++i)
    {
        const QString& method = methods[i];
        method_list_widget_->addItem(method);
        cached_method_names_.push_back(method.toStdString());

        if (method == previous_method)
        {
            preferred_method_row = i;
        }
    }

    trajectory_panel_status_label_->setText(
        QString("%1 methods / %2 trajectories")
            .arg(method_list_widget_->count())
            .arg(static_cast<int>(cached_trajectory_records_.size())));

    if (method_list_widget_->count() > 0)
    {
        if (preferred_method_row >= 0)
        {
            method_list_widget_->setCurrentRow(preferred_method_row);
        }
        else
        {
            method_list_widget_->setCurrentRow(0);
        }

        updateTrajectoryListForSelectedMethod(previous_trajectory_id);
    }
}
void MainWindow::updateTrajectoryListForSelectedMethod(int preferred_trajectory_id)
{
    if (!method_list_widget_ || !trajectory_list_widget_ || !trajectory_panel_status_label_)
    {
        return;
    }

    trajectory_list_widget_->clear();

    QListWidgetItem* current_method_item = method_list_widget_->currentItem();
    if (!current_method_item)
    {
        trajectory_panel_status_label_->setText("No method selected.");
        return;
    }

    const QString selected_method = current_method_item->text();
    int count = 0;
    int preferred_row = -1;

    for (const auto& record : cached_trajectory_records_)
    {
        if (QString::fromStdString(record.method_name) != selected_method)
        {
            continue;
        }

        QString display_text = QString("轨迹%1").arg(record.episode_index);

        QListWidgetItem* item = new QListWidgetItem(display_text, trajectory_list_widget_);
        item->setData(Qt::UserRole, record.id);
        item->setToolTip(QString::fromStdString(record.name));

        trajectory_list_widget_->addItem(item);

        if (record.id == preferred_trajectory_id)
        {
            preferred_row = count;
        }

        ++count;
    }

    trajectory_panel_status_label_->setText(
        QString("Method [%1] contains %2 trajectories")
            .arg(selected_method)
            .arg(count));

    if (trajectory_list_widget_->count() > 0)
    {
        if (preferred_row >= 0)
        {
            trajectory_list_widget_->setCurrentRow(preferred_row);
        }
        else
        {
            trajectory_list_widget_->setCurrentRow(0);
        }
    }
}

void MainWindow::loadSelectedTrajectoryMeta()
{
    if (!trajectory_list_widget_ || !map_view_widget_)
    {
        return;
    }

    QListWidgetItem* current_item = trajectory_list_widget_->currentItem();
    if (!current_item)
    {
        if (log_text_edit_)
        {
            log_text_edit_->append("[WARN] No trajectory selected.");
        }
        return;
    }

    const int trajectory_id = current_item->data(Qt::UserRole).toInt();

    std::string error_message;
    TrajectoryRecord record;
    if (!trajectory_storage_.loadTrajectoryById(trajectory_id, record, error_message))
    {
        if (log_text_edit_)
        {
            log_text_edit_->append(
                QString("[WARN] Failed to load trajectory: %1")
                    .arg(QString::fromStdString(error_message)));
        }
        return;
    }

    map_view_widget_->setSelectedTrajectory(record);

    if (trajectory_panel_status_label_)
    {
        trajectory_panel_status_label_->setText(
            QString("Loaded [%1], points=%2")
                .arg(QString::fromStdString(record.name))
                .arg(static_cast<int>(record.points.size())));
    }

    if (log_text_edit_)
    {
        log_text_edit_->append(
            QString("[INFO] Loaded trajectory: %1, points=%2")
                .arg(QString::fromStdString(record.name))
                .arg(static_cast<int>(record.points.size())));
    }
}


// Slots

void MainWindow::onMethodSelectionChanged()
{
    updateTrajectoryListForSelectedMethod(-1);
}

void MainWindow::onTrajectorySelectionChanged()
{
    if (!trajectory_list_widget_ || !trajectory_panel_status_label_)
    {
        return;
    }

    QListWidgetItem* current_item = trajectory_list_widget_->currentItem();
    if (!current_item)
    {
        return;
    }

    const int trajectory_id = current_item->data(Qt::UserRole).toInt();
    const QString traj_name = current_item->text();

    trajectory_panel_status_label_->setText(
        QString("Selected trajectory: %1 (id=%2)")
            .arg(traj_name)
            .arg(trajectory_id));
}

}  // namespace robot_monitor
#include "robot_monitor/main_window.h"

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
      odom_logged_(false)
{
    ros_interface_.init(nh_, "/robot/robotnik_base_control/odom");

    setupUi();

    connect(ros_timer_, &QTimer::timeout, this, &MainWindow::onRosSpinOnce);
    connect(ui_timer_, &QTimer::timeout, this, &MainWindow::onUpdateUi);

    ros_timer_->start(20);
    ui_timer_->start(100);
}

MainWindow::~MainWindow()
{
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
    QMenu* view_menu = menuBar()->addMenu("View");
    QMenu* help_menu = menuBar()->addMenu("Help");
    Q_UNUSED(view_menu);

    QAction* action_exit = new QAction("Exit", this);
    connect(action_exit, &QAction::triggered, this, &QWidget::close);
    file_menu->addAction(action_exit);

    QAction* action_about = new QAction("About", this);
    help_menu->addAction(action_about);
}

void MainWindow::setupToolBar()
{
    QToolBar* tool_bar = addToolBar("MainToolBar");
    tool_bar->setMovable(false);

    QAction* action_start = new QAction("Start", this);
    QAction* action_stop = new QAction("Stop", this);
    QAction* action_estop = new QAction("E-Stop", this);
    QAction* action_reset = new QAction("Reset", this);

    tool_bar->addAction(action_start);
    tool_bar->addAction(action_stop);
    tool_bar->addSeparator();
    tool_bar->addAction(action_estop);
    tool_bar->addAction(action_reset);
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
    QDockWidget* metrics_dock = new QDockWidget("Real-time Metrics", this);
    metrics_dock->setAllowedAreas(Qt::LeftDockWidgetArea | Qt::RightDockWidgetArea);

    QWidget* metrics_widget = new QWidget(metrics_dock);
    QVBoxLayout* metrics_layout = new QVBoxLayout(metrics_widget);

    label_pose_ = new QLabel("Pose: waiting for /odom ...", metrics_widget);
    label_velocity_ = new QLabel("Velocity: waiting for /odom ...", metrics_widget);
    label_battery_ = new QLabel("Battery: N/A", metrics_widget);
    label_system_ = new QLabel("System: waiting for odom", metrics_widget);

    metrics_layout->addWidget(label_pose_);
    metrics_layout->addWidget(label_velocity_);
    metrics_layout->addWidget(label_battery_);
    metrics_layout->addWidget(label_system_);
    metrics_layout->addStretch();

    metrics_dock->setWidget(metrics_widget);
    addDockWidget(Qt::RightDockWidgetArea, metrics_dock);

    QDockWidget* log_dock = new QDockWidget("Data Log", this);
    log_dock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);

    log_text_edit_ = new QTextEdit(log_dock);
    log_text_edit_->setReadOnly(true);
    log_text_edit_->append("[INFO] Robot monitor started.");
    log_text_edit_->append("[INFO] Subscribing to /odom ...");

    log_dock->setWidget(log_text_edit_);
    addDockWidget(Qt::BottomDockWidgetArea, log_dock);

    QDockWidget* playback_dock = new QDockWidget("Trajectory Playback", this);
    playback_dock->setAllowedAreas(Qt::BottomDockWidgetArea | Qt::TopDockWidgetArea);

    QWidget* playback_widget = new QWidget(playback_dock);
    QVBoxLayout* playback_layout = new QVBoxLayout(playback_widget);

    QLabel* playback_label = new QLabel(
        "Playback controls placeholder",
        playback_widget);
    playback_label->setAlignment(Qt::AlignCenter);

    playback_layout->addWidget(playback_label);
    playback_dock->setWidget(playback_widget);

    addDockWidget(Qt::BottomDockWidgetArea, playback_dock);
    splitDockWidget(log_dock, playback_dock, Qt::Horizontal);
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

    label_pose_->setText(
        QString("Pose: x=%1, y=%2, yaw=%3 rad")
            .arg(odom.x, 0, 'f', 3)
            .arg(odom.y, 0, 'f', 3)
            .arg(odom.yaw, 0, 'f', 3));

    label_velocity_->setText(
        QString("Velocity: v=%1 m/s, w=%2 rad/s")
            .arg(odom.linear_velocity, 0, 'f', 3)
            .arg(odom.angular_velocity, 0, 'f', 3));

    label_system_->setText("System: odom online");
    statusBar()->showMessage("ROS running: odom connected");

    if (!odom_logged_)
    {
        log_text_edit_->append("[INFO] First odom message received.");
        odom_logged_ = true;
    }
}

}  // namespace robot_monitor
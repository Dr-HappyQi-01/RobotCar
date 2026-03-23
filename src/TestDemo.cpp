#include <ros/ros.h>
#include <geometry_msgs/Twist.h>
#include <nav_msgs/Odometry.h>
#include <std_msgs/String.h>
#include <tf/tf.h> 
#include <cmath>
#include <random>
#include <sqlite3.h>  // 添加SQLite头文件

// 全局变量：存储机器人当前的真实坐标和朝向
double current_x = 0.0;
double current_y = 0.0;
double current_yaw = 0.0;
bool odom_received = false;

// 添加全局变量用于数据库操作
sqlite3* db = nullptr;
int episode_counter = 1;  // 回合数计数器
std::random_device rd_reward;  // 随机奖励生成器
std::mt19937 gen_reward(rd_reward());
std::uniform_real_distribution<> reward_dis(-100.0, 100.0);  // -100到100的随机数分布

// 里程计回调函数：实时获取机器人位置
void odomCallback(const nav_msgs::Odometry::ConstPtr& msg) {
    current_x = msg->pose.pose.position.x;
    current_y = msg->pose.pose.position.y;
    current_yaw = tf::getYaw(msg->pose.pose.orientation);
    odom_received = true;
}

// 数据库初始化函数
bool initDatabase() {
    const char* db_path = "/home/s/catkin_ws/src/robot_monitor/data/trajectory.db";
    int rc = sqlite3_open(db_path, &db);
    
    if (rc != SQLITE_OK) {
        ROS_ERROR("Cannot open database: %s", sqlite3_errmsg(db));
        return false;
    }
    
    // 创建表（如果不存在）
    const char* create_table_sql = R"(
        CREATE TABLE IF NOT EXISTS method_episode_rewards (
            id INTEGER PRIMARY KEY AUTOINCREMENT, 
            method_name TEXT NOT NULL, 
            episode INTEGER NOT NULL, 
            reward REAL NOT NULL, 
            timestamp REAL DEFAULT 0
        );
    )";
    
    char* err_msg = 0;
    rc = sqlite3_exec(db, create_table_sql, 0, 0, &err_msg);
    
    if (rc != SQLITE_OK) {
        ROS_ERROR("SQL error: %s", err_msg);
        sqlite3_free(err_msg);
        return false;
    }
    
    ROS_INFO("Database initialized successfully.");
    return true;
}

// 插入奖励数据到数据库
bool insertEpisodeReward(const std::string& method_name, int episode, double reward, double timestamp) {
    if (!db) {
        ROS_ERROR("Database not initialized!");
        return false;
    }
    
    const char* sql = "INSERT INTO method_episode_rewards (method_name, episode, reward, timestamp) VALUES (?, ?, ?, ?);";
    sqlite3_stmt* stmt;
    
    int rc = sqlite3_prepare_v2(db, sql, -1, &stmt, NULL);
    if (rc != SQLITE_OK) {
        ROS_ERROR("Failed to prepare statement: %s", sqlite3_errmsg(db));
        return false;
    }
    
    sqlite3_bind_text(stmt, 1, method_name.c_str(), -1, SQLITE_STATIC);
    sqlite3_bind_int(stmt, 2, episode);
    sqlite3_bind_double(stmt, 3, reward);
    sqlite3_bind_double(stmt, 4, timestamp);
    
    rc = sqlite3_step(stmt);
    if (rc != SQLITE_DONE) {
        ROS_ERROR("Failed to insert data: %s", sqlite3_errmsg(db));
        sqlite3_finalize(stmt);
        return false;
    }
    
    sqlite3_finalize(stmt);
    ROS_INFO("Inserted reward: Method='%s', Episode=%d, Reward=%.2f, Timestamp=%.2f", 
             method_name.c_str(), episode, reward, timestamp);
    
    return true;
}

// 三段式状态机
enum RobotState {
    GENERATE_NEW_TARGET, // 生成随机目标
    DRIVE_TO_TARGET,     // 自动开过去
    DRAWING_CIRCLE       // 原地画圆
};

int main(int argc, char** argv) {
    ros::init(argc, argv, "draw_circle");
    ros::NodeHandle nh;

    // 订阅里程计，发布速度与状态
    ros::Subscriber odom_sub = nh.subscribe("/odom", 10, odomCallback);
    ros::Publisher cmd_pub = nh.advertise<geometry_msgs::Twist>("/cmd_vel", 10);
    ros::Publisher status_pub = nh.advertise<std_msgs::String>("/experiment/episode_event", 10);

    ros::Rate loop_rate(20); 

    // 初始化随机数生成器 (保证圆心在 -10 到 10 之间)
    std::random_device rd;
    std::mt19937 gen(rd());
    std::uniform_real_distribution<> dis(-10.0, 10.0);

    RobotState state = GENERATE_NEW_TARGET;
    
    // 轨迹参数
    double target_center_x = 0.0;
    double target_center_y = 0.0;
    double radius = 3.0;          // 半径为3
    
    // 起跑点坐标 (小车需要开到这个点才能开始画圆)
    double start_edge_x = 0.0;
    double start_edge_y = 0.0;

    // 画圆运动参数
    ros::Time draw_start_time;
    double circle_v = 0.6; // 画圆时的线速度
    double circle_w = circle_v / radius; // 画圆时的角速度
    double time_needed_for_one_circle = (2.0 * M_PI * radius) / circle_v; // 跑完一圈的时间

    ROS_INFO("Waiting for Odometry data...");
    while (ros::ok() && !odom_received) {
        ros::spinOnce();
        loop_rate.sleep();
    }
    ROS_INFO("Odometry received! Starting autonomous navigation & drawing task.");

    // 初始化数据库连接
    if (!initDatabase()) {
        ROS_ERROR("Failed to initialize database, shutting down...");
        return -1;
    }

    while (ros::ok()) {
        geometry_msgs::Twist cmd;
        std_msgs::String status_msg;

        switch (state) {
            case GENERATE_NEW_TARGET: {
                // 1. 生成随机圆心
                target_center_x = dis(gen);
                target_center_y = dis(gen);
                
                // 2. 设定起跑点 (这里定为圆心正右侧3米处)
                start_edge_x = target_center_x + radius;
                start_edge_y = target_center_y;

                // 3. 发布【开始】信号
                status_msg.data = "start";
                status_pub.publish(status_msg);
                ROS_INFO("Event Published: %s | Target Center: (%.2f, %.2f)", status_msg.data.c_str(), target_center_x, target_center_y);

                // 4. 切换状态，让小车开过去
                state = DRIVE_TO_TARGET;
                break;
            }

            case DRIVE_TO_TARGET: {
                // 核心逻辑：利用 P 控制器让小车自己开到目标起跑点
                double dx = start_edge_x - current_x;
                double dy = start_edge_y - current_y;
                double distance = std::hypot(dx, dy); // 距离目标的直线距离
                
                double target_yaw = std::atan2(dy, dx); // 目标朝向
                double yaw_error = target_yaw - current_yaw; // 角度偏差

                // 将角度偏差标准化到 [-pi, pi] 之间，防止小车无限原地转圈
                while (yaw_error > M_PI) yaw_error -= 2.0 * M_PI;
                while (yaw_error < -M_PI) yaw_error += 2.0 * M_PI;

                if (distance > 0.15) { // 误差允许范围：0.15米
                    if (std::abs(yaw_error) > 0.2) { 
                        // 如果偏航角大于约11度，先原地旋转对准目标
                        cmd.angular.z = (yaw_error > 0) ? 0.5 : -0.5;
                        cmd.linear.x = 0.0;
                    } else { 
                        // 对准之后，一边往前开，一边微调角度
                        cmd.linear.x = 0.6; // 直线行驶速度
                        cmd.angular.z = yaw_error * 1.5; 
                    }
                } else {
                    // 到达起跑点！刹车，记录时间，准备画圆
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    
                    draw_start_time = ros::Time::now();
                    state = DRAWING_CIRCLE;
                    ROS_INFO("Reached target edge. Start drawing circle...");
                }
                break;
            }

            case DRAWING_CIRCLE: {
                // 检查画圆时间是否足够跑完一圈
                if ((ros::Time::now() - draw_start_time).toSec() < time_needed_for_one_circle) {
                    cmd.linear.x = circle_v;
                    cmd.angular.z = circle_w;
                } else {
                    // 跑完一圈了，刹车！
                    cmd.linear.x = 0.0;
                    cmd.angular.z = 0.0;
                    
                    // 生成随机奖励值 (-100 到 100)
                    double reward = reward_dis(gen_reward);
                    double current_timestamp = ros::Time::now().toSec();
                    
                    // 插入数据库
                    insertEpisodeReward("方法一", episode_counter, reward, current_timestamp);
                    
                    // 发布【结束】信号
                    status_msg.data = "end";
                    status_pub.publish(status_msg);
                    ROS_INFO("Event Published: %s | Circle completed. Reward: %.2f | Episode: %d", 
                             status_msg.data.c_str(), reward, episode_counter);

                    // 回合数递增
                    episode_counter++;
                    
                    // 切回第一步，重新生成下一个随机点，循环往复
                    state = GENERATE_NEW_TARGET;
                }
                break;
            }
        }

        // 下发速度指令
        cmd_pub.publish(cmd);
        ros::spinOnce();
        loop_rate.sleep();
    }
    
    // 关闭数据库连接
    if (db) {
        sqlite3_close(db);
    }

    return 0;
}
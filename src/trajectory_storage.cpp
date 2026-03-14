#include "robot_monitor/trajectory_storage.h"

#include <sqlite3.h>

namespace robot_monitor
{

TrajectoryStorage::TrajectoryStorage()
    : db_(nullptr)
{
}

TrajectoryStorage::~TrajectoryStorage()
{
    close();
}

bool TrajectoryStorage::open(const std::string& db_path, std::string& error_message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (db_ != nullptr)
    {
        return true;
    }

    if (sqlite3_open(db_path.c_str(), &db_) != SQLITE_OK)
    {
        error_message = sqlite3_errmsg(db_);
        sqlite3_close(db_);
        db_ = nullptr;
        return false;
    }

    return true;
}

void TrajectoryStorage::close()
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (db_ != nullptr)
    {
        sqlite3_close(db_);
        db_ = nullptr;
    }
}

bool TrajectoryStorage::executeSql(const std::string& sql, std::string& error_message)
{
    char* err_msg = nullptr;
    const int rc = sqlite3_exec(db_, sql.c_str(), nullptr, nullptr, &err_msg);

    if (rc != SQLITE_OK)
    {
        if (err_msg != nullptr)
        {
            error_message = err_msg;
            sqlite3_free(err_msg);
        }
        else
        {
            error_message = "Unknown SQLite error";
        }
        return false;
    }

    return true;
}

bool TrajectoryStorage::initTables(std::string& error_message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (db_ == nullptr)
    {
        error_message = "Database is not open";
        return false;
    }

    const std::string create_trajectories_sql =
        "CREATE TABLE IF NOT EXISTS trajectories ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "name TEXT NOT NULL,"
        "method_name TEXT NOT NULL,"
        "episode_index INTEGER NOT NULL,"
        "start_time REAL,"
        "end_time REAL,"
        "point_count INTEGER"
        ");";

    const std::string create_points_sql =
        "CREATE TABLE IF NOT EXISTS trajectory_points ("
        "id INTEGER PRIMARY KEY AUTOINCREMENT,"
        "trajectory_id INTEGER NOT NULL,"
        "seq INTEGER NOT NULL,"
        "x REAL NOT NULL,"
        "y REAL NOT NULL,"
        "yaw REAL NOT NULL,"
        "linear_velocity REAL,"
        "angular_velocity REAL,"
        "timestamp REAL"
        ");";

    if (!executeSql(create_trajectories_sql, error_message))
    {
        return false;
    }

    if (!executeSql(create_points_sql, error_message))
    {
        return false;
    }

    return true;
}

int TrajectoryStorage::getNextEpisodeIndex(const std::string& method_name, std::string& error_message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (db_ == nullptr)
    {
        error_message = "Database is not open";
        return -1;
    }

    const char* sql =
        "SELECT COALESCE(MAX(episode_index), 0) "
        "FROM trajectories "
        "WHERE method_name = ?;";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK)
    {
        error_message = sqlite3_errmsg(db_);
        return -1;
    }

    sqlite3_bind_text(stmt, 1, method_name.c_str(), -1, SQLITE_TRANSIENT);

    int next_index = 1;

    if (sqlite3_step(stmt) == SQLITE_ROW)
    {
        const int max_index = sqlite3_column_int(stmt, 0);
        next_index = max_index + 1;
    }

    sqlite3_finalize(stmt);
    return next_index;
}

bool TrajectoryStorage::saveTrajectory(const TrajectoryRecord& record, std::string& error_message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (db_ == nullptr)
    {
        error_message = "Database is not open";
        return false;
    }

    if (record.points.empty())
    {
        error_message = "Trajectory has no points";
        return false;
    }

    if (!executeSql("BEGIN TRANSACTION;", error_message))
    {
        return false;
    }

    sqlite3_stmt* traj_stmt = nullptr;
    const char* traj_sql =
        "INSERT INTO trajectories "
        "(name, method_name, episode_index, start_time, end_time, point_count) "
        "VALUES (?, ?, ?, ?, ?, ?);";

    if (sqlite3_prepare_v2(db_, traj_sql, -1, &traj_stmt, nullptr) != SQLITE_OK)
    {
        error_message = sqlite3_errmsg(db_);
        executeSql("ROLLBACK;", error_message);
        return false;
    }

    sqlite3_bind_text(traj_stmt, 1, record.name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_text(traj_stmt, 2, record.method_name.c_str(), -1, SQLITE_TRANSIENT);
    sqlite3_bind_int(traj_stmt, 3, record.episode_index);
    sqlite3_bind_double(traj_stmt, 4, record.start_time);
    sqlite3_bind_double(traj_stmt, 5, record.end_time);
    sqlite3_bind_int(traj_stmt, 6, static_cast<int>(record.points.size()));

    if (sqlite3_step(traj_stmt) != SQLITE_DONE)
    {
        error_message = sqlite3_errmsg(db_);
        sqlite3_finalize(traj_stmt);
        executeSql("ROLLBACK;", error_message);
        return false;
    }

    sqlite3_finalize(traj_stmt);

    const sqlite3_int64 trajectory_id = sqlite3_last_insert_rowid(db_);

    sqlite3_stmt* point_stmt = nullptr;
    const char* point_sql =
        "INSERT INTO trajectory_points "
        "(trajectory_id, seq, x, y, yaw, linear_velocity, angular_velocity, timestamp) "
        "VALUES (?, ?, ?, ?, ?, ?, ?, ?);";

    if (sqlite3_prepare_v2(db_, point_sql, -1, &point_stmt, nullptr) != SQLITE_OK)
    {
        error_message = sqlite3_errmsg(db_);
        executeSql("ROLLBACK;", error_message);
        return false;
    }

    for (size_t i = 0; i < record.points.size(); ++i)
    {
        const TrajectoryPoint& pt = record.points[i];

        sqlite3_reset(point_stmt);
        sqlite3_clear_bindings(point_stmt);

        sqlite3_bind_int64(point_stmt, 1, trajectory_id);
        sqlite3_bind_int(point_stmt, 2, static_cast<int>(i));
        sqlite3_bind_double(point_stmt, 3, pt.x);
        sqlite3_bind_double(point_stmt, 4, pt.y);
        sqlite3_bind_double(point_stmt, 5, pt.yaw);
        sqlite3_bind_double(point_stmt, 6, pt.linear_velocity);
        sqlite3_bind_double(point_stmt, 7, pt.angular_velocity);
        sqlite3_bind_double(point_stmt, 8, pt.timestamp);

        if (sqlite3_step(point_stmt) != SQLITE_DONE)
        {
            error_message = sqlite3_errmsg(db_);
            sqlite3_finalize(point_stmt);
            executeSql("ROLLBACK;", error_message);
            return false;
        }
    }

    sqlite3_finalize(point_stmt);

    if (!executeSql("COMMIT;", error_message))
    {
        executeSql("ROLLBACK;", error_message);
        return false;
    }

    return true;
}

bool TrajectoryStorage::listTrajectories(std::vector<TrajectoryRecord>& records, std::string& error_message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    records.clear();

    if (db_ == nullptr)
    {
        error_message = "Database is not open";
        return false;
    }

    const char* sql =
        "SELECT id, name, method_name, episode_index, start_time, end_time, point_count "
        "FROM trajectories "
        "ORDER BY id DESC;";

    sqlite3_stmt* stmt = nullptr;
    if (sqlite3_prepare_v2(db_, sql, -1, &stmt, nullptr) != SQLITE_OK)
    {
        error_message = sqlite3_errmsg(db_);
        return false;
    }

    while (sqlite3_step(stmt) == SQLITE_ROW)
    {
        TrajectoryRecord record;
        record.id = sqlite3_column_int(stmt, 0);
        record.name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 1));
        record.method_name = reinterpret_cast<const char*>(sqlite3_column_text(stmt, 2));
        record.episode_index = sqlite3_column_int(stmt, 3);
        record.start_time = sqlite3_column_double(stmt, 4);
        record.end_time = sqlite3_column_double(stmt, 5);

        records.push_back(record);
    }

    sqlite3_finalize(stmt);
    return true;
}

bool TrajectoryStorage::loadTrajectoryById(int trajectory_id,
                                           TrajectoryRecord& record,
                                           std::string& error_message)
{
    std::lock_guard<std::mutex> lock(mutex_);

    record = TrajectoryRecord();

    if (db_ == nullptr)
    {
        error_message = "Database is not open";
        return false;
    }

    // 先读取轨迹元信息
    const char* traj_sql =
        "SELECT id, name, method_name, episode_index, start_time, end_time, point_count "
        "FROM trajectories "
        "WHERE id = ?;";

    sqlite3_stmt* traj_stmt = nullptr;
    if (sqlite3_prepare_v2(db_, traj_sql, -1, &traj_stmt, nullptr) != SQLITE_OK)
    {
        error_message = sqlite3_errmsg(db_);
        return false;
    }

    sqlite3_bind_int(traj_stmt, 1, trajectory_id);

    if (sqlite3_step(traj_stmt) == SQLITE_ROW)
    {
        record.id = sqlite3_column_int(traj_stmt, 0);
        record.name = reinterpret_cast<const char*>(sqlite3_column_text(traj_stmt, 1));
        record.method_name = reinterpret_cast<const char*>(sqlite3_column_text(traj_stmt, 2));
        record.episode_index = sqlite3_column_int(traj_stmt, 3);
        record.start_time = sqlite3_column_double(traj_stmt, 4);
        record.end_time = sqlite3_column_double(traj_stmt, 5);
    }
    else
    {
        sqlite3_finalize(traj_stmt);
        error_message = "Trajectory not found";
        return false;
    }

    sqlite3_finalize(traj_stmt);

    // 再读取轨迹所有点
    const char* points_sql =
        "SELECT seq, x, y, yaw, linear_velocity, angular_velocity, timestamp "
        "FROM trajectory_points "
        "WHERE trajectory_id = ? "
        "ORDER BY seq ASC;";

    sqlite3_stmt* points_stmt = nullptr;
    if (sqlite3_prepare_v2(db_, points_sql, -1, &points_stmt, nullptr) != SQLITE_OK)
    {
        error_message = sqlite3_errmsg(db_);
        return false;
    }

    sqlite3_bind_int(points_stmt, 1, trajectory_id);

    while (sqlite3_step(points_stmt) == SQLITE_ROW)
    {
        TrajectoryPoint pt;
        pt.x = sqlite3_column_double(points_stmt, 1);
        pt.y = sqlite3_column_double(points_stmt, 2);
        pt.yaw = sqlite3_column_double(points_stmt, 3);
        pt.linear_velocity = sqlite3_column_double(points_stmt, 4);
        pt.angular_velocity = sqlite3_column_double(points_stmt, 5);
        pt.timestamp = sqlite3_column_double(points_stmt, 6);

        record.points.push_back(pt);
    }

    sqlite3_finalize(points_stmt);

    return true;
}

}  // namespace robot_monitor
#ifndef ROBOT_MONITOR_TRAJECTORY_STORAGE_H
#define ROBOT_MONITOR_TRAJECTORY_STORAGE_H

#include <mutex>
#include <string>
#include <vector>

#include "robot_monitor/trajectory_types.h"

struct sqlite3;

namespace robot_monitor
{

class TrajectoryStorage
{
public:
    TrajectoryStorage();
    ~TrajectoryStorage();

    bool open(const std::string& db_path, std::string& error_message);
    void close();

    bool initTables(std::string& error_message);

    int getNextEpisodeIndex(const std::string& method_name, std::string& error_message);

    bool saveTrajectory(const TrajectoryRecord& record, std::string& error_message);

    bool listTrajectories(std::vector<TrajectoryRecord>& records, std::string& error_message);
    
    bool loadTrajectoryById(int trajectory_id, TrajectoryRecord& record, std::string& error_message);

    bool deleteTrajectoryById(int trajectory_id, std::string& error_message);
    bool deleteTrajectoriesByMethod(const std::string& method_name, std::string& error_message);

    bool insertEpisodeReward(const std::string& method_name,
                        int episode,
                        double reward,
                        double timestamp,
                        std::string& error_message);

    bool loadEpisodeRewardsByMethod(const std::string& method_name,
                                    std::vector<EpisodeRewardPoint>& rewards,
                                    std::string& error_message);

private:
    bool executeSql(const std::string& sql, std::string& error_message);

private:
    mutable std::mutex mutex_;
    sqlite3* db_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_TRAJECTORY_STORAGE_H
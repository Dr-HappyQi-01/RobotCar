#ifndef ROBOT_MONITOR_TRAJECTORY_RECORDER_H
#define ROBOT_MONITOR_TRAJECTORY_RECORDER_H

#include <mutex>
#include <string>

#include "robot_monitor/trajectory_types.h"

namespace robot_monitor
{

class TrajectoryRecorder
{
public:
    TrajectoryRecorder();
    ~TrajectoryRecorder();

    bool startRecording(const std::string& method_name, int episode_index);
    bool stopRecording(TrajectoryRecord& finished_record);

    void appendPoint(const TrajectoryPoint& point);

    bool isRecording() const;
    std::string currentTrajectoryName() const;
    TrajectoryRecord currentRecord() const;

    void reset();

private:
    mutable std::mutex mutex_;
    bool recording_;
    TrajectoryRecord current_record_;
};

}  // namespace robot_monitor

#endif  // ROBOT_MONITOR_TRAJECTORY_RECORDER_H
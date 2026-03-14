#include "robot_monitor/trajectory_recorder.h"

namespace robot_monitor
{

TrajectoryRecorder::TrajectoryRecorder()
    : recording_(false)
{
}

TrajectoryRecorder::~TrajectoryRecorder()
{
}

bool TrajectoryRecorder::startRecording(const std::string& method_name, int episode_index)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (recording_)
    {
        return false;
    }

    current_record_ = TrajectoryRecord();
    current_record_.method_name = method_name;
    current_record_.episode_index = episode_index;
    current_record_.name = method_name + "-轨迹" + std::to_string(episode_index);

    recording_ = true;
    return true;
}

bool TrajectoryRecorder::stopRecording(TrajectoryRecord& finished_record)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!recording_)
    {
        return false;
    }

    recording_ = false;
    finished_record = current_record_;
    return true;
}

void TrajectoryRecorder::appendPoint(const TrajectoryPoint& point)
{
    std::lock_guard<std::mutex> lock(mutex_);

    if (!recording_)
    {
        return;
    }

    if (current_record_.points.empty())
    {
        current_record_.start_time = point.timestamp;
    }

    current_record_.points.push_back(point);
    current_record_.end_time = point.timestamp;
}

bool TrajectoryRecorder::isRecording() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return recording_;
}

std::string TrajectoryRecorder::currentTrajectoryName() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return current_record_.name;
}

TrajectoryRecord TrajectoryRecorder::currentRecord() const
{
    std::lock_guard<std::mutex> lock(mutex_);
    return current_record_;
}

void TrajectoryRecorder::reset()
{
    std::lock_guard<std::mutex> lock(mutex_);
    recording_ = false;
    current_record_ = TrajectoryRecord();
}



}  // namespace robot_monitor
/*
 * @Author: Shengyu Han hanshy2@shanghai-electric.com
 * @Date: 2026-07-14 19:06:11
 * @LastEditors: Shengyu Han hanshy2@shanghai-electric.com
 * @LastEditTime: 2026-07-17 13:50:35
 * @FilePath: /slam_nav/merman_communication/src/merman_communication_interface/merman_communication_interface.cpp
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#include <utility>

#include "merman_communication_interface/merman_communication_interface.h"
#include "manager/manager.h"


MermanCommunicationInterface::MermanCommunicationInterface()
    : manager_(std::make_unique<Manager>(*this))
{
}

MermanCommunicationInterface::~MermanCommunicationInterface()
{
    stop();
}

std::string MermanCommunicationInterface::get_communication_version(){
    return "v26.07.08";
}

bool MermanCommunicationInterface::init(const CommunicationConfig& config)
{
    return manager_->init(config);
}

bool MermanCommunicationInterface::start()
{
    return manager_->start();
}

void MermanCommunicationInterface::stop()
{
    if (manager_) {
        manager_->stop();
    }
}

void MermanCommunicationInterface::UpdateBattery(const BatteryData &battery) {
    manager_->UpdateBattery(battery);
}

bool MermanCommunicationInterface::publishJson(const std::string& topic_name, const std::string& json)
{
    return manager_->publishJson(topic_name, json);
}

void MermanCommunicationInterface::setCommandCallback(CommandCallback callback)
{
    manager_->setCommandCallback(std::move(callback));
}

void MermanCommunicationInterface::set_callbacks(
    CommunicationCallbacks callbacks)
{
    std::lock_guard<std::mutex> lock(callbacks_mutex_);
    callbacks_ = std::move(callbacks);
}

bool MermanCommunicationInterface::get_task_running() const
{
    std::function<bool()> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.get_task_running;
    }

    if (!callback) {
        return false;
    }

    return callback();
}

bool MermanCommunicationInterface::StartTask() {
    std::function<bool()> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.start_task;
    }

    if (!callback) {
        return false;
    }

    return callback();
}

bool MermanCommunicationInterface::StopTask() {
    std::function<bool()> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.stop_task;
    }

    if (!callback) {
        return false;
    }

    return callback();
}

bool MermanCommunicationInterface::set_nav2_waypoints(std::vector<NavigationWaypoint>& waypoints){
    manager_->set_nav2_waypoints(waypoints);
    return true;
}

bool MermanCommunicationInterface::PublishCmdVel(double linear_x, double angular_z){
    manager_->PublishCmdVel(linear_x, angular_z);
    return true;
}

bool MermanCommunicationInterface::PoseArrived(int32_t pose_id, bool play_audio) {
    manager_->PoseArrived(pose_id, play_audio);
    return true;
}

bool MermanCommunicationInterface::AudioDone(int32_t audio_id) {
    std::function<bool(int32_t)> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.audio_done;
    }

    if (!callback) {
        return false;
    }

    return callback(audio_id);
}

bool MermanCommunicationInterface::PauseTask(bool pause_task) {
    std::function<bool(bool)> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.pause_task;
    }

    if (!callback) {
        return false;
    }

    return callback(pause_task);
}

bool MermanCommunicationInterface::set_nav2_status(std::string& nav2_status) {
        std::function<bool(std::string&)> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.set_nav2_status;
    }

    if (!callback) {
        return false;
    }

    return callback(nav2_status);
}

bool MermanCommunicationInterface::get_pose(Pose2D& pose) {
    manager_->get_pose(pose);
    return true;
}

bool MermanCommunicationInterface::CancelWaypointNavigation() {
    manager_->CancelWaypointNavigation();
    return true;
}

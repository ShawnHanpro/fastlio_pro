/*
 * @Author: Shengyu Han hanshy2@shanghai-electric.com
 * @Date: 2026-07-14 19:06:11
 * @LastEditors: Shengyu Han hanshy2@shanghai-electric.com
 * @LastEditTime: 2026-07-17 13:50:47
 * @FilePath: /slam_nav/merman_communication/include/merman_communication_interface/merman_communication_interface.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <memory>
#include <string>
#include <functional>
#include <mutex>

#include "merman_types.h"

class Manager;

struct CommunicationCallbacks
{
    // 通信模块查询控制模块状态
    std::function<bool()> get_task_running;
    std::function<std::string()> get_control_state;

    // 通信模块向控制模块发送命令
    std::function<void(const CommunicationCommand&)> on_command;

    std::function<bool()> start_task;
    std::function<bool()> stop_task;

    std::function<bool(int32_t)> audio_done;
    std::function<bool(std::string&)> set_nav2_status;
    std::function<bool(bool)> pause_task;
};

class MermanCommunicationInterface
{
public:
    MermanCommunicationInterface();
    ~MermanCommunicationInterface();

    MermanCommunicationInterface(
    const MermanCommunicationInterface&) = delete;

    MermanCommunicationInterface& operator=(
        const MermanCommunicationInterface&) = delete;

    static std::string get_communication_version();

    bool init(const CommunicationConfig& config);
    bool start();
    void stop();

    void UpdateBattery(const BatteryData &battery);

    bool publishJson(const std::string& topic_name, const std::string& json);

    void setCommandCallback(CommandCallback callback);

    void set_callbacks(CommunicationCallbacks callbacks);

    bool set_nav2_status(std::string& nav2_status);

    bool set_nav2_waypoints(std::vector<NavigationWaypoint>& waypoints);
    bool PublishCmdVel(double linear_x, double angular_z);

    bool PoseArrived(int32_t pose_id, bool play_audio);

    bool AudioDone(int32_t audio_id);
    bool PauseTask(bool pause_task);
    
    bool get_task_running() const;
    bool get_pose(Pose2D& pose);
    bool StartTask();
    bool StopTask();

    bool CancelWaypointNavigation();
    
private:
    CommunicationCallbacks callbacks_;
    mutable std::mutex callbacks_mutex_;

    std::unique_ptr<Manager> manager_;
};
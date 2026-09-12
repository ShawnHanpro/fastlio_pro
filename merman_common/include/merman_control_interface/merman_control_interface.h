/*
 * @Author: Shengyu Han hanshy2@shanghai-electric.com
 * @Date: 2026-07-14 19:06:11
 * @LastEditors: Shengyu Han hanshy2@shanghai-electric.com
 * @LastEditTime: 2026-07-17 09:04:50
 * @FilePath: /slam_nav/merman_control/include/merman_control_interface/merman_control_interface.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <atomic>
#include <iostream>
#include <memory>
#include <mutex>
#include <queue>
#include <string>
#include <thread>

#include "merman_slam_interface/merman_slam_interface.h"
#include "merman_communication_interface/merman_communication_interface.h"
#include "merman_types.h"

class StateMachine;
class TaskManager;
class ManagerBase;
enum class RobotEvent;

class MermanControlInterface {
private:
    MermanControlInterface();
    ~MermanControlInterface();

public:
    static MermanControlInterface *GetInstance() {
        static MermanControlInterface instance;
        static bool                   has_init = false;
        if (!has_init) {
            // instance.Init();
            has_init = true;
        }
        return &instance;
    }

    static std::string get_control_version();
    static std::string get_slam_version();
    static std::string get_navigation_version();
    static std::string get_communication_version();

    bool Init();

    bool BindModuleCallbacks();
    bool StartCommunication();
    void StopCommunication();

    void UpdateLaserScan(const LaserScan &laser_scan);
    void UpdateImage(const Image &camera);
    void UpdateOdom(const Odometry &odom);

    void UpdateBattery(const BatteryData &battery);

    void StartFsmThread();
    void StopFsmThread();
    void StartCommunicationThread();
    void StopCommunicationThread();

    void StartMapping();
    void StopMapping();
    void StartLocalization();
    void StartNavigation();
    void StartTask(const std::string& task_name = std::string());
    void FinishNavigation();
    void FinishTask();
    void StartCharging();
    void FinishCharging();
    void ReportError();
    void Reset();
    std::string CurrentStateName() const;

    void get_map();
    void get_pose();

    TaskManager* get_task_manager() noexcept;
    const TaskManager* get_task_manager() const noexcept;

    void PostEvent(RobotEvent event);

private:
    void onCommunicationCommand(
    const CommunicationCommand& cmd);

private:

    MermanCommunicationInterface communication_;

    std::thread       hsm_thread_;
    std::atomic<bool> running_{false};

    // PIMPL 模式 解耦与隐藏
private:
    std::unique_ptr<StateMachine> state_machine_;
    std::unique_ptr<ManagerBase> manager_;
};

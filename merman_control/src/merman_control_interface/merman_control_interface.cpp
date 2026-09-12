#include <chrono>

#include "merman_control_interface/merman_control_interface.h"
#include "merman_types.h"
#include "states/mapping_state.h"
#include "states/standby_state.h"
#include "state_machine/state_machine.h"
#include "state_machine/robot_event.h"

#include "config_managers/communication_config.h"
#include "config_managers/task_config.h"

#include "managers/task_manager.h"

#include <cstdlib>
#include <filesystem>
#include <iostream>
#include <stdexcept>

namespace fs = std::filesystem;

namespace {

fs::path GetMermanConfigDir() {
    // 1. systemd / shell 可以显式指定配置目录
    const char* config_dir = std::getenv("MERMAN_CONFIG_DIR");

    if (config_dir != nullptr && config_dir[0] != '\0') {
        return fs::path(config_dir);
    }

    // 2. 根据当前可执行文件位置寻找
    //
    // /proc/self/exe
    //      ↓
    // /home/niic/slam_nav/merman_common/bin/merman_robot
    //
    // parent_path()
    //      ↓
    // /home/niic/slam_nav/merman_common/bin
    //
    // parent_path()
    //      ↓
    // /home/niic/slam_nav/merman_common
    //
    std::error_code ec;

    fs::path exe_path = fs::read_symlink("/proc/self/exe", ec);

    if (!ec) {
        return exe_path.parent_path().parent_path() / "config";
    }

    throw std::runtime_error("Cannot determine merman config directory");
}

}  // namespace

MermanControlInterface::MermanControlInterface() {
    state_machine_ = std::make_unique<StateMachine>();

    if (!Init()) {
        throw std::runtime_error(
            "MermanControlInterface initialization failed");
    }
}

MermanControlInterface::~MermanControlInterface() {

    StopCommunication();
    StopFsmThread();
}

std::string MermanControlInterface::get_control_version() { return "v1.0"; }

std::string MermanControlInterface::get_slam_version() {
    return MermanSlamInterface::get_slam_version();
}

std::string MermanControlInterface::get_navigation_version() { return ""; }
std::string MermanControlInterface::get_communication_version() { 
    return MermanCommunicationInterface::get_communication_version();
}

bool MermanControlInterface::Init() {
    const auto config_dir = GetMermanConfigDir();

    std::cout << "[MermanControl] config dir: " << config_dir << std::endl;

    // =========================================================
    // 1. TaskManager
    // =========================================================
    TaskConfig task_config;

    const std::string task_config_path = (config_dir / "task.yaml").string();

    std::cout << "[MermanControl] task config: " << task_config_path
              << std::endl;

    if (!LoadTaskConfig(task_config_path, task_config)) {
        std::cerr << "[MermanControl] load task config failed: "
                  << task_config_path << std::endl;

        return false;
    }

    manager_ = std::make_unique<TaskManager>(task_config);

    // =========================================================
    // 2. Communication
    // =========================================================
    CommunicationConfig communication_config;

    const std::string communication_config_path =
        (config_dir / "communication.yaml").string();

    std::cout << "[MermanControl] communication config: "
              << communication_config_path << std::endl;

    if (!LoadCommunicationConfig(communication_config_path,
                                 communication_config)) {
        std::cerr << "[MermanControl] load communication config failed: "
                  << communication_config_path << std::endl;

        return false;
    }

    // =========================================================
    // 3. 绑定 TaskManager <-> Communication 回调
    // =========================================================
    if (!BindModuleCallbacks()) {
        return false;
    }

    // =========================================================
    // 4. 初始化通信模块
    // =========================================================
    if (!communication_.init(communication_config)) {
        return false;
    }

    return true;
}

bool MermanControlInterface::BindModuleCallbacks()
{
    TaskManager* task_manager =
        get_task_manager();

    if (task_manager == nullptr) {
        std::cerr
            << "manager is not TaskManager"
            << std::endl;

        return false;
    }

    /*
     * Communication -> TaskManager
     */
    CommunicationCallbacks communication_callbacks;

    communication_callbacks.get_task_running =
        [this]() -> bool {
            const TaskManager* task_manager =
                get_task_manager();

            if (task_manager == nullptr) {
                return false;
            }

            return task_manager->get_task_running();
        };

    communication_callbacks.start_task =
        [this]() -> bool {
            TaskManager* task_manager =
                get_task_manager();

            if (task_manager == nullptr) {
                return false;
            }

            return task_manager->Start();
        };

    communication_callbacks.stop_task = [this]() -> bool {
        TaskManager* task_manager = get_task_manager();

        if (task_manager == nullptr) {
            return false;
        }

        return task_manager->Stop();
    };

    communication_callbacks.audio_done =
    [this](int32_t audio_id) -> bool {
        TaskManager* task_manager = get_task_manager();

        if (task_manager == nullptr) {
            return false;
        }

        return task_manager->AudioDone(audio_id);
    };

    communication_callbacks.pause_task =
    [this](bool pause_task) -> bool {
        TaskManager* task_manager = get_task_manager();

        if (task_manager == nullptr) {
            return false;
        }

        return task_manager->PauseTask(pause_task);
    };

    communication_callbacks.set_nav2_status =
    [this](std::string& nav2_status) -> bool {
        TaskManager* task_manager = get_task_manager();

        if (task_manager == nullptr) {
            return false;
        }

        return task_manager->set_nav2_status(nav2_status);
    };

    communication_.set_callbacks(
        std::move(communication_callbacks));

    /*
     * TaskManager -> Communication
     */
    TaskCallbacks task_callbacks;

    task_callbacks.pose_arrived =
        [this](int32_t pose_id, bool play_audio) -> bool {
        return communication_.PoseArrived(pose_id, play_audio);
    };

    task_callbacks.set_nav2_waypoints =
        [this](std::vector<NavigationWaypoint>& waypoints) -> bool {
        return communication_.set_nav2_waypoints(waypoints);
    };

    task_callbacks.set_cmd_vel = [this](double linear_x,
                                        double angular_z) -> bool {
        return communication_.PublishCmdVel(linear_x, angular_z);
    };

    task_callbacks.get_pose =
        [this](Pose2D& pose) -> bool {
        return communication_.get_pose(pose);
    };

    task_callbacks.cancel_nav2_waypoints =
        [this]() -> bool {
        return communication_.CancelWaypointNavigation();
    };


    task_manager->set_callbacks(
        std::move(task_callbacks));

    return true;
}

TaskManager* MermanControlInterface::get_task_manager() noexcept {
    return dynamic_cast<TaskManager*>(manager_.get());
}

const TaskManager* MermanControlInterface::get_task_manager() const noexcept {
    return dynamic_cast<const TaskManager*>(manager_.get());
}

void MermanControlInterface::UpdateLaserScan(const LaserScan& laser_scan) {
    auto slam_ptr = MermanSlamInterface::GetInstance();  // 更新SLAM用的激光数据
    if (!laser_scan.ranges.empty()) {
        slam_ptr->UpdateLaserScan(laser_scan);
    }
}

void MermanControlInterface::UpdateBattery(const BatteryData &battery) {
    communication_.UpdateBattery(battery);
}

void MermanControlInterface::StartFsmThread() {
    bool expected = false;
    if (!running_.compare_exchange_strong(expected, true)) {
        return;
    }

    hsm_thread_ = std::thread([this]() {
        state_machine_->Initialize();

        while (running_) {
            state_machine_->Run();
            std::this_thread::sleep_for(std::chrono::milliseconds(50));
        }

        state_machine_->Shutdown();
    });
}

void MermanControlInterface::StopFsmThread() {
    running_ = false;

    if (hsm_thread_.joinable() &&
        hsm_thread_.get_id() != std::this_thread::get_id()) {
        hsm_thread_.join();
    }
}

void MermanControlInterface::StartMapping() {
    PostEvent(RobotEvent::StartMapping);
}

void MermanControlInterface::StopMapping() {
    PostEvent(RobotEvent::StopMapping);
}

void MermanControlInterface::StartLocalization() {
    PostEvent(RobotEvent::StartLocalization);
}

void MermanControlInterface::StartNavigation() {
    PostEvent(RobotEvent::StartNavigation);
}

void MermanControlInterface::StartTask(const std::string& task_name) {
    if (state_machine_) {
        state_machine_->PostEvent(RobotEvent::StartTask, task_name);
    }
}

void MermanControlInterface::FinishNavigation() {
    PostEvent(RobotEvent::NavigationFinished);
}

void MermanControlInterface::FinishTask() {
    PostEvent(RobotEvent::TaskFinished);
}

void MermanControlInterface::StartCharging() {
    PostEvent(RobotEvent::LowBattery);
}

void MermanControlInterface::FinishCharging() {
    PostEvent(RobotEvent::ChargeFinished);
}

void MermanControlInterface::ReportError() {
    PostEvent(RobotEvent::Error);
}

void MermanControlInterface::Reset() {
    PostEvent(RobotEvent::Reset);
}

std::string MermanControlInterface::CurrentStateName() const {
    if (!state_machine_) {
        return "None";
    }

    return state_machine_->CurrentStateName();
}

void MermanControlInterface::StartCommunicationThread() {
    StartCommunication();
}

void MermanControlInterface::StopCommunicationThread() {
    StopCommunication();
}

void MermanControlInterface::get_map(){
    auto slam_ptr = MermanSlamInterface::GetInstance();
    slam_ptr->get_map();
}

void MermanControlInterface::get_pose(){
    auto slam_ptr = MermanSlamInterface::GetInstance();
    slam_ptr->get_pose();
}

void MermanControlInterface::PostEvent(RobotEvent event) {
    if (state_machine_) {
        state_machine_->PostEvent(event);
    }
}

bool MermanControlInterface::StartCommunication()
{
    return communication_.start();
}

void MermanControlInterface::StopCommunication()
{
    communication_.stop();
}

void MermanControlInterface::onCommunicationCommand(
    const CommunicationCommand& cmd)
{
    // 这里进入状态机处理
    //
    // cmd.source:
    //   LOCAL / CLOUD / SBUS / TCP / MQTT
    //
    // cmd.payload_json:
    //   具体控制命令 JSON

    if (cmd.source == CommandSource::CLOUD) {
        // 云端控制命令
    } else if (cmd.source == CommandSource::LOCAL) {
        // 本地控制命令
    }

    // 推荐这里把通讯命令转换成状态机事件
    // state_machine_.handleEvent(...);
}
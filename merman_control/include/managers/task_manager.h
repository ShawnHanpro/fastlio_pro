/*
 * @Author: Shengyu Han hanshy2@shanghai-electric.com
 * @Date: 2026-07-15 18:19:20
 * @LastEditors: Shengyu Han hanshy2@shanghai-electric.com
 * @LastEditTime: 2026-07-17 18:28:28
 * @FilePath: /slam_nav/merman_control/include/managers/task_manager.h
 * @Description: 这是默认设置,请设置`customMade`, 打开koroFileHeader查看配置 进行设置: https://github.com/OBKoro1/koro1FileHeader/wiki/%E9%85%8D%E7%BD%AE
 */
#pragma once

#include <string>

#include "managers/manager_base.h"
#include "merman_types.h"

struct TaskCallbacks {
    // TaskManager 向 Communication 发送导航点
    std::function<bool(std::vector<NavigationWaypoint>&)> set_nav2_waypoints;

    // TaskManager 向 Communication 发送速度
    std::function<bool(double linear_x, double angular_z)> set_cmd_vel;

    std::function<bool(Pose2D& pose)> get_pose;
    std::function<bool(int32_t pose_id, bool play_audio)> pose_arrived;

    std::function<bool()> cancel_nav2_waypoints;
};

struct WayPoints {
    int point_id;
    std::string type;
    NavigationWaypoint point;
};

enum class Nav2Status : uint8_t {
    IDLE = 0,

    // Nav2已经接受目标
    ACCEPTED,

    // 正在执行导航
    RUNNING,

    // 所有导航点执行成功
    SUCCEEDED,

    // FollowWaypoints结束，但存在未到达点
    PARTIAL_SUCCEEDED,

    // Nav2主动中止
    ABORTED,

    // 导航任务被取消
    CANCELED,

    // Nav2拒绝目标
    REJECTED,

    // 无法识别的状态
    UNKNOWN
};

class TaskManager : public ManagerBase {
public:
    explicit TaskManager(TaskConfig config);
    ~TaskManager() override;

    bool Start() override;
    bool Stop() override;

    void Update() override;
    bool IsRunning() const override;
    bool IsFinished() const override;
    bool HasError() const override;
    std::string Name() const override;

    bool AudioDone(int32_t audio_id);
    bool PauseTask(bool pause_task);
    
    bool PoseArrived(int32_t pose_id, bool play_audio);

    bool get_task_running() const;
    bool set_nav2_waypoints(std::vector<NavigationWaypoint>& navigation_waypoints);
    bool set_cmd_vel(double linear_x, double angular_z);
    bool get_pose(Pose2D& pose);

    bool set_nav2_status(std::string& nav2_status);

    bool CancelWaypointNavigation();

    bool SetNav2Waypoints();

    void set_callbacks(TaskCallbacks callbacks);

    void clear_callbacks()
    {
        callbacks_ = {};
    }

private:
    bool LoadNavigationWaypoints(std::string points_file);
    bool HandleWaypoints();
    Nav2Status ParseNav2Status(const std::string& status);

    bool Rotation(double target_theta);

    double NormalizeAngle(double angle);

    // bool StartRotationAsync(double target_theta);

    bool StopRotation();

    // 传入完整点位，避免旋转线程中读取已经变化的 current_waypoint_
    bool StartRotationAsync(WayPoints waypoint);

    // 当前点完成后，从待执行列表中移除
    bool RemoveCompletedWaypoint(int32_t point_id,
                                 bool& has_next_waypoint);

    // 根据点位类型判断是否播放语音
    static bool ShouldPlayAudio(const WayPoints& waypoint);

    bool AdvanceToNextWaypoint(int32_t completed_pose_id);

    /**
     * 等待音频播放完成的状态。
     *
     * 使用mutex保护，避免重复AudioDone或者错误ID导致任务重复推进。
     */
    std::mutex audio_state_mutex_;
    bool waiting_audio_done_{false};
    int32_t waiting_audio_pose_id_{-1};

    // 防止同一个导航点的 succeeded 状态被重复处理
    std::atomic_bool nav2_success_handled_{false};

    std::thread rotation_thread_;
    std::mutex rotation_thread_mutex_;

    std::atomic<bool> rotation_running_{false};
    std::atomic<bool> rotation_stop_requested_{false};

    // 上一次成功发送给 Nav2 的位置
    std::mutex last_nav2_point_mutex_;

    bool has_last_nav2_point_{false};
    NavigationWaypoint last_nav2_point_;

private:
    std::vector<WayPoints> waypoints_;
    WayPoints current_waypoint_;

    std::mutex waypoints_mutex_;

    TaskCallbacks callbacks_;
    mutable std::mutex callbacks_mutex_;

    std::mutex start_mutex_;

    std::atomic<bool> task_running_{false};
    std::atomic<bool> pause_task_{false};

    bool task_finished_{false};
    bool has_error_{false};

    TaskConfig config_;

    Pose2D pose_;
};
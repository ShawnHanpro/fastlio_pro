#pragma once

#include <atomic>
#include <geometry_msgs/msg/pose_stamped.hpp>
// #include <geometry_msgs/msg/pose_with_covariance_stamped.hpp>
#include <geometry_msgs/msg/pose_stamped.hpp>
#include <geometry_msgs/msg/twist.hpp>
#include <map>
#include <memory>
#include <mutex>
#include <nav2_msgs/action/follow_waypoints.hpp>
#include <rclcpp/rclcpp.hpp>
#include <rclcpp_action/rclcpp_action.hpp>
#include <std_msgs/msg/int32.hpp>
#include <std_msgs/msg/bool.hpp>
#include <std_msgs/msg/string.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <string>
#include <thread>
#include <queue>
#include <condition_variable>
#include <cstdint>
#include <vector>

#include <rclcpp/callback_group.hpp>
#include <rclcpp/executors/multi_threaded_executor.hpp>

#include "base/channel_base.h"
#include "merman_types.h"

#include "merman_ros2_types/srv/pose_arrived.hpp"
#include "merman_ros2_types/msg/pose_arrived_data.hpp"

using PoseArrivedSrv = merman_ros2_types::srv::PoseArrived;
using AudioPlayMsg = std_msgs::msg::Int32;

using Trigger = std_srvs::srv::Trigger;
using FollowWaypoints = nav2_msgs::action::FollowWaypoints;
using FollowWaypointsGoalHandle =
    rclcpp_action::ClientGoalHandle<FollowWaypoints>;

class MermanCommunicationInterface;

class Ros2Channel : public ChannelBase {
public:
    explicit Ros2Channel(MermanCommunicationInterface& communication);
    ~Ros2Channel() override;

    bool init(const CommunicationConfig& config) override;
    bool start() override;
    void stop() override;

    void StartStatusUploadThread();
    void StopStatusUploadThread();

    bool PublishRobotStatus(const RobotStatus& robot_status) override;

    bool set_waypoints(std::vector<NavigationWaypoint>& waypoints) override;

    bool get_pose(Pose2D& pose) override;

    bool CancelWaypointNavigation() override;

    bool PublishCmdVel(double linear_x, double angular_z) override;
    bool PoseArrived(int32_t pose_id, bool play_audio) override;
    bool StopRobot();

    bool publishJson(const std::string& topic_name,
                     const std::string& json) override;

    void setCommandCallback(CommandCallback callback) override;

private:
    void onCommandMessage(const std_msgs::msg::String::SharedPtr msg,
                          CommandSource                          source);

    std::string RobotStatusToJson(const RobotStatus& robot_status);

    RobotStatus get_robot_status() const;

    // void batteryCallback(const std_msgs::msg::Float32::SharedPtr msg);
    void PoseCallback(
        const geometry_msgs::msg::PoseStamped::SharedPtr msg);
    void CmdVelCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // void velocityCallback(const geometry_msgs::msg::Twist::SharedPtr msg);
    // void emergencyStopCallback(const std_msgs::msg::Bool::SharedPtr msg);
    void AudioDoneCallback(const std_msgs::msg::Int32::SharedPtr msg);
    void PauseTaskCallback(const std_msgs::msg::Bool::SharedPtr msg);

    void HandleStartTask(const std::shared_ptr<Trigger::Request> request,
                         std::shared_ptr<Trigger::Response>      response);
    void HandleStopTask(const std::shared_ptr<Trigger::Request> request,
                        std::shared_ptr<Trigger::Response>      response);

    bool                            StartWaypointNavigation();
    geometry_msgs::msg::PoseStamped WaypointToPose(
        const NavigationWaypoint& waypoint) const;

    void HandleWaypointGoalResponse(
        FollowWaypointsGoalHandle::SharedPtr goal_handle);
    void HandleWaypointFeedback(
        FollowWaypointsGoalHandle::SharedPtr                   goal_handle,
        const std::shared_ptr<const FollowWaypoints::Feedback> feedback);
    void HandleWaypointResult(
        const FollowWaypointsGoalHandle::WrappedResult& result);

private:
    MermanCommunicationInterface& communication_;
    CommunicationConfig           config_;

    rclcpp::Node::SharedPtr                                    node_;
    // std::shared_ptr<rclcpp::executors::SingleThreadedExecutor> executor_;
    rclcpp::Executor::SharedPtr executor_;
    rclcpp::CallbackGroup::SharedPtr pause_task_callback_group_;

    rclcpp::Publisher<std_msgs::msg::String>::SharedPtr robot_status_pub_;
    rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_pub_;
    rclcpp::Publisher<AudioPlayMsg>::SharedPtr audio_play_pub_;

    // rclcpp::Subscription<std_msgs::msg::Float32>::SharedPtr battery_sub_;
    rclcpp::Subscription<
        geometry_msgs::msg::PoseStamped>::SharedPtr pose_sub_;
    rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr    cmd_vel_sub_;
    // rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr velocity_sub_;
    // rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr emergency_stop_sub_;
    rclcpp::Subscription<std_msgs::msg::Int32>::SharedPtr audio_done_sub_;
    rclcpp::Subscription<std_msgs::msg::Bool>::SharedPtr pause_task_sub_;

    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr local_cmd_sub_;
    // rclcpp::Subscription<std_msgs::msg::String>::SharedPtr cloud_cmd_sub_;

    rclcpp::Service<Trigger>::SharedPtr start_task_service_;
    rclcpp::Service<Trigger>::SharedPtr stop_task_service_;

    rclcpp_action::Client<FollowWaypoints>::SharedPtr follow_waypoints_client_;
    rclcpp::Client<PoseArrivedSrv>::SharedPtr pose_arrived_client_;
    FollowWaypointsGoalHandle::SharedPtr follow_waypoints_goal_handle_;

    std::mutex pose_arrived_client_mutex_;

    bool owns_rclcpp_init_ = false;

    RobotStatus robot_status_;

    // 整个 Ros2Channel 是否处于运行状态
    std::atomic_bool running_{false};

    // 状态上传线程运行状态
    std::atomic_bool status_upload_running_{false};

    std::thread spin_thread_;
    std::thread status_upload_thread_;

    // 防止多个线程同时调用 stop()
    std::mutex stop_mutex_;

    // robot_status_pub_ 的发布与销毁保护
    std::mutex robot_status_pub_mutex_;

    // generic_publishers_ 和 node_->create_publisher() 保护
    std::mutex generic_pub_mutex_;

    // 状态上传线程等待与唤醒
    std::mutex              status_upload_wait_mutex_;
    std::condition_variable status_upload_cv_;

    // RobotStatus 数据保护
    mutable std::mutex robot_status_mutex_;
    mutable std::mutex cmd_vel_pub_mutex_;
    mutable std::mutex audio_play_pub_mutex_;

    // 回调函数保护
    std::mutex callback_mutex_;

    mutable std::mutex waypoints_mutex_;
    mutable std::mutex goal_handle_mutex_;

    std::vector<NavigationWaypoint> waypoints_;

    std::atomic<bool> waypoint_task_running_{false};

    std::atomic<uint32_t> current_waypoint_index_{0};

    CommandCallback command_callback_;

    std::map<std::string, rclcpp::Publisher<std_msgs::msg::String>::SharedPtr>
        generic_publishers_;
};
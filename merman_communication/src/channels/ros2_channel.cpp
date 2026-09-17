#include <chrono>
#include <cmath>
#include <cstddef>
#include <iostream>
#include <nlohmann/json.hpp>
#include <stdexcept>
#include <utility>

#include "channels/ros2_channel.h"
#include "merman_communication_interface/merman_communication_interface.h"
#include "merman_logger/merman_logger.hpp"


namespace {
rclcpp::QoS CreateRos2Qos(const CommunicationConfig& config) {
    if (config.qos_depth <= 0) {
        throw std::invalid_argument("ROS2 QoS depth must be greater than 0");
    }

    rclcpp::QoS qos(
        rclcpp::KeepLast(static_cast<std::size_t>(config.qos_depth)));

    if (config.qos_reliability == "reliable") {
        qos.reliable();
    } else if (config.qos_reliability == "best_effort") {
        qos.best_effort();
    } else {
        throw std::invalid_argument(
            "unsupported ROS2 QoS reliability: " + config.qos_reliability);
    }

    return qos;
}

void ValidateRos2Config(const CommunicationConfig& config) {
    if (config.follow_waypoints_wait_timeout_sec < 0) {
        throw std::invalid_argument(
            "follow_waypoints_wait_timeout_sec must not be negative");
    }

    if (config.status_upload_period_ms <= 0) {
        throw std::invalid_argument(
            "status_upload_period_ms must be greater than 0");
    }

    if (config.pose_log_throttle_ms < 0) {
        throw std::invalid_argument(
            "pose_log_throttle_ms must not be negative");
    }

    (void)CreateRos2Qos(config);
}
}

Ros2Channel::Ros2Channel(MermanCommunicationInterface& communication)
    : communication_(communication) {}

Ros2Channel::~Ros2Channel() {
    // 必须调用。
    // 即使外部忘记显式 stop()，也能释放线程和 ROS2 对象。
    stop();
}

bool Ros2Channel::init(const CommunicationConfig& config) {
    if (node_ || executor_ || running_.load()) {
        stop();
    }

    config_ = config;
    owns_rclcpp_init_ = false;

    try {
        ValidateRos2Config(config_);

        if (!rclcpp::ok()) {
            int    argc = 0;
            char** argv = nullptr;

            rclcpp::init(argc, argv);
            owns_rclcpp_init_ = true;
        }

        node_ = std::make_shared<rclcpp::Node>(config_.ros_node_name);

        const auto qos = CreateRos2Qos(config_);

        robot_status_pub_ = node_->create_publisher<std_msgs::msg::String>(
            config_.robot_status_topic, qos);

        // cmd_vel_pub_ = node_->create_publisher<std_msgs::msg::String>(
        //     config_.robot_status_topic, qos);

        cmd_vel_pub_ =
            node_->create_publisher<geometry_msgs::msg::Twist>(
                config_.pub_cmd_vel_topic, qos);

        audio_play_pub_ = node_->create_publisher<AudioPlayMsg>(
            config_.pub_audio_play_topic, qos);

        // local_cmd_sub_ = node_->create_subscription<std_msgs::msg::String>(
        //     config_.local_cmd_topic, qos,
        //     [this](const std_msgs::msg::String::SharedPtr msg) {
        //         onCommandMessage(msg, CommandSource::LOCAL);
        //     });

        // cloud_cmd_sub_ = node_->create_subscription<std_msgs::msg::String>(
        //     config_.cloud_cmd_topic, qos,
        //     [this](const std_msgs::msg::String::SharedPtr msg) {
        //         onCommandMessage(msg, CommandSource::CLOUD);
        //     });

        pose_sub_ = node_->create_subscription<geometry_msgs::msg::PoseStamped>(
            config_.sub_pose_topic, qos,
            [this](const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
                PoseCallback(msg);
            });

        cmd_vel_sub_ = node_->create_subscription<geometry_msgs::msg::Twist>(
            config_.sub_cmd_vel_topic,
            qos,
            [this](const geometry_msgs::msg::Twist::SharedPtr msg) {
                CmdVelCallback(msg);
            });

        audio_done_sub_ = node_->create_subscription<std_msgs::msg::Int32>(
            config_.sub_audio_done_topic,
            qos,
            [this](const std_msgs::msg::Int32::SharedPtr msg) {
                AudioDoneCallback(msg);
            });

        pause_task_callback_group_ =
            node_->create_callback_group(
                rclcpp::CallbackGroupType::MutuallyExclusive);

        rclcpp::SubscriptionOptions pause_task_options;

        pause_task_options.callback_group =
            pause_task_callback_group_;

        pause_task_sub_ =
            node_->create_subscription<std_msgs::msg::Bool>(
                config_.sub_pause_task_topic,
                qos,
                [this](const std_msgs::msg::Bool::SharedPtr msg) {
                    PauseTaskCallback(msg);
                },
                pause_task_options);

        // audio_done_sub_ =
        //     node_->create_subscription<std_msgs::msg::Int32>(
        //         config_.sub_audio_done_topic,
        //         qos,
        //         std::bind(
        //             &Ros2Channel::AudioDoneCallback,
        //             this,
        //             std::placeholders::_1));

        start_task_service_ = node_->create_service<Trigger>(
            config_.start_task_service,
            [this](const std::shared_ptr<Trigger::Request> request,
                   std::shared_ptr<Trigger::Response>      response) {
                HandleStartTask(request, response);
            });

        stop_task_service_ = node_->create_service<Trigger>(
            config_.stop_task_service,
            [this](const std::shared_ptr<Trigger::Request> request,
                   std::shared_ptr<Trigger::Response>      response) {
                HandleStopTask(request, response);
            });

        follow_waypoints_client_ =
            rclcpp_action::create_client<FollowWaypoints>(
                node_, config_.follow_waypoints_action);

        pose_arrived_client_ =
            node_->create_client<PoseArrivedSrv>(
                config_.pose_arrived_service);

        RCLCPP_INFO(node_->get_logger(), "Ros2Channel initialized.");

        RCLCPP_INFO(node_->get_logger(), "robot_status_topic: %s",
                    config_.robot_status_topic.c_str());

        RCLCPP_INFO(node_->get_logger(), "local_cmd_topic: %s",
                    config_.local_cmd_topic.c_str());

        RCLCPP_INFO(node_->get_logger(), "cloud_cmd_topic: %s",
                    config_.cloud_cmd_topic.c_str());

        RCLCPP_INFO(node_->get_logger(), "sub_pose_topic: %s",
                    config_.sub_pose_topic.c_str());

        RCLCPP_INFO(node_->get_logger(),
                    "任务接口初始化完成: start=%s, stop=%s",
                    config_.start_task_service.c_str(),
                    config_.stop_task_service.c_str());

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Ros2Channel] init exception: " << e.what() << std::endl;

        stop();
        return false;

    } catch (...) {
        std::cerr << "[Ros2Channel] init unknown exception" << std::endl;

        stop();
        return false;
    }
}

bool Ros2Channel::start() {
    if (running_.load()) {
        return true;
    }

    if (!node_ || !rclcpp::ok()) {
        std::cerr << "[Ros2Channel] start failed: "
                  << "node is null or rclcpp is not running" << std::endl;

        return false;
    }

    try {
        executor_ = std::make_shared<rclcpp::executors::MultiThreadedExecutor>(
            rclcpp::ExecutorOptions(), 2);

        executor_->add_node(node_);

        running_.store(true);

        /*
         * 在线程中保存 executor 和 node 的 shared_ptr 副本。
         * 可以避免直接在 spin 线程里反复访问成员变量。
         */
        const auto executor = executor_;
        const auto node = node_;

        spin_thread_ = std::thread([executor, node]() {
            try {
                RCLCPP_INFO(node->get_logger(),
                            "Ros2Channel executor started.");

                executor->spin();

                RCLCPP_INFO(node->get_logger(),
                            "Ros2Channel executor stopped.");

            } catch (const std::exception& e) {
                RCLCPP_ERROR(node->get_logger(), "Executor spin exception: %s",
                             e.what());

            } catch (...) {
                RCLCPP_ERROR(node->get_logger(),
                             "Executor spin unknown exception.");
            }
        });

        StartStatusUploadThread();

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Ros2Channel] start exception: " << e.what() << std::endl;

        /*
         * start 中途失败时也要释放已经创建的 executor、
         * thread、publisher 和 subscription。
         */
        stop();
        return false;

    } catch (...) {
        std::cerr << "[Ros2Channel] start unknown exception" << std::endl;

        stop();
        return false;
    }
}

void Ros2Channel::stop() {
    /*
     * 不使用 running_.compare_exchange_strong() 提前返回。
     *
     * 因为可能出现：
     * init() 成功创建 publisher/subscription，
     * 但 start() 尚未调用或 start() 中途失败。
     *
     * 此时 running_ 为 false，但仍然需要清理资源。
     */
    std::lock_guard<std::mutex> stop_lock(stop_mutex_);

    // 1. 立刻禁止新的 publish 操作
    running_.store(false);

    // 禁止业务层认为导航任务仍可继续操作
    waypoint_task_running_.store(false);

    // 2. 停止主动发布状态的线程
    StopStatusUploadThread();

    /*
     * 3. 在 executor、node、Action Client 仍然存活时，
     * 请求 Nav2 取消当前导航。
     *
     * 这一步必须放在 executor_->cancel() 之前。
     */
    CancelWaypointNavigation();

    // 4. 通知 executor 停止 spin
    if (executor_) {
        try {
            executor_->cancel();
        } catch (const std::exception& e) {
            std::cerr << "[Ros2Channel] executor cancel exception: " << e.what()
                      << std::endl;
        } catch (...) {
            std::cerr << "[Ros2Channel] executor cancel "
                      << "unknown exception" << std::endl;
        }
    }

    // 5. 等待 ROS2 spin 线程退出
    if (spin_thread_.joinable()) {
        if (spin_thread_.get_id() != std::this_thread::get_id()) {
            spin_thread_.join();

        } else {
            /*
             * 正常情况下 stop() 应当由 main/control 线程调用，
             * 不应该在 subscription callback 中直接调用。
             */
            std::cerr << "[Ros2Channel] stop() called from "
                      << "spin thread; cannot join itself" << std::endl;
        }
    }

    /*
     * 到这里后：
     * - 状态发布线程已退出
     * - executor spin 已退出
     * - subscription callback 不会继续执行
     */

    // 6. 从 executor 移除 node
    if (executor_ && node_) {
        try {
            executor_->remove_node(node_);
        } catch (const std::exception& e) {
            std::cerr << "[Ros2Channel] remove_node exception: " << e.what()
                      << std::endl;
        } catch (...) {
            std::cerr << "[Ros2Channel] remove_node "
                      << "unknown exception" << std::endl;
        }
    }

    // 7. 释放 Action GoalHandle 和 Action Client
    {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);

        /*
         * 先释放 GoalHandle，再释放 Client。
         *
         * GoalHandle 不是独立的 DDS Entity，
         * 但它保存了目标状态以及 feedback/result 回调。
         */
        follow_waypoints_goal_handle_.reset();

        /*
         * Action Client 内部包含：
         * - send_goal service client
         * - get_result service client
         * - cancel_goal service client
         * - feedback subscription
         * - status subscription
         *
         * 因此必须在 node_ 和 rclcpp::shutdown() 之前释放。
         */
        follow_waypoints_client_.reset();
    }

    current_waypoint_index_.store(0);

    // 8. 释放所有 subscription service
    // local_cmd_sub_.reset();
    // cloud_cmd_sub_.reset();
    pose_sub_.reset();
    cmd_vel_sub_.reset();
    audio_done_sub_.reset();
    pause_task_sub_.reset();

    // subscription 已经销毁之后，再销毁 callback group
    pause_task_callback_group_.reset();

    // 9. 释放 services
    start_task_service_.reset();
    stop_task_service_.reset();

    /*
     * 如果后面增加了其他 subscription，
     * 也必须在这里 reset：
     *
     * battery_sub_.reset();
     * velocity_sub_.reset();
     * emergency_stop_sub_.reset();
     */

    // 10. 释放固定状态 publisher
    {
        std::lock_guard<std::mutex> lock(robot_status_pub_mutex_);

        robot_status_pub_.reset();
    }

    {
        std::lock_guard<std::mutex> lock(cmd_vel_pub_mutex_);

        cmd_vel_pub_.reset();
    }

    {
        std::lock_guard<std::mutex> lock(audio_play_pub_mutex_);
        audio_play_pub_.reset();
    }

    {
        std::lock_guard<std::mutex> lock(pose_arrived_client_mutex_);
        pose_arrived_client_.reset();
    }

    /*
     * 11. 释放 publishJson() 动态创建的所有 publisher。
     *
     * 这一项非常重要。
     * 你原来的代码没有清空 generic_publishers_，
     * 所以这些 publisher 会一直存活到 Ros2Channel 析构。
     *
     * 如果已经先调用 rclcpp::shutdown()，
     * 后续 generic publisher 析构时就可能出现：
     *
     * cannot publish data
     * Failed to delete datawriter/datareader
     */
    {
        std::lock_guard<std::mutex> lock(generic_pub_mutex_);

        generic_publishers_.clear();
    }

    // 12. executor 已经不再持有 node
    executor_.reset();

    // 13. 所有 publisher/subscription 都释放后再释放 node
    node_.reset();

    /*
     * 14. 最后关闭 ROS2。
     *
     * 只有确实由 Ros2Channel 调用了 rclcpp::init()，
     * 才允许在这里 shutdown。
     */
    if (owns_rclcpp_init_ && rclcpp::ok()) {
        try {
            rclcpp::shutdown();
        } catch (const std::exception& e) {
            std::cerr << "[Ros2Channel] rclcpp shutdown exception: " << e.what()
                      << std::endl;
        } catch (...) {
            std::cerr << "[Ros2Channel] rclcpp shutdown "
                      << "unknown exception" << std::endl;
        }
    }

    owns_rclcpp_init_ = false;
}

bool Ros2Channel::PublishRobotStatus(const RobotStatus& robot_status) {
    /*
     * 不需要检查 status_upload_running_。
     *
     * publishRobotStatus 是一个正常发布接口，
     * 它既可以被状态上传线程调用，
     * 也可以被业务层直接调用。
     */
    if (!running_.load()) {
        return false;
    }

    /*
     * stop() reset robot_status_pub_ 时也使用这把锁。
     * 因此 publisher 不会在 publish 过程中被销毁。
     */
    std::lock_guard<std::mutex> lock(robot_status_pub_mutex_);

    // 获取锁后必须重新检查
    if (!running_.load() || !rclcpp::ok() || !robot_status_pub_) {
        return false;
    }

    try {
        std_msgs::msg::String msg;
        msg.data = RobotStatusToJson(robot_status);

        robot_status_pub_->publish(msg);

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Ros2Channel] publishRobotStatus exception: " << e.what()
                  << std::endl;

        return false;

    } catch (...) {
        std::cerr << "[Ros2Channel] publishRobotStatus "
                  << "unknown exception" << std::endl;

        return false;
    }
}

bool Ros2Channel::PublishCmdVel(double linear_x, double angular_z) {
    /*
     * 拒绝 NaN 和无穷大，避免向底盘发送非法速度。
     */
    if (!std::isfinite(linear_x) || !std::isfinite(angular_z)) {
        std::cerr << "[Ros2Channel] PublishCmdVel invalid velocity: "
                  << "linear_x=" << linear_x
                  << ", angular_z=" << angular_z
                  << std::endl;
        return false;
    }

    if (!running_.load()) {
        return false;
    }

    /*
     * stop() reset cmd_vel_pub_ 时也要使用这把锁，
     * 保证 publisher 不会在 publish 过程中被销毁。
     */
    std::lock_guard<std::mutex> lock(cmd_vel_pub_mutex_);

    // 获取锁之后重新检查
    if (!running_.load() || !rclcpp::ok() || !cmd_vel_pub_) {
        return false;
    }

    try {
        geometry_msgs::msg::Twist msg;

        // 差速机器人通常只使用这两个分量
        msg.linear.x = linear_x;
        msg.linear.y = 0.0;
        msg.linear.z = 0.0;

        msg.angular.x = 0.0;
        msg.angular.y = 0.0;
        msg.angular.z = angular_z;

        cmd_vel_pub_->publish(msg);

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Ros2Channel] PublishCmdVel exception: "
                  << e.what() << std::endl;
        return false;

    } catch (...) {
        std::cerr << "[Ros2Channel] PublishCmdVel unknown exception"
                  << std::endl;
        return false;
    }
}

/*
bool Ros2Channel::PoseArrived(int32_t pose_id, bool play_audio) {
    if (!running_.load() || !rclcpp::ok()) {
        return false;
    }

    rclcpp::Client<PoseArrivedSrv>::SharedPtr client;

    {
        std::lock_guard<std::mutex> lock(pose_arrived_client_mutex_);
        client = pose_arrived_client_;
    }

    if (!client) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "PoseArrived client is not initialized");
        return false;
    }

    // 不建议在这里长时间 wait_for_service，否则可能阻塞业务线程
    if (!client->service_is_ready()) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PoseArrived service is not ready");
        return false;
    }

    auto request = std::make_shared<PoseArrivedSrv::Request>();

    request->pose_id = pose_id;
    request->play_audio = play_audio;

    // 避免异步回调直接捕获 this，防止对象销毁后访问悬空指针
    const auto logger = node_->get_logger();

    try {
        client->async_send_request(
            request,
            [logger, pose_id](
                rclcpp::Client<PoseArrivedSrv>::SharedFuture future) {
                try {
                    const auto response = future.get();

                    if (!response) {
                        RCLCPP_ERROR(
                            logger,
                            "PoseArrived returned an empty response");
                        return;
                    }

                    if (response->success) {
                        RCLCPP_INFO(
                            logger,
                            "PoseArrived succeeded, pose_id=%d",
                            pose_id);
                    } else {
                        RCLCPP_WARN(
                            logger,
                            "PoseArrived failed, pose_id=%d, message=%s",
                            pose_id,
                            response->message.c_str());
                    }
                } catch (const std::exception& e) {
                    RCLCPP_ERROR(
                        logger,
                        "PoseArrived response exception: %s",
                        e.what());
                }
            });

        AudioPlayMsg msg;

        msg.pose_id = pose_id;
        msg.play_audio = play_audio;

        {
            std::lock_guard<std::mutex> lock(audio_play_pub_mutex_);
            audio_play_pub_->publish(msg);
        }


        // 表示请求成功提交，不代表服务端执行成功
        return true;

    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "PoseArrived async_send_request exception: %s",
            e.what());
        return false;
    }
}
*/

bool Ros2Channel::PoseArrived(int32_t pose_id, bool play_audio) {
    if (pose_id < 0) {
        return false;
    }

    /*
     * 防止执行过程中 stop() 销毁 node、client 和 publisher。
     *
     * stop() 本身也持有 stop_mutex_，所以：
     * - PoseArrived 执行期间 stop() 会等待
     * - stop() 执行期间 PoseArrived 会等待，之后检查到 running_ == false
     */
    std::lock_guard<std::mutex> lifecycle_lock(stop_mutex_);

    if (!running_.load() || !rclcpp::ok() || !node_) {
        return false;
    }

    rclcpp::Client<PoseArrivedSrv>::SharedPtr client;

    {
        std::lock_guard<std::mutex> lock(pose_arrived_client_mutex_);
        client = pose_arrived_client_;
    }

    if (!client) {
        RCLCPP_ERROR(
            node_->get_logger(),
            "PoseArrived client is not initialized");

        return false;
    }

    /*
     * 不在这里长时间 wait_for_service，
     * 避免阻塞 Task 业务线程。
     */
    if (!client->service_is_ready()) {
        RCLCPP_WARN(
            node_->get_logger(),
            "PoseArrived service is not ready");

        // return false;
    }

    auto request = std::make_shared<PoseArrivedSrv::Request>();

    request->pose_id = pose_id;
    request->play_audio = play_audio;

    const auto logger = node_->get_logger();

    /*
     * 先向上肢提交到点通知。
     */
    try {
        client->async_send_request(
            request,
            [logger, pose_id](
                rclcpp::Client<PoseArrivedSrv>::SharedFuture future) {
                try {
                    const auto response = future.get();

                    if (!response) {
                        RCLCPP_ERROR(
                            logger,
                            "PoseArrived returned an empty response, "
                            "pose_id=%d",
                            pose_id);

                        return;
                    }

                    if (response->success) {
                        RCLCPP_INFO(
                            logger,
                            "PoseArrived succeeded, pose_id=%d",
                            pose_id);
                    } else {
                        RCLCPP_WARN(
                            logger,
                            "PoseArrived failed, "
                            "pose_id=%d, message=%s",
                            pose_id,
                            response->message.c_str());
                    }

                } catch (const std::exception& e) {
                    RCLCPP_ERROR(
                        logger,
                        "PoseArrived response exception: %s",
                        e.what());

                } catch (...) {
                    RCLCPP_ERROR(
                        logger,
                        "PoseArrived response unknown exception");
                }
            });

    } catch (const std::exception& e) {
        RCLCPP_ERROR(
            logger,
            "PoseArrived async_send_request exception: %s",
            e.what());

        return false;

    } catch (...) {
        RCLCPP_ERROR(
            logger,
            "PoseArrived async_send_request unknown exception");

        return false;
    }

    /*
     * 不需要播放音频时，到这里已经完成消息提交。
     */
    if (!play_audio) {
        return true;
    }

    /*
     * 需要播放音频时，再发布 audio play Topic。
     */
    {
        std::lock_guard<std::mutex> lock(audio_play_pub_mutex_);

        /*
         * 即使有 lifecycle_lock，也建议检查空指针，
         * 防止 init() 中途失败或者 Publisher 没有成功创建。
         */
        if (!running_.load() ||
            !rclcpp::ok() ||
            !audio_play_pub_) {
            RCLCPP_ERROR(
                logger,
                "Audio play publisher is not available, pose_id=%d",
                pose_id);

            return false;
        }

        try {
            AudioPlayMsg msg;

            msg.data = pose_id;

            audio_play_pub_->publish(msg);

            RCLCPP_INFO(
                logger,
                "Audio play message published, pose_id=%d",
                pose_id);

        } catch (const std::exception& e) {
            RCLCPP_ERROR(
                logger,
                "Audio play publish exception: %s",
                e.what());

            return false;

        } catch (...) {
            RCLCPP_ERROR(
                logger,
                "Audio play publish unknown exception");

            return false;
        }
    }

    /*
     * 这里只表示：
     * 1. 上肢 Service 请求已提交
     * 2. 音频 Topic 已发布
     *
     * 不表示上肢动作或音频播放已经完成。
     */
    return true;
}

bool Ros2Channel::StopRobot() {
    return PublishCmdVel(0.0, 0.0);
}

void Ros2Channel::PoseCallback(
    const geometry_msgs::msg::PoseStamped::SharedPtr msg) {
    if (!msg) {
        return;
    }

    const auto& position = msg->pose.position;
    const auto& orientation = msg->pose.orientation;

    // 四元数转换为绕 Z 轴旋转角 yaw
    const double siny_cosp =
        2.0 * (orientation.w * orientation.z + orientation.x * orientation.y);

    const double cosy_cosp = 1.0 - 2.0 * (orientation.y * orientation.y +
                                          orientation.z * orientation.z);

    const double theta = std::atan2(siny_cosp, cosy_cosp);

    {
        std::lock_guard<std::mutex> lock(robot_status_mutex_);

        robot_status_.pose.x = position.x;

        robot_status_.pose.y = position.y;

        robot_status_.pose.theta = theta;
    }

    /*
     * 不建议每个 Pose 回调都 std::cout。
     * 高频日志会影响 executor，并造成终端大量输出。
     *
     * 调试时可以使用节流日志：
     */
    if (node_) {
        RCLCPP_DEBUG_THROTTLE(
            node_->get_logger(), *node_->get_clock(),
            config_.pose_log_throttle_ms,
            "pose: x=%.3f, y=%.3f, theta=%.3f", position.x,
            position.y, theta);
    }
}

void Ros2Channel::AudioDoneCallback(const std_msgs::msg::Int32::SharedPtr msg) {
    if (!msg) {
        return;
    }

    const int32_t audio_id = msg->data;

    RCLCPP_INFO(node_->get_logger(), "收到 %s 消息: %d",
                config_.sub_audio_done_topic.c_str(), audio_id);

    if (!communication_.get_task_running()) {
        RCLCPP_WARN(node_->get_logger(),
                    "当前没有正在执行的导航任务，忽略 audio_done=%d", audio_id);
        return;
    }

    if (audio_id == -1) {
        RCLCPP_INFO(node_->get_logger(), "收到跳过当前任务点指令");
    } else {
        RCLCPP_INFO(node_->get_logger(),
                    "音频 %d 播放完成，准备导航到下一个任务点", audio_id);
    }

    // navigateToNextWaypoint();
    communication_.AudioDone(audio_id);
}

void Ros2Channel::CmdVelCallback(
    const geometry_msgs::msg::Twist::SharedPtr msg) {
    if (!msg) {
        return;
    }

    {
        std::lock_guard<std::mutex> lock(robot_status_mutex_);

        robot_status_.motion.linear_velocity = msg->linear.x;

        robot_status_.motion.angular_velocity = msg->angular.z;
    }
}

void Ros2Channel::HandleStartTask(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response>      response) {
    (void)request;

    if (waypoint_task_running_.load() || communication_.get_task_running()) {
        response->success = false;
        response->message = "已有任务正在执行";

        return;
    }

    communication_.StartTask();

    response->success = true;

    // 注意：这里只表示成功提交，不代表导航完成
    response->message = "导览任务已提交给Nav2";

    RCLCPP_INFO(node_->get_logger(), "任务状态: %s", response->message.c_str());
    // ML_INFO("任务状态: {}", response->message);
}

void Ros2Channel::HandleStopTask(
    const std::shared_ptr<Trigger::Request> request,
    std::shared_ptr<Trigger::Response>      response) {
    (void)request;

    communication_.StopTask();

    // NavigateGoalHandle::SharedPtr goal_to_cancel;
    // bool                          was_running = false;

    // {
    //     std::lock_guard<std::mutex> lock(task_mutex_);

    //     was_running = task_running_;

    //     /*
    //      * 使所有旧的 action 回调立即失效。
    //      */
    //     ++task_generation_;

    //     task_running_ = false;
    //     task_state_ = TourTaskState::STOPPED;

    //     current_audio_id_ = -1;

    //     goal_to_cancel = active_nav_goal_;
    //     active_nav_goal_.reset();
    // }

    // if (goal_to_cancel && navigate_to_pose_client_) {
    //     navigate_to_pose_client_->async_cancel_goal(goal_to_cancel);
    // }

    // if (was_running) {
    //     publishNavEnd(false, "stopped_by_service");

    //     response->success = true;
    //     response->message = "任务已停止";
    // } else {
    //     response->success = true;
    //     response->message = "当前没有运行中的任务";
    // }

    RCLCPP_INFO(node_->get_logger(), "收到 stop_task，任务已停止");
}

void Ros2Channel::PauseTaskCallback(
    const std_msgs::msg::Bool::SharedPtr msg) {
    if (!msg) {
        return;
    }

    // 上层协议：
    // false = 暂停
    // true  = 恢复
    const bool resume_task = msg->data;

    RCLCPP_INFO(node_->get_logger(),
                "收到 %s 消息: pause_task=%s",
                config_.sub_pause_task_topic.c_str(),
                resume_task ? "true" : "false");

    if (!communication_.get_task_running()) {
        RCLCPP_WARN(node_->get_logger(),
                    "当前没有正在执行的导航任务，忽略 pause_task=%s",
                    resume_task ? "true" : "false");
        return;
    }

    if (resume_task) {
        // true = 恢复
        RCLCPP_INFO(node_->get_logger(), "恢复当前任务");
    } else {
        // false = 暂停
        RCLCPP_INFO(node_->get_logger(), "暂停当前任务");
    }

    /*
     * communication_.PauseTask() 原有内部语义：
     * true  = 暂停
     * false = 恢复
     *
     * 因此外部协议需要取反。
     */
    communication_.PauseTask(!resume_task);
}

bool Ros2Channel::set_waypoints(
    std::vector<NavigationWaypoint>& waypoints) {

    {
        std::lock_guard<std::mutex> lock(waypoints_mutex_);
        waypoints_ = waypoints;
    }

    RCLCPP_INFO(node_->get_logger(), "已加载 %zu 个导航任务点",
                waypoints.size());

    if (!StartWaypointNavigation()) {
        RCLCPP_ERROR(node_->get_logger(), "任务点加载成功，但启动导航失败");
        return false;
    }

    return true;
}

bool Ros2Channel::get_pose(Pose2D& pose) {
    {
        std::lock_guard<std::mutex> lock(robot_status_mutex_);

        pose.x = robot_status_.pose.x;

        pose.y = robot_status_.pose.y;

        pose.theta = robot_status_.pose.theta;
    }
    return true;
}

geometry_msgs::msg::PoseStamped Ros2Channel::WaypointToPose(
    const NavigationWaypoint& waypoint) const {
    geometry_msgs::msg::PoseStamped pose;

    pose.header.frame_id = config_.navigation_frame;
    pose.header.stamp = node_->now();

    pose.pose.position.x = waypoint.x;
    pose.pose.position.y = waypoint.y;
    pose.pose.position.z = 0.0;

    const double normalized_theta =
        std::atan2(std::sin(waypoint.theta), std::cos(waypoint.theta));

    const double half_theta = normalized_theta * 0.5;

    pose.pose.orientation.x = 0.0;
    pose.pose.orientation.y = 0.0;
    pose.pose.orientation.z = std::sin(half_theta);

    pose.pose.orientation.w = std::cos(half_theta);

    return pose;
}

bool Ros2Channel::StartWaypointNavigation() {
    if (!follow_waypoints_client_) {
        RCLCPP_ERROR(node_->get_logger(),
                     "FollowWaypoints Action客户端未初始化");

        return false;
    }

    bool expected = false;

    if (!waypoint_task_running_.compare_exchange_strong(expected, true)) {
        RCLCPP_WARN(node_->get_logger(), "已有导航任务正在执行");

        return false;
    }

    std::vector<NavigationWaypoint> waypoints;

    {
        std::lock_guard<std::mutex> lock(waypoints_mutex_);

        waypoints = waypoints_;
    }

    if (waypoints.empty()) {
        waypoint_task_running_.store(false);

        RCLCPP_WARN(node_->get_logger(), "没有任务点，无法启动导航");

        return false;
    }

    if (!follow_waypoints_client_->wait_for_action_server(
            std::chrono::seconds(
                config_.follow_waypoints_wait_timeout_sec))) {
        waypoint_task_running_.store(false);

        RCLCPP_ERROR(node_->get_logger(), "Nav2 Action服务器不可用: %s",
                     config_.follow_waypoints_action.c_str());

        return false;
    }

    FollowWaypoints::Goal goal;

    goal.poses.reserve(waypoints.size());

    for (const auto& waypoint : waypoints) {
        goal.poses.push_back(WaypointToPose(waypoint));
    }

    current_waypoint_index_.store(0);

    rclcpp_action::Client<FollowWaypoints>::SendGoalOptions options;

    options.goal_response_callback =
        [this](FollowWaypointsGoalHandle::SharedPtr goal_handle) {
            HandleWaypointGoalResponse(goal_handle);
        };

    options.feedback_callback =
        [this](
            FollowWaypointsGoalHandle::SharedPtr                   goal_handle,
            const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
            HandleWaypointFeedback(goal_handle, feedback);
        };

    options.result_callback =
        [this](const FollowWaypointsGoalHandle::WrappedResult& result) {
            HandleWaypointResult(result);
        };

    try {
        follow_waypoints_client_->async_send_goal(goal, options);
    } catch (const std::exception& exception) {
        waypoint_task_running_.store(false);

        RCLCPP_ERROR(node_->get_logger(), "发送导航任务异常: %s",
                     exception.what());

        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "已向Nav2提交导航任务，共 %zu 个点",
                waypoints.size());

    return true;
}

// 处理目标是否被Nav2接受
void Ros2Channel::HandleWaypointGoalResponse(
    FollowWaypointsGoalHandle::SharedPtr goal_handle) {
    if (!goal_handle) {
        waypoint_task_running_.store(false);

        RCLCPP_ERROR(node_->get_logger(), "Nav2拒绝了FollowWaypoints任务");

        return;
    }

    {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);

        follow_waypoints_goal_handle_ = goal_handle;
    }

    RCLCPP_INFO(node_->get_logger(), "Nav2已接受FollowWaypoints任务");

    // 这里可以同步给控制模块：
    // communication_.set_task_running(true);
}

// 获取当前执行到哪个点
void Ros2Channel::HandleWaypointFeedback(
    FollowWaypointsGoalHandle::SharedPtr,
    const std::shared_ptr<const FollowWaypoints::Feedback> feedback) {
    if (!feedback) {
        return;
    }

    const uint32_t previous_index =
        current_waypoint_index_.exchange(feedback->current_waypoint);

    if (previous_index != feedback->current_waypoint) {
        RCLCPP_INFO(node_->get_logger(), "开始执行第 %u 个导航点",
                    feedback->current_waypoint);
    }
}

// 处理最终结果
void Ros2Channel::HandleWaypointResult(
    const FollowWaypointsGoalHandle::WrappedResult& result) {
    waypoint_task_running_.store(false);

    {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);

        follow_waypoints_goal_handle_.reset();
    }

    std::string nav2_status = "";

    switch (result.code) {
        case rclcpp_action::ResultCode::SUCCEEDED:
            if (result.result && !result.result->missed_waypoints.empty()) {
                RCLCPP_WARN(node_->get_logger(),
                            "导航任务结束，但有 %zu 个任务点未到达",
                            result.result->missed_waypoints.size());

                for (const auto index : result.result->missed_waypoints) {
                    RCLCPP_WARN(node_->get_logger(), "未到达任务点下标: %d",
                                index);
                }
            } else {
                RCLCPP_INFO(node_->get_logger(), "所有导航任务点执行完成");
            }
            nav2_status = "succeeded";

            break;

        case rclcpp_action::ResultCode::ABORTED:
            RCLCPP_ERROR(node_->get_logger(), "导航任务执行失败");

            break;

        case rclcpp_action::ResultCode::CANCELED:
            RCLCPP_WARN(node_->get_logger(), "导航任务已取消");

            break;

        default:
            RCLCPP_ERROR(node_->get_logger(), "导航任务返回未知结果");

            break;
    }

    communication_.set_nav2_status(nav2_status);

    // 同步业务状态
    // communication_.set_task_running(false);
}

bool Ros2Channel::CancelWaypointNavigation() {
    if (!follow_waypoints_client_) {
        return false;
    }

    FollowWaypointsGoalHandle::SharedPtr goal_handle;

    {
        std::lock_guard<std::mutex> lock(goal_handle_mutex_);

        goal_handle = follow_waypoints_goal_handle_;
    }

    if (!goal_handle) {
        RCLCPP_WARN(node_->get_logger(), "当前没有可取消的导航任务");

        return false;
    }

    try {
        follow_waypoints_client_->async_cancel_goal(goal_handle);
    } catch (const std::exception& exception) {
        RCLCPP_ERROR(node_->get_logger(), "取消导航任务失败: %s",
                     exception.what());

        return false;
    }

    RCLCPP_INFO(node_->get_logger(), "已向Nav2发送取消导航请求");

    return true;
}

std::string Ros2Channel::RobotStatusToJson(const RobotStatus& robot_status) {
    if (!robot_status.raw_json.empty()) {
        return robot_status.raw_json;
    }

    nlohmann::ordered_json json_data = {
        {"pose",
         {{"x", robot_status.pose.x},
          {"y", robot_status.pose.y},
          {"theta", robot_status.pose.theta}}},
        {"map_id", robot_status.map_id},
        {"motion",
         {{"linear_velocity", robot_status.motion.linear_velocity},
          {"angular_velocity", robot_status.motion.angular_velocity}}}};

    return json_data.dump();
}

RobotStatus Ros2Channel::get_robot_status() const {
    std::lock_guard<std::mutex> lock(robot_status_mutex_);

    return robot_status_;
}

void Ros2Channel::StartStatusUploadThread() {
    bool expected = false;

    if (!status_upload_running_.compare_exchange_strong(expected, true)) {
        // 已经启动
        return;
    }

    try {
        status_upload_thread_ = std::thread([this]() {
            std::unique_lock<std::mutex> wait_lock(status_upload_wait_mutex_);

            while (status_upload_running_.load() && running_.load()) {
                /*
                 * 不持有等待锁进行：
                 * - RobotStatus 拷贝
                 * - JSON 转换
                 * - ROS2 publish
                 */
                wait_lock.unlock();

                RobotStatus status = get_robot_status();

                /*
                 * 每次发送时更新 timestamp。
                 * 如果业务层必须自己提供时间戳，
                 * 可以删除这段赋值。
                 */
                const auto now =
                    std::chrono::system_clock::now().time_since_epoch();

                status.timestamp_ms =
                    std::chrono::duration_cast<std::chrono::milliseconds>(now)
                        .count();

                PublishRobotStatus(status);

                wait_lock.lock();

                /*
                 * StopStatusUploadThread() 会 notify_all()，
                 * 所以退出时不用额外等待完整的上报周期。
                 */
                status_upload_cv_.wait_for(
                    wait_lock,
                    std::chrono::milliseconds(
                        config_.status_upload_period_ms),
                    [this]() {
                        return !status_upload_running_.load() ||
                               !running_.load();
                    });
            }
        });

    } catch (...) {
        status_upload_running_.store(false);
        throw;
    }
}

void Ros2Channel::StopStatusUploadThread() {
    status_upload_running_.store(false);

    // 立即唤醒 wait_for()
    status_upload_cv_.notify_all();

    if (status_upload_thread_.joinable()) {
        if (status_upload_thread_.get_id() != std::this_thread::get_id()) {
            status_upload_thread_.join();

        } else {
            std::cerr << "[Ros2Channel] cannot join status upload "
                      << "thread from itself" << std::endl;
        }
    }
}

bool Ros2Channel::publishJson(const std::string& topic_name,
                              const std::string& json) {
    if (topic_name.empty()) {
        return false;
    }

    if (!running_.load()) {
        return false;
    }

    /*
     * 这把锁同时保护：
     * - node_->create_publisher()
     * - generic_publishers_
     * - publisher->publish()
     * - stop() 中 generic_publishers_.clear()
     */
    std::lock_guard<std::mutex> lock(generic_pub_mutex_);

    // 获取锁之后重新检查，防止等待锁期间 stop() 已开始
    if (!running_.load() || !rclcpp::ok() || !node_) {
        return false;
    }

    try {
        auto it = generic_publishers_.find(topic_name);

        if (it == generic_publishers_.end()) {
            const auto qos = CreateRos2Qos(config_);

            auto publisher =
                node_->create_publisher<std_msgs::msg::String>(topic_name, qos);

            const auto result =
                generic_publishers_.emplace(topic_name, std::move(publisher));

            it = result.first;
        }

        if (!it->second) {
            return false;
        }

        std_msgs::msg::String msg;
        msg.data = json;

        it->second->publish(msg);

        return true;

    } catch (const std::exception& e) {
        std::cerr << "[Ros2Channel] publishJson exception, topic=" << topic_name
                  << ", error=" << e.what() << std::endl;

        return false;

    } catch (...) {
        std::cerr << "[Ros2Channel] publishJson unknown exception, "
                  << "topic=" << topic_name << std::endl;

        return false;
    }
}

void Ros2Channel::setCommandCallback(CommandCallback callback) {
    std::lock_guard<std::mutex> lock(callback_mutex_);

    command_callback_ = std::move(callback);
}

void Ros2Channel::onCommandMessage(const std_msgs::msg::String::SharedPtr msg,
                                   CommandSource source) {
    if (!msg) {
        return;
    }

    CommandCallback callback;

    {
        std::lock_guard<std::mutex> lock(callback_mutex_);

        callback = command_callback_;
    }

    if (!callback) {
        return;
    }

    CommunicationCommand command;

    command.source = source;
    command.name = "ros2_topic_command";
    command.payload_json = msg->data;

    const auto now = std::chrono::system_clock::now().time_since_epoch();

    command.timestamp_ms =
        std::chrono::duration_cast<std::chrono::milliseconds>(now).count();

    /*
     * 不持有 callback_mutex_ 调用外部回调。
     * 避免回调内部再次调用 setCommandCallback() 时死锁。
     */
    try {
        callback(command);

    } catch (const std::exception& e) {
        std::cerr << "[Ros2Channel] command callback exception: " << e.what()
                  << std::endl;

    } catch (...) {
        std::cerr << "[Ros2Channel] command callback "
                  << "unknown exception" << std::endl;
    }
}

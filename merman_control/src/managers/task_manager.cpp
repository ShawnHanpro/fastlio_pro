#include "managers/task_manager.h"
#include "merman_logger/merman_logger.hpp"

#include <utility>
#include <unordered_set>
#include <yaml-cpp/yaml.h>
#include <cmath>

TaskManager::TaskManager(TaskConfig config)
    : config_(std::move(config))
{
}

TaskManager::~TaskManager()
{
    Stop();
}

bool TaskManager::Start()
{
    if (task_running_.load()) {
        std::cerr
            << "[TaskManager] TaskManager is already running"
            << std::endl;

        return false;
    }

    StopRotation();

    // task_finished_.store(false);
    // has_error_.store(false);
    nav2_success_handled_.store(false);

    {
        std::lock_guard<std::mutex> lock(audio_state_mutex_);
        waiting_audio_done_ = false;
        waiting_audio_pose_id_ = -1;
    }

    try {
        if (!LoadNavigationWaypoints(
                config_.waypoints_file)) {
            // has_error_.store(true);
            return false;
        }
    } catch (const std::exception& exception) {
        std::cerr
            << "[TaskManager] 加载导航点失败: "
            << exception.what()
            << std::endl;

        // has_error_.store(true);
        return false;
    }

    {
        std::lock_guard<std::mutex> lock(waypoints_mutex_);

        if (waypoints_.empty()) {
            std::cerr
                << "[TaskManager] 导航点列表为空"
                << std::endl;

            // has_error_.store(true);
            return false;
        }
    }

    pause_task_.store(false);
    task_running_.store(true);

    if (!SetNav2Waypoints()) {
        task_running_.store(false);
        // has_error_.store(true);

        return false;
    }

    return true;
}

bool TaskManager::Stop()
{
    task_running_.store(false);
    pause_task_.store(false);

    {
        std::lock_guard<std::mutex> lock(audio_state_mutex_);
        waiting_audio_done_ = false;
        waiting_audio_pose_id_ = -1;
    }

    StopRotation();
    CancelWaypointNavigation();

    return true;
}

void TaskManager::Update()
{
    // 暂时不做任何处理，仅用于实现 ManagerBase 接口。
}

bool TaskManager::IsRunning() const
{
    return task_running_;
}

bool TaskManager::IsFinished() const
{
    return task_finished_;
}

bool TaskManager::HasError() const
{
    return has_error_;
}

std::string TaskManager::Name() const
{
    return "TaskManager";
}

bool TaskManager::AudioDone(int32_t audio_id)
{
    if (pause_task_.load()) {
        std::cout << "[TaskManager] 当前任务暂停，忽略AudioDone"
                  << ", audio_id=" << audio_id << std::endl;

        return false;
    }

    int32_t completed_pose_id = -1;

    {
        std::lock_guard<std::mutex> lock(audio_state_mutex_);

        if (!task_running_.load()) {
            std::cerr
                << "[TaskManager] 收到AudioDone，但任务未运行"
                << ", audio_id=" << audio_id
                << std::endl;

            return false;
        }

        if (!waiting_audio_done_) {
            std::cerr
                << "[TaskManager] 当前没有等待音频播放完成"
                << ", audio_id=" << audio_id
                << std::endl;

            return false;
        }

        /*
         * audio_id为-1时强制跳过当前点。
         * 其他值必须和当前等待音频的点位ID一致。
         */
        if (audio_id != -1 &&
            waiting_audio_pose_id_ != audio_id) {
            std::cerr
                << "[TaskManager] AudioDone的ID无效"
                << ", received_audio_id=" << audio_id
                << ", waiting_pose_id="
                << waiting_audio_pose_id_
                << std::endl;

            return false;
        }

        /*
         * 保存真正完成的点位ID。
         * audio_id可能为-1，不能直接用于删除当前点。
         */
        completed_pose_id = waiting_audio_pose_id_;

        /*
         * 必须先清除等待状态，再执行下一点。
         * 这样重复收到相同AudioDone时不会重复发送下一点。
         */
        waiting_audio_done_ = false;
        waiting_audio_pose_id_ = -1;
    }

    std::cout
        << "[TaskManager] AudioDone有效，可以执行下一个点"
        << ", received_audio_id=" << audio_id
        << ", completed_pose_id=" << completed_pose_id
        << std::endl;

    return AdvanceToNextWaypoint(completed_pose_id);
}

bool TaskManager::PauseTask(bool pause_task)
{
    if (!task_running_.load()) {
        std::cout
            << "[TaskManager] 当前没有正在执行的任务，忽略暂停/恢复请求"
            << std::endl;
        return false;
    }

    const bool old_pause = pause_task_.exchange(pause_task);

    // 重复命令，例如连续收到两次 pause=true
    if (old_pause == pause_task) {
        std::cout
            << "[TaskManager] pause_task 状态没有变化: "
            << std::boolalpha << pause_task
            << std::endl;
        return true;
    }

    if (pause_task) {
        std::cout
            << "[TaskManager] 暂停当前任务"
            << std::endl;

        StopRotation();

        /*
         * 2. 取消当前 Nav2 导航
         *
         * 注意：
         * Nav2 随后很可能会上报 CANCELED，
         * 后面必须修改 set_nav2_status()，
         * 不能因为暂停导致 task_running_=false。
         */
        CancelWaypointNavigation();

        // 再发一次速度为0
        set_cmd_vel(0.0, 0.0);

        return true;
    }

    std::cout << "[TaskManager] 恢复当前任务" << std::endl;

    /*
     * 如果当前已经到点，并且正在等待 audio_done，
     * 恢复时不能重新发送 Nav2。
     *
     * 正确行为：
     * - 只解除 pause_task_
     * - 保留 waiting_audio_done_
     * - 等 audio_done 回来后，由 AudioDone() 推进下一个点
     */
    {
        std::lock_guard<std::mutex> lock(audio_state_mutex_);

        if (waiting_audio_done_) {
            std::cout
                << "[TaskManager] 当前正在等待AudioDone，恢复后不重新发送Nav2"
                << ", waiting_pose_id=" << waiting_audio_pose_id_ << std::endl;

            return true;
        }
    }

    nav2_success_handled_.store(false);

    return SetNav2Waypoints();
}

Nav2Status TaskManager::ParseNav2Status(const std::string& status) {
    std::string normalized_status = status;

    // 统一转换成小写
    std::transform(
        normalized_status.begin(),
        normalized_status.end(),
        normalized_status.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });

    // 允许 partial-succeeded、partial succeeded 等写法
    std::replace(normalized_status.begin(),
                 normalized_status.end(),
                 '-',
                 '_');

    std::replace(normalized_status.begin(),
                 normalized_status.end(),
                 ' ',
                 '_');

    if (normalized_status == "idle") {
        return Nav2Status::IDLE;
    }

    if (normalized_status == "accepted") {
        return Nav2Status::ACCEPTED;
    }

    if (normalized_status == "running") {
        return Nav2Status::RUNNING;
    }

    if (normalized_status == "succeeded") {
        return Nav2Status::SUCCEEDED;
    }

    if (normalized_status == "partial_succeeded") {
        return Nav2Status::PARTIAL_SUCCEEDED;
    }

    if (normalized_status == "aborted") {
        return Nav2Status::ABORTED;
    }

    if (normalized_status == "canceled" ||
        normalized_status == "cancelled") {
        return Nav2Status::CANCELED;
    }

    if (normalized_status == "rejected") {
        return Nav2Status::REJECTED;
    }

    return Nav2Status::UNKNOWN;
}

bool TaskManager::set_nav2_status(std::string& status)
{
    const Nav2Status parsed_status = ParseNav2Status(status);

    if (parsed_status == Nav2Status::UNKNOWN) {
        std::cerr << "[TaskManager] 收到无法识别的Nav2状态: "
                  << status << std::endl;
        return false;
    }

    switch (parsed_status) {
        case Nav2Status::ACCEPTED:
            std::cout << "[TaskManager] Nav2已接受导航任务"
                      << std::endl;
            return true;

        case Nav2Status::RUNNING:
            std::cout << "[TaskManager] Nav2导航任务正在执行"
                      << std::endl;
            return true;

        case Nav2Status::SUCCEEDED: {
            std::cout << "[TaskManager] Nav2导航任务执行成功" << std::endl;

            if (!task_running_.load()) {
                std::cerr << "[TaskManager] 当前任务已停止，忽略succeeded状态"
                          << std::endl;
                return false;
            }

            // ============================================================
            // 新增
            // 暂停状态下不处理 succeeded
            // ============================================================
            if (pause_task_.load()) {
                std::cout << "[TaskManager] 当前任务已暂停，忽略Nav2 succeeded"
                          << std::endl;
                return true;
            }

            bool expected = false;

            if (!nav2_success_handled_.compare_exchange_strong(expected,
                                                               true)) {
                std::cout << "[TaskManager] "
                             "当前导航点的succeeded已经处理，忽略重复状态"
                          << std::endl;
                return true;
            }

            WayPoints waypoint;

            {
                std::lock_guard<std::mutex> lock(waypoints_mutex_);

                if (waypoints_.empty()) {
                    std::cerr << "[TaskManager] Nav2成功，但当前没有执行点"
                              << std::endl;

                    task_running_.store(false);
                    return false;
                }

                waypoint = current_waypoint_;
            }

            /*
             * 再检查一次。
             *
             * 因为上面的处理过程中，另一个ROS线程可能刚好收到pause。
             */
            if (pause_task_.load()) {
                std::cout << "[TaskManager] 到点处理前检测到任务暂停"
                          << std::endl;
                return true;
            }

            if (!StartRotationAsync(waypoint)) {
                std::cerr << "[TaskManager] 启动到点处理失败:"
                          << " point_id=" << waypoint.point_id << std::endl;

                task_running_.store(false);
                return false;
            }

            return true;
        }

        case Nav2Status::PARTIAL_SUCCEEDED:
            std::cerr << "[TaskManager] Nav2导航结束，但存在未到达点"
                      << std::endl;

            // has_error_.store(true);
            task_running_.store(false);
            return true;

        case Nav2Status::ABORTED:
            std::cerr << "[TaskManager] Nav2导航任务执行失败"
                      << std::endl;

            // has_error_.store(true);
            task_running_.store(false);
            return true;

        case Nav2Status::CANCELED:
            if (pause_task_.load()) {
                std::cout
                    << "[TaskManager] Nav2导航因任务暂停而取消"
                    << std::endl;

                return true;
            }

            std::cout
                << "[TaskManager] Nav2导航任务已取消"
                << std::endl;

            task_running_.store(false);

            return true;

        case Nav2Status::REJECTED:
            std::cerr << "[TaskManager] Nav2拒绝了导航任务"
                      << std::endl;

            // has_error_.store(true);
            task_running_.store(false);
            return true;

        case Nav2Status::IDLE:
            std::cout << "[TaskManager] Nav2当前空闲"
                      << std::endl;
            return true;

        case Nav2Status::UNKNOWN:
            return false;
    }

    return false;
}

bool TaskManager::get_task_running() const
{
    return task_running_;
}

bool TaskManager::PoseArrived(int32_t pose_id, bool play_audio) {
    if (pause_task_.load()) {
        std::cout << "[TaskManager] 当前任务暂停，禁止发送PoseArrived"
                  << ", pose_id=" << pose_id << std::endl;

        return false;
    }

    std::function<bool(int32_t, bool)> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.pose_arrived;
    }

    if (!callback) {
        std::cerr << "[TaskManager] pose_arrived回调未注册" << std::endl;
        return false;
    }

    try {
        return callback(pose_id, play_audio);
    } catch (const std::exception& exception) {
        std::cerr << "[TaskManager] PoseArrived回调异常: " << exception.what()
                  << std::endl;
        return false;
    } catch (...) {
        std::cerr << "[TaskManager] PoseArrived回调发生未知异常" << std::endl;
        return false;
    }
}

bool TaskManager::set_nav2_waypoints(std::vector<NavigationWaypoint>& navigation_waypoints)
{
    std::function<bool(std::vector<NavigationWaypoint>&)> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.set_nav2_waypoints;
    }

    if (!callback) {
        return false;
    }

    return callback(navigation_waypoints);
}

// bool TaskManager::SetNav2Waypoints()
// {
//     WayPoints waypoint;

//     {
//         std::lock_guard<std::mutex> lock(waypoints_mutex_);

//         if (waypoints_.empty()) {
//             task_running_.store(false);
//             // task_finished_.store(true);

//             std::cout << "[TaskManager] 所有导航点均已完成"
//                       << std::endl;

//             return true;
//         }

//         // 每次只发送第一个待执行点
//         current_waypoint_ = waypoints_.front();
//         waypoint = current_waypoint_;
//     }

//     std::vector<NavigationWaypoint> nav2_waypoints;
//     nav2_waypoints.emplace_back(waypoint.point);

//     // 新的导航点允许处理一次 succeeded
//     nav2_success_handled_.store(false);

//     std::cout << "[TaskManager] 发送导航点到Nav2:"
//               << " id=" << waypoint.point_id
//               << ", type=" << waypoint.type
//               << ", x=" << waypoint.point.x
//               << ", y=" << waypoint.point.y
//               << ", theta=" << waypoint.point.theta
//               << std::endl;

//     if (!set_nav2_waypoints(nav2_waypoints)) {
//         std::cerr << "[TaskManager] 发送导航点到Nav2失败:"
//                   << " id=" << waypoint.point_id
//                   << std::endl;

//         // has_error_.store(true);
//         task_running_.store(false);
//         return false;
//     }

//     return true;
// }

bool TaskManager::SetNav2Waypoints()
{
    if (pause_task_.load()) {
        std::cout << "[TaskManager] 当前任务暂停，不发送Nav2导航点"
                  << std::endl;

        return false;
    }
    
    WayPoints waypoint;

    {
        std::lock_guard<std::mutex> lock(waypoints_mutex_);

        if (waypoints_.empty()) {
            task_running_.store(false);

            std::cout
                << "[TaskManager] 所有导航点均已完成"
                << std::endl;

            return true;
        }

        // 每次只处理第一个待执行点
        current_waypoint_ = waypoints_.front();
        waypoint = current_waypoint_;
    }

    std::vector<NavigationWaypoint> nav2_waypoints;
    nav2_waypoints.emplace_back(waypoint.point);

    // 新的Nav2导航点允许处理一次succeeded
    nav2_success_handled_.store(false);

    std::cout
        << "[TaskManager] 发送导航点到Nav2:"
        << " id=" << waypoint.point_id
        << ", type=" << waypoint.type
        << ", x=" << waypoint.point.x
        << ", y=" << waypoint.point.y
        << ", theta=" << waypoint.point.theta;

    std::cout << std::endl;

    if (!set_nav2_waypoints(nav2_waypoints)) {
        std::cerr
            << "[TaskManager] 发送导航点到Nav2失败:"
            << " id=" << waypoint.point_id
            << std::endl;

        task_running_.store(false);
        return false;
    }

    return true;
}

bool TaskManager::ShouldPlayAudio(const WayPoints& waypoint)
{
    std::string normalized_type = waypoint.type;

    std::transform(
        normalized_type.begin(),
        normalized_type.end(),
        normalized_type.begin(),
        [](unsigned char character) {
            return static_cast<char>(std::tolower(character));
        });

    // task点播放语音，via点不播放
    return normalized_type == "task";
}

bool TaskManager::RemoveCompletedWaypoint(
    int32_t point_id,
    bool& has_next_waypoint)
{
    std::lock_guard<std::mutex> lock(waypoints_mutex_);

    has_next_waypoint = false;

    if (waypoints_.empty()) {
        std::cerr << "[TaskManager] 删除完成点失败，点位列表为空"
                  << std::endl;
        return false;
    }

    // 正常情况下，完成点必须是队列中的第一个点
    if (waypoints_.front().point_id != point_id) {
        std::cerr << "[TaskManager] 完成点ID与当前队首ID不匹配:"
                  << " completed_id=" << point_id
                  << ", front_id=" << waypoints_.front().point_id
                  << std::endl;
        return false;
    }

    waypoints_.erase(waypoints_.begin());

    has_next_waypoint = !waypoints_.empty();

    return true;
}

bool TaskManager::set_cmd_vel(double linear_x, double angular_z)
{
    std::function<bool(double linear_x, double angular_z)> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.set_cmd_vel;
    }

    if (!callback) {
        return false;
    }

    return callback(linear_x, angular_z);
}

bool TaskManager::get_pose(Pose2D& pose) {
    std::function<bool(Pose2D& pose)> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.get_pose;
    }

    if (!callback) {
        return false;
    }

    auto succ = callback(pose);

    return succ;
}

bool TaskManager::CancelWaypointNavigation() {
    std::function<bool()> callback;

    {
        std::lock_guard<std::mutex> lock(callbacks_mutex_);
        callback = callbacks_.cancel_nav2_waypoints;
    }

    if (!callback) {
        return false;
    }

    auto succ = callback();

    return succ;
}

void TaskManager::set_callbacks(
    TaskCallbacks callbacks)
{
    callbacks_ = std::move(callbacks);
}

bool TaskManager::LoadNavigationWaypoints(std::string points_file) {

    const YAML::Node root = YAML::LoadFile(points_file);

    const YAML::Node points_node = root["points"];

    if (!points_node) {
        throw std::runtime_error("YAML中不存在points节点");
    }

    if (!points_node.IsSequence()) {
        throw std::runtime_error("points节点必须是一个列表");
    }

    /*
        * 先解析到临时容器。
        * 只有全部解析成功后才替换输出参数，
        * 避免解析一半失败时污染原有数据。
        */
    std::vector<WayPoints> parsed_waypoints;
    parsed_waypoints.reserve(points_node.size());

    // 用来检查重复的导航点ID
    std::unordered_set<int> point_ids;

    for (std::size_t index = 0; index < points_node.size(); ++index) {
        const YAML::Node point_node = points_node[index];

        if (!point_node.IsMap()) {
            std::ostringstream stream;
            stream << "points[" << index << "]必须是一个对象";
            throw std::runtime_error(stream.str());
        }

        // 检查必要字段
        const char* required_fields[] = {
            "id",
            "type",
            "x",
            "y",
            "theta"
        };

        for (const char* field : required_fields) {
            if (!point_node[field]) {
                std::ostringstream stream;
                stream << "points[" << index
                        << "]缺少字段: " << field;
                throw std::runtime_error(stream.str());
            }
        }

        WayPoints waypoint;

        waypoint.point_id = point_node["id"].as<int>();
        waypoint.type = point_node["type"].as<std::string>();

        waypoint.point.x = point_node["x"].as<double>();
        waypoint.point.y = point_node["y"].as<double>();
        waypoint.point.theta = point_node["theta"].as<double>();

        if (waypoint.point_id < 0) {
            std::ostringstream stream;
            stream << "points[" << index
                    << "]的id不能小于0";
            throw std::runtime_error(stream.str());
        }

        if (waypoint.type.empty()) {
            std::ostringstream stream;
            stream << "points[" << index
                    << "]的type不能为空";
            throw std::runtime_error(stream.str());
        }

        const auto [iterator, inserted] =
            point_ids.insert(waypoint.point_id);

        if (!inserted) {
            std::ostringstream stream;
            stream << "存在重复的导航点id: "
                    << waypoint.point_id;
            throw std::runtime_error(stream.str());
        }

        parsed_waypoints.emplace_back(std::move(waypoint));
    }

    {
        std::lock_guard<std::mutex> lock(waypoints_mutex_);
        waypoints_ = std::move(parsed_waypoints);
    }

    return true;
}

bool TaskManager::HandleWaypoints() {
    std::vector<WayPoints> waypoints;
    {
        std::lock_guard<std::mutex> lock(waypoints_mutex_);
        waypoints = waypoints_;
    }

    if (waypoints.empty()) {
        return false;
    }

    current_waypoint_ = waypoints.front();
    return true;
}

constexpr double kPi = 3.14159265358979323846;

/**
 * 将角度归一化到 [-pi, pi]
 */
double TaskManager::NormalizeAngle(double angle) {
    while (angle > kPi) {
        angle -= 2.0 * kPi;
    }

    while (angle < -kPi) {
        angle += 2.0 * kPi;
    }

    return angle;
}

/**
 * 角度转弧度
 */
constexpr double DegreeToRadian(double degree) {
    return degree * kPi / 180.0;
}

bool TaskManager::Rotation(double target_theta)
{
    const double angle_tolerance = DegreeToRadian(
        config_.rotation_angle_tolerance_deg);

    const auto control_period = std::chrono::milliseconds(
        config_.rotation_control_period_ms);

    const auto rotation_timeout = std::chrono::seconds(
        config_.rotation_timeout_sec);

    target_theta = NormalizeAngle(target_theta);

    const auto start_time = std::chrono::steady_clock::now();

    int stable_count = 0;
    int pose_failure_count = 0;

    const auto stop_robot = [this]() {
        return set_cmd_vel(0.0, 0.0);
    };

    while (true) {

        if (pause_task_.load()) {
            std::cout
                << "[Rotation] 任务暂停，停止旋转"
                << std::endl;

            stop_robot();

            return false;
        }

        // 外部请求停止旋转
        if (rotation_stop_requested_.load()) {
            stop_robot();
            return false;
        }

        if (std::chrono::steady_clock::now() - start_time >
            rotation_timeout) {
            stop_robot();
            return false;
        }

        Pose2D current_pose{};

        if (!get_pose(current_pose)) {
            ++pose_failure_count;

            if (pose_failure_count >=
                config_.rotation_max_pose_failure_count) {
                stop_robot();
                return false;
            }

            std::this_thread::sleep_for(control_period);
            continue;
        }

        pose_failure_count = 0;

        const double current_theta =
            NormalizeAngle(current_pose.theta);

        const double angle_error =
            NormalizeAngle(target_theta - current_theta);

        if (std::abs(angle_error) <= angle_tolerance) {
            stop_robot();

            ++stable_count;

            if (stable_count >=
                config_.rotation_stable_count_required) {
                return true;
            }

            std::this_thread::sleep_for(control_period);
            continue;
        }

        stable_count = 0;

        double angular_z = std::clamp(
            config_.rotation_kp * angle_error,
            -config_.rotation_max_angular_speed_rad_s,
            config_.rotation_max_angular_speed_rad_s);

        if (std::abs(angular_z) <
            config_.rotation_min_angular_speed_rad_s) {
            angular_z = std::copysign(
                config_.rotation_min_angular_speed_rad_s,
                angular_z);
        }

        std::cout
            << "[Rotation] target_theta=" << target_theta
            << ", current_theta=" << current_theta
            << ", angle_error=" << angle_error
            << ", angular_z=" << angular_z
            << std::endl;

        if (!set_cmd_vel(0.0, angular_z)) {
            stop_robot();
            return false;
        }

        std::this_thread::sleep_for(control_period);
    }
}

bool TaskManager::StartRotationAsync(WayPoints waypoint)
{
    std::lock_guard<std::mutex> lock(rotation_thread_mutex_);

    if (rotation_running_.load()) {
        std::cerr
            << "[TaskManager] 当前已有到点处理正在执行"
            << std::endl;

        return false;
    }

    /*
     * 保留原来的线程对象和停止接口，避免修改头文件和外部调用。
     * 但这里不再执行 Rotation()，Nav2 到点后直接进入到点通知/音频流程。
     */
    if (rotation_thread_.joinable()) {
        if (rotation_thread_.get_id() !=
            std::this_thread::get_id()) {
            rotation_thread_.join();
        } else {
            rotation_thread_.detach();
        }
    }

    rotation_stop_requested_.store(false);
    rotation_running_.store(true);

    rotation_thread_ = std::thread(
        [this, waypoint]() {
            // 不再执行原地旋转，只保证底盘停止后继续后续到点流程。
            set_cmd_vel(0.0, 0.0);

            std::cout
                << "[TaskManager] Nav2到点后不执行原地旋转，直接处理到点"
                << ", pose_id=" << waypoint.point_id
                << ", target_theta=" << waypoint.point.theta
                << std::endl;

            /*
             * 必须在可能发送下一个导航点前设置为false，
             * 否则下一个点很快成功时会被判断为仍在处理。
             */
            rotation_running_.store(false);

            if (rotation_stop_requested_.load() ||
                !task_running_.load()) {
                return;
            }

            if (pause_task_.load()) {
                std::cout << "[TaskManager] 任务已暂停，不发送PoseArrived"
                          << ", pose_id=" << waypoint.point_id << std::endl;

                return;
            }

            const bool play_audio =
                ShouldPlayAudio(waypoint);

            /*
             * 需要播放音频时，必须先设置等待状态，
             * 然后再调用PoseArrived。
             *
             * 防止播放模块响应很快，在等待状态设置之前
             * 就回调AudioDone。
             */
            if (play_audio) {
                std::lock_guard<std::mutex> audio_lock(
                    audio_state_mutex_);

                waiting_audio_done_ = true;
                waiting_audio_pose_id_ =
                    waypoint.point_id;
            }

            std::cout
                << "[TaskManager] 发送PoseArrived"
                << ", pose_id=" << waypoint.point_id
                << ", play_audio="
                << std::boolalpha << play_audio
                << std::endl;

            if (!PoseArrived(waypoint.point_id, play_audio)) {
                /*
                 * 如果是因为暂停导致没有发送，
                 * 不认为任务失败。
                 */
                if (pause_task_.load()) {
                    if (play_audio) {
                        std::lock_guard<std::mutex> audio_lock(
                            audio_state_mutex_);

                        waiting_audio_done_ = false;
                        waiting_audio_pose_id_ = -1;
                    }

                    std::cout << "[TaskManager] PoseArrived因暂停未发送"
                              << std::endl;

                    return;
                }

                std::cerr << "[TaskManager] PoseArrived发送失败"
                          << ", pose_id=" << waypoint.point_id << std::endl;

                if (play_audio) {
                    std::lock_guard<std::mutex> audio_lock(audio_state_mutex_);

                    /*
                     * 只有等待的仍然是当前点时才清理。
                     * 防止同步回调AudioDone已经完成了状态推进。
                     */
                    if (waiting_audio_done_ &&
                        waiting_audio_pose_id_ == waypoint.point_id) {
                        waiting_audio_done_ = false;
                        waiting_audio_pose_id_ = -1;
                    }
                }

                // has_error_.store(true);
                task_running_.store(false);

                return;
            }

            if (play_audio) {
                std::cout
                    << "[TaskManager] 等待音频播放完成"
                    << ", pose_id=" << waypoint.point_id
                    << std::endl;

                /*
                 * 此处不删除当前点，也不发送下一个点。
                 * 等待AudioDone()推进流程。
                 */
                return;
            }

            /*
             * 不需要播放音频：
             * PoseArrived发送成功后立即执行下一点。
             */
            if (!AdvanceToNextWaypoint(
                    waypoint.point_id)) {
                std::cerr
                    << "[TaskManager] 推进下一个点位失败"
                    << ", pose_id=" << waypoint.point_id
                    << std::endl;
            }
        });

    return true;
}

bool TaskManager::StopRotation()
{
    // 通知 Rotation 循环退出
    rotation_stop_requested_.store(true);

    // 立即发送零速度，避免最多等待一个控制周期
    const bool stop_command_success =
        set_cmd_vel(0.0, 0.0);

    std::thread thread_to_join;

    {
        std::lock_guard<std::mutex> lock(rotation_thread_mutex_);

        if (rotation_thread_.joinable()) {
            // 把线程对象移出来，避免持锁 join
            thread_to_join = std::move(rotation_thread_);
        }
    }

    if (thread_to_join.joinable()) {
        // 防止旋转线程自己 join 自己
        if (thread_to_join.get_id() !=
            std::this_thread::get_id()) {
            thread_to_join.join();
        } else {
            thread_to_join.detach();
        }
    }

    rotation_running_.store(false);

    return stop_command_success;
}

bool TaskManager::AdvanceToNextWaypoint(int32_t completed_pose_id) {
    if (pause_task_.load()) {
        std::cout
            << "[TaskManager] 当前任务暂停，不推进下一个点"
            << ", current_pose_id=" << completed_pose_id
            << std::endl;

        return false;
    }

    bool has_next_waypoint = false;

    {
        std::lock_guard<std::mutex> lock(waypoints_mutex_);

        if (waypoints_.empty()) {
            std::cerr << "[TaskManager] 点位完成处理失败，点位列表为空"
                      << ", completed_pose_id=" << completed_pose_id
                      << std::endl;

            return false;
        }

        /*
         * 当前完成的点必须是待执行列表中的第一个点。
         * 这样可以避免延迟的AudioDone错误地删除下一个点。
         */
        if (waypoints_.front().point_id != completed_pose_id) {
            std::cerr << "[TaskManager] 完成点ID与当前点不一致"
                      << ", completed_pose_id=" << completed_pose_id
                      << ", current_pose_id=" << waypoints_.front().point_id
                      << std::endl;

            return false;
        }

        std::cout << "[TaskManager] 当前点位执行完成，删除点位"
                  << ", pose_id=" << completed_pose_id << std::endl;

        waypoints_.erase(waypoints_.begin());

        has_next_waypoint = !waypoints_.empty();
    }

    /*
     * 所有点位执行完成。
     */
    if (!has_next_waypoint) {
        // task_finished_.store(true);
        task_running_.store(false);

        std::cout << "[TaskManager] 所有导航点执行完成" << std::endl;

        return true;
    }

    /*
     * 外部可能在当前点完成时调用Stop()。
     * 已停止的任务不能继续发送下一个点。
     */
    if (!task_running_.load()) {
        std::cout << "[TaskManager] 任务已经停止，不再执行下一个点"
                  << std::endl;

        return false;
    }

    std::cout << "[TaskManager] 开始发送下一个导航点" << std::endl;

    if (!SetNav2Waypoints()) {
        std::cerr << "[TaskManager] 发送下一个导航点失败" << std::endl;

        // has_error_.store(true);
        task_running_.store(false);

        return false;
    }

    return true;
}
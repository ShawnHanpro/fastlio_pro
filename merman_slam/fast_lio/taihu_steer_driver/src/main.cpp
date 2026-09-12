#include <algorithm>
#include <array>
#include <atomic>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <iostream>
#include <memory>
#include <mutex>
#include <optional>
#include <string>
#include <thread>
#include <vector>

#include <ament_index_cpp/get_package_share_directory.hpp>
#include <rclcpp/rclcpp.hpp>
#include <sensor_msgs/msg/joint_state.hpp>
#include <std_srvs/srv/trigger.hpp>
#include <std_msgs/msg/float64_multi_array.hpp>

#include "taihu_steer_driver/msg/driver_state.hpp"
#include "ecat_master.hpp"

namespace
{
constexpr int kExpectedAxisCount = 4;
constexpr double kTwoPi = 6.283185307179586476925286766559;
}

class TaihuSteerDriverNode : public rclcpp::Node
{
public:
    explicit TaihuSteerDriverNode(EcatMaster *master)
        : Node("taihu_steer_driver"), master_(master)
    {
        joint_names_ = declare_parameter<std::vector<std::string>>(
            "joint_names", {"joint_1", "joint_2", "joint_3", "joint_4"});
        auto_enable_ = declare_parameter<bool>("auto_enable", false);
        encoder_resolution_ = declare_parameter<double>("encoder_resolution", 262144.0);
        max_motor_speed_rpm_ = declare_parameter<int>("max_motor_speed_rpm", 1200);
        pp_velocity_ = declare_parameter<int>("pp_velocity", 4500000);
        pp_acceleration_ = declare_parameter<int>("pp_acceleration", 30000000);
        pp_deceleration_ = declare_parameter<int>("pp_deceleration", 30000000);
        pp_jerk_ = declare_parameter<int>("pp_jerk", 1310720);
        motor_max_torque_nm_ = declare_parameter<std::vector<double>>(
            "motor_max_torque_nm", std::vector<double>(kExpectedAxisCount, 1.0));
        emergency_stop_shutdown_enabled_ = declare_parameter<bool>("emergency_stop_shutdown_enabled", true);
        emergency_stop_error_code_ = declare_parameter<int>("emergency_stop_error_code", 12576);
        emergency_stop_shutdown_consecutive_cycles_ = static_cast<int>(
            std::max<std::int64_t>(
                1, declare_parameter<std::int64_t>("emergency_stop_shutdown_consecutive_cycles", 3)));

        if (motor_max_torque_nm_.size() < kExpectedAxisCount)
        {
            motor_max_torque_nm_.resize(kExpectedAxisCount, 1.0);
        }

        joint_state_pub_ = create_publisher<sensor_msgs::msg::JointState>("steer/joint_states", 10);
        driver_state_pub_ = create_publisher<taihu_steer_driver::msg::DriverState>("steer/driver_state", 10);

        pos_cmd_sub_ = create_subscription<std_msgs::msg::Float64MultiArray>(
            "steer/position_cmd", 10,
            std::bind(&TaihuSteerDriverNode::on_position_command, this, std::placeholders::_1));

        enable_srv_ = create_service<std_srvs::srv::Trigger>(
            "steer/enable",
            std::bind(
                &TaihuSteerDriverNode::on_enable_service, this,
                std::placeholders::_1, std::placeholders::_2));
        disable_srv_ = create_service<std_srvs::srv::Trigger>(
            "steer/disable",
            std::bind(
                &TaihuSteerDriverNode::on_disable_service, this,
                std::placeholders::_1, std::placeholders::_2));
        reset_fault_srv_ = create_service<std_srvs::srv::Trigger>(
            "steer/reset_fault",
            std::bind(
                &TaihuSteerDriverNode::on_reset_fault_service, this,
                std::placeholders::_1, std::placeholders::_2));

        feedback_timer_ = create_wall_timer(
            std::chrono::milliseconds(5),
            std::bind(&TaihuSteerDriverNode::publish_feedback, this));
    }

    std::string eni_file() const
    {
        const std::string package_share_dir =
            ament_index_cpp::get_package_share_directory("taihu_steer_driver");
        return declare_or_get<std::string>(
            "eni_file", package_share_dir + "/config/NIIC_ENI_M0_steer.xml");
    }

    int cpu_affinity() const { return declare_or_get<int>("cpu_affinity", 1); }
    int priority() const { return declare_or_get<int>("priority", 90); }
    int interval() const { return declare_or_get<int>("interval", 0); }
    std::int64_t cycle_time_ns() const { return declare_or_get<std::int64_t>("cycle_time_ns", 5000000); }
    std::int64_t shift_time_ns() const { return declare_or_get<std::int64_t>("shift_time_ns", 0); }

    bool emergency_shutdown_requested() const
    {
        return emergency_shutdown_requested_.load();
    }

    std::vector<int> motor_offset() const
    {
        auto raw = declare_or_get<std::vector<int64_t>>("motor_offset", {19330, 22949, 9908, 66259});
        std::vector<int> out(raw.begin(), raw.end());
        return out;
    }

    std::vector<int> motor_dir() const
    {
        auto raw = declare_or_get<std::vector<int64_t>>("motor_dir", {1, 1, 1, 1});
        std::vector<int> out(raw.begin(), raw.end());
        return out;
    }

    void apply_pp_profile_parameters()
    {
        std::cout << "max vel acc "<<max_motor_speed_rpm_<<" ,"<<pp_velocity_<<" ,"<<pp_acceleration_<<" ,"<<pp_deceleration_<<" ,"<<pp_jerk_<<std::endl;
        std::int16_t KP = 3000;
        std::int16_t KD = 0;
        std::int16_t KV = 1000;
        std::int16_t KI = 500;
        const int axis_n = std::min(master_->axis_count, kExpectedAxisCount);
        for (int i = 0; i < axis_n; ++i)
        {
            try
            {
                const auto slave_pos = static_cast<std::uint16_t>(master_->axes[i]->slave_pos);
                write_pp_profile_param_with_retry(
                    slave_pos, ecat::sdo_idx{0x6080, 0},
                    max_motor_speed_rpm_, "max motor speed");
                write_pp_profile_param_with_retry(
                    slave_pos, ecat::sdo_idx{0x6081, 0},
                    pp_velocity_, "profile velocity");
                write_pp_profile_param_with_retry(
                    slave_pos, ecat::sdo_idx{0x6083, 0},
                    pp_acceleration_, "profile acceleration");
                write_pp_profile_param_with_retry(
                    slave_pos, ecat::sdo_idx{0x6084, 0},
                    pp_deceleration_, "profile deceleration");
                write_pp_profile_param_with_retry(
                    slave_pos, ecat::sdo_idx{0x60A4, 1},
                    pp_jerk_, "profile jerk", false);
                write_pid_profile_param_with_retry(
                    slave_pos, ecat::sdo_idx{0x60FB, 1},
                    KP, "KP");
                write_pid_profile_param_with_retry(
                    slave_pos, ecat::sdo_idx{0x60FB, 2},
                    KD, "KD");
                write_pid_profile_param_with_retry(
                    slave_pos, ecat::sdo_idx{0x60F9, 1},
                    KV, "KV");
                write_pid_profile_param_with_retry(
                    slave_pos, ecat::sdo_idx{0x60F9, 2},
                    KI, "KI");
                read_back_pp_profile_parameters(slave_pos);
            }
            catch (const std::exception &e)
            {
                RCLCPP_ERROR(
                    get_logger(),
                    "Failed to set PP profile params on slave %d: %s",
                    i, e.what());
                throw;
            }
        }

        RCLCPP_INFO(
            get_logger(),
            "Applied PP profile parameters: max_motor_speed=%u rpm, velocity=%u cnt/s, acceleration=%u cnt/s^2, deceleration=%u cnt/s^2, jerk=%u cnt/s^3",
            max_motor_speed_rpm_, pp_velocity_, pp_acceleration_, pp_deceleration_, pp_jerk_);
    }

    void write_pp_profile_param_with_retry(
        const std::uint16_t slave_pos,
        const ecat::sdo_idx index,
        const std::uint32_t value,
        const char *name,
        const bool required = true)
    {
        constexpr int kMaxAttempts = 5;
        constexpr auto kRetryDelay = std::chrono::milliseconds(200);

        for (int attempt = 1; attempt <= kMaxAttempts; ++attempt)
        {
            try
            {
                master_->task.sdo_download(slave_pos, index, false, value);
                return;
            }
            catch (const std::exception &e)
            {
                if (attempt == kMaxAttempts)
                {
                    if (!required)
                    {
                        RCLCPP_WARN(
                            get_logger(),
                            "Failed to set optional PP %s on slave %u after %d attempts: %s; ignoring",
                            name, slave_pos, kMaxAttempts, e.what());
                        return;
                    }
                    throw;
                }

                RCLCPP_WARN(
                    get_logger(),
                    "Failed to set PP %s on slave %u (attempt %d/%d): %s; retrying",
                    name, slave_pos, attempt, kMaxAttempts, e.what());
                std::this_thread::sleep_for(kRetryDelay);
            }
        }
    }

    void write_pid_profile_param_with_retry(
        const std::uint16_t slave_pos,
        const ecat::sdo_idx index,
        const std::int16_t value,
        const char *name,
        const bool required = true)
    {
        constexpr int kMaxAttempts = 5;
        constexpr auto kRetryDelay = std::chrono::milliseconds(200);

        for (int attempt = 1; attempt <= kMaxAttempts; ++attempt)
        {
            try
            {
                master_->task.sdo_download(slave_pos, index, false, value);
                return;
            }
            catch (const std::exception &e)
            {
                if (attempt == kMaxAttempts)
                {
                    if (!required)
                    {
                        RCLCPP_WARN(
                            get_logger(),
                            "Failed to set optional PP %s on slave %u after %d attempts: %s; ignoring",
                            name, slave_pos, kMaxAttempts, e.what());
                        return;
                    }
                    throw;
                }

                RCLCPP_WARN(
                    get_logger(),
                    "Failed to set PP %s on slave %u (attempt %d/%d): %s; retrying",
                    name, slave_pos, attempt, kMaxAttempts, e.what());
                std::this_thread::sleep_for(kRetryDelay);
            }
        }
    }

    std::optional<std::uint32_t> read_pp_profile_param_with_retry(
        const std::uint16_t slave_pos,
        const ecat::sdo_idx index,
        const char *name)
    {
        constexpr int kMaxAttempts = 5;
        constexpr auto kRetryDelay = std::chrono::milliseconds(200);

        for (int attempt = 1; attempt <= kMaxAttempts; ++attempt)
        {
            try
            {
                auto *master_ptr = master_->task.get_master_ptr();
                if (master_ptr == nullptr)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Failed to read PP %s on slave %u: EtherCAT master pointer is null",
                        name, slave_pos);
                    return std::nullopt;
                }
                return master_ptr->sdo_upload<std::uint32_t>(slave_pos, index, false);
            }
            catch (const std::exception &e)
            {
                if (attempt == kMaxAttempts)
                {
                    RCLCPP_WARN(
                        get_logger(),
                        "Failed to read PP %s on slave %u after %d attempts: %s; ignoring",
                        name, slave_pos, kMaxAttempts, e.what());
                    return std::nullopt;
                }

                RCLCPP_WARN(
                    get_logger(),
                    "Failed to read PP %s on slave %u (attempt %d/%d): %s; retrying",
                    name, slave_pos, attempt, kMaxAttempts, e.what());
                std::this_thread::sleep_for(kRetryDelay);
            }
        }

        return std::nullopt;
    }

    void log_pp_readback_value(
        const std::uint16_t slave_pos,
        const char *name,
        const char *unit,
        const std::optional<std::uint32_t> value)
    {
        if (!value.has_value())
        {
            return;
        }

        RCLCPP_INFO(
            get_logger(),
            "Readback PP %s on slave %u: %u %s",
            name, slave_pos, value.value(), unit);
    }

    void read_back_pp_profile_parameters(const std::uint16_t slave_pos)
    {
        log_pp_readback_value(
            slave_pos, "max motor speed (0x6080:00)", "rpm",
            read_pp_profile_param_with_retry(slave_pos, ecat::sdo_idx{0x6080, 0}, "max motor speed"));
        log_pp_readback_value(
            slave_pos, "profile velocity (0x6081:00)", "cnt/s",
            read_pp_profile_param_with_retry(slave_pos, ecat::sdo_idx{0x6081, 0}, "profile velocity"));
        log_pp_readback_value(
            slave_pos, "profile acceleration (0x6083:00)", "cnt/s^2",
            read_pp_profile_param_with_retry(slave_pos, ecat::sdo_idx{0x6083, 0}, "profile acceleration"));
        log_pp_readback_value(
            slave_pos, "profile deceleration (0x6084:00)", "cnt/s^2",
            read_pp_profile_param_with_retry(slave_pos, ecat::sdo_idx{0x6084, 0}, "profile deceleration"));
        log_pp_readback_value(
            slave_pos, "profile jerk (0x60A4:01)", "cnt/s^3",
            read_pp_profile_param_with_retry(slave_pos, ecat::sdo_idx{0x60A4, 1}, "profile jerk"));

        RCLCPP_INFO(
            get_logger(),
            "Local encoder_resolution for slave %u: %.3f cnt/rev (configured locally, not read from motor SDO)",
            slave_pos, encoder_resolution_);
    }

    void ecat_cycle()
    {
        master_->set_control_modes(1);  // PP mode
        const int axis_n = std::min(master_->axis_count, kExpectedAxisCount);
        std::array<int, kExpectedAxisCount> motor_actual_cnt{};
        std::array<int, kExpectedAxisCount> motor_target_cnt{};
        std::array<int32_t, kExpectedAxisCount> raw_velocity{};
        std::array<int16_t, kExpectedAxisCount> raw_torque{};
        std::array<uint16_t, kExpectedAxisCount> status_word{};
        std::array<uint16_t, kExpectedAxisCount> error_code{};
        std::array<int8_t, kExpectedAxisCount> mode_display{};

        const bool should_enable = auto_enable_ || enable_requested_.load();
        if (reset_fault_requested_.exchange(false))
        {
            master_->reset_faults();
            if (!should_enable)
            {
                master_->disable_slaves();
            }
        }
        else if (should_enable && !master_->check_slaves_enable())
        {
            master_->enable_slaves();
        }
        else if (!should_enable && master_->check_slaves_enable())
        {
            master_->disable_slaves();
        }

        std::array<double, kExpectedAxisCount> q_cmd_local{};
        {
            std::lock_guard<std::mutex> lk(data_mtx_);
            q_cmd_local = q_cmd_;
        }

        const bool all_enabled = master_->check_slaves_enable();
        int pp_trigger_phase = pp_trigger_phase_.load();

        for (int i = 0; i < axis_n; ++i)
        {
            motor_actual_cnt[i] = *master_->axes[i]->position_actual;
            raw_velocity[i] = *master_->axes[i]->velocity_actual;
            raw_torque[i] = *master_->axes[i]->torque_actual;
            status_word[i] = *master_->axes[i]->status_word;
            error_code[i] = *master_->axes[i]->error_code;
            mode_display[i] = *master_->axes[i]->mode_of_operation_display;

            motor_target_cnt[i] = static_cast<int>(q_cmd_local[i] / kTwoPi * encoder_resolution_) *
                                      master_->motor_dir[i] +
                                  master_->motor_offset[i];
            *master_->axes[i]->target_position = motor_target_cnt[i];

            if (should_enable && all_enabled)
            {
                if (pp_trigger_phase == 1)
                {
                    *master_->axes[i]->control_word = 0x000F;  // clear bit4 first
                }
                else if (pp_trigger_phase == 2)
                {
                    *master_->axes[i]->control_word = 0x003F;  // new set-point trigger, change immediately
                }
                else if (pp_trigger_phase == 3)
                {
                    *master_->axes[i]->control_word = 0x000F;  // clear bit4 after edge
                }
            }
        }

        if (should_enable && all_enabled && pp_trigger_phase > 0)
        {
            if (pp_trigger_phase < 3)
            {
                pp_trigger_phase_.store(pp_trigger_phase + 1);
            }
            else
            {
                pp_trigger_phase_.store(0);
            }
        }

        {
            std::lock_guard<std::mutex> lk(data_mtx_);
            for (int i = 0; i < axis_n; ++i)
            {
                q_fdb_[i] = counts_to_radians(motor_actual_cnt[i], master_->motor_offset[i], master_->motor_dir[i]);
                qd_fdb_[i] = counts_per_second_to_rad_per_sec(raw_velocity[i], master_->motor_dir[i]);
                torque_fdb_[i] = torque_raw_to_nm(raw_torque[i], motor_max_torque_nm_[i]);
                status_word_[i] = status_word[i];
                error_code_[i] = error_code[i];
                mode_display_[i] = mode_display[i];
                enable_state_[i] = master_->check_slave_enable(i);
            }
        }
        update_emergency_shutdown_state(error_code, axis_n);

        RCLCPP_INFO_THROTTLE(
            get_logger(), *get_clock(), 1000,
            "ecat_loop enable_req=%d axis_n=%d | q(rad) fdb=[%.3f, %.3f, %.3f, %.3f], cmd=[%.3f, %.3f, %.3f, %.3f] | "
            "qd(rad/s)=[%.3f, %.3f, %.3f, %.3f] | torque(Nm)=[%.3f, %.3f, %.3f, %.3f] | enabled=[%d, %d, %d, %d] | count =[%d, %d, %d, %d]",
            static_cast<int>(should_enable), axis_n,
            q_fdb_[0], q_fdb_[1], q_fdb_[2], q_fdb_[3],
            q_cmd_local[0], q_cmd_local[1], q_cmd_local[2], q_cmd_local[3],
            qd_fdb_[0], qd_fdb_[1], qd_fdb_[2], qd_fdb_[3],
            torque_fdb_[0], torque_fdb_[1], torque_fdb_[2], torque_fdb_[3],
            static_cast<int>(enable_state_[0]), static_cast<int>(enable_state_[1]),
            static_cast<int>(enable_state_[2]), static_cast<int>(enable_state_[3]),
            motor_actual_cnt[0], motor_actual_cnt[1], motor_actual_cnt[2], motor_actual_cnt[3]);
    }

private:
    template <typename T>
    T declare_or_get(const std::string &name, const T &default_value) const
    {
        if (!has_parameter(name))
        {
            const_cast<TaihuSteerDriverNode *>(this)->declare_parameter<T>(name, default_value);
        }
        return get_parameter(name).get_value<T>();
    }

    double counts_to_radians(const int32_t counts, const int offset, const int dir) const
    {
        return static_cast<double>(counts - offset) * dir / encoder_resolution_ * kTwoPi;
    }

    double counts_per_second_to_rad_per_sec(const int32_t counts_per_second, const int dir) const
    {
        return static_cast<double>(counts_per_second) * dir / encoder_resolution_ * kTwoPi / 101.0;
    }

    static double torque_raw_to_nm(const int16_t raw_torque, const double max_torque_nm)
    {
        return static_cast<double>(raw_torque) / 1000.0 * max_torque_nm;
    }

    void update_emergency_shutdown_state(
        const std::array<uint16_t, kExpectedAxisCount> &error_code,
        const int axis_n)
    {
        if (!emergency_stop_shutdown_enabled_)
        {
            return;
        }

        const auto target_error = static_cast<uint16_t>(emergency_stop_error_code_);
        int matched_axis = -1;
        for (int i = 0; i < axis_n; ++i)
        {
            if (error_code[i] == target_error)
            {
                matched_axis = i;
                break;
            }
        }

        if (matched_axis < 0)
        {
            emergency_stop_error_count_.store(0);
            return;
        }

        const int count = emergency_stop_error_count_.fetch_add(1) + 1;
        if (count >= emergency_stop_shutdown_consecutive_cycles_ &&
            !emergency_shutdown_requested_.exchange(true))
        {
            emergency_error_axis_.store(matched_axis);
            enable_requested_.store(false);
            RCLCPP_ERROR(
                get_logger(),
                "Emergency stop error detected on axis %d: error_code=%u for %d consecutive EtherCAT cycles; requesting node shutdown",
                matched_axis, static_cast<unsigned>(target_error), count);
            if (master_ != nullptr)
            {
                master_->task.break_();
            }
            rclcpp::shutdown();
        }
    }

    void on_position_command(const std_msgs::msg::Float64MultiArray::SharedPtr msg)
    {
        if (msg->data.size() != kExpectedAxisCount)
        {
            RCLCPP_WARN_THROTTLE(
                get_logger(), *get_clock(), 2000,
                "position_cmd size should be %d, got %zu", kExpectedAxisCount, msg->data.size());
            return;
        }

        std::lock_guard<std::mutex> lk(data_mtx_);
        for (int i = 0; i < kExpectedAxisCount; ++i)
        {
            q_cmd_[i] = msg->data[i];
        }
        pp_trigger_phase_.store(1);
    }

    void on_enable_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        enable_requested_.store(true);
        response->success = true;
        response->message = "enable requested";
    }

    void on_disable_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        enable_requested_.store(false);
        response->success = true;
        response->message = "disable requested";
    }

    void on_reset_fault_service(
        const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
        std::shared_ptr<std_srvs::srv::Trigger::Response> response)
    {
        reset_fault_requested_.store(true);
        response->success = true;
        response->message = "fault reset requested";
    }

    void publish_feedback()
    {
        sensor_msgs::msg::JointState js;
        js.header.stamp = now();
        js.name = joint_names_;
        js.position.resize(kExpectedAxisCount, 0.0);
        js.velocity.resize(kExpectedAxisCount, 0.0);
        js.effort.resize(kExpectedAxisCount, 0.0);

        taihu_steer_driver::msg::DriverState driver_state;
        driver_state.header.stamp = js.header.stamp;
        driver_state.joint_names = joint_names_;
        driver_state.enabled.resize(kExpectedAxisCount, false);
        driver_state.position.resize(kExpectedAxisCount, 0.0);
        driver_state.cmd_position.resize(kExpectedAxisCount, 0.0);
        driver_state.velocity.resize(kExpectedAxisCount, 0.0);
        driver_state.torque.resize(kExpectedAxisCount, 0.0);
        driver_state.status_word.resize(kExpectedAxisCount, 0);
        driver_state.error_code.resize(kExpectedAxisCount, 0);
        driver_state.mode_display.resize(kExpectedAxisCount, 0);

        {
            std::lock_guard<std::mutex> lk(data_mtx_);
            for (int i = 0; i < kExpectedAxisCount; ++i)
            {
                js.position[i] = q_fdb_[i];
                js.velocity[i] = qd_fdb_[i];
                js.effort[i] = torque_fdb_[i];

                driver_state.enabled[i] = enable_state_[i];
                driver_state.position[i] = q_fdb_[i];
                driver_state.velocity[i] = qd_fdb_[i];
                driver_state.torque[i] = torque_fdb_[i];
                driver_state.status_word[i] = status_word_[i];
                driver_state.error_code[i] = error_code_[i];
                driver_state.mode_display[i] = mode_display_[i];
                driver_state.cmd_position[i] = q_cmd_[i];
            }
        }

        joint_state_pub_->publish(js);
        driver_state_pub_->publish(driver_state);
    }

private:
    EcatMaster *master_{nullptr};

    rclcpp::Publisher<sensor_msgs::msg::JointState>::SharedPtr joint_state_pub_;
    rclcpp::Publisher<taihu_steer_driver::msg::DriverState>::SharedPtr driver_state_pub_;
    rclcpp::Subscription<std_msgs::msg::Float64MultiArray>::SharedPtr pos_cmd_sub_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
    rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr reset_fault_srv_;
    rclcpp::TimerBase::SharedPtr feedback_timer_;

    std::vector<std::string> joint_names_;
    bool auto_enable_{false};
    bool emergency_stop_shutdown_enabled_{true};
    double encoder_resolution_{262144.0};
    std::uint32_t max_motor_speed_rpm_{1200};
    std::uint32_t pp_velocity_{4500000};
    std::uint32_t pp_acceleration_{30000000};
    std::uint32_t pp_deceleration_{30000000};
    std::uint32_t pp_jerk_{1310720};
    int emergency_stop_error_code_{12576};
    int emergency_stop_shutdown_consecutive_cycles_{3};
    std::vector<double> motor_max_torque_nm_;

    std::mutex data_mtx_;
    std::array<double, kExpectedAxisCount> q_cmd_{};
    std::array<double, kExpectedAxisCount> q_fdb_{};
    std::array<double, kExpectedAxisCount> qd_fdb_{};
    std::array<double, kExpectedAxisCount> torque_fdb_{};
    std::array<uint16_t, kExpectedAxisCount> status_word_{};
    std::array<uint16_t, kExpectedAxisCount> error_code_{};
    std::array<int8_t, kExpectedAxisCount> mode_display_{};
    std::array<bool, kExpectedAxisCount> enable_state_{};
    std::atomic<bool> enable_requested_{false};
    std::atomic<bool> reset_fault_requested_{false};
    std::atomic<bool> emergency_shutdown_requested_{false};
    std::atomic<int> emergency_stop_error_count_{0};
    std::atomic<int> emergency_error_axis_{-1};
    std::atomic<int> pp_trigger_phase_{0};
};

int main(int argc, char **argv)
{
    rclcpp::init(argc, argv);

    EcatMaster master(0);
    auto node = std::make_shared<TaihuSteerDriverNode>(&master);
    qiuniu_init();
    master.init(
        node->cpu_affinity(),
        node->priority(),
        node->interval(),
        node->cycle_time_ns(),
        node->shift_time_ns(),
        node->eni_file());

    master.task.set_receive_callback([&]() {
        node->ecat_cycle();
    });
    master.start();

    auto motor_offset = node->motor_offset();
    auto motor_dir = node->motor_dir();
    if (motor_offset.size() < static_cast<size_t>(master.axis_count))
    {
        motor_offset.resize(master.axis_count, 0);
    }
    if (motor_dir.size() < static_cast<size_t>(master.axis_count))
    {
        motor_dir.resize(master.axis_count, 1);
    }
    master.set_motor_config(motor_offset.data(), motor_dir.data(), master.axis_count);
    node->apply_pp_profile_parameters();

    master.task.record(false);

    rclcpp::spin(node);
    const bool emergency_shutdown = node->emergency_shutdown_requested();
    RCLCPP_INFO(node->get_logger(), "Shutting down: disabling EtherCAT slaves");
    master.disable_slaves();
    std::this_thread::sleep_for(std::chrono::milliseconds(100));
    master.task.break_();
    master.wait();
    master.release();
    if (rclcpp::ok())
    {
        rclcpp::shutdown();
    }
    return emergency_shutdown ? 1 : 0;
}

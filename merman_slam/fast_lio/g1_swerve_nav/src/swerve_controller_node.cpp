#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <functional>
#include <memory>
#include <mutex>
#include <stdexcept>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "zlac8015d_four_wheel_driver_cpp/msg/four_wheel_state.hpp"

#include "g1_swerve_nav/swerve_kinematics.hpp"

using namespace std::chrono_literals;
using std::placeholders::_1;
using std::placeholders::_2;

namespace g1_swerve_nav
{
namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;

bool finite_twist(const geometry_msgs::msg::Twist & msg)
{
  return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
         std::isfinite(msg.angular.z);
}

double clamp_abs(double value, double limit)
{
  return std::clamp(value, -std::abs(limit), std::abs(limit));
}
}  // namespace

class SwerveControllerNode : public rclcpp::Node
{
public:
  SwerveControllerNode()
  : Node("swerve_controller")
  {
    wheelbase_ = declare_parameter<double>("wheelbase", 0.50);
    track_width_ = declare_parameter<double>("track_width", 0.40);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.085);

    steering_min_rad_ = declare_parameter<double>("steering_min_rad", -kPi / 2.0);
    steering_max_rad_ = declare_parameter<double>("steering_max_rad", kPi / 2.0);
    max_wheel_rpm_ = declare_parameter<double>("max_wheel_rpm", 100.0);
    vector_deadband_mps_ = declare_parameter<double>("wheel_vector_deadband_mps", 0.008);
    boundary_hold_enter_rad_ =
      declare_parameter<double>("boundary_hold_enter_rad", 82.0 * kPi / 180.0);
    boundary_hold_exit_rad_ =
      declare_parameter<double>("boundary_hold_exit_rad", 75.0 * kPi / 180.0);

    // Large steering reconfiguration guard. Normal small/medium corrections
    // remain continuous while driving. If any wheel needs a very large angle
    // change, first stop the drive wheels, latch one stable steering target set,
    // let the Taihu PP axes converge, then resume motion. This prevents tyres
    // from being dragged while MPPI keeps changing the requested body twist.
    large_reconfig_enter_rad_ =
      declare_parameter<double>("large_reconfig_enter_rad", 80.0 * kPi / 180.0);
    large_reconfig_exit_rad_ =
      declare_parameter<double>("large_reconfig_exit_rad", 10.0 * kPi / 180.0);
    large_reconfig_debounce_sec_ =
      declare_parameter<double>("large_reconfig_debounce_sec", 0.30);
    large_reconfig_cmd_speed_threshold_ =
      declare_parameter<double>("large_reconfig_cmd_speed_threshold", 0.10);
    large_reconfig_cooldown_sec_ =
      declare_parameter<double>("large_reconfig_cooldown_sec", 1.50);

    max_vx_ = declare_parameter<double>("max_vx", 0.50);
    max_vy_ = declare_parameter<double>("max_vy", 0.50);
    max_wz_ = declare_parameter<double>("max_wz", 0.60);
    control_frequency_ = declare_parameter<double>("control_frequency", 50.0);
    cmd_timeout_sec_ = declare_parameter<double>("cmd_timeout_sec", 0.30);
    steering_feedback_timeout_sec_ =
      declare_parameter<double>("steering_feedback_timeout_sec", 0.30);
    wheel_feedback_timeout_sec_ =
      declare_parameter<double>("wheel_feedback_timeout_sec", 0.50);
    wheel_stop_rpm_threshold_ =
      declare_parameter<double>("wheel_stop_rpm_threshold", 2.0);

    // During ordinary steering corrections, keep driving while smoothly
    // reducing wheel RPM as steering error grows. Large errors are handled by
    // the stable-target reconfiguration guard above.
    drive_full_steering_error_rad_ =
      declare_parameter<double>("drive_full_steering_error_rad", 12.0 * kPi / 180.0);
    drive_stop_steering_error_rad_ =
      declare_parameter<double>("drive_stop_steering_error_rad", 75.0 * kPi / 180.0);
    alignment_exponent_ = declare_parameter<double>("alignment_exponent", 1.0);
    minimum_alignment_scale_ = declare_parameter<double>("minimum_alignment_scale", 0.0);

    // The steering actuator angle is a directed mechanical coordinate, but the
    // tyre rolling axis is undirected because wheel RPM can reverse. Keep those
    // two errors separate. A +80 -> -80 deg steering move is ~160 deg of motor
    // travel, but only ~20 deg of rolling-axis mismatch.
    branch_transition_physical_error_rad_ =
      declare_parameter<double>("branch_transition_physical_error_rad", 95.0 * kPi / 180.0);
    branch_transition_axis_error_rad_ =
      declare_parameter<double>("branch_transition_axis_error_rad", 60.0 * kPi / 180.0);
    branch_transition_complete_error_rad_ =
      declare_parameter<double>("branch_transition_complete_error_rad", 15.0 * kPi / 180.0);
    branch_transition_chassis_scale_ =
      declare_parameter<double>("branch_transition_chassis_scale", 0.18);
    branch_transition_axis_trigger_rad_ =
      declare_parameter<double>("branch_transition_axis_trigger_rad", 45.0 * kPi / 180.0);
    branch_transition_max_rpm_ =
      declare_parameter<double>("branch_transition_max_rpm", 12.0);
    branch_transition_timeout_sec_ =
      declare_parameter<double>("branch_transition_timeout_sec", 5.0);
    fixed_target_retrigger_sec_ =
      declare_parameter<double>("fixed_target_retrigger_sec", 0.80);
    fixed_target_progress_rad_ =
      declare_parameter<double>("fixed_target_progress_rad", 1.0 * kPi / 180.0);

    wheel_rpm_accel_limit_ = declare_parameter<double>("wheel_rpm_accel_limit", 300.0);
    wheel_rpm_decel_limit_ = declare_parameter<double>("wheel_rpm_decel_limit", 400.0);
    zero_cmd_bypass_rpm_slew_ =
      declare_parameter<bool>("zero_cmd_bypass_rpm_slew", true);

    cmd_vel_topic_ = declare_parameter<std::string>("cmd_vel_topic", "/cmd_vel_safe");
    steer_cmd_topic_ = declare_parameter<std::string>("steer_cmd_topic", "/steer/position_cmd");
    steer_state_topic_ = declare_parameter<std::string>("steer_state_topic", "/steer/joint_states");
    wheel_cmd_topic_ =
      declare_parameter<std::string>("wheel_cmd_topic", "/wheel_control_can/wheel_rpm_cmd");
    wheel_state_topic_ =
      declare_parameter<std::string>("wheel_state_topic", "/wheel_control_can/state");

    steer_joint_names_fl_fr_rl_rr_ = declare_parameter<std::vector<std::string>>(
      "steer_joint_names_fl_fr_rl_rr",
      {"joint_3", "joint_4", "joint_2", "joint_1"});

    steer_cmd_indices_fl_fr_rl_rr_ = declare_parameter<std::vector<int64_t>>(
      "steer_cmd_indices_fl_fr_rl_rr", {2, 3, 1, 0});
    wheel_cmd_indices_fl_fr_rl_rr_ = declare_parameter<std::vector<int64_t>>(
      "wheel_cmd_indices_fl_fr_rl_rr", {2, 0, 3, 1});

    validate_parameters();

    const double hx = wheelbase_ * 0.5;
    const double hy = track_width_ * 0.5;
    wheel_positions_ = {{
      {hx, hy},    // FL
      {hx, -hy},   // FR
      {-hx, hy},   // RL
      {-hx, -hy}   // RR
    }};

    KinematicLimits limits;
    limits.wheel_radius_m = wheel_radius_;
    limits.steering_min_rad = steering_min_rad_;
    limits.steering_max_rad = steering_max_rad_;
    limits.max_wheel_rpm = max_wheel_rpm_;
    limits.wheel_vector_deadband_mps = vector_deadband_mps_;
    limits.boundary_hold_enter_rad = boundary_hold_enter_rad_;
    limits.boundary_hold_exit_rad = boundary_hold_exit_rad_;
    kinematics_ = std::make_unique<SwerveKinematics>(wheel_positions_, limits);

    cmd_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      cmd_vel_topic_, rclcpp::QoS(10),
      std::bind(&SwerveControllerNode::on_cmd_vel, this, _1));

    steer_state_sub_ = create_subscription<sensor_msgs::msg::JointState>(
      steer_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SwerveControllerNode::on_steer_state, this, _1));

    wheel_state_sub_ = create_subscription<
      zlac8015d_four_wheel_driver_cpp::msg::FourWheelState>(
      wheel_state_topic_, rclcpp::SensorDataQoS(),
      std::bind(&SwerveControllerNode::on_wheel_state, this, _1));

    steer_pub_ = create_publisher<std_msgs::msg::Float64MultiArray>(steer_cmd_topic_, 10);
    wheel_pub_ = create_publisher<std_msgs::msg::Float32MultiArray>(wheel_cmd_topic_, 10);
    target_steering_debug_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("~/target_steering_fl_fr_rl_rr", 10);
    target_rpm_debug_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>("~/target_rpm_fl_fr_rl_rr", 10);

    recenter_service_ = create_service<std_srvs::srv::Trigger>(
      "~/recenter_steering",
      std::bind(&SwerveControllerNode::on_recenter, this, _1, _2));

    last_control_time_ = now();
    const auto period = std::chrono::duration<double>(1.0 / control_frequency_);
    control_timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&SwerveControllerNode::control_loop, this));

    const double steering_span_deg =
      (steering_max_rad_ - steering_min_rad_) * 180.0 / kPi;
    if (steering_max_rad_ - steering_min_rad_ < kPi - 1e-3) {
      RCLCPP_WARN(
        get_logger(),
        "Configured steering span is %.1f deg (<180 deg). Some wheel velocity directions "
        "cannot be represented exactly; true holonomic motion is not guaranteed.",
        steering_span_deg);
    }

    RCLCPP_INFO(
      get_logger(),
      "New swerve controller ready: wheelbase=%.3f m track=%.3f m radius=%.3f m, "
      "steering=[%.1f, %.1f] deg, max_wheel=%.1f rpm, input=%s",
      wheelbase_, track_width_, wheel_radius_,
      steering_min_rad_ * 180.0 / kPi, steering_max_rad_ * 180.0 / kPi,
      max_wheel_rpm_, cmd_vel_topic_.c_str());
  }

private:
  void validate_parameters()
  {
    if (wheelbase_ <= 0.0 || track_width_ <= 0.0 || wheel_radius_ <= 0.0) {
      throw std::runtime_error("wheelbase, track_width and wheel_radius must be > 0");
    }
    if (control_frequency_ <= 0.0 || cmd_timeout_sec_ <= 0.0 ||
      steering_feedback_timeout_sec_ <= 0.0)
    {
      throw std::runtime_error("control and timeout parameters must be > 0");
    }
    if (steering_min_rad_ >= steering_max_rad_) {
      throw std::runtime_error("steering_min_rad must be < steering_max_rad");
    }
    if (max_wheel_rpm_ <= 0.0 || max_vx_ <= 0.0 || max_vy_ <= 0.0 || max_wz_ <= 0.0) {
      throw std::runtime_error("velocity limits must be > 0");
    }
    if (minimum_alignment_scale_ < 0.0 || minimum_alignment_scale_ > 1.0) {
      throw std::runtime_error("minimum_alignment_scale must be in [0,1]");
    }
    if (alignment_exponent_ <= 0.0) {
      throw std::runtime_error("alignment_exponent must be > 0");
    }
    if (branch_transition_physical_error_rad_ <= kPi / 2.0 ||
      branch_transition_physical_error_rad_ >= kPi ||
      branch_transition_axis_error_rad_ <= 0.0 ||
      branch_transition_axis_error_rad_ >= kPi / 2.0 ||
      branch_transition_complete_error_rad_ <= 0.0 ||
      branch_transition_complete_error_rad_ >= kPi / 2.0 ||
      branch_transition_chassis_scale_ <= 0.0 ||
      branch_transition_chassis_scale_ > 1.0 ||
      branch_transition_axis_trigger_rad_ <= 0.0 ||
      branch_transition_axis_trigger_rad_ >= kPi / 2.0 ||
      branch_transition_max_rpm_ <= 0.0 ||
      branch_transition_timeout_sec_ <= 0.0 ||
      fixed_target_retrigger_sec_ <= 0.0 ||
      fixed_target_progress_rad_ <= 0.0)
    {
      throw std::runtime_error("invalid branch-transition parameters");
    }
    if (large_reconfig_exit_rad_ <= 0.0 ||
      large_reconfig_enter_rad_ <= large_reconfig_exit_rad_ ||
      large_reconfig_enter_rad_ >= kPi / 2.0 ||
      large_reconfig_debounce_sec_ < 0.0 ||
      large_reconfig_cmd_speed_threshold_ < 0.0 ||
      large_reconfig_cooldown_sec_ < 0.0)
    {
      throw std::runtime_error("invalid large steering reconfiguration parameters");
    }
    if (drive_full_steering_error_rad_ < 0.0 ||
      drive_stop_steering_error_rad_ <= drive_full_steering_error_rad_ ||
      drive_stop_steering_error_rad_ >= kPi / 2.0)
    {
      throw std::runtime_error("invalid drive steering-error envelope");
    }
    if (boundary_hold_exit_rad_ <= 0.0 ||
      boundary_hold_enter_rad_ <= boundary_hold_exit_rad_ ||
      boundary_hold_enter_rad_ > kPi / 2.0 + 1e-6)
    {
      throw std::runtime_error("invalid boundary hold hysteresis");
    }
    if (steer_joint_names_fl_fr_rl_rr_.size() != kWheelCount ||
      steer_cmd_indices_fl_fr_rl_rr_.size() != kWheelCount ||
      wheel_cmd_indices_fl_fr_rl_rr_.size() != kWheelCount)
    {
      throw std::runtime_error("wheel name/order parameter arrays must each contain exactly 4 entries");
    }
    validate_permutation(steer_cmd_indices_fl_fr_rl_rr_, "steer_cmd_indices_fl_fr_rl_rr");
    validate_permutation(wheel_cmd_indices_fl_fr_rl_rr_, "wheel_cmd_indices_fl_fr_rl_rr");
  }

  static void validate_permutation(const std::vector<int64_t> & v, const char * name)
  {
    std::array<bool, kWheelCount> seen{};
    for (const auto index : v) {
      if (index < 0 || index >= static_cast<int64_t>(kWheelCount) || seen[index]) {
        throw std::runtime_error(std::string(name) + " must be a permutation of [0,1,2,3]");
      }
      seen[index] = true;
    }
  }

  void on_cmd_vel(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    if (!finite_twist(*msg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected NaN/Inf cmd_vel");
      return;
    }

    geometry_msgs::msg::Twist limited = *msg;
    limited.linear.x = clamp_abs(limited.linear.x, max_vx_);
    limited.linear.y = clamp_abs(limited.linear.y, max_vy_);
    limited.angular.z = clamp_abs(limited.angular.z, max_wz_);

    std::lock_guard<std::mutex> lock(mutex_);
    latest_cmd_ = limited;
    last_cmd_stamp_ = now();
    have_cmd_ = true;

    // A recenter request must not swallow a new intentional keyboard/nav
    // command. Wait until the 0-rad PP target has actually been sent before
    // allowing cancellation so velocity-smoother residuals from the preceding
    // stop cannot immediately cancel the recenter operation.
    const bool meaningful_new_cmd =
      std::hypot(limited.linear.x, limited.linear.y) > 0.02 ||
      std::abs(limited.angular.z) > 0.04;
    if (recenter_requested_ && recenter_steer_sent_ && meaningful_new_cmd) {
      recenter_requested_ = false;
      recenter_steer_sent_ = false;
      RCLCPP_INFO(
        get_logger(),
        "Steering recenter canceled by new motion command");
    }
  }

  void on_steer_state(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::array<double, kWheelCount> feedback{};
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const auto & wanted = steer_joint_names_fl_fr_rl_rr_[i];
      const auto it = std::find(msg->name.begin(), msg->name.end(), wanted);
      if (it == msg->name.end()) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Steering feedback joint '%s' not found", wanted.c_str());
        return;
      }
      const auto index = static_cast<std::size_t>(std::distance(msg->name.begin(), it));
      if (index >= msg->position.size() || !std::isfinite(msg->position[index])) {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 2000,
          "Invalid steering feedback for '%s'", wanted.c_str());
        return;
      }
      feedback[i] = msg->position[index];
    }

    std::lock_guard<std::mutex> lock(mutex_);
    steering_feedback_rad_ = feedback;
    have_steering_feedback_ = true;
    last_steering_feedback_stamp_ = now();
  }

  void on_wheel_state(
    const zlac8015d_four_wheel_driver_cpp::msg::FourWheelState::SharedPtr msg)
  {
    std::array<double, kWheelCount> rpm{{
      msg->left_front_actual_rpm,
      msg->right_front_actual_rpm,
      msg->left_rear_actual_rpm,
      msg->right_rear_actual_rpm
    }};

    std::lock_guard<std::mutex> lock(mutex_);
    wheel_feedback_rpm_ = rpm;
    have_wheel_feedback_ = true;
    last_wheel_feedback_stamp_ = now();
  }

  void on_recenter(
    const std::shared_ptr<std_srvs::srv::Trigger::Request>,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    std::lock_guard<std::mutex> lock(mutex_);
    recenter_requested_ = true;
    recenter_steer_sent_ = false;

    latest_cmd_ = geometry_msgs::msg::Twist{};
    last_cmd_stamp_ = now();
    have_cmd_ = true;
    response->success = true;
    response->message =
      "recenter accepted: drive RPM will be zeroed before steering targets move to 0 rad";
  }

  static double physical_steering_error(double target, double feedback)
  {
    return std::abs(SwerveKinematics::normalize_angle(target - feedback));
  }

  static double rolling_axis_error(double target, double feedback)
  {
    // Steering axis is equivalent modulo pi because wheel RPM may reverse.
    // Convert the directed actuator angle error [0, pi] into an undirected
    // rolling-axis error [0, pi/2].
    double error = physical_steering_error(target, feedback);
    if (error > kPi / 2.0) {
      error = kPi - error;
    }
    return std::max(0.0, error);
  }

  double compute_alignment_scale(
    const std::array<WheelState, kWheelCount> & target,
    const std::array<double, kWheelCount> & feedback,
    const std::array<bool, kWheelCount> & branch_transition) const
  {
    double scale = 1.0;
    bool any_branch_transition = false;

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      if (branch_transition[i]) {
        any_branch_transition = true;
        continue;
      }

      const double error = rolling_axis_error(target[i].steering_rad, feedback[i]);

      double wheel_scale = 1.0;
      if (error >= drive_stop_steering_error_rad_) {
        wheel_scale = 0.0;
      } else if (error > drive_full_steering_error_rad_) {
        const double u = (error - drive_full_steering_error_rad_) /
          (drive_stop_steering_error_rad_ - drive_full_steering_error_rad_);
        const double cosine_window = 0.5 * (1.0 + std::cos(kPi * u));
        wheel_scale = std::pow(std::max(0.0, cosine_window), alignment_exponent_);
      }
      scale = std::min(scale, wheel_scale);
    }

    // During a branch-equivalent long motor move, do not stop the entire
    // chassis just because one steering actuator must traverse ~180 deg.
    // Instead, slow the chassis and let each wheel's projected RPM naturally
    // pass through zero while the steering axis crosses the singular region.
    if (any_branch_transition) {
      scale = std::min(scale, branch_transition_chassis_scale_);
    }

    return std::max(scale, minimum_alignment_scale_);
  }

  double max_physical_steering_error(
    const std::array<WheelState, kWheelCount> & target,
    const std::array<double, kWheelCount> & feedback) const
  {
    double max_error = 0.0;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      max_error = std::max(
        max_error, physical_steering_error(target[i].steering_rad, feedback[i]));
    }
    return max_error;
  }

  double max_nonbranch_axis_error(
    const std::array<WheelState, kWheelCount> & target,
    const std::array<double, kWheelCount> & feedback,
    const std::array<bool, kWheelCount> & branch_transition) const
  {
    double max_error = 0.0;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      if (branch_transition[i]) {
        continue;
      }
      max_error = std::max(
        max_error, rolling_axis_error(target[i].steering_rad, feedback[i]));
    }
    return max_error;
  }

  bool has_coordinated_branch_candidate(
    const std::array<WheelState, kWheelCount> & target,
    const std::array<double, kWheelCount> & feedback) const
  {
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const double physical =
        physical_steering_error(target[i].steering_rad, feedback[i]);
      const double axis =
        rolling_axis_error(target[i].steering_rad, feedback[i]);
      if (physical >= branch_transition_physical_error_rad_ &&
        axis <= branch_transition_axis_trigger_rad_)
      {
        return true;
      }
    }
    return false;
  }

  double max_axis_error(
    const std::array<WheelState, kWheelCount> & target,
    const std::array<double, kWheelCount> & feedback) const
  {
    double max_error = 0.0;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      max_error = std::max(
        max_error, rolling_axis_error(target[i].steering_rad, feedback[i]));
    }
    return max_error;
  }

  void cap_wheel_rpm(std::array<WheelState, kWheelCount> & target, double max_abs_rpm) const
  {
    for (auto & wheel : target) {
      wheel.wheel_rpm = std::clamp(wheel.wheel_rpm, -max_abs_rpm, max_abs_rpm);
    }
  }

  void apply_actual_axis_projection(
    std::array<WheelState, kWheelCount> & target,
    const geometry_msgs::msg::Twist & cmd,
    const std::array<double, kWheelCount> & steering_feedback) const
  {
    // Command each wheel only along its ACTUAL rolling axis. The wheel-speed
    // sign therefore changes automatically when the equivalent steering branch
    // changes, instead of forcing the tyre to fight the current steering axis.
    double peak_rpm = 0.0;
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const double wheel_vx = cmd.linear.x - cmd.angular.z * wheel_positions_[i].y;
      const double wheel_vy = cmd.linear.y + cmd.angular.z * wheel_positions_[i].x;
      const double theta = steering_feedback[i];
      const double projected_mps =
        wheel_vx * std::cos(theta) + wheel_vy * std::sin(theta);
      target[i].wheel_rpm =
        SwerveKinematics::mps_to_rpm(projected_mps, wheel_radius_);
      peak_rpm = std::max(peak_rpm, std::abs(target[i].wheel_rpm));
    }

    if (peak_rpm > max_wheel_rpm_) {
      const double scale = max_wheel_rpm_ / peak_rpm;
      for (auto & wheel : target) {
        wheel.wheel_rpm *= scale;
      }
    }
  }

  double equivalent_command_speed(const geometry_msgs::msg::Twist & cmd) const
  {
    const double steering_radius = 0.5 * std::hypot(wheelbase_, track_width_);
    return std::hypot(cmd.linear.x, cmd.linear.y) +
      steering_radius * std::abs(cmd.angular.z);
  }

  bool wheels_physically_stopped(const rclcpp::Time & stamp) const
  {
    if (!have_wheel_feedback_) {
      return false;
    }
    if ((stamp - last_wheel_feedback_stamp_).seconds() > wheel_feedback_timeout_sec_) {
      return false;
    }
    for (const double rpm : wheel_feedback_rpm_) {
      if (std::abs(rpm) > wheel_stop_rpm_threshold_) {
        return false;
      }
    }
    return true;
  }

  void apply_rpm_slew(std::array<WheelState, kWheelCount> & target, double dt)
  {
    if (dt <= 0.0) {
      return;
    }
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      const double current = previous_target_rpm_[i];
      const double requested = target[i].wheel_rpm;
      const bool increasing_magnitude = std::abs(requested) > std::abs(current) &&
        (current == 0.0 || requested * current >= 0.0);
      const double limit = increasing_magnitude ? wheel_rpm_accel_limit_ : wheel_rpm_decel_limit_;
      const double max_delta = std::max(0.0, limit) * dt;
      const double delta = std::clamp(requested - current, -max_delta, max_delta);
      target[i].wheel_rpm = current + delta;
    }
  }

  void publish_target(const std::array<WheelState, kWheelCount> & target, bool send_steering = true)
  {
    std_msgs::msg::Float64MultiArray steer_msg;
    steer_msg.data.resize(kWheelCount, 0.0);
    std_msgs::msg::Float32MultiArray wheel_msg;
    wheel_msg.data.resize(kWheelCount, 0.0f);

    std_msgs::msg::Float64MultiArray steer_debug;
    std_msgs::msg::Float64MultiArray rpm_debug;
    steer_debug.data.resize(kWheelCount);
    rpm_debug.data.resize(kWheelCount);

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      steer_msg.data[static_cast<std::size_t>(steer_cmd_indices_fl_fr_rl_rr_[i])] =
        target[i].steering_rad;
      wheel_msg.data[static_cast<std::size_t>(wheel_cmd_indices_fl_fr_rl_rr_[i])] =
        static_cast<float>(target[i].wheel_rpm);
      steer_debug.data[i] = target[i].steering_rad;
      rpm_debug.data[i] = target[i].wheel_rpm;
    }

    if (send_steering) {
      steer_pub_->publish(steer_msg);
    }
    wheel_pub_->publish(wheel_msg);
    target_steering_debug_pub_->publish(steer_debug);
    target_rpm_debug_pub_->publish(rpm_debug);
  }

  void publish_zero_drive_hold_steer(
    const std::array<double, kWheelCount> & steering_hold,
    bool send_steering = false)
  {
    std::array<WheelState, kWheelCount> target{};
    for (std::size_t i = 0; i < kWheelCount; ++i) {
      target[i].steering_rad = steering_hold[i];
      target[i].wheel_rpm = 0.0;
    }
    publish_target(target, send_steering);
    previous_target_rpm_.fill(0.0);
  }

  void control_loop()
  {
    const auto stamp = now();
    double dt = (stamp - last_control_time_).seconds();
    last_control_time_ = stamp;
    if (!std::isfinite(dt) || dt <= 0.0 || dt > 0.5) {
      dt = 1.0 / control_frequency_;
    }

    geometry_msgs::msg::Twist cmd;
    std::array<double, kWheelCount> steering_feedback{};
    bool have_steering = false;
    bool steering_fresh = false;
    bool recenter = false;
    bool stopped = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      have_steering = have_steering_feedback_;
      steering_fresh = have_steering &&
        (stamp - last_steering_feedback_stamp_).seconds() <= steering_feedback_timeout_sec_;
      steering_feedback = steering_feedback_rad_;
      recenter = recenter_requested_;
      stopped = wheels_physically_stopped(stamp);

      if (have_cmd_ && (stamp - last_cmd_stamp_).seconds() <= cmd_timeout_sec_) {
        cmd = latest_cmd_;
      }
    }

    if (!steering_fresh) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "No fresh steering feedback; drive command forced to zero");
      const auto hold = targets_initialized_ ? previous_target_steering_rad_ : steering_feedback;
      publish_zero_drive_hold_steer(hold);
      return;
    }

    if (!targets_initialized_) {
      previous_target_steering_rad_ = steering_feedback;
      targets_initialized_ = true;
    }

    if (recenter) {
      steering_settle_active_ = false;
      steering_settle_candidate_active_ = false;
      steering_settle_steer_sent_ = false;
      branch_transition_active_.fill(false);
      coordinated_branch_active_ = false;
      coordinated_branch_steer_sent_ = false;
      post_rotation_settle_pending_ = false;

      // 先保证驱动轮真正停止，避免回正时拖胎
      if (!stopped) {
        publish_zero_drive_hold_steer(previous_target_steering_rad_);
        return;
      }

      std::array<WheelState, kWheelCount> centered{};

      for (std::size_t i = 0; i < kWheelCount; ++i) {
        centered[i].steering_rad = 0.0;
        centered[i].wheel_rpm = 0.0;
      }

      // Taihu PP 模式不要 50Hz 重复触发同一个位置目标。
      // 回正目标只发送一次。
      if (!recenter_steer_sent_) {
        publish_target(centered, true);

        recenter_steer_sent_ = true;
        previous_target_steering_rad_.fill(0.0);
        previous_target_rpm_.fill(0.0);

        RCLCPP_INFO(
          get_logger(),
          "Steering recenter started: target=[0 0 0 0] deg");
      } else {
        // 后续只保持驱动 RPM=0，不重复触发 Taihu PP
        publish_target(centered, false);
      }

      const double recenter_error =
        max_physical_steering_error(centered, steering_feedback);

      // 5°以内认为回正完成
      constexpr double kRecenterToleranceRad =
        5.0 * kPi / 180.0;

      if (recenter_error <= kRecenterToleranceRad) {
        {
          std::lock_guard<std::mutex> lock(mutex_);
          recenter_requested_ = false;
        }

        recenter_steer_sent_ = false;

        RCLCPP_INFO(
          get_logger(),
          "Steering recenter complete: max_error=%.1f deg",
          recenter_error * 180.0 / kPi);
      }

      return;
    }

    const bool requested_zero =
      std::abs(cmd.linear.x) < 1e-6 && std::abs(cmd.linear.y) < 1e-6 &&
      std::abs(cmd.angular.z) < 1e-6;

    const double planar_speed =
      std::hypot(cmd.linear.x, cmd.linear.y);

    // RotationShim 原地旋转：基本没有 vx/vy，只有 wz
    const bool rotate_only_cmd =
      planar_speed < 0.02 &&
      std::abs(cmd.angular.z) > 0.08;

    // 已经开始进入平移阶段
    const bool translation_cmd =
      planar_speed > 0.05;

    if (rotate_only_cmd) {
      post_rotation_settle_pending_ = true;
      last_rotate_only_sec_ = stamp.seconds();
    }

    // 防止导航结束后这个状态残留到下一次任务。
    // RotationShim -> MPPI 的正常交接通常远小于 1 秒。
    if (post_rotation_settle_pending_ &&
        last_rotate_only_sec_ > 0.0 &&
        stamp.seconds() - last_rotate_only_sec_ > 1.0)
    {
      post_rotation_settle_pending_ = false;
    }

    // A canceled/finished navigation command must never leave the steering
    // controller completing a stale large-angle reconfiguration.
    if (requested_zero) {
      steering_settle_active_ = false;
      steering_settle_candidate_active_ = false;
      steering_settle_steer_sent_ = false;
      branch_transition_active_.fill(false);
      coordinated_branch_active_ = false;
      coordinated_branch_steer_sent_ = false;
      publish_zero_drive_hold_steer(previous_target_steering_rad_);
      return;
    }

    auto ideal_target = kinematics_->inverse(
      cmd.linear.x,
      cmd.linear.y,
      cmd.angular.z,
      steering_feedback,
      previous_target_steering_rad_,
      previous_target_rpm_);

    branch_transition_active_.fill(false);
    // ============================================================
    // RotationShim -> MPPI handoff
    //
    // 车身原地旋转完成以后，不允许立即带着旋转姿态的舵轮开始平移。
    // 先让四个舵轮对准当前第一帧平移运动学目标，再恢复驱动。
    // ============================================================
    if (post_rotation_settle_pending_ &&
        translation_cmd &&
        !steering_settle_active_ &&
        !coordinated_branch_active_)
    {
      const double handoff_error =
        max_physical_steering_error(ideal_target, steering_feedback);

      post_rotation_settle_pending_ = false;

      if (handoff_error > large_reconfig_exit_rad_) {
        steering_settle_active_ = true;
        steering_settle_candidate_active_ = false;
        steering_settle_steer_sent_ = false;

        for (std::size_t i = 0; i < kWheelCount; ++i) {
          steering_settle_target_[i] = ideal_target[i].steering_rad;
        }
        steering_settle_last_error_rad_ = handoff_error;
        steering_settle_last_progress_time_ = stamp;

        RCLCPP_INFO(
          get_logger(),
          "Post-rotation steering settle latched: error=%.1f deg, "
          "target_deg[FL FR RL RR]=[%.1f %.1f %.1f %.1f]",
          handoff_error * 180.0 / kPi,
          steering_settle_target_[0] * 180.0 / kPi,
          steering_settle_target_[1] * 180.0 / kPi,
          steering_settle_target_[2] * 180.0 / kPi,
          steering_settle_target_[3] * 180.0 / kPi);
      }
    }
    const double ideal_max_axis_error = max_axis_error(ideal_target, steering_feedback);
    const bool meaningful_cmd =
      equivalent_command_speed(cmd) >= large_reconfig_cmd_speed_threshold_;

    // --------------------------------------------------------------
    // Large-angle steering reconfiguration guard
    // --------------------------------------------------------------
    // This guard now reacts only to TRUE rolling-axis mismatch. Directed
    // steering travel can be >90 deg during an equivalent branch change, but
    // that alone is not a reason to stop because wheel RPM can reverse.
    // A short debounce is kept only for genuinely near-perpendicular axes.
    const bool reconfig_cooldown_elapsed =
      !have_last_reconfig_complete_time_ ||
      (stamp - last_reconfig_complete_time_).seconds() >= large_reconfig_cooldown_sec_;

    if (!steering_settle_active_ && !coordinated_branch_active_) {
      if (reconfig_cooldown_elapsed && meaningful_cmd &&
        ideal_max_axis_error >= large_reconfig_enter_rad_)
      {
        if (!steering_settle_candidate_active_) {
          steering_settle_candidate_active_ = true;
          steering_settle_candidate_since_ = stamp;
        }

        // Stop drive immediately during debounce. Hold the existing steering
        // command until the body command has proved stable long enough to latch.
        if ((stamp - steering_settle_candidate_since_).seconds() <
          large_reconfig_debounce_sec_)
        {
          publish_zero_drive_hold_steer(previous_target_steering_rad_);
          return;
        }

        steering_settle_active_ = true;
        steering_settle_candidate_active_ = false;
        steering_settle_steer_sent_ = false;
        for (std::size_t i = 0; i < kWheelCount; ++i) {
          steering_settle_target_[i] = ideal_target[i].steering_rad;
        }
        steering_settle_last_error_rad_ =
          max_physical_steering_error(ideal_target, steering_feedback);
        steering_settle_last_progress_time_ = stamp;

        RCLCPP_WARN(
          get_logger(),
          "Large steering reconfiguration latched: max_axis_error=%.1f deg, "
          "target_deg[FL FR RL RR]=[%.1f %.1f %.1f %.1f]. "
          "Drive held at zero until steering converges.",
          ideal_max_axis_error * 180.0 / kPi,
          steering_settle_target_[0] * 180.0 / kPi,
          steering_settle_target_[1] * 180.0 / kPi,
          steering_settle_target_[2] * 180.0 / kPi,
          steering_settle_target_[3] * 180.0 / kPi);
      } else {
        steering_settle_candidate_active_ = false;
      }
    }

    if (steering_settle_active_) {
      std::array<WheelState, kWheelCount> settle_target{};
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        settle_target[i].steering_rad = steering_settle_target_[i];
        settle_target[i].wheel_rpm = 0.0;
      }

      // First make sure the drive wheels have actually stopped. Changing a
      // steering axis while the tyre is still driven is exactly the scrub/drag
      // failure seen in the second navigation segment.
      if (!stopped) {
        publish_zero_drive_hold_steer(previous_target_steering_rad_);
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Large steering reconfiguration: waiting for drive RPM to reach zero");
        return;
      }

      // Taihu uses CiA402 Profile Position mode and every received position
      // message triggers a new PP set-point. Send the latched steering target
      // once, rather than retriggering a moving PP target at the 50 Hz drive
      // loop rate.
      if (!steering_settle_steer_sent_) {
        publish_target(settle_target, true);
        steering_settle_steer_sent_ = true;
        steering_settle_last_progress_time_ = stamp;
        previous_target_steering_rad_ = steering_settle_target_;
        previous_target_rpm_.fill(0.0);
      } else {
        publish_target(settle_target, false);
      }

      const double settle_error =
        max_physical_steering_error(settle_target, steering_feedback);

      if (settle_error <
        steering_settle_last_error_rad_ - fixed_target_progress_rad_)
      {
        steering_settle_last_error_rad_ = settle_error;
        steering_settle_last_progress_time_ = stamp;
      } else if (settle_error > large_reconfig_exit_rad_ &&
        steering_settle_steer_sent_ &&
        (stamp - steering_settle_last_progress_time_).seconds() >=
          fixed_target_retrigger_sec_)
      {
        // Taihu PP occasionally stops progressing after a single trigger.
        // Re-send only the same fixed target once after a real stall; never
        // return to continuously retriggering position commands at 50 Hz.
        publish_target(settle_target, true);
        steering_settle_last_error_rad_ = settle_error;
        steering_settle_last_progress_time_ = stamp;
        RCLCPP_WARN(
          get_logger(),
          "Fixed steering settle stalled at %.1f deg; retriggering target once",
          settle_error * 180.0 / kPi);
      }

      if (settle_error <= large_reconfig_exit_rad_) {
        RCLCPP_INFO(
          get_logger(),
          "Large steering reconfiguration complete: max_error=%.1f deg; resuming drive",
          settle_error * 180.0 / kPi);
        steering_settle_active_ = false;
        steering_settle_steer_sent_ = false;
        last_reconfig_complete_time_ = stamp;
        have_last_reconfig_complete_time_ = true;
      } else {
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Large steering reconfiguration settling: max_error=%.1f deg "
          "target_deg=[%.1f %.1f %.1f %.1f] fdb_deg=[%.1f %.1f %.1f %.1f]",
          settle_error * 180.0 / kPi,
          steering_settle_target_[0] * 180.0 / kPi,
          steering_settle_target_[1] * 180.0 / kPi,
          steering_settle_target_[2] * 180.0 / kPi,
          steering_settle_target_[3] * 180.0 / kPi,
          steering_feedback[0] * 180.0 / kPi,
          steering_feedback[1] * 180.0 / kPi,
          steering_feedback[2] * 180.0 / kPi,
          steering_feedback[3] * 180.0 / kPi);
      }
      return;
    }

    // --------------------------------------------------------------
    // Coordinated equivalent-branch transition
    // --------------------------------------------------------------
    // A long directed steering move (typically +/-80 deg to the opposite
    // equivalent branch) is NOT allowed to chase a moving MPPI target. Latch
    // one complete rigid-body steering solution for all four modules, send the
    // steering set-point once, and creep using wheel RPM projected onto the
    // measured steering axes. As a module crosses the near-perpendicular part
    // of the sweep, the global rolling-axis scale naturally falls toward zero
    // and recovers on the other side. This removes prolonged loaded scrub while
    // avoiding the full stop-turn-go behavior of the large reconfiguration.
    if (!coordinated_branch_active_ && meaningful_cmd &&
      ideal_max_axis_error < large_reconfig_enter_rad_ &&
      has_coordinated_branch_candidate(ideal_target, steering_feedback))
    {
      coordinated_branch_active_ = true;
      coordinated_branch_steer_sent_ = false;
      coordinated_branch_cmd_ = cmd;
      coordinated_branch_start_ = stamp;
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        coordinated_branch_target_[i] = ideal_target[i].steering_rad;
      }
      coordinated_branch_last_error_rad_ =
        max_physical_steering_error(ideal_target, steering_feedback);
      coordinated_branch_last_progress_time_ = stamp;

      RCLCPP_INFO(
        get_logger(),
        "Coordinated branch transition latched: target_deg[FL FR RL RR]="
        "[%.1f %.1f %.1f %.1f]",
        coordinated_branch_target_[0] * 180.0 / kPi,
        coordinated_branch_target_[1] * 180.0 / kPi,
        coordinated_branch_target_[2] * 180.0 / kPi,
        coordinated_branch_target_[3] * 180.0 / kPi);
    }

    if (coordinated_branch_active_) {
      std::array<WheelState, kWheelCount> branch_target{};
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        branch_target[i].steering_rad = coordinated_branch_target_[i];
      }

      apply_actual_axis_projection(
        branch_target, coordinated_branch_cmd_, steering_feedback);

      const std::array<bool, kWheelCount> no_branch{};
      const double axis_scale =
        compute_alignment_scale(branch_target, steering_feedback, no_branch);
      const double creep_scale = std::min(axis_scale, branch_transition_chassis_scale_);
      for (auto & wheel : branch_target) {
        wheel.wheel_rpm *= creep_scale;
      }
      cap_wheel_rpm(branch_target, branch_transition_max_rpm_);

      const double elapsed = (stamp - coordinated_branch_start_).seconds();
      if (elapsed >= branch_transition_timeout_sec_) {
        for (auto & wheel : branch_target) {
          wheel.wheel_rpm = 0.0;
        }
        RCLCPP_WARN_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Coordinated branch transition exceeded %.1f s; drive held at zero while "
          "the fixed steering target continues settling",
          branch_transition_timeout_sec_);
      }

      if (!coordinated_branch_steer_sent_) {
        publish_target(branch_target, true);
        coordinated_branch_steer_sent_ = true;
        coordinated_branch_last_progress_time_ = stamp;
        previous_target_steering_rad_ = coordinated_branch_target_;
      } else {
        publish_target(branch_target, false);
      }

      for (std::size_t i = 0; i < kWheelCount; ++i) {
        previous_target_rpm_[i] = branch_target[i].wheel_rpm;
      }

      const double physical_error =
        max_physical_steering_error(branch_target, steering_feedback);
      const double axis_error = max_axis_error(branch_target, steering_feedback);

      if (physical_error <
        coordinated_branch_last_error_rad_ - fixed_target_progress_rad_)
      {
        coordinated_branch_last_error_rad_ = physical_error;
        coordinated_branch_last_progress_time_ = stamp;
      } else if (physical_error > branch_transition_complete_error_rad_ &&
        coordinated_branch_steer_sent_ &&
        (stamp - coordinated_branch_last_progress_time_).seconds() >=
          fixed_target_retrigger_sec_)
      {
        publish_target(branch_target, true);
        coordinated_branch_last_error_rad_ = physical_error;
        coordinated_branch_last_progress_time_ = stamp;
        RCLCPP_WARN(
          get_logger(),
          "Coordinated branch steering stalled at %.1f deg; "
          "retriggering fixed steering target once",
          physical_error * 180.0 / kPi);
      }

      if (physical_error <= branch_transition_complete_error_rad_) {
        RCLCPP_INFO(
          get_logger(),
          "Coordinated branch transition complete: physical=%.1f deg axis=%.1f deg",
          physical_error * 180.0 / kPi,
          axis_error * 180.0 / kPi);
        coordinated_branch_active_ = false;
        coordinated_branch_steer_sent_ = false;
      } else {
        RCLCPP_INFO_THROTTLE(
          get_logger(), *get_clock(), 1000,
          "Coordinated branch transition: physical=%.1f deg axis=%.1f deg "
          "scale=%.3f max_rpm=%.1f",
          physical_error * 180.0 / kPi,
          axis_error * 180.0 / kPi,
          creep_scale,
          branch_transition_max_rpm_);
      }
      return;
    }

    // --------------------------------------------------------------
    // Normal continuous omnidirectional tracking
    // --------------------------------------------------------------
    // For ordinary steering changes the robot keeps moving. Wheel RPM is
    // projected onto the measured steering axis, then a chassis-wide scale is
    // applied using rolling-axis error. Branch-equivalent long actuator travel
    // is deliberately allowed to continue at reduced speed.
    auto target = ideal_target;
    apply_actual_axis_projection(target, cmd, steering_feedback);
    const double alignment_scale =
      compute_alignment_scale(target, steering_feedback, branch_transition_active_);

    if (alignment_scale < 0.20) {
      std::array<double, kWheelCount> err_deg{};
      std::array<double, kWheelCount> target_deg{};
      std::array<double, kWheelCount> feedback_deg{};
      for (std::size_t i = 0; i < kWheelCount; ++i) {
        target_deg[i] = target[i].steering_rad * 180.0 / kPi;
        feedback_deg[i] = steering_feedback[i] * 180.0 / kPi;
        err_deg[i] =
          rolling_axis_error(target[i].steering_rad, steering_feedback[i]) * 180.0 / kPi;
      }
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Steering alignment limiting drive: scale=%.3f cmd=[%.3f %.3f %.3f] "
        "tgt_deg[FL FR RL RR]=[%.1f %.1f %.1f %.1f] "
        "fdb_deg=[%.1f %.1f %.1f %.1f] axis_err_deg=[%.1f %.1f %.1f %.1f] "
        "branch=[%d %d %d %d]",
        alignment_scale, cmd.linear.x, cmd.linear.y, cmd.angular.z,
        target_deg[0], target_deg[1], target_deg[2], target_deg[3],
        feedback_deg[0], feedback_deg[1], feedback_deg[2], feedback_deg[3],
        err_deg[0], err_deg[1], err_deg[2], err_deg[3],
        branch_transition_active_[0] ? 1 : 0,
        branch_transition_active_[1] ? 1 : 0,
        branch_transition_active_[2] ? 1 : 0,
        branch_transition_active_[3] ? 1 : 0);
    }

    for (auto & wheel : target) {
      wheel.wheel_rpm *= alignment_scale;
    }

    // If steering is beyond the safe rolling envelope, stop immediately rather
    // than slewing down from the previous wheel RPM for several control cycles.
    const bool hard_alignment_stop =
      ideal_max_axis_error >= drive_stop_steering_error_rad_ || alignment_scale <= 1e-6;
    if (hard_alignment_stop) {
      for (auto & wheel : target) {
        wheel.wheel_rpm = 0.0;
      }
    } else {
      apply_rpm_slew(target, dt);
    }

    publish_target(target, true);

    for (std::size_t i = 0; i < kWheelCount; ++i) {
      previous_target_steering_rad_[i] = target[i].steering_rad;
      previous_target_rpm_[i] = target[i].wheel_rpm;
    }
  }

  std::mutex mutex_;
  std::unique_ptr<SwerveKinematics> kinematics_;
  std::array<WheelPosition, kWheelCount> wheel_positions_{};

  double wheelbase_{0.50};
  double track_width_{0.40};
  double wheel_radius_{0.085};
  double steering_min_rad_{-kPi / 2.0};
  double steering_max_rad_{kPi / 2.0};
  double max_wheel_rpm_{100.0};
  double vector_deadband_mps_{0.008};
  double boundary_hold_enter_rad_{82.0 * kPi / 180.0};
  double boundary_hold_exit_rad_{75.0 * kPi / 180.0};
  double large_reconfig_enter_rad_{80.0 * kPi / 180.0};
  double large_reconfig_exit_rad_{10.0 * kPi / 180.0};
  double large_reconfig_debounce_sec_{0.30};
  double large_reconfig_cmd_speed_threshold_{0.10};
  double large_reconfig_cooldown_sec_{1.50};
  double max_vx_{0.50};
  double max_vy_{0.50};
  double max_wz_{0.60};
  double control_frequency_{50.0};
  double cmd_timeout_sec_{0.30};
  double steering_feedback_timeout_sec_{0.30};
  double wheel_feedback_timeout_sec_{0.50};
  double wheel_stop_rpm_threshold_{2.0};
  double drive_full_steering_error_rad_{12.0 * kPi / 180.0};
  double drive_stop_steering_error_rad_{75.0 * kPi / 180.0};
  double alignment_exponent_{1.0};
  double minimum_alignment_scale_{0.0};
  double branch_transition_physical_error_rad_{95.0 * kPi / 180.0};
  double branch_transition_axis_error_rad_{60.0 * kPi / 180.0};
  double branch_transition_complete_error_rad_{15.0 * kPi / 180.0};
  double branch_transition_chassis_scale_{0.18};
  double branch_transition_axis_trigger_rad_{45.0 * kPi / 180.0};
  double branch_transition_max_rpm_{12.0};
  double branch_transition_timeout_sec_{5.0};
  double fixed_target_retrigger_sec_{0.80};
  double fixed_target_progress_rad_{1.0 * kPi / 180.0};
  double wheel_rpm_accel_limit_{300.0};
  double wheel_rpm_decel_limit_{400.0};
  bool zero_cmd_bypass_rpm_slew_{true};

  std::string cmd_vel_topic_;
  std::string steer_cmd_topic_;
  std::string steer_state_topic_;
  std::string wheel_cmd_topic_;
  std::string wheel_state_topic_;
  std::vector<std::string> steer_joint_names_fl_fr_rl_rr_;
  std::vector<int64_t> steer_cmd_indices_fl_fr_rl_rr_;
  std::vector<int64_t> wheel_cmd_indices_fl_fr_rl_rr_;

  geometry_msgs::msg::Twist latest_cmd_{};
  rclcpp::Time last_cmd_stamp_{0, 0, RCL_ROS_TIME};
  bool have_cmd_{false};

  std::array<double, kWheelCount> steering_feedback_rad_{};
  rclcpp::Time last_steering_feedback_stamp_{0, 0, RCL_ROS_TIME};
  bool have_steering_feedback_{false};

  std::array<double, kWheelCount> wheel_feedback_rpm_{};
  rclcpp::Time last_wheel_feedback_stamp_{0, 0, RCL_ROS_TIME};
  bool have_wheel_feedback_{false};

  std::array<double, kWheelCount> previous_target_steering_rad_{};
  std::array<double, kWheelCount> previous_target_rpm_{};
  std::array<bool, kWheelCount> branch_transition_active_{};
  bool coordinated_branch_active_{false};
  bool   post_rotation_settle_pending_{false};
  double last_rotate_only_sec_{-1.0};
  bool coordinated_branch_steer_sent_{false};
  std::array<double, kWheelCount> coordinated_branch_target_{};
  geometry_msgs::msg::Twist coordinated_branch_cmd_{};
  rclcpp::Time coordinated_branch_start_{0, 0, RCL_ROS_TIME};
  double coordinated_branch_last_error_rad_{0.0};
  rclcpp::Time coordinated_branch_last_progress_time_{0, 0, RCL_ROS_TIME};
  bool targets_initialized_{false};
  bool recenter_requested_{false};
  bool recenter_steer_sent_{false};

  bool steering_settle_candidate_active_{false};
  rclcpp::Time steering_settle_candidate_since_{0, 0, RCL_ROS_TIME};
  bool steering_settle_active_{false};
  bool steering_settle_steer_sent_{false};
  std::array<double, kWheelCount> steering_settle_target_{};
  double steering_settle_last_error_rad_{0.0};
  rclcpp::Time steering_settle_last_progress_time_{0, 0, RCL_ROS_TIME};
  rclcpp::Time last_reconfig_complete_time_{0, 0, RCL_ROS_TIME};
  bool have_last_reconfig_complete_time_{false};

  rclcpp::Time last_control_time_{0, 0, RCL_ROS_TIME};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr steer_state_sub_;
  rclcpp::Subscription<zlac8015d_four_wheel_driver_cpp::msg::FourWheelState>::SharedPtr
    wheel_state_sub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steer_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr wheel_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_steering_debug_pub_;
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr target_rpm_debug_pub_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr recenter_service_;
  rclcpp::TimerBase::SharedPtr control_timer_;
};

}  // namespace g1_swerve_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_swerve_nav::SwerveControllerNode>());
  rclcpp::shutdown();
  return 0;
}

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"

using std::placeholders::_1;

namespace
{
constexpr double kPi = 3.14159265358979323846;
constexpr double kEpsilon = 1e-9;

double normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= 2.0 * kPi;
  }
  while (angle < -kPi) {
    angle += 2.0 * kPi;
  }
  return angle;
}

struct NormalizedWheel
{
  double steering_angle;
  double speed_sign;
};

NormalizedWheel normalize_wheel_angle(double raw_angle)
{
  double angle = normalize_angle(raw_angle);
  double sign = 1.0;

  // 转角压到 [-90°, 90°]。
  // 如果目标方向超过 90°，轮子反转，避免舵轮多转 180°。
  if (angle > kPi * 0.5) {
    angle -= kPi;
    sign = -1.0;
  } else if (angle < -kPi * 0.5) {
    angle += kPi;
    sign = -1.0;
  }

  return {angle, sign};
}

double angle_error(double target, double current)
{
  return std::abs(normalize_angle(target - current));
}
}  // namespace


class PseudoDiff4WisController : public rclcpp::Node
{
public:
  PseudoDiff4WisController()
  : Node("pseudo_diff_4wis_controller")
  {
    wheelbase_ = declare_parameter<double>("wheelbase", 0.5);
    track_width_ = declare_parameter<double>("track_width", 0.4);
    wheel_radius_ = declare_parameter<double>("wheel_radius", 0.085);

    max_wheel_speed_ =
      declare_parameter<double>("max_wheel_speed_rad_s", 10.0);
    max_wheel_accel_ =
      declare_parameter<double>("max_wheel_accel_rad_s2", 5.0);

    max_wheel_decel_ =
      declare_parameter<double>("max_wheel_decel_rad_s2", 12.0);
    max_steering_angle_ =
      declare_parameter<double>(
        "max_steering_angle_rad",
        75.0 * kPi / 180.0);

    command_timeout_sec_ =
      declare_parameter<double>("command_timeout_sec", 0.2);
    publish_period_ms_ =
      declare_parameter<int>("publish_period_ms", 50);

    // 最近多少秒收到 /cmd_vel，认为人工正在接管。
    teleop_priority_timeout_sec_ =
      declare_parameter<double>("teleop_priority_timeout_sec", 0.5);

    steering_slowdown_angle_ =
      declare_parameter<double>("steering_slowdown_angle_rad", 0.6);
    min_speed_scale_ =
      declare_parameter<double>("min_speed_scale", 0.15);

    steering_gate_engage_angle_ =
      declare_parameter<double>("steering_gate_engage_angle_rad", 0.12);
    steering_gate_release_angle_ =
      declare_parameter<double>("steering_gate_release_angle_rad", 0.07);

    // ------------------------------------------------------
    // Nav2 专用：禁止 DWB 直接驱动“边走边转”
    //
    // TELEOP 完全不受影响。
    //
    // STRAIGHT:
    //   保留 vx，强制 wz=0
    //
    // ROTATE:
    //   强制 vx=0，只执行 wz
    //
    // 使用 enter / exit 回差避免模式在阈值附近抖动。
    // ------------------------------------------------------
    nav_force_rotate_then_straight_ =
      declare_parameter<bool>("nav_force_rotate_then_straight", true);

    nav_rotate_enter_wz_ =
      declare_parameter<double>("nav_rotate_enter_wz_rad_s", 0.12);

    nav_rotate_exit_wz_ =
      declare_parameter<double>("nav_rotate_exit_wz_rad_s", 0.06);

    nav_linear_zero_threshold_ =
      declare_parameter<double>("nav_linear_zero_threshold_mps", 0.03);

    require_steering_feedback_ =
      declare_parameter<bool>("require_steering_feedback", true);

    steer_joint_names_rr_rl_fl_fr_ =
      declare_parameter<std::vector<std::string>>(
        "steer_joint_names_rr_rl_fl_fr",
        {"joint_1", "joint_2", "joint_3", "joint_4"});

    teleop_cmd_vel_topic_ =
      declare_parameter<std::string>(
        "teleop_cmd_vel_topic",
        "/cmd_vel");

    nav2_cmd_vel_topic_ =
      declare_parameter<std::string>(
        "nav2_cmd_vel_topic",
        "/cmd_vel_nav");

    steer_state_topic_ =
      declare_parameter<std::string>(
        "steer_state_topic",
        "/steer/joint_states");

    steer_cmd_topic_ =
      declare_parameter<std::string>(
        "steer_cmd_topic",
        "/steer/position_cmd");

    wheel_cmd_topic_ =
      declare_parameter<std::string>(
        "wheel_cmd_topic",
        "/wheel_control_can/wheel_rpm_cmd");

    target_steering_.fill(0.0);
    current_steering_.fill(0.0);
    last_published_wheel_speed_.fill(0.0);

    last_publish_time_ = now();
    last_teleop_cmd_time_ = now();
    last_nav2_cmd_time_ = now();

    teleop_cmd_vel_sub_ =
      create_subscription<geometry_msgs::msg::Twist>(
        teleop_cmd_vel_topic_,
        10,
        std::bind(
          &PseudoDiff4WisController::teleop_cmd_vel_callback,
          this,
          _1));

    nav2_cmd_vel_sub_ =
      create_subscription<geometry_msgs::msg::Twist>(
        nav2_cmd_vel_topic_,
        10,
        std::bind(
          &PseudoDiff4WisController::nav2_cmd_vel_callback,
          this,
          _1));

    steer_state_sub_ =
      create_subscription<sensor_msgs::msg::JointState>(
        steer_state_topic_,
        10,
        std::bind(
          &PseudoDiff4WisController::steer_state_callback,
          this,
          _1));

    steer_cmd_pub_ =
      create_publisher<std_msgs::msg::Float64MultiArray>(
        steer_cmd_topic_,
        10);

    wheel_cmd_pub_ =
      create_publisher<std_msgs::msg::Float32MultiArray>(
        wheel_cmd_topic_,
        10);

    timer_ =
      create_wall_timer(
        std::chrono::milliseconds(publish_period_ms_),
        std::bind(
          &PseudoDiff4WisController::timer_callback,
          this));

    RCLCPP_INFO(
      get_logger(),
      "Pseudo-diff 4WIS started: teleop=%s, nav2=%s",
      teleop_cmd_vel_topic_.c_str(),
      nav2_cmd_vel_topic_.c_str());

    RCLCPP_INFO(
      get_logger(),
      "Command priority: TELEOP overrides Nav2 for %.3f sec",
      teleop_priority_timeout_sec_);
  }

  ~PseudoDiff4WisController() override
  {
    publish_stop();
  }

private:
  // ----------------------------------------------------------
  // 命令输入
  // ----------------------------------------------------------

  void teleop_cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);

      latest_teleop_cmd_ = *msg;
      last_teleop_cmd_time_ = now();
      have_teleop_cmd_ = true;
    }

    RCLCPP_INFO_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "TELEOP recv: vx=%.3f vy=%.3f wz=%.3f",
      msg->linear.x,
      msg->linear.y,
      msg->angular.z);
  }

  void nav2_cmd_vel_callback(
    const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    std::lock_guard<std::mutex> lock(mutex_);

    latest_nav2_cmd_ = *msg;
    last_nav2_cmd_time_ = now();
    have_nav2_cmd_ = true;
  }


  // ----------------------------------------------------------
  // Nav2 专用运动模式过滤
  //
  // 目的不是修改 TELEOP，而是适配 G1 的 4WIS 机械特性：
  //   - 需要转向时：原地旋转
  //   - 航向基本正确后：直线行驶
  //
  // steering_motion_gate_active_ 为 true 时不允许切换模式，
  // 避免舵轮还没到当前目标角，DWB 又给出另一组 vx/wz，
  // 导致目标舵角在原地不断改变。
  // ----------------------------------------------------------
  void filter_nav2_command(
    geometry_msgs::msg::Twist & cmd)
  {
    if (!nav_force_rotate_then_straight_) {
      cmd.linear.y = 0.0;
      return;
    }

    const double vx = cmd.linear.x;
    const double wz = cmd.angular.z;

    const bool pure_rotation_request =
      std::abs(vx) <= nav_linear_zero_threshold_ &&
      std::abs(wz) > kEpsilon;

    // 舵轮正在追当前目标时，锁住当前运动模式。
    if (!steering_motion_gate_active_) {
      bool new_rotate_mode = nav_rotate_mode_;

      if (pure_rotation_request) {
        // RotationShim 的纯旋转命令必须完整保留，
        // 即使接近目标时 wz 已经很小。
        new_rotate_mode = true;
      }
      else if (nav_rotate_mode_) {
        if (std::abs(wz) <= nav_rotate_exit_wz_) {
          new_rotate_mode = false;
        }
      }
      else {
        if (std::abs(wz) >= nav_rotate_enter_wz_) {
          new_rotate_mode = true;
        }
      }

      if (new_rotate_mode != nav_rotate_mode_) {
        nav_rotate_mode_ = new_rotate_mode;

        RCLCPP_INFO(
          get_logger(),
          "NAV motion mode -> %s (raw vx=%.3f wz=%.3f)",
          nav_rotate_mode_ ? "ROTATE" : "STRAIGHT",
          vx,
          wz);
      }
    }

    cmd.linear.y = 0.0;

    if (nav_rotate_mode_) {
      // 原地旋转：禁止同时前进。
      cmd.linear.x = 0.0;
    }
    else {
      // 直行：禁止 DWB 用小角速度不断改变舵角。
      cmd.angular.z = 0.0;
    }

    RCLCPP_DEBUG_THROTTLE(
      get_logger(),
      *get_clock(),
      1000,
      "NAV filtered: raw(vx=%.3f wz=%.3f) -> %s(vx=%.3f wz=%.3f)",
      vx,
      wz,
      nav_rotate_mode_ ? "ROTATE" : "STRAIGHT",
      cmd.linear.x,
      cmd.angular.z);
  }


  // ----------------------------------------------------------
  // vx + wz -> 四轮四转
  //
  // 内部顺序：
  //   [FL, FR, RL, RR]
  // ----------------------------------------------------------

  void calculate_4wis(
    const geometry_msgs::msg::Twist & cmd,
    std::array<double, 4> & steering,
    std::array<double, 4> & wheel_speed)
  {
    steering.fill(0.0);
    wheel_speed.fill(0.0);

    const double vx = cmd.linear.x;
    const double wz = cmd.angular.z;

    if (!std::isfinite(vx) ||
        !std::isfinite(cmd.linear.y) ||
        !std::isfinite(wz))
    {
      return;
    }

    // 伪差速：永远忽略横移 vy。
    if (std::abs(cmd.linear.y) > 1e-4) {
      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        2000,
        "linear.y=%.3f ignored in pseudo-diff mode",
        cmd.linear.y);
    }

    const double half_l = wheelbase_ * 0.5;
    const double half_w = track_width_ * 0.5;

    const std::array<std::array<double, 2>, 4> wheel_positions = {{
      {{ half_l,  half_w }},  // FL
      {{ half_l, -half_w }},  // FR
      {{-half_l,  half_w }},  // RL
      {{-half_l, -half_w }},  // RR
    }};

    double max_abs_speed = 0.0;

    for (size_t i = 0; i < wheel_positions.size(); ++i) {
      const double x = wheel_positions[i][0];
      const double y = wheel_positions[i][1];

      // body vy 固定为 0。
      const double wheel_vx = vx - wz * y;
      const double wheel_vy = wz * x;

      const double speed_mps =
        std::hypot(wheel_vx, wheel_vy);

      if (speed_mps < kEpsilon) {
        steering[i] = 0.0;
        wheel_speed[i] = 0.0;
        continue;
      }

      const auto normalized =
        normalize_wheel_angle(
          std::atan2(wheel_vy, wheel_vx));

      steering[i] =
        std::clamp(
          normalized.steering_angle,
          -max_steering_angle_,
          max_steering_angle_);

      wheel_speed[i] =
        speed_mps /
        wheel_radius_ *
        normalized.speed_sign;

      max_abs_speed =
        std::max(
          max_abs_speed,
          std::abs(wheel_speed[i]));
    }

    if (max_wheel_speed_ > 0.0 &&
        max_abs_speed > max_wheel_speed_)
    {
      const double scale =
        max_wheel_speed_ / max_abs_speed;

      for (double & speed : wheel_speed) {
        speed *= scale;
      }
    }
  }


  // ----------------------------------------------------------
  // Taihu steering feedback
  // ----------------------------------------------------------

  void steer_state_callback(
    const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    if (steer_joint_names_rr_rl_fl_fr_.size() != 4) {
      return;
    }

    std::array<int, 4> index_rr_rl_fl_fr =
      {-1, -1, -1, -1};

    for (size_t i = 0; i < msg->name.size(); ++i) {
      for (size_t j = 0; j < 4; ++j) {
        if (msg->name[i] ==
            steer_joint_names_rr_rl_fl_fr_[j])
        {
          index_rr_rl_fl_fr[j] =
            static_cast<int>(i);
        }
      }
    }

    for (const int index : index_rr_rl_fl_fr) {
      if (index < 0 ||
          static_cast<size_t>(index) >=
          msg->position.size())
      {
        return;
      }
    }

    std::array<double, 4> current{};

    // 参数顺序：
    //   [RR, RL, FL, FR]
    //
    // 转成内部：
    //   [FL, FR, RL, RR]
    current[0] =
      msg->position[
        static_cast<size_t>(
          index_rr_rl_fl_fr[2])];

    current[1] =
      msg->position[
        static_cast<size_t>(
          index_rr_rl_fl_fr[3])];

    current[2] =
      msg->position[
        static_cast<size_t>(
          index_rr_rl_fl_fr[1])];

    current[3] =
      msg->position[
        static_cast<size_t>(
          index_rr_rl_fl_fr[0])];

    {
      std::lock_guard<std::mutex> lock(mutex_);
      current_steering_ = current;
      have_steering_feedback_ = true;
    }
  }


  // ----------------------------------------------------------
  // 20 Hz 输出
  // ----------------------------------------------------------

  void timer_callback()
  {
    const rclcpp::Time current_time = now();

    double dt =
      (current_time - last_publish_time_).seconds();

    if (dt <= 0.0 || dt > 1.0) {
      dt =
        static_cast<double>(
          publish_period_ms_) /
        1000.0;
    }

    last_publish_time_ = current_time;

    geometry_msgs::msg::Twist selected_cmd;

    bool have_selected_cmd = false;
    bool selected_teleop = false;

    std::array<double, 4> steering_feedback{};
    bool have_feedback = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      steering_feedback = current_steering_;
      have_feedback = have_steering_feedback_;

      const double teleop_age =
        have_teleop_cmd_
          ? (current_time -
             last_teleop_cmd_time_).seconds()
          : 1e9;

      const double nav2_age =
        have_nav2_cmd_
          ? (current_time -
             last_nav2_cmd_time_).seconds()
          : 1e9;

      // ------------------------------------------------------
      // 优先级保持不变：
      //
      // 1. TELEOP
      // 2. Nav2
      // 3. STOP
      // ------------------------------------------------------
      if (have_teleop_cmd_ &&
          teleop_age <=
            teleop_priority_timeout_sec_)
      {
        selected_cmd = latest_teleop_cmd_;
        have_selected_cmd = true;
        selected_teleop = true;
      }
      else if (
        have_nav2_cmd_ &&
        nav2_age <= command_timeout_sec_)
      {
        selected_cmd = latest_nav2_cmd_;
        have_selected_cmd = true;
      }
    }

    std::array<double, 4> steering{};
    std::array<double, 4> wheel_speed{};

    // ------------------------------------------------------
    // Nav2：先离散为 ROTATE / STRAIGHT。
    //
    // TELEOP：完全保持原来的 vx+wz -> 4WIS 行为。
    // ------------------------------------------------------
    if (have_selected_cmd && !selected_teleop) {
      filter_nav2_command(selected_cmd);
    }

    if (have_selected_cmd) {
      calculate_4wis(
        selected_cmd,
        steering,
        wheel_speed);

      if (!selected_teleop &&
          steering_motion_gate_active_)
      {
        // Nav2 安全门期间必须锁住上一组目标舵角。
        //
        // DWB 即使在这期间不断改变 vx/wz，也不能重新改变
        // 舵轮目标。先把当前动作做完，再允许切换。
        std::lock_guard<std::mutex> lock(mutex_);
        steering = target_steering_;
        wheel_speed.fill(0.0);
      }
      else {
        std::lock_guard<std::mutex> lock(mutex_);
        target_steering_ = steering;
      }

      if (selected_teleop) {
        RCLCPP_DEBUG_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "cmd source: TELEOP");
      }
    }
    else {
      wheel_speed.fill(0.0);

      // 无命令时保持上一目标转向角。
      std::lock_guard<std::mutex> lock(mutex_);
      steering = target_steering_;
    }

    bool force_immediate_stop = false;

    // Taihu feedback 没有打通时禁止驱动轮运动。
    if (require_steering_feedback_ &&
        !have_feedback)
    {
      wheel_speed.fill(0.0);
      force_immediate_stop = true;

      RCLCPP_WARN_THROTTLE(
        get_logger(),
        *get_clock(),
        3000,
        "Waiting for steering feedback: %s; wheel RPM forced to zero",
        steer_state_topic_.c_str());
    }

    // ------------------------------------------------------
    // 硬安全门
    //
    // 对 Nav2：
    //   gate active 时 steering 已经被锁定，不再追逐 DWB
    //   每周期变化的目标。
    //
    // 对 TELEOP：
    //   行为保持原来逻辑，用户的新指令可实时改变目标角。
    // ------------------------------------------------------
    double max_steering_error = 0.0;

    if (have_feedback) {
      for (size_t i = 0; i < 4; ++i) {
        max_steering_error =
          std::max(
            max_steering_error,
            angle_error(
              steering[i],
              steering_feedback[i]));
      }

      if (!steering_motion_gate_active_ &&
          max_steering_error > steering_gate_engage_angle_)
      {
        steering_motion_gate_active_ = true;

        // 此时 target_steering_ 已经保存了本次确定的目标角。
        if (!selected_teleop) {
          RCLCPP_INFO(
            get_logger(),
            "NAV steering target locked; max_error=%.3f rad",
            max_steering_error);
        }
      }

      if (steering_motion_gate_active_ &&
          max_steering_error < steering_gate_release_angle_)
      {
        steering_motion_gate_active_ = false;

        if (!selected_teleop) {
          RCLCPP_INFO(
            get_logger(),
            "NAV steering target reached; max_error=%.3f rad",
            max_steering_error);
        }
      }

      if (steering_motion_gate_active_) {
        wheel_speed.fill(0.0);
        force_immediate_stop = true;

        RCLCPP_WARN_THROTTLE(
          get_logger(),
          *get_clock(),
          1000,
          "STEERING GATE: max_error=%.3f rad, wheel RPM forced to zero",
          max_steering_error);
      }
    }

    // 进入安全角度后继续保留原有柔性降速。
    if (have_feedback &&
        !steering_motion_gate_active_ &&
        steering_slowdown_angle_ > kEpsilon)
    {
      const double speed_scale =
        std::clamp(
          1.0 -
            max_steering_error /
            steering_slowdown_angle_,
          min_speed_scale_,
          1.0);

      for (double & speed : wheel_speed) {
        speed *= speed_scale;
      }
    }

    // 安全门触发时立即输出 0，不经过减速斜坡。
    if (force_immediate_stop) {
      wheel_speed.fill(0.0);
      last_published_wheel_speed_.fill(0.0);
    }
    else {
      limit_wheel_acceleration(
        wheel_speed,
        dt);
    }

    publish_commands(
      steering,
      wheel_speed);
  }


  void limit_wheel_acceleration(
    std::array<double, 4> & target,
    double dt)
  {
    if (dt <= 0.0) {
      last_published_wheel_speed_ = target;
      return;
    }

    for (size_t i = 0; i < 4; ++i) {
      const double previous = last_published_wheel_speed_[i];
      double desired = target[i];

      // 正反方向切换时先停到 0，再反向。
      if (previous * desired < 0.0 &&
          std::abs(previous) > kEpsilon)
      {
        desired = 0.0;
      }

      const bool accelerating =
        std::abs(desired) > std::abs(previous);

      const double limit =
        accelerating ? max_wheel_accel_ : max_wheel_decel_;

      if (limit <= 0.0) {
        target[i] = desired;
        continue;
      }

      const double max_delta = limit * dt;

      target[i] =
        previous +
        std::clamp(
          desired - previous,
          -max_delta,
          max_delta);
    }

    last_published_wheel_speed_ = target;
  }


  static double rad_s_to_rpm(double value)
  {
    return
      value *
      60.0 /
      (2.0 * kPi);
  }


  void publish_commands(
    const std::array<double, 4> & steering,
    const std::array<double, 4> & wheel_speed)
  {
    std_msgs::msg::Float64MultiArray steer_msg;

    // internal:
    //   [FL, FR, RL, RR]
    //
    // Taihu:
    //   [RR, RL, FL, FR]
    steer_msg.data = {
      steering[3],
      steering[2],
      steering[0],
      steering[1],
    };

    steer_cmd_pub_->publish(
      steer_msg);

    std_msgs::msg::Float32MultiArray wheel_msg;

    // internal:
    //   [FL, FR, RL, RR]
    //
    // ZLAC:
    //   [RF, RR, LF, LR]
    wheel_msg.data = {
      static_cast<float>(
        rad_s_to_rpm(
          wheel_speed[1])),

      static_cast<float>(
        rad_s_to_rpm(
          wheel_speed[3])),

      static_cast<float>(
        rad_s_to_rpm(
          wheel_speed[0])),

      static_cast<float>(
        rad_s_to_rpm(
          wheel_speed[2])),
    };

    wheel_cmd_pub_->publish(
      wheel_msg);
  }


  void publish_stop()
  {
    std::array<double, 4> steering{};

    {
      std::lock_guard<std::mutex> lock(mutex_);
      steering = target_steering_;
    }

    std::array<double, 4> zero_speed{};
    zero_speed.fill(0.0);

    publish_commands(
      steering,
      zero_speed);
  }


private:
  double wheelbase_;
  double track_width_;
  double wheel_radius_;

  double max_wheel_speed_;
  double max_wheel_accel_;
  double max_wheel_decel_;
  double max_steering_angle_;

  double command_timeout_sec_;
  int publish_period_ms_;
  double teleop_priority_timeout_sec_;

  double steering_slowdown_angle_;
  double min_speed_scale_;

  double steering_gate_engage_angle_;
  double steering_gate_release_angle_;

  bool nav_force_rotate_then_straight_;
  double nav_rotate_enter_wz_;
  double nav_rotate_exit_wz_;
  double nav_linear_zero_threshold_;
  bool nav_rotate_mode_{false};

  bool require_steering_feedback_;
  bool steering_motion_gate_active_{false};

  std::vector<std::string>
    steer_joint_names_rr_rl_fl_fr_;

  std::string teleop_cmd_vel_topic_;
  std::string nav2_cmd_vel_topic_;

  std::string steer_state_topic_;
  std::string steer_cmd_topic_;
  std::string wheel_cmd_topic_;

  geometry_msgs::msg::Twist
    latest_teleop_cmd_;

  geometry_msgs::msg::Twist
    latest_nav2_cmd_;

  bool have_teleop_cmd_{false};
  bool have_nav2_cmd_{false};

  rclcpp::Time last_teleop_cmd_time_;
  rclcpp::Time last_nav2_cmd_time_;
  rclcpp::Time last_publish_time_;

  std::array<double, 4>
    target_steering_;

  std::array<double, 4>
    current_steering_;

  std::array<double, 4>
    last_published_wheel_speed_;

  bool have_steering_feedback_{false};

  std::mutex mutex_;

  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr
    teleop_cmd_vel_sub_;

  rclcpp::Subscription<
    geometry_msgs::msg::Twist>::SharedPtr
    nav2_cmd_vel_sub_;

  rclcpp::Subscription<
    sensor_msgs::msg::JointState>::SharedPtr
    steer_state_sub_;

  rclcpp::Publisher<
    std_msgs::msg::Float64MultiArray>::SharedPtr
    steer_cmd_pub_;

  rclcpp::Publisher<
    std_msgs::msg::Float32MultiArray>::SharedPtr
    wheel_cmd_pub_;

  rclcpp::TimerBase::SharedPtr timer_;
};


int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);

  auto node =
    std::make_shared<
      PseudoDiff4WisController>();

  rclcpp::spin(node);

  rclcpp::shutdown();
  return 0;
}

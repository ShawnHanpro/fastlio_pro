#include <array>
#include <algorithm>
#include <cmath>
#include <memory>
#include <mutex>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "geometry_msgs/msg/twist.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_msgs/msg/float64_multi_array.hpp"
#include "sensor_msgs/msg/joint_state.hpp"
#include "nav_msgs/msg/odometry.hpp"
#include "tf2/LinearMath/Quaternion.h"
#include "tf2_ros/transform_broadcaster.h"
#include "geometry_msgs/msg/transform_stamped.hpp"

#include "steering_controllers_library/steering_odometry.hpp"
#include "zlac8015d_four_wheel_driver_cpp/msg/four_wheel_state.hpp"

using std::placeholders::_1;

class AckermannKinematicsNode : public rclcpp::Node
{
public:

  AckermannKinematicsNode()
  : Node("ackermann_kinematics_node"),
    current_steering_angles_{0.0, 0.0, 0.0, 0.0},
    cached_steering_angles_{0.0, 0.0, 0.0, 0.0},
    cached_wheel_velocities_{0.0, 0.0, 0.0, 0.0}
  {
    this->declare_parameter("wheel.max_accel", 5.0);
    this->declare_parameter("wheel.max_speed", 10.0);
    this->declare_parameter("steering.max_angle", 75.0 * M_PI / 180.0);
    this->declare_parameter("wheelbase", 0.5);
    this->declare_parameter("track_width", 0.4);
    this->declare_parameter("wheel_radius", 0.085);
    this->declare_parameter("base_frame_id", "base_link");
    this->declare_parameter("odom_frame_id", "odom");
    this->declare_parameter("enable_odom_tf", false);
    this->declare_parameter(
      "steer_joint_names_rr_rl_fl_fr",
      std::vector<std::string>{"joint_1", "joint_2", "joint_3", "joint_4"});

    max_wheel_accel_ = this->get_parameter("wheel.max_accel").as_double();
    max_wheel_speed_ = this->get_parameter("wheel.max_speed").as_double();
    max_steering_angle_ = this->get_parameter("steering.max_angle").as_double();
    wheelbase_ = this->get_parameter("wheelbase").as_double();
    track_width_ = this->get_parameter("track_width").as_double();
    wheel_radius_ = this->get_parameter("wheel_radius").as_double();
    base_frame_id_ = this->get_parameter("base_frame_id").as_string();
    odom_frame_id_ = this->get_parameter("odom_frame_id").as_string();
    enable_odom_tf_ = this->get_parameter("enable_odom_tf").as_bool();
    steer_joint_names_rr_rl_fl_fr_ =
      this->get_parameter("steer_joint_names_rr_rl_fl_fr").as_string_array();

    odometry_.set_wheel_params(wheel_radius_, wheelbase_, track_width_);
    odometry_.set_command_limits(max_wheel_speed_, max_steering_angle_);
    odometry_.set_odometry_type(steering_odometry::FOUR_WHEEL_STEERING_CONFIG);

    last_odom_update_time_ = this->now();
    last_publish_time_ = this->now();
    last_wheel_vel_ = {0.0, 0.0, 0.0, 0.0};

    cmd_vel_sub_ = this->create_subscription<geometry_msgs::msg::Twist>(
      "/cmd_vel", 10, std::bind(&AckermannKinematicsNode::cmd_vel_callback, this, _1));

    steer_joint_state_sub_ = this->create_subscription<sensor_msgs::msg::JointState>(
      "/steer/joint_states", 10,
      std::bind(&AckermannKinematicsNode::steer_joint_state_callback, this, _1));
    wheel_state_sub_ = this->create_subscription<zlac8015d_four_wheel_driver_cpp::msg::FourWheelState>(
      "/wheel_control_can/state", 10,
      std::bind(&AckermannKinematicsNode::wheel_state_callback, this, _1));

    last_cmd_time_ = this->now();

    command_timer_ = this->create_wall_timer(
      std::chrono::milliseconds(50),
      std::bind(&AckermannKinematicsNode::command_timer_callback, this));

    steering_pub_ = this->create_publisher<std_msgs::msg::Float64MultiArray>(
      "/steer/position_cmd", 10);
    velocity_pub_ = this->create_publisher<std_msgs::msg::Float32MultiArray>(
      "/wheel_control_can/wheel_rpm_cmd", 10);
    odom_pub_ = this->create_publisher<nav_msgs::msg::Odometry>("/odom_wheel", 10);
    tf_broadcaster_ = std::make_unique<tf2_ros::TransformBroadcaster>(*this);

    RCLCPP_INFO(
      this->get_logger(),
      "Ackermann Kinematics Node Started. Commands -> /steer/position_cmd and /wheel_control_can/wheel_rpm_cmd");
  }

  ~AckermannKinematicsNode()
  {
    RCLCPP_INFO(this->get_logger(), "Shutting down Ackermann Kinematics Node...");
    publish_stop_command();
  }


private:
  double angle_difference(double target, double current) {
    double diff = target - current;
    while (diff > M_PI) diff -= 2.0 * M_PI;
    while (diff < -M_PI) diff += 2.0 * M_PI;
    return std::abs(diff);
  }

  double calculate_max_angle_change(const std::vector<double>& target_angles) {
    if (target_angles.size() != 4) return 0.0;

    double max_change = 0.0;
    for (size_t i = 0; i < 4; ++i) {
      double change = angle_difference(target_angles[i], current_steering_angles_[i]);
      max_change = std::max(max_change, change);
    }
    return max_change;
  }

  void limit_wheel_acceleration(
    std::vector<double>& target,
    const std::vector<double>& last,
    double dt)
  {
    if (dt <= 0.0) return;

    double max_delta = max_wheel_accel_ * dt;

    for (size_t i = 0; i < target.size(); ++i) {
      double diff = target[i] - last[i];
      diff = std::clamp(diff, -max_delta, max_delta);
      target[i] = last[i] + diff;
    }
  }

  void cmd_vel_callback(const geometry_msgs::msg::Twist::SharedPtr msg)
  {
    last_cmd_time_ = this->now();

    double vx = msg->linear.x;
    double vy = msg->linear.y;
    double wz = msg->angular.z;

    if (std::isnan(vx) || std::isnan(vy) || std::isnan(wz) || 
        std::isinf(vx) || std::isinf(vy) || std::isinf(wz)) {
      RCLCPP_WARN_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                          "Invalid cmd_vel: vx=%.3f, vy=%.3f, wz=%.3f", vx, vy, wz);
      return;
    }

    std::vector<double> wheel_vel, steer;
    try {
      std::tie(wheel_vel, steer) =
        odometry_.get_commands_omnidirectional(vx, vy, wz);
    } catch (const std::exception& e) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Error in get_commands_omnidirectional: %s", e.what());
      // 发生错误时，停止运动
      std::lock_guard<std::mutex> lock(cache_mutex_);
      cached_steering_angles_ = {0.0, 0.0, 0.0, 0.0};
      cached_wheel_velocities_ = {0.0, 0.0, 0.0, 0.0};
      return;
    }

    if (wheel_vel.size() != 4 || steer.size() != 4) {
      RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                           "Invalid command size: wheel_vel=%zu, steer=%zu", 
                           wheel_vel.size(), steer.size());
      return;
    }

    for (size_t i = 0; i < 4; ++i) {
      if (std::isnan(wheel_vel[i]) || std::isinf(wheel_vel[i]) ||
          std::isnan(steer[i]) || std::isinf(steer[i])) {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000,
                             "NaN/Inf in computed commands: wheel[%zu]=%.3f, steer[%zu]=%.3f",
                             i, wheel_vel[i], i, steer[i]);
        return;
      }
    }

    double max_delta = calculate_max_angle_change(steer);

    constexpr double MAX_DELTA = 0.6;   // rad
    constexpr double MIN_SCALE = 0.15;

    double scale = std::clamp(
      1.0 - max_delta / MAX_DELTA,
      MIN_SCALE,
      1.0
    );

    for (auto &v : wheel_vel)
      v *= scale;

    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      cached_steering_angles_ = steer;
      cached_wheel_velocities_ = wheel_vel;
    }
  }

  void steer_joint_state_callback(const sensor_msgs::msg::JointState::SharedPtr msg)
  {
    std::array<int, 4> rr_rl_fl_fr_indices = {-1, -1, -1, -1};

    for (size_t i = 0; i < msg->name.size(); ++i)
    {
      for (size_t j = 0; j < steer_joint_names_rr_rl_fl_fr_.size() && j < rr_rl_fl_fr_indices.size(); ++j)
      {
        if (msg->name[i] == steer_joint_names_rr_rl_fl_fr_[j]) {
          rr_rl_fl_fr_indices[j] = static_cast<int>(i);
        }
      }
    }

    if (msg->position.size() > static_cast<size_t>(std::max({rr_rl_fl_fr_indices[0], rr_rl_fl_fr_indices[1], rr_rl_fl_fr_indices[2], rr_rl_fl_fr_indices[3]})))
    {
      if (rr_rl_fl_fr_indices[2] >= 0) {
        current_steering_angles_[0] = msg->position[static_cast<size_t>(rr_rl_fl_fr_indices[2])];
      }
      if (rr_rl_fl_fr_indices[3] >= 0) {
        current_steering_angles_[1] = msg->position[static_cast<size_t>(rr_rl_fl_fr_indices[3])];
      }
      if (rr_rl_fl_fr_indices[1] >= 0) {
        current_steering_angles_[2] = msg->position[static_cast<size_t>(rr_rl_fl_fr_indices[1])];
      }
      if (rr_rl_fl_fr_indices[0] >= 0) {
        current_steering_angles_[3] = msg->position[static_cast<size_t>(rr_rl_fl_fr_indices[0])];
      }
    }

    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      for (size_t i = 0; i < latest_steering_angles_.size(); ++i) {
        latest_steering_angles_[i] = current_steering_angles_[i];
      }
      have_steering_feedback_ = true;
    }
  }

  void wheel_state_callback(
    const zlac8015d_four_wheel_driver_cpp::msg::FourWheelState::SharedPtr msg)
  {
    std::array<double, 4> steering_angles{};
    {
      std::lock_guard<std::mutex> lock(feedback_mutex_);
      if (!have_steering_feedback_) {
        RCLCPP_WARN_THROTTLE(
          this->get_logger(), *this->get_clock(), 3000,
          "Waiting for /steer/joint_states before publishing odom");
        return;
      }
      steering_angles = latest_steering_angles_;
    }

    rclcpp::Time current_time;
    if (msg->header.stamp.sec == 0 && msg->header.stamp.nanosec == 0) {
      current_time = this->now();
    } else {
      current_time = msg->header.stamp;
    }

    double dt = (current_time - last_odom_update_time_).seconds();
    if (dt < 1e-4) return;
    last_odom_update_time_ = current_time;

    const double vel_fl = rpm_to_rad_per_sec(msg->left_front_actual_rpm);
    const double vel_fr = rpm_to_rad_per_sec(msg->right_front_actual_rpm);
    const double vel_rl = rpm_to_rad_per_sec(msg->left_rear_actual_rpm);
    const double vel_rr = rpm_to_rad_per_sec(msg->right_rear_actual_rpm);

    const bool update_success = odometry_.update_from_velocity_4wis(
      vel_fl, vel_fr, vel_rl, vel_rr,
      steering_angles[0], steering_angles[1], steering_angles[2], steering_angles[3],
      dt);

    if (update_success) {
      publish_odom(current_time);
    }
  }

  void publish_odom(const rclcpp::Time& timestamp)
  {
    // 1. Publish /odom message
    nav_msgs::msg::Odometry odom_msg;
    odom_msg.header.stamp = timestamp;
    odom_msg.header.frame_id = odom_frame_id_;
    odom_msg.child_frame_id = base_frame_id_;

    odom_msg.pose.pose.position.x = odometry_.get_x();
    odom_msg.pose.pose.position.y = odometry_.get_y();
    odom_msg.pose.pose.position.z = 0.0;

    tf2::Quaternion q;
    q.setRPY(0, 0, odometry_.get_heading());
    odom_msg.pose.pose.orientation.x = q.x();
    odom_msg.pose.pose.orientation.y = q.y();
    odom_msg.pose.pose.orientation.z = q.z();
    odom_msg.pose.pose.orientation.w = q.w();

    odom_msg.twist.twist.linear.x = odometry_.get_linear();
    odom_msg.twist.twist.linear.y = odometry_.get_lateral();
    odom_msg.twist.twist.angular.z = odometry_.get_angular();

    odom_pub_->publish(odom_msg);

    // 2. Broadcast TF
    if (enable_odom_tf_) {
        geometry_msgs::msg::TransformStamped t;
        t.header.stamp = timestamp;
        t.header.frame_id = odom_frame_id_;
        t.child_frame_id = base_frame_id_;

        t.transform.translation.x = odometry_.get_x();
        t.transform.translation.y = odometry_.get_y();
        t.transform.translation.z = 0.0;

        t.transform.rotation = odom_msg.pose.pose.orientation;

        tf_broadcaster_->sendTransform(t);
    }
  }

  void publish_commands(const std::vector<double>& steering_angles, const std::vector<double>& wheel_velocities)
  {
    static bool first_time = true;
    if (first_time) {
      RCLCPP_INFO(this->get_logger(), 
        "Publishing commands to:\n"
        "  Steering topic: /steer/position_cmd (order [RR, RL, FL, FR])\n"
        "  Velocity topic: /wheel_control_can/wheel_rpm_cmd (order [RF, RR, LF, LR])");
      first_time = false;
    }

    std_msgs::msg::Float64MultiArray steer_msg;
    steer_msg.data = {
      steering_angles[3],
      steering_angles[2],
      steering_angles[0],
      steering_angles[1]
    };
    steering_pub_->publish(steer_msg);

    std_msgs::msg::Float32MultiArray vel_msg;
    vel_msg.data = {
      static_cast<float>(rad_per_sec_to_rpm(wheel_velocities[1])),
      static_cast<float>(rad_per_sec_to_rpm(wheel_velocities[3])),
      static_cast<float>(rad_per_sec_to_rpm(wheel_velocities[0])),
      static_cast<float>(rad_per_sec_to_rpm(wheel_velocities[2]))
    };
    velocity_pub_->publish(vel_msg);
  }

  void publish_stop_command()
  {
    publish_commands({0.0, 0.0, 0.0, 0.0}, {0.0, 0.0, 0.0, 0.0});
  }

  void command_timer_callback()
  {
    constexpr double timeout = 0.2;
    rclcpp::Time now = this->now();
    double dt = (now - last_publish_time_).seconds();
    if (dt <= 0.0) dt = 0.05;
    last_publish_time_ = now;

    std::vector<double> vel, steer;
    {
      std::lock_guard<std::mutex> lock(cache_mutex_);
      vel = cached_wheel_velocities_;
      steer = cached_steering_angles_;
    }

    if ((now - last_cmd_time_).seconds() > timeout) {
      vel = {0,0,0,0};
    }

    limit_wheel_acceleration(vel, last_wheel_vel_, dt);
    last_wheel_vel_ = vel;

    publish_commands(steer, vel);
  }

  double rad_per_sec_to_rpm(double value) const
  {
    return value * 60.0 / (2.0 * M_PI);
  }

  double rpm_to_rad_per_sec(double value) const
  {
    return value * (2.0 * M_PI) / 60.0;
  }

  // Parameters
  double max_wheel_accel_;
  double max_wheel_speed_;
  double max_steering_angle_;
  double wheelbase_;
  double track_width_;
  double wheel_radius_;
  std::string base_frame_id_;
  std::string odom_frame_id_;
  bool enable_odom_tf_;
  std::vector<std::string> steer_joint_names_rr_rl_fl_fr_;

  // Math Library
  steering_odometry::SteeringOdometry odometry_;
  
  // Timing
  rclcpp::Time last_odom_update_time_;
  rclcpp::Time last_publish_time_;                    // 上次发布命令的时间

  // 运动状态机变量
  std::vector<double> current_steering_angles_;       // 当前转向角度 [FL, FR, RL, RR]
  std::vector<double> cached_steering_angles_;        // 缓存的目标转向角度
  std::vector<double> cached_wheel_velocities_;       // 缓存的驱动轮速度
  rclcpp::Time last_cmd_time_;                        // 最近一次收到 cmd_vel 的时间
  std::vector<double> last_wheel_vel_;                // 上次发布的轮速（用于加速度限制）
  std::mutex cache_mutex_;                            // 保护缓存数据的互斥锁
  std::mutex feedback_mutex_;
  std::array<double, 4> latest_steering_angles_{0.0, 0.0, 0.0, 0.0};
  bool have_steering_feedback_{false};
  
  // Interfaces
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr cmd_vel_sub_;
  rclcpp::Subscription<sensor_msgs::msg::JointState>::SharedPtr steer_joint_state_sub_;
  rclcpp::Subscription<zlac8015d_four_wheel_driver_cpp::msg::FourWheelState>::SharedPtr wheel_state_sub_;
  
  rclcpp::Publisher<std_msgs::msg::Float64MultiArray>::SharedPtr steering_pub_;
  rclcpp::Publisher<std_msgs::msg::Float32MultiArray>::SharedPtr velocity_pub_;
  rclcpp::Publisher<nav_msgs::msg::Odometry>::SharedPtr odom_pub_;
  
  rclcpp::TimerBase::SharedPtr command_timer_;  // 命令发布定时器
  std::unique_ptr<tf2_ros::TransformBroadcaster> tf_broadcaster_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  auto node = std::make_shared<AckermannKinematicsNode>();
  rclcpp::spin(node);
  rclcpp::shutdown();
  return 0;
}

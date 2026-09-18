#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"
#include "std_srvs/srv/trigger.hpp"

namespace g1_swerve_nav
{

class CmdVelArbiter : public rclcpp::Node
{
public:
  CmdVelArbiter()
  : Node("cmd_vel_arbiter")
  {
    nav_topic_ = declare_parameter<std::string>("nav_topic", "/cmd_vel_nav");
    behavior_topic_ = declare_parameter<std::string>("behavior_topic", "/cmd_vel_behavior");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_selected");
    mode_topic_ = declare_parameter<std::string>("mode_topic", "/cmd_vel_mux/mode");
    nav_timeout_ = declare_parameter<double>("nav_timeout", 0.35);
    behavior_timeout_ = declare_parameter<double>("behavior_timeout", 0.40);
    nav_recenter_delay_sec_ = declare_parameter<double>("nav_recenter_delay_sec", 1.50);
    publish_frequency_ = declare_parameter<double>("publish_frequency", 50.0);

    nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {store_nav(*msg);});
    behavior_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      behavior_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {store_behavior(*msg);});

    auto mode_qos = rclcpp::QoS(rclcpp::KeepLast(1));
    mode_qos.reliable().transient_local();
    mode_sub_ = create_subscription<std_msgs::msg::String>(
      mode_topic_, mode_qos,
      [this](std_msgs::msg::String::SharedPtr msg) {store_mode(*msg);});

    pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);

    recenter_client_ = create_client<std_srvs::srv::Trigger>(
        "/swerve_controller/recenter_steering");

    const auto period = std::chrono::duration<double>(1.0 / publish_frequency_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelArbiter::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "autonomous cmd_vel arbiter: behavior(%s) > nav(%s) -> %s, mode=%s",
      behavior_topic_.c_str(), nav_topic_.c_str(), output_topic_.c_str(),
      mode_topic_.c_str());
  }

private:
  enum class Source { NONE, NAV, BEHAVIOR };
  enum class DriveMode { NONE, MANUAL, NAVIGATION };

  static bool valid(const geometry_msgs::msg::Twist & msg)
  {
    return std::isfinite(msg.linear.x) && std::isfinite(msg.linear.y) &&
           std::isfinite(msg.angular.z);
  }

  void store_nav(const geometry_msgs::msg::Twist & msg)
  {
    if (!valid(msg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected NaN/Inf Nav Twist");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    nav_cmd_ = msg;
    nav_stamp_ = now();
    have_nav_ = true;
  }

  void store_behavior(const geometry_msgs::msg::Twist & msg)
  {
    if (!valid(msg)) {
      RCLCPP_WARN_THROTTLE(get_logger(), *get_clock(), 1000, "Rejected NaN/Inf Behavior Twist");
      return;
    }
    std::lock_guard<std::mutex> lock(mutex_);
    behavior_cmd_ = msg;
    behavior_stamp_ = now();
    have_behavior_ = true;
  }

  void store_mode(const std_msgs::msg::String & msg)
  {
    DriveMode mode = DriveMode::NONE;
    if (msg.data == "NAVIGATION") {
      mode = DriveMode::NAVIGATION;
    } else if (msg.data == "MANUAL") {
      mode = DriveMode::MANUAL;
    } else if (msg.data != "NONE") {
      RCLCPP_WARN(
        get_logger(),
        "Unknown drive mode '%s'; treating it as NONE",
        msg.data.c_str());
    }

    {
      std::lock_guard<std::mutex> lock(mutex_);
      drive_mode_ = mode;
      if (drive_mode_ != DriveMode::NAVIGATION) {
        nav_recenter_pending_ = false;
      }
    }

    const char * name =
      mode == DriveMode::NAVIGATION ? "NAVIGATION" :
      mode == DriveMode::MANUAL ? "MANUAL" : "NONE";
    RCLCPP_INFO(get_logger(), "cmd_vel drive mode -> %s", name);
  }

  void tick()
  {
    geometry_msgs::msg::Twist output;
    Source source = Source::NONE;
    const auto stamp = now();
    bool source_changed = false;
    bool trigger_recenter = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool behavior_fresh = have_behavior_ &&
        (stamp - behavior_stamp_).seconds() <= behavior_timeout_;
      const bool nav_fresh = have_nav_ &&
        (stamp - nav_stamp_).seconds() <= nav_timeout_;

      if (behavior_fresh) {
        output = behavior_cmd_;
        source = Source::BEHAVIOR;
      } else if (nav_fresh) {
        output = nav_cmd_;
        source = Source::NAV;
      }
      const bool nav_to_none = last_source_ == Source::NAV && source == Source::NONE;
      source_changed = source != last_source_;
      last_source_ = source;

      // A raw navigation publisher going quiet is only allowed to arm recenter
      // while the final mode mux is still selecting autonomous navigation.
      if (nav_to_none && drive_mode_ == DriveMode::NAVIGATION) {
        nav_recenter_pending_ = true;
        nav_recenter_pending_since_ = stamp;
      }

      // A resumed autonomous source or leaving NAVIGATION mode invalidates the
      // idle window. This prevents recenter while the chassis is under manual
      // control, without feeding final /cmd_vel back into this arbiter.
      if (source != Source::NONE || drive_mode_ != DriveMode::NAVIGATION) {
        nav_recenter_pending_ = false;
      }

      if (nav_recenter_pending_ &&
        (stamp - nav_recenter_pending_since_).seconds() >= nav_recenter_delay_sec_)
      {
        nav_recenter_pending_ = false;
        trigger_recenter = true;
      }
    }

    if (source_changed) {
      const char * name =
        source == Source::BEHAVIOR ? "behavior" :
        source == Source::NAV ? "nav" : "none";
      RCLCPP_INFO(get_logger(), "autonomous cmd_vel active source -> %s", name);
    }

    pub_->publish(output);

    if (trigger_recenter) {
      request_recenter("navigation finished / stable idle in NAVIGATION mode");
    }
  }

  void request_recenter(const char * reason)
  {
    if (!recenter_client_->service_is_ready()) {
      RCLCPP_WARN(
        get_logger(),
        "Recenter service not ready, reason=%s",
        reason);
      return;
    }

    auto request =
      std::make_shared<std_srvs::srv::Trigger::Request>();

    (void)recenter_client_->async_send_request(request);

    RCLCPP_INFO(
      get_logger(),
      "Steering recenter requested: %s",
      reason);
  }

  std::mutex mutex_;
  geometry_msgs::msg::Twist nav_cmd_{};
  geometry_msgs::msg::Twist behavior_cmd_{};
  rclcpp::Time nav_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time behavior_stamp_{0, 0, RCL_ROS_TIME};
  bool have_nav_{false};
  bool have_behavior_{false};
  Source last_source_{Source::NONE};
  DriveMode drive_mode_{DriveMode::NONE};

  std::string nav_topic_;
  std::string behavior_topic_;
  std::string output_topic_;
  std::string mode_topic_;
  double nav_timeout_{0.35};
  double behavior_timeout_{0.40};
  double nav_recenter_delay_sec_{1.50};
  double publish_frequency_{50.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr behavior_sub_;
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr mode_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool nav_recenter_pending_{false};
  rclcpp::Time nav_recenter_pending_since_{0, 0, RCL_ROS_TIME};
  rclcpp::Client<std_srvs::srv::Trigger>::SharedPtr recenter_client_;
};

}  // namespace g1_swerve_nav

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  rclcpp::spin(std::make_shared<g1_swerve_nav::CmdVelArbiter>());
  rclcpp::shutdown();
  return 0;
}

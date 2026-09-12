#include <chrono>
#include <cmath>
#include <functional>
#include <memory>
#include <mutex>
#include <string>

#include "geometry_msgs/msg/twist.hpp"
#include "rclcpp/rclcpp.hpp"
#include "std_srvs/srv/trigger.hpp"

using std::placeholders::_1;

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
    // teleop_twist_keyboard publishes /cmd_vel by default. Keep /cmd_vel_teleop
    // as an alias so both the standard command and the explicit remap work.
    teleop_topic_ = declare_parameter<std::string>("teleop_topic", "/cmd_vel");
    teleop_alias_topic_ = declare_parameter<std::string>("teleop_alias_topic", "/cmd_vel_teleop");
    output_topic_ = declare_parameter<std::string>("output_topic", "/cmd_vel_selected");
    nav_timeout_ = declare_parameter<double>("nav_timeout", 0.35);
    behavior_timeout_ = declare_parameter<double>("behavior_timeout", 0.40);
    teleop_timeout_ = declare_parameter<double>("teleop_timeout", 0.80);
    nav_recenter_delay_sec_ = declare_parameter<double>("nav_recenter_delay_sec", 1.50);
    publish_frequency_ = declare_parameter<double>("publish_frequency", 50.0);

    nav_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      nav_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {store_nav(*msg);});
    behavior_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      behavior_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {store_behavior(*msg);});
    teleop_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      teleop_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {store_teleop(*msg);});
    teleop_alias_sub_ = create_subscription<geometry_msgs::msg::Twist>(
      teleop_alias_topic_, rclcpp::QoS(10),
      [this](geometry_msgs::msg::Twist::SharedPtr msg) {store_teleop(*msg);});
    pub_ = create_publisher<geometry_msgs::msg::Twist>(output_topic_, 10);

    recenter_client_ = create_client<std_srvs::srv::Trigger>(
        "/swerve_controller/recenter_steering");

    const auto period = std::chrono::duration<double>(1.0 / publish_frequency_);
    timer_ = create_wall_timer(
      std::chrono::duration_cast<std::chrono::nanoseconds>(period),
      std::bind(&CmdVelArbiter::tick, this));

    RCLCPP_INFO(
      get_logger(),
      "cmd_vel arbiter: teleop(%s,%s) > behavior(%s) > nav(%s) -> %s",
      teleop_topic_.c_str(), teleop_alias_topic_.c_str(), behavior_topic_.c_str(),
      nav_topic_.c_str(), output_topic_.c_str());
  }

private:
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

  void store_teleop(const geometry_msgs::msg::Twist & msg)
  {
    if (!valid(msg)) {
      RCLCPP_WARN_THROTTLE(
        get_logger(), *get_clock(), 1000,
        "Rejected NaN/Inf Teleop Twist");
      return;
    }

    const bool zero = zero_twist(msg);
    bool trigger_recenter = false;

    {
      std::lock_guard<std::mutex> lock(mutex_);

      teleop_cmd_ = msg;
      teleop_stamp_ = now();
      have_teleop_ = true;

      // teleop_twist_keyboard 的 k 会发送全零 Twist
      if (zero && !teleop_zero_latched_) {
        teleop_zero_latched_ = true;
        trigger_recenter = true;
      } else if (!zero) {
        teleop_zero_latched_ = false;
      }
    }

    if (trigger_recenter) {
      request_recenter("teleop zero / keyboard k");
    }
  }

  void tick()
  {
    enum class Source { NONE, NAV, BEHAVIOR, TELEOP };
    geometry_msgs::msg::Twist output;
    Source source = Source::NONE;
    const auto stamp = now();
    {
      std::lock_guard<std::mutex> lock(mutex_);
      const bool teleop_fresh = have_teleop_ &&
        (stamp - teleop_stamp_).seconds() <= teleop_timeout_;
      const bool behavior_fresh = have_behavior_ &&
        (stamp - behavior_stamp_).seconds() <= behavior_timeout_;
      const bool nav_fresh = have_nav_ &&
        (stamp - nav_stamp_).seconds() <= nav_timeout_;

      if (teleop_fresh) {
        output = teleop_cmd_;
        source = Source::TELEOP;
      } else if (behavior_fresh) {
        output = behavior_cmd_;
        source = Source::BEHAVIOR;
      } else if (nav_fresh) {
        output = nav_cmd_;
        source = Source::NAV;
      }
    }

    const int source_id = static_cast<int>(source);

    const bool nav_to_none =
      last_source_id_ == static_cast<int>(Source::NAV) &&
      source == Source::NONE;

    if (source_id != last_source_id_) {
      const char * name =
        source == Source::TELEOP ? "teleop" :
        source == Source::BEHAVIOR ? "behavior" :
        source == Source::NAV ? "nav" : "none";

      RCLCPP_INFO(
        get_logger(),
        "cmd_vel active source -> %s",
        name);

      last_source_id_ = source_id;
    }

    pub_->publish(output);

    // NAV -> NONE is not sufficient evidence that navigation finished. During
    // recovery, TF hiccups, or controller restarts Nav2 can briefly stop
    // publishing velocity commands and then resume. Arm a pending recenter, but
    // only execute it after a stable idle window with no active command source.
    if (nav_to_none) {
      nav_recenter_pending_ = true;
      nav_recenter_pending_since_ = stamp;
    }

    // Any active source means the idle gap was temporary, so do not recenter.
    if (source != Source::NONE) {
      nav_recenter_pending_ = false;
    }

    if (nav_recenter_pending_ &&
      source == Source::NONE &&
      (stamp - nav_recenter_pending_since_).seconds() >= nav_recenter_delay_sec_)
    {
      nav_recenter_pending_ = false;
      request_recenter("navigation finished / stable idle");
    }
  }

  static bool zero_twist(const geometry_msgs::msg::Twist & msg)
  {
    return
      std::abs(msg.linear.x) < 1e-6 &&
      std::abs(msg.linear.y) < 1e-6 &&
      std::abs(msg.angular.z) < 1e-6;
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
  geometry_msgs::msg::Twist teleop_cmd_{};
  rclcpp::Time nav_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time behavior_stamp_{0, 0, RCL_ROS_TIME};
  rclcpp::Time teleop_stamp_{0, 0, RCL_ROS_TIME};
  bool have_nav_{false};
  bool have_behavior_{false};
  bool have_teleop_{false};
  int last_source_id_{-1};

  std::string nav_topic_;
  std::string behavior_topic_;
  std::string teleop_topic_;
  std::string teleop_alias_topic_;
  std::string output_topic_;
  double nav_timeout_{0.35};
  double behavior_timeout_{0.40};
  double teleop_timeout_{0.80};
  double nav_recenter_delay_sec_{1.50};
  double publish_frequency_{50.0};

  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr nav_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr behavior_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_sub_;
  rclcpp::Subscription<geometry_msgs::msg::Twist>::SharedPtr teleop_alias_sub_;
  rclcpp::Publisher<geometry_msgs::msg::Twist>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;

  bool teleop_zero_latched_{false};
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

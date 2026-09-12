#include <arpa/inet.h>
#include <linux/can.h>
#include <linux/can/raw.h>
#include <net/if.h>
#include <sys/ioctl.h>
#include <sys/select.h>
#include <sys/socket.h>
#include <unistd.h>

#include <algorithm>
#include <array>
#include <chrono>
#include <cmath>
#include <cstdint>
#include <cstring>
#include <memory>
#include <mutex>
#include <optional>
#include <sstream>
#include <thread>
#include <stdexcept>
#include <string>
#include <vector>

#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/float32_multi_array.hpp"
#include "std_srvs/srv/trigger.hpp"
#include "zlac8015d_four_wheel_driver_cpp/msg/four_wheel_state.hpp"
#include "zlac8015d_four_wheel_driver_cpp/srv/set_four_wheel_speeds.hpp"

using namespace std::chrono_literals;
using SetFourWheelSpeeds = zlac8015d_four_wheel_driver_cpp::srv::SetFourWheelSpeeds;
using FourWheelState = zlac8015d_four_wheel_driver_cpp::msg::FourWheelState;

namespace
{
constexpr uint16_t INDEX_CONTROLWORD = 0x6040;
constexpr uint16_t INDEX_STATUSWORD = 0x6041;
constexpr uint16_t INDEX_MODE_OF_OPERATION = 0x6060;
constexpr uint16_t INDEX_MODE_DISPLAY = 0x6061;
constexpr uint16_t INDEX_TARGET_SPEED = 0x60FF;
constexpr uint16_t INDEX_FAULT_CODE = 0x603F;
constexpr uint16_t INDEX_SYNC_ASYNC_FLAG = 0x200F;
constexpr uint16_t INDEX_ACCEL_TIME = 0x6083;
constexpr uint16_t INDEX_DECEL_TIME = 0x6084;
constexpr uint16_t INDEX_ACTUAL_SPEED = 0x606C;
constexpr uint16_t INDEX_RPDO1_COMM_PARAM = 0x1401;
constexpr uint16_t INDEX_RPDO1_MAPPING = 0x1601;

constexpr uint16_t CONTROLWORD_SHUTDOWN = 0x0006;
constexpr uint16_t CONTROLWORD_SWITCH_ON = 0x0007;
constexpr uint16_t CONTROLWORD_ENABLE_OP = 0x000F;
constexpr uint16_t CONTROLWORD_DISABLE_VOLTAGE = 0x0000;
constexpr uint16_t CONTROLWORD_FAULT_RESET = 0x0080;
constexpr int8_t MODE_PROFILE_VELOCITY = 3;
constexpr uint8_t PDO_TRANSMISSION_EVENT = 0xFE;
constexpr uint32_t RPDO1_DEFAULT_COB_ID_BASE = 0x300;
constexpr uint32_t TARGET_SPEED_COMBINED_MAPPING = 0x60FF0320;

int16_t clamp_rpm(double value)
{
  const auto rounded = std::llround(value);
  return static_cast<int16_t>(std::clamp<long long>(rounded, -1000LL, 1000LL));
}

std::string hex_u32(uint32_t v)
{
  std::ostringstream oss;
  oss << "0x" << std::hex << std::uppercase << v;
  return oss.str();
}

std::string decode_cia402_state(uint16_t sw)
{
  const bool b0 = (sw & (1u << 0)) != 0;
  const bool b1 = (sw & (1u << 1)) != 0;
  const bool b2 = (sw & (1u << 2)) != 0;
  const bool b3 = (sw & (1u << 3)) != 0;
  const bool b5 = (sw & (1u << 5)) != 0;
  const bool b6 = (sw & (1u << 6)) != 0;

  std::string state = "unknown";
  if (!b6 && !b5 && !b3 && !b2 && !b1 && !b0) {
    state = "not_ready_to_switch_on";
  } else if (b6 && !b3 && !b2 && !b1 && !b0) {
    state = "switch_on_disabled";
  } else if (!b6 && b5 && !b3 && !b2 && !b1 && b0) {
    state = "ready_to_switch_on";
  } else if (!b6 && b5 && !b3 && !b2 && b1 && b0) {
    state = "switched_on";
  } else if (!b6 && b5 && !b3 && b2 && b1 && b0) {
    state = "operation_enabled";
  } else if (!b6 && !b5 && !b3 && b2 && b1 && b0) {
    state = "quick_stop_active";
  } else if (!b6 && b3 && !b2 && !b1 && !b0) {
    state = "fault";
  } else if (!b6 && b3 && b2 && b1 && b0) {
    state = "fault_reaction_active";
  }

  std::ostringstream oss;
  oss << state
      << ",alarm=" << (((sw >> 7) & 0x1) ? "1" : "0")
      << ",running=" << (((sw >> 14) & 0x1) ? "1" : "0")
      << ",ext_estop=" << (((sw >> 15) & 0x1) ? "1" : "0");
  return oss.str();
}

std::string decode_motor_state(uint16_t val)
{
  switch (val) {
    case 0: return "stopped";
    case 1: return "running";
    default: {
      std::ostringstream oss;
      oss << "unknown(" << val << ")";
      return oss.str();
    }
  }
}

uint16_t motor_state_from_status(uint16_t status_word)
{
  return static_cast<uint16_t>(((status_word >> 14) & 0x1) ? 1 : 0);
}

struct ControllerSnapshot
{
  uint16_t left_status{0};
  uint16_t right_status{0};
  float left_actual_rpm{0.0f};
  float right_actual_rpm{0.0f};
  uint16_t left_motor_state{0};
  uint16_t right_motor_state{0};
  uint32_t fault_code{0};
};
}  // namespace

class SocketCanInterface
{
public:
  explicit SocketCanInterface(const std::string & ifname)
  {
    sock_ = ::socket(PF_CAN, SOCK_RAW, CAN_RAW);
    if (sock_ < 0) {
      throw std::runtime_error("socket() failed: " + std::string(std::strerror(errno)));
    }

    struct ifreq ifr {};
    std::snprintf(ifr.ifr_name, IFNAMSIZ, "%s", ifname.c_str());
    if (::ioctl(sock_, SIOCGIFINDEX, &ifr) < 0) {
      const auto msg = "ioctl(SIOCGIFINDEX) failed for " + ifname + ": " + std::string(std::strerror(errno));
      ::close(sock_);
      throw std::runtime_error(msg);
    }

    struct sockaddr_can addr {};
    addr.can_family = AF_CAN;
    addr.can_ifindex = ifr.ifr_ifindex;
    if (::bind(sock_, reinterpret_cast<struct sockaddr *>(&addr), sizeof(addr)) < 0) {
      const auto msg = "bind() failed for " + ifname + ": " + std::string(std::strerror(errno));
      ::close(sock_);
      throw std::runtime_error(msg);
    }
  }

  ~SocketCanInterface()
  {
    if (sock_ >= 0) {
      ::close(sock_);
    }
  }

  void send_frame(uint32_t can_id, const std::vector<uint8_t> & data)
  {
    if (data.size() > 8) {
      throw std::runtime_error("CAN frame data too long");
    }
    struct can_frame frame {};
    frame.can_id = can_id;
    frame.can_dlc = static_cast<__u8>(data.size());
    std::copy(data.begin(), data.end(), frame.data);

    const auto nbytes = ::write(sock_, &frame, sizeof(frame));
    if (nbytes != static_cast<ssize_t>(sizeof(frame))) {
      throw std::runtime_error("write() failed: " + std::string(std::strerror(errno)));
    }
  }

  std::optional<can_frame> recv_frame(std::chrono::milliseconds timeout)
  {
    fd_set readset;
    FD_ZERO(&readset);
    FD_SET(sock_, &readset);

    struct timeval tv {};
    tv.tv_sec = static_cast<long>(timeout.count() / 1000);
    tv.tv_usec = static_cast<long>((timeout.count() % 1000) * 1000);

    const int ret = ::select(sock_ + 1, &readset, nullptr, nullptr, &tv);
    if (ret < 0) {
      throw std::runtime_error("select() failed: " + std::string(std::strerror(errno)));
    }
    if (ret == 0) {
      return std::nullopt;
    }

    struct can_frame frame {};
    const auto nbytes = ::read(sock_, &frame, sizeof(frame));
    if (nbytes < 0) {
      throw std::runtime_error("read() failed: " + std::string(std::strerror(errno)));
    }
    if (nbytes < static_cast<ssize_t>(sizeof(struct can_frame))) {
      return std::nullopt;
    }
    return frame;
  }

private:
  int sock_{-1};
};

class FourWheelDriverNode : public rclcpp::Node
{
public:
  FourWheelDriverNode()
  : Node("zlac8015d_four_wheel_driver")
  {
    can_interface_ = this->declare_parameter<std::string>("can_interface", "can0");
    right_controller_node_id_ = this->declare_parameter<int>("right_controller_node_id", 1);
    left_controller_node_id_ = this->declare_parameter<int>("left_controller_node_id", 2);
    control_hz_ = this->declare_parameter<double>("control_hz", 50.0);
    state_hz_ = this->declare_parameter<double>("state_hz", 10.0);
    command_timeout_sec_ = this->declare_parameter<double>("command_timeout_sec", 0.3);
    accel_ms_ = this->declare_parameter<int>("accel_ms", 100);
    decel_ms_ = this->declare_parameter<int>("decel_ms", 100);
    sync_control_ = this->declare_parameter<bool>("sync_control", true);
    use_pdo_commands_ = this->declare_parameter<bool>("use_pdo_commands", true);
    configure_pdo_mapping_ = this->declare_parameter<bool>("configure_pdo_mapping", true);
    sdo_timeout_ms_ = this->declare_parameter<int>("sdo_timeout_ms", 200);
    state_sdo_timeout_ms_ = this->declare_parameter<int>("state_sdo_timeout_ms", 50);
    right_wheel_sign_ = this->declare_parameter<double>("right_wheel_sign", -1.0);
    left_wheel_sign_ = this->declare_parameter<double>("left_wheel_sign", 1.0);
    emergency_stop_shutdown_enabled_ =
      this->declare_parameter<bool>("emergency_stop_shutdown_enabled", true);
    state_read_failure_shutdown_consecutive_cycles_ = static_cast<int>(
      std::max<std::int64_t>(
        1,
        this->declare_parameter<std::int64_t>(
          "state_read_failure_shutdown_consecutive_cycles", 5)));

    can_ = std::make_unique<SocketCanInterface>(can_interface_);

    state_pub_ = this->create_publisher<FourWheelState>("state", 10);

    cmd_sub_ = this->create_subscription<std_msgs::msg::Float32MultiArray>(
      "wheel_rpm_cmd", 10,
      std::bind(&FourWheelDriverNode::on_wheel_rpm_cmd, this, std::placeholders::_1));

    set_speed_srv_ = this->create_service<SetFourWheelSpeeds>(
      "set_wheel_speeds",
      std::bind(&FourWheelDriverNode::on_set_wheel_speeds, this, std::placeholders::_1, std::placeholders::_2));

    enable_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "enable",
      std::bind(&FourWheelDriverNode::on_enable, this, std::placeholders::_1, std::placeholders::_2));

    disable_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "disable",
      std::bind(&FourWheelDriverNode::on_disable, this, std::placeholders::_1, std::placeholders::_2));

    clear_fault_srv_ = this->create_service<std_srvs::srv::Trigger>(
      "clear_fault",
      std::bind(&FourWheelDriverNode::on_clear_fault, this, std::placeholders::_1, std::placeholders::_2));

    last_cmd_time_ = this->now();

    control_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / control_hz_)),
      std::bind(&FourWheelDriverNode::control_loop, this));

    state_timer_ = this->create_wall_timer(
      std::chrono::duration_cast<std::chrono::milliseconds>(std::chrono::duration<double>(1.0 / state_hz_)),
      std::bind(&FourWheelDriverNode::publish_state, this));

    RCLCPP_INFO(
      this->get_logger(),
      "Started on %s, right_controller_id=%d, left_controller_id=%d, use_pdo_commands=%s, sync_control=%s. Startup will NOT auto-enable or auto-clear faults.",
      can_interface_.c_str(), right_controller_node_id_, left_controller_node_id_,
      use_pdo_commands_ ? "true" : "false", sync_control_ ? "true" : "false");
  }

  bool emergency_shutdown_requested() const
  {
    return emergency_shutdown_requested_;
  }

private:
  std::chrono::milliseconds sdo_timeout() const
  {
    return std::chrono::milliseconds(std::max(1, sdo_timeout_ms_));
  }

  std::chrono::milliseconds state_sdo_timeout() const
  {
    return std::chrono::milliseconds(std::max(1, state_sdo_timeout_ms_));
  }

  int16_t apply_right_sign(double value) const
  {
    return clamp_rpm(right_wheel_sign_ * value);
  }

  int16_t apply_left_sign(double value) const
  {
    return clamp_rpm(left_wheel_sign_ * value);
  }

  float controller_to_logical_right(float value) const
  {
    return static_cast<float>(right_wheel_sign_) * value;
  }

  float controller_to_logical_left(float value) const
  {
    return static_cast<float>(left_wheel_sign_) * value;
  }

  void on_wheel_rpm_cmd(const std_msgs::msg::Float32MultiArray::SharedPtr msg)
  {
    if (msg->data.size() != 4) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "wheel_rpm_cmd expects 4 values: [right_front, right_rear, left_front, left_rear], got %zu",
        msg->data.size());
      return;
    }

    std::scoped_lock<std::mutex> lock(cmd_mutex_);
    if (!drivers_enabled_) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "drivers are not enabled; call /enable first");
      return;
    }

    target_right_front_rpm_ = apply_right_sign(msg->data[0]);
    target_right_rear_rpm_ = apply_right_sign(msg->data[1]);
    target_left_front_rpm_ = apply_left_sign(msg->data[2]);
    target_left_rear_rpm_ = apply_left_sign(msg->data[3]);
    last_cmd_time_ = this->now();
  }

  void on_set_wheel_speeds(
    const std::shared_ptr<SetFourWheelSpeeds::Request> request,
    std::shared_ptr<SetFourWheelSpeeds::Response> response)
  {
    std::scoped_lock<std::mutex> lock(cmd_mutex_);
    if (!drivers_enabled_) {
      response->success = false;
      response->message = "drivers are not enabled; call /enable first";
      return;
    }

    target_right_front_rpm_ = apply_right_sign(request->right_front_rpm);
    target_right_rear_rpm_ = apply_right_sign(request->right_rear_rpm);
    target_left_front_rpm_ = apply_left_sign(request->left_front_rpm);
    target_left_rear_rpm_ = apply_left_sign(request->left_rear_rpm);
    last_cmd_time_ = this->now();

    try {
      // ID=1 right side: left motor=right rear, right motor=right front
      set_dual_speed(right_controller_node_id_, target_right_rear_rpm_, target_right_front_rpm_);
      // ID=2 left side: left motor=left front, right motor=left rear
      set_dual_speed(left_controller_node_id_, target_left_front_rpm_, target_left_rear_rpm_);
      last_sent_right_front_rpm_ = target_right_front_rpm_;
      last_sent_right_rear_rpm_ = target_right_rear_rpm_;
      last_sent_left_front_rpm_ = target_left_front_rpm_;
      last_sent_left_rear_rpm_ = target_left_rear_rpm_;
      response->success = true;
      response->message = "speed command accepted by both controllers";
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("speed command failed: ") + e.what();
    }
  }

  void on_enable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      send_nmt_preoperational(right_controller_node_id_);
      send_nmt_preoperational(left_controller_node_id_);
      configure_velocity_mode(right_controller_node_id_);
      configure_velocity_mode(left_controller_node_id_);
      send_nmt_start(right_controller_node_id_);
      send_nmt_start(left_controller_node_id_);
      enable_sequence(right_controller_node_id_);
      enable_sequence(left_controller_node_id_);
      drivers_enabled_ = true;
      response->success = true;
      response->message = "both controllers enabled and configured for velocity mode";
    } catch (const std::exception & e) {
      drivers_enabled_ = false;
      response->success = false;
      response->message = std::string("enable failed: ") + e.what();
    }
  }

  void on_disable(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      stop_and_disable(right_controller_node_id_);
      stop_and_disable(left_controller_node_id_);
      {
        std::scoped_lock<std::mutex> lock(cmd_mutex_);
        target_right_front_rpm_ = 0;
        target_right_rear_rpm_ = 0;
        target_left_front_rpm_ = 0;
        target_left_rear_rpm_ = 0;
        last_sent_right_front_rpm_ = 0;
        last_sent_right_rear_rpm_ = 0;
        last_sent_left_front_rpm_ = 0;
        last_sent_left_rear_rpm_ = 0;
      }
      drivers_enabled_ = false;
      response->success = true;
      response->message = "both controllers disabled";
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("disable failed: ") + e.what();
    }
  }

  void on_clear_fault(
    const std::shared_ptr<std_srvs::srv::Trigger::Request> /*request*/,
    std::shared_ptr<std_srvs::srv::Trigger::Response> response)
  {
    try {
      write_u16(right_controller_node_id_, INDEX_CONTROLWORD, 0x00, CONTROLWORD_FAULT_RESET);
      write_u16(left_controller_node_id_, INDEX_CONTROLWORD, 0x00, CONTROLWORD_FAULT_RESET);
      response->success = true;
      response->message = "clear fault command acknowledged by both controllers";
    } catch (const std::exception & e) {
      response->success = false;
      response->message = std::string("clear fault failed: ") + e.what();
    }
  }

  void control_loop()
  {
    try {
      if (!drivers_enabled_) {
        return;
      }

      int16_t rf = 0;
      int16_t rr = 0;
      int16_t lf = 0;
      int16_t lr = 0;
      {
        std::scoped_lock<std::mutex> lock(cmd_mutex_);
        const auto age = (this->now() - last_cmd_time_).seconds();
        if (age <= command_timeout_sec_) {
          rf = target_right_front_rpm_;
          rr = target_right_rear_rpm_;
          lf = target_left_front_rpm_;
          lr = target_left_rear_rpm_;
        }
      }

      set_dual_speed(right_controller_node_id_, rr, rf);
      set_dual_speed(left_controller_node_id_, lf, lr);

      last_sent_right_front_rpm_ = rf;
      last_sent_right_rear_rpm_ = rr;
      last_sent_left_front_rpm_ = lf;
      last_sent_left_rear_rpm_ = lr;
    } catch (const std::exception & e) {
      RCLCPP_ERROR_THROTTLE(
        this->get_logger(), *this->get_clock(), 2000,
        "control loop failed: %s", e.what());
    }
  }

  void publish_state()
  {
    try {
      const auto right_snapshot = read_snapshot(right_controller_node_id_);
      const auto left_snapshot = read_snapshot(left_controller_node_id_);

      FourWheelState msg;
      msg.header.stamp = this->now();

      msg.right_front_cmd_rpm = controller_to_logical_right(static_cast<float>(last_sent_right_front_rpm_));
      msg.right_rear_cmd_rpm = controller_to_logical_right(static_cast<float>(last_sent_right_rear_rpm_));
      msg.left_front_cmd_rpm = controller_to_logical_left(static_cast<float>(last_sent_left_front_rpm_));
      msg.left_rear_cmd_rpm = controller_to_logical_left(static_cast<float>(last_sent_left_rear_rpm_));

      msg.right_front_actual_rpm = controller_to_logical_right(right_snapshot.right_actual_rpm);
      msg.right_rear_actual_rpm = controller_to_logical_right(right_snapshot.left_actual_rpm);
      msg.left_front_actual_rpm = controller_to_logical_left(left_snapshot.left_actual_rpm);
      msg.left_rear_actual_rpm = controller_to_logical_left(left_snapshot.right_actual_rpm);

      msg.right_front_motor_state = decode_motor_state(right_snapshot.right_motor_state);
      msg.right_rear_motor_state = decode_motor_state(right_snapshot.left_motor_state);
      msg.left_front_motor_state = decode_motor_state(left_snapshot.left_motor_state);
      msg.left_rear_motor_state = decode_motor_state(left_snapshot.right_motor_state);

      msg.right_driver_state = summarize_driver_state(right_snapshot);
      msg.left_driver_state = summarize_driver_state(left_snapshot);

      msg.right_fault_code = right_snapshot.fault_code;
      msg.left_fault_code = left_snapshot.fault_code;

      state_pub_->publish(msg);
      state_read_failure_count_ = 0;
    } catch (const std::exception & e) {
      RCLCPP_WARN_THROTTLE(
        this->get_logger(), *this->get_clock(), 3000,
        "state read failed: %s", e.what());
      handle_state_read_failure(e.what());
    }
  }

  void handle_state_read_failure(const std::string & reason)
  {
    if (!emergency_stop_shutdown_enabled_ || emergency_shutdown_requested_) {
      return;
    }

    ++state_read_failure_count_;
    if (state_read_failure_count_ < state_read_failure_shutdown_consecutive_cycles_) {
      return;
    }

    request_emergency_shutdown(
      "state read failed for " + std::to_string(state_read_failure_count_) +
      " consecutive cycles: " + reason);
  }

  void request_emergency_shutdown(const std::string & reason)
  {
    if (emergency_shutdown_requested_) {
      return;
    }

    emergency_shutdown_requested_ = true;
    drivers_enabled_ = false;
    {
      std::scoped_lock<std::mutex> lock(cmd_mutex_);
      target_right_front_rpm_ = 0;
      target_right_rear_rpm_ = 0;
      target_left_front_rpm_ = 0;
      target_left_rear_rpm_ = 0;
    }
    if (control_timer_) {
      control_timer_->cancel();
    }
    if (state_timer_) {
      state_timer_->cancel();
    }

    RCLCPP_ERROR(
      this->get_logger(),
      "Emergency stop / motor power loss detected by ZLAC state communication failure; requesting node shutdown: %s",
      reason.c_str());
    rclcpp::shutdown();
  }

  std::string summarize_driver_state(const ControllerSnapshot & s) const
  {
    const auto left = decode_cia402_state(s.left_status);
    const auto right = decode_cia402_state(s.right_status);
    if (left == right) {
      return left;
    }
    return std::string("mixed(left=") + left + ",right=" + right + ")";
  }

  void configure_velocity_mode(int node_id)
  {
    write_u16(node_id, INDEX_SYNC_ASYNC_FLAG, 0x00, sync_control_ ? 1u : 0u);
    if (use_pdo_commands_ && configure_pdo_mapping_) {
      configure_velocity_rpdo_mapping(node_id);
    }
    write_u8(node_id, INDEX_MODE_OF_OPERATION, 0x00, static_cast<uint8_t>(MODE_PROFILE_VELOCITY));
    const auto mode_display = read_u8(node_id, INDEX_MODE_DISPLAY, 0x00);
    if (mode_display != static_cast<uint8_t>(MODE_PROFILE_VELOCITY)) {
      throw std::runtime_error("mode display mismatch on node " + std::to_string(node_id));
    }
    write_u32(node_id, INDEX_ACCEL_TIME, 0x01, static_cast<uint32_t>(accel_ms_));
    write_u32(node_id, INDEX_ACCEL_TIME, 0x02, static_cast<uint32_t>(accel_ms_));
    write_u32(node_id, INDEX_DECEL_TIME, 0x01, static_cast<uint32_t>(decel_ms_));
    write_u32(node_id, INDEX_DECEL_TIME, 0x02, static_cast<uint32_t>(decel_ms_));
  }

  void configure_velocity_rpdo_mapping(int node_id)
  {
    write_u8(node_id, INDEX_RPDO1_MAPPING, 0x00, 0x00);
    write_u32(node_id, INDEX_RPDO1_MAPPING, 0x01, TARGET_SPEED_COMBINED_MAPPING);
    write_u8(node_id, INDEX_RPDO1_COMM_PARAM, 0x02, PDO_TRANSMISSION_EVENT);
    write_u8(node_id, INDEX_RPDO1_MAPPING, 0x00, 0x01);
    RCLCPP_INFO(
      this->get_logger(),
      "Configured node %d RPDO1: COB-ID=%s maps 0x60FF:03 combined target speed",
      node_id, hex_u32(RPDO1_DEFAULT_COB_ID_BASE + static_cast<uint32_t>(node_id)).c_str());
  }

  void enable_sequence(int node_id)
  {
    write_u16(node_id, INDEX_CONTROLWORD, 0x00, CONTROLWORD_SHUTDOWN);
    std::this_thread::sleep_for(20ms);
    write_u16(node_id, INDEX_CONTROLWORD, 0x00, CONTROLWORD_SWITCH_ON);
    std::this_thread::sleep_for(20ms);
    write_u16(node_id, INDEX_CONTROLWORD, 0x00, CONTROLWORD_ENABLE_OP);
    std::this_thread::sleep_for(20ms);
  }

  void stop_and_disable(int node_id)
  {
    set_dual_speed(node_id, 0, 0);
    write_u16(node_id, INDEX_CONTROLWORD, 0x00, CONTROLWORD_DISABLE_VOLTAGE);
  }

  void send_nmt_preoperational(int node_id)
  {
    std::scoped_lock<std::mutex> lock(can_mutex_);
    can_->send_frame(0x000, {0x80, static_cast<uint8_t>(node_id)});
  }

  void send_nmt_start(int node_id)
  {
    std::scoped_lock<std::mutex> lock(can_mutex_);
    can_->send_frame(0x000, {0x01, static_cast<uint8_t>(node_id)});
  }

  ControllerSnapshot read_snapshot(int node_id)
  {
    ControllerSnapshot s;
    const auto timeout = state_sdo_timeout();
    const auto status = read_u32(node_id, INDEX_STATUSWORD, 0x00, timeout);
    const auto actual = read_u32(node_id, INDEX_ACTUAL_SPEED, 0x03, timeout);
    const auto fault = read_u32(node_id, INDEX_FAULT_CODE, 0x00, timeout);

    s.left_status = static_cast<uint16_t>(status & 0xFFFF);
    s.right_status = static_cast<uint16_t>((status >> 16) & 0xFFFF);
    s.left_actual_rpm = static_cast<float>(static_cast<int16_t>(actual & 0xFFFF)) / 10.0f;
    s.right_actual_rpm = static_cast<float>(static_cast<int16_t>((actual >> 16) & 0xFFFF)) / 10.0f;
    s.left_motor_state = motor_state_from_status(s.left_status);
    s.right_motor_state = motor_state_from_status(s.right_status);
    s.fault_code = fault;
    return s;
  }

  void set_dual_speed(int node_id, int16_t left_rpm, int16_t right_rpm)
  {
    if (use_pdo_commands_) {
      send_dual_speed_rpdo(node_id, left_rpm, right_rpm);
    } else {
      write_dual_speed_sdo(node_id, left_rpm, right_rpm);
    }
  }

  void write_dual_speed_sdo(int node_id, int16_t left_rpm, int16_t right_rpm)
  {
    const uint32_t data = static_cast<uint16_t>(left_rpm) |
      (static_cast<uint32_t>(static_cast<uint16_t>(right_rpm)) << 16);
    write_u32(node_id, INDEX_TARGET_SPEED, 0x03, data);
  }

  void send_dual_speed_rpdo(int node_id, int16_t left_rpm, int16_t right_rpm)
  {
    const uint32_t data = static_cast<uint16_t>(left_rpm) |
      (static_cast<uint32_t>(static_cast<uint16_t>(right_rpm)) << 16);
    const uint32_t cob_id = RPDO1_DEFAULT_COB_ID_BASE + static_cast<uint32_t>(node_id);
    std::vector<uint8_t> req = {
      static_cast<uint8_t>(data & 0xFF),
      static_cast<uint8_t>((data >> 8) & 0xFF),
      static_cast<uint8_t>((data >> 16) & 0xFF),
      static_cast<uint8_t>((data >> 24) & 0xFF)};
    std::scoped_lock<std::mutex> lock(can_mutex_);
    can_->send_frame(cob_id, req);
  }

  void write_u8(int node_id, uint16_t index, uint8_t subindex, uint8_t value)
  {
    const uint32_t cob_id = 0x600 + static_cast<uint32_t>(node_id);
    std::vector<uint8_t> req = {
      0x2F,
      static_cast<uint8_t>(index & 0xFF),
      static_cast<uint8_t>((index >> 8) & 0xFF),
      subindex,
      value, 0x00, 0x00, 0x00};
    std::scoped_lock<std::mutex> lock(can_mutex_);
    can_->send_frame(cob_id, req);
    expect_sdo_write_ack(node_id, index, subindex, sdo_timeout());
  }

  void write_u16(int node_id, uint16_t index, uint8_t subindex, uint16_t value)
  {
    const uint32_t cob_id = 0x600 + static_cast<uint32_t>(node_id);
    std::vector<uint8_t> req = {
      0x2B,
      static_cast<uint8_t>(index & 0xFF),
      static_cast<uint8_t>((index >> 8) & 0xFF),
      subindex,
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
      0x00,
      0x00};
    std::scoped_lock<std::mutex> lock(can_mutex_);
    can_->send_frame(cob_id, req);
    expect_sdo_write_ack(node_id, index, subindex, sdo_timeout());
  }

  void write_u32(int node_id, uint16_t index, uint8_t subindex, uint32_t value)
  {
    const uint32_t cob_id = 0x600 + static_cast<uint32_t>(node_id);
    std::vector<uint8_t> req = {
      0x23,
      static_cast<uint8_t>(index & 0xFF),
      static_cast<uint8_t>((index >> 8) & 0xFF),
      subindex,
      static_cast<uint8_t>(value & 0xFF),
      static_cast<uint8_t>((value >> 8) & 0xFF),
      static_cast<uint8_t>((value >> 16) & 0xFF),
      static_cast<uint8_t>((value >> 24) & 0xFF)};
    std::scoped_lock<std::mutex> lock(can_mutex_);
    can_->send_frame(cob_id, req);
    expect_sdo_write_ack(node_id, index, subindex, sdo_timeout());
  }

  uint8_t read_u8(int node_id, uint16_t index, uint8_t subindex)
  {
    const auto resp = sdo_read(node_id, index, subindex, sdo_timeout());
    return resp[4];
  }

  uint16_t read_u16(int node_id, uint16_t index, uint8_t subindex)
  {
    const auto resp = sdo_read(node_id, index, subindex, sdo_timeout());
    return static_cast<uint16_t>(resp[4]) | (static_cast<uint16_t>(resp[5]) << 8);
  }

  uint32_t read_u32(int node_id, uint16_t index, uint8_t subindex)
  {
    const auto resp = sdo_read(node_id, index, subindex, sdo_timeout());
    return static_cast<uint32_t>(resp[4]) |
           (static_cast<uint32_t>(resp[5]) << 8) |
           (static_cast<uint32_t>(resp[6]) << 16) |
           (static_cast<uint32_t>(resp[7]) << 24);
  }

  uint32_t read_u32(
    int node_id, uint16_t index, uint8_t subindex, std::chrono::milliseconds timeout)
  {
    const auto resp = sdo_read(node_id, index, subindex, timeout);
    return static_cast<uint32_t>(resp[4]) |
           (static_cast<uint32_t>(resp[5]) << 8) |
           (static_cast<uint32_t>(resp[6]) << 16) |
           (static_cast<uint32_t>(resp[7]) << 24);
  }

  std::array<uint8_t, 8> sdo_read(
    int node_id, uint16_t index, uint8_t subindex, std::chrono::milliseconds timeout)
  {
    const uint32_t cob_id = 0x600 + static_cast<uint32_t>(node_id);
    std::vector<uint8_t> req = {
      0x40,
      static_cast<uint8_t>(index & 0xFF),
      static_cast<uint8_t>((index >> 8) & 0xFF),
      subindex,
      0, 0, 0, 0};
    std::scoped_lock<std::mutex> lock(can_mutex_);
    can_->send_frame(cob_id, req);

    const uint32_t resp_id = 0x580 + static_cast<uint32_t>(node_id);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      auto frame = can_->recv_frame(20ms);
      if (!frame.has_value()) {
        continue;
      }
      if ((frame->can_id & CAN_EFF_MASK) != resp_id) {
        continue;
      }
      if (frame->data[1] != static_cast<uint8_t>(index & 0xFF) ||
          frame->data[2] != static_cast<uint8_t>((index >> 8) & 0xFF) ||
          frame->data[3] != subindex) {
        continue;
      }
      if (frame->data[0] == 0x80) {
        const uint32_t abort_code = static_cast<uint32_t>(frame->data[4]) |
          (static_cast<uint32_t>(frame->data[5]) << 8) |
          (static_cast<uint32_t>(frame->data[6]) << 16) |
          (static_cast<uint32_t>(frame->data[7]) << 24);
        throw std::runtime_error(
          "SDO read abort node=" + std::to_string(node_id) +
          " index=" + hex_u32(index) +
          " sub=" + hex_u32(subindex) +
          " code=" + hex_u32(abort_code));
      }
      std::array<uint8_t, 8> resp{};
      std::copy(std::begin(frame->data), std::end(frame->data), resp.begin());
      return resp;
    }
    throw std::runtime_error(
      "timeout waiting for SDO read response, node=" + std::to_string(node_id) +
      " index=" + hex_u32(index));
  }

  void expect_sdo_write_ack(
    int node_id, uint16_t index, uint8_t subindex, std::chrono::milliseconds timeout)
  {
    const uint32_t resp_id = 0x580 + static_cast<uint32_t>(node_id);
    const auto deadline = std::chrono::steady_clock::now() + timeout;
    while (std::chrono::steady_clock::now() < deadline) {
      auto frame = can_->recv_frame(20ms);
      if (!frame.has_value()) {
        continue;
      }
      if ((frame->can_id & CAN_EFF_MASK) != resp_id) {
        continue;
      }
      if (frame->data[1] != static_cast<uint8_t>(index & 0xFF) ||
          frame->data[2] != static_cast<uint8_t>((index >> 8) & 0xFF) ||
          frame->data[3] != subindex) {
        continue;
      }
      if (frame->data[0] == 0x60) {
        return;
      }
      if (frame->data[0] == 0x80) {
        const uint32_t abort_code = static_cast<uint32_t>(frame->data[4]) |
          (static_cast<uint32_t>(frame->data[5]) << 8) |
          (static_cast<uint32_t>(frame->data[6]) << 16) |
          (static_cast<uint32_t>(frame->data[7]) << 24);
        throw std::runtime_error(
          "SDO write abort node=" + std::to_string(node_id) +
          " index=" + hex_u32(index) +
          " sub=" + hex_u32(subindex) +
          " code=" + hex_u32(abort_code));
      }
    }
    throw std::runtime_error(
      "timeout waiting for SDO write ACK, node=" + std::to_string(node_id) +
      " index=" + hex_u32(index));
  }

private:
  std::string can_interface_;
  int right_controller_node_id_{1};
  int left_controller_node_id_{2};
  double control_hz_{50.0};
  double state_hz_{10.0};
  double command_timeout_sec_{0.3};
  int accel_ms_{100};
  int decel_ms_{100};
  bool sync_control_{true};
  bool use_pdo_commands_{true};
  bool configure_pdo_mapping_{true};
  int sdo_timeout_ms_{200};
  int state_sdo_timeout_ms_{50};
  double right_wheel_sign_{-1.0};
  double left_wheel_sign_{1.0};
  bool emergency_stop_shutdown_enabled_{true};
  int state_read_failure_shutdown_consecutive_cycles_{5};

  std::unique_ptr<SocketCanInterface> can_;
  std::mutex can_mutex_;

  rclcpp::Publisher<FourWheelState>::SharedPtr state_pub_;
  rclcpp::Subscription<std_msgs::msg::Float32MultiArray>::SharedPtr cmd_sub_;
  rclcpp::Service<SetFourWheelSpeeds>::SharedPtr set_speed_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr enable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr disable_srv_;
  rclcpp::Service<std_srvs::srv::Trigger>::SharedPtr clear_fault_srv_;
  rclcpp::TimerBase::SharedPtr control_timer_;
  rclcpp::TimerBase::SharedPtr state_timer_;

  std::mutex cmd_mutex_;
  bool drivers_enabled_{false};
  bool emergency_shutdown_requested_{false};
  int state_read_failure_count_{0};
  int16_t target_right_front_rpm_{0};
  int16_t target_right_rear_rpm_{0};
  int16_t target_left_front_rpm_{0};
  int16_t target_left_rear_rpm_{0};
  int16_t last_sent_right_front_rpm_{0};
  int16_t last_sent_right_rear_rpm_{0};
  int16_t last_sent_left_front_rpm_{0};
  int16_t last_sent_left_rear_rpm_{0};
  rclcpp::Time last_cmd_time_;
};

int main(int argc, char ** argv)
{
  rclcpp::init(argc, argv);
  int exit_code = 0;
  try {
    auto node = std::make_shared<FourWheelDriverNode>();
    rclcpp::spin(node);
    if (node->emergency_shutdown_requested()) {
      exit_code = 1;
    }
  } catch (const std::exception & e) {
    std::fprintf(stderr, "Fatal error: %s\n", e.what());
    exit_code = 1;
  }
  if (rclcpp::ok()) {
    rclcpp::shutdown();
  }
  return exit_code;
}

#ifndef STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_
#define STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_

#include <array>
#include <cmath>
#include <tuple>
#include <utility>
#include <vector>

#include <rclcpp/time.hpp>
#include "rcppmath/rolling_mean_accumulator.hpp"

namespace steering_odometry
{
const unsigned int BICYCLE_CONFIG = 0;
const unsigned int TRICYCLE_CONFIG = 1;
const unsigned int ACKERMANN_CONFIG = 2;
const unsigned int FOUR_WHEEL_STEERING_CONFIG = 3;  // 四轮四转配置

inline bool is_close_to_zero(double val) { return std::fabs(val) < 1e-6; }

/**
 * \brief The Odometry class handles odometry readings
 * (2D pose and velocity with related timestamp)
 */
class SteeringOdometry
{
public:
  /**
   * \brief Constructor
   * Timestamp will get the current time value
   * Value will be set to zero
   * \param velocity_rolling_window_size Rolling window size used to compute the velocity mean
   *
   */
  explicit SteeringOdometry(size_t velocity_rolling_window_size = 10);

  /**
   * \brief Initialize the odometry
   * \param time Current time
   */
  void init(const rclcpp::Time & time);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param traction_wheel_pos  traction wheel position [rad]
   * \param steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_position(
    const double traction_wheel_pos, const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_pos  Right traction wheel velocity [rad]
   * \param left_traction_wheel_pos  Left traction wheel velocity [rad]
   * \param steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_position(
    const double right_traction_wheel_pos, const double left_traction_wheel_pos,
    const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_pos  Right traction wheel position [rad]
   * \param left_traction_wheel_pos  Left traction wheel position [rad]
   * \param right_steer_pos Right steer wheel position [rad]
   * \param left_steer_pos Left steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_position(
    const double right_traction_wheel_pos, const double left_traction_wheel_pos,
    const double right_steer_pos, const double left_steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param traction_wheel_vel  Traction wheel velocity [rad/s]
   * \param steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_velocity(
    const double traction_wheel_vel, const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_vel  Right traction wheel velocity [rad/s]
   * \param left_traction_wheel_vel  Left traction wheel velocity [rad/s]
   * \param steer_pos Steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_velocity(
    const double right_traction_wheel_vel, const double left_traction_wheel_vel,
    const double steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with latest wheels position
   * \param right_traction_wheel_vel  Right traction wheel velocity [rad/s]
   * \param left_traction_wheel_vel  Left traction wheel velocity [rad/s]
   * \param right_steer_pos Right steer wheel position [rad]
   * \param left_steer_pos Left steer wheel position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_velocity(
    const double right_traction_wheel_vel, const double left_traction_wheel_vel,
    const double right_steer_pos, const double left_steer_pos, const double dt);

  /**
   * \brief Updates the odometry class with all four wheels (4WIS)
   * \param fl_wheel_vel  Front-left traction wheel velocity [rad/s]
   * \param fr_wheel_vel  Front-right traction wheel velocity [rad/s]
   * \param rl_wheel_vel  Rear-left traction wheel velocity [rad/s]
   * \param rr_wheel_vel  Rear-right traction wheel velocity [rad/s]
   * \param fl_steer_pos  Front-left steer position [rad]
   * \param fr_steer_pos  Front-right steer position [rad]
   * \param rl_steer_pos  Rear-left steer position [rad]
   * \param rr_steer_pos  Rear-right steer position [rad]
   * \param dt      time difference to last call
   * \return true if the odometry is actually updated
   */
  bool update_from_velocity_4wis(
    const double fl_wheel_vel, const double fr_wheel_vel,
    const double rl_wheel_vel, const double rr_wheel_vel,
    const double fl_steer_pos, const double fr_steer_pos,
    const double rl_steer_pos, const double rr_steer_pos,
    const double dt);

  /**
   * \brief Updates the odometry class with latest velocity command
   * \param v_bx  Linear velocity   [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt      time difference to last call
   */
  void update_open_loop(const double v_bx, const double omega_bz, const double dt);

  /**
   * \brief Set odometry type
   * \param type odometry type
   */
  void set_odometry_type(const unsigned int type);

  /**
   * \brief heading getter
   * \return heading [rad]
   */
  double get_heading() const { return heading_; }

  /**
   * \brief x position getter
   * \return x position [m]
   */
  double get_x() const { return x_; }

  /**
   * \brief y position getter
   * \return y position [m]
   */
  double get_y() const { return y_; }

  /**
   * \brief linear velocity getter
   * \return linear velocity [m/s]
   */
  double get_linear() const { return linear_; }

  /**
   * \brief lateral velocity getter
   * \return lateral velocity [m/s]
   */
  double get_lateral() const { return lateral_; }

  /**
   * \brief angular velocity getter
   * \return angular velocity [rad/s]
   */
  double get_angular() const { return angular_; }

  /**
   * \brief Sets the wheel parameters: radius, separation and wheelbase
   */
  void set_wheel_params(
    const double wheel_radius, const double wheelbase = 0.0, const double wheel_track = 0.0);

  /**
   * \brief Sets command limits used by native 4WIS inverse kinematics.
   * \param max_wheel_speed Maximum absolute wheel angular speed [rad/s]. <= 0 disables limiting.
   * \param max_steering_angle Maximum absolute normalized steering angle [rad]. <= 0 disables limiting.
   */
  void set_command_limits(const double max_wheel_speed, const double max_steering_angle);

  /**
   * \brief Velocity rolling window size setter
   * \param velocity_rolling_window_size Velocity rolling window size
   */
  void set_velocity_rolling_window_size(const size_t velocity_rolling_window_size);

  /**
   * \brief Calculates inverse kinematics for the desired linear and angular velocities
   * \param v_bx     Desired linear velocity of the robot in x_b-axis direction
   * \param omega_bz Desired angular velocity of the robot around x_z-axis
   * \param open_loop If false, the IK will be calculated using measured steering angle
   * \param reduce_wheel_speed_until_steering_reached Reduce wheel speed until the steering angle
   * has been reached
   * \return Tuple of velocity commands and steering commands
   */
  std::tuple<std::vector<double>, std::vector<double>> get_commands(
    const double v_bx, const double omega_bz, const bool open_loop = true,
    const bool reduce_wheel_speed_until_steering_reached = false);

  /**
   * \brief Calculates inverse kinematics for omnidirectional motion (4WIS only)
   * \param v_bx     Desired linear velocity of the robot in x_b-axis direction (forward/backward)
   * \param v_by     Desired linear velocity of the robot in y_b-axis direction (left/right)
   * \param omega_bz Desired angular velocity of the robot around z-axis
   * \return Tuple of velocity commands and steering commands
   */
  std::tuple<std::vector<double>, std::vector<double>> get_commands_omnidirectional(
    const double v_bx, const double v_by, const double omega_bz);

  /**
   *  \brief Reset poses, heading, and accumulators
   */
  void reset_odometry();

private:
  static constexpr size_t kWheelCount = 4;

  struct NormalizedSteering
  {
    double angle;
    double speed_multiplier;
  };

  static double normalize_angle(const double angle);
  static NormalizedSteering normalize_steering_angle(const double angle);

  std::array<std::pair<double, double>, kWheelCount> wheel_positions() const;

  void apply_wheel_speed_limit(std::vector<double> & wheel_speeds) const;

  /**
   * \brief Uses precomputed linear and angular velocities to compute odometry
   * \param v_bx  Linear  velocity   [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt      time difference to last call
   */
  bool update_odometry(const double v_bx, const double omega_bz, const double dt);

  /**
   * \brief Uses precomputed body-frame twist to compute odometry
   * \param v_bx Longitudinal velocity in base frame [m/s]
   * \param v_by Lateral velocity in base frame [m/s]
   * \param omega_bz Angular velocity around z [rad/s]
   * \param dt time difference to last call
   */
  bool update_odometry(
    const double v_bx, const double v_by, const double omega_bz, const double dt);

  /**
   * \brief Integrates the velocities (linear and angular) using 2nd order Runge-Kutta
   * \param v_bx Linear velocity [m/s]
   * \param v_by Lateral velocity [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt time difference to last call
   */
  void integrate_runge_kutta_2(
    const double v_bx, const double v_by, const double omega_bz, const double dt);

  /**
   * \brief Integrates the velocities (linear and angular)
   * \param v_bx Linear velocity [m/s]
   * \param v_by Lateral velocity [m/s]
   * \param omega_bz Angular velocity [rad/s]
   * \param dt time difference to last call
   */
  void integrate_fk(const double v_bx, const double v_by, const double omega_bz, const double dt);

  /**
   * \brief Calculates steering angle from the desired twist
   * \param v_bx     Linear velocity of the robot in x_b-axis direction
   * \param omega_bz Angular velocity of the robot around x_z-axis
   */
  double convert_twist_to_steering_angle(const double v_bx, const double omega_bz);

  /**
   * \brief Calculates linear velocity of a robot with double traction axle
   * \param right_traction_wheel_vel  Right traction wheel velocity [rad/s]
   * \param left_traction_wheel_vel  Left traction wheel velocity [rad/s]
   * \param steer_pos Steer wheel position [rad]
   */
  double get_linear_velocity_double_traction_axle(
    const double right_traction_wheel_vel, const double left_traction_wheel_vel,
    const double steer_pos);

  /**
   *  \brief Reset linear and angular accumulators
   */
  void reset_accumulators();

  /// Current timestamp:
  rclcpp::Time timestamp_;

  /// Current pose:
  double x_;          //   [m]
  double y_;          //   [m]
  double steer_pos_;  // [rad]
  double heading_;    // [rad]

  /// Current velocity:
  double linear_;   //   [m/s]
  double lateral_;  //   [m/s]
  double angular_;  // [rad/s]

  /// Kinematic parameters
  double wheel_track_;   // [m]
  double wheelbase_;     // [m]
  double wheel_radius_;  // [m]
  double max_wheel_speed_;     // [rad/s]
  double max_steering_angle_;  // [rad]

  /// Configuration type used for the forward kinematics
  int config_type_ = -1;

  /// Previous wheel position/state [rad]:
  double traction_wheel_old_pos_;
  double traction_right_wheel_old_pos_;
  double traction_left_wheel_old_pos_;
  /// Rolling mean accumulators for the linear and angular velocities:
  size_t velocity_rolling_window_size_;
  rcppmath::RollingMeanAccumulator<double> linear_acc_;
  rcppmath::RollingMeanAccumulator<double> lateral_acc_;
  rcppmath::RollingMeanAccumulator<double> angular_acc_;
};
}  // namespace steering_odometry

#endif  // STEERING_CONTROLLERS_LIBRARY__STEERING_ODOMETRY_HPP_

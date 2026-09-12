#include "steering_controllers_library/steering_odometry.hpp"

#include <algorithm>
#include <cmath>
#include <limits>
#include <stdexcept>

namespace steering_odometry
{
namespace
{
constexpr double kDefaultMaxWheelSpeed = 10.0;  // rad/s
constexpr double kDefaultMaxSteeringAngle = 75.0 * M_PI / 180.0;
constexpr double kMinDt = 0.0001;
constexpr double kEpsilon = 1e-9;
}  // namespace

SteeringOdometry::SteeringOdometry(size_t velocity_rolling_window_size)
: timestamp_(0.0),
  x_(0.0),
  y_(0.0),
  steer_pos_(0.0),
  heading_(0.0),
  linear_(0.0),
  lateral_(0.0),
  angular_(0.0),
  wheel_track_(0.0),
  wheelbase_(0.0),
  wheel_radius_(0.0),
  max_wheel_speed_(kDefaultMaxWheelSpeed),
  max_steering_angle_(kDefaultMaxSteeringAngle),
  traction_wheel_old_pos_(0.0),
  traction_right_wheel_old_pos_(0.0),
  traction_left_wheel_old_pos_(0.0),
  velocity_rolling_window_size_(velocity_rolling_window_size),
  linear_acc_(velocity_rolling_window_size),
  lateral_acc_(velocity_rolling_window_size),
  angular_acc_(velocity_rolling_window_size)
{
}

void SteeringOdometry::init(const rclcpp::Time & time)
{
  reset_accumulators();
  timestamp_ = time;
}

double SteeringOdometry::normalize_angle(const double angle)
{
  double normalized = std::fmod(angle + M_PI, 2.0 * M_PI);
  if (normalized < 0.0)
  {
    normalized += 2.0 * M_PI;
  }
  return normalized - M_PI;
}

SteeringOdometry::NormalizedSteering SteeringOdometry::normalize_steering_angle(
  const double angle)
{
  double normalized = normalize_angle(angle);
  double speed_multiplier = 1.0;

  if (normalized > M_PI_2)
  {
    normalized -= M_PI;
    speed_multiplier = -1.0;
  }
  else if (normalized < -M_PI_2)
  {
    normalized += M_PI;
    speed_multiplier = -1.0;
  }

  return {normalized, speed_multiplier};
}

std::array<std::pair<double, double>, SteeringOdometry::kWheelCount>
SteeringOdometry::wheel_positions() const
{
  const double half_wheelbase = wheelbase_ * 0.5;
  const double half_track = wheel_track_ * 0.5;
  return {{
    {half_wheelbase, half_track},    // FL
    {half_wheelbase, -half_track},   // FR
    {-half_wheelbase, half_track},   // RL
    {-half_wheelbase, -half_track},  // RR
  }};
}

void SteeringOdometry::apply_wheel_speed_limit(std::vector<double> & wheel_speeds) const
{
  if (max_wheel_speed_ <= 0.0)
  {
    return;
  }

  double max_abs_speed = 0.0;
  for (const double speed : wheel_speeds)
  {
    max_abs_speed = std::max(max_abs_speed, std::abs(speed));
  }

  if (max_abs_speed <= max_wheel_speed_)
  {
    return;
  }

  const double scale = max_wheel_speed_ / max_abs_speed;
  for (double & speed : wheel_speeds)
  {
    speed *= scale;
  }
}

bool SteeringOdometry::update_odometry(
  const double linear_velocity, const double angular_velocity, const double dt)
{
  return update_odometry(linear_velocity, 0.0, angular_velocity, dt);
}

bool SteeringOdometry::update_odometry(
  const double linear_velocity, const double lateral_velocity, const double angular_velocity,
  const double dt)
{
  if (dt < kMinDt)
  {
    return false;
  }

  integrate_fk(linear_velocity, lateral_velocity, angular_velocity, dt);

  linear_acc_.accumulate(linear_velocity);
  lateral_acc_.accumulate(lateral_velocity);
  angular_acc_.accumulate(angular_velocity);

  linear_ = linear_acc_.getRollingMean();
  lateral_ = lateral_acc_.getRollingMean();
  angular_ = angular_acc_.getRollingMean();

  return true;
}

bool SteeringOdometry::update_from_position(
  const double traction_wheel_pos, const double steer_pos, const double dt)
{
  const double traction_wheel_est_pos_diff = traction_wheel_pos - traction_wheel_old_pos_;
  traction_wheel_old_pos_ = traction_wheel_pos;

  return update_from_velocity(traction_wheel_est_pos_diff / dt, steer_pos, dt);
}

bool SteeringOdometry::update_from_position(
  const double traction_right_wheel_pos, const double traction_left_wheel_pos,
  const double steer_pos, const double dt)
{
  const double traction_right_wheel_est_pos_diff =
    traction_right_wheel_pos - traction_right_wheel_old_pos_;
  const double traction_left_wheel_est_pos_diff =
    traction_left_wheel_pos - traction_left_wheel_old_pos_;

  traction_right_wheel_old_pos_ = traction_right_wheel_pos;
  traction_left_wheel_old_pos_ = traction_left_wheel_pos;

  return update_from_velocity(
    traction_right_wheel_est_pos_diff / dt, traction_left_wheel_est_pos_diff / dt, steer_pos, dt);
}

bool SteeringOdometry::update_from_position(
  const double traction_right_wheel_pos, const double traction_left_wheel_pos,
  const double right_steer_pos, const double left_steer_pos, const double dt)
{
  const double traction_right_wheel_est_pos_diff =
    traction_right_wheel_pos - traction_right_wheel_old_pos_;
  const double traction_left_wheel_est_pos_diff =
    traction_left_wheel_pos - traction_left_wheel_old_pos_;

  traction_right_wheel_old_pos_ = traction_right_wheel_pos;
  traction_left_wheel_old_pos_ = traction_left_wheel_pos;

  return update_from_velocity(
    traction_right_wheel_est_pos_diff / dt, traction_left_wheel_est_pos_diff / dt, right_steer_pos,
    left_steer_pos, dt);
}

bool SteeringOdometry::update_from_velocity(
  const double traction_wheel_vel, const double steer_pos, const double dt)
{
  steer_pos_ = steer_pos;
  const double linear_velocity = traction_wheel_vel * wheel_radius_;
  const double angular_velocity = std::tan(steer_pos) * linear_velocity / wheelbase_;

  return update_odometry(linear_velocity, angular_velocity, dt);
}

double SteeringOdometry::get_linear_velocity_double_traction_axle(
  const double right_traction_wheel_vel, const double left_traction_wheel_vel,
  const double steer_pos)
{
  const double turning_radius = wheelbase_ / std::tan(steer_pos);
  const double vel_wheel_r = right_traction_wheel_vel * wheel_radius_;
  const double vel_wheel_l = left_traction_wheel_vel * wheel_radius_;

  if (std::isinf(turning_radius))
  {
    return (vel_wheel_r + vel_wheel_l) * 0.5;
  }

  const double vel_r = vel_wheel_r * turning_radius / (turning_radius + wheel_track_ * 0.5);
  const double vel_l = vel_wheel_l * turning_radius / (turning_radius - wheel_track_ * 0.5);
  return (vel_r + vel_l) * 0.5;
}

bool SteeringOdometry::update_from_velocity(
  const double right_traction_wheel_vel, const double left_traction_wheel_vel,
  const double steer_pos, const double dt)
{
  steer_pos_ = steer_pos;
  const double linear_velocity = get_linear_velocity_double_traction_axle(
    right_traction_wheel_vel, left_traction_wheel_vel, steer_pos_);
  const double angular_velocity = std::tan(steer_pos_) * linear_velocity / wheelbase_;

  return update_odometry(linear_velocity, angular_velocity, dt);
}

bool SteeringOdometry::update_from_velocity(
  const double right_traction_wheel_vel, const double left_traction_wheel_vel,
  const double right_steer_pos, const double left_steer_pos, const double dt)
{
  const double right_steer_pos_est = std::atan(
    wheelbase_ * std::tan(right_steer_pos) /
    (wheelbase_ - wheel_track_ * 0.5 * std::tan(right_steer_pos)));
  const double left_steer_pos_est = std::atan(
    wheelbase_ * std::tan(left_steer_pos) /
    (wheelbase_ + wheel_track_ * 0.5 * std::tan(left_steer_pos)));
  steer_pos_ = (right_steer_pos_est + left_steer_pos_est) * 0.5;

  const double linear_velocity = get_linear_velocity_double_traction_axle(
    right_traction_wheel_vel, left_traction_wheel_vel, steer_pos_);
  const double angular_velocity = std::tan(steer_pos_) * linear_velocity / wheelbase_;

  return update_odometry(linear_velocity, angular_velocity, dt);
}

bool SteeringOdometry::update_from_velocity_4wis(
  const double fl_wheel_vel, const double fr_wheel_vel,
  const double rl_wheel_vel, const double rr_wheel_vel,
  const double fl_steer_pos, const double fr_steer_pos,
  const double rl_steer_pos, const double rr_steer_pos,
  const double dt)
{
  const std::array<double, kWheelCount> wheel_velocities = {
    fl_wheel_vel, fr_wheel_vel, rl_wheel_vel, rr_wheel_vel};
  const std::array<double, kWheelCount> steering_positions = {
    fl_steer_pos, fr_steer_pos, rl_steer_pos, rr_steer_pos};
  const auto positions = wheel_positions();

  double sum_vx = 0.0;
  double sum_vy = 0.0;
  double sum_omega = 0.0;
  size_t omega_count = 0;

  for (size_t i = 0; i < kWheelCount; ++i)
  {
    const double wheel_linear_speed = wheel_velocities[i] * wheel_radius_;
    const double vx = wheel_linear_speed * std::cos(steering_positions[i]);
    const double vy = wheel_linear_speed * std::sin(steering_positions[i]);
    const auto [x, y] = positions[i];
    const double radius_sq = x * x + y * y;

    sum_vx += vx;
    sum_vy += vy;

    if (radius_sq > kEpsilon)
    {
      sum_omega += (-y * vx + x * vy) / radius_sq;
      ++omega_count;
    }
  }

  const double linear_velocity = sum_vx / static_cast<double>(kWheelCount);
  const double lateral_velocity = sum_vy / static_cast<double>(kWheelCount);
  const double angular_velocity =
    omega_count > 0 ? sum_omega / static_cast<double>(omega_count) : 0.0;

  steer_pos_ =
    (fl_steer_pos + fr_steer_pos + rl_steer_pos + rr_steer_pos) /
    static_cast<double>(kWheelCount);

  return update_odometry(linear_velocity, lateral_velocity, angular_velocity, dt);
}

void SteeringOdometry::update_open_loop(const double v_bx, const double omega_bz, const double dt)
{
  update_odometry(v_bx, 0.0, omega_bz, dt);
}

void SteeringOdometry::set_wheel_params(double wheel_radius, double wheelbase, double wheel_track)
{
  wheel_radius_ = wheel_radius;
  wheelbase_ = wheelbase;
  wheel_track_ = wheel_track;
}

void SteeringOdometry::set_command_limits(
  const double max_wheel_speed, const double max_steering_angle)
{
  max_wheel_speed_ = max_wheel_speed;
  max_steering_angle_ = max_steering_angle;
}

void SteeringOdometry::set_velocity_rolling_window_size(size_t velocity_rolling_window_size)
{
  velocity_rolling_window_size_ = velocity_rolling_window_size;
  reset_accumulators();
}

void SteeringOdometry::set_odometry_type(const unsigned int type)
{
  config_type_ = static_cast<int>(type);
}

double SteeringOdometry::convert_twist_to_steering_angle(double v_bx, double omega_bz)
{
  if (std::abs(v_bx) < kEpsilon)
  {
    if (std::abs(omega_bz) > kEpsilon)
    {
      return omega_bz > 0.0 ? M_PI_2 : -M_PI_2;
    }
    return 0.0;
  }

  const double phi = std::atan(omega_bz * wheelbase_ / v_bx);
  return std::isfinite(phi) ? phi : 0.0;
}

std::tuple<std::vector<double>, std::vector<double>> SteeringOdometry::get_commands(
  const double v_bx, const double omega_bz, const bool open_loop,
  const bool reduce_wheel_speed_until_steering_reached)
{
  if (config_type_ == FOUR_WHEEL_STEERING_CONFIG)
  {
    auto [wheel_speeds, steering_angles] = get_commands_omnidirectional(v_bx, 0.0, omega_bz);

    if (!open_loop && reduce_wheel_speed_until_steering_reached)
    {
      const double target_steer_pos =
        steering_angles.empty() ? 0.0 :
        (steering_angles[0] + steering_angles[1] + steering_angles[2] + steering_angles[3]) * 0.25;
      const double steer_error = std::abs(normalize_angle(target_steer_pos - steer_pos_));
      constexpr double kSteerReachThreshold = M_PI / 6.0;
      if (steer_error > kSteerReachThreshold)
      {
        std::fill(wheel_speeds.begin(), wheel_speeds.end(), 0.0);
      }
    }

    return std::make_tuple(wheel_speeds, steering_angles);
  }

  const double phi = convert_twist_to_steering_angle(v_bx, omega_bz);
  const double phi_for_speed = open_loop ? phi : steer_pos_;
  double wheel_speed = wheel_radius_ > kEpsilon ? v_bx / wheel_radius_ : 0.0;

  if (!open_loop && reduce_wheel_speed_until_steering_reached)
  {
    const double steer_error = std::abs(normalize_angle(steer_pos_ - phi));
    const double min_steer_error = M_PI / 6.0;
    if (steer_error >= min_steer_error)
    {
      const double scale = std::max(0.01, std::cos(std::min(steer_error, M_PI_2))) /
        std::cos(min_steer_error);
      wheel_speed *= scale;
    }
  }

  if (config_type_ == BICYCLE_CONFIG)
  {
    return std::make_tuple(std::vector<double>{wheel_speed}, std::vector<double>{phi});
  }

  if (config_type_ == TRICYCLE_CONFIG)
  {
    if (is_close_to_zero(phi_for_speed))
    {
      return std::make_tuple(
        std::vector<double>{wheel_speed, wheel_speed}, std::vector<double>{phi});
    }

    const double turning_radius = wheelbase_ / std::tan(phi_for_speed);
    return std::make_tuple(
      std::vector<double>{
        wheel_speed * (turning_radius + wheel_track_ * 0.5) / turning_radius,
        wheel_speed * (turning_radius - wheel_track_ * 0.5) / turning_radius},
      std::vector<double>{phi});
  }

  if (config_type_ == ACKERMANN_CONFIG)
  {
    if (is_close_to_zero(phi_for_speed))
    {
      return std::make_tuple(
        std::vector<double>{wheel_speed, wheel_speed}, std::vector<double>{phi, phi});
    }

    const double turning_radius = wheelbase_ / std::tan(phi_for_speed);
    const double numerator = 2.0 * wheelbase_ * std::sin(phi);
    const double denominator_first_member = 2.0 * wheelbase_ * std::cos(phi);
    const double denominator_second_member = wheel_track_ * std::sin(phi);

    return std::make_tuple(
      std::vector<double>{
        wheel_speed * (turning_radius + wheel_track_ * 0.5) / turning_radius,
        wheel_speed * (turning_radius - wheel_track_ * 0.5) / turning_radius},
      std::vector<double>{
        std::atan2(numerator, denominator_first_member + denominator_second_member),
        std::atan2(numerator, denominator_first_member - denominator_second_member)});
  }

  throw std::runtime_error("Config not implemented");
}

std::tuple<std::vector<double>, std::vector<double>> SteeringOdometry::get_commands_omnidirectional(
  const double v_bx, const double v_by, const double omega_bz)
{
  if (!std::isfinite(v_bx) || !std::isfinite(v_by) || !std::isfinite(omega_bz))
  {
    throw std::runtime_error("Invalid input to get_commands_omnidirectional");
  }
  if (config_type_ != FOUR_WHEEL_STEERING_CONFIG)
  {
    throw std::runtime_error("Omnidirectional motion requires FOUR_WHEEL_STEERING_CONFIG");
  }
  if (wheel_radius_ <= kEpsilon)
  {
    throw std::runtime_error("wheel_radius must be positive for 4WIS kinematics");
  }

  std::vector<double> wheel_speeds;
  std::vector<double> steering_angles;
  wheel_speeds.reserve(kWheelCount);
  steering_angles.reserve(kWheelCount);

  for (const auto & [x, y] : wheel_positions())
  {
    const double wheel_vx = v_bx - omega_bz * y;
    const double wheel_vy = v_by + omega_bz * x;
    const double wheel_speed_mps = std::hypot(wheel_vx, wheel_vy);

    if (wheel_speed_mps < kEpsilon)
    {
      wheel_speeds.push_back(0.0);
      steering_angles.push_back(0.0);
      continue;
    }

    const double raw_angle = std::atan2(wheel_vy, wheel_vx);
    const auto normalized = normalize_steering_angle(raw_angle);
    double steering_angle = normalized.angle;

    if (max_steering_angle_ > 0.0)
    {
      steering_angle = std::clamp(steering_angle, -max_steering_angle_, max_steering_angle_);
    }

    wheel_speeds.push_back(wheel_speed_mps / wheel_radius_ * normalized.speed_multiplier);
    steering_angles.push_back(steering_angle);
  }

  apply_wheel_speed_limit(wheel_speeds);
  return std::make_tuple(wheel_speeds, steering_angles);
}

void SteeringOdometry::reset_odometry()
{
  x_ = 0.0;
  y_ = 0.0;
  heading_ = 0.0;
  linear_ = 0.0;
  lateral_ = 0.0;
  angular_ = 0.0;
  reset_accumulators();
}

void SteeringOdometry::integrate_runge_kutta_2(
  const double v_bx, const double v_by, const double omega_bz, const double dt)
{
  const double theta_mid = heading_ + omega_bz * 0.5 * dt;
  x_ += (v_bx * std::cos(theta_mid) - v_by * std::sin(theta_mid)) * dt;
  y_ += (v_bx * std::sin(theta_mid) + v_by * std::cos(theta_mid)) * dt;
  heading_ = normalize_angle(heading_ + omega_bz * dt);
}

void SteeringOdometry::integrate_fk(
  const double v_bx, const double v_by, const double omega_bz, const double dt)
{
  integrate_runge_kutta_2(v_bx, v_by, omega_bz, dt);
}

void SteeringOdometry::reset_accumulators()
{
  linear_acc_ = rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
  lateral_acc_ = rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
  angular_acc_ = rcppmath::RollingMeanAccumulator<double>(velocity_rolling_window_size_);
}

}  // namespace steering_odometry

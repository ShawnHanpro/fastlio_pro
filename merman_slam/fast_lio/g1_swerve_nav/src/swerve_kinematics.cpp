#include "g1_swerve_nav/swerve_kinematics.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <stdexcept>

namespace g1_swerve_nav
{
namespace
{
constexpr double kPi = 3.1415926535897932384626433832795;
constexpr double kTwoPi = 2.0 * kPi;
constexpr double kEps = 1e-9;
}

SwerveKinematics::SwerveKinematics(
  const std::array<WheelPosition, kWheelCount> & positions,
  const KinematicLimits & limits)
: positions_(positions), limits_(limits)
{
  if (limits_.wheel_radius_m <= 0.0) {
    throw std::invalid_argument("wheel_radius_m must be > 0");
  }
  if (limits_.steering_min_rad >= limits_.steering_max_rad) {
    throw std::invalid_argument("steering_min_rad must be < steering_max_rad");
  }
  if (limits_.max_wheel_rpm <= 0.0) {
    throw std::invalid_argument("max_wheel_rpm must be > 0");
  }
  if (limits_.wheel_vector_deadband_mps < 0.0) {
    throw std::invalid_argument("wheel_vector_deadband_mps must be >= 0");
  }
  if (limits_.boundary_hold_exit_rad <= 0.0 ||
    limits_.boundary_hold_enter_rad <= limits_.boundary_hold_exit_rad ||
    limits_.boundary_hold_enter_rad > kPi / 2.0 + 1e-6)
  {
    throw std::invalid_argument("invalid steering boundary hysteresis");
  }
}

double SwerveKinematics::normalize_angle(double angle)
{
  while (angle > kPi) {
    angle -= kTwoPi;
  }
  while (angle < -kPi) {
    angle += kTwoPi;
  }
  return angle;
}

double SwerveKinematics::rpm_to_mps(double rpm, double wheel_radius_m)
{
  return rpm * kTwoPi / 60.0 * wheel_radius_m;
}

double SwerveKinematics::mps_to_rpm(double mps, double wheel_radius_m)
{
  return mps / wheel_radius_m * 60.0 / kTwoPi;
}

WheelState SwerveKinematics::solve_wheel(
  double wheel_vx,
  double wheel_vy,
  double /*current_steering_rad*/,
  double previous_target_steering_rad,
  double /*previous_target_rpm*/) const
{
  const double requested_speed = std::hypot(wheel_vx, wheel_vy);
  if (requested_speed < limits_.wheel_vector_deadband_mps) {
    return WheelState{
      std::clamp(previous_target_steering_rad,
        limits_.steering_min_rad, limits_.steering_max_rad),
      0.0};
  }

  // Exact no-slip rolling-axis solution for the requested rigid-body twist.
  // Because wheel drive can reverse, steering angle is equivalent modulo pi.
  // For a physical [-90,+90] steering interval there is always one exact axis
  // representation in the interval.
  double exact = normalize_angle(std::atan2(wheel_vy, wheel_vx));
  while (exact > limits_.steering_max_rad + kEps) {
    exact -= kPi;
  }
  while (exact < limits_.steering_min_rad - kEps) {
    exact += kPi;
  }
  exact = std::clamp(exact, limits_.steering_min_rad, limits_.steering_max_rad);

  double target = exact;

  // Narrow endpoint bridge ONLY around the +/-90 degree discontinuity.
  //
  // +89 deg and a raw +91 deg vector have exact representations +89 and -89
  // (with opposite wheel-speed sign). Mechanically commanding +89 -> -89 for
  // a 2 degree vector change is undesirable, so while BOTH the previous target
  // and the new exact target are close to opposite physical endpoints we hold
  // the previous endpoint and use a small projection error.
  //
  // IMPORTANT: once the exact target moves away from the opposite endpoint
  // (e.g. -60, -30, +20 deg), the bridge is released immediately. Keeping an
  // endpoint across a wide angular region makes the four wheel vectors cease to
  // represent one rigid-body vx/vy/wz and causes tyre scrub/drag.
  const bool previous_near_positive =
    previous_target_steering_rad >= limits_.boundary_hold_enter_rad;
  const bool previous_near_negative =
    previous_target_steering_rad <= -limits_.boundary_hold_enter_rad;

  if (previous_near_positive && exact <= -limits_.boundary_hold_enter_rad) {
    target = limits_.steering_max_rad;
  } else if (previous_near_negative && exact >= limits_.boundary_hold_enter_rad) {
    target = limits_.steering_min_rad;
  } else if (previous_target_steering_rad >= limits_.steering_max_rad - kEps &&
    exact <= -limits_.boundary_hold_exit_rad)
  {
    target = limits_.steering_max_rad;
  } else if (previous_target_steering_rad <= limits_.steering_min_rad + kEps &&
    exact >= limits_.boundary_hold_exit_rad)
  {
    target = limits_.steering_min_rad;
  }

  const double projected_mps =
    wheel_vx * std::cos(target) + wheel_vy * std::sin(target);

  return WheelState{
    target,
    mps_to_rpm(projected_mps, limits_.wheel_radius_m)};
}

std::array<WheelState, kWheelCount> SwerveKinematics::inverse(
  double vx,
  double vy,
  double wz,
  const std::array<double, kWheelCount> & current_steering_rad,
  const std::array<double, kWheelCount> & previous_target_steering_rad,
  const std::array<double, kWheelCount> & previous_target_rpm) const
{
  std::array<WheelState, kWheelCount> result{};
  double peak_rpm = 0.0;

  for (std::size_t i = 0; i < kWheelCount; ++i) {
    const double wheel_vx = vx - wz * positions_[i].y;
    const double wheel_vy = vy + wz * positions_[i].x;

    result[i] = solve_wheel(
      wheel_vx,
      wheel_vy,
      current_steering_rad[i],
      previous_target_steering_rad[i],
      previous_target_rpm[i]);
    peak_rpm = std::max(peak_rpm, std::abs(result[i].wheel_rpm));
  }

  if (peak_rpm > limits_.max_wheel_rpm) {
    const double scale = limits_.max_wheel_rpm / peak_rpm;
    for (auto & wheel : result) {
      wheel.wheel_rpm *= scale;
    }
  }

  return result;
}

}  // namespace g1_swerve_nav

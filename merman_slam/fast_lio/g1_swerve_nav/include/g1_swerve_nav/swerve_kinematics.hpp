#pragma once

#include <array>
#include <cstddef>

namespace g1_swerve_nav
{

constexpr std::size_t kWheelCount = 4;

enum class Wheel : std::size_t
{
  FL = 0,
  FR = 1,
  RL = 2,
  RR = 3
};

struct WheelPosition
{
  double x{0.0};
  double y{0.0};
};

struct WheelState
{
  double steering_rad{0.0};
  double wheel_rpm{0.0};
};

struct KinematicLimits
{
  double wheel_radius_m{0.085};
  double steering_min_rad{-1.5707963267948966};
  double steering_max_rad{1.5707963267948966};
  double max_wheel_rpm{100.0};
  double wheel_vector_deadband_mps{0.008};

  // Steering-axis branch hysteresis for a finite +/-90 deg mechanism.
  // The wheel axis is physically equivalent modulo pi if wheel speed is
  // reversed, but the steering motor itself cannot jump from +90 to -90.
  // When a target crosses that boundary, hold the current endpoint until the
  // requested exact axis has moved sufficiently far into the opposite branch.
  double boundary_hold_enter_rad{1.4311699866353502};  // 82 deg
  double boundary_hold_exit_rad{1.3089969389957472};   // 75 deg
};

class SwerveKinematics
{
public:
  SwerveKinematics(
    const std::array<WheelPosition, kWheelCount> & positions,
    const KinematicLimits & limits);

  std::array<WheelState, kWheelCount> inverse(
    double vx,
    double vy,
    double wz,
    const std::array<double, kWheelCount> & current_steering_rad,
    const std::array<double, kWheelCount> & previous_target_steering_rad,
    const std::array<double, kWheelCount> & previous_target_rpm) const;

  const KinematicLimits & limits() const {return limits_;}

  static double normalize_angle(double angle);
  static double rpm_to_mps(double rpm, double wheel_radius_m);
  static double mps_to_rpm(double mps, double wheel_radius_m);

private:
  WheelState solve_wheel(
    double wheel_vx,
    double wheel_vy,
    double current_steering_rad,
    double previous_target_steering_rad,
    double previous_target_rpm) const;

  std::array<WheelPosition, kWheelCount> positions_{};
  KinematicLimits limits_{};
};

}  // namespace g1_swerve_nav

#!/usr/bin/env bash
set -u

ok=0
bad=0

check_topic() {
  local topic="$1"
  local expected_type="$2"
  local actual_type

  actual_type="$(ros2 topic type "$topic" 2>/dev/null || true)"
  if [ "$actual_type" = "$expected_type" ]; then
    printf '[OK]   topic %s [%s]\n' "$topic" "$actual_type"
    ok=$((ok+1))
  elif [ -n "$actual_type" ]; then
    printf '[TYPE] topic %s expected [%s], got [%s]\n' \
      "$topic" "$expected_type" "$actual_type"
    bad=$((bad+1))
  else
    printf '[MISS] topic %s\n' "$topic"
    bad=$((bad+1))
  fi
}

check_topic /map nav_msgs/msg/OccupancyGrid
check_topic /Odometry_loc nav_msgs/msg/Odometry
check_topic /nav2_scan sensor_msgs/msg/LaserScan
check_topic /steer/joint_states sensor_msgs/msg/JointState
check_topic /wheel_control_can/state zlac8015d_four_wheel_driver_cpp/msg/FourWheelState
check_topic /cmd_vel_nav geometry_msgs/msg/Twist
check_topic /cmd_vel_behavior geometry_msgs/msg/Twist
check_topic /cmd_vel_selected geometry_msgs/msg/Twist
check_topic /cmd_vel_smoothed geometry_msgs/msg/Twist
check_topic /cmd_vel_safe geometry_msgs/msg/Twist
check_topic /cmd_vel_mux/mode std_msgs/msg/String
check_topic /cmd_vel geometry_msgs/msg/Twist

printf '\nExpected command paths with g1_swerve_nav and teleop_sbus running:\n'
printf '  NAVIGATION: /cmd_vel_nav + /cmd_vel_behavior -> /cmd_vel_selected\n'
printf '              -> /cmd_vel_smoothed -> /cmd_vel_safe -> mode mux -> /cmd_vel\n'
printf '  MANUAL:     /cmd_vel_joy -> mode mux -> /cmd_vel (bypasses collision_monitor)\n'
printf '  /steer/position_cmd\n'
printf '  /wheel_control_can/wheel_rpm_cmd\n'
printf '\nSummary: %d present, %d missing\n' "$ok" "$bad"

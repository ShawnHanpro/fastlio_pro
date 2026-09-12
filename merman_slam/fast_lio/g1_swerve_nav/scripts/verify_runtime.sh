#!/usr/bin/env bash
set -u

ok=0
bad=0

check_topic() {
  local topic="$1"
  if ros2 topic info "$topic" >/dev/null 2>&1; then
    printf '[OK]   topic %s\n' "$topic"
    ok=$((ok+1))
  else
    printf '[MISS] topic %s\n' "$topic"
    bad=$((bad+1))
  fi
}

for t in /map /odom /scan /steer/joint_states /wheel_control_can/state; do
  check_topic "$t"
done

printf '\nExpected command outputs after bringup:\n'
printf '  /cmd_vel_nav -> /cmd_vel_selected -> /cmd_vel_smoothed -> /cmd_vel_safe\n'
printf '  /steer/position_cmd\n'
printf '  /wheel_control_can/wheel_rpm_cmd\n'
printf '\nSummary: %d present, %d missing\n' "$ok" "$bad"

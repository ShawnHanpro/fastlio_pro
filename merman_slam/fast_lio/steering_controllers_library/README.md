# steering_controllers_library

Native four-wheel independent steering (4WIS) kinematics and odometry library.

## Contents

- `include/steering_controllers_library/steering_odometry.hpp`
  Public C++ API for 4WIS inverse kinematics, forward kinematics, steering normalization, command limits, and odometry getters.
- `src/steering_odometry.cpp`
  Implementation of the 4WIS math.

## Scope

This package is intentionally kept as a small algorithm library. ROS nodes, launch files, adapters, and hardware-specific topic conversions belong in higher-level packages.

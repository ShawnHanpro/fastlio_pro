# v9-fix2: command handoff and stalled-steering robustness

This package keeps the v9 exact kinematics and coordinated branch design, with
small runtime fixes based on the latest robot log:

- teleop freshness increased from 0.40 s to 0.80 s to reduce keyboard source dropouts;
- automatic steering recenter no longer treats every `NAV -> NONE` gap as goal completion;
  it requires 1.5 s of stable idle and is canceled if any source resumes;
- a new intentional motion command can cancel an in-progress recenter after the
  recenter PP target has actually been sent;
- fixed steering targets used by large-settle and coordinated-branch states are
  re-triggered only if steering makes less than 1 deg progress for 0.8 s;
- velocity-smoother X/Y deceleration is increased to 2.0 m/s^2 so keyboard
  forward-to-rotate transitions shed residual translation faster and reach the
  true rotate-in-place steering geometry sooner.

The swerve kinematics equations, wheel mappings, RotationShim configuration,
branch thresholds, and normal v9 tracking behavior are otherwise unchanged.

---

# v9: coordinated locked branch transition

This revision is based on v7, not v8. It removes the v8 per-wheel +/-90 deg endpoint hold because the v8 log showed that hold could create mixed steering targets that no longer represented one rigid-body chassis twist.

For an equivalent branch crossing (large directed steering travel but modest rolling-axis mismatch), v9 now:

- keeps the exact four-wheel rigid-body steering solution;
- latches one complete four-wheel steering snapshot instead of chasing MPPI targets at 50 Hz;
- sends the Taihu steering set-point once;
- projects wheel RPM onto the measured steering axes;
- caps branch-transition drive to 18% and 12 rpm;
- lets rolling-axis alignment smoothly reduce drive near the perpendicular part of the sweep;
- times out to zero drive after 5 s if the fixed steering target does not converge.

Nav2, BT, launch, collision monitor, teleop/arbiter, hardware mappings, and the v7 exact kinematics are unchanged.

---

# v6 exact rigid-body steering fix

This revision fixes tyre scrub observed in v5. The v5 wide +/-90-degree branch hold could command endpoint angles (for example rear wheels at -90 deg) even when the exact rigid-body solution was around +40 deg. That makes the four module velocity vectors inconsistent with one chassis Twist.

v6 uses the exact modulo-pi wheel direction for normal motion and keeps endpoint bridging only inside a narrow 75-90 degree neighborhood. Large unavoidable branch changes are handled by the existing zero-drive reconfiguration guard. Nav2, MPPI, BT, collision monitor and teleop are unchanged.

## v5: smooth steering branch hold

This revision targets the repeated `move -> stop -> steer -> move` behavior seen on
multi-segment Nav2 routes. The safety reconfiguration state is retained, but it is
no longer used for ordinary 30--60 degree tracking errors.

Key changes:

- keep a +/-90 deg endpoint branch until the exact equivalent axis returns within
  10 deg of the center region; projected wheel RPM decreases continuously while held
- trigger a fixed-target stop/reconfiguration only for persistent physical steering
  mismatch above 80 deg
- full drive up to 12 deg steering tracking error, then smooth cosine reduction to
  zero at 65 deg instead of stopping at 30 deg
- 1.0 s reconfiguration cooldown prevents adjacent wheel modules from causing a
  stop-turn-go cascade
- Nav2, MPPI, BT, collision monitor, cmd_vel arbiter, and keyboard behavior are
  unchanged from v4

# g1_swerve_nav

ROS 2 Humble package for a true 3-DOF planar four-wheel independent steering / four-wheel drive chassis.

## Design goal

The package does **not** inherit the old chassis state machine or old navigation behavior. The supplied historical files were used only to recover hardware facts: geometry, topic/message units, and wheel ordering.

The command chain is:

```text
Nav2 MPPI Omni (/cmd_vel_nav) ----\
                                  > cmd_vel_arbiter
Teleop (/cmd_vel or /cmd_vel_teleop) --------/        |
                                           v
                                  /cmd_vel_selected
                                           |
                                  velocity_smoother
                                           |
                                  /cmd_vel_smoothed
                                           |
                                  collision_monitor
                                           |
                                    /cmd_vel_safe
                                           |
                                  swerve_controller
                                      /          \
                     /steer/position_cmd     /wheel_control_can/wheel_rpm_cmd
```

Fast-LIO / localization remains external and must provide `/odom` and TF. The 2D map remains external and must provide `/map`.

## Kinematics

For a wheel at body-frame position `(x_i, y_i)` and requested chassis twist `(vx, vy, wz)`:

```text
wheel_vx = vx - wz * y_i
wheel_vy = vy + wz * x_i
```

The exact wheel vector is then converted to steering angle and wheel speed. Equivalent solutions `theta` and `theta + pi` with reversed wheel speed are searched inside the mechanical steering interval. Selection is based primarily on **measured steering travel**, with a small history/reversal hysteresis to prevent solution chatter.

All four RPMs are scaled together when a wheel reaches the RPM limit, preserving the requested chassis twist ratio.

Normal steering corrections remain continuous while the robot is moving. A safety reconfiguration state is reserved for persistent >80 deg physical steering mismatches only; this is intended for genuine direction changes, not routine path tracking. Steering feedback otherwise produces a smooth speed reduction rather than repeated stop-turn-go behavior.

A zero Twist holds the current steering target. Re-centering is explicit:

```bash
ros2 service call /swerve_controller/recenter_steering std_srvs/srv/Trigger {}
```

This separation is important because MPPI can briefly output zero while optimizing and Nav2 final pose convergence should not trigger an unrelated steering-centering action.

## Hardware facts recovered from supplied files

Current defaults:

- wheelbase: `0.50 m`
- track width: `0.40 m`
- wheel radius: `0.085 m`
- configured steering range: `-90 .. +90 deg` (**physically verify**)
- configured wheel limit: `100 RPM`
- Taihu command: `/steer/position_cmd`, `std_msgs/Float64MultiArray`, radians
- Taihu feedback: `/steer/joint_states`, `sensor_msgs/JointState`
- Taihu logical wheel mapping: `joint_1=RR, joint_2=RL, joint_3=FL, joint_4=FR`
- ZLAC command: `/wheel_control_can/wheel_rpm_cmd`, `std_msgs/Float32MultiArray`, order `[RF, RR, LF, LR]`, logical RPM
- ZLAC feedback: `/wheel_control_can/state`, `zlac8015d_four_wheel_driver_cpp/msg/FourWheelState`

The lower-level ZLAC driver already applies left/right electrical sign conventions. This package therefore outputs **logical wheel RPM**, not CAN-motor signed RPM.

## Two parameters still need physical confirmation

1. `steering_min_rad / steering_max_rad`: true omni needs a total reachable steering span of at least 180 degrees when wheel reversal is used. The old runtime YAML requested `+/-90 deg`, but the physical hard stops were not proven by the supplied files.
2. Nav2 footprint: the supplied Nav2 config used `0.8 x 0.5 m`. Measure the maximum collision outline of the actual robot and update both local/global costmaps and collision-monitor zones.

Everything else needed for the first version is already parameterized.

## Nav2 profile

Default stable planner/controller:

- Global planner: `nav2_smac_planner/SmacPlanner2D`
- Local controller: `nav2_mppi_controller::MPPIController`
- Motion model: `Omni`
- Velocity smoothing: enabled on X/Y/Yaw
- Collision monitor: full-body stop + slowdown zones around the entire robot
- Goal checker: `0.04 m` XY and `0.035 rad` yaw initial target

There is intentionally no `PreferForwardCritic` and no rotate-to-goal controller. `GoalAngleCritic` becomes active while the chassis is still approaching so MPPI can use `vx + vy + wz` simultaneously.

## External prerequisites

Before Nav2 starts, these must exist:

```text
/map
/odom
/scan
TF map -> odom
TF odom -> base_link
/steer/joint_states
/wheel_control_can/state
```

Use:

```bash
ros2 run g1_swerve_nav verify_runtime.sh
```

## Build

Copy `g1_swerve_nav` into your ROS 2 Humble workspace `src/` directory, then:

```bash
source /opt/ros/humble/setup.bash
cd <workspace>
colcon build --packages-select g1_swerve_nav --symlink-install
source install/setup.bash
```

Install the Nav2 components if needed using the packages provided by your Humble installation. `nav2_mppi_controller` and `nav2_collision_monitor` must be installed.

## Start

Start the existing steering/drive hardware drivers, Fast-LIO/localization, and `/map` provider first. Then:

```bash
ros2 launch g1_swerve_nav bringup.launch.py
```

For navigation only:

```bash
ros2 launch g1_swerve_nav nav2.launch.py
```

For chassis adapter only:

```bash
ros2 launch g1_swerve_nav controller.launch.py
```

## Keyboard input

The standard ROS `teleop_twist_keyboard` may publish to `/cmd_vel` directly. `/cmd_vel_teleop` remains a compatible alias.

The arbiter gives a fresh teleop command priority over Nav2 for 0.4 s. Nav2 automatically resumes when teleop stops publishing. Do not publish keyboard Twist directly to `/cmd_vel_safe` because that bypasses the velocity smoother and collision monitor.

## Initial low-speed validation sequence

Raise the chassis off the ground or use an open secured area for the first test. Keep the default low limits. Verify in this order:

1. `vx > 0, vy=0, wz=0`: all wheel angles near 0 and logical wheel motion forward.
2. `vx=0, vy>0, wz=0`: all modules reach approximately +90/-90 equivalent solutions and the robot moves laterally.
3. `vx=0, vy=0, wz>0`: four wheel vectors are tangent to circles around `base_link` and body yaw changes in place.
4. Combined `vx + vy`: diagonal translation.
5. Combined `vx + vy + wz`: translation while body orientation changes.
6. Nav2 goal with a deliberately different final yaw: verify XY and yaw converge together.

If test 2 cannot physically reach its requested steering angles, do not tune Nav2 around it: first correct the mechanical steering limits because exact omni motion is then impossible.

## Why wheel odometry is not published here

This package deliberately does not create a second `odom -> base_link` source. Your localization stack already owns that transform. Duplicate odometry/TF publishers are a common cause of jumps and final-pose instability.

## Later State Lattice upgrade

The default `SmacPlanner2D + MPPI Omni` is selected for first-pass robustness. Once the chassis is validated, `SmacPlannerLattice` can be added if you want the **global** planner itself to reason about SE(2) omnidirectional motion primitives. It is not required for moving laterally or rotating while translating; those capabilities already come from MPPI Omni + the swerve controller.


## v2 runtime fixes

- Plain `ros2 run teleop_twist_keyboard teleop_twist_keyboard` now works on `/cmd_vel`.
- `/cmd_vel_teleop` remains accepted as an alias.
- Nav2 recovery velocity is isolated on `/cmd_vel_behavior`.
- Arbiter priority is teleop > behavior > navigation.
- Progress checker allowance is increased for steering-vector transitions.
- MPPI batch size is reduced to improve 20 Hz deadline margin.
- TwirlingCritic is removed so combined translation + yaw is not unnecessarily penalized.
- Global replanning is reduced to 0.5 Hz to reduce path churn.

## v4 steering reconfiguration fix

This revision changes only the low-level swerve steering behavior. Nav2 planner,
MPPI configuration, cmd_vel arbitration, and the standard ROS
`teleop_twist_keyboard` key mapping are unchanged from v3.

The fix addresses large wheel-angle changes observed when starting a new
navigation segment:

- +/-90 degree branch hysteresis remains enabled.
- Normal small/medium steering corrections continue while driving.
- If any wheel needs a persistent large reorientation (>35 deg by default),
  drive RPM is commanded directly to zero.
- The controller waits for measured wheel RPM to stop before changing the
  steering axes by a large amount.
- After a 120 ms debounce, one stable four-wheel steering target is latched.
- The latched target is sent once to the Taihu Profile Position driver instead
  of retriggering a moving PP target at the 50 Hz chassis loop rate.
- Motion resumes when all measured steering angles are within 8 deg of the
  latched targets.

The relevant parameters are in `config/kinematics.yaml`:

- `large_reconfig_enter_rad`
- `large_reconfig_exit_rad`
- `large_reconfig_debounce_sec`
- `large_reconfig_cmd_speed_threshold`
- `drive_full_steering_error_rad`
- `drive_stop_steering_error_rad`

## v6.1 startup fix

This revision keeps the v6 exact swerve kinematics unchanged. It only delays
`lifecycle_manager_navigation` by 5 seconds so all Nav2 lifecycle services are
ready before automatic configure/activate. This addresses startup runs where
`/controller_server/change_state` times out, which otherwise leaves the
velocity smoother and collision monitor inactive and prevents `/cmd_vel` from
reaching `/cmd_vel_safe`.

## v7 rolling-axis transition fix

This revision keeps the v6.1 Nav2 startup fix and the v6 exact rigid-body steering targets.

The important controller change is that **steering-motor travel** and **tyre rolling-axis
misalignment** are no longer treated as the same quantity.

For a reversible drive wheel:

- steering actuator `+80 deg -> -80 deg` is about `160 deg` of mechanical motor travel;
- but the undirected tyre rolling axes differ by only `20 deg`;
- therefore this branch-equivalent transition must not automatically stop the whole robot.

During a branch transition the controller now:

1. continues commanding the exact steering target;
2. calculates wheel RPM by projecting the requested rigid-body wheel velocity onto the
   **measured current steering axis**;
3. lets wheel RPM pass through zero / reverse sign naturally while the steering motor moves;
4. caps chassis drive during the transition instead of entering stop-turn-go;
5. reserves the old latched stop/reorientation guard for true near-perpendicular rolling-axis
   errors only.

The supplied `g1_swerve_nav(6).log` contained 11 old large-reconfiguration events. Re-evaluating
the logged target/feedback pairs modulo 180 degrees shows 10 were branch-equivalent false
positives; only one had a true rolling-axis error of about 85 degrees.

### New tuning parameters

```yaml
branch_transition_physical_error_rad: 1.6580627894  # 95 deg
branch_transition_axis_error_rad: 1.0471975512      # 60 deg
branch_transition_complete_error_rad: 0.2617993878  # 15 deg
branch_transition_chassis_scale: 0.40

drive_full_steering_error_rad: 0.2094395102          # 12 deg
drive_stop_steering_error_rad: 1.3089969390          # 75 deg rolling-axis
large_reconfig_enter_rad: 1.3962634016               # 80 deg rolling-axis
```

A true near-90-degree rolling-axis mismatch can still require a short reorientation because a
finite `[-90,+90]` steering mechanism cannot continuously realize every possible requested wheel
axis without passing through a singular configuration. The goal of v7 is to make that rare rather
than treating every equivalent branch change as a stop event.

# micro_lqr_controller — standard discrete LQR replacement

This package replaces the former hand-entered four-gain controller with a complete model-based digital LQR pipeline:

1. construct the continuous linearized inverted-pendulum/cart model;
2. use exact zero-order-hold discretization at `control_period_s`;
3. verify controllability;
4. solve the discrete algebraic Riccati equation (DARE);
5. calculate `K` at node startup;
6. execute `u_total = -K * state_error` at 333.333 Hz;
7. split total axle torque equally between the two wheel motors.

The state order follows the supplied course code:

```text
[pitch, pitch_rate, x, x_dot]
```

The model input is explicitly defined as:

```text
u_total = left-wheel forward torque + right-wheel forward torque
```

Therefore each wheel receives `0.5*u_total` before applying mirrored motor signs.

## Important model assumption

The default YAML uses values previously stated for the robot:

- total mass about 2.8 kg;
- two wheel assemblies about 0.13 kg each;
- wheel radius 0.03 m;
- axle-to-COM distance about 0.12 m.

`model.body_pitch_inertia_kg_m2` is currently the course-style approximation `I=m*h^2`. Replace it with the Inventor/CAD inertia about the **body COM pitch axis** before final tuning.

## Build

Install Eigen if required:

```bash
sudo apt update
sudo apt install libeigen3-dev
```

Build this overlay after the workspace that provides `custom_msgs`:

```bash
source /opt/ros/humble/setup.bash
source /home/uonaim/micro/install/setup.bash

cd /home/uonaim/micro_lqr_standard_ws
rm -rf build install log
colcon build --symlink-install --packages-select micro_lqr_controller
source install/setup.bash
```

Launch:

```bash
ros2 launch micro_lqr_controller micro_lqr.launch.py
```

With the default model and `R=10`, startup should print a gain close to:

```text
K=[-3.08055268 -0.51371579 -0.29873677 -0.47617184]
```

Small differences from Eigen iteration tolerances are normal.

## First hardware run

Keep the robot supported and keep:

```yaml
dry_run: true
torque_limit: 0.05
```

Start EtherCAT first, then start this node. Use the right RC switch:

```text
up (1)     calibrate current upright IMU and wheel zero
middle (3) arm
bottom (2) disable
```

### State sign test

Observe:

```bash
ros2 topic echo /micro_lqr/debug
```

The critical fields are:

```text
0  x
1  x_dot used by LQR
2  pitch
3  pitch_rate
24 left raw position
25 right raw position
26 left raw velocity
27 right raw velocity
28 x_dot from position finite difference
35 x_dot from motor velocity
36 motor-based minus finite-difference velocity
```

Verify all of these before enabling motors:

1. Push the complete upright robot in the defined positive-forward direction: `x` and both velocity estimates must become positive.
2. Tip the body in the defined positive pitch direction: `pitch` must become positive.
3. Move it in the same rotational direction: `pitch_rate` must become positive.
4. Hold wheel contact approximately fixed and rotate only the body: compensated `x` and `x_dot` should stay near zero.

Correct measurement signs with:

```yaml
left_encoder_sign
right_encoder_sign
imu_angle_sign
imu_rate_sign
pitch_position_compensation_sign
pitch_rate_compensation_sign
```

### Actuator sign test

The model defines positive `u_total` as the torque direction that should increase positive `x_dot` near upright. If hardware produces the opposite acceleration, flip only:

```yaml
output_gain_sign: -1.0
```

Do **not** repair an actuator sign error by changing individual components of `K`. The node computes one mathematically consistent `K`; hardware mapping is isolated in `output_gain_sign` and the two motor signs.

After signs are verified:

```bash
ros2 param set /micro_lqr_controller dry_run false
```

This disarms the node. Move the RC switch away from ARM and back to ARM.

## LQR tuning

The cost is:

```text
sum(x'Qx + u'Ru)
```

with:

```text
Q = diag(q_pitch, q_pitch_rate, q_position, q_velocity)
R = r_total_torque
```

General effects:

- increase `q_pitch`: stronger upright-angle correction;
- increase `q_pitch_rate`: stronger angular-rate damping;
- increase `q_position`: stronger return to the arm position;
- increase `q_velocity`: stronger longitudinal-speed damping;
- increase `r_total_torque`: gentler, lower-torque control;
- decrease `r_total_torque`: more aggressive control.

Model or Q/R changes require editing YAML and restarting the node. Runtime parameters intended for cautious testing are:

```bash
ros2 param set /micro_lqr_controller torque_limit 0.03
ros2 param set /micro_lqr_controller lqr_gain_scale 0.7
ros2 param set /micro_lqr_controller output_gain_sign -1.0
```

Every such change disarms the controller.

## MATLAB verification

Open:

```text
tools/design_lqr.m
```

It builds the same model, performs exact ZOH discretization, calls `dlqr`, prints `K`, checks controllability, and simulates a small initial pitch error.

The original uploaded course file is retained at:

```text
reference/course_wheel_control_original.m
```

The parameter `model.use_course_legacy_b4` reproduces its exact fourth `B` entry. The default `false` uses the standard coefficient obtained by solving the coupled cart-pole equations.

## Debug array

`/micro_lqr/debug` is `std_msgs/msg/Float64MultiArray`:

| Index | Meaning |
|---:|---|
| 0 | x [m] |
| 1 | x_dot used by LQR [m/s] |
| 2 | pitch [rad] |
| 3 | pitch_rate [rad/s] |
| 4 | target x [m] |
| 5 | target x_dot [m/s] |
| 6 | unsaturated model total axle torque [N m] |
| 7 | common per-wheel torque after output sign and limit [N m] |
| 8 | armed |
| 9 | dry_run |
| 10 | output_gain_sign |
| 11 | per-wheel torque limit |
| 12 | raw pitch |
| 13 | raw pitch rate |
| 14 | position-state torque contribution |
| 15 | velocity-state torque contribution |
| 16 | pitch-state torque contribution |
| 17 | pitch-rate-state torque contribution |
| 18 | limited total axle torque |
| 19 | saturation flag |
| 20 | actual control dt |
| 21–23 | IMU/left/right message ages |
| 24–27 | raw motor positions and velocities |
| 28 | x_dot from finite difference |
| 29 | position error |
| 30 | velocity error |
| 31–34 | K in state order `[pitch,pitch_rate,x,x_dot]` |
| 35 | motor-feedback x_dot |
| 36 | velocity-estimator mismatch |
| 37 | controllability rank |
| 38 | largest closed-loop pole magnitude |
| 39 | lqr_gain_scale |
| 40 | course legacy B4 flag |

## Safety

The controller disables motor output on stale messages, offline RC/motors, motor fault flags, invalid IMU data, RC switch leaving ARM, or fall-angle violation. `hard_torque_limit` remains the final absolute per-wheel clamp.

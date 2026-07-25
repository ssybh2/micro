# Yaw-rate damping (no yaw-angle hold)

## Requirement implemented

This controller does **not** remember, integrate, or regulate a yaw angle.
The yaw-rate setpoint comes from `ReadDJIRC.left_x`. With the stick centered it is zero:

```text
yaw_rate_error = target_yaw_rate - filtered_imu_angular_velocity_z
```

Releasing the stick therefore requests zero yaw rate, damps the remaining rotation, and then
accepts whatever yaw angle the robot has reached.

The differential wheel torque is:

```text
yaw_diff_raw = yaw.output_sign * (
    yaw.rate_kp_per_wheel_nm_per_rad_s * yaw_rate_error
  - yaw.rate_kd_per_wheel_nm_per_rad_s2 * filtered_yaw_acceleration)
```

Default `yaw.rate_kd...` is zero, so the default controller is pure rate damping.
Inside `yaw.rate_deadband_rad_s`, the differential output is forced to zero immediately.
Therefore, after a disturbance rotates the robot to a new yaw angle and yaw rate returns to zero,
the yaw controller stops acting and accepts the new angle.

## Torque mixing

The existing pitch/translation controller produces a common physical wheel torque:

```text
common = total_axle_torque / 2
```

Yaw is added as an equal-and-opposite physical differential:

```text
left_physical  = common + yaw_diff
right_physical = common - yaw_diff
```

The existing motor installation signs are applied only after this physical mixing.

## Balance priority

Yaw cannot take torque authority away from balance. Available yaw headroom is:

```text
headroom = torque_limit - abs(common)
```

The yaw differential is limited by both `yaw.differential_torque_limit_nm` and this headroom.
If balance uses the full per-wheel software limit, yaw authority automatically becomes zero.

## IMU source

The controller reads `sensor_msgs/msg/Imu.angular_velocity.z` in rad/s. The AIMEtherCAT
HIPNUC task publishes all three angular velocity components and performs no axis remapping.
Use `yaw.imu_rate_sign` for IMU installation direction and `yaw.output_sign` for wheel/yaw
actuation direction.

## Sign test

1. Keep `dry_run: true`.
2. Calibrate and arm.
3. Rotate the body by hand around yaw.
4. Confirm `yawRate` follows the expected sign and `yawDiff` has the opposite control sense.
5. Set `dry_run: false`, suspend the robot, and use a very small limit (0.010 N·m).
6. Apply a small yaw disturbance. If it accelerates the disturbance, immediately disable and
   flip only `yaw.output_sign`.

Do not repair yaw direction by changing left/right motor signs; those signs are shared with the
existing pitch controller.

## Initial tuning

Start:

```yaml
yaw.rate_kp_per_wheel_nm_per_rad_s: 0.015
yaw.rate_kd_per_wheel_nm_per_rad_s2: 0.0
yaw.rate_deadband_rad_s: 0.02
yaw.rate_filter_hz: 15.0
yaw.differential_torque_limit_nm: 0.025
yaw.differential_slew_rate_nm_s: 0.50
```

- Yaw resistance too weak: increase Kp in steps of 0.005.
- Slow yaw oscillation: reduce Kp or increase deadband slightly.
- High-frequency left/right chatter: lower rate filter from 15 Hz to 10 Hz, or reduce Kp.
- Output frequently reports `yaw_lim=1`: common balance torque or yaw limit is constraining yaw.
- Do not enable yaw Kd until the P-only controller is stable and the yaw acceleration estimate is clean.

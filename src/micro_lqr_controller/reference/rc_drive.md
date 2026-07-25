# DJI RC drive commands

The controller subscribes to `custom_msgs/msg/ReadDJIRC` from EcatV2.

## Mapping

- `right_y`: forward/backward target velocity.
- `left_x`: target yaw angular velocity.
- `right_switch`: unchanged safety state machine (`1` calibrate, `3` arm, `2` disable by default).

Both stick axes are normalized to `[-1, 1]` by EcatV2. The controller applies a continuous
deadband, rescales the remaining stick travel back to full range, applies an adjustable sign,
and then applies a slew-rate limiter.

## Translation behavior

`right_y` produces `target_velocity_mps`. The existing controller integrates this velocity into
`target_position_m`, so releasing the stick stops the moving position target and the robot holds
the final commanded position. Existing pitch/position gains are unchanged.

## Yaw behavior

`left_x` produces `target_yaw_rate_rad_s`. The yaw controller uses:

```text
yaw_rate_error = target_yaw_rate - measured_yaw_rate
```

It never stores yaw angle. Releasing the stick commands zero yaw rate; it damps the remaining
rotation and then releases differential torque as soon as measured yaw rate enters the deadband.

## Direction checks

- If `right_y` forward drives backward, flip `rc.forward_sign`.
- If `left_x` direction is reversed, flip `rc.yaw_sign`.
- If yaw feedback amplifies rotation instead of opposing rate error, flip `yaw.output_sign`.

Do not alter the existing motor installation signs to fix remote-control direction.

# DJI RC drive validation report

## Source mapping verified

EcatV2 `custom_msgs/msg/ReadDJIRC` exposes `right_y` and `left_x` as normalized floating-point
values in `[-1, 1]`. The controller subscribes to the existing RC topic and copies those two
fields without changing the existing right-switch safety state machine.

## Static checks completed

- YAML parsing for `lqr.yaml` and `lqr_no_yaw.yaml`.
- Python syntax check for launch and pole visualizer scripts.
- Shell syntax check for `install_replace.sh`.
- XML parsing for `package.xml`.
- C++ delimiter and required-symbol checks.
- Confirmed the old `rc_.left_y` drive mapping is absent.
- Confirmed `right_y` drives target velocity and `left_x` drives target yaw rate.

## Numerical invariants checked

- Stick deadband produces exactly zero inside the configured band.
- Full stick rescales to exactly `+/-1` after deadband shaping.
- Full `right_y` requests the configured maximum velocity.
- Full `left_x` requests the configured maximum yaw rate.
- A nonzero yaw-rate command creates control effort even when measured yaw rate starts at zero.
- Centered yaw stick plus stopped yaw releases differential torque.
- Equal/opposite yaw mixing preserves the existing average wheel balance torque.

## Runtime limitation

This environment does not contain ROS 2 Humble, the project `custom_msgs`, or EtherCAT hardware,
so a real `colcon build`, receiver direction check, and powered robot test must be run on the
robot computer. The supplied YAML keeps `dry_run: true` for that first check.

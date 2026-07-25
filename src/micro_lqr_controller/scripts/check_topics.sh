#!/usr/bin/env bash
set -euo pipefail

echo "=== Required interfaces ==="
ros2 interface show sensor_msgs/msg/Imu
ros2 interface show custom_msgs/msg/ReadDJIRC
ros2 interface show custom_msgs/msg/ReadDmMotor
ros2 interface show custom_msgs/msg/WriteDmMotorMITControl

echo
echo "=== Required topics ==="
for topic in \
  /ecat/sn2031674/app1/read \
  /ecat/sn2031674/app2/read \
  /ecat/sn2031674/app3/read \
  /ecat/sn2031674/app3/write \
  /ecat/sn2031674/app4/read \
  /ecat/sn2031674/app4/write \
  /micro_lqr/debug \
  /micro_lqr/poles
do
  printf '%-38s ' "$topic"
  ros2 topic type "$topic" 2>/dev/null || echo "NOT FOUND"
done

echo
echo "=== Controller architecture ==="
ros2 param get /micro_lqr_controller control_mode || true
ros2 param get /micro_lqr_controller dry_run || true
ros2 param get /micro_lqr_controller output_gain_sign || true
ros2 param get /micro_lqr_controller torque_limit || true
ros2 param get /micro_lqr_controller cascade.pitch_limit_deg || true
ros2 param get /micro_lqr_controller cascade.outer_velocity_filter_hz || true
ros2 param get /micro_lqr_controller yaw.enable || true
ros2 param get /micro_lqr_controller yaw.imu_rate_sign || true
ros2 param get /micro_lqr_controller yaw.output_sign || true
ros2 param get /micro_lqr_controller yaw.rate_kp_per_wheel_nm_per_rad_s || true
ros2 param get /micro_lqr_controller yaw.differential_torque_limit_nm || true


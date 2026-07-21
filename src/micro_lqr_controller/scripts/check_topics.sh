#!/usr/bin/env bash
set -euo pipefail

echo "=== Interfaces ==="
ros2 interface show sensor_msgs/msg/Imu
ros2 interface show custom_msgs/msg/ReadDJIRC
ros2 interface show custom_msgs/msg/ReadDmMotor
ros2 interface show custom_msgs/msg/WriteDmMotorMITControl

echo
echo "=== Topics ==="
ros2 topic list | grep -E \
  '/ecat/sn2031674/app[1-4]/(read|write)|/micro_lqr/debug' || true

echo
echo "=== Topic types ==="
for topic in \
  /ecat/sn2031674/app1/read \
  /ecat/sn2031674/app2/read \
  /ecat/sn2031674/app3/read \
  /ecat/sn2031674/app3/write \
  /ecat/sn2031674/app4/read \
  /ecat/sn2031674/app4/write
do
  echo "--- ${topic}"
  ros2 topic type "${topic}" || true
done

echo
echo "Run these separately to inspect live signs:"
echo "ros2 topic echo /ecat/sn2031674/app1/read --once"
echo "ros2 topic echo /ecat/sn2031674/app2/read --once"
echo "ros2 topic echo /ecat/sn2031674/app3/read --once"
echo "ros2 topic echo /ecat/sn2031674/app4/read --once"

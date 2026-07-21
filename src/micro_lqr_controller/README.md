# micro_lqr_controller

ROS 2 Humble C++ controller for the fixed-leg two-wheel robot using
`AIMEtherCAT/EcatV2_Master`.

## Signal mapping

```text
app1/read  sensor_msgs/msg/Imu
app2/read  custom_msgs/msg/ReadDJIRC
app3/read  custom_msgs/msg/ReadDmMotor
app3/write custom_msgs/msg/WriteDmMotorMITControl
app4/read  custom_msgs/msg/ReadDmMotor
app4/write custom_msgs/msg/WriteDmMotorMITControl
```

The IMU installation is handled as:

```text
body pitch      = relative IMU roll
body pitch rate = IMU angular_velocity.x
```

The four-state vector is:

```text
[x, x_dot, body_pitch, body_pitch_rate]
```

The output is pure MIT torque:

```text
enable = 1
p_des = 0
v_des = 0
kp = 0
kd = 0
torque = LQR result
```

## Copy into the existing workspace

The ZIP contains:

```text
micro_lqr_ros2_ws/
└── src/
    └── micro_lqr_controller/
```

You can either build it as a separate overlay workspace or copy the package into
the existing EcatV2 workspace.

Separate overlay example:

```bash
cd /home/uonaim
unzip micro_lqr_ros2_ws.zip

source /opt/ros/humble/setup.bash

# Source the workspace in which EcatV2_Master/custom_msgs is already built.
source /home/uonaim/micro/install/setup.bash

cd /home/uonaim/micro_lqr_ros2_ws
colcon build --symlink-install --packages-select micro_lqr_controller
source install/setup.bash
```

Check interfaces and topic types:

```bash
ros2 run micro_lqr_controller check_topics.sh
```

Launch:

```bash
ros2 launch micro_lqr_controller micro_lqr.launch.py
```

## First-run RC workflow

DJI switch values are:

```text
up     = 1
middle = 3
bottom = 2
```

Default behavior:

```text
up:     disable motors and calibrate the current upright IMU/wheel zero
middle: arm LQR
bottom: disable motors
```

Start with the robot supported by hand.

1. Put the right switch at bottom.
2. Start EcatV2_Master and this controller.
3. Put the right switch at up while the robot is physically upright.
4. Put the right switch at middle.
5. Initially `dry_run=true`, so motors remain disabled. Check `/micro_lqr/debug`.
6. Confirm angle/rate/encoder signs.
7. Set `dry_run=false`; the parameter change automatically disarms.
8. Move the switch away from middle and back to middle to re-arm.

## Dry run

The default configuration does not enable either motor:

```yaml
dry_run: true
torque_limit: 0.05
```

Observe the state and predicted torque:

```bash
ros2 topic echo /micro_lqr/debug
```

Debug array order:

```text
0  x
1  x_dot
2  pitch rad
3  pitch_rate rad/s
4  target_x
5  target_velocity
6  raw_lqr_output
7  common_torque_after_sign_and_limit
8  armed
9  dry_run
10 output_gain_sign
11 torque_limit
```

Enable real output:

```bash
ros2 param set /micro_lqr_controller dry_run false
```

## Change the complete output direction

The requested adjustable sign is available as a live ROS parameter:

```bash
ros2 param set /micro_lqr_controller output_gain_sign -1.0
```

or:

```bash
ros2 param set /micro_lqr_controller output_gain_sign 1.0
```

Changing this parameter disarms the controller. Cycle the RC switch before
arming again.

The global sign is separate from the mirrored motor command signs:

```yaml
output_gain_sign: 1.0
left_motor_sign: -1.0
right_motor_sign: 1.0
```

## Increase torque gradually

The hardware peak torque is kept as an absolute limit:

```yaml
hard_torque_limit: 0.45
```

The initial operational limit is deliberately small:

```yaml
torque_limit: 0.05
```

Raise gradually while supporting the robot:

```bash
ros2 param set /micro_lqr_controller torque_limit 0.10
ros2 param set /micro_lqr_controller torque_limit 0.20
ros2 param set /micro_lqr_controller torque_limit 0.30
ros2 param set /micro_lqr_controller torque_limit 0.45
```

Each change disarms the controller.

## Encoder signs

If pushing the robot physically forward does not produce positive `x_dot`,
change one or both:

```yaml
left_encoder_sign: 1.0
right_encoder_sign: -1.0
```

The DM position feedback is unwrapped across ±P_MAX. Your EtherCAT config uses
`conf_pmax=pi`, so:

```yaml
motor_position_wrap_half_range: 3.141592653589793
```

## IMU signs

When the body tips in the defined positive pitch direction, `pitch` and
`pitch_rate` in the debug topic should use the same physical sign. Adjust:

```yaml
imu_angle_sign: 1.0
imu_rate_sign: 1.0
```

## Optional remote velocity command

Disabled by default:

```yaml
enable_velocity_command: false
```

When enabled, `left_y` controls target forward speed and the position reference
is integrated from that speed:

```bash
ros2 param set /micro_lqr_controller enable_velocity_command true
```

## Safety actions

The node sends `enable=0` and zero torque when any of these occurs:

- RC, IMU, or motor feedback timeout;
- RC offline;
- either motor offline;
- DM motor over/under-voltage, overcurrent, temperature, communication, or
  overload fault;
- right switch not in ARM;
- pitch exceeds `fall_cutoff_deg`;
- invalid quaternion;
- a sensitive parameter is changed.

After a fault or parameter change, cycle the RC switch before re-arming.

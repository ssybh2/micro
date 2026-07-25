# `/micro_lqr/debug` schema

Type: `std_msgs/msg/Float64MultiArray`

```text
0  position
1  velocity
2  pitch
3  pitch_rate
4  target_position
5  target_velocity
6  model_total_unsaturated
7  per_wheel_common_torque after sign/limit, before motor sign
8  armed
9  dry_run
10 output_gain_sign
11 per_wheel_torque_limit
12 pitch_raw
13 pitch_rate_raw
14 equivalent position contribution
15 equivalent velocity contribution
16 pitch contribution
17 pitch-rate contribution
18 total_torque_after_limit
19 torque_saturation_flag
20 actual_dt
21 imu_age_s
22 left_motor_age_s
23 right_motor_age_s
24 left_position_raw
25 right_position_raw
26 left_velocity_raw
27 right_velocity_raw
28 velocity_from_position
29 position_error
30 velocity_error
31 equivalent_K_pitch
32 equivalent_K_pitch_rate
33 equivalent_K_position
34 equivalent_K_velocity
35 velocity_motor_based
36 velocity_mismatch
37 controllability_rank
38 active_local_max_pole_abs
39 equivalent_gain_scale
40 use_course_legacy_b4
41 cascade_mode_flag
42 pitch_setpoint_raw
43 pitch_setpoint_limited
44 pitch_setpoint_command
45 outer_velocity_filtered
46 position_integral
47 attitude_error
48 outer_saturation_flag
49 cascade_pitch_limit_rad
50 cascade_pitch_slew_rate_rad_s
```

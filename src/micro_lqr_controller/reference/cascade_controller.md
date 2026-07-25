# Cascade position-to-pitch controller

## Motivation

The previous direct law was locally equivalent to:

```text
u = K_pitch*pitch + K_rate*pitch_rate + K_x*x_error + K_v*v_error
```

Large position error could therefore create a large torque contribution directly. In the recorded tests, the position contribution and attitude contribution repeatedly became large and opposite, causing the body to tilt several degrees while the cart crossed the target position repeatedly.

The new controller separates responsibilities:

```text
outer position loop -> pitch setpoint
inner attitude loop -> axle torque
```

## Outer loop

```text
v_outer = LPF(v_error, cascade.outer_velocity_filter_hz)

pitch_sp_raw = position_to_pitch_sign * (
    Kp_position * limited_position_error
  + Kd_velocity * limited_filtered_velocity_error
  + Ki_position * integral
)

pitch_sp_limited = clamp(pitch_sp_raw, +/- pitch_limit)
pitch_sp = slew_limit(pitch_sp_limited, pitch_slew_rate)
```

Default protections:

```text
pitch-setpoint limit: +/-2 degrees
pitch-setpoint slew:  5 degrees/second
outer velocity LPF:   5 Hz
position input limit: +/-0.50 m
velocity input limit: +/-0.80 m/s
```

## Inner loop

```text
attitude_error = pitch - pitch_sp
u_total = attitude_k_pitch * attitude_error
        + attitude_k_pitch_rate * pitch_rate
```

`u_total` is the model-sign total axle torque. The existing `output_gain_sign`, motor signs, per-wheel sharing, software limit, hard limit, RC safety, timeout safety, and fall cutoff are preserved.

## Default local equivalent gain

Ignoring setpoint saturation, slew limit, filters and integral, the cascade law has the local equivalent direct gain:

```text
K_equivalent = [-0.80, -0.04, -0.04, -0.04]
scale = 1.0
```

For the current nominal YAML model and 3-ms sample time, the local spectral radius is approximately 0.9987. This is intentionally not forced into the visualizer's green zone: the nonlinear pitch-request limit and rate limit are more important for preventing the large position-attitude exchange seen on the real robot.

## Debug topic extension

The original `/micro_lqr/debug` indices 0–40 are retained. New fields are appended:

```text
41 cascade_mode_flag
42 pitch_setpoint_raw [rad]
43 pitch_setpoint_limited [rad]
44 pitch_setpoint_command after slew limit [rad]
45 outer_velocity_filtered [m/s]
46 position_integral [m*s]
47 attitude_error [rad]
48 outer_saturation_flag
49 cascade_pitch_limit [rad]
50 cascade_pitch_slew_rate [rad/s]
```

In cascade mode, legacy indices 14–17 show the local-equivalent position, velocity, pitch and pitch-rate torque contributions. Because the pitch setpoint is limited and slew-limited, indices 14–17 do not necessarily sum exactly to the final torque during nonlinear limiting.

## Pole visualizer meaning

The monitor publishes the cascade controller's **local unsaturated equivalent four-state feedback**. It cannot include:

- pitch-setpoint saturation;
- pitch-setpoint slew limiting;
- the 5-Hz outer velocity filter state;
- torque saturation;
- friction, delay, backlash, tire slip or sensor noise.

Use the plot to reject a locally unstable parameter set, not as proof that a real-hardware setting is safe.

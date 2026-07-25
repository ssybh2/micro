# Live local pole visualizer

The controller supports two architectures:

```text
control_mode="lqr"      direct four-state feedback
control_mode="cascade"  position-to-pitch outer loop plus attitude inner loop
```

In `cascade` mode, the monitor publishes the local, unsaturated equivalent feedback gain:

```text
u = -K_equivalent * [pitch, pitch_rate, position_error, velocity_error]
```

It then plots the eigenvalues of:

```text
Acl = Ad - Bd*K_equivalent
```

The monitor deliberately does not model the pitch-setpoint clamp, pitch-setpoint slew limiter, outer velocity low-pass, integral state, torque saturation, delays, friction or noise. Therefore the plot is a local screening tool, not a complete robust-stability certificate.

## Message schema

Topic: `/micro_lqr/poles`

Type: `std_msgs/msg/Float64MultiArray`

```text
0    schema version (=1)
1    stable flag
2    local spectral radius
3    local radial margin 1-rho
4    published gain scale
5    sample time
6    manual/equivalent gain flag
7:10 active or equivalent K
11:30 four poles, each [real, imag, magnitude, damping ratio, frequency Hz]
31   cascade mode flag (optional extension)
32   pitch-setpoint limit [deg]
33   pitch-setpoint slew limit [deg/s]
```

## Interpretation

All local poles must remain inside the unit circle. In cascade mode, do not tune only to force the display into the green band. A slightly slower local position mode with a hard ±2° pitch request and a 5°/s pitch-setpoint slew is generally more useful on this robot than an aggressive raw velocity gain that produces a lower nominal rho but causes real high-frequency chatter.

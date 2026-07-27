# Adaptive trim and position-difference velocity filtering

This package is a local replacement derived from the public `ssybh2/micro` main-branch
interface and parameter layout. It does not modify the remote GitHub repository.

## New estimator

`velocity_from_position_raw` is only recomputed after both wheel callbacks have delivered
new samples. It then passes through `position_velocity_filter_hz` before blending with the
motor-reported velocity.

## Trim definition

`pitch_reference = manual_trim + auto_trim + position_correction`.

- `manual_trim`: fixed IMU mounting and fixed-structure bias.
- `auto_trim`: slowly learned load-dependent balance angle.
- `position_correction`: normal position/velocity outer-loop output.

Auto trim learns only when target velocity and yaw command are zero, translation speed and
pitch rate are small, pitch is safe, position error is bounded, and neither the outer loop
nor total torque is saturated. It waits for `dwell_s` before learning.

## Debug extension

Indices 0..72 retain the old layout. New indices are appended:

- 73 raw position-difference velocity
- 74 filtered position-difference velocity
- 75 manual trim
- 76 auto trim
- 77 total trim
- 78 complete pitch reference
- 79 auto-trim learning flag
- 80 encoder-pair update flag
- 81 optional stiction compensation total torque

# Live closed-loop pole visualizer

## What it shows

The monitor reads the parameters of the running `/micro_lqr_controller` node, rebuilds the same continuous model, exact-ZOH discrete model, and DARE solution used by `micro_lqr_node.cpp`, then evaluates the active feedback law

```text
u = -lqr_gain_scale * K * state_error
```

with

```text
state_error = [pitch, pitch_rate, position_error, velocity_error]
```

The plotted poles are the eigenvalues of

```text
Acl = Ad - Bd * (lqr_gain_scale * K)
```

For a discrete-time nominal model, all poles must satisfy `abs(lambda) < 1`.

## Install dependencies

```bash
sudo apt update
sudo apt install -y python3-numpy python3-matplotlib
```

## Build

From the ROS 2 workspace root:

```bash
colcon build --packages-select micro_lqr_controller --symlink-install
source install/setup.bash
```

## Launch

```bash
ros2 launch micro_lqr_controller micro_lqr.launch.py
```

The controller, pole monitor, and plot window start together.

Disable only the GUI while keeping the pole topic:

```bash
ros2 launch micro_lqr_controller micro_lqr.launch.py show_poles:=false
```

Change the monitor refresh period:

```bash
ros2 launch micro_lqr_controller micro_lqr.launch.py pole_monitor_period_s:=0.2
```

## Runtime tuning behavior

The current controller permits `lqr_gain_scale` to change at runtime, so this command updates the plot without restarting:

```bash
ros2 param set /micro_lqr_controller lqr_gain_scale 0.6
```

The current controller intentionally rejects runtime changes to `model.*` and `lqr.*` parameters, including manual K. Edit `config/lqr.yaml`, rebuild if necessary, and relaunch to visualize a new manual K.

## Message schema

Topic: `/micro_lqr/poles`

Type: `std_msgs/msg/Float64MultiArray`

```text
0    schema version (=1)
1    stable flag
2    spectral radius max(abs(lambda))
3    radial margin 1 - spectral radius
4    lqr_gain_scale
5    sample time [s]
6    manual-gain flag
7:10 active K = [K_pitch, K_pitch_rate, K_position, K_velocity]
11:30 four poles; each pole stores [real, imag, magnitude, damping ratio, damped frequency Hz]
```

## Interpretation limits

The unit-circle test is a nominal, unsaturated, linear-model test. It does not include motor delay, sensor filtering delay, static friction, tire slip, torque saturation, inaccurate mass/inertia/COM values, or an incorrect hardware sign mapping. A pole just inside the unit circle, such as `abs(lambda)=0.9996`, has very little practical margin.

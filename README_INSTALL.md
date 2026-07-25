# micro LQR pole visualizer overlay

This overlay was generated for `ssybh2/micro` and is intended to be extracted at the ROS 2 workspace root that already contains `src/micro_lqr_controller`.

## Apply

```bash
cd ~/micro/micro_lqr_ros2_ws
unzip -o micro_lqr_pole_visualizer_overlay.zip
sudo apt update
sudo apt install -y python3-numpy python3-matplotlib
colcon build --packages-select micro_lqr_controller --symlink-install
source install/setup.bash
ros2 launch micro_lqr_controller micro_lqr.launch.py
```

The overlay replaces only:

- `src/micro_lqr_controller/CMakeLists.txt`
- `src/micro_lqr_controller/package.xml`
- `src/micro_lqr_controller/launch/micro_lqr.launch.py`

and adds:

- `src/micro_lqr_controller/src/micro_lqr_pole_monitor.cpp`
- `src/micro_lqr_controller/scripts/pole_visualizer.py`
- `src/micro_lqr_controller/reference/pole_visualizer.md`

It does not modify `micro_lqr_node.cpp` or your `config/lqr.yaml` gains.

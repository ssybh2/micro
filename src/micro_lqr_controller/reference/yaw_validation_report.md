# Yaw-rate damping validation report

Validated in the artifact environment:

1. The base `micro_lqr_node.cpp` Git blob matched the current repository blob SHA
   `3b8884b2c1af9d56c1831756bdbf05328d7148b3` before modification.
2. C++ source delimiter check passed.
3. `clang++ -std=c++17 -fsyntax-only` passed using interface stubs for ROS 2,
   custom messages, and Eigen. This checks the complete modified translation unit's syntax,
   but is not a replacement for a real ROS 2/colcon build.
4. Python launch and visualizer files passed `py_compile`.
5. Shell scripts passed `bash -n`.
6. YAML files parsed successfully.
7. `package.xml` parsed successfully.
8. Standalone yaw mixer invariant test passed across common torque from -0.12 to +0.12 N*m
   and yaw rates from -3 to +3 rad/s:
   - neither physical wheel exceeded the per-wheel software limit;
   - average left/right physical torque exactly preserved the original common torque;
   - yaw differential became exactly zero inside the deadband;
   - positive and negative yaw rates produced opposite differential commands.

Not available in the artifact environment:

- ROS 2 Humble and the user's `custom_msgs` installation;
- EtherCAT hardware and HIPNUC IMU;
- real `colcon build` against the user's workspace;
- physical yaw sign verification.

The first hardware run must therefore keep `dry_run: true`, followed by a supported,
low-authority sign test with `yaw.differential_torque_limit_nm: 0.010`.

#!/usr/bin/env bash
set -euo pipefail
ros2 topic list | grep -E '(/micro_lqr/debug|/ecat/)' || true
ros2 node info /micro_lqr_controller || true

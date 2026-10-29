#!/usr/bin/env bash
set -eo pipefail
cd "$(dirname "$0")"
export ROS_DOMAIN_ID=42 ROS_LOCALHOST_ONLY=1
source ../camera/scripts/handeye_env.sh
source ../jia/ros2_ws/install_wsl/local_setup.bash
set -u
exec python3 run_task_b.py "$@"

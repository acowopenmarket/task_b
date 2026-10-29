#!/usr/bin/env bash
set -eo pipefail
source /opt/ros/humble/setup.bash
cd "$HOME/orbbec_ws"
# Moderate parallelism to avoid needlessly consuming resources of other projects.
export CMAKE_BUILD_PARALLEL_LEVEL=4
colcon build --symlink-install --parallel-workers 2 \
  --event-handlers console_direct+ --cmake-args -DCMAKE_BUILD_TYPE=Release

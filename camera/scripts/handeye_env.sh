#!/usr/bin/env bash
camera_project_root="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
source "$camera_project_root/scripts/camera_env.sh"
estun_workspace="${ESTUN_WS:-$camera_project_root/../estun/ros2_ws}"
if [[ -f "$estun_workspace/install/local_setup.bash" ]]; then
  source "$estun_workspace/install/local_setup.bash"
fi
export PYTHONPATH="$camera_project_root${PYTHONPATH:+:$PYTHONPATH}"

#!/usr/bin/env bash
set -eo pipefail
source "$(dirname "${BASH_SOURCE[0]}")/camera_env.sh"
mkdir -p "$HOME/orbbec_ws/results"
echo $$ > "$HOME/orbbec_ws/results/camera_launch.pid"
exec ros2 launch orbbec_camera gemini_330_series.launch.py \
  camera_name:=camera depth_registration:=true align_mode:=SW \
  align_target_stream:=COLOR enable_frame_sync:=true enable_depth_scale:=true \
  enable_color_undistortion:=true \
  color_width:=640 color_height:=480 color_fps:=15 color_format:=MJPG \
  depth_width:=640 depth_height:=480 depth_fps:=15 depth_format:=Y16 \
  enable_point_cloud:=false enable_colored_point_cloud:=false \
  enable_left_ir:=false enable_right_ir:=false publish_tf:=true "$@"

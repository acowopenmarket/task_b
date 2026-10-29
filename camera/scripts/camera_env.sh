#!/usr/bin/env bash
# Camera-only DDS domain; deliberately do not source the robot workspaces.
source "/opt/ros/${ROS_DISTRO:-humble}/setup.bash"
source "${ORBBEC_WS:-$HOME/orbbec_ws}/install/local_setup.bash"
export ROS_DOMAIN_ID="${ROS_DOMAIN_ID:-87}"
export ROS_LOCALHOST_ONLY="${ROS_LOCALHOST_ONLY:-1}"

#!/bin/bash
# Run the Gazebo GUI inside the Xvfb desktop served by noVNC.
export DISPLAY=:1
unset WAYLAND_DISPLAY
export XDG_SESSION_TYPE=x11
export QT_QPA_PLATFORM=xcb
unset GALLIUM_DRIVER
export LIBGL_ALWAYS_SOFTWARE=1
source /opt/ros/jazzy/setup.bash
source /home/zrtzrt/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${1:-192}
export GZ_PARTITION=${2:-vnc}
exec ros2 launch /home/zrtzrt/tune/gazebo_nav2.launch.py \
  params_file:=${3:-/home/zrtzrt/tune/nav2_baseline14.yaml} \
  headless:=false start_nav2:=false

#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/zrtzrt/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${1:-190}
export GZ_PARTITION=${2:-vis}
export DISPLAY=:0
export LIBGL_ALWAYS_SOFTWARE=1
export GALLIUM_DRIVER=llvmpipe
export QT_QPA_PLATFORM=xcb
unset WAYLAND_DISPLAY
exec ros2 launch /home/zrtzrt/tune/gazebo_nav2.launch.py \
  params_file:=${3:-/home/zrtzrt/tune/nav2_baseline14.yaml} \
  headless:=false start_nav2:=false

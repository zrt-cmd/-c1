#!/bin/bash
# Launch the avoidance simulation inside the Xvfb desktop (DISPLAY=:1).
export DISPLAY=:1
unset WAYLAND_DISPLAY
export XDG_SESSION_TYPE=x11
export QT_QPA_PLATFORM=xcb
unset GALLIUM_DRIVER
export LIBGL_ALWAYS_SOFTWARE=1

source /opt/ros/jazzy/setup.bash
source ~/ros2_ws/install/setup.bash

ros2 launch car1_gazebo gazebo_avoidance.launch.py

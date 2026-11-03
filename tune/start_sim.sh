#!/bin/bash
source /opt/ros/jazzy/setup.bash
source /home/zrtzrt/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=${1:-160}
export GZ_PARTITION=${2:-log}
exec ros2 launch /home/zrtzrt/tune/gazebo_nav2.launch.py \
  params_file:=${3:-/home/zrtzrt/tune/nav2_baseline14.yaml} \
  headless:=true start_nav2:=${4:-true} proxy:=${5:-false}

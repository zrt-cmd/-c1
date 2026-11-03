#!/bin/bash
# Usage: sweep.sh MODE [MODE...]
source /opt/ros/jazzy/setup.bash
source /home/zrtzrt/ros2_ws/install/setup.bash
for m in "$@"; do
  pkill -9 -f "gz sim" 2>/dev/null
  pkill -9 -f "gz-sim" 2>/dev/null
  pkill -9 -f "parameter_bridge" 2>/dev/null
  pkill -9 -f "ros_gz" 2>/dev/null
  pkill -9 -f "ros2 launch" 2>/dev/null
  pkill -9 -f "robot_state_publisher" 2>/dev/null
  pkill -9 -f "joint_state_publisher" 2>/dev/null
  sleep 2
  python3 /home/zrtzrt/tune/setup_mecanum.py "$m"
  ROS_DOMAIN_ID=180 GZ_PARTITION=m$m nohup bash /home/zrtzrt/tune/start_sim.sh \
    180 m$m /home/zrtzrt/tune/nav2_baseline14.yaml false > /tmp/sim_m$m.log 2>&1 &
  echo "=== MODE $m ==="
  GZ_PARTITION=m$m python3 /home/zrtzrt/tune/test_vy.py 2>&1 | tail -3
  sleep 1
done
pkill -9 -f "gz sim" 2>/dev/null
pkill -9 -f "ros2 launch" 2>/dev/null
echo SWEEP_DONE

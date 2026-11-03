#!/bin/bash
source /opt/ros/jazzy/setup.bash
source "$HOME/ros2_ws/install/setup.bash"

for i in $(seq 1 60); do
    if ros2 action list 2>/dev/null | grep -q navigate_to_pose; then
        echo NAV2_READY
        exit 0
    fi
    sleep 2
done

echo NAV2_TIMEOUT
exit 1

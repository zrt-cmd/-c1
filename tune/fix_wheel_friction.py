#!/usr/bin/env python3
"""Raise wheel friction so the diff drive does not slip in Gazebo."""
p = "/home/zrtzrt/ros2_ws/src/xflz_eai_car1/simulations/car1_gazebo/urdf/car1.gazebo.xacro"
s = open(p).read()

marker = "<!-- 2D 激光雷达"
blocks = []
for w in ("wheel_lf_link", "wheel_lb_link", "wheel_rf_link", "wheel_rb_link"):
    blocks.append(
        '    <gazebo reference="%s">\n'
        '        <mu1>2.5</mu1>\n        <mu2>2.5</mu2>\n'
        '        <kp>1000000</kp>\n        <kd>100</kd>\n    </gazebo>\n' % w)

insert = "    <!-- 轮子摩擦（防止仿真打滑） -->\n" + "".join(blocks) + "\n"
if "轮子摩擦" not in s:
    s = s.replace(marker, insert + marker, 1)
    open(p, "w").write(s)
    print("inserted friction blocks")
else:
    print("already present")

# C1 仿真建图实验

使用车模的 240° 雷达，赛道名义窄道宽度 45 cm。扫图车由实验脚本沿参考路线移动，SLAM 使用记录的雷达和位姿数据。

- `slam_full_deBKMT/` 保存首轮扫图和错位结果，扫过主要区域后在回程超时。
- `slam_turn_check_JzNzWq/` 保存 90° 转向实测。
- `slam_g_to_c_loop_compare_20261112/` 用同一趟 G→C 对照开关回环。
- `slam_blindspots_CYaIrE/` 在 C 暂停建图，转 180° 后恢复扫描并返程。
- `slam_odom_ab_GL1CPa/` 保存同趟轮式里程计和 Gazebo 真值对照、完整 MCAP、重放脚本、直接投影图，以及收紧搜索角度后的 2 cm 和 1 cm 结果。
- `slam_smoke_VJacZy/` 是 G 点首次局部建图，`slam_fixed_heading_20261112/` 是中途停止的试跑。

最新轮式里程计图是 `slam_odom_ab_GL1CPa/map_tight_wheel_1cm.yaml`。保持关回环和 ±3° 粗角度搜索，重放 269 帧后，六处 45 cm 窄道测得约 42–44 cm。G 附近残留重影，按 36 × 32 cm 车身每边加 2.5 cm 余量检查，G→C 仍不通。Gazebo 真值直接投影的 1 cm 对照图不加余量时连通，加余量仍不通。

## 从仓库根目录复现

在 ROS 2 Jazzy 环境下执行。以下命令分析录制数据并重建真值投影图。

```bash
exp=tune/experiments/slam_20261112
run="$exp/slam_odom_ab_GL1CPa"
world=ucar-3.0/src/xflz_eai_car1/simulations/car1_gazebo/worlds/math_room.sdf
python3 "$run/compare_poses.py" "$run"
python3 "$run/project_scans.py" "$run" 0.01
python3 "$exp/slam_full_deBKMT/audit.py" "$run" "$world" map_truth_direct_1cm
```

轮式里程计 SLAM 重放使用隔离的 ROS domain。在两个终端分别运行下面两段，两边都先加载 Jazzy 环境并进入仓库根目录。

```bash
export ROS_DOMAIN_ID=190 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
run=tune/experiments/slam_20261112/slam_odom_ab_GL1CPa
ros2 run slam_toolbox async_slam_toolbox_node --ros-args \
  -r __ns:=/wheel -r /map:=/wheel/map -r /tf:=/tf_wheel_replay \
  --params-file "$PWD/$run/tight_wheel_1cm.yaml"
```

```bash
export ROS_DOMAIN_ID=190 ROS_AUTOMATIC_DISCOVERY_RANGE=LOCALHOST
run=tune/experiments/slam_20261112/slam_odom_ab_GL1CPa
ros2 lifecycle set /wheel/slam_toolbox configure
ros2 lifecycle set /wheel/slam_toolbox activate
python3 "$run/replay_single.py" "$run"
ros2 run nav2_map_server map_saver_cli -t /wheel/map -f /tmp/c1_replay_1cm \
  --ros-args -p map_subscribe_transient_local:=true
```

# 只给 C 点的导航测试

测试从 G 点启动，按已验证的方式初始化 beam AMCL。唯一的导航目标是地图坐标
`(-1.80, 1.20)`。脚本调用 `ComputePathToPose` 时设 `use_start=false`，
ThetaStar 从当前 TF 位姿计算路线。给定参考路线的 JSON 文件仅用于旧的跟踪模式。

全局规划使用独立的 1 cm 占据图。在图上按控制器的矩形车体尺寸加 2.5 cm 余量
扩大墙体占据区域，表示车体中心不能到的位置。此计算假定车头保持地图 x 轴方向。
规划图发布在 `/planning_map`，AMCL 和局部代价图继续使用原始 `/map`。
[Nav2 静态地图层](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/core_servers/costmap_2d/costmap_plugins/static/)

从 ThetaStar 返回的路线自动提取折线段，逐段检查是否穿入规划图的占据区域。
每段交给 DWB 的 `FollowPath`，前一段到点后自动提交下一段。
第一次采用八邻域时，生成路线有若干对角线擦过规划图的占据区域，预检查拦下了行驶。
改为四邻域展开后，生成的折线通过了检查。
[ThetaStar 参数](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/planners_plugins/thetastar/configuring_thetastar/)

## 复现

在 WSL 中加载 Jazzy 和已安装的 C1 仿真工作区后，从仓库根目录运行：

```bash
python3 tune/run_follow_path.py --goal -1.8 1.2 --out tune/results/goal_c
```

输出目录须是新目录。可加 `--visual` 显示 Gazebo。
脚本会记录原始规划路径、自动生成的分段点、实际车体轨迹、速度指令和碰墙数据。
发车前确认了 19 个墙面接触话题，测试方块在南外墙触发物理接触消息后被移除。
Gazebo 真实位姿用于独立的离墙保护和终点评估。
当前验证针对静态比赛地图，按目标坐标规划一次再执行。

## 结果

- `rejected_path/` 记录八邻域候选路线的规划输出及拦截结果。
- `passed/` 记录四邻域试验。ThetaStar 成功，自动生成的 27 个 DWB 跟踪动作全部成功。
- 仿真时间 87.846 秒，实际行驶 9.316 m，终点误差 2.07 cm。
- 最小规划包络几何余量约 3.50 cm，最小实际包络几何余量约 2.45 cm。
- 正式行驶期间墙面接触消息为 0，记录到的最高横向速度指令为 0.12 m/s。

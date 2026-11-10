# AMCL beam 射线模型对照

## 可视化复跑补充

用户要求打开仿真再开始后，重新启动并从 G 再跑一次，配置和最高速度保持不变。
`replay/` 保存了该次初始化、原始遥测、结果及两张 Gazebo 原生窗口截图。
Windows 截屏接口未能正确捕获 WSLg 窗口，截图因此使用 X11 的原生窗口内容。

第二次全程同样到达 C，脚本退出码 0：仿真时间 98.802 秒，路程 10.1922 m，
最大位置误差约 1.40 cm，最小车体包络几何余量约 3.57 cm，终点误差约 2.0 mm。
至此有两次完整参考路线测试通过，尚不足以统计稳定成功率或验证自主规划、物理接触与实车表现。

配置：`tune/config/localization_beam.yaml`。
相对 `localization_precision_v2.yaml`，改为 `laser_model_type: beam`，
z_hit/z_rand 保持 0.8/0.2，z_short/z_max 设为 0，使实验只保留同样的命中和随机分量。
其他定位设置、1 cm 地图、起点、参考路线和 0.12 m/s 最高速度均保持一致。
这组分量是静态仿真对照配置，未验证动态障碍或实车传感器。

初始化使用 `initialize_localization.py`：已知 G 点，位置标准差 5 cm、朝向标准差约 5.7 度，
通过 `/initialpose` 设置；再请求 30 次静止扫描更新。没有使用 Gazebo 真值初始化粒子或修正速度。

## 结果

- `corner/`：独立启动的首个弯道测试，完成前两段。最大误差约 1.20 cm，最小包络余量约 3.64 cm。
  旧的 TF 监听线程清理代码使该进程在打印完整结果后退出码为 1；已补上显式停止线程。
- `full/`：重新启动后用同配置跑完整条参考路线，脚本退出码为 0。
  完成 12 段，从 G 到 C，98.604 秒仿真时间，路径约 10.1909 m。
- 全程最大位置误差约 1.98 cm；采样误差中位数约 6.1 mm，95 分位约 9.9 mm。
- 终点误差约 3.2 mm；最小车体矩形包络与墙的几何余量约 36.0 mm。
- TF 采样最大延迟约 20 ms；原始遥测共 742 行，详见 `full/tracking/telemetry.csv`。

## 解释与限制

控制使用 AMCL 的 map->base TF。Gazebo 真值仅记录误差、确认终点和执行离墙保护。
人工指定路线由简单全向速度控制器跟踪，未启动 Nav2 的局部控制器，
全局规划器虽由诊断 launch 启动，但本次行驶未使用其路径。
因此本结果验证的是本轮正常定位下的参考路线跟踪，不能宣称 Nav2 自主规划和避障已通过。

最初一次全程复测成功，后又按用户要求可视化复跑成功一次；尚未统计重复运行的成功率。
余量基于 0.36 x 0.32 m 矩形包络，不是接触传感器记录，不能保证模型所有零件无接触。
仍需后续重复验证、物理碰撞检测与真车复测。
当前 beam 是仿真候选配置，未替换原始真车参数或所有仿真默认模板。

## 复现顺序

在 WSL source ROS Jazzy 和 ros2_ws 后，使用独立的 ROS_DOMAIN_ID/GZ_PARTITION：

1. 用 `localization_probe.launch.py` 和 `localization_beam.yaml` 启动。
2. 运行 `initialize_localization.py --out <目录>/initialization.json`。
3. 运行 `drive_manual_g_to_c.py --use-amcl --speed 0.12 --world <math_room.sdf路径> --out <目录>/tracking`。

另一台设备需把候选配置中的地图路径改为本机的 `tune/maps/math_room_1cm.yaml`。

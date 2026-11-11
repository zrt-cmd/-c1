# Nav2 参考路线跟踪

给定路线是先前验证过的 12 段 G 到 C 参考路线。控制器使用 DWB，
定位使用 beam AMCL、显式初始协方差和 30 次静止更新。
局部代价图使用 1 cm 静态地图和 36×32 cm 的车体包络，另加 5 mm padding。
控制器启用了正、负横移，平移速度上限 0.12 m/s。
[DWB 参数说明](https://docs.nav2.org/jazzy/configuration_and_development/configuration_guide/controller_plugins/dwb_controller/dwb_params/controller/)

测试启动时复制比赛场景，在副本的 19 个墙体碰撞几何上添加接触传感器。
行驶前在南侧外墙放置小方块，收到带双方碰撞体名称的物理接触消息后移除方块。
脚本记录 Gazebo 位置、Nav2 速度、墙面接触与几何余量。
实际终点距 C 不超过 5 cm、所有跟踪动作成功、墙面接触消息为零时通过。
车体几何余量低于 8 mm 会提前中止测试。
[Gazebo 接触传感器](https://gazebosim.org/docs/harmonic/sensors/#contact-sensor)

## 复现

在 WSL 中加载 Jazzy 和已安装的 C1 仿真工作区后，从仓库根目录运行：

```bash
python3 tune/run_follow_path.py --out tune/results/follow_path_run
```

输出目录须是新的目录。加 `--visual` 可显示 Gazebo。
仿真工作区的 Gazebo 启动文件需包含本次新增的 `world_file` 和 `headless` 参数。
脚本默认逐段提交 `FollowPath`，每段到点后自动发下一段。
用 `--continuous` 可提交一条完整路径供对照，`--params` 可使用记录中的控制器参数。

## 调整记录

- 首轮使用一个整路径动作，在第一个弯后往复移动，累计行驶 1.84 m 后报进度不足。
- 缩短前视距离并增大目标距离权重后，整路径动作提前切弯，在累计行驶 1.06 m 时触发离墙保护。
- 按拐点分段后，第三轮在距第二段拐点约 2.3 cm 处不再发出运动速度，未达到该轮的到点条件。
- 最终测试使用 17×17 个平移速度样本、0.6 s 预测时长以及 2 cm 的 Nav2 到点容差。

## 实测结果

`continuous_a/`、`continuous_b/` 和 `segmented_c/` 保存以上未通过的三轮。
`segmented_pass/` 保存最终通过的一轮，启动时验证了全部 19 个墙面话题，
并在南外墙收到测试方块接触消息后移除了方块。
一次方块创建请求超时已在评估脚本中加了有限重试。

Nav2 12 个动作全部成功，真实车体到达 C 点，终点误差 1.35 cm。
行驶 10.02 m，仿真时间 89.43 s，实际观察到横向速度指令最高 0.12 m/s。
墙面接触消息为零，最小几何离墙余量约 3.42 cm。
本轮输入已给定的参考路线，局部代价图使用静态墙体图，验证的是 Nav2 跟踪能力。
全局规划、动态障碍和重复运行成功率仍待验证。

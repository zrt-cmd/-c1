# C1 智能小车项目

两台电脑共享的小车代码、地图、Gazebo 仿真、调参脚本及进度记录。

## 换电脑接着做

第一次在另一台电脑运行

```bash
git clone https://gitee.com/zrt-cmd/intelligent-life-robot-car-c1.git
cd intelligent-life-robot-car-c1
```

以后在开始工作前运行 `git pull --ff-only`。换电脑前，在仓库根目录提交并推送自己的修改。

```bash
git status
git add <这次修改的文件或目录>
git commit -m "说明这次改动"
git push
```

两台电脑都以本仓库为工作目录。请让另一边的 Codex 先阅读 `PROJECT_STATUS.md`，再核对本地配置。对话的关键进度保存在该文件中。

## 目录

- `ucar-3.0/` 保存 Windows 原始工作副本的代码、地图和本地修改。上游为 https://gitee.com/gzist-iflytek/ucar-3.0 ，授权见其中的 `LICENSE`。
- `tune/` 保存调参脚本、Nav2 参数快照、横移诊断程序和可视化证据。
- `wsl_snapshot/` 保存 WSL 运行副本与 Windows 源码存在实际差异的两个文件，以及来源清单。比较时已排除 CRLF/LF 差异。
- 根目录的 Markdown 课程资料、地图绘制与连通性检查脚本一并保存。

上游基准提交为 `adc0117ccdeef3031d1e32d3f24f6379e638cfef`。本仓库以当前文件快照建立独立历史。

## 仿真状态

比赛目标为 G 点到 C 点，复杂通道宽 45 厘米，碰壁判失败。

临时副本已验证前进、左右横移及原地旋转，实测与原生 Gazebo 渲染截图一致。验证中隔离了机械臂/夹爪的 `gz_ros2_control` 插件，修正了轮子摩擦方向的参照坐标系。这项修正目前应用在诊断程序生成的临时模型中。

完整启动、导航横移及 G→C 无碰撞通关还待验证。详细结果与后续工作见 `PROJECT_STATUS.md`。

## 在 WSL 复现横移测试

需要 Ubuntu 24.04、ROS 2 Jazzy、Gazebo Harmonic，以及原项目的 Gazebo/ROS 依赖。真车使用 ROS 2 Humble。Git 同步的是代码与记录，各电脑的运行环境需要单独准备。

在 WSL 中进入本仓库，使用一个尚不存在或已备份的 `~/ros2_ws` 工作空间。以下命令会复制源码，并以保存的 WSL 快照覆盖两个有差异的文件。

```bash
mkdir -p ~/ros2_ws/src
cp -a ucar-3.0/src/xflz_eai_car1 ~/ros2_ws/src/
cp -a wsl_snapshot/src/xflz_eai_car1/. ~/ros2_ws/src/xflz_eai_car1/

source /opt/ros/jazzy/setup.bash
cd ~/ros2_ws
colcon build --symlink-install --packages-up-to car1_gazebo
source install/setup.bash
```

返回本仓库目录，运行已验证的独立底盘测试。

```bash
python3 tune/verify_lateral_drive.py --body-frame --workspace ~/ros2_ws
```

模型输入已保存在 `tune/fixtures/car1_lateral_current.sdf`，运行结果写入 `tune/results/`。测试会在临时世界中使用地图下方空地，并隔离机械臂控制插件，结束后清理自己的进程。

可视化模式带截图检查点，会等待同名的 `.continue` 文件，主要用于人工逐帧核对，使用前请查看 `tune/verify_lateral_drive.py`。已有截图在 `tune/visual_check_01/`。

## 历史脚本与参数

`tune` 中保留了前期排查脚本、`nav2_baseline*.yaml` 及历史 Optuna 数据库。部分历史脚本和参数包含 `/home/zrtzrt` 或 Windows 桌面的绝对路径，在其他电脑使用前需要替换。它们包含会改动仿真模型的实验程序，应按进度记录选择运行。

历史 Optuna 结果包含启动失败和评估失真的轮次，尚未得到可靠的通关配置。旧的激光距离阈值碰撞判定还需要以车身实际接触验证。

noVNC 辅助脚本使用前需要通过环境变量设置 `VNC_PASS`。

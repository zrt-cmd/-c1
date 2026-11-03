# 教程冲突与待核对点

下列记录用于防止把课程材料中的示意、旧版本和现场事实混在一起。存在矛盾时回查源码和运行数据。

## 已发现的内部矛盾

- 固定抓取章节的原理和易错点明确解释 `/arm/grasp_state` 不存在、没有奇异检测、部分参数未使用，但本页目标与总结仍提及该话题、奇异检查和六阶段状态遥测。这两处说法相互冲突。使用前查代码与 `ros2 topic list`，不能凭总结承诺接口存在。[原理与源码提示](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-06-fixed-grasp/index.html#s2) [本页总结](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-06-fixed-grasp/index.html#s10)
- 视觉抓取易错点解释 `rgb_annotated` / `depth_overlay` 不存在，以及没有已抓物体记忆去重；本页部分验收文字和总结却仍要求这些图像、称自动 skip 重复目标。按本页自己的源码说明，应核对 `/visual_grasp/visualization`、`/visual_grasp/mask` 与确认服务，并检查当前代码版本。[源码边界提示](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-07-visual-grasp/index.html#s9) [本页总结](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-07-visual-grasp/index.html#s10)
- Nav2 章节说明未启用 backup，DWA 章节与首页的个别概述却写到后退恢复。是否后退以 `behavior_plugins` 和行为树 XML 为准。[导航配置说明](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-03-nav2-navigation/index.html#s2) [多点导航恢复说明](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-06-radar-multiwaypoint/index.html#s9)

## 与本项目现场的差异

- 教程建图基线包含 `minimum_travel_distance=0.01`、`minimum_travel_heading=0.02`、`map_update_interval=0.1`。2026-11-02 SSH 读到的车上阈值与此一致，但现场 `max_laser_range=8.0`、`minimum_time_interval=0.2`、`transform_timeout=0.5`，与教程 12m 等说明有差异。不能拿本地旧快照的转角阈值 2.0 代替现场 0.02。[教程参数表](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-02-slam-toolbox/index.html#s7)
- 教程把里程计输入称作轮速，2026-11-02 实际读取的 `odom_publisher_node.py` 将速度指令积分并写入 odom_raw。该源码证据只证明该节点的数据来源，车轮实际运动和 IMU 刻度仍需实测。
- `map_save` 默认保存到安装包 maps/，当前导航入口却从源码 maps/ 取图。保存和加载应明确同一路径，不能认为默认目录天然一致。[保存步骤](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-02-slam-toolbox/index.html#s8)
- 小地图不能单独证明建图崩溃。最新读取的现场日志表明 SLAM 在 Ctrl+C 时正常退出，启动期出现过 TF 消息过滤队列满，尚无持续故障的运行记录。参数偏密也不能直接证明 CPU 超载。

## 教程中尚待源码确认的表述

- DWB 章节称评分归一化后选择最高分，且用“车宽加两倍 inflation_radius”断言窄道必不可行。这些说法涉及具体插件评分与软硬代价处理，使用前核查 Nav2 对应版本实现，不能作为已验证的计算公式。[原文](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-05-dwa-teb-avoidance/index.html)
- 讲义中的帧率、成功率、板卡内存/算力、接口路径是对应实验的记录或验收目标。部分章节属于实现练习或无配套资源，不能据此断言用户的车已经部署对应模型/功能。
- 保留的 shell、Python 和 ROS 命令含伪代码、绝对路径与示例值。现场使用前应核验入口、环境、串口占用和消息类型。

## 缺失材料

- 小车智能工厂 EXP-02/03/04 为建设中占位页。
- RTAB-Map 实验明确暂缺项目专属源码。
- 30 个同站点附件返回 HTTP 404，含一张 rqt 图片及多段视频。具体 URL 见 COVERAGE.md。
- 未逐项观看图片/动画或运行实验，材料的完整归档不等于真机测试完成。

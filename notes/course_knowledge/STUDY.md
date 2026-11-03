# 课程核读笔记

笔记只记录课程主线和明确的工程边界。完整步骤、代码和数值以链接章节全文为准。下列平台不能混用硬件参数。

## 核心课程

- **ROS2，20 实验。** 计算图与 DDS 发现、工作空间 overlay、包与依赖、节点执行器、话题/服务/动作、自定义接口、参数、时间、域隔离、多机、rqt/RViz、rosbag、launch、元包、TF2、URDF。数据链路排错顺序为节点存在、话题名/类型、QoS 兼容、DOMAIN_ID、时间戳与 TF。参数声明类型、运行时应用逻辑和持久化 YAML 是不同环节。源码修改是否重编应以安装文件是否符号链接判断。[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/index.html)
- **视觉，12 实验。** 颜色阈值、二维码、轮廓、工业缺陷、垃圾分类、分割、DeepSORT 跟踪、人脸关键点、人体关键点、深度测距、体感、手势。传统图像处理和神经网络分别部署。YOLO 的预处理、RKNN 导出输出布局、后处理解码必须配套，模型文件扩展名不能证明精度或量化方式。课程序号与仓库 vision/ 目录编号顺序不同，应按实验名定位。深度单位和光学坐标系在抓取前统一。[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/index.html)
- **语音与大模型，6 实验。** 六麦串口唤醒、流式 ASR、Matcha-TTS+Vocos、对话状态机、RKLLM 意图/工具调用、情绪融合。硬件唤醒与软件 KWS 有不同入口。播报要先检查声卡播放，再检查合成模型。双轮排空、互斥事件和打断后的旧回复失效处理防止自问自答。情绪课包含需自行实现的步骤，不能直接假定现有车有完整成品。[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp002-speech-interaction/index.html)
- **SLAM 与导航，7 实验。** URDF/TF → SLAM Toolbox → Nav2 → AMCL → DWB 避障 → 多航点 → RTAB-Map 原理。建图时 map→odom 由 SLAM 发布，导航定位时由 AMCL 发布，odom→base_footprint 来自 EKF。课程基线的 0.01m/0.02rad 阈值和 0.1s 刷图间隔不能单凭大小断言故障。先确认 /scan、TF、车速及重影出现时刻，再单变量对照。RTAB-Map 章节明确缺项目专用源码。[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/index.html)
- **机械臂，7 实验及标定专题。** 模型与关节 → 串口/PWM → FK → IK → 动作回放 → 固定抓取 → 视觉抓取。教学简化模型和真机几何/舵机零位分别对照。PWM 动作回放与弧度关节指令有不同入口。set_pick_target 与 target_pose 有不同的执行语义。视觉抓取需观察锁定坐标、实测关节刷新、深度去离群和 TF 转换。内参、舵机零位、TCP、手眼外参各自标定，手眼求解结果还要确认确实接入运行链路。[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/index.html) [标定专题](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/calibration/index.html)

## 综合案例

- **智慧工厂 B2，9 实验。** 需求→架构→接口→语音→视觉装货→分拣→码垛→急停→整体验证。调度、抓取、输送线、语音、安全监护分工明确。视觉装货与固定示教分拣使用不同定位链路。危险解除后回 work 并重新触发任务，不能据此推断自动续跑。急停、力矩释放与重新上力要按该平台实际接口核验。[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006-smart-factory/index.html)
- **小车智能工厂 C1，4 实验。** EXP-01 有建图导航正文，EXP-02 视觉抓取、EXP-03 语音大模型、EXP-04 综合闭环目前均为建设中占位页。不能声称这三章提供可运行教程。EXP-01 的视频本身也标作占位素材。[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp007-intelligent-factory/index.html)
- **老人跌倒监护，9 实验。** 检测/姿态→持续时间判定→询问确认→告警与证据，呼救另有直接告警路径。教程使用六状态和超时约束，RTMPose/BlazePose 可作不同后端。VLM 默认启用状态和内存预算需按该案例配置检查。教程中案例模型不等于已在用户车上部署。[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006a-elderly-companion/index.html)
- **通用目标追踪，9 实验。** 检测/跟踪提供候选 ID，LLM 门控或 VLM 拼图选择必须映射回实际候选。清除词优先、缺失/歧义 ID 拒绝、epoch 防迟到、单写者队列应用副作用。文本测试、fake 桩和摄像头真链路有不同的验收含义，跟踪框与实际底盘跟随需要分别核对。[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006b-object-tracking/index.html)
- **智慧交通 C1，4 实验。** 沙盘建图与语义点位→红绿灯/标志视觉→循线与信号状态机→泊车任务编排。课程的示例话题、阈值和点位用于该沙盘，应先确认对应程序是否已存在于当前车的仓库。[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp008-smart-traffic/index.html)

## 行业课程

- **端到端平台 LZ-XL01，4 实验。** 主机环境、双路供电与舵机编号、相机别名和主从臂标定、遥操作验收。串口设备序号会变，稳定别名依赖设备身份或 USB 拓扑。主从范围和中位校准必须对应同一平台。[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind01-endtoend-platform/index.html)
- **双臂采训推，4 实验。** 遥操作采集→LeRobot 数据转换→ACT 训练和检查点→真机推理/VLA 对照。图像键、状态/动作维度、顺序、时间对齐和归一化需贯穿采集与部署。训练 loss 与任务成功率分别验收，VLA 的能力不能从介绍直接推断到未训练任务。[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind02-act-vla-pipeline/index.html)
- **物料分拣 LDT-01，4 实验。** 左黑件入左格、右银件入右格，示教质控、数据列修复/转换、训练、带起步和限速平滑的推理。LDT-01 的关节语义、16 维状态和双臂通信不适用于 C1 五自由度臂。[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind03-material-sorting/index.html)
- **家庭整理 LDT-01，4 实验。** 左手开盖→右手入罐→左手盖盖的时序示范，训练数据与摄像头布局保持一致。推理用单独决策/发布节拍及平滑限速，并检验物体摆放偏移后的成功率。课程数值属于该检查点/平台的示例配置。[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind04-home-tidying/index.html)
- **四足巡检 Go2 Edu，4 实验。** SDK 运动准备→3D 点云 SLAM→定位/多点导航→视觉巡检与报告。3D 激光和 Jetson 的配置与 C1 的 2D 雷达/RK3576 分开使用。投影栅格、重定位和视觉识别在部署前需检查各自真实话题与程序入口。[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind05-industrial-inspection/index.html)

## 回答小车问题时的核对顺序

1. 原文确认讲的是哪个平台、哪一章、示意还是实现。
2. 查当前仓库中接口与参数，再查 install/ 是否使用同一版本。
3. 真车问题使用运行参数、节点、话题和时间戳日志验证，保存失败发生前后的数据。
4. 对建图先看 scan/TF/航向/回环的时间关系。对抓取先看关节实测/内参/深度/外参/IK 的误差来源。对播报先分离音频播放链与 TTS 合成链。
5. 每次只改变一个变量，记录改前改后同一路线或同一物块的结果。

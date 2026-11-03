# 页面中的代码示例

以下为网页 pre 区块的原文提取，未执行。可能包含占位路径、伪代码和版本差异。

## EXP-01 · 需求分析（R1-R10） — 智慧工厂装货分拣码垛综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006-smart-factory/exp-01-requirements/index.html)

````text
# 物流链（每个阶段收尾都有"回 work"这一步安全握手）
装货区  →  分拣区  →  流水线（定点停靠）  →  码垛区
 R4 视觉抓取    R5 抓取放流水线   送料+光电定点停      R6 定点取料码放
        ⟳ 回work        ⟳ 回work        ⟳ 回work        ⟳ 回work  (R4.1)
````

````text
# ① 在 eai_factory 包目录下：快速定位各条款行号
ls docs/
grep -n "R[0-9]" docs/01_需求总览.md      # 定位 R1-R10 各条款
grep -n "R8\|R9" docs/05_流程与安全.md    # 定位打断与安全章节
# L62-L68(R8)  L70-L78(R9)  L80-L88(R9.1) ...

# ② 架构速览与关键决策（只记录，不执行）
head -50 README.md
# 架构速览：N1 语音 / N2 调度器 / N-grasp 双模式 / N3 安全 / N-conveyor 流水线
# 复用清单：car1_bringup / car1_arm_controller / car1_peripherals / apriltag_detector

# ③ 一键启动命令（本实验仅记录语法，7 个开关 EXP-02 展开）
ros2 launch eai_factory eai_factory.launch.py
# 7 开关：start_base / start_viz / start_voice / start_vision
#         / start_grasp / start_conveyor / start_radar
````

## EXP-02 · 五节点系统架构 — 智慧工厂装货分拣码垛综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006-smart-factory/exp-02-architecture/index.html)

````text
def _run_task(self, task):
    if task == 'palletize':
        self._start_palletize_sequence()   # 码垛：先送料后抓取（异步两步）
    elif task in ('load', 'sort'):
        self._trigger_grasp(task)          # 装货/分拣：直接触发抓取
    else:
        self.get_logger().warn(f'[SCHEDULER] unknown intent: {task}')

def _conveyor_state_cb(self, msg):
    # palletize：货物到达光电(sensor_blocked) -> 在取料位触发抓取
    if self._awaiting_sensor and msg.data == 'sensor_blocked':
        self._awaiting_sensor = False
        self._trigger_grasp('palletize')
````

````text
# ① 列出调度器全部回调/方法及行号
grep -n "def _" eai_factory/scheduler_node.py
# _run_task / _start_palletize_sequence / _conveyor_state_cb
# _trigger_grasp / _retry_grasp / _grasp_status_cb ...

# ② 三类执行节点的服务/话题实现位置
grep -n "create_service\|create_subscription\|create_publisher" \
  eai_factory/grasp_node.py eai_factory/conveyor_node.py eai_factory/safety_node.py

# ③ launch 的 7 开关与两种启动方式
grep -n "DeclareLaunchArgument\|ExecuteProcess\|Node(" launch/eai_factory.launch.py
# start_base/viz/voice/vision/grasp/conveyor/radar（radar 默认 false）
````

## EXP-03 · 接口设计 — 智慧工厂装货分拣码垛综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006-smart-factory/exp-03-interface-design/index.html)

````text
mode: production            # L7：debug/production 双模式
vision: { tag_id: 3, tag_size: 0.025, tag_family: tag36h11 }   # L56-L63
grasp.tasks:                              # L65-L90 每任务 [x,y,z,pitch] 四元组 + vision 标志
  load:      { vision: true,  observe_pose: [0.16,0.0,0.27,2.5307], place_pose: [0.05675,-0.13166,0.20386,2.93322] }
  sort:      { vision: false, observe_pose: [0.0622,-0.1158,0.1951,3.0371], place_pose: [0.1434,-0.1296,0.2075,2.7283] }
  palletize: { vision: false, observe_pose: [0.145,0.0027,0.1850,3.1381], place_pose: [0.0608,0.1345,0.2051,2.6461] }
safety:  { front_start_deg: -30, front_end_deg: 30, danger_dist: 0.20, persist_time: 0.5 }  # L92-L98
intents:                                   # L110-L117 意图 ↔ 服务名 表驱动映射
  load:      { action: "/eai_factory/start_load" }
  sort:      { action: "/eai_factory/start_sort" }
  palletize: { action: "/eai_factory/start_palletize" }
````

````text
# ① 五节点的订阅/发布/服务/客户端实现位置（一把抓）
grep -n "create_publisher\|create_subscription\|create_service\|create_client" eai_factory/*.py
# voice_node.py L250-L253: A1 intent / A3 interrupt / A2 speak
# safety_node.py L56-L63:   C1/C2/C3 发布 + C4 客户端
# conveyor_node.py L109-L111: F3 Trigger 服务端
# scheduler_node.py L62-L67:  E1-E3 服务端注册

# ② 运行时验证（可选，需系统已启动）
ros2 node list | grep eai_factory        # 应看到 5 个节点
ros2 topic list | grep eai_factory       # intent/speak/interrupt/danger/...
ros2 service list | grep eai_factory     # start_load/start_sort/start_palletize/grasp/start/...
ros2 interface show std_srvs/srv/Trigger # bool success  +  string message
ros2 interface show geometry_msgs/msg/PoseStamped

# ③ 手动触发装货（须安全环境！Trigger 请求体为空 {}）
ros2 service call /eai_factory/start_load std_srvs/srv/Trigger "{}"
# response: success: True / message: 'load task triggered'
````

## EXP-04 · 语音交互集成 — 智慧工厂装货分拣码垛综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006-smart-factory/exp-04-voice-integration/index.html)

````text
curl http://127.0.0.1:8080/v1/models
# 返回模型清单即为在线，否则先启动 Qwen3.5-0.8B 推理服务
````

````text
# 热词表：6 条词条，score=2.0
cat hotwords.txt
# intents 映射与唤醒词
grep -n -A6 "intents" config/eai_factory.yaml
grep -n "wakeup_words" config/eai_factory.yaml
````

````text
ros2 launch eai_factory eai_factory.launch.py start_voice:=true
````

## EXP-05 · 视觉自适应装货 — 智慧工厂装货分拣码垛综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006-smart-factory/exp-05-visual-loading/index.html)

````text
ros2 launch eai_factory eai_factory.launch.py start_voice:=false
# 观察启动日志：vision_node 加载 venv cv2、tag 参数与内参
````

````text
ros2 topic hz /eai_factory/vision/detected
ros2 topic echo /eai_factory/vision/target --once
position: z ≈ 0.19（PoseStamped 在 base_link 下）
````

````text
ros2 param set /eai_factory_grasp grasp.pick_offset_y -0.015
# 每改一档试抓 2~3 次；调完写回 yaml 固化
````

## EXP-06 · 智能分拣 — 智慧工厂装货分拣码垛综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006-smart-factory/exp-06-sorting/index.html)

````text
grep -A4 "sort:" config/eai_factory.yaml
sort: { vision: false, observe: [0.0622, -0.1158, 0.1951, 3.0371],
             place: [0.1434, -0.1296, 0.2075, 2.7283] }
````

````text
ros2 launch eai_factory eai_factory.launch.py
# 语音喊"执行分拣"，或直接：
ros2 service call /eai_factory/start_sort std_srvs/srv/Trigger
````

````text
ros2 param set /eai_factory_grasp grasp.lift_height 0.05
# 注意：码垛也用该参数，调完要做回归：再跑一次 start_palletize
````

## EXP-07 · 自主码垛 — 智慧工厂装货分拣码垛综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006-smart-factory/exp-07-palletizing/index.html)

````text
sudo bash scripts/setup_conveyor_gpio.sh
cat /sys/class/gpio/gpio17/value   # 输出 0 或 1 即成功
````

````text
source /opt/ros/humble/setup.bash
source /home/iflytek/ros2_ws/install/setup.bash
ros2 launch eai_factory eai_factory.launch.py \
  start_voice:=false start_vision:=false start_viz:=false
````

````text
sudo bash scripts/setup_conveyor_gpio.sh
cat /sys/class/gpio/gpio17/value        # 0 或 1
ros2 launch eai_factory eai_factory.launch.py \
  start_voice:=false start_vision:=false start_grasp:=true start_viz:=false
````

````text
ros2 topic echo /eai_factory/conveyor/state      # 终端1
ros2 service call /eai_factory/conveyor/run_until_sensor std_srvs/srv/Trigger
# 触发后尽快把物品放到流水线起始端
````

````text
ros2 topic echo /eai_factory/conveyor/state      # 三终端同开
ros2 topic echo /eai_factory/grasp/status
ros2 service call /eai_factory/start_palletize std_srvs/srv/Trigger
````

````text
ros2 service call /eai_factory/conveyor/emergency_stop std_srvs/srv/Trigger
ros2 service call /eai_factory/conveyor/run std_srvs/srv/Trigger
success: False — "emergency active — call reset_emergency first"
ros2 service call /eai_factory/conveyor/reset_emergency std_srvs/srv/Trigger
ros2 service call /eai_factory/conveyor/run std_srvs/srv/Trigger   # 恢复成功
````

## EXP-08 · 危险检测与急停 — 智慧工厂装货分拣码垛综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006-smart-factory/exp-08-emergency-stop/index.html)

````text
ros2 launch eai_factory eai_factory.launch.py \
  start_base:=false start_voice:=false start_vision:=false start_grasp:=false \
  start_conveyor:=false start_radar:=true
````

````text
ros2 topic echo /eai_factory/safety/status      # 初始 safe
ros2 topic echo /eai_factory/safety/danger
ros2 topic echo /car1_base_driver/bus_servo/set_state
````

````text
ros2 launch eai_factory eai_factory.launch.py start_viz:=false
ros2 service call /eai_factory/start_sort std_srvs/srv/Trigger
# 机械臂运动中，雷达正前方放障碍 ≥0.5s
````

````text
ros2 topic pub -1 /car1_base_driver/bus_servo/set_state car1_base_driver_msgs/msg/SetBusServoState \
  "{state: [{target_id: [0,1,2,3,4,5], enable_torque: [0,0,0,0,0,0]}], duration: 0.2}"
# 释放后务必恢复：
ros2 topic pub -1 /car1_base_driver/bus_servo/set_state car1_base_driver_msgs/msg/SetBusServoState \
  "{state: [{target_id: [0,1,2,3,4,5], enable_torque: [1,1,1,1,1,1]}], duration: 0.2}"
````

## EXP-09 · 全功能快速验证 — 智慧工厂装货分拣码垛综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006-smart-factory/exp-09-full-verification/index.html)

````text
ls /home/iflytek/models/rkllm/Qwen3.5-0.8B_w4a16_g128_rk3576.rkllm
/home/iflytek/ros2_venv/bin/python3 --version     # 3.10
sudo bash scripts/setup_conveyor_gpio.sh          # GPIO 幂等导出
````

````text
source /opt/ros/humble/setup.bash
source /home/iflytek/ros2_ws/install/setup.bash
# 若报找不到包：colcon build --packages-select eai_factory
````

````text
source /opt/ros/humble/setup.bash
source /home/iflytek/ros2_ws/install/setup.bash
ros2 launch eai_factory eai_factory.launch.py
# 观察：bringup → arm_controller → peripherals → voice(LLM) →
#      scheduler/safety/vision/grasp/conveyor → 约5s 后 rqt 画面
ros2 node list | grep eai_factory
ros2 service list | grep -E "start_(load|sort|palletize)"
````

````text
bash scripts/stop_eai_factory.sh
# 结尾应显示"已全部清理干净"
ros2 node list                 # 无 eai_factory 节点
ss -tlnp | grep 8080           # 无输出
ps aux | grep -E "voice_node|flask_server|eai_factory" | grep -v grep
````

## EXP-01 · 需求分析（R1–R12） — 老人跌倒监护综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006a-elderly-companion/exp-01-requirements/index.html)

````text
# ① 进入工程目录
cd ~/ros2_ws/src/xflz_eai_car1/app/elderly_companion

# ② 确认需求文档与配置存在（后续实验反复用到）
ls docs/ config/fall_config.yaml

# ③ state 段关键默认值 —— 三条核心时序的时间参数来源（config L57-L62 附近）
#   angle_thresh: 50.0      躯干角阈值（度）   → 判定需求 R4
#   aspect_thresh: 0.5      宽高比阈值        → 判定需求 R4
#   falling_window: 1.5     跌倒窗口（秒）     → 检测确认时序
#   lying_time: 3.0         躺姿持续（秒）     → 确认后才询问
#   ask_timeout: 12.0       询问超时（秒）     → 超时升级告警 R5
#   cooldown: 60.0          告警冷却（秒）     → 防重复轰炸
````

## EXP-02 · 分层系统架构 — 老人跌倒监护综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006a-elderly-companion/exp-02-architecture/index.html)

````text
# ① 进入工程并自检
cd ~/ros2_ws/src/xflz_eai_car1/app/elderly_companion
python3 common/env_check.py --config config/fall_config.yaml
# 预期：[env] 自检通过（缺项时逐条 MISSING，EXIT 2）

# ② 定位三大接口（行号应与本课第 03 节一致）
grep -n "def process_frame\|def has_ask_session\|def on_kws" fall/monitor.py
# 247: def process_frame(...)      每帧入口
# 152: def has_ask_session(...)    询问会话只读查询
# 168: def on_kws(...)             硬件/KWS 唤醒回调

# ③ 定位语音状态机宿主与文本入口
grep -n "def run\|def feed_text" voice_loop.py
# 224: def run(...)     交互主循环
# 154: def feed_text(...)  文本通道 → 同一状态机（R12）

# ④ 文本通道跑最小闭环（无需麦克风/摄像头）
python3 apps/speech_text_demo.py
# 依次输入：小飞小飞 → （系统询问后）没事 / 救命
````

## EXP-03 · fall_config.yaml 配置详解 — 老人跌倒监护综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006a-elderly-companion/exp-03-config/index.html)

````text
# ① 进入工程并运行环境自检（只读探测，不会自动安装任何东西）
cd ~/ros2_ws/src/xflz_eai_car1/app/elderly_companion
python3 common/env_check.py --config config/fall_config.yaml
# 预期逐条输出：[env] OK numpy 1.24.3 ... 直到 [env] 自检通过（EXIT 0）
# 探测 7 库：numpy / cv2 / scipy / rknnlite / sherpa_onnx / serial / yaml
# 同时探测模型文件与串口设备；缺项输出 [env] MISSING /dev/ttyUSB0（EXIT 2）

# ② schema 报错演示：把 detect.model_path 拼成 detect.model_pth 后启动
python3 apps/fall_video_demo.py --video testdata/fall_slow_x264.mp4 --text
# 预期：config error: missing key  → 立刻改回（必填轨道，绝不带病启动）

# ③ CLI 覆盖实验：临时值生效，yaml 保持原样
python3 apps/fall_video_demo.py --video testdata/fall_slow_x264.mp4 --text --score-thr 0.4 --log-level DEBUG
# 优先级：--score-thr 0.4 > yaml 的 score_thr: 0.25 > schema 默认
````

## EXP-04 · 语音交互子系统 — 老人跌倒监护综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006a-elderly-companion/exp-04-voice/index.html)

````text
# ① 文本模式状态机验证（无需任何硬件，stdin 即"音频帧"）
cd /home/iflytek/ros2_ws/src/xflz_eai_car1/app/elderly_companion
python3 apps/speech_text_demo.py
# 输入: 小飞小飞 → 唤醒; 你好 → LLM对话; ask → 模拟跌倒; 救命 → 告警; exit → 退出
# 预期日志: [文本] 唤醒 → 开始聆听（mode=DIALOG） → >>> ASK 确认（ALERT）

# ② 三个子系统单点测试（记录模型参数与实时率）
python3 apps/speech_kws_test.py   # KWS 就绪（5 词）: 小飞同学/你好小飞/小飞小飞/救命/帮帮我
python3 apps/speech_asr_test.py   # 逐 wav 输出 partial + 实时率（0.1x~0.3x）
python3 apps/speech_tts_test.py   # 合成耗时 2.03s / RTF 0.39（<1 即实时）

# ③ 全链路集成（--text 不做语音播报，验证状态机 + 打断 + 超时）
python3 apps/fall_video_demo.py --video testdata/fall_slow_x264.mp4 --text
# 有硬件时: 换 --mjpeg，真人对六麦说"小飞小飞"验证提示音+录音线程启动

# ④ 清理：ALSA 残留 + LLM 服务
pkill -9 aplay; pkill -f llm_server
````

## EXP-05 · 人体检测与姿态估计 — 老人跌倒监护综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006a-elderly-companion/exp-05-pose-detection/index.html)

````text
# ① 单图行人检测（预期: 76ms / 13.1fps / score 0.837）
cd ~/ros2_ws/src/xflz_eai_car1/app/elderly_companion
python3 apps/detect_demo.py --input testdata/stand_up.jpg --save
# OK: 1 detections in 76 ms → cls=0(person) box=(31.2,7.8,183.2,393.5)
# 标注图存 runs/out/detect_image.jpg；无人图返回 0 框不崩溃

# ② 单图姿态（默认 model.pose_type: rtmpose → 17 keypoints）
python3 apps/pose_demo.py --input testdata/fall_1.png --save
grep -A2 "pose_type" config/fall_config.yaml   # 确认当前后端

# ③ 切 blazepose 对比 33 点，完成后必须改回 rtmpose
sed -i 's/pose_type: rtmpose/pose_type: blazepose/' config/fall_config.yaml
python3 apps/pose_demo.py --input testdata/fall_1.png --save
sed -i 's/pose_type: blazepose/pose_type: rtmpose/' config/fall_config.yaml

# ④ 视频集成 + 文本叠加 + 可选 MJPEG 预览（浏览器 :8088/fall.mjpg）
python3 apps/fall_video_demo.py --video testdata/fall_slow_x264.mp4 --text --mjpeg
# 逐帧: [frame N] person_id=1 box=... pose: 17 keypoints → IDLE→FALLING→LYING

# ⑤ 阈值调参（CLI 覆盖不落盘，记录三档检出数）
python3 apps/detect_demo.py --input testdata/stand_up.jpg --save --score-thr 0.3
python3 apps/detect_demo.py --input testdata/stand_up.jpg --save --score-thr 0.7
# 图片模式等价性: python3 apps/fall_app.py --mode image --input testdata/fall_1.png --save
````

## EXP-01 · 需求分析（R1-R13） — 通用目标追踪综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006b-object-tracking/exp-01-requirements/index.html)

````text
# 进入源码目录（后续所有实验统一起点）
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking

# 先看目录结构，确认 docs 十份文档就位
ls docs/
# 01_需求总览.md  02_架构设计.md  03_开发模式与验证.md  04_配置方案.md  05_流程与安全.md ...
````

````text
# ① 一次抓全 13 条编号需求（行号即锚点）
grep -n "R[0-9]" docs/01_需求总览.md

# ② 圈出全部安全条款：门控 / fail-closed / 清除 / 防呆
grep -n "门控\|fail-closed\|清除\|防呆" docs/05_流程与安全.md

# ③ README 架构速览前 60 行，对上七层模块自述
head -60 README.md

# ④ 顺手确认核心词在工程里的真实出场位置
grep -rn "clear_words\|cell_cap\|min_confidence" config/ docs/ | head -20
````

## EXP-02 · TrackEngine 单状态机架构 — 通用目标追踪综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006b-object-tracking/exp-02-track-engine/index.html)

````text
# 进入工程，确认修改时间与你要读的文件同批
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking
ls apps/ tracking/ session/ 2>/dev/null || ls apps/ tracking/

# 准备一份离线测试视频（讲义所用样例）
# /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4
````

````text
# ① 五处架构锚点一次抓齐（源讲义原命令）
grep -n "class CommandWorker\|_native_gate\|submit_effect\|pending\|epoch" apps/track_app.py

# ② 主循环五步：打开 run 看 L89-93，发布在 L63
grep -n "def run\|_drain_commands\|_compute_draw\|_publish" tracking/engine.py

# ③ 文本模式实测一条指令（离线样例视频）
python3 apps/track_app.py --video /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4 --text
# 终端敲：追踪 id3
# [cmd] track_ids {3}   → 回复"好的，只追踪 id3"，主循环第③步 apply

# ④ session 两个状态入口
grep -n "set_follow\|visible_ids" tracking/session.py   # L12-25
````

## EXP-03 · track_config.yaml 配置详解 — 通用目标追踪综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006b-object-tracking/exp-03-config/index.html)

````text
# ① 备份永远是第一步（改坏随时还原）
cp config/track_config.yaml /tmp/tc.bak

# ② 先跑一次正向校验，确认当前文件是好的
python3 -c "from common.config import load_config; from config.config_schema import REQUIRED; load_config('config/track_config.yaml', REQUIRED); print('OK')"

# ③ 快速浏览顶层键
grep -n "^[a-z_]*:" config/track_config.yaml
````

````text
# 改错①：类型错 —— sed 精确替换（注意行尾空格，源讲义原文）
sed -i 's/min_confidence: 0.3 /min_confidence: 0.3x /' config/track_config.yaml
python3 -c "from common.config import load_config; from config.config_schema import REQUIRED; load_config('config/track_config.yaml', REQUIRED)"
# ConfigError: tracker.min_confidence: expected float, got str
# Traceback: common/config.py L37 → L49 → _validate 逐键核对类型

# 改错②：int 段类型错
# detect.imgsz: 640 → 640x  →  ConfigError: expected int, got str

# 改错③：删 REQUIRED 必填键 tracking.pause_wait（用编辑器删除该行）
# →  ConfigError: missing key: tracking.pause_wait

# 还原，再做语义实验：class_filter 扩成空（放行 COCO 全部 80 类）
sed -i 's/class_filter: \[car\]/class_filter: []/' config/track_config.yaml
python3 apps/track_demo.py --video /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4 --text
# 现象：人/物全进追踪 → 一辆车被切成多个框竞争 → ID 急速增长

# 收工必还原
cp /tmp/tc.bak config/track_config.yaml
````

## EXP-04 · DeepSORT 追踪底座 — 通用目标追踪综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006b-object-tracking/exp-04-deepsort-tracking/index.html)

````text
← 依次点按：先感受纯 IoU 判定，再看遮挡场景下 ReID 如何救场
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking
python3 -m pytest tests/ -v
platform linux -- Python 3.10.12, pytest-6.2.5 ...
rootdir: /home/iflytek/ros2_ws/src/xflz_eai_car1/app/object_tracking
collected 97 items
tests/test_tracker_session.py::... PASSED    # ID 分配 / follow 过滤 / 显隐
tests/test_reid_tracking.py::... PASSED     # use_reid=true 遮挡保原 ID vs false 新建
tests/test_reid_crop_heal.py::... PASSED    # 越界 crop 不喂错特征 / extract 超时零哨兵
tests/test_pipeline_tracker.py::... PASSED  # min_box 剔除 / stale 超龄剔除
tests/test_native_wiring.py::... PASSED     # 门控路由（EXP-05 复用）
# 全部 PASSED 即通过——这是后续所有实验的行为基线
````

````text
cp config/track_config.yaml /tmp/tc.bak
sed -i 's/min_confidence: 0.3 /min_confidence: 0.1 /' config/track_config.yaml
python3 apps/track_demo.py --video /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4 --text --list --mjpeg
# 现象：低置信噪声进 tracker，大量 Tentative 新建 → ID 频繁跳变
cp /tmp/tc.bak config/track_config.yaml   # 恢复 0.3，验证 ID 恢复稳定
````

````text
printf '追踪 id3\n' | python3 apps/track_demo.py --video /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4 --text --list --mjpeg
[命令] 追踪 id3
当前目标： （当前无确认目标）
id3 已消失
全部跟随目标已消失，恢复全显示
# --list 每帧打印 scene_summary；锁定 → 仅绘该 ID；丢失 → lost + unlock_on_loss 恢复全显示
````

## EXP-05 · LLM 指令门控（FunctionCall） — 通用目标追踪综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006b-object-tracking/exp-05-llm-gating/index.html)

````text
# config/track_config.yaml L76 起（interaction 词表，讲义 5.6 JSON）
interaction:
  clear_words:   [清除, 解除, 取消, 停, 不追, 恢复全, 不跟]
  track_words:   [追踪, 追, 锁定, 识别, 找, 看, 跟, 停]
  instruct_words:[追踪, 追, 停, 取消, 解除, 不追, 恢复全, 不跟, 清除]
  color_words:   [红, 蓝, 白, 黑, 绿, 黄, 灰, 粉, 紫]
  desc_words:    [红, 黄, 蓝, 白, 黑, 绿, 灰, 粉, 紫, 卡车, 轿车, suv, 大, 小, 新, 旧]
````

````text
# ① 快速回归（跳过真模型用例，x86 可跑）
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking
python3 -m pytest tests/ -q -k "not test_real"

# ② 查看门控与兜底的源码锚点
grep -n "NATIVE_TOOLS\|_native_gate\|_rule_fallback" apps/track_app.py
grep -n "clear_words" config/track_config.yaml   # L76 词表
````

````text
# 工作目录（object_tracking 模块）
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking

# 1. 纯逻辑门控用例（x86，无真实 LLM）
python3 -m pytest tests/ -q -k "not test_real"   # 覆盖 _native_gate/test_native_wiring/闲聊

# 2. 文本全链路（LLM FunctionCall + 门控）——依次在提示符输入三句
python3 apps/track_app.py --video /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4 --text --mjpeg
小飞小飞
追踪id3
# 期望：[cmd] track_ids {3} → 播报「好的，只追踪 id3」
清除
# 期望：[cmd] 清除语义指令，执行 clear_track → 播报「好的，已解除全部锁定」
你好
# 期望：[LLM] 自然回复，[cmd] 不存在——闲聊回落干净请求，无工具执行

# 3. 规则兜底演练：杀 LLM 后重跑第 2 条命令
pkill -f llm_server
# 「追踪 id3」→ parse_instruction L45-68 接管照常锁定；「你好」→ _rule_fallback 回「没听懂」（设计预期）

# 4. 实验收尾：清理常驻子进程
pkill -f llm_server
````

## EXP-06 · VLM 描述选目标 — 通用目标追踪综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006b-object-tracking/exp-06-vlm-selection/index.html)

````text
# ① 测试（假件链，不依赖真模型）
python3 -m pytest tests/ -q -k "vlm"

# ② 启动文本模式（llm_server 由 ensure_running 自动拉起）
python3 apps/track_app.py \
  --video /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4 \
  --text --mjpeg

# ③ 配置确认（fake_result_id 位置）
grep -n "fake_result_id" config/track_config.yaml   # L33 默认 3
````

````text
# ① 抽帧切网格（frame-step 200 → 第 200/400 帧）
python3 apps/dump_track_grids.py \
  --video /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4 \
  --frame-step 200
→ runs/vlm_frames/grid_f200_rx2c2.jpg / .json（含 cell_cap、order、cells、448×448）

# ② 指定颜色选格
python3 apps/test_vlm_select.py \
  --image runs/vlm_frames/grid_f200_rx2c2.jpg \
  --json  runs/vlm_frames/grid_f200_rx2c2.json --color 红
frame200 n=4 ids=[29,30,32,33] grid 2×2 → PROMPT → 答 '3'（5s）→ cell_ids[2]=32
frame400 n=11 → 4×4 网格（仍 ≤ cell_cap=16）

# ③ 完整链路成功日志（--text 模式逐句验证）
[cmd] 追踪红色小轿车 → native tool=track_vlm
[cmd] track_ids 无 id 但含描述，改 VLM 选车
[track_vlm] dump 带标签网格 → [track_vlm] 锁定目标 ID 24

# ④ 失败链路（如「追踪火车」画面中不存在）
[cmd] track_ids 无可用 id，回引导 —— 不锁定任何 ID

# ⑤ 行号锚点
grep -n "def build_grid\|def _pick_grid\|def _parse_tile" vision/.../vlm_real.py
grep -n "_vlm_none" apps/track_app.py        # L402-405
grep -n "cell_cap" apps/track_app.py config/track_config.yaml
````

## EXP-07 · 语音交互（流式 TTS 打断） — 通用目标追踪综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006b-object-tracking/exp-07-voice-interrupt/index.html)

````text
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking
pkill -9 aplay; pkill -f llm_server   # 清播放设备与 8080 端口
grep -n "kws.enabled\|threshold\|hardware_wakeup" config/track_config.yaml   # 确认唤醒配置
````

````text
← 按唤醒 → 说话 → 定稿 → 播报的顺序点击；再到 SPEAKING 时点"新唤醒"看 epoch 打断
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking
python3 apps/track_app.py --video /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4 --text
小飞小飞   → [文本] 唤醒 → 开始聆听（mode=DIALOG）
追踪红色皮卡 → LLM →（描述无 id）→ VLM 选车 ；你好 → 闲聊 ；清除 → 解除
# _merge_args L74-93：--text 置 speech.text_mode=True，关 kws.enabled / hardware_wakeup.enabled
````

````text
pkill -9 aplay; pkill -f llm_server
# 另终端起相机：
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch car1_peripherals depth_camera.launch.py
python3 apps/track_app.py --camera --mjpeg     # 喊 小飞小飞 → 检测到唤醒 → 录音线程: plughw:4,0

# 报"XFM 串口无响应"时按序排查：
sleep 120 && grep -n "XFM\|xfm_wake" runs/logs/track_app.log   # ① 等冷启动自愈
python /home/iflytek/ros2_ws/src/xflz_eai_car1/interaction/car1_speech/tools/xf_mic_array.py   # ② 独立探测固件
pkill -f log_server; ls -l /dev/ttyACM0         # ③ 停固件 log 占用（VID:PID=1a86:55d4）
````

````text
[ASR 结果] 追踪红色皮卡 → [cmd] track_vlm（描述带色无 id → VLM 网格选车）
[tts] 合成第 1 句 → 立即 aplay 播出"好的"（不等整段合成）   # _SENTENCE_RE 切句、max_num_sentences=1
# 播报中再喊"小飞小飞"：_interrupt L398-404 → epoch+=1 → 杀 aplay 清队列
[track_app] epoch 已变，丢弃迟到指令            # worker 在 apply 副作用前校验（L256-259）
````

````text
cp config/track_config.yaml /tmp/tc.bak
sed -i 's/kws.threshold: 0.25/kws.threshold: 0.4/' config/track_config.yaml
python3 apps/track_app.py --camera --mjpeg      # 固定场景重跑，统计误报/漏报
grep -n "decoding_method\|hotwords" config/track_config.yaml   # greedy_search 下热词不生效！
cp /tmp/tc.bak config/track_config.yaml         # 实验完还原
````

## EXP-08 · 事件日志与故障降级 — 通用目标追踪综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006b-object-tracking/exp-08-events-degrade/index.html)

````text
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking
> runs/logs/events.jsonl            # 清空便于判定新增（log.dir 可配置）
tail -f runs/logs/events.jsonl     # 另终端常开，实时观察追加
````

````text
← 点击左侧任一故障/事件：观察降级路径、events.jsonl 新增行与副作用结论
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking
python3 apps/track_app.py --video /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4 --text --mjpeg
小飞小飞 → 追踪 id3 → 待目标出画 → 清除
# 另终端：
tail -f runs/logs/events.jsonl
{"t": ..., "kind": "command", "ids": [3], ...}       # 锁定留痕
{"t": ..., "kind": "lost", "ids": [3]}             # 差分 prev_ids-cur_ids，TTS"id3 已消失"
{"t": ..., "kind": "unlock_all", "ids": [3]}       # 全清才写；部分丢失不写
````

````text
追踪一百号目标 → track_ids 数值不对齐 {100} → [LLM] 未找到该 ID 目标   # E1：不锁、不走 VLM、不 grid dump
追踪蓝色皮卡（无蓝车）→ [track_vlm] 无法识别 → 回引导                # E2：_vlm_none，不锁
清除 → [track_app] 清除语义指令，执行 clear_track                     # E3：clear_words 拦截短路
追踪 id999 → 未找到该 ID 目标                                            # E5：不存在的 id
# E6：连发指令撑满队列 → put_nowait 满丢最旧（q maxsize=8，L220/L247-250）
````

````text
pkill -f llm_server        # 或 config 关 llm.enabled=false 重跑
python3 apps/track_app.py --video .../test.mp4 --text --mjpeg
小飞小飞 → 追踪 id3
[track_app] chat fallback failed → _rule_fallback L620-628：parse_instruction 直解 → 追踪/清除仍可用
[LLM] 没听懂…              # 闲聊无规则可兜——设计预期不是缺陷
# ReID 全 worker 崩：extract get(timeout=10) 超时 → 全零特征
# pipeline L44-52：features 长度≠dets 数 → 归零 → tracker.update(dets, features=None) 纯 IoU，不断流
````

````text
python3 apps/track_app.py --camera --mjpeg
# 注入① 拔相机线：连续 30 次读帧失败（_cam_fail>=30，engine L115-122）
{"t": ..., "kind": "camera_stall"} + 暂停 1s 周期重连（其余失败仅 sleep 0.2s）
{"t": ..., "kind": "pause_timeout"}  # LLM 挂死期间 pending 超 30.0s → 清 pending 解冻（L101-107）
# 注入② 另终端杀播放器看降级：
kill -9 $(pgrep aplay)   → _restart_playback 重启；持续失败置 _failed → 打印降级，状态机不卡 SPEAKING
tail -f runs/logs/events.jsonl   # 收尾：核对本轮产生的各 kind 是否齐全
````

## EXP-09 · 全功能验证 — 通用目标追踪综合项目

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp006b-object-tracking/exp-09-full-verify/index.html)

````text
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking
ps aux | grep -E "track_app|llm_server|aplay" | grep -v grep   # 基线：应无残留
# 若有残留：pkill -f llm_server; pkill -9 aplay 后再进实验
````

````text
← 从 Step1 开始按序点亮；验收灯 0/5，当前未开始
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/app/object_tracking
python3 -m pytest tests/ -v
# 覆盖 tools/ 门控 / VLM 虚测(vlm_fake 固定 fake_result_id=3) / 单帧 pipeline / 录音自愈
# 全绿才继续；实机出问题时也先复跑本回归，把框架缺陷与环境问题分开
````

````text
python3 apps/track_app.py --video /home/iflytek/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking/test.mp4 --text
小飞小飞          # stdin 首行唤醒词才进 DIALOG（--text 已关 KWS/硬件唤醒）
追踪 id3          # 观察：唤醒 → 追踪锁定 → 出画 lost → 清除 unlock_all
# 核对 runs/logs/events.jsonl：command/lost/unlock_all 条目齐全；--text 走 feed_text 同一回调链
````

````text
# 另一终端提供相机话题：
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 launch car1_peripherals depth_camera.launch.py
# 主终端（--save 落盘标注帧，frames:1920 fps:10）：
python3 apps/track_app.py --camera --mjpeg --save
浏览器打开 http://<IP>:8088/track.mjpg   # 持续 MJPEG 流，ID 稳定由 min_confidence=0.3 + [car] 双闸压噪
小飞小飞 → 追踪红色皮卡 → 解除         # 语音链路闭环；headless 下 display.show 默认关，别强开 --show
````

````text
# 分层排查锚点：[diag] 录音 50 帧/5s（音频层）· [ASR 实时]（识别层）· [cmd] 逐分支（指令层）· jsonl（状态层）
#   pause_timeout → 查 LLM（pending 超 30s）；camera_stall → 查相机（连续 30 次读帧失败）
tail -f runs/logs/events.jsonl   # 收尾：状态变化齐全；runs/out 标注帧带 follow 过滤不空
pkill -f llm_server; pkill -9 aplay   # 独占资源：8080 / 播放设备 / 串口必须释放
ps aux | grep -E "track_app|llm_server|aplay" | grep -v grep   # 应无残留；回滚 git revert 前同样先 pkill
````

## 实验 01 · 激光 SLAM 建图与导航原理

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp007-intelligent-factory/exp-01-slam-nav/index.html)

````text
# 一键启动建图（SLAM Toolbox）
ros2 launch slam_toolbox online_async_launch.py \
    use_sim_time=false resolution:=0.05

# 观察数据流：谁在发布？谁在订阅？
ros2 topic list            # /scan  /odom  /map  /tf
ros2 topic hz /scan        # 应稳定在 ~10Hz
ros2 run tf2_ros tf2_echo map base_link   # 实时查看位姿
````

## EXP-01 · 沙盘环境认知与建图导航 | CASE-05 机器人小车智慧交通综合案例

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp008-smart-traffic/exp-01-sandbox-mapping/index.html)

````text
# ① 预检：雷达与底盘话题在线
ros2 topic echo /scan --once
ros2 topic list | grep -E "scan|cmd_vel"

# ② 启动激光 SLAM 建图（终端1）+ RViz2 观察（终端2）
ros2 launch car1_slam slam.launch.py
rviz2 -d ~/car1_ws/src/car1_slam/rviz/slam.rviz

# ③ 遥控遍历沙盘闭环（低速起步！）+ 查看 TF 树确认 frame 完整
ros2 run teleop_twist_keyboard teleop_twist_keyboard
ros2 run tf2_tools view_frames

# ④ 保存地图为 slam_map（遍历闭环并回到起点后执行）
ros2 run nav2_map_server map_saver_cli -f slam_map

# ⑤ 加载 slam_map 启动 Nav2 + AMCL 定位
ros2 launch car1_navigation navigation.launch.py map:=slam_map

# ⑥ RViz2 设初始位姿（2D Pose Estimate）后再下发目标点
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 1.2, y: 0.8}}}}"

# ⑦ 监控定位质量：粒子云应收敛成一撮，TF 持续刷新
ros2 topic echo /amcl_pose --once
ros2 run tf2_ros tf2_echo map base_link
````

## EXP-02 · 交通标志与红绿灯识别 | CASE-05 机器人小车智慧交通综合案例

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp008-smart-traffic/exp-02-traffic-vision/index.html)

````text
# ① 启动 HP60C 相机并核对话题树（EXP-01 用不到相机，本实验起全程在线）
ros2 launch ascamera hp60c.launch.py
ros2 topic list | grep ascamera_hp60c
ros2 topic hz /ascamera_hp60c/color_image        # ≈30Hz

# ② 截帧做 HSV 标定（三态灯各一张）+ 运行灯态识别节点
ros2 run image_tools cam2image --ros-args -p output_image:=/tmp/red.png
python3 src/light_node.py --config config/hsv_config.yaml
ros2 topic echo /traffic/light                   # 期望 R / Y / G 稳定输出

# ③ 手动发布一条灯态供 EXP-03 联调（无相机时的替身）
ros2 topic pub --once /traffic/light std_msgs/String "{data: 'R'}"

# ④ YOLO 训练→NPU 部署（PC 训练，C1 推理）
yolo train model=yolov8n.yaml data=traffic_sign.yaml epochs=120 imgsz=640
python3 src/export_rknn.py --model best.pt --quantize int8 --platform rk3588
python3 src/app.py --model models/traffic_sign.rknn --topic /ascamera_hp60c/color_image

# ⑤ 观察检测输出 / 端到端延迟
ros2 topic echo /traffic/sign                    # {cls, conf, dist}
ros2 topic hz /traffic/sign                      # ≥15Hz
````

## EXP-03 · 车道保持与路口信号通行 | CASE-05 机器人小车智慧交通综合案例

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp008-smart-traffic/exp-03-lane-signal/index.html)

````text
# ① 起相机（EXP-02 驱动复用）+ 灯态识别节点
ros2 launch ascamera hp60c.launch.py
python3 src/light_node.py --config config/hsv_config.yaml

# ② 启动巡线 + 路口状态机节点（参数包 lane_follow.yaml）
ros2 run car1_control lane_follow_node.py --ros-args \
  --params-file src/car1_control/config/lane_follow.yaml
ros2 topic echo /cmd_vel                       # 看 v/ω 是否平滑
ros2 topic echo /traffic/light                 # R/Y/G 联动输入

# ③ 手动微调 PID（运行中热改，改完立刻跑 5m 直道）
ros2 param set /lane_follow_node kp 0.60
ros2 param set /lane_follow_node kd 0.15
ros2 param set /lane_follow_node ki 0.02

# ④ 无灯时手动注入灯态，联调状态机
ros2 topic pub -r 1 /traffic/light std_msgs/String "{data: 'R'}"

# ⑤ 联调录包回放：迁移日志 + TF + 偏差话题全录
ros2 bag record -a -o lane_interface_bag
ros2 bag play lane_interface_bag
ros2 run tf2_tools view_frames                 # out/frames.pdf 查 TF 完整性
````

## EXP-04 · 智慧泊车与综合路测 | CASE-05 机器人小车智慧交通综合案例

[来源](https://aibittek.github.io/course_generate_agent/courses/cases/exp008-smart-traffic/exp-04-smart-parking/index.html)

````text
# ① 起相机 + 车位检测节点（标线参数包 slot_vision.yaml）
ros2 launch ascamera hp60c.launch.py
python3 src/parking/perception.py --config config/slot_vision.yaml
ros2 topic echo /parking/slot                  # type/corners/available

# ② 目标位姿换算（map→车体变换，核对 TF 链）
ros2 run tf2_ros tf2_echo map base_link
ros2 run tf2_tools view_frames                 # out/frames.pdf 查 map→odom→base_link

# ③-A 几何轨迹入库（两段圆弧，开环 /cmd_vel）
ros2 run car1_parking arc_park_node.py --ros-args \
  --params-file src/car1_parking/config/arc_park.yaml
ros2 topic echo /parking/status                # IDLE/PLANNING/DONE + 实时侧隙

# ③-B Nav2 精点入库（先在 RViz2 用 2D Pose Estimate 设初始位姿！）
ros2 launch car1_navigation navigation.launch.py map:=slam_map
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: map}, pose: {position: {x: 3.42, y: -1.08}, orientation: {z: 0.0, w: 1.0}}}}"

# ④ 全流程任务节点（六阶段计分）+ car1_speech 语音播报
ros2 launch car1_speech speech.launch.py
ros2 run car1_mission route_mission.py --ros-args -p mode:=full
ros2 topic pub --once /speech/tts std_msgs/String "{data: '任务完成，总分95，泊车满分'}"
ros2 bag record -a -o smart_route_bag          # 全程留痕回放复盘
````

## EXP-01 · 颜色识别（HSV 阈值）— 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-01-color-hsv/index.html)

````text
colors:
  - name: 红色
    h: [0, 15]        # 第一段：0~15
    s: [100, 255]
    v: [80, 255]
    h2: [160, 180]    # ← 第二段！红色跨了 0 的边界
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/03_lab_color_recognition
cat config/lab_config.yaml
````

````text
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/03_lab_color_recognition
python src/app.py --ros
````

````text
python src/app.py --tune 红色 --ros
````

````text
- name: 黄色
  h: [20, 35]
  s: [100, 255]
  v: [100, 255]
````

````text
# 概念示意
err  = (cx_ball - W/2) / (W/2)   # 归一化到 [-1, 1]
angular = -0.8 * err             # 目标在右 → 右转 → 角速度为负（ROS 约定逆时针为正）
linear  =  0.2 if abs(err) < 0.15 else 0.05   # 转得急就先慢一点
````

## EXP-02 · 二维码生成与识别 — 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-02-qrcode/index.html)

````text
# 配置里的生成参数
generate:
  version: 1              # 1-40，fit=True 时自动扩版
  error_correction: M     # L/M/Q/H — 本板实测别用 H
  box_size: 5             # 每格 5 像素
  border: 4               # 白边 4 格

detect:
  scale: 0.5              # 检测前先缩一半，提速
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/04_qrcode
python src/app.py --generate "HELLO-ROBOT-001" --save my_qr.png
````

````text
python src/app.py --detect --image my_qr.png --save out.png
````

````text
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/04_qrcode
python src/app.py --detect --ros
````

````text
nano config/qrcode_config.yaml
````

````text
generate:
  error_correction: M   ← 改成 L / Q / H 各试一次
  box_size: 5
  border: 4
````

````text
python src/app.py --generate "HELLO-ROBOT-001" --save qr_L.png
````

## EXP-03 · 多边形轮廓拟合 — 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-03-contour-polygon/index.html)

````text
# 关键配置（config/line_config.yaml）
preprocess:
  threshold_mode: adaptive    # binary | adaptive
  threshold_value: 50
  invert: false               # ← 亮物暗底用 false，暗物亮底改 true
  morph_kernel: 3

contour:
  min_area: 100               # 小于这个面积的轮廓直接丢掉（滤噪）

polygon:
  approx_epsilon: 0.02       # ← 最该调的那个数
  use_hull: true              # 先凸包再逼近
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/05_line_following
cat config/line_config.yaml
````

````text
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/05_line_following
python src/app.py --ros
````

````text
nano config/line_config.yaml
# 把 invert: false 改成 true
````

````text
圆度 = 4π × 面积 / 周长²
````

````text
offset = (质心x - 画面中心x) / (画面宽/2)   # 归一化到 -1 ~ +1

角速度 = -k × offset       # 正偏移（偏右）→ 负角速度（左转）
````

## EXP-04 工业表面缺陷检测 — 机器人视觉应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-04-industrial-defect/index.html)

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/01_industrial_defect_detection
ls models/
ls images/
````

````text
python src/app.py --model models/defect_yolov8n_int8.rknn \
       --image images/1.jpg --save out.jpg
````

````text
# 终端 1：相机驱动
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
# 终端 2：实时检测
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
cd ~/ros2_ws/src/xflz_eai_car1/vision/01_industrial_defect_detection
python src/app.py --model models/defect_yolov8n_int8.rknn --ros
````

````text
http://<板子IP>:8080/defect.mjpg
````

````text
nano config/defect_config.yaml
# 找到 infer 这一段
````

````text
infer:
  imgsz: 640
  score_thr: 0.25   ← 改这个，试试 0.05 和 0.60
  iou_thr: 0.45      ← 或者改这个，试试 0.30 和 0.70
````

````text
python src/eval_map.py --backend onnx   # 浮点基准
python src/eval_map.py --backend rknn   # int8 实测
````

````text
scratches: 12 处
inclusion: 3 处
patches: 1 处
````

## EXP-05 · 垃圾分类（检测头 20 类）— 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-05-garbage-classification/index.html)

````text
model:
  category_names:
    "1": recyclable        # 可回收物
    "2": kitchen_waste     # 厨余垃圾
    "3": hazardous_waste   # 有害垃圾
    "4": other_waste       # 其他垃圾
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/02_garbage_classification
ls models/ config/ images/
head config/labels.txt
head config/category_map.txt
````

````text
python src/app.py --image images/1.jpg --save out.jpg
````

````text
# 终端 1
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
# 终端 2：先单线程跑一遍
cd ~/ros2_ws/src/xflz_eai_car1/vision/02_garbage_classification
python src/app.py --ros --tpes 1
````

````text
python src/app.py --ros --tpes 3
````

````text
nano config/garbage_config.yaml
````

````text
infer:
  score_thr: 0.25    ← 改这个：0.10 更容易报，0.55 更保守
  nms_thr: 0.45

pool:
  tpes: 2             ← 改这个：1 / 2 / 3，看帧率差多少
  core_mask: auto     ← 别动！这块板子上改成 round_robin 会卡死
````

## EXP-06 · 语义分割与背景替换 — 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-06-semantic-segmentation/index.html)

````text
# 关键配置（config/seg_config.yaml）
model:
  rknn: models/yolov8n-seg.rknn
  classes: [0]          # 0=person；空列表 = 全部 80 类
  mask_thr: 0.5         # 蒙版二值化阈值

background:
  mode: black           # black | solid | image
  color: [0, 0, 0]        # solid 模式的颜色（BGR 顺序！）
  image: ""

pool:
  workers: 3              # 多线程推理，板上有几个核就开几个
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/06_semantic_segmentation
ls models/ images/
python src/app.py --image models/bus.jpg --save out.jpg
````

````text
nano config/seg_config.yaml
````

````text
background:
  mode: black       # ① 全黑，最容易看出抠图边缘
  mode: solid       # ② 纯色，颜色在 color 里改（BGR）
  color: [0, 200, 0]  #    BGR → 这是绿色
  mode: image       # ③ 换成一张自己的图
  image: "images/bg.jpg"
````

````text
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/06_semantic_segmentation
python src/app.py --ros
````

````text
背景部分 = cv2.GaussianBlur(原图, (25,25), 0)
结果 = 人物区域用原图 + 背景区域用模糊图
````

````text
BGRA = 原图 + alpha 通道（蒙版）
cv2.imwrite('cutout.png', BGRA)
````

## EXP-07 · 多目标跟踪（DeepSORT）— 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-07-deepsort-tracking/index.html)

````text
# 概念示意，不是真实代码
代价 = 0.2 × 位置距离  +  0.8 × 外观距离
#     ↑ 权重小，因为人会突然转弯   ↑ 权重大，因为衣服不会突然换
````

````text
# 关键配置（config/track_config.yaml）
detect:
  classes: [2]          # 2=car；改成 [0] 就跟踪人；[] = 全部
  score_thr: 0.25

tracker:
  mode: deepsort         # deepsort | sort（切 sort 可对照体验差异）
  max_dist: 0.2          # 外观差异门限，越小越严格
  max_age: 70            # 丢多久之内还保留编号
  n_init: 3              # 连续命中几帧才给正式编号
  min_confidence: 0.6    # 低于这个分数的检测不进跟踪
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking
ls -lh models/
````

````text
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/07_multi_object_tracking
python src/app.py --ros
````

````text
nano config/track_config.yaml
# tracker.mode: deepsort → sort
````

````text
维护一个集合 seen = set()
每帧遍历所有 track：
    如果 track.id 不在 seen 里，且 中心x > 中线x：
        count += 1
        seen.add(track.id)
````

## EXP-08 · 人脸关键点（468 点）— 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-08-face-landmarks/index.html)

````text
# 关键配置（config/face_config.yaml）
model:
  detector_model: models/face_detection_short_range.rknn
  landmark_model: models/face_landmark.rknn
  num_faces: 2                # 最多几张脸
  min_detection_confidence: 0.5

backend:
  core_mask: 0                # 0=AUTO；1=CORE0；2=CORE1；4=CORE2

source:
  mode: image                 # image | ros | camera
  rgb_path: images/face_demo.jpg

server:
  port: 8081                  # 注意：不是 8080，避免和其他实验撞
  stream_name: face.mjpg
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/09_face_landmarks
ls car1_face_landmark/face_landmark_core/
python -m face_landmark_core.demo_image --config config/face_config.yaml --save out.jpg
````

````text
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/09_face_landmarks
python -m face_landmark_core.camera_demo --config config/face_config.yaml --ros
````

````text
nano config/face_config.yaml
````

````text
model:
  num_faces: 2                    # 改 1 试试
  min_detection_confidence: 0.5    # 改 0.3 / 0.8 试试

backend:
  core_mask: 0   # 0=自动  1=核0  2=核1  4=核2  3=核0+1  7=三核全用
````

````text
眼睛开合比 = 上下眼睑距离 / 左右眼角距离
````

````text
scale = 两眼距离 / 贴纸基准宽度
位置  = 鼻尖坐标 - 贴纸中心偏移
````

## EXP-09 · 骨骼关键点（33 点姿态）— 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-09-pose-landmark/index.html)

````text
# 关键配置（config/pose_config.yaml）
model:
  pose_model: models/pose_landmark_full.rknn
  min_detection_confidence: 0.5   # 低于这个分数的姿态直接丢弃

source:
  mode: image                      # image | ros | camera
  rgb_path: images/pose_demo.jpg

server:
  port: 8082                       # 09=8081、10=8082、11=8083、12=8084
  stream_name: pose.mjpg
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/10_pose_landmark/car1_pose_landmark
python -m pose_landmark_core.demo_image --config config/pose_config.yaml --save out.jpg
````

````text
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/10_pose_landmark/car1_pose_landmark
python -m pose_landmark_core.camera_demo --config config/pose_config.yaml --ros
````

````text
# 关键点索引（MediaPipe 约定）
L_SHOULDER, R_SHOULDER = 11, 12
L_WRIST,    R_WRIST    = 15, 16

# 图像 y 轴向下，所以 y 更小 = 更高
左手举起 = 手腕15.y < 肩膀11.y  and  手腕15.vis > 0.5
右手举起 = 手腕16.y < 肩膀12.y  and  手腕16.vis > 0.5
````

````text
身长 = |肩中点 − 髋中点|
归一化手腕高度 = (髋中点y − 手腕y) / 身长
````

## EXP-10 · 深度相机与 3D 定位 — 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-10-depth-3d/index.html)

````text
# 水平视场角（度）
FOV ≈ 2 × arctan(width / (2·fx)) = 2 × arctan(640 / (2×608.4)) ≈ 54.9°
# 垂直视场角
FOV_v ≈ 2 × arctan(480 / (2×611.2)) ≈ 43.0°
````

````text
# 已知内参
fx = 608.39958    fy = 611.20714
cx = 328.30096    cy = 226.46483

# 反投影
X = (400 − 328.30) × 0.5 / 608.40 = 0.0589 米   ← 往右 5.9 厘米
Y = (240 − 226.46) × 0.5 / 611.21 = 0.0111 米   ← 往下 1.1 厘米
Z = 0.5 米                     ← 前方 50 厘米
````

````text
# 关键配置（config/depth3d_config.yaml）
depth:
  scale: 0.001           # 毫米 → 米（÷1000）
  invalid_value: 0        # 0 = 测不到（平台事实）
  colormap: jet
  display_scale: 0.666    # 显示时缩到 2/3 → 点击坐标要除以它还原

cam3d:
  fx: 608.39958          # ← 内参，反投影用的
  fy: 611.20714
  cx: 328.30096
  cy: 226.46483
  width: 640
  height: 480

source:
  mode: ros              # ros | image

server:
  port: 8083
  stream_name: depth.mjpg
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/08_depth_3d_detection
cat config/depth3d_config.yaml | head -20
````

````text
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/08_depth_3d_detection
python src/app.py --ros
````

````text
python src/app.py --ros --display
````

````text
# 手算模板（拿计算器也行）
X = (u − 328.30096) × Z / 608.39958
Y = (v − 226.46483) × Z / 611.20714
````

````text
u=412 → X = (412 − 328.30) × 0.483 / 608.40 = 0.0664 m   ← u>cx，所以 X>0（在右侧）
v=238 → Y = (238 − 226.46) × 0.483 / 611.21 = 0.0091 m   ← v>cy，所以 Y>0（在下方）
````

````text
for v in range(480):
    for u in range(640):
        Z = depth[v,u] / 1000.0
        if Z == 0: continue                      # 跳过无效
        X = (u - cx) * Z / fx
        Y = (v - cy) * Z / fy
        points.append((X, Y, Z))
````

## EXP-11 · 体感控制（动作 → 小车指令）— 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-11-pose-control/index.html)

````text
# 概念示意，不是真实代码
身长 = |髋中点 − 肩中点|          # 这个距离作为"1 个单位"

# 归一化后的高度（0 = 髋部，1 = 肩膀，1.5 = 头顶以上）
手腕高度 = (髋中点y − 手腕y) / 身长

# 判断规则就变成了"比值比阈值大"
if 手腕高度 > 0.72:  举手
````

````text
# 关键配置（config/pose_control_config.yaml）
model:
  pose_model: models/pose_landmark_full.rknn
  min_detection_confidence: 0.5

server:
  port: 8083                        # 本实验用 8083
  stream_name: pose_control.mjpg

control:
  publish_cmd_vel: false           # ← 默认关闭！要手动改成 true
  linear: 0.2                      # 前进/后退线速度 m/s（车端 clamp ±0.3）
  angular: 0.8                     # 转向角速度 rad/s（车端 clamp ±1.0）
  topic: /cmd_vel
  print_interval: 2.0               # 终端打印间隔（秒）
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/11_pose_control/car1_pose_control
# 终端 1：起相机（同前）
# 终端 2：
python -m pose_control_core.camera_demo \
    --config config/pose_control_config.yaml --ros
````

````text
source /opt/ros/humble/setup.bash
ros2 run turtlesim turtlesim_node
````

````text
ros2 topic info /cmd_vel    # 应该看到至少 1 个订阅者（底盘控制器）
````

````text
nano config/pose_control_config.yaml
# control.publish_cmd_vel: false → true
# control.topic:  B2 用 /turtle1/cmd_vel    C1 用 /cmd_vel

python -m pose_control_core.camera_demo \
    --config config/pose_control_config.yaml --ros
````

````text
grep -rn "classify_action" src/ car1_pose_control/ 2>/dev/null | head
````

````text
# 概念示意
if 髋部高度 < 阈值:            return BACKWARD   # 下蹲，先判
if 左手高 and 右手高:          return FORWARD    # 双手举
if 左手高:                     return TURN_LEFT
if 右手高:                     return TURN_RIGHT
return STOP                                   # 兜底
````

````text
候选动作 = classify_action(...)
if 候选动作 == 上一个候选:
    连续计数 += 1
else:
    连续计数 = 1
    上一个候选 = 候选动作

if 连续计数 >= 3:        # 稳定 3 帧
    当前动作 = 候选动作
````

````text
手高度 = 归一化手腕高度
速度 = clamp((手高度 − 0.4) / (1.2 − 0.4), 0, 1) × 最大速度
````

## EXP-12 · 手势识别（4 模型级联）— 机器人视觉入门课

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp001-robot-vision/exp-12-hand-control/index.html)

````text
# 关键配置（config/hand_control_config.yaml）
model:
  detector_model: models/hand_detector.rknn
  landmark_model: models/hand_landmarks_detector.rknn
  embedder_model: models/gesture_embedder.rknn
  classifier_model: models/canned_gesture_classifier.rknn
  max_hands: 2                     # 最多几只手
  min_detection_confidence: 0.5
  min_tracking_confidence: 0.5

server:
  port: 8084                        # 本实验用 8084
  stream_name: hand.mjpg

# 按 'f' 键可以循环切换 5 种画面特效
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/12_hand_control/car1_hand_control
ls models/
python -m hand_control_core.demo_image --config config/hand_control_config.yaml --save out.jpg
````

````text
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
export ROS_DOMAIN_ID=2
ros2 launch ascamera hp60c.launch.py
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/vision/12_hand_control/car1_hand_control
python -m hand_control_core.camera_demo --config config/hand_control_config.yaml --ros
````

````text
# 拇指尖 = 第 4 号点，食指尖 = 第 8 号点
# 用这两个点和指根构成向量，算夹角（余弦定理）
角度 = arccos( (a·b) / (|a|×|b|) )

# 映射成 0~255 的控制量
值 = 255 × (角度 / 最大角度)
````

````text
nano config/hand_control_config.yaml
````

````text
model:
  max_hands: 2                     # 改 1 看看
  min_detection_confidence: 0.5     # 手掌检测门限
  min_tracking_confidence: 0.5      # 21 点 presence 门限 ← 关键
````

````text
if Fingers == 5 and 稳定 10 帧:
    开灯 / 启动 / 触发某个动作
if Fingers == 0 and 稳定 10 帧:
    关灯 / 停止
````

````text
✋ Open_Palm   →  停止
☝️ Pointing_Up →  前进
👍 Thumb_Up    →  左转
👎 Thumb_Down  →  右转
✌️ Victory     →  原地转圈
````

## EXP-01 · 六麦阵列串口唤醒 — 机器人自然语言交互应用

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp002-speech-interaction/exp-01-mic-array-wakeup/index.html)

````text
# ① 硬件：讯飞 XFM 六麦阵列 → USB 接入工控主机
lsusb | grep -i 1a86
Bus 001 Device 005: ID 1a86:55d4 QinHeng CH340 serial converter

# ② 确认设备节点（可能是 ttyACM0 或 ttyUSB0）
ls -l /dev/ttyACM*
crw-rw---- 1 root dialout 188, 0 ... /dev/ttyACM0

# ③ 把当前用户加进 dialout 组（否则 Permission denied）
sudo usermod -aG dialout $USER   # 重新登录后生效

# ④ 依赖与素材
pip3 install pyserial
# 确认 xf_mic_array.py / uv_device.py / 讲义 resources/ 在实验目录就位
````

````text
# —— 核心：帧校验与小端解析（xf_mic_array.py 节选）——
def _checksum(data: bytes) -> int:
    return (~sum(data) + 1) & 0xFF      # 一元反码 +1，即二补码

HDR, BEAM_EVT, IVW_EVT = 0xA5, 0x09, 0x12
buf += ser.read(ser.in_waiting)            # 非阻塞灌入接收缓冲
if buf and buf[0] != HDR: buf = buf[1:]   # 丢弃错位字节，重新找帧头
ptype, plen = buf[1], buf[2]
frame, buf = buf[:7+plen+1], buf[7+plen+1:]  # 7 字节头 + N 载荷 + 1 校验
if _checksum(frame[:7+plen]) != frame[-1]:
    raise ValueError("checksum mismatch")
payload = frame[7:7+plen]
if ptype == BEAM_EVT:                    # 波束事件：小端取角度
    beam = payload[0]; angle = payload[1] | (payload[2] << 8)
    print(f"[BEAM] id={beam} angle={angle}°")
elif ptype == IVW_EVT:                   # 唤醒事件 → 防抖后上报
    now = time.time()
    if not wakeup_lock and now - system_ts > 3.0:
        system_ts, wakeup_lock = now, True
        print(f"[WAKEUP] beam={beam} {angle}°")

# —— 运行与自检 ——
python3 -m xfmicarray.uv_device            # 枚举并直读原始帧
# [BEAM] id=4 angle=315° ...（手指绕阵列走，看角度跟着转）
# [WAKEUP] beam=4 angle=300°（喊"小飞小飞"）
python3 xf_mic_array_gui.py                # Tkinter 界面：指针 + 事件日志
````

## EXP-02 · 声学模型 ASR 语音识别 — 机器人自然语言交互应用

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp002-speech-interaction/exp-02-asr/index.html)

````text
# ① 安装推理框架（版本必须锁定，混装易 ABI 报错）
pip3 install sherpa-onnx==1.10.0

# ② 模型文件就位（三件套 + 词表 + 热词文件）
ls models/zipformer-transducer/
encoder_fp16.rknn  decoder_fp16.rknn  joiner_fp16.rknn  tokens.txt  hotwords.txt

# ③ 确认阵列声卡（数字为示例，以 arecord -l 实际输出为准）
arecord -l | grep -i xfm
card 4: XFM1100 [XFM1100], device 0: ...

# ④ 确认 NPU 驱动在位
ls /dev/rknpu
/dev/rknpu   ← 存在即 RKNN 加速可用
````

````text
# —— 识别器构建（asr_test.py 核心配置）——
import sherpa_onnx
config = sherpa_onnx.OfflineRecognizerConfig.from_transducer(
    encoder=sherpa_onnx.OfflineTransducerModelConfig(
        encoder=sherpa_onnx.CTCModelConfig(
            model="models/zipformer-transducer/encoder_fp16.rknn"),
        decoder=..., joiner=...),   # 三件套齐
    tokens="models/zipformer-transducer/tokens.txt", tokens_type="zh")
config.model.provider = "rknn"            # 改 "cpu" 做对照组
config.decoding_method = "modified_beam_search"  # 热词仅此模式生效！
config.hotwords_file = "hotwords.txt"       # 打开灯 关闭灯 前进 后退 左转 右转 停下
config.hotwords_score = 2.0                  # 每个热词 token 的加分
config.rule1_min_trailing_silence = 4.5      # 静音超 4.5s 判停
config.rule2_min_trailing_silence = 1.5      # 检出句号且尾静音 1.5s 判停
config.rule3_max_utterance_length = 300.0    # 硬上限 300s

# —— 拾音与分块（100ms 一口喂进去）——
import numpy as np, sounddevice as sd
device = detect_xfm_devices()     # arecord -l | grep XFM → "plughw:4,0"
def callback(indata, frames, t, status):
    stream.accept_waveform(sample_rate, indata[:, 0])   # 边说边解
    while stream.is_ready(): recognizer.decode_stream(stream)
    print("partial:", stream.result.text)              # 实时回显
    if stream.detected_endpoint(): print("final:", ...)

python3 asr_test.py --mic-array
[ASR] provider=rknn  decoding=modified_beam_search  hotwords=7
partial: 打开实验室的灯
final: 打开实验室的灯 → COMMAND_MAP 命中: 打开灯

# —— 热词失效排查（三连）——
# ① decoding_method 是不是 modified_beam_search？② 热词是否用空格分词？③ 路径对不对？
````

## EXP-03 · TTS 语音合成 — 机器人自然语言交互应用

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp002-speech-interaction/exp-03-tts/index.html)

````text
# ① 模型就位（声学模型 + 声码器 + 规则文件）
ls models/tts/
model-steps-3.onnx   vocos-16khz-univ.onnx   rule_fsts/  lexicon.txt

# ② 播放链自检：放一段已知正常的测试音（不经过 TTS 模型）
python3 audio_test.py --play ref_16k.wav
[audio] playing ref_16k.wav on plughw:4,0 ... OK

# ③ 查看当前 ALSA 各通道音量（对比实验前后）
amixer sget Master | head -5
amixer sget Output          # 阵列侧混音控件名以 amixer 管单为准

# ④ 依赖
pip3 install onnxruntime numpy soundfile sounddevice
````

````text
# —— 合成主链（tts_test.py 核心，节选）——
import numpy as np, onnxruntime as ort
text_raw = "今天是2026年9月24日，气温26度"
text = run_fsts(text_raw, ["number-zh","date-zh","phone-zh"])  # 数字/日期/电话正读
tokens, durs = matcha_encoder(text)        # 音素 + 每音时长
mel  = matcha.run(tokens, durs, steps=3)   # 梅尔谱 [T, n_mels]
wav  = vocos.run(mel)                      # float 波形 [-1,1] @16kHz
wav  = np.tanh(wav)                        # 软削幅，防 >1.0 削顶爆音

# —— 音量：百分比 → dB（pct_to_db）——
def pct_to_db(pct):
    if pct <= 0: return -96.0              # 数字音频静音下限
    return 20 * np.log10(pct / 100)

# —— 播放：单声道必须复制成双声道（I2S 立体声出）——
pcm = (wav * 32767).astype(np.int16)
stereo = np.column_stack([pcm, pcm]).ravel()   # 关键三行之一：防 2 倍速
sd.play(stereo, samplerate=16000, device="plughw:4,0")

# —— 命令行 ——
python3 tts_test.py --text "你好，小飞已就位"
[TTS] syn 0.62s / audio 1.8s → RTF=0.34 ✓ → play plughw:4,0
python3 tts_test.py --text "3pm meeting, 2026-09-24" --speed 1.2 --sid 1 --playback-volume 70
````

## EXP-04 端到端语音对话 · CORE-03 教程

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp002-speech-interaction/exp-04-voice-dialog/index.html)

````text
# 0) 确认六麦阵列与 ALSA 设备（详见 EXP-01/02）
lsusb | grep -i 1a86          # ID 1a86:55d4 QinHeng
arecord -l                    # card 4: XFMDPV0018, device 0

# 1) 确认依赖模型就位（EXP-02/03 已生成）
#    ASR: sherpa-onnx-streaming-zipformer-* + tokens.txt
#    TTS: model-steps-3.onnx + vocos-16khz-univ.onnx
#    LLM: Qwen3.5-0.8B_w4a16_g128_rk3576.rkllm + /usr/lib/librkllmrt.so
````

````text
# 终端 1（auto_start_llm=False 时需手动；否则由 voice_interaction 自动拉起）
python3 llm_server.py

# 终端 2：主命令
cd ros2_ws/src/xflz_eai_car1/interaction/car1_interaction/tools/
python3 voice_interaction.py --mic-array
# [Main] 找到录音设备: plughw:4,0 (XFMDPV0018)
# [KWS] 设置唤醒词: 小飞小飞, 小飞同学, 你好小飞
# [LLM] 启动 LLM: Qwen3.5-0.8B_w4a16_g128_rk3576.rkllm (端口 8080)
# [TTS] speed=1.0 gain=2.0
# 就绪 — 说唤醒词开始对话, Ctrl+C 退出
````

````text
# [Main] 播放提示音 (0.67s, 42880 bytes)
# [Main] 排空音频队列: 第1轮 3 帧, 第2轮 1 帧
# [Main] IDLE → LISTENING    [ASR] 今天北京天气怎么样？
# [LLM]  我没有实时查询天气的能力…（逐句 TTS 播放）
# [Main] SPEAKING → IDLE
````

````text
# 2a. 持久化 aplay：另开终端确认只有一个 aplay 常驻进程
ps aux | grep aplay
# /usr/bin/aplay -t raw -f S16_LE -c 2 -r 16000 -D plughw:4,0 -q -

# 2b. 打断测试：长回答播放中再次说"小飞小飞"
# [KWS] 打断: 小飞小飞 → [Main] SPEAKING → LISTENING (打断)

# 2c. ASR 黑名单：唤醒后只说"Hi"
# [ASR] 结果无效丢弃: "HI" → IDLE（VIVO/HELLO/OK 等噪声词直接丢弃）
````

````text
# voice_interaction.py L328-330（原式含逗号、顿号，切得细、延迟低）
SENTENCE_RE = re.compile(r'([。！？；，、\n]|[.?!,](?=\s|$))'
# 保守版：只整句送 TTS，断句奇怪但合成次数少
SENTENCE_RE = re.compile(r'([。！？\n]|[.?!](?=\s|$))'
````

````text
python voice_tools_interaction.py --mic-array
# [Main] 已加载 4 个机械臂模式: ['home', 'pick', 'place', 'work']
# [Main] 机械臂串口已打开: /dev/ttyUSB0 @ 1000000
# 说"机械臂请抓取" → 关键词路径（核心字"抓"）<1s 执行 pick
# 说"机械臂攻坚"   → fuzzy_map 纠正 攻坚→工作 → work
# 问"今天天气怎么样" → LLM 输出 none → 不动作，回 IDLE
````

## EXP-05 大模型语义理解 · CORE-03 教程

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp002-speech-interaction/exp-05-llm-understanding/index.html)

````text
# 0) 确认板载运行时存在
ls -l /usr/lib/librkllmrt.so        # 软链目标，llm_server 首启自动 symlink
# 1) 启动服务（终端 1）
cd ros2_ws/src/xflz_eai_car1/interaction/car1_llm/tools/
python3 llm_server.py               # =========init....=========== rkllm init success!
# 2) 客户端连通性（终端 2）
curl http://127.0.0.1:8080/v1/models # {"data":[{"id":"Qwen3.5-0.8B_w4a16_g128_rk3576"...]}
````

````text
python3 llm_test.py
# You > 我叫小明   → AI > 你好，小明！很高兴认识你。
# You > 我刚才说我叫什么？ → AI > 你刚才说你叫小明。（messages 历史生效）
# You > clear 一键清空历史    You > exit 退出
# 采样对比：事实问题低温，创意任务高温
python3 llm_test.py --prompt "1+1=?" --temperature 0.1 --no-stream
python3 llm_test.py --prompt "写一首关于机器人的诗" --temperature 1.5
````

````text
messages = [
  {"role": "system", "content":
   "你是语义解析器。只输出JSON：{\\"intent\\":查询|控制|闲聊, \\"slots\\":{...}}，禁止解释。"},
  {"role": "user", "content": "把机械臂放到右边并抓取方块"},
]
# 期望: {"intent":"控制","slots":{"动作":["放置","抓取"],"位置":"右边"}}
resp = client.chat_once(messages, temperature=0.1, max_tokens=200)
````

````text
python3 llm_test.py --mode tools --prompt "现在几点了？"
# 可用工具: get_current_time, get_system_info, calculator, get_battery_status
# model → tool_calls: get_current_time() → FUNCTION_MAP 执行
# → {"role":"tool","name":...,"content":"{...}"} → 二次请求 → "现在是 14:30"
# 数学题同理: "1+1等于几?" → calculator(expression="1+1") → {"result": 2}
python3 llm_test.py --mode tools --prompt "计算 123 * 456"
````

## EXP-06 多模态语音情绪感知 · CORE-03 教程

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp002-speech-interaction/exp-06-emotion-perception/index.html)

````text
# 0) 前置就绪检查（三站分别是 EXP-02 / EXP-05 主命令）
python3 asr_test.py --mic-array   # 文本通道原料：ASR 文本
python3 llm_test.py --prompt "你好"  # LLM 连通（可承担 text 分类器）
# 1) 录一段情绪样本（平静/愤怒/悲伤/开心 各 1 条）
arecord -D plughw:4,0 -r 16000 -f S16_LE -c 1 -d 3 angry.wav
````

````text
arecord -D plughw:4,0 -r 16000 -f S16_LE -c 1 -d 3 samples/angry_01.wav
arecord -D plughw:4,0 -r 16000 -f S16_LE -c 1 -d 3 samples/sad_01.wav
# 真实语料优先用 EXP-02 链路边说边存：唤醒 → 朗读情绪句 → 得 (wav, text) 对
````

````text
LEX = {"棒": +2, "喜欢": +1.5, "烦": -2, "太": 1.5, "不": -1}
def text_score(s):
    # 遍历命中词 → 前置程度词加倍、前置否定词取反 → clip 到 [-3, +3]
    # 分数段映射六类: >1.5 开心, >0.3 积极, >-0.3 中性, >-1.5 消极, >=愤怒词 愤怒, 否则 悲伤

# 或直接委托 LLM（白名单约束，承接 EXP-05 的 system prompt 手法）：
"你是情绪标注员，只输出: 开心|悲伤|愤怒|积极|消极|中性"
````

````text
# F0 均值/方差（基频轨迹近似）、RMS 平均能量、字率与静音占比
# 归一化到该说话人基线后映射: 高能量+高语速 → 愤怒(+强度)
#                             低能量+慢语速+F0平 → 悲伤; 温和起伏 → 开心
````

````text
final = w*text + (1-w)*voice   # 冲突(异号)则 confidence *= 0.5
# 标签写入 LLM 上下文 → 回复语气调整：
"用户当前情绪:悲伤。回复更温和简短,先安抚再解决。"
# 播报风格呼应 EXP-03 参数（稍慢、降增益）：
python3 tts_test.py --text "别急，我们马上帮您处理。" --gain 1.5
````

## EXP-01 · ROS2 简介 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-01-ros2-intro/index.html)

````text
# ① 确认发行版（预期输出 humble）
echo $ROS_DISTRO
humble

# ② 加载 ROS2 环境（每个新开终端都要执行！）
source /opt/ros/humble/setup.bash

# ③ 看 ros2 命令认识自己的能力清单
ros2 --help
# ... verbs: action bag interface launch node param pkg run service topic ...
#            ↑ 八大动词组，EXP-02 逐一拆解
````

````text
# ① 装包（只需一次）
sudo apt install ros-humble-turtlesim

# ② 启动海龟窗口，保持开着（拆解见下面的讲解卡）
ros2 run turtlesim turtlesim_node

# ③ 再开一个终端，按 ↑↓←→ 方向键遥控海龟
ros2 run turtlesim turtle_teleop_key
# 报 No executable found？说明命令被拼接过，先跑 ros2 pkg executables turtlesim 核对
````

````text
# ① 首次先装官方示例包（已装跳过；缺它必报 package not found）
sudo apt install ros-humble-demo-nodes-cpp
# 核对: ros2 pkg executables demo_nodes_cpp  →  应列出 talker / listener

# ② 终端 A：加载环境（新开终端必做）
source /opt/ros/humble/setup.bash

# ③ 启动 talker，保持运行
ros2 run demo_nodes_cpp talker
# [INFO] [1712...]: Publishing: 'Hello World: 1'
# [INFO] [1712...]: Publishing: 'Hello World: 2'
````

````text
# 终端 C：以第三方身份俯瞰整个系统
ros2 node list
# /listener  /talker
ros2 topic list
# /chatter  /parameter_events  /rosout
ros2 topic info /chatter
# Type: std_msgs/msg/String   Publisher count: 1   Subscription count: 1
ros2 topic echo /chatter        # 第三方"匿名"订阅，实时看到 data
ros2 topic hz /chatter          # average rate: 1.000
````

````text
#include "rclcpp/rclcpp.hpp"          // ROS2 C++ 客户端库
#include "std_msgs/msg/string.hpp"      // 消息类型：字符串

class Talker : public rclcpp::Node   // ① 任何节点都继承 Node
{
public:
  Talker() : Node("talker")              // ② 节点名就叫 talker
  {
    // ③ 建一个发布者：往 "chatter" 这个话题上发 String
    pub_ = create_publisher<std_msgs::msg::String>("chatter", 10);

    // ④ 每秒触发一次，发一条编号递增的消息
    timer_ = create_wall_timer(1s, [this]() {
      auto msg = std_msgs::msg::String();
      msg.data = "Hello World: " + std::to_string(count_++);
      RCLCPP_INFO(get_logger(), "Publishing: '%s'", msg.data.c_str());
      pub_->publish(msg);            // ← 发布动作：把消息推上 chatter
    });
  }
private:
  size_t count_ = 1;
  rclcpp::Publisher<std_msgs::msg::String>::SharedPtr pub_;
  rclcpp::TimerBase::SharedPtr timer_;
};
````

````text
#include "rclcpp/rclcpp.hpp"
#include "std_msgs/msg/string.hpp"

class Listener : public rclcpp::Node   // ① 同样继承 Node
{
public:
  Listener() : Node("listener")            // ② 节点名叫 listener
  {
    // ③ 建一个订阅者：盯住同一个 "chatter" 话题
    sub_ = create_subscription<std_msgs::msg::String>(
      "chatter", 10,
      [this](std_msgs::msg::String::ConstSharedPtr msg) {   // ④ 回调
        RCLCPP_INFO(get_logger(), "I heard: [%s]", msg->data.c_str());
      });                          // ← 回调：每收到一条就执行一次
  }
private:
  rclcpp::Subscription<std_msgs::msg::String>::SharedPtr sub_;
};
````

## EXP-02 · ROS2 常用命令工具 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-02-cli-tools/index.html)

````text
if not cli.service_is_ready():
    cli.wait_for_service()
    # 服务永远不起 → 死等
````

````text
ok = cli.wait_for_service(
        timeout_sec=2.0)
if not ok:
    # 2 秒后主动放弃，去处理
    return 'service 不在线'
````

````text
rclpy.spin_until_future_complete(
    node, future)
# 服务不回包 → 死等
````

````text
rclpy.spin_until_future_complete(
    node, future, timeout_sec=2.0)
if not future.done():
    # 超时了，主线程继续走
````

````text
# 准备：终端 A 跑 talker，终端 B 备用
ros2 run demo_nodes_cpp talker

# ① 先看 topic 的"发完就走"：终端 C
ros2 topic pub --once /chatter std_msgs/msg/String "{data: 'hello'}"
# → 瞬间返回。即使终端 B 的 listener 没开，这条命令也照样"成功"

# ② 再看 service 的"必须等到回包"：终端 C
ros2 service call /talker/get_parameters rcl_interfaces/srv/GetParameters "{names: ['use_sim_time']}"
# → 立刻返回 response: values=[false]（服务端在线，所以很快）

# ③ 关键对比：把终端 A 的 talker 杀掉，再执行 ②
# → 卡住了。屏幕停在 "waiting for service to become available..."
# → 此时 ① 依然瞬间返回。这就是异步与同步的鸿沟
# → 只能 Ctrl+C 中断：CLI 没有超时参数，它不会自己放弃
````

````text
# 语法：timeout [选项] 秒数 命令...
# ③ 的改进版：最多等 3 秒，超时自动杀掉
timeout 3 ros2 service call /talker/get_parameters rcl_interfaces/srv/GetParameters "{names: ['use_sim_time']}"
# → 3 秒后进程被终止，提示：Terminated
# → 终端立即回到提示符，不用手动 Ctrl+C

# 想知道"到底是超时了，还是命令自己失败"？用 --preserve-status 区分退出码
timeout --preserve-status 3 ros2 service call ...
echo $?      # 124 = 超时被杀；0 = 正常返回；其他 = 命令自身报错

# 常用选项
#   -s SIGINT   超时后发 SIGINT（相当于替你按 Ctrl+C，更温和）
#   --preserve-status  保留命令自身的退出码，便于脚本判断
#   -k 2        如果 SIGINT 不管用，2 秒后补一个 SIGKILL 强制杀
````

````text
# 终端 C：
ros2 node info /talker
# Publishers:
#   /chatter            ← 它往外说的话题
#   /parameter_events   ← 参数变更广播（每个节点都有）
#   /rosout             ← 日志话题（每个节点都有）
# Subscribers:
#   /parameter_events /rosout
# Service Servers:
#   /talker/describe_parameters   ← 自带参数服务器！
#   /talker/get_parameters  /talker/set_parameters ...
````

````text
# 终端 C：观察
ros2 topic list -t
# /chatter [std_msgs/msg/String]  /parameter_events  /rosout
ros2 topic hz /chatter                  # average rate: 1.000 · min/max 软实时统计

# 终端 D：动手改系统
ros2 topic pub --once /chatter std_msgs/msg/String "{data: 'injected'}"
# 验证点：终端 B 的 listener 多打印一行 I heard: [injected] —— 发布者是谁它根本不关心

# 参数组三连（终端 C）
ros2 param list /talker                  # /talker  use_sim_time
ros2 param dump /talker > talker.yaml    # 导出快照（生成了 yaml 文件即成功）
ros2 param set /talker use_sim_time false # 返回 Success 即热改生效
````

````text
# 终端 D：录制（跑着的 talker/listener 会一起被录下来）
ros2 bag record -a -o exp02_test        # -a=all  10~20 秒就 Ctrl+C
ros2 bag info exp02_test
# Files: exp02_test_0.db3        Duration: 12.3s
# Topic /chatter: 12 msgs    Serialization: cdr

# 关键验证：先杀掉终端 A 的 talker，再回放
ros2 bag play exp02_test                # 时间戳原样 → 终端 B 恢复打印 Hello World
ros2 bag play exp02_test --rate 2.0    # 2 倍速回放，调试效率翻倍
````

## EXP-03 · 工作空间与 colcon — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-03-workspace/index.html)

````text
# ~/ros2_ws/ —— 设备上的真实工作空间（src 下的目录全部列出）
~/ros2_ws/                          ← 工作空间根目录，colcon build 就在这一层敲
├── src/                            ← 源码层：所有 ROS2 工程都住在这里（唯一手写、进 git 的层）
│   └── xflz_eai_car1/              ← 业务顶层目录，src 下的目录全在它里面，再按用途分类
│       ├── bringup/               ← 整车一键启动（launch/bringup.launch.py）
│       ├── driver/                ← 驱动层
│       │   ├── car1_base_driver/        ← 底盘驱动（/car1_base_driver/* 话题）
│       │   └── ros_sensor/              ← 射频传感：ros_sensor + ros_sensor_interfaces 两个工程
│       ├── navigation/            ← 导航
│       │   └── car1_navigation/         ← 导航栈（launch / rviz / 参数）
│       ├── arm/                   ← 机械臂
│       │   ├── car1_arm_controller/     ← 底层协议与动作控制
│       │   └── kinematics/              ← 正 / 逆运动学解算
│       ├── simulations/           ← 仿真三件套
│       │   ├── car1_description/        ← URDF 模型
│       │   ├── car1_moveit_config/      ← MoveIt 配置
│       │   └── car1_gazebo/             ← Gazebo 仿真
│       ├── calibration/           ← 舵机标定（servo_calib.launch.py、servo_offsets.yaml）
│       ├── vision/                ← 视觉算法工程，编号 01～12 共 12 个
│       │   ├── 01_industrial_defect_detection/   ← 工业缺陷检测
│       │   ├── 02_garbage_classification/        ← 垃圾分类
│       │   ├── 03_lab_color_recognition/         ← LAB 颜色识别
│       │   ├── 04_qrcode/                     ← 二维码识别
│       │   ├── 05_line_following/              ← 巡线
│       │   ├── 06_semantic_segmentation/       ← 语义分割
│       │   ├── 07_multi_object_tracking/       ← 多目标跟踪
│       │   ├── 08_depth_3d_detection/          ← 深度 3D 检测
│       │   ├── 09_face_landmarks/              ← 人脸关键点
│       │   ├── 10_pose_landmark/               ← 人体姿态关键点
│       │   ├── 11_pose_control/                ← 姿态控制
│       │   └── 12_hand_control/                ← 手部控制
│       ├── interaction/           ← 语音 / 多模态交互
│       │   ├── car1_interaction/           ← 语音交互与工具调用
│       │   └── car1_multi_interaction/     ← 相机 + 多模态交互节点
│       ├── app/                   ← 应用类工程
│       │   ├── object_tracking/           ← 目标跟踪应用
│       │   ├── eai_factory/               ← 工厂场景应用
│       │   ├── car1_app/                  ← 机械臂视觉抓取
│       │   └── elderly_companion/         ← 陪伴场景应用
│       ├── test/                  ← 测试 / 演示工具
│       │   ├── car1_web_server/           ← Web 运动学校真
│       │   └── camera_web_viewer/         ← 相机 Web 预览
│       ├── examples/              ← 外部示例（camera/HP60C 相机 SDK 例程）
│       └── ros2_demos/            ← 官方示例源码：talker / listener，本课练手用
│           ├── demo_nodes_cpp/            ← 工程①：C++ 版
│           ├── demo_nodes_py/             ← 工程②：Python 版
│           └── example_interfaces/        ← 工程③：消息/服务接口，被上面两个依赖
├── build/                        ← 编译中间产物（第一次 colcon build 后出现）
├── install/                      ← 编译成品 + 环境脚本
└── log/                          ← 构建日志
````

````text
# 列出每个工程的"身份证"
find ~/ros2_ws/src -name package.xml
# 统计工程总数 —— 设备上会输出几十行，一个个 cd 进去手动编译根本不现实
find ~/ros2_ws/src -name package.xml | wc -l
````

````text
cd ~/ros2_ws
colcon build            # 一条命令构建全部工程；末尾 Summary 汇总成功/失败与耗时
````

````text
# ~/ros2_ws/src/xflz_eai_car1/ros2_demos/demo_nodes_cpp/
demo_nodes_cpp/
├── package.xml                ← 包的"身份证"，缺它 colcon 扫到这里连停都不会停
├── CMakeLists.txt             ← 构建说明书：怎么编、装哪、命令叫什么
├── include/demo_nodes_cpp/    ← 头文件目录，供源码 #include
└── src/topics/
    ├── talker.cpp             ← 发布者节点源码（你 echo 到的 Hello World 就从这几行产生）
    └── listener.cpp           ← 订阅者节点源码，一发一收对讲
````

````text
# ① 确认底层已加载（装完 ROS2 的默认动作，或显式执行）
source /opt/ros/humble/setup.bash
# ② 在工作空间根目录（有 src/ 的那层）执行编译
colcon build
# ③ 激活你的工作空间（把 install/ 送进环境）
source install/setup.bash
# ④ 验证
ros2 pkg prefix demo_nodes_cpp   # 输出 ~/ros2_ws/install 即成功；还是 /opt/ros 说明 source 没做对
````

````text
# 高频选项 —— 不用背，过一遍，用到时回来查
colcon build --packages-select demo_nodes_cpp    # 只编指定的包（大工程提速关键）
colcon build --packages-up-to car1_navigation     # 编这个包，连同它依赖的所有包一起
colcon build --packages-skip demo_nodes_py       # 跳过指定的包
colcon build --parallel-workers 2                # 最多 2 个进程并行；默认占满 CPU，机器卡时调低
colcon build --executor sequential               # 逐包按顺序编，排查问题时日志更清楚
colcon build --symlink-install                   # Python/launch 改动免重编，直接生效
colcon build --event-handlers console_direct+    # 实时滚动输出，失败原因一目了然
# 成功标志：Summary: N packages finished [2.34s]
````

````text
# ① 检查工具链：应显示 build/test/list 等子命令
colcon --help | head -1
# command not found？ sudo apt install python3-colcon-common-extensions -y

# ② 从零建骨架 —— 此时 ~/my_ws 底下干干净净只有 src/
mkdir -p ~/my_ws/src
cp -r ~/ros2_ws/src/xflz_eai_car1/ros2_demos ~/my_ws/src/
ls ~/my_ws/src/ros2_demos     # 三个工程：demo_nodes_cpp / demo_nodes_py / example_interfaces

# ③ 回根目录全量编译（不指定包，colcon 按依赖自动排序）
cd ~/my_ws
colcon build                      # 预期 Summary: 3 packages finished
source install/setup.bash      # 激活自己的工作空间，每个新终端都要执行

# 窗口 1：跑 C++ 版
ros2 run demo_nodes_cpp talker     # 每秒一条 Hello World: N，这窗口先留着

# 窗口 2：拿上一章的三件套验收
ros2 topic list | grep chatter      # 话题在不在？
ros2 topic echo /chatter            # 能不能收到 "Hello World: N"？
ros2 topic hz /chatter              # 是不是约 1.0 Hz？
````

````text
# demo_nodes_py/：少了 CMakeLists.txt，多了个 setup.py 当"命令注册表"
demo_nodes_py/
├── package.xml                ← build_type 换成了 ament_python，工具链就切过去了
├── setup.py                   ← entry_points：'talker = demo_nodes_py.topics.talker:main'
├── setup.cfg                  ← 声明入口脚本装到 lib/demo_nodes_py/
├── resource/demo_nodes_py     ← 一个空文件，文件名就是包名，给索引用
└── demo_nodes_py/
    └── topics/
        ├── talker.py          ← 六十来行，Node 子类加个定时器
        └── listener.py
````

````text
# ④ Python 版：激活同一个工作空间，只换包名
source ~/my_ws/install/setup.bash
ros2 run demo_nodes_py talker          # 另一个窗口照旧 list / echo / hz
````

````text
# ⑤ 删产物重编 + 覆盖验收（生成物删了永远安全，正好练一次 out-of-source）
cd ~/my_ws
rm -rf build install log              # 第 03 节说过：删了随时能重新生成
colcon build
source install/setup.bash
ls ~/my_ws                       # 四层齐了：build/ install/ log/ src/
ros2 pkg prefix demo_nodes_cpp    # 指向 ~/my_ws/install 而非 /opt/ros —— 覆盖成功 ✅
ros2 pkg prefix demo_nodes_py     # Python 版同理 ✅
````

````text
# ① 正确顺序：系统层先，自己层后（全新终端执行）
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix demo_nodes_cpp   # → /home/<你>/ros2_ws/install/demo_nodes_cpp ✅ 晚进列表的排在前，Overlay 赢

# ② 新开一个终端（绝不要沿用①的终端！）：先只加载系统层
source /opt/ros/humble/setup.bash
ros2 pkg prefix demo_nodes_cpp   # → /opt/ros/humble/... ✅ 此刻只有系统层，它独占第一（无 Overlay 时回落系统层）

# ③ 同一终端再补上自己的层：首次进列表 → 插到最前
source ~/ros2_ws/install/setup.bash
ros2 pkg prefix demo_nodes_cpp   # → /home/<你>/ros2_ws/install/demo_nodes_cpp ✅ 后 source 者胜

# ④ 看清真实顺序（自上而下 = 优先级从高到低）
echo $AMENT_PREFIX_PATH | tr ':' '\n'
echo $COLCON_PREFIX_PATH | tr ':' '\n'

# ⑤ 反向验证：新开终端，只加载自己的层、跳过 Underlay
source ~/ros2_ws/install/setup.bash
ros2 --help                   # → 仍 command not found ⚠
# 原因：Overlay 只"追加"路径，不自带 ros2 本体 —— Underlay 不能省

# ⑥ 一键恢复：删产物重新来过（永远安全的操作）
rm -rf build install log && colcon build
````

## EXP-04 · 功能包与依赖管理 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-04-package/index.html)

````text
# ① C++ 那个：源码是 .cpp，构建说明书叫 CMakeLists.txt
ls ~/ros2_ws/src/xflz_eai_car1/ros2_demos/demo_nodes_cpp/
# package.xml  CMakeLists.txt  include/  src/

# ② Python 那个：没有 CMakeLists，换成 setup.py
ls ~/ros2_ws/src/xflz_eai_car1/ros2_demos/demo_nodes_py/
# package.xml  setup.py  setup.cfg  resource/  demo_nodes_py/

# ③ 两份"说明书"各看一眼（下面是节选对照）
cat ~/ros2_ws/src/xflz_eai_car1/ros2_demos/demo_nodes_cpp/package.xml
cat ~/ros2_ws/src/xflz_eai_car1/ros2_demos/demo_nodes_py/package.xml
````

````text
# package.xml 主线字段：两边结构一样，只差几行依赖（节选）
<package format="3">
  <name>demo_nodes_cpp</name>                     # ← 我是谁
  <buildtool_depend>ament_cmake</buildtool_depend> # ← 用什么构建（py 包这里换成 ament_python）
  <depend>rclcpp</depend>                         # ← 依赖谁（py 包写 rclpy）
  <depend>std_msgs</depend>                       # ← 消息类型
  <export><build_type>ament_cmake</build_type></export>
</package>
````

````text
# 节选自真机 demo_nodes_cpp/package.xml（cat ~/ros2_ws/src/xflz_eai_car1/ros2_demos/demo_nodes_cpp/package.xml，真文件更长）
<package format="3">
  <name>demo_nodes_cpp</name>
  <version>…</version>                    # 版本/维护者/license 等四件套从略
  # ① 构建工具期：用什么构建我
  <buildtool_depend>ament_cmake</buildtool_depend>
  # ② 仅编译期（build_depend）：g++ 编代码时要用
  <build_depend>rclcpp</build_depend>
  <build_depend>std_msgs</build_depend>
  # ③ 仅运行期（exec_depend）：跑起来/launch 时才用
  <exec_depend>launch_ros</exec_depend>
  <exec_depend>rclcpp</exec_depend>
  # ④ 测试期（test_depend）：colcon test 才用
  <test_depend>ament_lint_auto</test_depend>
  <export><build_type>ament_cmake</build_type></export>
</package>
````

````text
# 对照 demo_nodes_py/package.xml（同样节选）——rclpy、std_msgs 挂在 exec_depend 下：
# Python 不编译，不存在"仅编译期"要装而运行期不用的依赖，所以看不到 build_depend。
# 两个文件都可以真机 cat、再用 grep build_depend 对比着核对一遍
````

````text
cd ~/ros2_ws/src                                        # ① 进 src 目录才能建包
ros2 pkg create my_test_pkg --build-type ament_python \
  --node-name hello_node --dependencies rclpy            # ② 生成骨架：package.xml + setup.py + hello_node.py
cd ~/ros2_ws && colcon build --packages-select my_test_pkg # ③ 回根目录编译，产物进 build/ 与 install/
source install/setup.bash                                # ④ 加载环境——不 source，下面两条都找不到这个包
# ⑤ 查"配料表"：列出该包里所有可被 ros2 run 启动的可执行文件
#   （两列输出 = 包名 + 可执行名，正是 run 第二参数的合法值；清单为空 → 先查③④）
ros2 pkg executables my_test_pkg   # → my_test_pkg hello_node
# ⑥ 按配料表启动
ros2 run my_test_pkg hello_node    # 输出 Hi from my_test_pkg. 后直接退出——模板只是 4 行 print 脚本，还不是真节点
````

````text
cat ~/ros2_ws/src/my_test_pkg/my_test_pkg/hello_node.py
# def main():  print('Hi from my_test_pkg.')，加上两句入口调用——
# 没有 rclpy、没有节点、没有 spin，所以打完就退出，连 node list 都查不到
````

````text
import rclpy
from rclpy.node import Node
from std_msgs.msg import String   # 缺模块时：sudo apt install ros-humble-std-msgs
````

````text
class HelloNode(Node):
    def __init__(self):
        super().__init__('hello_node')
        self.pub = self.create_publisher(String, 'chatter', 10)  # 发布者：话题 chatter，队列 10
        self.timer = self.create_timer(1.0, self.tick)           # 每秒调一次 tick
        self.n = 0

    def tick(self):
        self.n += 1
        self.pub.publish(String(data=f'Hello World: {self.n}'))
        self.get_logger().info(f'发出: Hello World: {self.n}')
````

````text
def main():
    rclpy.init()
    rclpy.spin(HelloNode())
    rclpy.shutdown()
````

````text
# ⑦ 保存后重新编译、运行，用上一章的命令验收
cd ~/ros2_ws
colcon build --packages-select my_test_pkg   # 改了源码必须重编（--symlink-install 可省这步，见 EXP-03 第 03 节）
source install/setup.bash
ros2 run my_test_pkg hello_node              # 窗口 1：每秒打一条"发出: …"，保持运行

# 窗口 2：EXP-02 学的四条命令逐项验收
ros2 node list              # → /hello_node   节点活着
ros2 topic list             # → /chatter      话题在
ros2 topic echo /chatter    # → 每秒一条 data: 'Hello World: N'
ros2 topic hz /chatter      # → 约 1.0 Hz     频率对
````

````text
# ① 回到工作空间根目录，先编 C++ 那个——真编译，build/ 里会堆出 .o
cd ~/ros2_ws
colcon build --packages-select demo_nodes_cpp
ls build/demo_nodes_cpp/CMakeFiles/   # 大堆 .o 中间产物——g++ 真的干活了

# ② 再编 Python 那个——没有 g++ 这一步，秒过
colcon build --packages-select demo_nodes_py
ls build/demo_nodes_py/               # 几乎是空的：流程照走，只是不真编译

# ③ source 后对比两层产物去了哪
source install/setup.bash
ls install/demo_nodes_cpp/lib/demo_nodes_cpp/    # C++：可执行文件装这里
ls install/demo_nodes_py/lib/demo_nodes_py/      # Python：入口脚本也汇到这里
ls install/demo_nodes_cpp/share/demo_nodes_cpp/  # 两边都有 share/：清单、launch 都在

# ④ 清单与 CMake 成对核对：depend 决定装谁、find_package 决定链谁
grep depend ~/ros2_ws/src/xflz_eai_car1/ros2_demos/demo_nodes_cpp/package.xml
grep find_package ~/ros2_ws/src/xflz_eai_car1/ros2_demos/demo_nodes_cpp/CMakeLists.txt

# ⑤ 各跑一个节点，确认真能用
ros2 run demo_nodes_cpp talker   # 另开窗口 ros2 topic echo /chatter 看得到
ros2 run demo_nodes_py talker
````

## EXP-05 · 节点：ROS2 里的"一个岗位" — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-05-node/index.html)

````text
# 终端 A：发布者
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp talker

# 终端 B：订阅者
source /opt/ros/humble/setup.bash
ros2 run demo_nodes_cpp listener
````

````text
# -y 表示自动确认，省掉一次 y/n 交互
sudo apt install -y ros-humble-rqt-graph
````

````text
source /opt/ros/humble/setup.bash
ros2 run rqt_graph rqt_graph
````

````text
import rclpy
from rclpy.node import Node

# ── 同一个 context（同一个进程、同一套 ROS 运行环境）──
ctx1 = rclpy.Context()
rclpy.init(context=ctx1)
node_a = Node('a', context=ctx1)   # 节点 a
node_b = Node('b', context=ctx1)   # 节点 b —— 与 a 同进程、同 context
# 此时 `ros2 node list` 看到 2 个节点，但系统里只多了 1 个进程

# ── 再起一个 context：仍在这一个进程里，但是彼此独立的 ROS 实例 ──
ctx2 = rclpy.Context()
rclpy.init(context=ctx2)
node_c = Node('c', context=ctx2)   # 节点 c：同进程，但属于另一套 ROS 运行环境

# 结果：3 个节点全部在同一个 Python 进程内 —— 进程 1 个，节点 3 个
````

````text
from rclpy.node import Node

class GasSensorNode(Node):                 # 块1：继承 Node，拿到"入场券"
    def __init__(self):
        super().__init__('gas_sensor_node')      # ① 报名字 → 进 ROS2 图
        self.declare_parameter('publish_period', 1.0)  # ② 声明旋钮
        self.pub = self.create_publisher(Float32, '/gas_sensor', qos)  # ③ 开出口
        self.timer = self.create_timer(period, self.timer_cb)          # ④ 定闹钟
        # 注：period 在 __init__ 里就能读到，因为 super().__init__() 已执行完，
        #     节点此时已完整；但声明(②)一定要在读(period)之前。

    def timer_cb(self):                       # 块3：回调，被闹钟触发
        raw, v, ppm, err = read_mq5_once()
        self.pub.publish(Float32(data=ppm))
        self.get_logger().info(f'raw={raw} voltage={v:.3f}V ppm={ppm:.2f}')

def main(args=None):                         # 块4：入口，固定四步
    rclpy.init(args=args)                     # 开门前的准备
    node = GasSensorNode()                    # 装修 + 挂牌（构造）
    try: rclpy.spin(node)                   # 站到柜台后（运行）
    except KeyboardInterrupt: pass
    node.destroy_node(); rclpy.shutdown()     # 打烊摘牌（销毁）
````

````text
ros2 node list                  # 点名：现在都有谁在？→ /gas_sensor_node
ros2 node info /gas_sensor_node # 查户口：它发什么、收什么、提供哪些服务
#  Publishers: /gas_sensor [std_msgs/msg/Float32]   ← 构造函数第 ③ 步开的出口
#  Subscribers: /parameter_events, /rosout         ← ROS2 给每个节点自动挂的两个"系统话题"
#  Services: /gas_sensor_node/describe_parameters … ← 参数能力（EXP-10 细讲）
````

````text
# 最短的 main()：rclpy.spin 内部就是用 SingleThreadedExecutor 帮你 spin
def main():
    rclpy.init()
    node = GasSensorNode()
    try:
        rclpy.spin(node)          # 单线程，一次一个回调，其余排队
    except KeyboardInterrupt:
        pass
    finally:
        node.destroy_node()
        rclpy.shutdown()
````

````text
# 展开后的等价写法，看得更清楚
from rclpy.executors import SingleThreadedExecutor

executor = SingleThreadedExecutor()
executor.add_node(node)
try:
    executor.spin()
finally:
    executor.shutdown()
    node.destroy_node()
````

````text
# 多线程版：num_threads 决定最多几个回调同时跑
import threading
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.executors import MultiThreadedExecutor

class GasSensorNode(Node):
    def __init__(self):
        super().__init__('gas_sensor_node')
        # 共享数据必须先配一把锁
        self._lock = threading.Lock()
        self._last_ppm = 0.0
        self.create_timer(1.0, self.timer_cb)
        self.create_subscription(Float32, '/cmd', self.cmd_cb, 10)

    def timer_cb(self):
        ppm = read_mq5_once()
        with self._lock:              # 加锁写
            self._last_ppm = ppm

    def cmd_cb(self, msg):
        with self._lock:              # 加锁读
            print(f'当前 {self._last_ppm}')


def main():
    rclpy.init()
    node = GasSensorNode()

    # 关键一行：换成多线程执行器，开 4 个线程
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)

    try:
        executor.spin()             # 4 个线程同时从队列里取活干
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
````

````text
# 回调组：给回调分组，决定组内是"互斥"还是"可重入"
import time
import rclpy
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.callback_groups import (
    MutuallyExclusiveCallbackGroup,   # 同组互斥：一次只跑一个
    ReentrantCallbackGroup,           # 同组可重入：可并发跑多个
)
from rclpy.executors import MultiThreadedExecutor


class RobotNode(Node):
    def __init__(self):
        super().__init__('robot_node')

        # 组 A：慢活，互斥——同一时刻只允许一个在跑
        self.group_slow = MutuallyExclusiveCallbackGroup()

        # 组 B：快活，可重入——多个可以同时跑
        self.group_fast = ReentrantCallbackGroup()

        # 创建时把组传进去，这个回调就归到该组
        self.create_timer(5.0, self.slow_cb, callback_group=self.group_slow)
        self.create_subscription(
            Float32, '/cmd', self.fast_cb, 10,
            callback_group=self.group_fast)      # ← 关键参数

    def slow_cb(self):
        time.sleep(3)               # 慢：即便多个触发，同组互斥也不会叠加

    def fast_cb(self, msg):
        print(msg.data)            # 快：多条消息可并发处理


def main():
    rclpy.init()
    node = RobotNode()

    # 回调组必须配合多线程执行器才有多线程效果
    executor = MultiThreadedExecutor(num_threads=4)
    executor.add_node(node)
    try:
        executor.spin()
    except KeyboardInterrupt:
        pass
    finally:
        executor.shutdown()
        node.destroy_node()
        rclpy.shutdown()
````

````text
# 相对名 + 命名空间：同一份代码跑两台车，互不串台
ros2 run my_pkg my_node --ros-args -r __ns:=/r1   # 话题变 /r1/cmd_vel
ros2 run my_pkg my_node --ros-args -r __ns:=/r2   # 话题变 /r2/cmd_vel
# 若代码里写死全局名 /cmd_vel，上面这条就失效——两台车照样抢同一个话题
````

````text
source /opt/ros/humble/setup.bash
echo $ROS_DISTRO      # 打印 humble 才算加载成功
````

````text
cd ~/ros2_ws
colcon build --packages-select ros_sensor ros_sensor_interfaces
````

````text
source install/setup.bash
ros2 pkg executables ros_sensor   # 应列出 gas_sensor_node 等可执行名
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/driver/ros_sensor/ros_sensor/ros_sensor
cat gas_sensor_node.py
````

````text
grep -A6 console_scripts setup.py
````

````text
entry_points={
    'console_scripts': [
        'gas_sensor_node = ros_sensor.gas_sensor_node:main',
    ],
},
````

````text
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 run ros_sensor gas_sensor_node
````

````text
ros2 node list                     # 点名 → /gas_sensor_node
ros2 node info /gas_sensor_node    # 查户口：看 Publishers 段
ros2 topic list                    # → /gas_sensor、/parameter_events、/rosout
````

````text
# 终端 2：Ctrl+C 停掉节点，改完名字后重新编译 + source
cd ~/ros2_ws
colcon build --packages-select ros_sensor
source install/setup.bash
ros2 run ros_sensor gas_sensor_node
# 终端 3：确认名字变了
ros2 node list        # → /my_gas_node
````

````text
# 再开一个终端起第二个实例：名字被自动改成 _1
ros2 run ros_sensor gas_sensor_node
ros2 node list
# → /gas_sensor_node 和 /gas_sensor_node_1   （只给 WARN，不报错）

# 换命名空间：名字带上前辍，互不串台
ros2 run ros_sensor gas_sensor_node --ros-args -r __ns:=/r1
ros2 node list        # → /r1/gas_sensor_node
````

````text
# 终端 2：按 Ctrl+C，应干净回到提示符（不该有红色 traceback）
# 终端 3：立刻再查一次
ros2 node list        # → 空（或已无 gas_sensor_node）
````

````text
# 终端 1–2 各起一个节点：talker 与 listener
ros2 run demo_nodes_cpp talker
ros2 run demo_nodes_cpp listener

# 全景观察
ros2 node list
ros2 node info /listener
#  Subscriber /parameter_events: [rcl_interfaces/msg/ParameterEvent]
#  Service /listener/get_parameters: [rcl_interfaces/srv/GetParameters]

# 用命名空间再造一个"平行世界"
ros2 run demo_nodes_cpp talker --ros-args -r __ns:=/r2
ros2 node list   # 共存：/talker 与 /r2/talker，互不串线
````

## 实验 06 · ROS2 话题通讯原理（MQ5 气体传感器）

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-06-topic/index.html)

````text
# ① 进工作空间，并先加载 ROS2 本体（underlay）
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
# ② 只编译本实验需要的两个包：节点包 + 它依赖的接口包
#    用 --packages-select 点名，不碰工作空间里其它包，快且不易连带报错
colcon build --packages-select ros_sensor ros_sensor_interfaces
# ③ 把工作空间注册进当前终端（新终端必做，不能省）
source install/setup.bash
````

````text
# 新终端必做：进目录 + 加载本工作空间（不然找不到 ros_sensor）
cd ~/ros2_ws
source install/setup.bash
# 启动 MQ5 采集节点（普通用户即可，不需要 sudo）
ros2 run ros_sensor gas_sensor_node
````

````text
# 同样的固定开场：进目录 + source（新终端必做）
cd ~/ros2_ws
source install/setup.bash
# 启动订阅 / 报警节点
ros2 run ros_sensor gas_monitor
````

````text
# 新终端开场：进目录 + source（后面的命令都在这个终端里敲，不用再 source）
cd ~/ros2_ws
source install/setup.bash
# 给整个系统的节点"点名"
ros2 node list
````

````text
# 列出当前活跃的话题（本终端已 source 过，直接敲）
ros2 topic list
````

````text
# 看这条话题的类型 + 谁在发、谁在收
ros2 topic info /gas_sensor
# 再加一把 --verbose：还能看到两端的 QoS 参数
ros2 topic info /gas_sensor --verbose
````

````text
# 变成第三个订阅者实时"偷听"；随时 Ctrl+C 停止
ros2 topic echo /gas_sensor
````

````text
# 测这条话题的发布频率；随时 Ctrl+C 停止
ros2 topic hz /gas_sensor
````

````text
# 在终端 B 按 Ctrl+C 停掉，再用参数重启
ros2 run ros_sensor gas_sensor_node --ros-args -p publish_period:=0.5
````

````text
# 在终端 C 按 Ctrl+C 停掉，再用参数重启
ros2 run ros_sensor gas_monitor --ros-args -p alarm_threshold:=100.0
````

````text
# 来源：ros_sensor/common.py L13-33
V_REF, RL, R0 = 5.0, 1.0, 10.0      # 参考电压、负载电阻 kΩ、标定电阻 kΩ
CURVE_B, CURVE_M = 0.186, -0.362  # 灵敏特性曲线截距与斜率（斜率为负！）

def voltage_to_ppm(v):                        # 电压 → 浓度
    if v <= 0 or v >= V_REF:                 # ← 除零保护：量程外的电压不算
        return 0.0
    rs    = RL * (V_REF - v) / v               # ③ 由分压关系反推气敏电阻 Rs
    ratio = rs / R0                            # ③ 与标定点 R0 的比值
    return 10 ** ((math.log10(ratio) - CURVE_B) / CURVE_M)   # ④ 对数域线性插值
````

````text
# 来源：ros_sensor/common.py L36-47
def read_mq5_once():
    try:
        scale = float(read_adc_attr("in_voltage_scale"))      # ① 量程系数：1 格 = 多少毫伏
        raw   = int(read_adc_attr(f"in_voltage{MQ5_CH}_raw"))   # ② 原始格数（12 bit，0~4095）
        v = raw * scale / 1000.0                          # ③ 格数×毫伏/格=毫伏，÷1000→伏特
        return raw, v, voltage_to_ppm(v), ""            # ④ 一次返回四件事
    except Exception as e:
        return None, None, None, f"{type(e).__name__}: {e}"   # 失败信息交给调用方
````

````text
# 来源：ros_sensor/gas_sensor_node.py L16-44
from rclpy.node import Node
from std_msgs.msg import Float32
from rclpy.qos import QoSProfile, HistoryPolicy, ReliabilityPolicy
from .common import read_mq5_once                # 复用 ① 里的采集函数

class GasSensorNode(Node):
    def __init__(self):
        super().__init__('gas_sensor_node')                       # ① 注册节点名
        self.declare_parameter('publish_period', 1.0)         # ② 声明可调参数

        qos = QoSProfile(                                          # ③ 传输合同
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.pub = self.create_publisher(Float32, '/gas_sensor', qos)   # ④ 创建发布端

        period = float(self.get_parameter('publish_period').value)
        self.timer = self.create_timer(period, self.timer_cb)      # ⑤ 定时器：每 period 秒叫一次

    def timer_cb(self):                                            # ⑥ 每秒被触发一次的回调
        raw, v, ppm, err = read_mq5_once()                          #    读 + 算
        if err:
            self.get_logger().error(f'读取 ADC 失败: {err}')       #    出错只记日志，不崩溃
            return
        msg = Float32()                                             #    装消息
        msg.data = float(ppm)
        self.pub.publish(msg)                                       # ⑦ 发布：入队即返回
        self.get_logger().info(f'raw={raw} voltage={v:.3f}V ppm={ppm:.2f}')

def main():
    rclpy.init()                    # 初始化 rclpy
    node = GasSensorNode()          # 构造节点（此时才注册进计算图）
    try: rclpy.spin(node)         # 进入事件循环：驱动定时器与回调
    except KeyboardInterrupt: pass
    finally: node.destroy_node(); rclpy.shutdown()   # Ctrl+C 后干净退出
````

````text
# 来源：ros_sensor/gas_monitor.py L13-36
class GasMonitor(Node):
    def __init__(self):
        super().__init__('gas_monitor')                        # ① 另一个节点名
        self.declare_parameter('alarm_threshold', 500.0)    # ② 报警阈值也是参数
        self.threshold = float(self.get_parameter('alarm_threshold').value)

        qos = QoSProfile(                                       # ③ QoS 必须与发布端相容
            history=HistoryPolicy.KEEP_LAST, depth=10,
            reliability=ReliabilityPolicy.RELIABLE,
        )
        self.sub = self.create_subscription(                    # ④ 订阅：类型 + 话题 + 回调 + QoS
            Float32, '/gas_sensor', self.callback, qos)

    def callback(self, msg):                                    # ⑤ 消息到了才会被调用
        ppm = msg.data
        if ppm >= self.threshold:
            self.get_logger().warn(f'[报警] 气体浓度超标: {ppm:.2f} ppm >= {self.threshold} ppm')
        else:
            self.get_logger().info(f'浓度正常: {ppm:.2f} ppm')
````

## EXP-07 · 服务通信：指纹验证 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-07-service/index.html)

````text
cd ~/ros2_ws/src/xflz_eai_car1/driver/ros_sensor/ros_sensor/ros_sensor
````

````text
python3 fingerprint_enroll.py enroll -i 1
````

````text
python3 fingerprint_enroll.py list
````

````text
cd ~/ros2_ws
source install/setup.bash
ros2 run ros_sensor fingerprint_server
````

````text
cd ~/ros2_ws
source install/setup.bash
ros2 run ros_sensor fingerprint_client
````

````text
ros2 service list | grep verify
````

````text
ros2 service type /verify_fingerprint
````

````text
ros2 service call /verify_fingerprint ros_sensor_interfaces/srv/VerifyFingerprint "{}"
````

````text
# 来源：fingerprint_enroll.py L19-33（指令码与确认码定义）
# ── 指令码：每个都是"一条命令的编号"──
CMD_GET_IMAGE   = 0x01   # PS_GetImage   采集图像
CMD_GEN_CHAR    = 0x02   # PS_GenChar    生成特征
CMD_SEARCH      = 0x04   # PS_Search     搜索指纹库
CMD_REG_MODEL   = 0x05   # PS_RegModel   合并特征生成模板（录入用）
CMD_STORE_CHAR  = 0x06   # PS_StoreChar  储存模板（录入用）

# ── 确认码：模组对每条命令的"答复"──
# 0x00 成功   0x01 收包有错   0x02 无手指（状态，不是错误！）
````

````text
# 帧结构（_build_packet，L36-42）：一帧命令长这样——
┌────────┬──────────┬────────┬────────┬────────┬────────┬──────────┐
│ 包头    │ 地址      │ 包标识  │ 长度    │ 指令码  │ 参数    │ 校验和    │
│ EF01    │ FF FF FF │ 01     │ 2 字节  │ 1 字节  │ n 字节  │ 2 字节    │
│ 固定头   │ 默认地址  │ 命令包  │ 后段长度 │ 干什么  │ 附加数据 │ 防传输出错 │
└────────┴──────────┴────────┴────────┴────────┴────────┴──────────┘
# 校验和 = 对"包标识+长度+指令码+参数"逐字节求和取低 16 位。
# _send()（L56-73）负责发帧、等应答帧、校验确认码——带超时保护。
````

````text
# Fingerprint 类对外方法（接口示意，实现在 L45-124）
fp = Fingerprint(port, baud)      # 打开串口
code = fp.get_image()             # → 0x00 采到图 / 0x02 无手指 / 0x01 帧错误
code = fp.gen_char(buf_id)        # 图像 → 特征
result = fp.search()              # 搜索库：result.page=命中页码（未命中 -1）
# 录入脚本专用：fp.reg_model() 合并两次特征 / fp.store_char() 写入 Flash
````

````text
# 来源：fingerprint_server.py L19-26（节点初始化：参数声明 + 服务注册）
super().__init__('fingerprint_server')
self.declare_parameter('port', os.environ.get('FP_PORT', '/dev/ttyS10'))
self.declare_parameter('baud', int(os.environ.get('FP_BAUD', '57600')))
self.declare_parameter('retries', 50)          # 轮询次数：50×0.2s≈10s 窗口
self.srv = self.create_service(
    VerifyFingerprint, 'verify_fingerprint', self.handle_request)
#              ↑类型        ↑服务名（=客户端找它的地址）  ↑回调
````

````text
# 来源：fingerprint_server.py L46-87（handle_request：验证三步 + 串口保护）
def handle_request(self, request, response):
    fp = Fingerprint(port, baud)                 # 打开串口
    try:
        for _ in range(retries):                 # ① 轮询等手指
            code = fp.get_image()
            if code == 0x00: break              #    采到图 → 出循环
            if code == 0x02:                    #    无手指 = 状态，继续等
                time.sleep(0.2); continue
            response.success = False            #    其它码 = 真错误
            response.message = f'采集图像失败(code={code})'
            return response
        fp.gen_char(1)                          # ② 生成特征
        result = fp.search()                    # ③ 搜索比对
        if result.page >= 0:
            response.success = True
            response.user_id = result.page      # 命中页码就是用户编号
            response.message = f'匹配成功 score={result.score}'
        else:
            response.success = False; response.user_id = -1
            response.message = '未匹配到指纹'
    finally:
        fp.close()   # 串口必须无条件归还，否则下次打开失败（泄漏）
    return response  # 返回的 response 即回传给客户端
````

````text
# 来源：fingerprint_client.py L16-40（创建客户端 → 发请求 → 等结果）
super().__init__('fingerprint_client')
self.cli = self.create_client(VerifyFingerprint, 'verify_fingerprint')
#                              ↑同名服务：客户端靠这个名字找到服务端
while not self.cli.wait_for_service(timeout_sec=1.0):   # ① 等上线
    self.get_logger().info('等待指纹服务 /verify_fingerprint ...')

req = VerifyFingerprint.Request()              # ② 本 srv 请求为空
future = self.cli.call_async(req)              # ③ 异步发：立即返回 Future
rclpy.spin_until_future_complete(self, future) # ④ 同步等：执行器转到 future 完成
resp = future.result()
if resp is None: ...                          # 防御：调用失败时为 None
if resp.success:
    self.get_logger().info(f'验证成功 user_id={resp.user_id} {resp.message}')
else:
    self.get_logger().error(f'验证失败 user_id={resp.user_id} {resp.message}')
````

## EXP-08 · 动作通信：RFID 读卡 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-08-action/index.html)

````text
# ① 进工作空间，并先加载 ROS2 本体（underlay）
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
# ② 只编译本实验需要的两个包：接口包 + 节点包（顺序由 colcon 按依赖自动排）
colcon build --packages-select ros_sensor ros_sensor_interfaces
# ③ 把工作空间注册进当前终端（新终端必做，不能省）
source install/setup.bash
````

````text
# 把 SPI 设备交给 dialout 组（iflytek 已在该组内）
sudo chgrp dialout /dev/spidev1.0
# 组内成员可读写该设备（660 = 属主读写 + 组读写）
sudo chmod 660 /dev/spidev1.0
````

````text
# 先把这两个引脚"导出"到用户空间，才会出现 gpio114/116 目录
sudo sh -c 'echo 114 > /sys/class/gpio/export; echo 116 > /sys/class/gpio/export'
# 放开 114 的方向与电平文件：666 让任何用户都能读写
sudo chmod 666 /sys/class/gpio/gpio114/direction /sys/class/gpio/gpio114/value
# 同样放开 116，复位引脚也是普通用户要拉的
sudo chmod 666 /sys/class/gpio/gpio116/direction /sys/class/gpio/gpio116/value
````

````text
# 进驱动目录，跑脱离 ROS 的裸硬件自检
cd src/xflz_eai_car1/driver/ros_sensor/ros_sensor/ros_sensor
# 自检 MFRC522：读版本寄存器 + 回读验证，PASS 即硬件通路正常
python3 rc522_reader.py --test
````

````text
cd ~/ros2_ws
source install/setup.bash
ros2 run ros_sensor rfid_action_server
````

````text
cd ~/ros2_ws
source install/setup.bash
ros2 run ros_sensor rfid_action_client 30
````

````text
ros2 action list
ros2 action info /rfid_scan
````

````text
ros2 action send_goal /rfid_scan ros_sensor_interfaces/action/RfidScan "{timeout_sec: 30}" --feedback
````

````text
# 结局 2 · 超时 abort：故意不放卡，等 timeout_sec 秒
# 与上一步唯一区别：把 30 改成 10，这样不用干等半分钟
ros2 action send_goal /rfid_scan ros_sensor_interfaces/action/RfidScan "{timeout_sec: 10}" --feedback
# → SCANNING 持续滚动 10s → Result: success=False msg=读卡超时

# 结局 3 · 取消 canceled：上一条跑起来后按 Ctrl+C（CLI 会发 cancel）
# 这次 timeout_sec 写大一点（如 60），才有时间在中途按 Ctrl+C
# → Feedback 中断 → Result: success=False msg=已取消
````

````text
# 源码目录（用一条命令即可跳到）
cd ~/ros2_ws/src/xflz_eai_car1/driver/ros_sensor/ros_sensor/ros_sensor
ls *.py
````

````text
# 来源：rc522_reader.py L296-305（完整读卡流程：寻卡 + 防冲突两步串成一步）
def read_uid(self) -> tuple:
    ok, atqa = self.request_a()          # ① REQA 寻卡：卡在不在射频场里？
    if not ok:
        return False, "", b''            #    不在 → SCANNING 下一轮再来
    ok, uid = self.anticoll()            # ② AntiCollision 防冲突读 UID
    if not ok:
        return False, f"ATQA=0x{atqa:04X}", b''
    uid_str = "%02X%02X%02X%02X" % (uid[0], uid[1], uid[2], uid[3])
    return True, uid_str, uid            # (成功, 卡号十六进制串, 原始字节)
````

````text
# 来源：rfid_action_server.py L41-48（三个回调一次注册）
self._action_server = ActionServer(
    self, RfidScan, 'rfid_scan',
    execute_callback=self.execute_cb,    # 干活的主体（协程）
    goal_callback=self.goal_cb,          # 收/拒目标
    cancel_callback=self.cancel_cb)      # 同意/拒绝取消
````

````text
# 来源：rfid_action_server.py L100-141（执行循环：查取消 → 发反馈 → 读卡）
async def execute_cb(self, goal_handle):
    deadline = time.time() + (goal_handle.request.timeout_sec or 30)
    feedback = RfidScan.Feedback()
    while time.time() < deadline:
        if goal_handle.is_cancel_requested:      # ← 必须在循环体内！
            goal_handle.canceled()               #    同意取消，就地收场
            result.success = False
            result.message = '已取消'
            return result
        feedback.status = 'SCANNING'            # 每轮都发反馈
        goal_handle.publish_feedback(feedback)
        ok, atqa = rc.request_a()                # 寻卡
        if not ok:
            time.sleep(0.1); continue            # 没卡：0.1s 后再来
        feedback.status = 'CARD_FOUND'
        goal_handle.publish_feedback(feedback)
        ok, uid = rc.anticoll()                  # 读 UID
        ...  # READING_UID → VERIFYING 反馈 → classify_card 判型
    goal_handle.abort()                          # deadline 耗尽仍无卡
    result.message = '读卡超时'
    return result
````

````text
# 来源：rfid_action_server.py L19-29（卡类型判定：ATQA + UID 长度）
def classify_card(atqa, uid):
    if atqa == 0x0044 and len(uid) == 7: return 'MIFARE Ultralight'
    if atqa == 0x4400: return 'MIFARE Ultralight C'
    if atqa == 0x0344: return 'MIFARE DESFire'
    if atqa in (0x0002, 0x0003, 0x0004): return 'MIFARE Classic'
    return 'Unknown'
````

````text
# 来源：rfid_action_client.py L51-66（发送 Goal：三层回调接力）
def send(self, timeout=30):
    if not self._client.wait_for_server(timeout_sec=3.0):
        self.get_logger().error('RFID Action 服务不可用')
        return False
    goal = RfidScan.Goal()
    goal.timeout_sec = timeout                    # Goal 参数从这里进
    future = self._client.send_goal_async(
        goal, feedback_callback=self.feedback_cb) # 反馈回调随 Goal 一起挂
    future.add_done_callback(self.goal_response_cb) # 第 1 环：接受了吗？
    while rclpy.ok() and not self._done:            # 必须轮转执行器！
        rclpy.spin_once(self, timeout_sec=0.1)      # 回调全靠它驱动
    return self._ok
````

````text
# 回调链的接力顺序（示意）
goal_response_cb(goal_future)      # 第 1 环：ACCEPTED？
    └─ goal_handle = goal_future.result()
    └─ result_future = goal_handle.get_result_async()
    └─ result_future.add_done_callback(self.result_cb)   # 第 2 环：终态
feedback_cb(feedback_msg)          # 插播：执行期间随时被调
    └─ self.get_logger().info(f'Feedback: {feedback_msg.feedback.status}')
result_cb(result_future)           # 第 2 环：读 Result，按状态判定
    └─ status == GoalStatus.STATUS_SUCCEEDED ? '成功' : '超时或被取消'
    └─ self._done = True          # 放行 send() 里的 while 循环
````

## EXP-09 · 自定义接口消息 · 温湿度传感器 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-09-custom-msg/index.html)

````text
# ① 进工作空间，并加载 ROS2 本体
cd ~/ros2_ws
source /opt/ros/humble/setup.bash
# ② 接口包 + 节点包一起编（顺序由 colcon 按依赖自动排）
colcon build --packages-select ros_sensor_interfaces ros_sensor
# ③ 注册进当前终端
source install/setup.bash
````

````text
# 用 sudo 起一个 root 的 bash，在里面依次加载 3 套环境，最后启动节点
# 行尾的 \ 是续行符：这 5 行在 shell 看来是"一整条命令"，必须整体复制
# ① source /opt/ros/humble/setup.bash → ROS2 本体环境（提供 ros2 命令与 rclpy）
# ② source ~/ros2_ws/install/setup.bash → 本工作空间（提供 ros_sensor 包与消息类）
# ③ source ~/ros2_venv/bin/activate → Python 虚拟环境（节点真正用的解释器）
sudo -E bash -c 'source /opt/ros/humble/setup.bash; \
source /home/iflytek/ros2_ws/install/setup.bash; \
source /home/iflytek/ros2_venv/bin/activate; \
ros2 run ros_sensor dht11_sensor_node'
````

````text
# 新终端开场：进目录 + source
cd ~/ros2_ws
source install/setup.bash
# 看话题列表（应该能找到 /temperature_humidity）
ros2 topic list
# 看这条话题用的什么消息类型
ros2 topic info /temperature_humidity
# 直接打印消息内容（Ctrl+C 停止）
ros2 topic echo /temperature_humidity
````

````text
# 查看消息定义（就是 .msg 文件的内容）
ros2 interface show ros_sensor_interfaces/msg/TemperatureHumidity
````

````text
/dht11/temperature   std_msgs/Float32   → 36.2
/dht11/humidity      std_msgs/Float32   → 82.0
````

````text
# 接口包（ament_cmake）CMakeLists.txt 核心段
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VerifyFingerprint.srv"
  "srv/TriggerRelay.srv"
  "action/RfidScan.action"
  "msg/TemperatureHumidity.msg"
  "msg/HeartRateSpO2.msg"
  "msg/PhotoState.msg"
  DEPENDENCIES std_msgs action_msgs builtin_interfaces   # ← 基础类型从哪来
)
ament_package()
````

````text
# 列出本接口包的所有接口，按 msg / srv / action 三类
ros2 interface list | grep ros_sensor_interfaces
````

````text
ros2 interface show ros_sensor_interfaces/srv/VerifyFingerprint
````

````text
ros2 interface show ros_sensor_interfaces/action/RfidScan
ros2 interface show ros_sensor_interfaces/msg/TemperatureHumidity
````

````text
# 进接口包目录，列出三类定义文件
cd ~/ros2_ws/src/xflz_eai_car1/driver/ros_sensor/ros_sensor_interfaces
ls -R msg srv action
````

````text
# 一次读完三份消息定义
cat msg/TemperatureHumidity.msg
cat msg/HeartRateSpO2.msg
cat msg/PhotoState.msg
````

````text
cat srv/VerifyFingerprint.srv
cat srv/TriggerRelay.srv
````

````text
cat action/RfidScan.action
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/driver/ros_sensor/ros_sensor_interfaces
cat > msg/Illuminance.msg << 'EOF'
# 光照度消息：单段，无分隔符
float32 lux          # 照度(lx)
bool valid           # 读数有效位（遮挡/超量程时 false）
builtin_interfaces/Time stamp   # 最近一次采样时刻
EOF
cat msg/Illuminance.msg   # 回读确认
````

````text
rosidl_generate_interfaces(${PROJECT_NAME}
  "srv/VerifyFingerprint.srv"
  "srv/TriggerRelay.srv"
  "action/RfidScan.action"
  "msg/TemperatureHumidity.msg"
  "msg/HeartRateSpO2.msg"
  "msg/PhotoState.msg"
  "msg/Illuminance.msg"        # ← 新增这一行
  DEPENDENCIES std_msgs action_msgs builtin_interfaces
)
````

````text
cd ~/ros2_ws
colcon build --packages-select ros_sensor_interfaces
source install/setup.bash
ros2 interface show ros_sensor_interfaces/msg/Illuminance
````

## EXP-10 · 参数服务 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-10-parameter/index.html)

````text
# 声明时附带描述符：自描述 + 可视化元数据
from rcl_interfaces.msg import ParameterDescriptor, FloatingPointRange

period_desc = ParameterDescriptor(
    description='MQ5 数据发布周期（秒），越小采样越快。',
    floating_point_range=[
        FloatingPointRange(from_value=0.02, to_value=10.0, step=0.01)
    ]
)
self.declare_parameter('publish_period', 1.0, period_desc)   # 第三参即描述符
````

````text
# 来源：param_sensor_node.py __init__（声明 → 读取 → 发布器/定时器）
self.declare_parameter('publish_period', 1.0)        # ① 声明：名字+默认值，1.0 同时定型为 double
self.declare_parameter('sensor_enabled', True)        #     True → bool；'x' 才是 str
self.declare_parameter('alarm_threshold_ppm', 500.0)  #     声明即登记进"参数注册表"（图 10-2）

self.publish_period = float(self.get_parameter('publish_period').value)     # ② 读取：注册表 → 成员变量
self.sensor_enabled = bool(self.get_parameter('sensor_enabled').value)   #     启动时读一次缓存
self.alarm_threshold = float(self.get_parameter('alarm_threshold_ppm').value)

self.pub = self.create_publisher(Float32, '/gas_sensor', qos)   # ③ 接线：与实验6完全一致
self.timer = self.create_timer(self.publish_period, self.timer_cb)   #     周期就是参数本身
````

````text
# 来源：param_sensor_node.py timer_cb（使能开关 + 阈值报警两处消费参数）
def timer_cb(self):
    if not self.sensor_enabled:          # 消费点①：使能开关——false 时直接返回，
        return                           #   定时器仍在走，只是采样被"跳过"（功耗极低）
    raw, v, ppm, err = read_mq5_once()   # 复用实验6 的真实 MQ5 读取
    msg = Float32(); msg.data = float(ppm)
    self.pub.publish(msg)
    if ppm >= self.alarm_threshold:      # 消费点②：阈值判断——参数一改，下一拍就用新值
        self.get_logger().warn(f'[报警] ppm={ppm:.2f} >= {self.alarm_threshold}ppm')
    else:
        self.get_logger().info(f'raw={raw} voltage={v:.3f}V ppm={ppm:.2f}')
````

````text
# __init__ 末尾注册（必须先导入：from rcl_interfaces.msg import SetParametersResult）
self.add_on_set_parameters_callback(self.on_parameters_set)

def _recreate_timer(self):
    """按新的发布周期重建定时器 —— 参数真正改变行为的关键一步"""
    self.destroy_timer(self.timer)
    self.timer = self.create_timer(self.publish_period, self.timer_cb)

def on_parameters_set(self, params):
    """params: 本次被修改的参数列表（可能一次多个）"""
    for p in params:
        if p.name == 'publish_period':
            if p.value < 0.02 or p.value > 10.0:        # ① 校验：范围外一票否决
                return SetParametersResult(successful=False,
                    reason=f'publish_period 必须在 [0.02, 10.0] 内，收到: {p.value}')
            self.publish_period = float(p.value)           # ② 应用：更新变量
            self._recreate_timer()                      #    + 重建定时器（行为才变）
            self.get_logger().info(f'发布周期已更新: {self.publish_period} s')
        elif p.name == 'sensor_enabled':
            self.sensor_enabled = bool(p.value)
        elif p.name == 'alarm_threshold_ppm':
            if p.value < 0.0:
                return SetParametersResult(successful=False, reason='报警阈值不能为负数')
            self.alarm_threshold = float(p.value)
    return SetParametersResult(successful=True)          # ③ 回执：每个出口都必须返回
````

````text
cd ~/ros2_ws
colcon build --packages-select ros_sensor
source install/setup.bash
````

````text
ros2 run ros_sensor param_sensor
````

````text
ros2 param list
````

````text
ros2 param get /param_sensor publish_period
ros2 run ros_sensor param_sensor --ros-args -p publish_period:=0.1 -p alarm_threshold_ppm:=100.0
````

````text
ros2 param describe /param_sensor publish_period
````

````text
ros2 param set /param_sensor publish_period 0.2
ros2 param set /param_sensor publish_period 100.0
ros2 param set /param_sensor sensor_enabled false
ros2 param set /param_sensor sensor_enabled true
````

````text
ros2 param dump /param_sensor > param_sensor.yaml
cat param_sensor.yaml
ros2 run ros_sensor param_sensor --ros-args --params-file ./param_sensor.yaml
````

## EXP-11 · 时间 API — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-11-time/index.html)

````text
cd ~/ros2_ws/src/xflz_eai_car1/driver/ros_sensor/ros_sensor/ros_sensor
python3 time_demo_node.py
````

````text
ros2 topic echo /sensor_data
````

````text
ros2 topic hz /sensor_data
````

````text
ros2 topic delay /sensor_data
````

````text
node.get_clock().now()
      │
      ├── use_sim_time = false（默认）
      │     时间源：操作系统墙钟（现实时间）
      │     特征：和现实世界一秒一秒同步，不能快进/暂停
      │     适用：真机、真实传感器（USB 相机 / IMU / 雷达）
      │
      └── use_sim_time = true
            时间源：订阅 /clock 话题
            特征：Gazebo / rosbag play 说了算——可 0.5 倍速、可暂停、可回退
            适用：仿真、录包回放

⚠️ 坑：true 了却没人发 /clock → 节点拿不到时间，死等不走（04 节实操 A）
````

````text
now = clock.now()
later = now + Duration(seconds=5.0)  # Time ± Duration
dt = t2 - t1                          # → 得 Duration
now > past                            # 可直接比较
````

````text
stamp:
  sec: 1786615681     # int32 · Unix 秒
  nanosec: 44442152  # uint32 · 纳秒
````

````text
# 来源：time_demo_node.py __init__
self._publisher = self.create_publisher(Header, 'sensor_data', 10)
# 用 Header 消息演示 stamp 字段——它正是所有传感器消息"头部"的原型

self._timer1 = self.create_timer(1.0, self._timer1_callback)   # 1Hz：采样发布
self._sample_count = 0
self._timer2 = self.create_timer(5.0, self._timer2_callback)   # 0.2Hz：状态摘要

self.get_logger().info(f'系统时钟模式: use_sim_time={self.get_parameter("use_sim_time").value}')
# use_sim_time 是系统预声明参数——不用 declare 就能 get，它由 --ros-args -p 控制
````

````text
# 来源：time_demo_node.py _timer1_callback（每秒一次）
now = self.get_clock().now()      # ① 在"采集动作"这一行取时间——不是函数末尾
now_msg = now.to_msg()            # ② API 类型 → 消息类型（rclpy.time.Time → builtin_interfaces/Time）

msg = Header()
msg.stamp = now_msg               # ③ ⭐ 核心：把采集时刻盖进消息头
msg.frame_id = 'sensor_frame'    #     坐标系名，TF2 章节会再见到它
self._publisher.publish(msg)

self.get_logger().info(
    f'采样 #{self._sample_count:04d} | stamp: {now_msg.sec}.{now_msg.nanosec:09d} | ...')
# nanosec 用 :09d 补零到 9 位——否则 44442152 会被误读成更高精度
````

````text
# 来源：time_demo_node.py _timer2_callback（每 5 秒摘要）
self.get_logger().info(
    f'[状态摘要] 累计采样次数: {self._sample_count}, '
    f'当前ROS2时间: {self.get_clock().now().nanoseconds} ns')
# nanoseconds：Unix 纪元起的总纳秒数（int，无溢出风险；C++ 要用 uint64_t）

# 时间运算三件套（可加进节点练习）
from rclpy.time import Duration
future = now + Duration(seconds=5.0)   # Time ± Duration = Time
diff   = now - past                    # Time − Time = Duration（.nanoseconds/1e9 得秒）
print(now > past)                      # Time 之间可直接比较
````

````text
cd ~/ros2_ws/src/xflz_eai_car1/driver/ros_sensor/ros_sensor/ros_sensor
python3 time_demo_node.py --ros-args -p use_sim_time:=true
````

````text
ros2 topic pub /clock rosgraph_msgs/msg/Clock "{clock: {sec: 100, nanosec: 0}}" --rate 1
````

## 实验 12 · DDS 通信中间件剖析（RMW · RTPS · 自动发现）

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-12-dds/index.html)

````text
ros2 run demo_nodes_cpp talker
````

````text
ss -unap | grep -E "7400|7411"
````

````text
ROS_DOMAIN_ID=7 ros2 topic list
````

````text
# 查看任一话题当前生效的完整 QoS 档案
ros2 topic info /chatter --verbose
# 输出含：Reliability: RELIABLE / Durability: VOLATILE / GID: 01.0f...

# 你的 QoS 代码里对应的声明（EXP-06 复习）
qos = QoSProfile(history=HistoryPolicy.KEEP_LAST, depth=10,
                 reliability=ReliabilityPolicy.RELIABLE)
````

````text
ros2 run demo_nodes_cpp talker
````

````text
ss -unap | grep -E "7400|7411"
````

````text
ROS_DOMAIN_ID=7 ros2 run demo_nodes_cpp talker
ss -unap | grep -E "9150|9161"
ROS_DOMAIN_ID=7 ros2 topic list
````

````text
ros2 multicast receive
````

````text
ros2 multicast send
````

````text
ros2 run demo_nodes_cpp talker &
ls -l /dev/shm | grep fast
df -h /dev/shm
````

````text
ros2 topic pub -r 2 --qos-durability transient_local --qos-depth 5 \
  /qos_tl std_msgs/msg/String "{data: retained}"
````

````text
ros2 topic echo --qos-durability transient_local /qos_tl
````

````text
ros2 topic pub -r 5 --qos-reliability best_effort /qos_be \
  std_msgs/msg/String "{data: be}"
````

````text
ros2 topic echo --qos-reliability reliable /qos_be
ros2 topic info -v /qos_be
````

````text
ros2 topic info -v /qos_tl
````

## EXP-13 · ROS2 分布式通讯 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-13-distributed/index.html)

````text
export ROS_DOMAIN_ID=42
ros2 run demo_nodes_cpp talker
````

````text
export ROS_DOMAIN_ID=42
ros2 run demo_nodes_cpp listener
````

````text
export ROS_DOMAIN_ID=99
ros2 topic list
````

````text
unset ROS_DOMAIN_ID
ros2 topic list
````

````text
# 给法 A · 命令行（先看效果最直观）
ros2 run demo_nodes_cpp talker --ros-args -r __ns:=/robot01
# 给法 B · launch 一行（后面多车实验的标准姿势）
Node(package=..., executable=..., namespace='robot01')
````

````text
<!-- peers.xml：固定队员的单播对端名单 -->
<participant><rtps><builtin><initialPeersLocator>
  <locator><udpv4><address>192.168.10.11</address></udpv4></locator>
  <locator><udpv4><address>192.168.10.12</address></udpv4></locator>
</initialPeersLocator></builtin></rtps></participant>
````

````text
export FASTRTPS_DEFAULT_PROFILES_FILE=$PWD/peers.xml   # 每个终端都要，写进 bashrc 固化
````

````text

````

````text
export ROS_DOMAIN_ID=5
echo $ROS_DOMAIN_ID          # 5
ros2 run demo_nodes_cpp talker   # 本终端启动的节点都进域 5
````

````text
export ROS_DOMAIN_ID=5
ros2 run demo_nodes_cpp listener   # 同域才能收
````

````text
ROS_DOMAIN_ID=6 ros2 run demo_nodes_cpp talker
echo $ROS_DOMAIN_ID          # 输出仍为空——环境没被改
````

````text
echo 'export ROS_DOMAIN_ID=5' >> ~/.bashrc
source ~/.bashrc        # 只影响新开终端；老终端不会变
````

````text
export ROS_DOMAIN_ID=42
ros2 run demo_nodes_cpp talker
````

````text
export ROS_DOMAIN_ID=42
ros2 topic list               # 能看到 /chatter
ros2 run demo_nodes_cpp listener
````

````text
export ROS_DOMAIN_ID=99
ros2 topic list               # 只有本机公共话题
ros2 run demo_nodes_cpp listener   # 一直空，等不到数据
````

````text
unset ROS_DOMAIN_ID
ros2 topic list
````

## EXP-14 · rqt 可视化工具箱 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-14-rqt/index.html)

````text
ros2 launch car1_bringup bringup.launch.py
````

````text
ros2 topic list | grep car1
ros2 topic hz /car1_base_driver/battery
ros2 topic echo /car1_base_driver/battery
````

````text
ros2 run rqt_graph rqt_graph
````

````text
ros2 run rqt_plot rqt_plot /car1_base_driver/battery/data
````

````text
ros2 launch car1_peripherals depth_camera.launch.py
ros2 run rqt_image_view rqt_image_view
````

````text
# 方式一：启动完整工作台（自带菜单可再开任意插件）
$ rqt

# 方式二：直启单个插件（省去菜单导航，脚本常用；本机均已安装）
$ rqt_graph
$ rqt_plot
$ rqt_image_view
$ rqt_topic

# 方式三：带参数启动（指定绘图话题，省手打）
$ ros2 run rqt_plot rqt_plot /car1_base_driver/battery/data
$ ros2 run rqt_image_view rqt_image_view
````

````text
ros2 node list                       # 先拿节点名，如 /car1_base_driver
ros2 service list | grep logger
````

````text
ros2 service call /car1_base_driver/set_logger_levels \
  rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{logger: '', level: 10}]}"   # 10=DEBUG
````

````text
ssh -X iflytek@10.1.134.9
rqt                              # 窗口经 X11 转发到本机显示
````

````text
# 同一 source 链 + 启动应用（下同，不再重抄）
source /opt/ros/humble/setup.bash && source /home/iflytek/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0
ros2 launch car1_bringup bringup.launch.py
````

````text
source /opt/ros/humble/setup.bash && source /home/iflytek/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0
ros2 run rqt_graph rqt_graph
````

````text
# 仅在 bringup 已退出时才需要重跑：
source /opt/ros/humble/setup.bash && source /home/iflytek/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0
ros2 launch car1_bringup bringup.launch.py
````

````text
source /opt/ros/humble/setup.bash && source /home/iflytek/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0
ros2 run rqt_plot rqt_plot /car1_base_driver/battery/data
````

````text
# 仅在 bringup 已退出时才需要重跑：
source /opt/ros/humble/setup.bash && source /home/iflytek/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0
ros2 launch car1_bringup bringup.launch.py
````

````text
source /opt/ros/humble/setup.bash && source /home/iflytek/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0
ros2 run rqt_topic rqt_topic
````

````text
source /opt/ros/humble/setup.bash && source /home/iflytek/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0
ros2 launch car1_peripherals peripherals.launch.py
````

````text
source /opt/ros/humble/setup.bash && source /home/iflytek/ros2_ws/install/setup.bash && export ROS_DOMAIN_ID=0
ros2 run rqt_image_view rqt_image_view /ascamera_hp60c/camera_publisher/rgb0/image

# 深度图备用话题——只换最后一段：
ros2 run rqt_image_view rqt_image_view /ascamera_hp60c/camera_publisher/depth0/image_raw
````

````text
ros2 topic list | grep car1
ros2 topic info /car1_base_driver/battery --verbose
ros2 topic hz /car1_base_driver/battery
````

````text
ros2 run rqt_graph rqt_graph
````

````text
ros2 run rqt_plot rqt_plot /car1_base_driver/battery/data
# 或先开空窗口，再在左上角输入框粘贴 /car1_base_driver/battery/data
````

````text
ros2 topic echo /car1_base_driver/battery --once
````

````text
ros2 topic list | grep -i image
ros2 topic info /camera/color/image_raw --verbose
ros2 topic hz /camera/color/image_raw
````

````text
ros2 run rqt_image_view rqt_image_view
````

````text
# 过滤：只留 camera 相关的 WARN 以上
ros2 topic echo /rosout | grep -E "camera|timeout"

# 动态提级（等价 rqt_logger_level，本机现成）：
ros2 service call /camera_node/set_logger_levels \
  rcl_interfaces/srv/SetLoggerLevels \
  "{levels: [{logger: '', level: 10}]}"
# 抓完调回 INFO（level: 20），别让 DEBUG 刷屏影响性能
````

````text
rqt                      # 打开工作台，逐一把 graph/plot/image_view 拖进同一窗口
# Perspectives → Save As → 命名 car_debug → 下次启动直接选它
````

## EXP-15 · rviz2 三维可视化 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-15-rviz2/index.html)

````text
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
````

````text
ros2 launch car1_description display.launch.py
````

````text

````

````text
# 标准启动（默认 Fixed Frame=map，Display 全手动添加）
$ rviz2

# 携带已保存配置启动（本课实操用的就是这条路径）
$ rviz2 -d ~/ros2_ws/install/car1_description/share/car1_description/rviz/car1.rviz

# 用 launch 起（自动带上本车配置，推荐）
$ ros2 launch car1_description display.launch.py          # 车模型 + 关节滑块
$ ros2 launch car1_navigation rviz_navigation.launch.py   # 导航视图

# 边用边查配置路径（不用记死）
$ ros2 pkg prefix car1_description
$ ls $(ros2 pkg prefix car1_description)/share/car1_description/rviz/

# 与导航一起跑时，让 rviz 也认仿真时钟（配合 EXP-11 的 --clock 回放）
$ rviz2 --ros-args -p use_sim_time:=true

# 嵌入式/远程：先确认 GPU 驱动支持 OpenGL，再 ssh -X 转发
$ glxinfo | grep "OpenGL renderer"   # 看是不是硬件渲染（如 Mali-G52）
$ ssh -X iflytek@10.1.134.9 && rviz2
````

````text
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 pkg list | grep car1_description
ros2 pkg prefix car1_description
ros2 pkg executables car1_description   # 预期无输出：本包只有 launch，没有可执行文件
````

````text
ros2 pkg prefix car1_description   # 复用它拿到 share 路径
cat /home/iflytek/ros2_ws/install/car1_description/share/car1_description/launch/display.launch.py
````

````text
ros2 launch car1_description display.launch.py
````

````text
# car1.rviz 里的关键设置（本机文件实测）
Global Options:
  Fixed Frame: base_footprint        # ← 观测基准 = 车底投影点
Displays:
  - Grid        (Value: true)            # 地面网格
  - RobotModel  (Topic: /robot_description)  # 车模型
  - TF          (Show Names: true, Show Axes: true)  # 坐标系轴+名字
````

````text
# rviz 视口鼠标操作速查
左键拖拽     → 旋转视角（绕着焦点转）
中键拖拽     → 平移视角（整个画面平移）
滚轮         → 缩放（近大远小）
右键短按     → 弹出上下文菜单（Focal Point / Reset View）
视口左上      → 显示当前视角名 "Current View"
下方工具栏     → Interact / MoveCamera / Select 三种工具切换
````

````text
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash

# ① 先确认生产者：这三个节点必须都在
ros2 node list

# ② 再看话题存不存在
ros2 topic list | grep -E "tf|robot_description"

# ③ 最后才查变换（前两步没做就查 = 必然报 frame does not exist）
ros2 run tf2_ros tf2_echo base_footprint base_link
````

````text
# 坑1：Ctrl+C 杀不干净 tf2_echo（ros2 run 是两层，包装器死了子进程变孤儿，会一直刷同一句报错）
pkill -x tf2_echo        # 确认 pgrep -x tf2_echo 为空

# 坑2：两个终端的域号必须一致，否则根本不在一个 ROS 图里
echo $ROS_DOMAIN_ID      # 终端 1 和终端 2 各查一次
````

````text
# 不用敲命令，直接拖动滑块窗里的滑条
# 若找不到滑块窗：它可能被 rviz 挡住，从任务栏切换到 joint_state_publisher_gui
````

````text
# 终端 1 按 Ctrl+C 停掉整组 launch（生产者一起没了）
rviz2 -d ~/ros2_ws/install/car1_description/share/car1_description/rviz/car1.rviz

# 另开终端复现报错：
ros2 topic list | grep -E "tf|robot_description"   # 空
ros2 run tf2_ros tf2_echo base_footprint base_link
````

````text
ros2 pkg prefix car1_navigation
cat /home/iflytek/ros2_ws/install/car1_navigation/share/car1_navigation/rviz/navigation.rviz | grep "Value: /"
````

````text
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch car1_navigation rviz_navigation.launch.py
````

````text
source /opt/ros/humble/setup.bash
source ~/ros2_ws/install/setup.bash
ros2 launch car1_navigation navigation.launch.py use_rviz:=false
````

````text
ros2 topic hz /scan                          # 雷达图层：COIN-D6 通常 10Hz 上下（以现场为准）
ros2 topic info /map --verbose               # 地图图层：QoS 里应有 Durability: TRANSIENT_LOCAL
ros2 topic echo /plan --once                 # 路径图层：派了目标点后才有数据
ros2 run tf2_ros tf2_echo map base_link      # 定位图层：map→车体
````

````text
# rviz 顶部工具栏两个关键按钮：
2D Pose Estimate  → 在地图上按住拖一个箭头 = 告诉 amcl "车大概在这，朝这个方向"
                     松手后 /initialpose 发出，粒子云立刻向该处聚拢
2D Goal Pose      → 拖一个目标箭头 = 给 nav2 派目标
                     松手后 /goal_pose 发出 → 出现全局路径(绿) + 局部路径(蓝)
````

````text
# 展开左侧 Displays 面板里的某项，改它的子属性：
LaserScan → Topic: /scan · Size (m): 0.03 · Color Transformer: Intensity
RobotModel → Alpha: 0.8        # 半透明，能看穿外壳看内部结构
TF        → Show Names: true · Marker Scale: 0.3
Map       → Color Scheme: map / costmap   # 地图配色
Grid      → Cell Size: 1 · Plane Cell Count: 20
````

````text
# rviz 菜单：File → Save Config As → 存到包内 config/nav_debug.rviz
git add config/nav_debug.rviz
git diff config/nav_debug.rviz   # 纯 YAML，谁改了哪层一目了然
# 队友复现：
rviz2 -d config/nav_debug.rviz
````

## EXP-16 · rosbag2 数据录制与回放 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-16-rosbag/index.html)

````text
# 终端 A · 发布端
ros2 run demo_nodes_cpp talker

# 终端 B · 订阅端
ros2 run demo_nodes_cpp listener
````

````text
# 终端 C
ros2 bag record -a -o exp16_full
````

````text
ros2 bag info exp16_full
````

````text
ros2 bag play exp16_full
````

````text
ros2 bag play exp16_full -r 2.0
````

````text
ls -l exp16_full/
````

````text

````

````text
mkdir -p ~/ros2_bags
cd ~/ros2_bags
pwd                              # 确认当前目录：/home/iflytek/ros2_bags
````

````text
# 终端 A：现场数据源（保持运行，不要 Ctrl+C）
ros2 run demo_nodes_cpp talker
# [INFO] [talker]: Publishing: 'Hello World: 1'
# [INFO] [talker]: Publishing: 'Hello World: 2'   ...每秒一条
````

````text
# 终端 B：监听端（保持运行）
ros2 run demo_nodes_cpp listener
# [INFO] [listener]: I heard: [Hello World: 1]   ...每秒一条
````

````text
# 终端 C：全量录制
ros2 bag record -a -o exp16_full
# 等待约 15 秒后按 Ctrl+C 停止录制
````

````text
ls -lh exp16_full/
````

````text
cat exp16_full/metadata.yaml
````

````text
ros2 bag info exp16_full
````

````text
# 终端 A、B 按 Ctrl+C 停掉 talker / listener
ros2 node list                       # 预期：空输出，确认系统空闲

# 新终端：起一个 listener 作为回放"观众"（保持运行）
ros2 run demo_nodes_cpp listener

# 终端 C：以原始速率回放
ros2 bag play exp16_full

# 调速实验：二倍速 / 半速
ros2 bag play exp16_full -r 2.0     # 间隔变 0.5s（原 1s）
ros2 bag play exp16_full -r 0.5     # 间隔变 2s

# 只回放指定话题 —— 注意 play 没有 -t 短选项，必须写全 --topics
ros2 bag play exp16_full --topics /chatter   # 忽略 /parameter_events、/rosout
# 从第 5 秒起播（跳过开头稳定段；注意不能超过包的 Duration）
ros2 bag play exp16_full --start-offset 5
````

````text
# 终端 A：回放数据并持续发布 /clock
ros2 bag play exp16_full --clock --loop

# 终端 B：启动 rviz，让视图也认仿真时间
rviz2 --ros-args -p use_sim_time:=true

# rviz 界面：菜单 Panels → Add New Panel → "Time"
#   → Time 面板应显示与 bag 回放同步的时间（而非当前系统时间）

# 再按包内容添加 Display 面板：
#   有 LaserScan 数据 → 加 LaserScan Display
#   有 TF 数据        → 加 TF Display
#   有 Path 数据      → 加 Path Display
#   有图像话题        → 加 Image Display（Topic: /camera/color/image_raw）

# 防御性检查：时间对不对 —— 必须用【带 header 的话题】！
ros2 topic delay /scan     # 负值 = 时钟没对齐
# ⚠ 不能对 /chatter 用 —— 它没有 header 字段，会报：
#   "msg does not have header" → 换有 header 的传感器话题（如 /scan、/odom）
````

## EXP-17 · Launch 启动文件 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-17-launch/index.html)

````text
$ ros2 launch car1_description display.launch.py
````

````text
# 案例：同一份 launch 管"改名 + 开关 + 接续"三件事
rear_lidar = Node(package='lidar_driver', executable='lidar',
                  name='rear_lidar', output='screen',
                  remappings=[('/scan', '/scan_rear')])    # 1) 改名：源码发 /scan，出来叫 /scan_rear
diag       = Node(package='diag_tools', executable='scan_diag',
                  name='scan_diag',
                  condition=IfCondition(
                      LaunchConfiguration('use_sim')))     # 2) 条件：use_sim:=true 才起
LaunchDescription([
    DeclareLaunchArgument('use_sim', default_value='false'),
    rear_lidar, diag,
    RegisterEventHandler(                                # 3) 事件：rear_lidar 退出 → 起补位驱动
        OnProcessExit(target_action=rear_lidar,
                      on_exit=[Node(package='lidar_driver',
                                    name='rear_lidar_backup', ...)])),
])
# 运行：ros2 launch pkg demo.launch.py use_sim:=true
````

````text
# ① 导入：剧本类、节点 Action、包路径工具
from launch import LaunchDescription
from launch_ros.actions import Node
from ament_index_python.packages import get_package_share_directory
import os


def generate_launch_description():      # ② 入口函数：名字必须一字不差，ros2 launch 靠它找剧本
    pkg_share = get_package_share_directory('car1_description')
    urdf_file = os.path.join(pkg_share, 'urdf', 'car1.urdf')

    with open(urdf_file, 'r') as f:
        robot_desc = f.read()                # ③ 读入 URDF —— 模型就是这一大段 XML 字符串

    rviz_config = os.path.join(pkg_share, 'rviz', 'car1.rviz')   # rviz 布局文件

    return LaunchDescription([
        # ④ 节点 1：关节滑条窗口（截图左侧），发布各关节角
        Node(
            package='joint_state_publisher_gui',
            executable='joint_state_publisher_gui',
            name='joint_state_publisher_gui',
            parameters=[{'robot_description': robot_desc}]
        ),
        # ⑤ 节点 2：拿 URDF + 关节角，发布 TF（车架坐标系）
        Node(
            package='robot_state_publisher',
            executable='robot_state_publisher',
            name='robot_state_publisher',
            parameters=[{'robot_description': robot_desc}],
            output='screen'                # 日志进 launch 终端，带 [robot_state_publisher-2] 前缀
        ),
        # ⑥ 节点 3：按 car1.rviz 配置打开 RViz，小车画面就在里面（截图右侧）
        Node(
            package='rviz2',
            executable='rviz2',
            name='rviz2',
            arguments=['-d', rviz_config],     # -d：启动时加载指定配置文件
            output='screen'
        )
    ])
````

## EXP-18 · 元功能包 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-18-metapackage/index.html)

````text
# 元包 navigation2：本体没有代码，清单里全是运行时依赖
$ ros2 pkg xml navigation2 | grep exec_depend
  <exec_depend>nav2_amcl</exec_depend>
  <exec_depend>nav2_planner</exec_depend>
  <exec_depend>nav2_controller</exec_depend>
  # ……一口气 20 余行：这就是它"聚合"的全部内容

# 普通包 nav2_planner：有代码要编译，所以多了 build_depend
$ ros2 pkg xml nav2_planner | grep -E "depend"
  <build_depend>nav2_core</build_depend>
  <depend>nav2_msgs</depend>
  <exec_depend>nav2_costmap_2d</exec_depend>
  # 它还有 src/ 与编译目标——这正是两类包的分界线
````

````text
# ① 进工作空间创建（必须 ament_cmake；选 ament_python 会去找 setup.py 直接失败）
$ cd ros2_ws/src/xflz_eai_car1
$ ros2 pkg create car1_meta --build-type ament_cmake

# ② package.xml：换成场景四个包，export 段打上元包标记
<package format="3">
  ...
  <exec_depend>car1_base_driver</exec_depend>
  <exec_depend>controller</exec_depend>
  <exec_depend>car1_description</exec_depend>
  <exec_depend>car1_navigation</exec_depend>
  <export>
    <build_type>ament_cmake</build_type>
    <metapackage/>
  </export>
</package>

# ③ CMakeLists.txt 只留四行（project 换成 car1_meta）
cmake_minimum_required(VERSION 3.8)
project(car1_meta)
find_package(ament_cmake REQUIRED)
ament_package()

# ④ 编译（约 0.5 秒，没代码可编，属正常）→ source → 回读验证
$ colcon build --packages-select car1_meta
$ source install/setup.bash
$ ros2 pkg xml car1_meta | grep exec_depend
  <exec_depend>car1_base_driver</exec_depend>
  <exec_depend>controller</exec_depend>
  <exec_depend>car1_description</exec_depend>
  <exec_depend>car1_navigation</exec_depend>
  # 四行齐 = 成功，这就是 car1_meta 的全部"内容"
````

````text
# ① 装打包工具（一台机器装一次即可）
$ sudo apt install python3-bloom build-essential

# ② 进入元包目录：补构建依赖 → 生成 Debian 打包骨架
$ cd ros2_ws/src/xflz_eai_car1/car1_meta
$ rosdep install --from-paths . --ignore-src -y
$ bloom-generate rosdebian --ros-distro humble

# ③ 真正产出 .deb（产物落在包目录的上一级）
$ fakeroot debian/rules binary
  # → ../ros-humble-car1-meta_0.1.0-1jammy..._amd64.deb

# ④ 本机试装，并查看它到底会拉起哪些子包
$ sudo dpkg -i ../ros-humble-car1-meta_*.deb
$ apt show ros-humble-car1-meta | grep Depends
  Depends: ros-humble-car1-base-driver, ros-humble-controller,
           ros-humble-car1-description, ros-humble-car1-navigation, ...
  # 这串 Depends 就是"装我=装全家"在 apt 里的真身
````

## EXP-19 · TF2 坐标系变换 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-19-tf2/index.html)

````text
ros2 launch car1_description display.launch.py
# [INFO] [joint_state_publisher_gui]: ...        ← 关节滑块窗弹出
# [INFO] [robot_state_publisher]: ...            ← TF 发布者已上线
# [INFO] [rviz2]: ...                            ← rviz2 主窗弹出（Grid + 小车 + TF 轴）
````

````text
source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash
ros2 run tf2_tools view_frames
````

````text
map                              ← 世界原点（SLAM 维护）
 └── odom                         ← 里程计原点（会漂移）
      └── base_link               ← 车体中心
           ├── laser_frame        ← 雷达（安装位置 = 静态变换）
           ├── camera_link        ← 相机
           └── arm_base
                └── gripper_link  ← 夹爪
````

````text
# 绕 Z 轴旋转 yaw：四元数只有 z、w 两个分量非零
z = sin(yaw / 2)
w = cos(yaw / 2)   # x = y = 0

# yaw = 90°  ⇒  z = sin45° = 0.7071,  w = cos45° = 0.7071
# yaw = 180° ⇒  z = sin90° = 1.0,     w = cos90° = 0.0
# 反解：yaw = 2·atan2(z, w)   ← 看代码时反过来算角度就用它
````

````text
<link name="base_footprint" />                    <!-- 根坐标系：车在地面的投影点 -->
<joint name="base_joint" type="fixed">            <!-- 第一条边，静态 -->
  <origin xyz="0 0 0.029" rpy="0 0 0" />          <!-- 边上存的量：z 抬高 2.9cm -->
  <parent link="base_footprint" />                <!-- 父 -->
  <child link="base_link" />                      <!-- 子：child 只出现一次 = 单父 -->
</joint>
<joint name="wheel_lf_joint" type="continuous">   <!-- 活动边：随时在变 -->
  <parent link="base_link" /><child link="wheel_lf_link" />
  <axis xyz="0 1 0" />                            <!-- 绕 y 轴滚 -->
</joint>
````

````text
with open(urdf_file, 'r') as f:
    robot_desc = f.read()                                # ① 整份 URDF 读成字符串

LaunchDescription([
    Node(package='joint_state_publisher_gui', ...        # ② 滑块窗：活动关节角的输入源
         parameters=[{'robot_description': robot_desc}]),
    Node(package='robot_state_publisher', ...            # ③ 树的发布者（本课主角）
         parameters=[{'robot_description': robot_desc}], output='screen'),
    Node(package='rviz2', arguments=['-d', rviz_config]) # ④ 消费者：读两条话题画轴
])
````

````text
ros2 node list
# /joint_state_publisher_gui   ← 滑块窗（活动关节角来源）
# /robot_state_publisher       ← 树的发布者（本课主角）
# /rviz2                       ← 消费者：读 /tf、/tf_static 画坐标轴
````

````text
base_footprint                          # 根
└── base_link                           # base_joint · fixed（静态边）
    ├── wheel_lf/rf/lb/rb_link …        # 4×continuous（动态边）
    ├── laser_link                      # laser_joint · fixed（静态边）
    ├── imu_link                        # imu_joint · fixed
    └── arm_base_link → arm_link1…5     # revolute 链（动态边）
        └── camera… / tcp_link          # 相机链挂在 arm_link4 下
````

````text
<!-- 雷达安装位：纯平移，车体前方 14.6cm、高 8.7cm -->
<joint name="laser_joint" type="fixed">
  <origin xyz="0.146213088049965 0 0.0874000000000005" rpy="0 0 0" />
  <parent link="base_link" /><child link="laser_link" />
</joint>

<!-- 深度相机光学系：rpy 把“z 朝前”的车体系扭成光学约定 -->
<joint name="camera_depth_optical_joint" type="fixed">
  <origin xyz="0 0.01655 0" rpy="-1.57079633 0 -1.57079633" />
  <parent link="camera_depth_link" /><child link="camera_depth_optical_frame" />
</joint>
````

````text
ros2 topic echo /tf_static --qos-durability transient_local
# transforms:                            ← 数组，每条 fixed 边一项
# - header: {frame_id: base_footprint}   ← 父
#   child_frame_id: base_link             ← 子
#   transform: {translation: {z: 0.029},
#               rotation: {w: 1.0}}      ← 数值 = URDF 里的原值
# - {frame_id: base_link, child_frame_id: laser_link, ...}
````

````text
# 假想雷达：车体前方 10cm、高 20cm、姿态不转
ros2 run tf2_ros static_transform_publisher \
  --x 0.1 --y 0 --z 0.2 \
  --yaw 0 --pitch 0 --roll 0 \
  --frame-id base_link --child-frame-id laser_frame
# --x/--y/--z           平移，单位米
# --yaw/--pitch/--roll  欧拉角，单位弧度（90° = 1.5708）
# --frame-id            父坐标系；--child-frame-id 子坐标系（别写反！）
# 旧式裸位置参数已弃用，与 flag 混用直接报错退出
````

````text
import rclpy
from rclpy.node import Node
from geometry_msgs.msg import TransformStamped
from tf2_ros import StaticTransformBroadcaster

class StaticTfPub(Node):
    def __init__(self):
        super().__init__('static_tf_pub')
        # ① StaticTransformBroadcaster —— 内部自动用 /tf_static + TransientLocal
        self.pub = StaticTransformBroadcaster(self)
        self.send()

    def send(self):
        t = TransformStamped()
        t.header.stamp = self.get_clock().now().to_msg()
        t.header.frame_id  = 'base_link'   # 父
        t.child_frame_id   = 'laser_frame'  # 子（只能有一个父！）
        t.transform.translation.x = 0.10    # 雷达在车体前方 10cm
        t.transform.translation.z = 0.20    # 高度 20cm（TF2 支持 3D）
        # 无旋转 ⇒ 单位四元数 (x=0, y=0, z=0, w=1)
        t.transform.rotation.w = 1.0
        self.pub.sendTransform(t)   # 只发这一次，后加入的订阅者也能收到
````

````text
<joint name="wheel_lf_joint" type="continuous">     <!-- 轮子：可无限转 -->
  <origin xyz="0.0973 0.1058 0.0195" rpy="0 0 0" />
  <parent link="base_link" /><child link="wheel_lf_link" />
  <axis xyz="0 1 0" />                              <!-- 绕 y 滚动 -->
</joint>
<joint name="arm_joint1" type="revolute">            <!-- 机械臂第一轴：正负限位 -->
  <parent link="arm_base_link" /><child link="arm_link1" />
  <axis xyz="0 0 1" />                              <!-- 绕 z 转 -->
  <limit lower="-3.14" upper="3.14" effort="100" velocity="1" />
</joint>
````

````text
ros2 topic hz /tf
# average rate: 33.3xx          ← 每秒约 33 条（随滑块活动波动）
# min: …s  max: …s  window: …s  ← 窗口内最快/最慢间隔
````

````text
ros2 topic echo /tf
# - header: {stamp: {sec: ...}, frame_id: base_link}  ← 时间戳持续前进
#   child_frame_id: wheel_lf_link                      ← 活动关节的子系
#   transform: {translation: {...},
#               rotation: {x, y, z, w}}               ← 数值随滑块实时变
````

````text
ros2 topic echo /joint_states
# header: {stamp: {sec: ...}}              ← 拖滑块时时间戳前进
# name: [arm_joint1, wheel_lb_joint, ...]   ← 关节名列表（对上 URDF 的 joint）
# position: [0.52, -1.20, ...]              ← 各关节当前角（弧度）——拖哪个，哪个变
````

````text
sudo apt install ros-humble-rqt-tf-tree      # ① 只需一次；不装则 ② 直接 command not found
ros2 run rqt_tf_tree rqt_tf_tree             # ② 打开常驻树窗口
# 树画出来后：拖 arm_joint1 滑块 → 点 ⟳ 刷新
#   · 树形不变 = 拓扑由 URDF 定死，拖滑块不增删边（正常！）
#   · 变的是边标签： Most recent transform 时间戳往前走、Average rate 显示在频发
# 数值平移/转角在第 5 步 tf2_echo，轴的转动在刚才的 RViz——三窗同看才完整
````

````text
ros2 run tf2_ros tf2_echo base_footprint base_link
# At time 1792054921.106               ← 查询时刻（默认取最新；TF 全程带时间戳）
# Translation: [0.000, 0.000, 0.029]    ← 平移，单位米 = URDF base_joint 的 z
# Rotation: in RPY (radian): [0.000, -0.000, 0.000]   ← 姿态，弧度；全 0 = 无旋转
# （还会持续打印 Quaternion x y z w 与 4×4 Matrix：左上 3×3 旋转、第四列平移）
````

````text
# 查雷达安装位（静态边）
ros2 run tf2_ros tf2_echo base_link laser_link
#   Translation ≈ [0.146, 0.000, 0.087]     ← URDF laser_joint 的 xyz 原值

# 两参数对调 = 求逆：数字取反
ros2 run tf2_ros tf2_echo base_link base_footprint
#   Translation ≈ [0.000, 0.000, -0.029]    ← 第一条查询的逆

# 查活动边：拖 arm_joint1 滑块，输出实时变
ros2 run tf2_ros tf2_echo base_link arm_link1
````

````text
from tf2_ros import Buffer, TransformListener

self.buffer = Buffer()
self.listener = TransformListener(self.buffer, self)   # 监听 /tf + /tf_static 填充 buffer

# 查询 laser_frame 的数据 → map 系；超时 1.0s（树还没齐时阻塞等待）
t = self.buffer.lookup_transform(
    'map', 'laser_frame',
    rclpy.time.Time(),      # Time()=最新时刻；传具体时间则做历史插值
    timeout=rclpy.duration.Duration(seconds=1.0))
tx = t.transform.translation.x
qz, qw = t.transform.rotation.z, t.transform.rotation.w
yaw = 2.0 * math.atan2(qz, qw)   # 03 节半角公式反解
````

## EXP-20 · URDF 与 Xacro 机器人建模 — 机器人 ROS2 应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp003-ros2-application/exp-20-urdf/index.html)

````text
节选自 car1.urdf 开头（robot 标签横跨全文到文件末尾）
<?xml version="1.0" encoding="utf-8"?>
<robot name="car1">
  <!-- 空 link：没有几何也没有质量，只当坐标系用——base_footprint 是整车"落地点" -->
  <link name="base_footprint" />
  <!-- 同款空 link 还有 tcp_link（末端工具点）、camera_depth_optical_frame（相机光轴系）…全文十几处 -->
</robot>
````

````text
节选自 car1.urdf 的 base_link（惯性矩阵共 9 个数，此处原样摘录）
<link name="base_link">
  <!-- inertial：动力学读它——质量 1.97 kg + SolidWorks 算好的 3×3 惯性矩阵 -->
  <inertial>
    <origin xyz="0.00119523538791205 0.00195957701632652 0.0457542650231294" rpy="0 0 0"/>
    <mass value="1.97049077979221"/>
    <inertia ixx="0.00486152635724755" ixy="2.00512337711509E-06" ixz="0.000162020407436135"
             iyy="0.00915836901735741" iyz="2.72820673876569E-06" izz="0.0130028044572217"/>
  </inertial>
  <!-- visual：RViz2 读它——STL 网格 + 颜色（rgba 值域是 [0,1]，不是 0~255） -->
  <visual>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry><mesh filename="package://car1_description/meshes/base_link.STL"/></geometry>
    <material name=""><color rgba="1 1 1 1"/></material>
  </visual>
  <!-- collision：Gazebo 碰撞检测读它，这里直接复用同一 STL（真实工程可简化成包围盒） -->
  <collision>
    <origin xyz="0 0 0" rpy="0 0 0"/>
    <geometry><mesh filename="package://car1_description/meshes/base_link.STL"/></geometry>
  </collision>
</link>
````

````text
节选自 car1.urdf：base_footprint 与 base_link 之间的"垫高"关节
<joint name="base_joint" type="fixed">
  <origin xyz="0 0 0.029" rpy="0 0 0"/>  <!-- 子 link 相对父 link：仅沿 Z 抬高 2.9 cm -->
  <parent link="base_footprint"/>
  <child link="base_link"/>
</joint>
````

````text
节选自 car1.urdf：深度相机末端的光学坐标系关节
<joint name="camera_depth_optical_joint" type="fixed">
  <origin xyz="0 0.01655 0"
          rpy="-1.57079633 0 -1.57079633"/>  <!-- -π/2 ≈ -1.5708，两个 90° 把车体系掰成相机光学系约定 -->
  <parent link="camera_depth_link"/>
  <child link="camera_depth_optical_frame"/>
</joint>
````

````text
节选自 car1.urdf：左前驱动轮（其余三个轮同构，仅 name/xyz 不同）
<joint name="wheel_lf_joint" type="continuous">
  <origin xyz="0.097300000000001 0.105834744636228 0.0195000000000009" rpy="0 0 0"/>
  <parent link="base_link"/>
  <child link="wheel_lf_link"/>
  <axis xyz="0 1 0"/>  <!-- 轮轴沿 Y（车宽方向）——绕它一直转 -->
</joint>
````

````text
节选自 car1.urdf：机械臂第一轴（arm_joint2~5、夹爪诸关节同款写法）
<joint name="arm_joint1" type="revolute">
  <origin xyz="0.0100000000000003 0 0.0434999999999999" rpy="0 0 0"/>
  <parent link="arm_base_link"/>
  <child link="arm_link1"/>
  <axis xyz="0 0 1"/>  <!-- 绕 Z 转（肩关节回转） -->
  <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>  <!-- 有限转动，四件套写全 -->
</joint>
````

````text
节选自 car1.urdf：左外指关节，从动于右关节
<joint name="l_joint" type="revolute">
  <origin xyz="0.01900706 0.01375021 0.0025" rpy="0 0 0"/>
  <parent link="arm_link6"/>
  <child link="l_link"/>
  <axis xyz="0 0 -1"/>
  <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>
  <mimic joint="r_joint" multiplier="-1" offset="0"/>  <!-- 跟随 r_joint：角度 ×(-1) 反向联动 -->
</joint>
````

````text
<joint name="wheel_joint" type="continuous">
  <parent link="base_link"/>
  <child link="wheel_link"/>
  <axis xyz="0 1 0"/>  <!-- 只给转轴，不写 limit —— 它可以一直转 -->
</joint>
````

````text
# car1 机械臂第一轴的原值四件套（02 节例 6 同款）
<joint name="arm_joint1" type="revolute">
  <parent link="arm_base_link"/>
  <child link="arm_link1"/>
  <axis xyz="0 0 1"/>
  <limit lower="-3.14" upper="3.14" effort="100" velocity="1"/>  <!-- 弧度限位 + 最大力矩 + 最大角速度 -->
</joint>
````

````text
<joint name="lift_joint" type="prismatic">
  <parent link="base_link"/>
  <child link="lift_link"/>
  <axis xyz="0 0 1"/>  <!-- 沿 Z 直线推 -->
  <limit lower="0" upper="0.3" effort="50" velocity="0.2"/>  <!-- 行程 0~0.3 m，单位是米！ -->
</joint>
````

````text
robot_description = open(urdf_path).read()
Node(package='robot_state_publisher',
     parameters=[{'robot_description': robot_description}])
Node(package='joint_state_publisher_gui')
````

````text
<robot xmlns:xacro="http://ros.org/wiki/xacro">
    <!-- Gazebo 物理参数宏：params="name" 声明形参，${name} 是调用处传入的实参 -->
    <xacro:macro name="gazebo_link" params="name">
        <gazebo reference="${name}">
            <kp>0</kp><kd>0</kd>
            <mu1>1</mu1><mu2>1</mu2>  <!-- 摩擦系数 -->
            <material>Gazebo/White</material>
        </gazebo>
    </xacro:macro>

    <!-- 夹爪换高刚度宏：kp=1e15 抓得稳、kd=1e13 减震——换个宏名即换整套参数 -->
    <xacro:macro name="gazebo_gripper" params="name">
        <gazebo reference="${name}">
            <kp>1e15</kp><kd>1e13</kd>
            <mu1>1.0</mu1><mu2>1.0</mu2>
            <material>Gazebo/Black</material>
        </gazebo>
    </xacro:macro>

    <!-- 宏调用：像普通标签一样使用；${prefix} 拼在每个名字前，多车不撞名 -->
    <xacro:macro name="car1_gazebo" params="prefix">
        <xacro:gazebo_link name="${prefix}arm_base_link"/>
        <xacro:gazebo_link name="${prefix}arm_link1"/>  …（7 节臂各一行）
        <xacro:gazebo_gripper name="${prefix}r_link"/>  …（6 个夹爪件）
        <xacro:gazebo_link name="${prefix}base_link"/>
    </xacro:macro>
</robot>
````

````text
<robot name="car1" xmlns:xacro="http://ros.org/wiki/xacro">
    <!-- 命令行/launch 可传参数：$(arg prefix) 逐处取值 -->
    <xacro:arg name="prefix" default=""/>

    <!-- 兼容实锤：include 的目标是一份普通 .urdf 文件，原样引入、一行不改 -->
    <xacro:include filename="$(find car1_description)/urdf/car1.urdf"/>

    <!-- 再叠加物理、传动、硬件接口三份宏文件 -->
    <xacro:include filename="$(find car1_description)/gazebo/car1.gazebo.xacro"/>
    <xacro:include filename="$(find car1_description)/gazebo/car1.transmission.xacro"/>
    <xacro:include filename="$(find car1_gazebo)/urdf/car1.gazebo.ros2_control.xacro"/>

    <!-- 宏调用：把物理参数/传动/硬件刷到每个 link、joint 上 -->
    <xacro:car1_gazebo prefix="$(arg prefix)"/>
    <xacro:car1_transmission prefix="$(arg prefix)"/>
    <xacro:car1_gazebo_ros2_control name="GazeboSimSystem"/>

    …（其后是 Gazebo<plugin>、相机<sensor> 等原生标签，按原样保留到文件尾）…
</robot>
````

````text
# 输出 ①：三行 = car1.urdf 解析通过；根 link 是 base_footprint，挂着 1 个直接子件（base_link）
$ check_urdf $(ros2 pkg prefix car1_description)/share/car1_description/urdf/car1.urdf
robot name is: car1
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)

# 代码段 ②：launch 怎么把文件送进节点（car1_description/launch/display.launch.py 核心 8 行）
pkg_share = get_package_share_directory('car1_description')
urdf_file = os.path.join(pkg_share, 'urdf', 'car1.urdf')
with open(urdf_file, 'r') as f:
    robot_desc = f.read()                      # 文件全文 → 字符串
Node(package='robot_state_publisher',
     parameters=[{'robot_description': robot_desc}])   # 注入参数（缺这步 = 黑屏）
Node(package='joint_state_publisher_gui', …)          # 发 /joint_states，带滑块
Node(package='rviz2', arguments=['-d', rviz_config])  # 用预配 car1.rviz 画出来

# 输出 ③：一键拉起三节点（launch 已写好，无需改）
$ ros2 launch car1_description display.launch.py
# RViz2：Fixed Frame 选 base_footprint → 底盘+4轮+机械臂+夹爪整体出现
# 拖滑块 arm_joint1   → 转到 ±3.14 rad 就停 ……… revolute 有限转动（02 节例 6）
# 拖滑块 wheel_lf_joint → 转圈没有尽头 …………… continuous 一直转动（02 节例 5）
````

````text
# ① 命令行展开 + 体检（car1_gazebo 总装文件 = 03 节案例 B，内部 include 了 car1.urdf）
$ sudo apt install ros-humble-xacro -y
$ xacro $(ros2 pkg prefix car1_gazebo)/share/car1_gazebo/urdf/car1.gazebo.xacro \
    > /tmp/car1_expanded.urdf
$ check_urdf /tmp/car1_expanded.urdf
robot name is: car1
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)
# 展开件与原文件同样三行通过 = 宏、变量、include 已全部翻译成标准 URDF

# ② 工程 launch 的真实做法（car1_gazebo/launch/gazebo_arm.launch.py 原代码）
urdf_content = subprocess.check_output(['xacro', xacro_file]).decode('utf-8')
Node(package='robot_state_publisher',
     parameters=[{'robot_description': urdf_content}])

# ③ 两条路线并排看——注入方式完全相同，唯一区别是文本从哪来
URDF 路线：open('car1.urdf').read()            ──┐
Xacro 路线：xacro 总装.xacro 的展开输出文本   ──┼─→ robot_description ─→ 三节点 ─→ RViz2/Gazebo
                                              ┘  下游感知不到差别 = 03 节"双向兼容"的落地
````

````text
# ① 语法+结构验证（步骤 ① 的原命令，汇总在此）
$ check_urdf $(ros2 pkg prefix car1_description)/share/car1_description/urdf/car1.urdf
robot name is: car1
---------- Successfully Parsed XML ---------------
root Link: base_footprint has 1 child(ren)

# ② 生成 link-joint 拓扑图（需 liburdfdom-tools）
$ sudo apt install liburdfdom-tools -y
$ urdf_to_graphiz /tmp/car1_expanded.urdf
# → 生成 PDF：base_footprint→base_link 分叉到 4 轮 / 激光 / IMU / 6 节臂 / 夹爪

# ③ Xacro 展开件复检（步骤 ② 的产物，两条命令即闭环）
$ xacro $(ros2 pkg prefix car1_gazebo)/share/car1_gazebo/urdf/car1.gazebo.xacro \
    > /tmp/car1_expanded.urdf
$ check_urdf /tmp/car1_expanded.urdf
# 同样三行通过 = 展开产物是合法标准 URDF（宏语法无误、include 拼接成功）
````

## EXP-01 · SLAM 原理与 URDF 模型 — 机器人 SLAM 与自主导航

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-01-slam-urdf/index.html)

````text
# ① 启动模型可视化（本实验核心命令，一条命令拉起三个节点）
ros2 launch car1_description display.launch.py

# ② 导出 TF 坐标树：生成 frames_<时间戳>.pdf 与 frames.gv（在命令执行的当前目录）
ros2 run tf2_tools view_frames

# ③ 数一数 URDF 里有多少连杆与关节（真实值：17 link / 21 joint）
grep -c "<link name=" car1.urdf
grep -c "<joint name=" car1.urdf

# ④（接实物）确认雷达数据在发
ros2 topic hz /scan
# 约 10.0 Hz

# car1.urdf 关键片段：根节点贴地，fixed 抬升 0.029 m 接车体
<link name="base_footprint" />
<joint name="base_joint" type="fixed">
  <origin xyz="0 0 0.029" rpy="0 0 0" />
  <parent link="base_footprint" />
  <child link="base_link" />
</joint>
````

## EXP-02 · SLAM Toolbox 建图 — 机器人 SLAM 与自主导航

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-02-slam-toolbox/index.html)

````text
# ① 启动 SLAM 建图（含底盘 bringup、雷达、10 秒延迟起的 slam_toolbox 与 rviz）
ros2 launch car1_slam slam.launch.py
#   不想开 rviz：追加 use_rviz:=false

# ② 另开终端：键盘遥控（w/s 前后 0.2m/s，a/d 左右转 0.5rad/s，空格停）
ros2 run car1_slam slam_teleop
#   注意：必须在真终端（tty）里运行，IDE 内嵌终端读不到按键

# ③ 逛圈前自检：雷达 10Hz、里程计 TF 正常
ros2 topic hz /scan
ros2 run tf2_ros tf2_echo odom base_footprint

# ④ 保存地图（逛完圈、触发回环之后再执行）
ros2 run car1_slam map_save --ros-args -p map_name:=map_01
#   默认写到 install/.../car1_slam/maps/ ；想改目录再加 -p map_dir:=/绝对/路径
#   注意 map_save 读的是参数 map_name，不是位置参数——直接写 map_01 不会生效

# 方式二：nav2 官方 CLI（transient_local 才能收到 latched 的 /map）
ros2 run nav2_map_server map_saver_cli -f map_01 \
  --ros-args -p map_subscribe_transient_local:=true

# ⑤ 停掉遥控（Ctrl+C 前先保存）
pkill -f slam_teleop
````

## EXP-03 · Nav2 机器人导航 — 机器人 SLAM 与自主导航

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-03-nav2-navigation/index.html)

````text
# ① 启动导航（map 参数是 maps/ 目录下的地图名，不含 .yaml 后缀）
ros2 launch car1_navigation navigation.launch.py map:=map_01

# ② 自检五大服务器与定位节点是否就位
ros2 node list | grep -E "planner|controller|bt_navigator|behavior|amcl|map_server|velocity_smoother"

# ③ 查看某服务器生命周期状态（预期 active）
ros2 lifecycle get /planner_server

# ④ 命令行发一个目标（与 RViz 点目标等价）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}}}"

# ⑤ 看速度有没有真的下发到车（注意：平滑后的名字是 controller/cmd_vel）
ros2 topic echo /controller/cmd_vel
# 现场手势：rviz 顶部工具栏 2D Pose Estimate → Nav2 Goal（若配置里是 2D Goal Pose 同理）
````

## EXP-04 · AMCL 自适应蒙特卡洛定位 — 机器人 SLAM 与自主导航

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-04-amcl-localization/index.html)

````text
# ① 启动导航（AMCL 随 localization.launch 起播，参数 set_initial_pose=true）
ros2 launch car1_navigation navigation.launch.py map:=map_01

# ② 观察粒子云与位姿话题（另开终端；注意话题名是 particle_cloud，没有下划线连写）
ros2 topic echo /particle_cloud --once
ros2 topic echo /amcl_pose --once

# ③ 命令行下发初始位姿（与 RViz 按钮等价）——朝向四元数 w=1 即 yaw 0
ros2 topic pub -1 /initialpose geometry_msgs/msg/PoseWithCovarianceStamped \
  "{header: {frame_id: 'map'}, pose: {pose: {position: {x: 0.5, y: 0.3}, orientation: {w: 1.0}}}}"

# ④ 绑架测试：手动把车搬到 2m 外，重新 pub /initialpose，秒表记录恢复时间
#    对比两种做法：① 手动重给位姿  ② 开 recovery_alpha_fast 0.001 让算法自己找回

# ⑤ 调优后必须整栈重启（Ctrl+C 再 launch），别热改参数
````

## EXP-05 · DWA/TEB 局部避障 — 机器人 SLAM 与自主导航

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-05-dwa-teb-avoidance/index.html)

````text
# ① 启动导航（含定位），map 指向 EXP-02 保存的地图名
ros2 launch car1_navigation navigation.launch.py map:=map_01

# ② 命令行下发穿过房间的目标（也可用 RViz 工具栏 Nav2 Goal）
ros2 action send_goal /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 2.0, y: 1.0}, orientation: {w: 1.0}}}}"

# ③ 实时观察速度变化（减速→绕行→回升；平滑后的名字是 controller/cmd_vel）
ros2 topic echo /controller/cmd_vel

# ④ RViz 添加 Display → By topic → /local_costmap/costmap（Color 显示）
#    放纸箱后应看到障碍处黑圈（254）与周围 1-252 渐变

# ⑤ 调参：改 nav2_params.yaml 的 controller_server / local_costmap 段后必须整栈重启
ros2 launch car1_navigation navigation.launch.py map:=map_01
````

## EXP-06 · 激光雷达多点导航避障 — 机器人 SLAM 与自主导航

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-06-radar-multiwaypoint/index.html)

````text
# ① 启动导航
ros2 launch car1_navigation navigation.launch.py map:=map_01

# ② 查看 action 定义（goal / feedback / result）
ros2 interface show nav2_msgs/action/NavigateToPose
ros2 interface show nav2_msgs/action/NavigateThroughPoses   # goal 是 poses[] 数组

# ③ 单点导航（frame_id 必须是 map）+ 实时反馈
ros2 action send_goal --feedback /navigate_to_pose nav2_msgs/action/NavigateToPose \
  "{pose: {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 0.5}, orientation: {w: 1.0}}}}"

# ④ 多点连续导航：A→B→C 一次下发（注意点是"掠过"而非"逐点停"）
ros2 action send_goal /navigate_through_poses nav2_msgs/action/NavigateThroughPoses \
  "{poses: [{header: {frame_id: 'map'}, pose: {position: {x: 0.0, y: 0.5}, orientation: {w: 1.0}}},
            {header: {frame_id: 'map'}, pose: {position: {x: 1.0, y: 1.0}, orientation: {w: 1.0}}},
            {header: {frame_id: 'map'}, pose: {position: {x: 1.5, y: 0.5}, orientation: {w: 1.0}}}]}" --feedback

# ⑤ 跟踪轨迹与到达误差
ros2 topic echo /amcl_pose     # 时间序列可绘制成轨迹图验证"依次经过"

# ⑥ 观察恢复动作有没有真的触发
ros2 topic echo /behavior_tree_log
````

````text
# ⑦ 调度脚本骨架：目标序列 + 失败重试（SUCCEEDED 才继续）
goals = [g1, g2, g3]
for g in goals:
    result = send_navigate_to_pose(g)          # 阻塞等待完成
    if result == SUCCEEDED:
        print(f"目标 {g} 完成")
    else:
        print(f"目标 {g} 失败 → 重试 1 次")
        result = send_navigate_to_pose(g)      # 仍失败则记录/跳过，禁止静默忽略
````

## EXP-07 · RTAB-Map 3D SLAM — 机器人 SLAM 与自主导航

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp004-slam-navigation/exp-07-rtabmap-3d/index.html)

````text
# ① 启动相机（命名空间 ascamera_hp60c），核对四话题
ros2 launch ascamera hp60c.launch.py
ros2 topic list | grep ascamera_hp60c
#   /ascamera_hp60c/camera_publisher/rgb0/image
#   /ascamera_hp60c/camera_publisher/depth0/image_raw
#   /ascamera_hp60c/camera_publisher/depth0/camera_info
ros2 topic hz /ascamera_hp60c/camera_publisher/depth0/image_raw
ros2 topic echo /ascamera_hp60c/camera_publisher/depth0/camera_info   # 抄 fx fy cx cy

# ② 查看深度图（近红远蓝，反光/超距处发黑）
ros2 run rqt_image_view rqt_image_view /ascamera_hp60c/camera_publisher/depth0/image_raw

# ③ 启动 RTAB-Map（通用命令：话题名不匹配就显式指定）
ros2 run rtabmap_slam rtabmap --ros-args -p subscribe_rgbd:=true \
  -p rgb_topic:=/ascamera_hp60c/camera_publisher/rgb0/image \
  -p depth_topic:=/ascamera_hp60c/camera_publisher/depth0/image_raw \
  -p approx_sync:=true

# ④ 遥控低速走闭环（角速度 ≤ 0.5 rad/s，转角停留 1-2s）观察 RViz 点云
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ⑤ 保存数据库 / 导出八叉树
ros2 service call /rtabmap/save std_srvs/srv/Empty
ros2 launch octomap_server octomap_mapping.launch.xml   # 通用
````

````text
# 反投影手验算（numpy）：中心像素应得到 X≈0, Y≈0, Z=d
import numpy as np
fx, fy, cx, cy = 380.0, 380.0, 320.0, 240.0   # ← 以实测 camera_info 为准
u, v, d = 320.0, 240.0, 1.500                 # 深度换算成「米」再算
X = (u - cx) * d / fx
Y = (v - cy) * d / fy
Z = d
print(X, Y, Z)   # 期望 ≈ 0, 0, 1.5
````

## EXP-01 URDF 模型与仿真控制 — 机器人机械臂应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-01-urdf-simulation/index.html)

````text
# 在带有 GUI 界面的终端运行：实验箱屏幕、VNC 或 MobaxTerm（纯 SSH 黑窗弹不出图形窗口）
ros2 launch car1_description display.launch.py
````

````text
# 就这三步——注意每个新终端都要重新 source 两次
colcon build --packages-select car1_description
source install/setup.bash
ros2 launch car1_description display.launch.py
````

````text
<robot name="car1">
  <!-- ① 每段"骨骼" = 你看到的一个零件 -->
  <link name="base_link">
    <visual>    外形 + mesh 资源路径（洋红色=这里写错） </visual>
    <collision> 碰撞外壳（为将来的物理世界预留） </collision>
    <inertial>  质量与惯量（Gazebo 才真正用到） </inertial>
  </link>

  <!-- ② 每个"关节" = 你拖的一个滑块（以底座回转为例） -->
  <joint name="arm_joint1" type="revolute">   <!-- 旋转关节 -->
    <parent link="base_link"/>                <!-- 挂在谁身上 -->
    <child  link="link1"/>                     <!-- 长出谁 -->
    <origin xyz="0 0 0.1" rpy="0 0 0"/>       <!-- 相对父的安装位姿 -->
    <axis   xyz="0 0 1"/>                     <!-- 绕哪根轴转=滑块调的量 -->
    <limit  lower="-3.14" upper="3.14"         <!-- 数字止挡=03 节限位表出处 -->
            effort="…" velocity="…"/>
  </joint>

  <!-- ③ 末端与眼睛：坐标系立在各自 link 上 -->
  <link name="tcp_link"/>     <!-- 指尖参考点：IK 落点 -->
  <link name="camera_link"/>  <!-- 相机光心：视觉起点 -->

  <!-- ④ 夹爪 r_joint 也是旋转关节：URDF 管行程，实机开合另有 PWM 口径 -->
  <joint name="r_joint" type="revolute">
    <limit lower="-3.14" upper="3.14"/>  <!-- ±180°；实机 800~1500 μs，EXP-02 换算 -->
  </joint>
</robot>
````

````text
# 1) 节点都在吗：launch 的三兄弟 jsp_gui / rsp / rviz2 都在即合格
#    多出来的名字有来路，不算异常：
#    · transform_listener_impl_xxxx 是 rviz2 的 TF 监听器自动注册的框架节点
#      （后缀十六进制每次启动都变，不是谁手写的）
ros2 node list

# 2) 话题清单：确认出现 /joint_states 与 /tf（/tf_static）
ros2 topic list

# 3) 谁发谁收：预期 publisher=1、subscription≥1（rsp 在收）
ros2 topic info /joint_states

# 4) 打包内容：name 与 position 两个数组同序、下标对应
ros2 topic echo /joint_states --once

# 5) 边拖滑块边跑：频率 > 0 = 心跳健康；按住不放看 position 变化
ros2 topic hz /joint_states
````

````text
# 生成 TF 树：预期 7+ 坐标系一张 PDF，base_link 单根无环
ros2 run tf2_tools view_frames

# 查心跳：拖动滑块期间频率 > 0、position 数值在变；为 0 则模型不会更新
ros2 topic hz /joint_states
````

````text
# 顶替 GUI 当发布者——谁往 /joint_states 发消息，订阅方（robot_state_publisher）就听谁的
# GUI 开着会持续发布盖过你的单发：先关掉左窗再发（发完姿态常驻，直到下次有人发言）
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "{name: [arm_joint1], position: [0.0]}"     # 正前（零位）0 rad
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "{name: [arm_joint1], position: [0.785]}"    # 斜前一翼 +45°
ros2 topic pub --once /joint_states sensor_msgs/msg/JointState "{name: [arm_joint1], position: [-0.785]}"   # 斜前另一翼 −45°（哪边是前左自己记录）
````

## EXP-02 关节电机底层控制与应用 — 机器人机械臂应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-02-serial-pwm/index.html)

````text
# ① 清理占用串口的应用（本实验全程不启动 bringup！）
bash cleanup_ros.sh

# ② 确认设备与权限
ls -l /dev/stm32        # 小车换成 /dev/car1_stm32
# 预期：软链存在且可读写；打不开 → sudo chmod 666 /dev/stm32
dmesg | grep tty     # 找不到设备时看内核插入日志
````

````text
# 启动与判定
python3 test_servo_direct.py
# 成功标志：打印 打开串口 /dev/stm32 @ 1000000... 后出现 串口已打开
# 失败会提示三种可能：bringup 没停 / 设备不存在 / 权限不够（对照 04 节逐条排除）

# 可选：串口助手抓一条真帧逐字节看（观察任务）
cutecom --device /dev/stm32 --baud 1000000
# 看点：发 work 时应出现 AA 55 开头的一串字节；1Mbps 下字节会瞬间刷过，
# 抓完暂停再翻，对照 03 节数字段
````

````text
# 安全流程：先回家，再微调
work            # 第一条永远是它：回到已知安全位
s 1 1520        # 1 号舵机中位+20μs，微调神器（换算 ≈ +0.0471 rad）
g 800           # 张开夹爪；g 1500 全闭（与 EXP-05 的 servo 口径一致）
free            # 手掰摆位；load 锁回去
````

````text
# 导出与安装（YAML 默认落在包的 config/actions/ 下）
yaml             # 按提示输入动作组名，如 pick_front
colcon build --packages-select car1_arm_controller
# 不 build 的后果：EXP-05 的 action_player 找不到文件
````

## EXP-03 正运动学基础 · 机器人机械臂应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-03-kinematics/index.html)

````text
R(θ) = [  cos θ   sin θ ]
       [ −sin θ   cos θ ]
````

````text
第 1 段：v1 = R(q2)      · (0, L1)
第 2 段：v2 = R(q2+q3)   · (0, L2)
第 3 段：v3 = R(q2+q3+q4)· (0, L3)
指尖 = 起点 + v1 + v2 + v3
````

````text
p = R · p    （先转）
p = p + d    （再挪）
````

````text
T = [ R(3×3)   d(3×1) ]      左上角管转，最后一列管挪
    [   0 0 0     1    ]
````

````text
T_total = T_base · T_J1 · T_J2 · T_J3 · T_J4 · T_J5 · T_TCP

从 T_total 里直接读两样东西：
  TCP 位置 = 最后一列
  工具朝向 = 第 3 列
````

````text
T = np.eye(4)                      # 从单位矩阵开始，相当于底座

for joint in joints:
    Tx = 平移矩阵(joint.xyz)        # 挪到关节所在的位置
    Ra = 旋转矩阵(joint.axis, θ)    # 绕这根轴转 θ
    T  = T @ Tx @ Ra                # 累乘，在已有变换上再叠一层

tcp = T[:3, 3]                     # 最后一列就是 TCP 位置
````

````text
【TCP 位置】
  X: +0.157 m    Y: +0.000 m    Z: +0.342 m
【TCP 姿态】(Roll-Pitch-Yaw)
  Roll: 接近水平   Pitch: 水平向前   Yaw: 正前方
【TCP Z 轴方向】
  向量: (+1.000, +0.000, -0.000)   ← 工具指向正前方
````

## EXP-04 逆运动学基础与 IK 解算 · 机器人机械臂应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-04-ik-solver/index.html)

````text
cos(夹角) = (d² − L1² − L2²) / (2 · L1 · L2)
````

````text
cosQ3rel = (distSq - L1² - L2²) / (2·L1·L2)
cosQ3rel = max(-1.0, min(1.0, cosQ3rel))   # ← 超界值被静默夹到 ±1
q3rel_mag = math.acos(cosQ3rel)
````

````text
解 1：q2 = −50.024°   q3 = +105.019°   q4 = +35.004°
解 2：q2 = +54.996°   q3 = −105.019°   q4 = +140.024°
````

````text
猜一组关节角 q
重复：
    算出当前指尖在哪  →  和目标差多少 e
    用雅可比反算：关节该挪多少 Δq
    q = q + Δq
    差得够小了就停
````

````text
目标点 → IK → 一组关节角 → FK → 算出来的指尖位置
                                      ↓
                              和目标点比，差多少？
````

## EXP-05 ROS2 架构与动作回放 — 机器人机械臂应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-05-action-replay/index.html)

````text
# YAML 结构（必须与 EXP-02 的 export_yaml 输出一致，由 action_player._load_action 解析）
name: pick_front            # 动作组名（导出时输入的那个）
description: pick_front action
actions:
  - name: pre_grasp
    duration: 2.0            # 秒；臂段固定 2.0，夹爪段 1.0，home 段 3.0
    servos:
      0: 1500
      1: 1100
      2: 2200
      3: 1850
      4: 1500
  - name: gripper_open
    duration: 1.0
    servos:
      5: 800              # 夹爪单独一段：id5, PWM 800（开）
  - name: grasp_down
    ...
````

````text
# 归一化"频率/话题"预期（启动后自检用）
ros2 topic hz /joint_states     # 预期 50Hz
ros2 topic hz /odom             # 预期 30~50Hz
ros2 topic hz /imu/data         # 预期 100~200Hz
````

````text
# 单手发一条弧度指令，直击 joint_command（观察手臂动没动）
# 关节名必须是 arm_joint1~5（URDF 里的原名）；写成 joint1 控制器收不到——
# arm_controller 只按 msg.position 的"顺序"取前 5 个数，名字写错不会报错，只会静默跑到别处
ros2 topic pub /arm_controller/joint_command sensor_msgs/msg/JointState \
  "{name: ['arm_joint1','arm_joint2','arm_joint3','arm_joint4','arm_joint5'], position: [0.0,-0.9424,1.6492,0.8246,0.0], effort: [3.0]}"

# effort[0] 是本次运动时长（秒）——这不是笔误，源码里真的借用了 effort 字段当 duration

# 三个遥控器（Trigger 请求体为空花括号）
ros2 service call /action_player/play std_srvs/srv/Trigger "{}"
ros2 service call /action_player/next std_srvs/srv/Trigger "{}"
ros2 service call /action_player/stop std_srvs/srv/Trigger "{}"

# 零位校准 GUI（找机械中位 → Apply，写入 servo_offsets.yaml）
ros2 launch calibration servo_calib.launch.py
````

## EXP-06 逆运动学固定抓取 · 机器人机械臂应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-06-fixed-grasp/index.html)

````text
# 步骤 1 · 启动（先 bringup 再控制器，顺序即安全）
ros2 launch car1_bringup bringup.launch.py
ros2 launch car1_arm_controller arm_controller.launch.py

# 步骤 2 · 勘察（确认接口，别猜）
ros2 topic list | grep arm_controller
ros2 service list | grep arm_controller
# 预期话题：/arm_controller/target_pose、/arm_controller/set_pick_target、/arm_controller/joint_command
# 预期服务：pick、place、set_grasp_target、set_gripper、go_home、go_work、emergency_stop
# 注意：/arm_controller/pick、/place 是 std_srvs/srv/Trigger（无请求字段）

# 步骤 3 · 离线预演（不动舵机）：五路 PWM → TCP 位姿交叉核对
cd ~/ros2_ws/src/xflz_eai_car1/arm/kinematics/kinematics
python3 pwm_to_tcp.py 1500 1100 2200 1850 1500
# 输出 TCP 位置 / RPY / 四元数，用于核对 set_pick_target 的目标
# 仿真预演：http://<设备IP>:8000/viewer  （/viewer 后缀不能省）

# 步骤 4a · 设置抓取目标点（只记账，臂不会动）
ros2 topic pub --once /arm_controller/set_pick_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.28, y: 0.0, z: 0.10}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
# orientation y=1,w=0 → pitch 180° 夹爪垂直朝下

# 步骤 4b · 触发抓取（Trigger 无请求字段，命令体留空 "{}"）
ros2 service call /arm_controller/pick std_srvs/srv/Trigger "{}"
# 约 5~8 秒：开爪 → 抬到 pre_grasp_height → 下探 → 合爪 → 抬升 → 回 post_grasp_pose
# 另一条路（立即执行用于预检）：ros2 topic pub --once /arm_controller/target_pose ...（会直接动！）
# 过程怎么看：盯 arm_controller 的终端日志（六步顺序 + IK 解的 5 路角度），没有状态话题可 echo
````

````text
# 步骤 5a · 设置放置目标点（仍走 set_pick_target，同时覆盖 pick_target 与 place_target）
ros2 topic pub --once /arm_controller/set_pick_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.059, y: 0.074, z: 0.219}, orientation: {x: 0.0, y: 0.9826, z: 0.0, w: 0.1856}}}"
# 步骤 5b · 触发放置（同样 Trigger 空体）
ros2 service call /arm_controller/place std_srvs/srv/Trigger "{}"

# 步骤 5c · gripper_depth 调优对比（基线/抬高/下探三档，用 -p 覆盖节点参数）
# 重启 arm_controller 并带上新参数：
ros2 run car1_arm_controller arm_controller --ros-args -p gripper_depth:=0.02
# 对照档：-0.040（yaml 实配，抬高）/ 0.0（节点默认，停在表面）/ +0.02（往下探）
# 可调项：pre_grasp_height 0.02~0.10（撞桌加大，抬升也靠它）；grasp_pitch 保持 180 夹爪朝下
# lift_height 改了没用（源码未使用）——这一点要在报告里实测证明一次
````

````text
# 方位一 · 正前（基线）
ros2 topic pub --once /arm_controller/set_pick_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.28, y: 0.0, z: 0.10}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
ros2 service call /arm_controller/pick std_srvs/srv/Trigger "{}"

# 方位二 · 前左（+y 侧，底座向左转 45°）
ros2 topic pub --once /arm_controller/set_pick_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.198, y: 0.198, z: 0.10}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
ros2 service call /arm_controller/pick std_srvs/srv/Trigger "{}"

# 方位三 · 前右（−y 侧，底座向右转 45°）
ros2 topic pub --once /arm_controller/set_pick_target geometry_msgs/msg/PoseStamped \
  "{header: {frame_id: 'base_link'}, pose: {position: {x: 0.198, y: -0.198, z: 0.10}, orientation: {x: 0.0, y: 1.0, z: 0.0, w: 0.0}}}"
ros2 service call /arm_controller/pick std_srvs/srv/Trigger "{}"

# 纪律：每换一个方位先用 target_pose 预检可达（09 节预检纪律），确认无 PWM 贴近 500/2500 再 pick
# 对照观察：STAGE 1 底座转向与 EXP-01 里你用话题转过的 ±45° 对得上号吗？/arm/grasp_state 看迁移
````

## EXP-07 视觉动态抓取 · 机器人机械臂应用与开发

[来源](https://aibittek.github.io/course_generate_agent/courses/core/exp005-robot-arm/exp-07-visual-grasp/index.html)

````text
# 步骤 1 · 一站式启动（分时拉 4 组东西：bringup → arm_controller → 相机 → 节点 → rqt）
ros2 launch car1_app visual_grasp.launch.py
# [INFO] [visual_grasp_node]: State IDLE, waiting for /visual_grasp/start
# 启动时序 t=0/5/8/12/14s：先串口与 TF → 控制器 → 相机出流 → 检测节点 → rqt 窗口
# 别在 t<12s 就急着 call start：节点还没起，服务不存在

# 步骤 2 · 三栏可视化（一个话题就够，launch 已自动开 rqt_image_view）
ros2 run rqt_image_view rqt_image_view /visual_grasp/visualization
# 合成图：左 RGB（绿轮廓+黄框+红十字质心）｜中 Depth 伪彩（标 mm 读数）｜右 HSV Mask 二值真相
# 想看纯掩膜：ros2 run rqt_image_view rqt_image_view /visual_grasp/mask
# hsv_tuner.py 是讲义提供的离线调参脚本；线上调参直接改 launch 参数里的两段 H/S/V

# 步骤 3 · 反投影自测（内参请从 camera_info 读，别写死）
ros2 topic echo /ascamera_hp60c/camera_publisher/depth0/camera_info --once
# 看 k[0]=fx、k[4]=fy、k[2]=cx、k[5]=cy；实测 ≈614/614/320/240
# 手算：u,v=320,240 → X=Y=0（正好光轴中心），Z=d/1000；偏 200px、d=0.5m → X=(200)*0.5/614≈0.163m
# 标定物实测：1m 距离误差预期 ±2-3cm；超差先查 camera_info 是否已更新

# 步骤 4 · TF 链与时序
ros2 run tf2_tools view_frames      # … → arm_link4 → … → tcp_link → camera_depth_optical_frame
ros2 run tf2_ros tf2_echo base_link camera_depth_optical_frame

# 步骤 5 · 人机协同四服务（Trigger 类，命令体留空）
ros2 service call /visual_grasp/start   std_srvs/srv/Trigger "{}"
# 日志：State IDLE -> ... -> DETECT → Contour: area=…, centroid=(u,v) → WAIT_CONFIRMATION
ros2 service call /visual_grasp/confirm std_srvs/srv/Trigger "{}"   # 确认后进 PICK
ros2 service call /visual_grasp/skip    std_srvs/srv/Trigger "{}"   # 放弃本目标
ros2 service call /visual_grasp/abort   std_srvs/srv/Trigger "{}"   # 中止回 IDLE
````

````text
# 步骤 6 · 换色验收（以抓 30~50mm 红色物体为例）
# 1) 改 HSV 两段阈值：优先确保"副段 160~180"覆盖住你手里的红块（实测 H 落在 167~178）
# 2) 桌面摆放物体，start → DETECT 出轮廓 → confirm → 看 PICK 是否 <8s 完成
# 3) 拿走物体再 start：轮廓面积 <100 应被熔断（"Largest red contour too small"）→ 不抓空气
# 4) 免确认直连（首次务必先过一遍带确认的）
ros2 launch car1_app visual_grasp.launch.py wait_for_confirmation:=false
# 安全预演：只定位不抓，机械臂只升到观察位
ros2 launch car1_app visual_grasp.launch.py dry_run:=true
# 验收指标：HSV 检测成功率>95% · 单周期<15s · 抓取成功率>85% · 误抓<5%
````

## EXP-01 · 主机环境搭建与硬件认知 — IND-01 具身智能端到端平台与基础

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind01-endtoend-platform/exp-01-platform-setup/index.html)

````text
# ① 安装 NVIDIA 驱动并重启验证（第 00 章 二）
sudo apt update
sudo apt install nvidia-driver-595-open nvidia-settings nvidia-prime -y
# 完成后：sudo reboot，再执行 nvidia-smi 验证

# ② 解压源码 + 静默安装 Miniconda（第 00 章 三、四）
unzip lerobot-0.4.1.zip
bash Miniconda3-latest-Linux-x86_64.sh -b
~/miniconda3/bin/conda init bash && source ~/.bashrc

# ③ 创建环境并安装 LeRobot（第 00 章 七、八）
conda create -n lerobot python=3.10 -y
cd ~/lerobot-0.4.1
python3 -m pip install --upgrade pip -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install -e . -i https://pypi.tuna.tsinghua.edu.cn/simple

# ④ 写入机械臂 udev 规则（4 条序列号别名，第 00 章 九）
sudo tee /etc/udev/rules.d/99-lerobot-arms.rules << 'EOF'
SUBSYSTEM=="tty", ATTRS{idVendor}=="1a86", ATTRS{idProduct}=="55d3",
ATTRS{serial}=="5B61034506", SYMLINK+="arm_master_left", GROUP="dialout", MODE="0660"
# 同样格式另写三条：5B79018622→arm_master_right 5B79019324→arm_slave_left 5B79018452→arm_slave_right
EOF
sudo usermod -aG dialout $USER

# ⑤ 写入相机 udev 规则（3 条端口别名，第 00 章 十）
sudo tee /etc/udev/rules.d/99-lerobot-cameras.rules << 'EOF'
SUBSYSTEM=="video4linux", ENV{ID_PATH}=="pci-0000:00:14.0-usb-0:7:1.0", ATTR{index}=="0", SYMLINK+="cam_left", GROUP="video", MODE="0660"
# 端口6→cam_right 端口1→cam_head 同理
EOF
sudo usermod -aG video $USER

# ⑥ 重载规则 + 三重验证（第 00 章 十一）
sudo udevadm control --reload-rules && sudo udevadm trigger
python --version   # → Python 3.10.x
ls /dev/arm_*      # → arm_master_left arm_master_right arm_slave_left arm_slave_right
ls /dev/cam_*      # → cam_head cam_left cam_right
````

## EXP-02 · 机械臂硬件组装与串口识别 — IND-01 具身智能端到端平台与基础

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind01-endtoend-platform/exp-02-arm-assembly/index.html)

````text
# ① 顺序：先将主从臂接通电源，再把 USB 插到电脑（顺序不要颠倒！）
conda activate lerobot

# ② 搜索端口（第 03 章）——列出当前所有端口后提示拔线
lerobot-find-port
Finding all available ports for the MotorsBus.
Ports before disconnecting: ['/dev/ttyACM0', '/dev/ttyACM2', '/dev/ttyACM3', '/dev/ttyACM1', …]
Remove the USB cable from your MotorsBus and press Enter when done.

# ③ 拔掉要找的那只臂的 USB → 回车，输出即为它的端口
The port of this MotorsBus is '/dev/ttyACM0'
Reconnect the USB cable.   # 插回继续找下一条（每次重启端口号可能会变化）

# ④ 查询串口信息，核对序列号与别名（示例为右主臂）
udevadm info --query=all --name=/dev/ttyACM0
S: arm_master_right
E: ID_SERIAL_SHORT=5B79018622
E: DEVLINKS=… /dev/arm_master_right …

# ⑤ 写入/检查四条别名规则（第 03 章 5）
sudo nano /etc/udev/rules.d/99-lerobot-arms.rules
#   5B61034506→arm_master_left   5B79018622→arm_master_right
#   5B79019324→arm_slave_left     5B79018452→arm_slave_right

# ⑥ 之后所有命令直接用稳定别名
#   /dev/arm_slave_left · /dev/arm_slave_right · /dev/arm_master_left · /dev/arm_master_right
````

## EXP-03 · 摄像头配置与主从臂校准 — IND-01 具身智能端到端平台与基础

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind01-endtoend-platform/exp-03-arm-calibration/index.html)

````text
# 四条臂的稳定串口（见 03-LZ-XL01机械臂串口）
/dev/arm_slave_left   # 从臂左
/dev/arm_slave_right  # 从臂右
/dev/arm_master_left  # 主臂左
/dev/arm_master_right # 主臂右

# 进入环境并放开权限（每终端执行一次）
conda activate lerobot
sudo chmod 666 /dev/arm_*
````

````text
lerobot-calibrate \
  --robot.type=so101_follower  \
  --robot.port=/dev/arm_slave_left \
  --robot.id=left_follower_arm
````

````text
INFO ... calibrate.py:74 {'robot': {'calibration_dir': None, 'id': 'left_follower_arm',
    'port': '/dev/arm_slave_left', 'use_degrees': False}, 'teleop': None}
INFO ... follower.py:104 left_follower_arm SO101Follower connected.
INFO ... follower.py:121 Running calibration of left_follower_arm SO101Follower
Move left_follower_arm SO101Follower to the middle of its range of motion and press ENTER....
Move all joints sequentially through their entire ranges of motion.
Recording positions. Press ENTER to stop...
-------------------------------------------
NAME            |    MIN |    POS |    MAX
shoulder_pan    |    859 |   1982 |   2740
shoulder_lift   |    850 |    864 |   3189
elbow_flex      |    947 |   3141 |   3142
wrist_flex      |    940 |   2955 |   3216
wrist_roll      |   1142 |   1987 |   2763
gripper         |   2046 |   2082 |   3264
````

````text
# 右从臂：follower -> --port=/dev/arm_slave_right --id=right_follower_arm
lerobot-calibrate \
  --robot.type=so101_follower  \
  --robot.port=/dev/arm_slave_right \
  --robot.id=right_follower_arm

# 左主臂：leader -> --teleop.type=so101_leader --teleop.port --teleop.id
lerobot-calibrate \
  --teleop.type=so101_leader  \
  --teleop.port=/dev/arm_master_left  \
  --teleop.id=left_leader_arm

# 右主臂：--teleop.port=/dev/arm_master_right --teleop.id=right_leader_arm
lerobot-calibrate \
  --teleop.type=so101_leader  \
  --teleop.port=/dev/arm_master_right  \
  --teleop.id=right_leader_arm
````

````text
连接后若出现"use existing calibration file or type 'c' to rerun"}，输入大小写
lerobot-calibrate ... 同上命令
INFO ... Press ENTER to use provided calibration file ... or type 'c' and press ENTER to run calibration:
c
````

````text
主臂校准文件：
/home/lz-xl01/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/
    left_leader_arm.json
    right_leader_arm.json

从臂校准文件：
/home/lz-xl01/.cache/huggingface/lerobot/calibration/teleoperators/so101_leader/
    left_follower_arm.json
    right_follower_arm.json

示例 right_leader_arm.json：
{
    "shoulder_pan" : { "id": 1, "drive_mode": 0, "homing_offset": -1499, "range_min": 972,  "range_max": 3363 },
    "shoulder_lift": { "id": 2, "drive_mode": 0, "homing_offset": 1569,  "range_min": 818,  "range_max": 3192 },
    "elbow_flex"   : { "id": 3, "drive_mode": 0, "homing_offset": -1817, "range_min": 827,  "range_max": 3033 },
    "wrist_flex"   : { "id": 4, "drive_mode": 0, "homing_offset": -881,  "range_min": 1028, "range_max": 3350 },
    "wrist_roll"   : { "id": 5, "drive_mode": 0, "homing_offset": -218,  "range_min": 1100, "range_max": 3183 },
    "gripper"      : { "id": 6, "drive_mode": 0, "homing_offset": -1946, "range_min": 1999, "range_max": 3265 }
}
````

````text
# ① 摄像头搜索与分辨（第 04 章）
conda activate lerobot
rm -rf /home/lz-xl01/lerobot-0.4.1/outputs/captured_images
lerobot-find-cameras opencv
#   Camera #0 → /dev/video0 · #1 → /dev/video3 · #2 → /dev/video5（YUYV 640×480@30）
udevadm info --query=all --name=/dev/video0     # 查 ID_PATH，核对 cam_head 等别名

# ② 从臂校准（第 05 章 2）——左右各一次，只换 port/id
sudo chmod 666 /dev/arm_*
lerobot-calibrate \
  --robot.type=so101_follower \
  --robot.port=/dev/arm_slave_left \
  --robot.id=left_follower_arm
#   右从臂：--robot.port=/dev/arm_slave_right --robot.id=right_follower_arm

# ③ 主臂校准（第 05 章 3）——注意用 --teleop.* 参数
lerobot-calibrate \
  --teleop.type=so101_leader \
  --teleop.port=/dev/arm_master_left \
  --teleop.id=left_leader_arm
#   右主臂：--teleop.port=/dev/arm_master_right --teleop.id=right_leader_arm

# ④ 两阶段交互：摆基准回车 → 逐关节掰全程 → 回车出表
Move … to the middle of its range of motion and press ENTER....
Move all joints sequentially through their entire ranges of motion. Recording positions. Press ENTER to stop...
NAME | MIN | POS | MAX   ← 初始全 2047（电机位置中间值），录完如 859|1982|2740

# ⑤ 重新校准：提示处输 c 回车（直接回车=沿用旧文件）
Press ENTER to use provided calibration file associated with the id … or type 'c' and press ENTER to run calibration:

# ⑥ 校准数据位置（第 05 章 5）
#   主臂 …/calibration/teleoperators/so101_leader/*.json
#   从臂 …/calibration/robots/so101_follower/*.json
#   字段例：{"id":1,"drive_mode":0,"homing_offset":-517,"range_min":854,"range_max":3266}
````

## EXP-04 · 遥操作自检与故障排查 — IND-01 具身智能端到端平台与基础

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind01-endtoend-platform/exp-04-teleop-checkout/index.html)

````text
# ① 单臂遥操作·不带摄像头（第 06 章 3）
conda activate lerobot
sudo chmod 666 /dev/arm*
lerobot-teleoperate \
  --robot.type=so101_follower \
  --robot.port=/dev/arm_slave_left \
  --robot.id=left_follower_arm \
  --teleop.type=so101_leader \
  --teleop.port=/dev/arm_master_left \
  --teleop.id=left_leader_arm
# 正常打印：connected … time: 16.74ms (60 Hz)，动主臂从臂跟随

# ② 单臂遥操作·带摄像头（第 06 章 4，追加两段）
lerobot-teleoperate … --robot.cameras="{ left: {type: opencv, index_or_path: \"/dev/cam_left\",
  width: 640, height: 480, fps: 30, fourcc: \"MJPG\"}, side: {type: opencv,
  index_or_path: \"/dev/cam_head\", width: 640, height: 480, fps: 30, fourcc: \"MJPG\"}}" \
  --display_data=true
# 打印 time: 31.59ms (32 Hz) + 六关节 NORM 表 + 视频窗口

# ③ 双臂前·复制校准 so101 → so100/bimanual（第 07 章 2，务必执行）
mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so100_follower/
mkdir -p ~/.cache/huggingface/lerobot/calibration/teleoperators/so100_leader/
cp …/robots/so101_follower/left_follower_arm.json  …/robots/so100_follower/bimanual_follower_left.json
cp …/robots/so101_follower/right_follower_arm.json …/robots/so100_follower/bimanual_follower_right.json
cp …/teleoperators/so101_leader/left_leader_arm.json  …/teleoperators/so100_leader/bimanual_leader_left.json
cp …/teleoperators/so101_leader/right_leader_arm.json …/teleoperators/so100_leader/bimanual_leader_right.json

# ④ 双臂遥操作（第 07 章 3，CTRL+C 只按一次）
lerobot-teleoperate \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/arm_slave_left --robot.right_arm_port=/dev/arm_slave_right \
  --robot.id=bimanual_follower \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/arm_master_left --teleop.right_arm_port=/dev/arm_master_right \
  --teleop.id=bimanual_leader --display_data=true
# 12 关节 NORM 表，time: 16.72ms (60 Hz)；带摄像头再追加三路 cameras JSON

# ⑤ 停止与脚本化（第 15 章 1）
bash 1_calibrate_all.sh        # 依次校准 左从→右从→左主→右主
bash 2.3_teleoperate_left_with_cameras.sh   # 左臂带摄像头遥操作
bash 3_teleoperate_dual_arm.sh # 内含 mkdir+4cp+双臂命令，一键完成
````

## IND-01 具身智能端到端平台与基础 · 行业应用课程

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind01-endtoend-platform/index.html)

````text
ind01-endtoend-platform/
├── index.html                  课程目录页（全内联 CSS，不引 shared）
├── shared/                     实验页共享素材
│   ├── tutorial.css            主题类与组件样式
│   └── tutorial.js             侧栏 scrollspy + reveal 动画
├── exp-01-platform-setup/
│   ├── index.html              十章节实验页（流程与硬件全部网页化）
│   └── images/                 6 张硬件实物图 photo- 前缀
├── exp-02-arm-assembly/
│   ├── index.html              十章节实验页
│   ├── images/                 fig-p02 / fig-p03 共 8 张截图
│   └── videos/                 9 段安装视频 + 9 张封面
├── exp-03-arm-calibration/
│   ├── index.html              十章节实验页
│   ├── images/                 fig-p04 / fig-p05 共 8 张截图
│   └── videos/                 3 段校准视频 + 3 张封面
└── exp-04-teleop-checkout/
    ├── index.html              十章节实验页
    ├── images/                 fig-p06 / p07 / p14 / p15 共 12 张截图
    └── videos/                 2 段遥操作视频 + 2 张封面
````

## EXP-01 遥操作与数据采集 · IND-02 双臂采训推全流程

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind02-act-vla-pipeline/exp-01-teleop-recording/index.html)

````text
# ── 单臂遥操作（第 06 章）
conda activate lerobot
sudo chmod 666 /dev/arm*
lerobot-teleoperate \
  --robot.type=so101_follower --robot.port=/dev/arm_slave_left \
  --robot.id=left_follower_arm \
  --teleop.type=so101_leader --teleop.port=/dev/arm_master_left \
  --teleop.id=left_leader_arm

# ── 双臂前置：复制 4 份校准文件（第 07 章，必须先执行）
mkdir -p ~/.cache/huggingface/lerobot/calibration/robots/so100_follower/
mkdir -p ~/.cache/huggingface/lerobot/calibration/teleoperators/so100_leader/
cp ~/.cache/huggingface/lerobot/calibration/robots/so101_follower/left_follower_arm.json \
   ~/.cache/huggingface/lerobot/calibration/robots/so100_follower/bimanual_follower_left.json
（right 从臂、左/右主臂 4 个文件同理，共 4 条 cp）

# ── 双臂遥操作（第 07 章）
lerobot-teleoperate \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/arm_slave_left \
  --robot.right_arm_port=/dev/arm_slave_right \
  --robot.id=bimanual_follower \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/arm_master_left \
  --teleop.right_arm_port=/dev/arm_master_right \
  --teleop.id=bimanual_leader --display_data=true

# ── 单臂数据采集（第 08 章）
rm -rf /home/lz-xl01/.cache/huggingface/lerobot/liming/put_cube_plate
lerobot-record \
  --robot.type=so101_follower --robot.port=/dev/arm_slave_left \
  --robot.id=left_follower_arm \
  --robot.cameras="{left: {index: 0, fps: 30, width: 640, height: 480, video_backend: av}, \
side: {index: 2, fps: 30, width: 640, height: 480, video_backend: av}}" \
  --teleop.type=so101_leader --teleop.port=/dev/arm_master_left \
  --teleop.id=left_leader_arm --display_data=true \
  --dataset.repo_id=liming/put_cube_plate --dataset.num_episodes=5 \
  --dataset.single_task="Put red cube into plate" --dataset.push_to_hub=false \
  --dataset.episode_time_s=16 --dataset.reset_time_s=6

# ── 双臂数据采集（第 09 章，三摄像头 + 12 关节）
lerobot-record \
  --robot.type=bi_so100_follower \
  --robot.left_arm_port=/dev/arm_slave_left \
  --robot.right_arm_port=/dev/arm_slave_right \
  --robot.id=bimanual_follower \
  --teleop.type=bi_so100_leader \
  --teleop.left_arm_port=/dev/arm_master_left \
  --teleop.right_arm_port=/dev/arm_master_right \
  --teleop.id=bimanual_leader \
  --robot.cameras="{left: {index: 0, ...}, \
right: {index: 3, ...}, side: {index: 5, ...}}" \
  --dataset.repo_id=test/bimanual_pick_place --dataset.num_episodes=3 \
  --dataset.single_task="Lift the dish lid, put in the red block, and cover the lid." \
  --dataset.episode_time_s=16 --dataset.reset_time_s=6 \
  --dataset.push_to_hub=false --display_data=true
````

## EXP-02 数据整理与 ACT 模型训练 · IND-02 双臂采训推全流程

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind02-act-vla-pipeline/exp-02-act-training/index.html)

````text
# ── 1) 查看与打包数据（第 10 章）
cd ~/.cache/huggingface/lerobot/liming/put_cube_plate/
zip -r put_cube_plate.zip ~/.cache/huggingface/lerobot/liming/put_cube_plate

# ── 2) ACT 训练（第 10 章原文命令）
rm -rf /home/lz-xl01/lerobot-0.4.1/outputs/put_cube_on_plate_act
lerobot-train --dataset.repo_id=liming/put_cube_plate --policy.type=act --policy.repo_id=put_cube_on_plate_act --policy.push_to_hub=false --batch_size=8 --steps=1000 --output_dir=./outputs/put_cube_on_plate_act --policy.device=cuda --num_workers=4 --wandb.enable=false --log_freq=100 --save_freq=1000 --eval_freq=1000

# ── 3) 健康日志长这样（step 涨、loss 降）
step:100 smpl:3K ep:7 epch:2.23 loss:8.088 grdn:133.488 lr:1.0e-05 updt_s:0.969 data_s:0.048
… loss 持续下降 → Checkpoint policy after step 5000 → End of training

# ── 4) GPU 监控
nvtop
````

## EXP-03 云服务器使用与真机推理 · IND-02 双臂采训推全流程

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind02-act-vla-pipeline/exp-03-cloud-inference/index.html)

````text
# ── 云实例部署（第 11 章）
cd /data/lerobot/
pip install -e .

# ── 云端训练（与第 10 章同款，数据已上传）
lerobot-train --dataset.repo_id=liming/put_cube_plate --policy.type=act --policy.repo_id=put_cube_on_plate_act --policy.push_to_hub=false --batch_size=8 --steps=1000 --output_dir=./outputs/put_cube_on_plate_act --policy.device=cuda --num_workers=4 --wandb.enable=false --log_freq=100 --save_freq=1000 --eval_freq=1000

# ── 本地真机推理（第 12 章原文命令：record + policy.path，无 teleop）
rm -rf /home/lz-xl01/.cache/huggingface/lerobot/liming/eval_put_cube_plate
lerobot-record --robot.type=so101_follower --robot.port=/dev/arm_slave_left --robot.id=left_follower_arm --robot.cameras='{left: {index: 0, fps: 30, width: 640, height: 480, video_backend: av}, side: {index: 2, fps: 30, width: 640, height: 480, video_backend: av}}' --display_data=true --dataset.repo_id=liming/eval_put_cube_plate --dataset.num_episodes=1 --dataset.single_task="Put red cube into plate" --dataset.push_to_hub=false --dataset.episode_time_s=60 --policy.path=/home/lz-xl01/lerobot-0.4.1/outputs/put_cube_on_plate_act/checkpoints/last/pretrained_model
````

## EXP-04 VLA 大模型路线与对比 · IND-02 双臂采训推全流程

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind02-act-vla-pipeline/exp-04-vla-model/index.html)

````text
# ── 1) 安装 SmolVLA 依赖（第 13 章原文）
pip install -e ".[smolvla]" -i https://pypi.tuna.tsinghua.edu.cn/simple
pip install "transformers==4.54.1" -i https://pypi.tuna.tsinghua.edu.cn/simple

# ── 2) HF 镜像（下载 SmolVLM2 底座用）
export HF_ENDPOINT=https://hf-mirror.com

# ── 3) VLA 微调（第 13 章原文命令）
lerobot-train --dataset.repo_id=liming/put_cube_plate --policy.path=lerobot/smolvla_base --policy.repo_id=put_cube_plate_vla --batch_size=16 --steps=1000 --output_dir=/home/lz-xl01/lerobot-0.4.1/outputs/put_cube_plate_smolvla --policy.device=cuda --num_workers=4 --wandb.enable=false --log_freq=100 --save_freq=1000 --eval_freq=1000 --rename_map='{"observation.images.right":"observation.images.camera1", "observation.images.side":"observation.images.camera2"}'

# ── 4) 健康日志（step 100 → 1000）
step:100  … loss:0.081 grdn:1.069 lr:8.4e-05   # 权重 accum_to_1000
step:1000 … loss:0.022 grdn:0.332 lr:3.3e-06
Checkpoint policy after step 1000
End of training
````

## EXP-01 · 成果演示与整体框架 | IND-03 物料分拣应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind03-material-sorting/exp-01-case-overview/index.html)

````text
# ① 数据转换：save_data 目录 → LeRobot v3.0 数据集（按 episode 区间）
python convert_lerobot_v3_fixed_simple.py \
  --input  /home/lindenbot/save_data \
  --output /home/lindenbot/converted_data \
  --start 180 --end 209 --task "Parts Categorization"

# ② 训练 ACT 策略（本地示例，完整参数见 EXP-03）
lerobot-train \
  --policy.type=act \
  --dataset.repo_id=lindenbot/parts_categorization \
  --dataset.root=/home/lindenbot/converted_data \
  --batch_size=12 --steps=50000 --save_freq=4000

# ③ 真机推理：三终端启动后，用服务开关控制自主分拣
ros2 service call /model_ifrun std_srvs/srv/SetBool "{data: true}"   # 开始
ros2 service call /model_ifrun std_srvs/srv/SetBool "{data: false}"  # 停止
````

## EXP-02 · LDT-01 遥操作与数据采集 | IND-03 物料分拣应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind03-material-sorting/exp-02-teleop-collection/index.html)

````text
# ① 启动遥操作（LDT-01 手册 · 01start_tele.sh）
cd start_bash/02tele_and_record
. ./01start_tele.sh          # source 后按步骤 5.1–5.3 上锁扭矩

# ② 切换关节模式：5 = 拖拽模式（0=安全 1=位置）
ros2 service call /isaac_joint_mode arm_lifting_msgs/srv/ArmMode "{mode: 5}"

# ③ 停止采集并保留数据 / 停止并删除本组数据
./021data_collect_stop.sh              # 正常收尾
./022data_collect_stop_and_del_data.sh # 废弃本组

# ④ 数据落盘位置与容量（ConfigCameras.yaml）
save_base_path : /home/lindenbot/save_data
image_save_num : 1000   # 50Hz 下约 20 秒；0 = 无穷
````

## EXP-03 · 数据转换与模型训练 | IND-03 物料分拣应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind03-material-sorting/exp-03-data-training/index.html)

````text
# ① 环境（03-数据训练：Python 必须 3.10）
conda create -n lerobot python=3.10 -y
conda activate lerobot && conda install ffmpeg -y
git clone <lerobot仓库> && cd lerobot && pip install -e .

# ② 转换的四种切法（02-数据转换 / 工厂场景案例）
python convert_lerobot_v3_fixed_simple.py --input ... --output ... \
  --start 180 --end 209                      # 场景1：连续区间
python convert_lerobot_v3_fixed_simple.py ... --skip 163,172,202  # 场景2：剔坏组
python convert_lerobot_v3_fixed_simple.py ... --task "Parts Categorization"  # 场景3：任务名

# ③ 训练（工厂场景案例真实命令）
lerobot-train \
  --policy.type=act \
  --dataset.repo_id=lindenbot/parts_categorization \
  --dataset.root=/home/lindenbot/converted_data \
  --output_dir=~/lerobot-0.4.1/output/act_fac \
  --job_name="place fruits in the box" \
  --batch_size=12 --steps=50000 --save_freq=4000 \
  --log_freq=100 --num_workers=8 \
  --wandb.enable=false --policy.push_to_hub=false --policy.use_amp=true
````

## EXP-04 · 真机推理与分拣复现 | IND-03 物料分拣应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind03-material-sorting/exp-04-real-robot/index.html)

````text
# ① 配置（04-推理）
MODEL_PATH="/home/lindenbot/lerobot-0.4.1/output/act_fac/050000/pretrained_model"
DRY_RUN = True   # 首跑空仿真；验证无误后改 False 真下发

# ② 三终端启动（原文 04-推理 运行方法）
终端1 bash /home/lindenbot/iflytek_armtele_project/start_bash/04model_run_demo/01start_robot_act.sh
终端2 bash /home/lindenbot/lerobot-0.4.1/run_act_node.sh
终端3 source /home/lindenbot/iflytek_armtele_project/install/setup.bash
ros2 service call /model_ifrun std_srvs/srv/SetBool "{data: true}"   # 启动

# ③ 推理进行/停止
ros2 service call /model_ifrun std_srvs/srv/SetBool "{data: true}"   # 开
ros2 service call /model_ifrun std_srvs/srv/SetBool "{data: false}"  # 停
# 日志出现 mode=1 切换与 step=0/10/20 输出 = 推理循环正常
````

````text
🚀 LeRobot ACT 推理节点启动
⚠️  DRY_RUN=True：只打印动作，不发布到机械臂
   ✓ 相机设备: /dev/cam_head
   ✓ 相机设备: /dev/cam_left
   ✓ 相机设备: /dev/cam_right
📦 加载 ACT 模型 ...
🖥️  设备: cuda
✅ ACT 模型/处理器加载成功 (chunk_size=100)
📡 创建订阅者 ...
   ✓ 机械臂状态: /cerebellum_sdk/arm/joint_states
   ✓ 夹爪状态: /slave/gripper/{left,right}/states
📤 创建发布者 ...
   ✓ 臂命令 /cerebellum_sdk/arm/joint_commands + 夹爪命令
✅ 节点初始化完成，等待 /model_ifrun 启动指令
🎬 启动推理
🔎 /joint_states 实际关节名(14): [left_shoulder_pitch_joint, left_shoulder_roll_joint, ...]
🪜 ramp 起点(ROS顺序) L_pitch=+0.123 R_pitch=-0.456 -> 目标 L_pitch=+0.130 R_pitch=-0.450
🔄 推理循环启动 (50 Hz)
📡 发布线程启动 (100 Hz, slew限速)
step=0 L_arm[+0.130 -0.200 +0.100 -0.050 +0.300 +0.010 -0.150] L_grip=0.750 ...
````

## EXP-01 · 案例总览与成果演示 | IND-04 家庭整理应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind04-home-tidying/exp-01-case-overview/index.html)

````text
# ① 数据转换：save_data 编号目录 → LeRobot v3.0 数据集
python convert_lerobot_v3_fixed_simple.py \
  --input  /home/lindenbot/save_data/<组目录> \
  --output /home/lindenbot/converted_data \
  --start 180 --end 209

# ② 训练 ACT 策略（家庭任务，本地示例，完整参数见 EXP-03）
lerobot-train --policy.type=act \
  --dataset.repo_id=lindenbot/put_fruit_in_box \
  --dataset.root=/home/lindenbot/converted_data \
  --batch_size=12 --steps=50000 --save_freq=4000

# ③ 真机推理：启动推理节点后，用服务开关控制自主收纳
ros2 service call /model_ifrun std_srvs/srv/SetBool "{data: true}"   # 开始
ros2 service call /model_ifrun std_srvs/srv/SetBool "{data: false}"  # 停止
````

## EXP-02 · LDT-01 遥操作数据采集 | IND-04 家庭整理应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind04-home-tidying/exp-02-teleop-collection/index.html)

````text
# ① 启动遥操作（摘自 LDT-01 开发者版）
cd /home/lindenbot/iflytek_armtele_project/start_bash/01teleoperation
. ./01start_tele.sh
# 等 5 秒听到「嗒」声：第 3 关节转相机朝两侧 → 第 4 关节举臂 → 全关节回初始姿态

# ② 上位机入口（浏览器）
localhost:8699/#/robotManage   # 任务管理 → 新建任务 → 预览 → 开始采集

# ③ 采集落盘结构（一组 = 一个编号目录）
/home/lindenbot/save_data/
  ├─ 180/            # 编号目录（最大编号 = 最新一组）
  │   ├─ robot_data.csv   # 每拍关节角
  │   └─ cam_head/ cam_left/ cam_right/   # 三路视频帧
  └─ ...
````

## EXP-03 · 数据转换与模型训练 | IND-04 家庭整理应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind04-home-tidying/exp-03-data-training/index.html)

````text
# ① 一键安装环境
cd /data/lerobot && pip install -e .
# ② 解压上传到云服务器的数据压缩包
unzip converted_data.zip
# ③ 进入训练路径，执行训练命令（完整参数见 §05④）
cd /data/lerobot && lerobot-train ...
````

````text
# ① 依赖与校验
pip install pandas pyarrow numpy tqdm Pillow
ffmpeg -version

# ② 数据转换：编号目录 → LeRobot v3.0（家庭原文路径：save_data 下最新编号目录）
python convert_lerobot_v3_fixed_simple.py \
  --input  /home/lindenbot/save_data/11 \
  --output /home/lindenbot/converted_data \
  --start 180 --end 209
# 若该段内存在采集失败的 episode，用 --skip 剔掉（场景2）
python convert_lerobot_v3_fixed_simple.py \
  --input  /home/lindenbot/save_data/11 \
  --output /home/lindenbot/converted_data \
  --start 180 --end 209 \
  --skip 163,172,202
# 任务详情写错时，用 --task 覆盖任务名（场景3，写入 meta/tasks.parquet）
python convert_lerobot_v3_fixed_simple.py \
  --input  /home/lindenbot/save_data/11 \
  --output /home/lindenbot/converted_data \
  --start 200 --end 250 \
  --task "place fruits in the box"

# ③ 本地训练（家庭案例：放入 /home/lindenbot/lerobot-0.4.1）
lerobot-train --policy.type=act \
  --dataset.repo_id=lindenbot/put_fruit_in_box \
  --dataset.root=/home/lindenbot/converted_data \
  --output_dir=/home/lindenbot/lerobot-0.4.1/outputs/put_fruit_in_box \
  --job_name="place fruits in the box" \
  --batch_size=12 --steps=50000 --save_freq=4000 \
  --log_freq=100 --num_workers=8 \
  --wandb.enable=false --policy.push_to_hub=false --policy.use_amp=true

# ④ 在线训练（云端路径换为 /root/gpufree-data）
lerobot-train --policy.type=act \
  --dataset.repo_id=lindenbot/put_fruit_in_box \
  --dataset.root=/root/gpufree-data/data/$input_data$ \
  --output_dir=/root/gpufree-data/outputs/$output_data$ \
  --job_name="place fruits in the box" --batch_size=12 --steps=50000 \
  --save_freq=4000 --log_freq=100 --num_workers=8 --wandb.enable=false
````

````text
# ---- 读取单个 episode：robot_data.csv ─────────────────────────────
# ⚠️ CSV 格式坑：原始表头 33 个列名，但数据行末尾多一个逗号（共 34 列）
#    pandas 默认 read_csv 会把第一列 timestamp 当成 index，
#    导致 l_a_0..r_s_7 全部循环左移 1 位（df['l_a_0'] 拿到的是原始 l_a_1）。
#    这里治本：显式给出 34 列名（含末尾哑列 '_trailing'），并 skiprows=1。
def read_episode_data(self, episode_dir):
    csv_path = episode_dir / "robot_data.csv"
    # 34 个字段：timestamp + l_a_0..7 + r_a_0..7 + l_s_0..7 + r_s_0..7 + 末尾空列
    col_names = (['timestamp']
                 + [f'l_a_{i}' for i in range(8)] + [f'r_a_{i}' for i in range(8)]
                 + [f'l_s_{i}' for i in range(8)] + [f'r_s_{i}' for i in range(8)]
                 + ['_trailing'])
    # skiprows=1 跳过原始 header；header=None + 显式列名，终结列偏移
    df = pd.read_csv(csv_path, header=None, names=col_names, skiprows=1, index_col=False)
    df = df.drop(columns=['_trailing'])            # 末尾哑列丢弃

    # action = 主动臂命令（8 左 + 8 右 = 16 维）
    action_cols = [f'l_a_{i}' for i in range(8)] + [f'r_a_{i}' for i in range(8)]
    action = df[action_cols].values.astype(np.float32)

    # state  = 从臂状态（用 0 填充 NaN，主要 r_s_7 硬件无反馈）
    state_cols = [f'l_s_{i}' for i in range(8)] + [f'r_s_{i}' for i in range(8)]
    state = np.nan_to_num(df[state_cols].values.astype(np.float32), nan=0.0)
    return action, state, len(df)
````

## EXP-04 · 真机推理与鲁棒性评估 | IND-04 家庭整理应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind04-home-tidying/exp-04-real-robot/index.html)

````text
# ──终端1：机器人硬件层（臂 SDK + 夹爪 + 碰撞检测，无 CamerasNode）──
bash /home/lindenbot/iflytek_armtele_project/start_bash/04model_run_demo/01start_robot_act.sh

# ──终端2：ACT 推理节点（首次测新模型务必先 DRY_RUN=True）──
bash /home/lindenbot/lerobot-0.4.1/run_act_node.sh

# ──终端3：启动 / 停止推理──
source /home/lindenbot/iflytek_armtele_project/install/setup.bash
ros2 service call /model_ifrun std_srvs/srv/SetBool "{data: true}"   # 启动
ros2 service call /model_ifrun std_srvs/srv/SetBool "{data: false}"  # 停止
````

````text
🚀 LeRobot ACT 推理节点启动
⚠️  DRY_RUN=True：只打印动作，不发布到机械臂
   ✓ 相机设备: /dev/cam_head
   ✓ 相机设备: /dev/cam_left
   ✓ 相机设备: /dev/cam_right
📦 加载 ACT 模型 ...
🖥️  设备: cuda
✅ ACT 模型/处理器加载成功 (chunk_size=100)
📡 创建订阅者 ...
   ✓ 机械臂状态: /cerebellum_sdk/arm/joint_states
   ✓ 夹爪状态: /slave/gripper/{left,right}/states
📤 创建发布者 ...
   ✓ 臂命令 /cerebellum_sdk/arm/joint_commands + 夹爪命令
✅ 节点初始化完成，等待 /model_ifrun 启动指令
🎬 启动推理
🔎 /joint_states 实际关节名(14): [left_shoulder_pitch_joint, left_shoulder_roll_joint, ...]
🪜 ramp 起点(ROS顺序) L_pitch=+0.123 R_pitch=-0.456 -> 目标 L_pitch=+0.130 R_pitch=-0.450
🔄 推理循环启动 (50 Hz)
📡 发布线程启动 (100 Hz, slew限速)
step=0 L_arm[+0.130 -0.200 +0.100 -0.050 +0.300 +0.010 -0.150] L_grip=0.750 ...
step=10 L_arm[+0.135 -0.205 +0.098 -0.048 +0.305 +0.012 -0.148] L_grip=0.752 ...
````

## EXP-01 · 控制与巡检准备 | IND-05 工业巡检应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind05-industrial-inspection/exp-01-sdk-motion/index.html)

````text
# ① 安装 Python SDK（依赖 cyclonedds==0.10.2 / numpy / opencv-python）
git clone https://github.com/unitreerobotics/unitree_sdk2py.git
cd unitree_sdk2py && pip3 install -e .

# ② 读高层状态（enp2s0 换成连接机器狗的网卡名）
python3 ./example/high_level/read_highstate.py enp2s0

# ③ 运动测试：站立/趴下、速度控制、姿态控制（择一放开注释）
python3 ./example/high_level/sportmode_test.py enp2s0
#   test.StandUpDown()      站立与趴下
#   test.VelocityMove()     速度控制（低速起步！）
#   test.BalanceAttitude()  姿态控制

# ④ ROS2 侧：source 环境后验证状态与状态机
source ~/unitree_ros2/setup.sh
ros2 topic echo /sportmodestate        # 运动状态：mode/gait/velocity
ros2 topic echo /lowstate              # 底层状态：关节、IMU、电池
ros2 topic echo /wirelesscontroller    # 遥控器摇杆 lx/ly/rx/ry 与 keys

# ⑤ 站立示例（example 工作空间编译后）
cd ~/unitree_ros2/example && colcon build
./install/unitree_ros2_example/bin/go2_stand_example
````

## EXP-02 · 3D 激光建图 | IND-05 工业巡检应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind05-industrial-inspection/exp-02-lidar-slam/index.html)

````text
# ① 预检：点云话题与坐标系（EXP-01 已通链路）
ros2 topic hz /utlidar/cloud
ros2 topic echo --no-arr /utlidar/cloud      # frame_id 应为 utlidar_lidar

# ② 一键启动（内置 pointcloud_to_laserscan + slam_toolbox + Nav2 + RViz + Foxglove）
export ROBOT_IP=192.168.123.161
export CONN_TYPE=cyclonedds                   # 网线直连；webrtc 可走 WiFi
ros2 launch go2_robot_sdk robot.launch.py

# ③ 键盘遥控探索（若 launch 未带遥控，可另开终端）
ros2 run teleop_twist_keyboard teleop_twist_keyboard

# ④ RViz 操作（SlamToolboxPlugin 面板，鼠标点击，无命令行）
#    Start At Dock → 慢速绕场一圈闭合回路 → Save Map → Serialize Map

# ⑤ 也可用 slam_toolbox 官方 CLI 存图（标准功能）
ros2 run slam_toolbox save_map -f ~/maps/patrol_site
````

## EXP-03 · 定位与多点导航巡检 | IND-05 工业巡检应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind05-industrial-inspection/exp-03-amcl-nav2/index.html)

````text
# ① 启动（内置 map + AMCL + Nav2 + RViz；建图节点需按步骤停掉）
export ROBOT_IP=192.168.123.161
export CONN_TYPE=cyclonedds
ros2 launch go2_robot_sdk robot.launch.py

# ② RViz 操作（鼠标）：2D Pose Estimate 给初值 → 等粒子抱团
#    nav2_params.yaml 关键片段（沿 Nav2 官方结构）：
#    amcl: robot_model_type: "nav2_amcl::OmniMotionModel"  # 四足全向
#          max_particles: 2000   laser_model_type: "likelihood_field"
#    controller_server: controller_frequency: 3.0           # 3Hz 控制循环

# ③ 多点巡检：一次下发巡检点队列（nav2_waypoint_follower 服务）
ros2 service call /follow_waypoints nav2_msgs/srv/FollowWaypoints \
  "{poses: [{header: {frame_id: map}, pose: {position: {x: 2.1, y: 0.6}, orientation: {w: 1.0}}},
            {header: {frame_id: map}, pose: {position: {x: 4.0, y: 2.5}, orientation: {w: 1.0}}}]}"
#    也可以不用命令行：RViz 顶部 Waypoint 工具逐点投放，效果相同

# ④ 监控定位质量（收敛判据看这两个）
ros2 topic echo /amcl_pose --once      # 协方差椭圆应越缩越小
ros2 topic echo /particle_cloud --once # 粒子云：抱团=收敛，满图飞=未收敛
````

## EXP-04 · 视觉巡检闭环与报告 | IND-05 工业巡检应用场景

[来源](https://aibittek.github.io/course_generate_agent/courses/industry/ind05-industrial-inspection/exp-04-vision-loop/index.html)

````text
# ① 话题侧：看图像帧、看检测结果
ros2 topic hz /go2_camera/color/image
ros2 topic echo /detected_objects        # coco_detector 输出：类别+框

# ② 启动检测节点（go2_ros2_sdk 提供，detection_threshold 可在此粗调）
ros2 run coco_detector coco_detector_node --ros-args -p detection_threshold:=0.5

# ③ SDK 侧前置相机拍照存帧（需图形界面，ESC 退出）
python3 ./example/front_camera/camera_opencv.py

# ④ 自训表计模型（smart-gas-meter / Ultralytics 官方 CLI）
yolo detect train model=yolov8n.pt data=meter.yaml epochs=50 imgsz=640
python3 train_yolo.py                    # 案例仓库训练脚本
python3 infer_yolo.py                    # 案例仓库推理脚本
yolo export model=best.pt format=onnx    # 导出 ONNX 上机载部署

# ⑤ 巡检终点=充电桩：waypoint 驶近后转入 docking 对接（示意调用）
ros2 service call /dock_robot nav2_msgs/srv/DockRobot \
  "{dock_id: 'home', start_policy: 0}"
````


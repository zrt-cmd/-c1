#!/bin/bash
# ROS2 进程清理脚本 —— 调用即彻底清理，无参数、无交互
#
# 为什么每次启动前跑：
#   FastDDS 在 /dev/shm 建共享内存段；上次崩溃/强杀(Nav2 反复 SIGSEGV)残留的段不清理，
#   会让下次 DDS 初始化变慢/冲突 → Nav2 生命周期节点(amcl/controller) activate 超时
#   → "第一次正常、后续加载不起来"。彻底清一次能打破这个恶性循环。
#   （内存不是瓶颈：本项目 7.7G 无 swap，未见 OOM；慢/起不来是 DDS 脏状态所致。）
#   重启前必须跑本脚本：Ctrl+C 停 launch 会把部分子进程逃成孤儿，只靠 Ctrl+C/shutdown
#   不彻底；本脚本补全了 chassis 层(odom_publisher/ekf/init_pose/servo_controller 等)
#   与 web_detect 等易漏进程，结尾若还有残留会打印具体名单，供把新漏网节点加进名单。
#
# 用法：
#   ./cleanup_ros.sh

# 颜色定义
RED='\033[0;31m'
GREEN='\033[0;32m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 要清理的 ROS 进程模式。
# 注意：pkill 只匹配“可执行名/路径”关键字；Ctrl+C 停 launch 常把子进程逃成孤儿，
# 只要名字漏一个就杀不掉 → 下次启动慢/起不来。下面已按 launch 树把 chassis 层
# (odom_publisher/ekf_node/init_pose/complementary_filter_node/servo_controller)
# 与网页(web_detect)等易漏项补全。
# “ros2 launch/run/daemon” 精确匹配，避免命中自身或 cmdline 含 ros2_ws 的调用 shell。
CLEAN_PROCESSES=(
    # ros2 启动/守护
    "ros2 launch" "ros2 run" "ros2-daemon"
    # 可视化 / 仿真
    "rviz2" "gazebo" "gz" "slam_toolbox"
    # Nav2 全家（含生命周期）
    "amcl" "map_server" "planner_server" "controller_server" "bt_navigator"
    "behavior_server" "velocity_smoother" "lifecycle_manager"
    # bringup + chassis_controller 层（含底盘驱动/IMU滤波/里程计/EKF/初始化位姿/舵机）
    "robot_state_publisher" "joint_state_publisher" "robot_description_pub"
    "base_driver" "complementary_filter_node" "odom_publisher" "ekf_node"
    "init_pose" "servo_controller"
    # 机械臂 / 相机 / 雷达
    "arm_controller" "visual_grasp" "ascamera" "depth_camera" "m1ct_d2"
    "camera_web_viewer"
    # 任务 / 感知 / 语音 / 网页（web_detect 与 cube_detector 是不同进程，都要列）
    "mission_orchestrator" "car1_brics_task" "wake_word_node" "asr_node"
    "tts_node" "ocr_node" "cube_detector" "item_obb" "live_view" "web_detect"
    "teleop"
    # 国赛 M1/M2 独立节点（brics_national）
    "wake_trigger" "channel_nav"
    # wake_word 内嵌派生的 LLM 服务(Qwen, 端口8080)：父 wake_word 被杀后它会变孤儿，
    # 占着 8080 + 持续吃 CPU → 下次启动 wake_word“跳过LLM启动”复用旧孤儿、发现变慢。
    # 必须一起杀，否则 cleanup 漏网导致每次重启环境不干净。
    "flask_server" "llm_server"
    # launch 入口（兜底父进程）
    "car1_navigation" "car1_bringup"
)

echo "======================================"
echo "彻底清理 ROS 残留"
echo "======================================"

echo "[1/4] 终止 ROS 进程 (TERM→2s→KILL)..."
for pat in "${CLEAN_PROCESSES[@]}"; do
    pkill -TERM -f "$pat" 2>/dev/null || true
done
sleep 2
for pat in "${CLEAN_PROCESSES[@]}"; do
    pkill -KILL -f "$pat" 2>/dev/null || true
done
# 兜底：杀干净仍匹配的可执行（含 .py 解释器子进程）
ps aux | grep -E "mission_orchestrator|controller_server|bt_navigator|amcl|planner_server|base_driver" \
    | grep -v grep | awk '{print $2}' | xargs -r kill -9 2>/dev/null || true
sleep 1

echo "[2/4] 清理 FastDDS 共享内存 + 信号量 (/dev/shm)..."
if [ -d "/dev/shm" ]; then
    n_before=$(ls /dev/shm/ 2>/dev/null | grep -icE "fastrtps|ros")
    sudo rm -f /dev/shm/fastrtps* /dev/shm/ros* 2>/dev/null
    sudo rm -f /dev/shm/sem.fastrtps* 2>/dev/null
    echo "    已清理 $n_before 个共享内存/信号量文件"
fi

echo "[3/4] 清理 /tmp 的 DDS/日志残留..."
rm -f /tmp/fastrtps* 2>/dev/null || true

echo "[4/4] 验证..."
# 用同一份名单反查，残留时把具体进程/文件名打出来，便于定位是哪个活着。
pat_re="$(printf '%s|' "${CLEAN_PROCESSES[@]}" | sed 's/|$//')"
rem_lines="$(ps aux | grep -E "$pat_re" | grep -v grep || true)"
rem="$(printf '%s\n' "$rem_lines" | grep -c .)"
shm="$(ls /dev/shm/ 2>/dev/null | grep -icE "fastrtps|ros")"
echo "    剩余相关进程: $rem | 剩余 shm 文件: $shm"
if [ "$rem" -eq 0 ] && [ "$shm" -eq 0 ]; then
    echo -e "${GREEN}✓ 清理干净，可以启动${NC}"
else
    if [ "$rem" -gt 0 ]; then
        echo -e "${YELLOW}⚠ 以下进程仍未退出（PID CMD）：${NC}"
        printf '%s\n' "$rem_lines" | awk '{printf "    PID %-7s %s\n", $2, $11}'
        echo -e "${YELLOW}   若进程名不在 CLEAN_PROCESSES 名单里，说明又新增了漏网节点，请把它加进名单。${NC}"
    fi
    echo -e "${YELLOW}⚠ 仍有残留（$rem 进程 / $shm shm）。残留进程会新建 /dev/shm 段 → 下次启动变慢/起不来，请先解决再重启。${NC}"
fi

echo "======================================"

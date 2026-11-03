#!/bin/bash
# Level 3 机械臂控制器验证脚本
# 使用方法: ./test_level3_arm_controller.sh [test_number]
# 不带参数运行全部测试

source /opt/ros/humble/setup.bash
source /home/iflytek/ros2_ws/install/setup.bash

echo "=========================================="
echo "Level 3 机械臂控制器验证"
echo "=========================================="
echo ""

# 颜色定义
GREEN='\033[0;32m'
RED='\033[0;31m'
YELLOW='\033[1;33m'
NC='\033[0m' # No Color

# 检查 bringup 是否运行
check_bringup() {
    if ! ros2 node list | grep -q "/car1_base_driver"; then
        echo -e "${RED}错误: bringup 未运行${NC}"
        echo "请先运行: ros2 launch car1_bringup bringup.launch.py"
        exit 1
    fi
}

# 检查 arm_controller 是否运行
check_arm_controller() {
    if ! ros2 node list | grep -q "/arm_controller"; then
        echo -e "${RED}错误: arm_controller 未运行${NC}"
        echo "请先运行: ros2 launch car1_arm_controller arm_controller.launch.py"
        exit 1
    fi
}

# 等待函数
wait_for_motion() {
    local duration=$1
    echo -e "${YELLOW}等待 ${duration} 秒...${NC}"
    sleep $duration
}

# 测试 1: 回零位
test_1_go_home() {
    echo ""
    echo "========== 测试 1: 回零位 =========="
    ros2 service call /arm_controller/go_home std_srvs/srv/Trigger
    wait_for_motion 3
    echo -e "${GREEN}✓ 回零位测试完成${NC}"
}

# 测试 2: 安全高度 IK 测试
test_2_safe_height() {
    echo ""
    echo "========== 测试 2: 安全高度 IK (pitch=90°竖直向下) =========="
    # 末端=tcp_link, X_arm=17cm, z=28cm, pitch=90°
    # orientation: pitch=π/2 → quaternion (x=0, y=0.7071, z=0, w=0.7071)
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.22, y: 0.0, z: 0.28},
        orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}
      }
    }" --once
    wait_for_motion 4
    echo -e "${GREEN}✓ 安全高度测试完成${NC}"
}

# 测试 3: 逐步降低高度
test_3_descend() {
    echo ""
    echo "========== 测试 3: 逐步降低高度 (pitch=90°, 末端=tcp_link) =========="
    # 使用扫描结果中 x=0.22 的垂直序列

    echo "高位 z=0.28m (X_arm=17cm, 高度28cm)"
    # pitch=90° → quaternion (x=0, y=0.7071, z=0, w=0.7071)
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.22, y: 0.0, z: 0.28},
        orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}
      }
    }" --once
    wait_for_motion 4

    echo "中位 z=0.24m"
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.22, y: 0.0, z: 0.24},
        orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}
      }
    }" --once
    wait_for_motion 4

    echo "低位 z=0.22m"
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.22, y: 0.0, z: 0.22},
        orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}
      }
    }" --once
    wait_for_motion 4

    echo -e "${YELLOW}警告: 接下来将降低到 z=0.20m (最低可达点)${NC}"
    echo "按 Enter 继续，或 Ctrl+C 取消..."
    read

    echo "超低位 z=0.20m"
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.22, y: 0.0, z: 0.20},
        orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}
      }
    }" --once
    wait_for_motion 4

    echo -e "${GREEN}✓ 降低高度测试完成${NC}"
}

# 测试 4: 横向移动（左右）
test_4_lateral() {
    echo ""
    echo "========== 测试 4: 横向移动（左右，pitch=60°） =========="
    # 使用 pitch=60° 在 z=0.20 的横向点，观察分步移动（底座先转）
    # pitch=60° → quaternion (x=0, y=0.5, z=0, w=0.866)

    echo "左侧 (y=0.05m, X_arm=23cm, z=20cm)"
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.28, y: 0.05, z: 0.20},
        orientation: {x: 0, y: 0.5, z: 0, w: 0.866}
      }
    }" --once
    wait_for_motion 4

    echo "右侧 (y=-0.05m)"
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.28, y: -0.05, z: 0.20},
        orientation: {x: 0, y: 0.5, z: 0, w: 0.866}
      }
    }" --once
    wait_for_motion 4

    echo "回中间 (y=0.0m)"
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.28, y: 0.0, z: 0.20},
        orientation: {x: 0, y: 0.5, z: 0, w: 0.866}
      }
    }" --once
    wait_for_motion 4

    echo -e "${GREEN}✓ 横向移动测试完成（观察底座是否先旋转）${NC}"
}

# 测试 5: 夹爪控制
test_5_gripper() {
    echo ""
    echo "========== 测试 5: 夹爪控制 =========="

    echo "打开夹爪"
    ros2 service call /arm_controller/set_gripper std_srvs/srv/SetBool "{data: true}"
    wait_for_motion 2

    echo "闭合夹爪"
    ros2 service call /arm_controller/set_gripper std_srvs/srv/SetBool "{data: false}"
    wait_for_motion 2

    echo "再次打开夹爪"
    ros2 service call /arm_controller/set_gripper std_srvs/srv/SetBool "{data: true}"
    wait_for_motion 2

    echo -e "${GREEN}✓ 夹爪控制测试完成${NC}"
}

# 测试 6: 紧急停止（需要两个终端）
test_6_emergency_stop() {
    echo ""
    echo "========== 测试 6: 紧急停止 =========="
    echo -e "${YELLOW}此测试需要手动验证${NC}"
    echo "1. 在另一个终端运行长时间运动命令"
    echo "2. 在运动过程中调用紧急停止"
    echo ""
    echo "终端1: 运行本脚本的测试2或测试3"
    echo "终端2: ros2 service call /arm_controller/emergency_stop std_srvs/srv/Trigger"
    echo ""
    echo "按 Enter 跳过此测试..."
    read
}

# 测试 7: 超限位测试（应该拒绝）
test_7_limit_check() {
    echo ""
    echo "========== 测试 7: 关节限位检查 =========="
    echo "尝试发送超出范围的位置（应该被拒绝）"

    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {position: {x: 0.50, y: 0.0, z: 0.30}}
    }" --once
    wait_for_motion 2

    echo -e "${YELLOW}检查日志中是否有 'IK failed' 或 'limit violation' 错误${NC}"
    echo -e "${GREEN}✓ 限位检查测试完成${NC}"
}

# 测试 9: IK 精度实测
test_9_ik_accuracy() {
    echo ""
    echo "========== 测试 9: IK→实物精度测试（末端=tcp_link） =========="
    echo -e "${YELLOW}需要手动测量！${NC}"
    echo ""
    echo "目标：验证 a4=0.151 是否对应 tcp_link（夹爪张开时的指尖位置）"
    echo ""
    echo "步骤："
    echo "1. 夹爪将张开"
    echo "2. 机械臂将移动到 (x=0.20, y=0, z=0.10, pitch=90°)"
    echo "3. 用尺子从 base_link 原点（四轮中心）量到指尖的水平距离"
    echo "4. 理论值应为 20cm"
    echo "5. 偏差 < 1cm 为合格"
    echo ""
    echo "按 Enter 开始，或 Ctrl+C 取消..."
    read

    echo "张开夹爪..."
    ros2 service call /arm_controller/set_gripper std_srvs/srv/SetBool "{data: true}"
    wait_for_motion 2

    echo "移动到测量位置 (x=0.20, z=0.10, pitch=90°)..."
    # 注意：从扫描结果看，pitch=90° 在 x=0.20, z=0.10 不可达
    # 改用 pitch=60° 的可达点 x=0.22, z=0.10
    echo -e "${YELLOW}注意：使用 pitch=60° (x=0.22, z=0.10)，因为 pitch=90° 此点不可达${NC}"
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.22, y: 0.0, z: 0.10},
        orientation: {x: 0, y: 0.5, z: 0, w: 0.866}
      }
    }" --once
    wait_for_motion 4

    echo ""
    echo -e "${YELLOW}请测量指尖水平距离（从 base_link）：${NC}"
    read -p "实测距离 (cm): " measured_distance

    target_cm=22
    error=$(echo "$measured_distance - $target_cm" | bc)
    error_abs=$(echo "$error" | awk '{print ($1<0)?-$1:$1}')

    echo ""
    echo "理论距离: ${target_cm} cm"
    echo "实测距离: ${measured_distance} cm"
    echo "误差: ${error} cm"

    if (( $(echo "$error_abs < 1.0" | bc -l) )); then
        echo -e "${GREEN}✓ 精度合格 (误差 < 1cm)${NC}"
    else
        echo -e "${RED}✗ 精度超限 (误差 >= 1cm)，可能需要微调 a4 参数${NC}"
    fi

    echo -e "${GREEN}✓ IK 精度测试完成${NC}"
}

# 测试 8: 不同距离测试
test_8_different_distances() {
    echo ""
    echo "========== 测试 8: 不同距离测试 (pitch=90°, 末端=tcp_link) =========="

    echo "中距离 (x=0.22m, X_arm=17cm)"
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.22, y: 0.0, z: 0.24},
        orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}
      }
    }" --once
    wait_for_motion 4

    echo "远距离 (x=0.24m, X_arm=19cm)"
    ros2 topic pub /arm_controller/target_pose geometry_msgs/msg/PoseStamped "{
      header: {frame_id: 'base_link'},
      pose: {
        position: {x: 0.24, y: 0.0, z: 0.24},
        orientation: {x: 0, y: 0.7071, z: 0, w: 0.7071}
      }
    }" --once
    wait_for_motion 4

    echo -e "${GREEN}✓ 不同距离测试完成${NC}"
}

# 主菜单
show_menu() {
    echo ""
    echo "=========================================="
    echo "选择测试项目："
    echo "=========================================="
    echo "1. 回零位"
    echo "2. 安全高度 IK 测试 (pitch=90°)"
    echo "3. 逐步降低高度 (pitch=90°)"
    echo "4. 横向移动（左右，pitch=60°）"
    echo "5. 夹爪控制"
    echo "6. 紧急停止（需要两个终端）"
    echo "7. 关节限位检查"
    echo "8. 不同距离测试 (pitch=90°)"
    echo "9. IK→实物精度测试（需手动测量）"
    echo "10. 运行全部测试（1-5, 7-9）"
    echo "0. 退出"
    echo "=========================================="
}

# 运行全部测试
run_all_tests() {
    test_1_go_home
    test_2_safe_height
    test_3_descend
    test_4_lateral
    test_5_gripper
    test_7_limit_check
    test_8_different_distances
    test_9_ik_accuracy

    echo ""
    echo "=========================================="
    echo -e "${GREEN}全部自动测试完成！${NC}"
    echo "=========================================="
    echo ""
    echo "Level 3 验证要点检查："
    echo "[ ] 分步移动：底座先转，臂后动（在测试4中观察）"
    echo "[ ] 紧急停止：需要手动运行测试6验证"
    echo "[ ] 限位检查：测试7应显示错误日志"
    echo "[ ] IK 无解处理：测试7应拒绝执行"
    echo "[ ] 抓取深度可配置：参数已在 arm_params.yaml 中定义"
    echo "[ ] IK 精度：测试9 实测偏差应 < 1cm"
    echo "[ ] 完整抓取-放置流程：需要使用 pick/place 服务测试"
    echo ""
}

# 主程序
main() {
    check_bringup
    check_arm_controller

    if [ $# -eq 0 ]; then
        # 交互模式
        while true; do
            show_menu
            read -p "请选择 (0-10): " choice
            case $choice in
                1) test_1_go_home ;;
                2) test_2_safe_height ;;
                3) test_3_descend ;;
                4) test_4_lateral ;;
                5) test_5_gripper ;;
                6) test_6_emergency_stop ;;
                7) test_7_limit_check ;;
                8) test_8_different_distances ;;
                9) test_9_ik_accuracy ;;
                10) run_all_tests ;;
                0) echo "退出"; exit 0 ;;
                *) echo -e "${RED}无效选择${NC}" ;;
            esac
        done
    else
        # 命令行模式
        case $1 in
            1) test_1_go_home ;;
            2) test_2_safe_height ;;
            3) test_3_descend ;;
            4) test_4_lateral ;;
            5) test_5_gripper ;;
            6) test_6_emergency_stop ;;
            7) test_7_limit_check ;;
            8) test_8_different_distances ;;
            9) test_9_ik_accuracy ;;
            10|all) run_all_tests ;;
            *) echo "用法: $0 [1-10|all]"; exit 1 ;;
        esac
    fi
}

main "$@"

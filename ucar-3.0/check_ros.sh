#!/bin/bash
# ROS2 状态诊断脚本
# 快速查看当前运行的节点、话题、服务

echo "======================================"
echo "ROS2 系统状态"
echo "======================================"
echo ""

# 检查 ROS2 环境
if [ -z "$ROS_DISTRO" ]; then
    echo "❌ ROS2 环境未加载"
    echo "请先执行: source /opt/ros/humble/setup.bash"
    echo "         source ~/ros2_ws/install/setup.bash"
    exit 1
fi

echo "✓ ROS2 发行版: $ROS_DISTRO"
echo ""

# 1. 检查运行的节点
echo "1. 运行中的 ROS2 节点"
echo "--------------------------------------"
nodes=$(ros2 node list 2>/dev/null)
if [ -z "$nodes" ]; then
    echo "  无运行中的节点"
else
    echo "$nodes" | nl
    node_count=$(echo "$nodes" | wc -l)
    echo ""
    echo "  共 $node_count 个节点"
fi

echo ""

# 2. 检查活跃话题
echo "2. 活跃话题（有发布者和订阅者）"
echo "--------------------------------------"
topics=$(ros2 topic list 2>/dev/null)
if [ -z "$topics" ]; then
    echo "  无活跃话题"
else
    active_count=0
    while IFS= read -r topic; do
        # 检查是否有发布者
        pub_count=$(ros2 topic info "$topic" 2>/dev/null | grep "Publisher count:" | awk '{print $3}')
        sub_count=$(ros2 topic info "$topic" 2>/dev/null | grep "Subscription count:" | awk '{print $3}')

        if [ "$pub_count" -gt 0 ] && [ "$sub_count" -gt 0 ]; then
            echo "  $topic (pub:$pub_count, sub:$sub_count)"
            active_count=$((active_count + 1))
        fi
    done <<< "$topics"

    if [ $active_count -eq 0 ]; then
        echo "  无活跃话题（有发布但无订阅，或有订阅但无发布）"
    else
        echo ""
        echo "  共 $active_count 个活跃话题"
    fi
fi

echo ""

# 3. 检查服务
echo "3. 可用服务"
echo "--------------------------------------"
services=$(ros2 service list 2>/dev/null)
if [ -z "$services" ]; then
    echo "  无可用服务"
else
    service_count=$(echo "$services" | wc -l)
    echo "$services" | head -10
    if [ $service_count -gt 10 ]; then
        echo "  ... (还有 $((service_count - 10)) 个服务)"
    fi
    echo ""
    echo "  共 $service_count 个服务"
fi

echo ""

# 4. 检查进程
echo "4. ROS2 相关进程"
echo "--------------------------------------"
ps aux | grep -E "ros2|rviz2|gazebo|slam|amcl|navigation" | grep -v grep | awk '{printf "  PID: %-6s CPU: %-4s MEM: %-4s CMD: %s\n", $2, $3"%", $4"%", $11}' | head -15

echo ""
echo "======================================"
echo "诊断完成"
echo ""
echo "提示："
echo "  - 如需清理残留进程: ./cleanup_ros.sh"
echo "  - 查看话题数据: ros2 topic echo /topic_name"
echo "  - 查看节点信息: ros2 node info /node_name"
echo "======================================"

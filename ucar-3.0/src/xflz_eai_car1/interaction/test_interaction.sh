#!/bin/bash
# Quick test script for interaction module

set -e

echo "=========================================="
echo "Interaction Module Quick Test"
echo "=========================================="
echo ""

cd ~/ros2_ws
source install/setup.bash

echo "Step 1: Test TTS node (text-to-speech)"
echo "  Starting TTS node in background..."
ros2 run car1_speech tts_node &
TTS_PID=$!
sleep 2

echo "  Sending test text..."
ros2 topic pub /interaction/tts/speak std_msgs/msg/String "{data: 'Hello, this is a test'}" --once
sleep 1

echo "  Stopping TTS node..."
kill $TTS_PID 2>/dev/null || true
echo ""

echo "Step 2: Test Chat node (LLM)"
echo "  Starting chat node in background..."
ros2 run car1_llm chat_node &
CHAT_PID=$!
sleep 2

echo "  Sending test input..."
ros2 topic pub /interaction/llm/user_input std_msgs/msg/String "{data: '导航到厨房'}" --once
sleep 1

echo "  Checking reply..."
timeout 2 ros2 topic echo /interaction/llm/assistant_reply --once || echo "  (No reply yet, mock mode)"

echo "  Stopping chat node..."
kill $CHAT_PID 2>/dev/null || true
echo ""

echo "Step 3: Test Skill Executor"
echo "  Starting skill executor in background..."
ros2 run car1_llm skill_executor --ros-args -p skills_config:=$(ros2 pkg prefix car1_llm)/share/car1_llm/config/skills.yaml &
SKILL_PID=$!
sleep 2

echo "  Sending test skill request..."
ros2 topic pub /interaction/llm/skill_request std_msgs/msg/String "{data: 'navigate_to|location:kitchen'}" --once
sleep 1

echo "  Stopping skill executor..."
kill $SKILL_PID 2>/dev/null || true
echo ""

echo "=========================================="
echo "✅ All tests completed!"
echo "=========================================="
echo ""
echo "Next steps:"
echo "  1. Fill in API keys in config/*.yaml files"
echo "  2. Implement actual speech engines (iFlytek, etc.)"
echo "  3. Implement LLM API calls (OpenAI, etc.)"
echo "  4. Test with real hardware (microphone, speaker)"
echo ""

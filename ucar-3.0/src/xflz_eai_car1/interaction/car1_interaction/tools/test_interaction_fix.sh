#!/bin/bash
# 语音交互系统测试脚本

echo "========================================"
echo "语音交互系统 - 修复验证测试"
echo "========================================"
echo ""

cd /home/iflytek/ros2_ws/src/xflz_eai_car1/interaction/car1_interaction/tools

# 检查文件是否存在
echo "[1/4] 检查文件..."
if [ ! -f "voice_interaction.py" ]; then
    echo "❌ voice_interaction.py 不存在"
    exit 1
fi

if [ ! -f "config.yaml" ]; then
    echo "❌ config.yaml 不存在"
    exit 1
fi

if [ ! -f "resource/kws.wav" ]; then
    echo "❌ resource/kws.wav 不存在"
    exit 1
fi

echo "✅ 所有文件完整"
echo ""

# 检查 Python 语法
echo "[2/4] 检查 Python 语法..."
python3 -m py_compile voice_interaction.py 2>&1
if [ $? -eq 0 ]; then
    echo "✅ Python 语法正确"
else
    echo "❌ Python 语法错误"
    exit 1
fi
echo ""

# 显示关键配置
echo "[3/4] 关键配置检查..."
echo "VAD threshold: $(grep 'threshold:' config.yaml | head -1 | awk '{print $2}')"
echo "VAD min_speech_duration: $(grep 'min_speech_duration:' config.yaml | head -1 | awk '{print $2}')"
echo "ASR endpoint_silence: $(grep 'endpoint_silence:' config.yaml | awk '{print $2}')"
echo "LLM system_prompt: $(grep 'system_prompt:' config.yaml | cut -d'"' -f2)"
echo ""

# 测试说明
echo "[4/4] 测试说明"
echo "========================================"
echo ""
echo "📋 测试步骤："
echo ""
echo "1. 说 '小飞同学' 唤醒系统"
echo "   → 应该听到提示音 '叮'"
echo "   → 看到日志: [State] IDLE → LISTENING"
echo ""
echo "2. 等待 1-2 秒，让环境安静"
echo "   → 看到日志: 提示音播放完成，等待用户说话..."
echo "   → 看到日志: 检测到静音，准备接收用户语音"
echo ""
echo "3. 清晰地说 '你会做什么'"
echo "   → 看到日志: [VAD] 检测到用户开始说话 → ASR 激活"
echo "   → 看到实时转写: [ASR] 你会做什么"
echo "   → 看到识别结果: [ASR] 识别结果: 你会做什么"
echo ""
echo "4. 等待 LLM 回答"
echo "   → 看到日志: [LLM] 我可以... (应该是中文)"
echo "   → 听到 TTS 播放中文语音"
echo ""
echo "✅ 修复验证点："
echo "   • 唤醒后不会自动进入识别（需要你说话）"
echo "   • 能看到实时转写的中文内容"
echo "   • 识别结果完整准确"
echo "   • LLM 输出是中文"
echo "   • 没有多次 '端点触发，无有效结果'"
echo ""
echo "❌ 常见问题排查："
echo "   • 如果唤醒后立即识别 → VAD threshold 太低"
echo "   • 如果识别不到语音 → VAD threshold 太高或麦克风音量低"
echo "   • 如果识别结果是 'VI' 等乱码 → 说话不够清晰或环境噪音大"
echo "   • 如果 LLM 输出英文 → 模型本身问题，检查 system_prompt"
echo ""
echo "========================================"
echo ""
echo "🚀 按 Enter 启动系统，按 Ctrl+C 退出"
read

python3 voice_interaction.py

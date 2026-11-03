#!/bin/bash
set +e  # 不因错误而中断

echo "========================================="
echo "   ASR 问题全面诊断"
echo "========================================="
echo ""

# 步骤 1：检查 input mux
echo "【步骤 1】检查麦克风输入源"
echo "-----------------------------------------"
LEFT_MUX=$(amixer -c 0 sget 'Left Line Mux' 2>/dev/null | grep "Item0:" | awk -F"'" '{print $2}')
RIGHT_MUX=$(amixer -c 0 sget 'Right Line Mux' 2>/dev/null | grep "Item0:" | awk -F"'" '{print $2}')
echo "  Left Line Mux:  $LEFT_MUX"
echo "  Right Line Mux: $RIGHT_MUX"
if [ "$LEFT_MUX" != "MicL" ] || [ "$RIGHT_MUX" != "MicR" ]; then
    echo "  ❌ 错误：输入源不是麦克风！"
    echo "  修复：amixer -c 0 sset 'Left Line Mux' 'MicL'"
    echo "        amixer -c 0 sset 'Right Line Mux' 'MicR'"
    exit 1
else
    echo "  ✓ 输入源正确"
fi
echo ""

# 步骤 2：测试环境底噪
echo "【步骤 2】测试环境底噪（保持安静 3 秒）"
echo "-----------------------------------------"
python3 audio_test.py --mode record -d 3 -o silence_test.wav --analyze 2>&1 | tee /tmp/silence_result.txt
SILENCE_PEAK=$(grep "Peak:" /tmp/silence_result.txt | awk '{print $2}')
echo ""
if [ ! -z "$SILENCE_PEAK" ]; then
    PEAK_NUM=$(echo $SILENCE_PEAK | sed 's/dB//' | sed 's/-//')
    if (( $(echo "$PEAK_NUM < 35" | bc -l) )); then
        echo "  ✓ 底噪正常 (Peak: $SILENCE_PEAK)"
    else
        echo "  ⚠️  底噪较高 (Peak: $SILENCE_PEAK)，可能导致误触发"
    fi
fi
echo ""

# 步骤 3：测试有声音的录音质量
echo "【步骤 3】测试录音质量"
echo "-----------------------------------------"
echo "  请说：小飞同学（3 秒后开始录音）"
sleep 3
python3 audio_test.py --mode record -d 3 -o speech_test.wav --analyze 2>&1 | tee /tmp/speech_result.txt
SPEECH_PEAK=$(grep "Peak:" /tmp/speech_result.txt | awk '{print $2}')
echo ""
if [ ! -z "$SPEECH_PEAK" ]; then
    echo "  录音 Peak: $SPEECH_PEAK"
    PEAK_NUM=$(echo $SPEECH_PEAK | sed 's/dB//' | sed 's/-//')
    if (( $(echo "$PEAK_NUM < 3" | bc -l) )); then
        echo "  ⚠️  可能削波，建议降低增益"
    elif (( $(echo "$PEAK_NUM > 20" | bc -l) )); then
        echo "  ⚠️  信号太弱，建议提高增益"
    else
        echo "  ✓ 音频电平合适"
    fi
fi
echo ""

# 步骤 4：检查 ONNX 模型文件
echo "【步骤 4】检查 ONNX 模型文件"
echo "-----------------------------------------"
ls -lh /home/iflytek/models/zipformer_onnx/*.onnx 2>&1 | grep -E "encoder|decoder|joiner" || echo "  ❌ 模型文件不存在或路径错误"
echo ""

# 步骤 5：CPU ASR 测试（ONNX 模型）
echo "【步骤 5】CPU ONNX ASR 识别测试"
echo "-----------------------------------------"
echo "  5a. 用 CPU (ONNX) 测试（10 秒后自动停止）"
echo "      请说：小飞同学"
timeout 10 python3 asr_test.py --provider cpu --capture-volume 88 2>&1 | tee /tmp/cpu_result.txt
echo ""

# 步骤 6：分析结果
echo "========================================="
echo "   诊断结果总结"
echo "========================================="
CPU_HAS_TEXT=$(grep -v "endpoint, no text" /tmp/cpu_result.txt | grep -E "^[0-9]+:" | wc -l)
RKNN_HAS_TEXT=$(grep -v "endpoint, no text" /tmp/rknn_result.txt | grep -E "^[0-9]+:" | wc -l)

echo "CPU 模式识别结果数: $CPU_HAS_TEXT"
echo "RKNN 模式识别结果数: $RKNN_HAS_TEXT"
echo ""

if [ $CPU_HAS_TEXT -gt 0 ] && [ $RKNN_HAS_TEXT -eq 0 ]; then
    echo "❌ 关键问题：RKNN 模型无法识别，但 CPU 可以"
    echo "   可能原因：RK3588 RKNN 模型与 RK3576 不兼容"
    echo "   建议：使用 --provider cpu，或重新编译适配 RK3576 的模型"
elif [ $CPU_HAS_TEXT -eq 0 ] && [ $RKNN_HAS_TEXT -eq 0 ]; then
    echo "❌ 严重问题：CPU 和 RKNN 都无法识别"
    echo "   可能原因：音频质量问题或模型路径错误"
    echo "   建议：检查录音文件 speech_test.wav 是否有声音"
else
    echo "✓ 识别器工作正常"
fi
echo ""
echo "生成的测试文件："
echo "  - silence_test.wav  (静音底噪测试)"
echo "  - speech_test.wav   (说话录音测试)"
echo ""
echo "========================================="

#!/usr/bin/env python3

# Real-time speech recognition from a microphone with sherpa-onnx Python API
# with endpoint detection.
#
# Note: This script uses ALSA and works only on Linux systems, especially
# for embedding Linux systems and for running Linux on Windows using WSL.
#
# Please refer to
# https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html
# to download pre-trained models

import argparse
import sys
from pathlib import Path

# Add sherpa-onnx path
sys.path.insert(0, "/home/iflytek/AI/sherpa-onnx/sherpa-onnx/python")
sys.path.append("/home/iflytek/AI/sherpa-onnx")

import sherpa_onnx
import time
import numpy as np


def assert_file_exists(filename: str):
    assert Path(filename).is_file(), (
        f"{filename} does not exist!\n"
        "Please refer to "
        "https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html to download it"
    )


def get_args():
    parser = argparse.ArgumentParser(
        formatter_class=argparse.ArgumentDefaultsHelpFormatter
    )

    parser.add_argument(
        "--tokens",
        type=str,
        required=False,
        default="/home/iflytek/models/zipformer_onnx/tokens.txt",
        help="Path to tokens.txt",
    )

    parser.add_argument(
        "--encoder",
        type=str,
        required=False,
        default="/home/iflytek/models/zipformer_onnx/encoder-epoch-99-avg-1.int8.onnx",
        help="Path to the encoder model (ONNX)",
    )

    parser.add_argument(
        "--decoder",
        type=str,
        required=False,
        default="/home/iflytek/models/zipformer_onnx/decoder-epoch-99-avg-1.onnx",
        help="Path to the decoder model (ONNX)",
    )

    parser.add_argument(
        "--joiner",
        type=str,
        required=False,
        default="/home/iflytek/models/zipformer_onnx/joiner-epoch-99-avg-1.int8.onnx",
        help="Path to the joiner model (ONNX)",
    )

    parser.add_argument(
        "--decoding-method",
        type=str,
        default="greedy_search",
        help="Valid values are greedy_search and modified_beam_search",
    )

    parser.add_argument(
        "--provider",
        type=str,
        default="cpu",
        help="Valid values: cpu, cuda, coreml, rknn (default: cpu = ONNX)",
    )

    parser.add_argument(
        "--hotwords-file",
        type=str,
        default="",
        help="""
        The file containing hotwords, one words/phrases per line, and for each
        phrase the bpe/cjkchar are separated by a space. For example:

        ▁HE LL O ▁WORLD
        你 好 世 界
        """,
    )

    parser.add_argument(
        "--hotwords-score",
        type=float,
        default=1.5,
        help="""
        The hotword score of each token for biasing word/phrase. Used only if
        --hotwords-file is given.
        """,
    )

    parser.add_argument(
        "--blank-penalty",
        type=float,
        default=0.0,
        help="""
        The penalty applied on blank symbol during decoding.
        Note: It is a positive value that would be applied to logits like
        this `logits[:, 0] -= blank_penalty` (suppose logits.shape is
        [batch_size, vocab] and blank id is 0).
        """,
    )

    parser.add_argument(
        "--device-name",
        type=str,
        required=False,
        default="plughw:4,0",
        help="""
The device name specifies which microphone to use in case there are several
on your system. You can use

  arecord -l

to find all available microphones on your computer. For instance, if it outputs

**** List of CAPTURE Hardware Devices ****
card 3: UACDemoV10 [UACDemoV1.0], device 0: USB Audio [USB Audio]
  Subdevices: 1/1
  Subdevice #0: subdevice #0

and if you want to select card 3 and device 0 on that card, please use:

  plughw:3,0

as the device_name.
        """,
    )

    return parser.parse_args()


def create_recognizer(args):
    assert_file_exists(args.encoder)
    assert_file_exists(args.decoder)
    assert_file_exists(args.joiner)
    assert_file_exists(args.tokens)
    # Please replace the model files if needed.
    # See https://k2-fsa.github.io/sherpa/onnx/pretrained_models/index.html
    # for download links.
    recognizer = sherpa_onnx.OnlineRecognizer.from_transducer(
        tokens=args.tokens,
        encoder=args.encoder,
        decoder=args.decoder,
        joiner=args.joiner,
        num_threads=1,
        sample_rate=16000,
        feature_dim=80,
        enable_endpoint_detection=True,
        rule1_min_trailing_silence=2.4,
        rule2_min_trailing_silence=1.2,
        rule3_min_utterance_length=300,  # it essentially disables this rule
        decoding_method=args.decoding_method,
        provider=args.provider,
        hotwords_file=args.hotwords_file,
        hotwords_score=args.hotwords_score,
        blank_penalty=args.blank_penalty,
    )
    return recognizer


def main():
    args = get_args()
    device_name = args.device_name
    print(f"device_name: {device_name}")
    alsa = sherpa_onnx.Alsa(device_name)

    print("Creating recognizer")
    recognizer = create_recognizer(args)
    print("Started! Please speak")

    sample_rate = 16000
    samples_per_read = int(0.1 * sample_rate)  # 0.1 second = 100 ms

    stream = recognizer.create_stream()

    last_result = ""
    segment_id = 0
    while True:
        samples = alsa.read(samples_per_read)  # a blocking read
        stream.accept_waveform(sample_rate, samples)
        while recognizer.is_ready(stream):
            recognizer.decode_stream(stream)

        is_endpoint = recognizer.is_endpoint(stream)

        result = recognizer.get_result(stream)

        if result and (last_result != result):
            last_result = result
            print("\r{}:{}".format(segment_id, result), end="", flush=True)
        if is_endpoint:
            if result:
                print("\r{}:{}".format(segment_id, result), flush=True)
                segment_id += 1
            recognizer.reset(stream)


def record(self, duration=8):
    """录制指定时长的音频并实时识别，采用offline-stt-play.py的流程但使用sounddevice"""
    print(f"开始录制，请说话...")
    
    # 创建识别流
    recognizer = self.stt.recognizer
    stream = recognizer.create_stream()
    
    # 录制变量初始化
    frames = []
    all_results = []  # 存储所有语音段结果
    last_result = ""
    segment_id = 0  # 语音段ID
    
    # 静音检测变量
    silence_start = None
    max_silence_duration = 2.5  # 静音2.5秒后停止
    is_speaking = False
    
    # 开始时间
    start_time = time.time()
    
    # 清空缓冲区
    while not self.audio_buffer.empty():
        self.audio_buffer.get()
    
    # 主循环
    while time.time() - start_time < duration:
        if not self.audio_buffer.empty():
            # 获取音频数据
            data = self.audio_buffer.get()
            frames.append(data.copy())
            
            # 准备音频数据
            if data.dtype != np.int16:
                audio_int16 = (data * 32767).astype(np.int16)
            else:
                audio_int16 = data
            
            # 送入识别流
            stream.accept_waveform(self.sample_rate, audio_int16.flatten())
            
            # 解码
            while recognizer.is_ready(stream):
                recognizer.decode_stream(stream)
            
            # 检查是否到达终点
            is_endpoint = recognizer.is_endpoint(stream)
            
            # 获取识别结果
            result = recognizer.get_result(stream)
            
            # 语音活动检测
            if result and not is_speaking:
                is_speaking = True
                silence_start = None
                print("\n检测到语音活动")
            
            # 更新并显示结果
            if result and (last_result != result):
                last_result = result
                print(f"\r{segment_id}:{result}", end="", flush=True)
            
            # 处理端点检测
            if is_endpoint:
                if result:
                    print(f"\r{segment_id}:{result}", flush=True)
                    all_results.append(result)  # 记录这个语音段的结果
                    segment_id += 1
                
                # 重置流以接收下一段语音
                recognizer.reset(stream)
                stream = recognizer.create_stream()
                
                # 如果之前正在说话，现在检测到端点，记录静音开始时间
                if is_speaking:
                    silence_start = time.time()
                    is_speaking = False
            
            # 检查静音时长，决定是否停止录制
            if silence_start and time.time() - silence_start > max_silence_duration:
                print(f"\n检测到静音{time.time() - silence_start:.2f}秒，停止录制")
                break
                
        # 小睡一会儿，减少CPU使用率
        time.sleep(0.01)
    
    # 合并所有识别结果
    final_result = " ".join(all_results).strip()
    if not final_result and last_result:  # 如果没有完整句子但有部分结果
        final_result = last_result
    
    # 打印最终结果
    print(f"\n最终识别结果: '{final_result}'")
    
    # 合并音频数据
    if frames:
        audio_data = np.concatenate(frames, axis=0)
        print(f"录制完成，总时长: {time.time() - start_time:.2f}秒，音频帧数: {len(frames)}")
        print(f"音频数据形状: {audio_data.shape}, 最大值: {np.max(np.abs(audio_data)):.4f}")
    else:
        audio_data = np.zeros(1, dtype=np.float32)
    
    # 如果有文本，打印要发送的内容
    if final_result:
        print(f"发送给大模型的文本: '{final_result}'")
    
    return audio_data, final_result


if __name__ == "__main__":
    try:
        main()
    except KeyboardInterrupt:
        print("\nCaught Ctrl + C. Exiting")

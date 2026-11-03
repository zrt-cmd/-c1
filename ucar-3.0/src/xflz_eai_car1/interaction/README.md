# Human-Robot Interaction Module

人机交互模块，包含语音交互和大模型对话功能。

---

## 目录结构

```
interaction/
├── car1_speech/              # 语音交互包
│   ├── car1_speech/
│   │   ├── wake_word_node.py    # 语音唤醒节点
│   │   ├── asr_node.py          # 语音识别节点（ASR）
│   │   └── tts_node.py          # 语音合成节点（TTS）
│   ├── config/
│   │   ├── wake_word.yaml
│   │   ├── asr.yaml
│   │   └── tts.yaml
│   └── launch/
│       └── speech.launch.py
│
├── car1_llm/                 # 大模型交互包
│   ├── car1_llm/
│   │   ├── chat_node.py         # 对话管理节点
│   │   └── skill_executor.py   # 技能执行节点
│   ├── config/
│   │   ├── llm.yaml             # LLM 配置（API key 等）
│   │   └── skills.yaml          # 技能映射配置
│   └── launch/
│       └── llm.launch.py
│
└── car1_interaction/         # 集成包
    ├── launch/
    │   └── full_interaction.launch.py  # 一键启动所有交互组件
    └── config/
        └── interaction.yaml
```

---

## 功能概述

### 1. 语音交互 (car1_speech)

**完整语音链路**：
```
用户说话 → 语音唤醒 → 语音识别 → 大模型处理 → 语音合成 → 播放回复
```

#### 1.1 语音唤醒 (wake_word_node)

- **功能**：持续监听麦克风，检测唤醒词（如"你好机器人"）
- **支持引擎**：Porcupine, Snowboy, 自定义模型
- **话题**：
  - 发布 `/interaction/wake_word/detected` (Bool) — 唤醒事件
  - 发布 `/interaction/wake_word/text` (String) — 唤醒词文本

#### 1.2 语音识别 (asr_node)

- **功能**：将语音转换为文字
- **支持引擎**：讯飞、百度、Google、Whisper
- **话题**：
  - 订阅 `/interaction/wake_word/detected` — 监听唤醒事件
  - 发布 `/interaction/asr/text` (String) — 识别结果
  - 发布 `/interaction/asr/is_listening` (Bool) — 监听状态

#### 1.3 语音合成 (tts_node)

- **功能**：将文字转换为语音并播放
- **支持引擎**：讯飞、百度、Google、Edge-TTS、pyttsx3
- **话题**：
  - 订阅 `/interaction/tts/speak` (String) — 待合成文本
  - 发布 `/interaction/tts/is_speaking` (Bool) — 播放状态
- **服务**：
  - `/interaction/tts/stop` (Trigger) — 停止播放

---

### 2. 大模型交互 (car1_llm)

#### 2.1 对话管理 (chat_node)

- **功能**：与大模型对话，理解用户意图并生成回复
- **支持模型**：OpenAI GPT、Claude、本地 Ollama、讯飞星火
- **话题**：
  - 订阅 `/interaction/asr/text` — 语音识别结果
  - 订阅 `/interaction/llm/user_input` — 直接文本输入
  - 发布 `/interaction/llm/assistant_reply` — 助手回复
  - 发布 `/interaction/tts/speak` — 自动发送到 TTS
  - 发布 `/interaction/llm/skill_request` — 技能调用请求

**技能标记格式**：
```
[SKILL:skill_name|param1:value1|param2:value2]
```

示例：
```
用户：导航到厨房
助手：好的，我现在开始导航。[SKILL:navigate_to|location:kitchen]
```

#### 2.2 技能执行 (skill_executor)

- **功能**：解析技能请求，调用对应的 ROS 服务/动作
- **话题**：
  - 订阅 `/interaction/llm/skill_request` — 技能请求

**技能映射示例**（`skills.yaml`）：
```yaml
skills:
  navigate_to:
    type: "action"
    action: "/navigate_to_pose"
    parameters:
      location:
        mapping:
          kitchen: [1.0, 2.0, 0.0]
          bedroom: [2.0, 0.0, 1.57]

  grasp_object:
    type: "service"
    service: "/visual_grasp/start"

  transport:
    type: "service"
    service: "/navigation_grasp_transport/start"
```

---

## 快速开始

### 编译

```bash
cd ~/ros2_ws
source /opt/ros/humble/setup.bash

# 编译交互模块
colcon build --packages-select car1_speech car1_llm --symlink-install

source install/setup.bash
```

### 配置 API 密钥

编辑配置文件，填入你的 API 密钥：

**语音识别/合成**（讯飞）：
```bash
nano src/xflz_eai_car1/interaction/car1_speech/config/asr.yaml
nano src/xflz_eai_car1/interaction/car1_speech/config/tts.yaml

# 填入：
app_id: "your_app_id"
api_key: "your_api_key"
api_secret: "your_api_secret"
```

**大模型**（OpenAI）：
```bash
nano src/xflz_eai_car1/interaction/car1_llm/config/llm.yaml

# 填入：
api_key: "sk-..."
```

### 启动

**方式 1：完整启动（语音 + LLM）**
```bash
ros2 launch car1_interaction full_interaction.launch.py
```

**方式 2：分别启动**
```bash
# 终端 1：语音链路
ros2 launch car1_speech speech.launch.py

# 终端 2：LLM
ros2 launch car1_llm llm.launch.py
```

### 测试

**测试语音合成**：
```bash
ros2 topic pub /interaction/tts/speak std_msgs/msg/String "{data: '你好，我是小飞机器人'}" --once
```

**测试对话**：
```bash
ros2 topic pub /interaction/llm/user_input std_msgs/msg/String "{data: '导航到厨房'}" --once

# 查看回复
ros2 topic echo /interaction/llm/assistant_reply
```

---

## 实现路线图

### Phase 1: 框架搭建 ✅
- [x] 创建包结构
- [x] 定义话题/服务接口
- [x] 实现节点框架

### Phase 2: 语音模块（TODO）
- [ ] 集成讯飞语音唤醒
- [ ] 集成讯飞语音识别
- [ ] 集成讯飞语音合成
- [ ] 音频设备配置与测试

### Phase 3: LLM 模块（TODO）
- [ ] 集成 OpenAI API
- [ ] 集成讯飞星火 API
- [ ] Prompt 工程与优化
- [ ] 对话历史管理

### Phase 4: 技能执行（TODO）
- [ ] 实现导航技能
- [ ] 实现抓取技能
- [ ] 实现运输技能
- [ ] 技能确认机制

### Phase 5: 端到端测试（TODO）
- [ ] 语音唤醒 → 识别 → LLM → 合成 完整流程
- [ ] 多轮对话测试
- [ ] 技能执行准确性测试
- [ ] 鲁棒性测试（噪声环境、口音）

---

## 依赖安装

### Python 依赖

```bash
# 讯飞 SDK
pip3 install websocket-client

# OpenAI
pip3 install openai

# Anthropic Claude
pip3 install anthropic

# 本地模型
pip3 install ollama

# 音频处理
pip3 install pyaudio sounddevice

# 离线 TTS（可选）
pip3 install pyttsx3

# 本地 ASR（可选）
pip3 install openai-whisper
```

### 讯飞账号申请

1. 注册讯飞开放平台账号：https://www.xfyun.cn/
2. 创建应用：
   - 语音听写（ASR）
   - 语音合成（TTS）
   - 语音唤醒（可选）
3. 获取 `APPID`, `API_KEY`, `API_SECRET`

---

## 常见问题

### Q: 麦克风权限问题
```bash
# 检查音频设备
arecord -l

# 测试录音
arecord -d 3 test.wav
aplay test.wav

# 授权当前用户
sudo usermod -aG audio $USER
```

### Q: 讯飞 API 调用失败
- 检查网络连接
- 确认 API 密钥正确
- 查看讯飞控制台配额是否用完

### Q: OpenAI API 国内访问问题
- 使用代理
- 或使用国内镜像 API（设置 `api_base`）

---

## 相关资源

- [讯飞语音文档](https://www.xfyun.cn/doc/)
- [OpenAI API 文档](https://platform.openai.com/docs)
- [Ollama 本地模型](https://ollama.ai/)
- [ROS2 官方教程](https://docs.ros.org/en/humble/)

---

**当前状态**：框架已完成，待集成具体 API  
**下一步**：实现语音唤醒 + ASR（讯飞）

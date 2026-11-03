# STM32F407VET6 固件烧录指南

## 📁 目录说明

```
ros2_ws/
└── fw/                     # 固件目录
    ├── flash_stm32.py      # 一键烧录脚本
    ├── README.md           # 本文档
    ├── base.hex            # 下位机固件（STM32F407VET6）
    ├── Keil.STM32F4xx_DFP.3.1.1.pack  # STM32F407 CMSIS pack（pyocd 烧录依赖）
    └── tools/              # 配置文件
        └── 99-wch-link.rules  # USB 权限配置
```

---

## 🚀 快速烧录（推荐）

### 一键烧录默认固件

```bash
cd /home/iflytek/ros2_ws/fw

# 一条命令完成所有操作
python3 flash_stm32.py
```

**脚本会自动完成**：
1. ✅ 检查并安装 pyocd（如未安装）
2. ✅ 配置 USB 权限（udev 规则）
3. ✅ 检测 WCH-Link 连接
4. ✅ 烧录固件到 STM32F407VET6

### 关于 pyocd 与 CMSIS Pack

pyocd 内置 target 列表**不包含 stm32f407ve**，
烧录脚本会自动通过 `--pack` 参数加载同目录下的
`Keil.STM32F4xx_DFP.3.1.1.pack`（STM32F4xx 官方 CMSIS Device Family Pack）来提供目标定义。

- pack 文件与脚本同目录（`fw/Keil.STM32F4xx_DFP.3.1.1.pack`），无需额外安装
- 只需 `python3 flash_stm32.py` 即可，脚本自动附带 `--pack`

### 烧录指定固件

```bash
# 烧录其他 .hex 文件
python3 flash_stm32.py path/to/firmware.hex

# 烧录 .bin 文件
python3 flash_stm32.py firmware.bin
```

---

## 📋 硬件连接

### WCH-Link 接线（SWD 接口）

| WCH-Link | STM32F407 | 说明 |
|----------|----------|------|
| **GND**  | **GND**  | **必须连接** |
| **SWDIO** | **PA13** | 数据线 |
| **SWCLK** | **PA14** | 时钟线 |
| 3V3      | 3V3      | 可选（如板上已供电可不接） |

**重要**：
- GND 必须连接，否则无法通信
- 确认目标板已供电（LED 亮起）
- WCH-Link 无需切换模式（支持 `1a86:8011` 和 `1a86:8012`）

---

## 🔧 手动安装（可选）

如果需要手动安装 pyocd：

```bash
# 使用国内镜像源安装（推荐）
pip3 install pyocd==0.36.0 -i https://pypi.tuna.tsinghua.edu.cn/simple

# 验证安装
pyocd --version
```

**注意**：
- 建议使用 `pyocd 0.36.0` 版本（稳定）
- 使用清华镜像源加速下载
- 脚本会自动执行此操作，无需手动安装

---

## 🔍 故障排查

### 问题 1：未检测到 WCH-Link

**症状**：
```
✗ 未检测到 WCH-Link USB 设备
```

**解决**：
1. 检查 USB 连接是否牢固
2. 运行 `lsusb | grep 1a86` 确认设备存在
3. 尝试更换 USB 端口或线缆
4. 重新插拔 WCH-Link

---

### 问题 2：权限错误

**症状**：
```
Error: could not open device 0x1a86:0x8011: Access denied
```

**解决**：
```bash
# 方式 1：重新运行烧录脚本（推荐，会自动配置）
python3 flash_stm32.py

# 方式 2：手动配置权限
sudo cp fw/tools/99-wch-link.rules /etc/udev/rules.d/
sudo udevadm control --reload-rules
sudo udevadm trigger

# 重新插拔 WCH-Link
```

---

### 问题 3：连接目标失败

**症状**：
```
Error: Target is not responding
```

**解决**：
1. **检查硬件连接**：
   - SWDIO、SWCLK、GND 是否正确连接
   - 目标板是否供电（LED 亮起）
   
2. **尝试按住复位键烧录**：
   ```bash
   # 按住目标板的复位按钮，同时执行烧录
   python3 flash_stm32.py
   ```

3. **降低 SWD 频率**（手动模式）：
   ```bash
   pyocd flash fw/base.hex --target stm32f407ve -a 0x08000000 --frequency 1000000
   ```

---

### 问题 4：pyocd 安装失败

**症状**：
```
pip install 超时或失败
```

**解决**：
```bash
# 方式 1：使用清华镜像源（推荐）
pip3 install pyocd==0.36.0 -i https://pypi.tuna.tsinghua.edu.cn/simple

# 方式 2：使用阿里云镜像源
pip3 install pyocd==0.36.0 -i https://mirrors.aliyun.com/pypi/simple/

# 方式 3：使用中科大镜像源
pip3 install pyocd==0.36.0 -i https://pypi.mirrors.ustc.edu.cn/simple/
```

---

## 🎯 手动烧录命令

如果脚本无法工作，可以使用手动命令：

```bash
cd /home/iflytek/ros2_ws/fw

# 1. 检查 WCH-Link 连接
lsusb | grep 1a86
# 期望输出: Bus XXX Device XXX: ID 1a86:8011 QinHeng Electronics WCH-Link

# 2. 检测调试器
pyocd list
# 期望输出:
#   #   Probe/Board       Unique ID      Target  
# -----------------------------------------------
#   0   wch.cn WCH-Link   0001A0000001   n/a

# 3. 烧录固件
pyocd flash base.hex --target stm32f407ve -a 0x08000000

# 4. 烧录并验证
pyocd flash base.hex --target stm32f407ve -a 0x08000000 --verify

# 5. 全片擦除后烧录
pyocd erase --chip --target stm32f407ve
pyocd flash base.hex --target stm32f407ve -a 0x08000000
```

---

## 📊 支持的文件格式

| 格式 | 说明 | 使用示例 |
|-----|------|---------|
| `.hex` | Intel HEX 格式（推荐） | `python3 flash_stm32.py base.hex` |
| `.bin` | 原始二进制格式 | `python3 flash_stm32.py base.bin` |
| `.elf` | ELF 格式（包含符号） | `python3 flash_stm32.py base.elf` |

**推荐使用 `.hex` 格式**，因为它包含地址信息，不容易出错。

---

## 🎯 目标芯片信息

- **芯片型号**: STM32F407VET6
- **CPU 架构**: ARM Cortex-M4
- **Flash 大小**: 512 KB
- **RAM 大小**: 192 KB (128KB + 64KB CCM)
- **Flash 起始地址**: 0x08000000
- **pyocd target**: `stm32f407ve` 或 `stm32f407vetx`

---

## 💡 常用命令速查

```bash
# 一键烧录（推荐）
python3 flash_stm32.py

# 烧录指定固件
python3 flash_stm32.py custom.hex

# 检查 WCH-Link 连接
lsusb | grep 1a86
pyocd list

# 查看支持的芯片列表
pyocd list --targets | grep stm32f407

# 全片擦除
pyocd erase --chip --target stm32f407ve

# 复位芯片
pyocd reset --target stm32f407ve
```

---

## 📖 参考资源

- **pyocd 官方文档**: https://pyocd.io/
- **WCH-Link 资料**: http://www.wch.cn/products/WCH-Link.html
- **STM32F407 数据手册**: https://www.st.com/resource/en/datasheet/stm32f407vg.pdf
- **清华镜像源**: https://mirrors.tuna.tsinghua.edu.cn/help/pypi/

---

## ✨ 总结

**最简单的使用方式**：

```bash
cd /home/iflytek/ros2_ws
python3 flash_stm32.py
```

一条命令搞定所有配置和烧录！🚀

**关键点**：
- ✅ 使用 pyocd 工具（稳定可靠）
- ✅ 支持 WCH-Link (1a86:8011/8012)
- ✅ 自动安装依赖和配置权限
- ✅ 使用国内镜像源（清华源）
- ✅ 无需手动切换 WCH-Link 模式

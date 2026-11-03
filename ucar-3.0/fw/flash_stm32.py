#!/usr/bin/env python3
"""
STM32F407VET6 下位机固件烧录工具
一键完成所有配置和烧录操作
"""

import os
import sys
import subprocess
import shutil
from pathlib import Path

# 配置
SCRIPT_DIR = Path(__file__).parent
FW_DIR = SCRIPT_DIR
DEFAULT_HEX = FW_DIR / "base.hex"
UDEV_RULES = FW_DIR / "tools" / "99-wch-link.rules"
TARGET_CHIP = "stm32f407ve"
FLASH_ADDR = "0x08000000"
# STM32F4xx CMSIS Device Family Pack：
# pyocd 内置 target 列表不含 stm32f407ve，烧录需通过 --pack 提供目标定义。
# pack 与脚本同目录（fw/Keil.STM32F4xx_DFP.3.1.1.pack）。
PACK_FILE = FW_DIR / "Keil.STM32F4xx_DFP.3.1.1.pack"

# 颜色输出
class Color:
    GREEN = '\033[92m'
    YELLOW = '\033[93m'
    RED = '\033[91m'
    BLUE = '\033[94m'
    CYAN = '\033[96m'
    END = '\033[0m'
    BOLD = '\033[1m'

def print_header(msg):
    print(f"\n{Color.BOLD}{Color.CYAN}{'='*60}{Color.END}")
    print(f"{Color.BOLD}{Color.CYAN}{msg:^60}{Color.END}")
    print(f"{Color.BOLD}{Color.CYAN}{'='*60}{Color.END}\n")

def print_step(step, msg):
    print(f"{Color.BLUE}[{step}]{Color.END} {msg}")

def print_success(msg):
    print(f"{Color.GREEN}✓{Color.END} {msg}")

def print_warning(msg):
    print(f"{Color.YELLOW}⚠{Color.END} {msg}")

def print_error(msg):
    print(f"{Color.RED}✗{Color.END} {msg}")

def print_info(msg):
    print(f"{Color.CYAN}ℹ{Color.END} {msg}")


def check_pyocd():
    """检查并安装 pyocd"""
    print_step("1/4", "检查 pyocd...")

    try:
        result = subprocess.run(['pyocd', '--version'],
                              capture_output=True, text=True, timeout=5)
        if result.returncode == 0:
            version_output = result.stdout.strip().split()
            version = version_output[1] if len(version_output) > 1 else "unknown"
            print_success(f"pyocd 已安装 (版本 {version})")
            return True
    except FileNotFoundError:
        pass
    except Exception as e:
        pass  # 静默失败，继续安装

    # 需要安装
    print_warning("pyocd 未安装，正在安装...")
    print_info("安装命令: pip install pyocd==0.36.0")

    try:
        # 指定版本和国内镜像源
        subprocess.run([sys.executable, '-m', 'pip', 'install', 'pyocd==0.36.0',
                       '-i', 'https://pypi.tuna.tsinghua.edu.cn/simple'],
                      check=True)
        print_success("pyocd 安装成功")
        return True
    except subprocess.CalledProcessError:
        print_error("pyocd 安装失败")
        print_info("请手动安装:")
        print("  pip3 install pyocd==0.36.0 -i https://pypi.tuna.tsinghua.edu.cn/simple")
        return False


def setup_udev_rules():
    """配置 USB 权限"""
    print_step("2/4", "配置 USB 权限...")

    if not UDEV_RULES.exists():
        print_error(f"udev 规则文件不存在: {UDEV_RULES}")
        return False

    udev_dest = Path("/etc/udev/rules.d/99-wch-link.rules")

    # 检查是否已安装
    if udev_dest.exists():
        # 对比内容是否一致
        try:
            if udev_dest.read_text() == UDEV_RULES.read_text():
                print_success("udev 规则已配置")
                return True
        except:
            pass

    # 需要安装
    print_info("正在配置 udev 规则...")
    try:
        # 复制规则文件
        subprocess.run(['sudo', 'cp', str(UDEV_RULES), str(udev_dest)], check=True)
        print_success("udev 规则已复制")

        # 重新加载
        subprocess.run(['sudo', 'udevadm', 'control', '--reload-rules'],
                      check=True, capture_output=True)
        subprocess.run(['sudo', 'udevadm', 'trigger', '--action=add'],
                      check=True, capture_output=True)
        print_success("udev 规则已生效")

        print_warning("如果仍有权限问题，请重新插拔 WCH-Link")
        return True

    except subprocess.CalledProcessError as e:
        print_error("udev 规则配置失败")
        print_info("请手动执行:")
        print(f"  sudo cp {UDEV_RULES} /etc/udev/rules.d/")
        print("  sudo udevadm control --reload-rules")
        print("  sudo udevadm trigger")
        return False


def check_wchlink():
    """检查 WCH-Link 连接"""
    print_step("3/4", "检查 WCH-Link 连接...")

    try:
        # 检查 USB 设备
        result = subprocess.run(['lsusb'], capture_output=True, text=True, timeout=3)
        if '1a86:8011' in result.stdout or '1a86:8012' in result.stdout:
            usb_id = '8011' if '8011' in result.stdout else '8012'
            print_success(f"检测到 WCH-Link (USB ID: 1a86:{usb_id})")
        else:
            print_error("未检测到 WCH-Link USB 设备")
            print_info("请检查:")
            print("  1. WCH-Link 是否已连接")
            print("  2. USB 线缆是否正常")
            print("  3. 运行 'lsusb' 查看设备列表")
            return False

        # 检查 pyocd 是否能识别
        result = subprocess.run(['pyocd', 'list'],
                              capture_output=True, text=True, timeout=10)

        if 'WCH-Link' in result.stdout or 'wch.cn' in result.stdout:
            print_success("pyocd 已识别调试器")
            # 显示调试器信息
            for line in result.stdout.split('\n'):
                if 'wch' in line.lower() or 'WCH' in line:
                    print_info(f"  {line.strip()}")
            return True
        else:
            print_warning("pyocd 未识别到调试器")
            print_info("调试器列表:")
            print(result.stdout)
            print_warning("可能需要重新插拔 WCH-Link")
            return False

    except Exception as e:
        print_error(f"检查失败: {e}")
        return False


def flash_firmware(hex_file: Path):
    """烧录固件"""
    print_step("4/4", "烧录固件...")

    if not hex_file.exists():
        print_error(f"固件文件不存在: {hex_file}")
        return False

    print_info(f"固件文件: {hex_file.name}")
    print_info(f"目标芯片: {TARGET_CHIP.upper()}")
    print_info(f"Flash 地址: {FLASH_ADDR}")
    print()

    # 构建命令
    cmd = [
        'pyocd', 'flash',
        str(hex_file),
        '--target', TARGET_CHIP,
        '-a', FLASH_ADDR
    ]

    # stm32f407ve 不在 pyocd 内置 target，需 --pack 指定本地 CMSIS pack
    if PACK_FILE.exists():
        cmd += ['--pack', str(PACK_FILE)]
    else:
        print_warning(f"未找到 CMSIS pack: {PACK_FILE}")
        print_info("若烧录失败，请确认 fw/Keil.STM32F4xx_DFP.3.1.1.pack 存在")

    print_info(f"执行命令: {' '.join(cmd)}")
    print()

    # 执行烧录
    try:
        result = subprocess.run(cmd, timeout=120)

        if result.returncode == 0:
            print()
            print_success("✓ 固件烧录成功！")
            return True
        else:
            print()
            print_error("✗ 固件烧录失败")
            return False

    except subprocess.TimeoutExpired:
        print_error("烧录超时（120秒）")
        return False
    except KeyboardInterrupt:
        print()
        print_warning("用户中断烧录")
        return False
    except Exception as e:
        print_error(f"烧录出错: {e}")
        return False


def main():
    """主函数"""
    print_header("STM32F407VET6 下位机固件烧录工具")

    # 解析参数
    hex_file = DEFAULT_HEX
    if len(sys.argv) > 1:
        hex_file = Path(sys.argv[1])
        if not hex_file.is_absolute():
            hex_file = Path.cwd() / hex_file

    # 检查固件文件
    if not hex_file.exists():
        print_error(f"固件文件不存在: {hex_file}")
        print_info("使用方法:")
        print(f"  {sys.argv[0]}                    # 烧录默认固件 (fw/base.hex)")
        print(f"  {sys.argv[0]} path/to/firmware.hex  # 烧录指定固件")
        return 1

    print_info(f"将要烧录: {hex_file}")
    print()

    # 步骤 1: 检查并安装 pyocd
    if not check_pyocd():
        return 1

    # 步骤 2: 配置 USB 权限
    if not setup_udev_rules():
        print_warning("udev 配置失败，但继续尝试烧录...")

    # 步骤 3: 检查 WCH-Link
    if not check_wchlink():
        return 1

    # 步骤 4: 烧录固件
    if not flash_firmware(hex_file):
        return 1

    print()
    print_header("✓ 所有操作完成")
    print_info("下位机固件已成功烧录到 STM32F407VET6")
    print()

    return 0


if __name__ == '__main__':
    try:
        sys.exit(main())
    except KeyboardInterrupt:
        print()
        print_warning("用户中断")
        sys.exit(130)

#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""RK3576 整机产测 · 纯终端版（无 UI / 不联网 / 不写日志文件）

用法:
    python3 prod_test.py                  # 交互菜单
    python3 prod_test.py --list            # 列出测试项
    python3 prod_test.py --only ddr,emmc    # 只跑指定项，打印汇总后退出

按键（单键即时生效，无需回车）:
    项前:  ⏎ 执行 | 空格 跳过(计 FAIL) | b 上一项 | q 中止
    项后:  ⏎ PASS | 空格 FAIL | r 重测 | b 上一项 | q 中止 | ? 帮助
    菜单:  1..9 单项 | A 全流程 | l 重列 | q 退出

判定标记: PASS / PASS(WARN) / FAIL ；退出码 0=无 FAIL, 1=有 FAIL, 2=中止
"""

import argparse
import array
import math
import os
import re
import shutil
import signal
import subprocess
import sys
import tempfile
import termios
import unicodedata
import time
import tty
import wave

try:
    import numpy as np
except ImportError:                 # 板上有 numpy；缺失时 mic 项记 FAIL
    np = None

try:
    from PIL import Image
except ImportError:
    Image = None

C_RESET = "\033[0m"
C_BOLD = "\033[1m"
C_GREEN = "\033[32m"
C_RED = "\033[31m"
C_YELLOW = "\033[33m"
C_CYAN = "\033[36m"


def _dwidth(text):
    """字符串显示宽度：CJK/全角字符按 2 列计。"""
    return sum(2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
               for ch in str(text))


def _pad(text, width, align="left"):
    """按显示宽度补齐（或截断加 …），保证多行表格对齐。"""
    text = str(text)
    if _dwidth(text) > width:
        out, used = "", 0
        for ch in text:
            cw = 2 if unicodedata.east_asian_width(ch) in ("W", "F") else 1
            if used + cw > width - 1:
                break
            out += ch
            used += cw
        return out + "…"
    fill = " " * (width - _dwidth(text))
    return fill + text if align == "right" else text + fill


def verdict_color(verdict):
    if verdict == "FAIL":
        return C_RED
    return C_YELLOW if "WARN" in verdict else C_GREEN


def key_of_line(line):
    """非 tty 行模式：把一整行归一化成按键名。"""
    raw = line.rstrip("\n")
    if raw == "":
        return "enter"
    if raw.strip() == "":
        return "space"
    return raw[0].lower()


class Term:
    """终端输出与按键。交互时 cbreak 单键；非 tty 时逐行读取（便于管道回归）。"""

    def __init__(self):
        self.interactive = sys.stdin.isatty() and sys.stdout.isatty()
        term = os.environ.get("TERM", "")
        self.color = (self.interactive and not os.environ.get("NO_COLOR")
                      and term not in ("", "dumb"))
        try:
            self.width = shutil.get_terminal_size().columns
        except Exception:
            self.width = 80
        self._saved = None

    def enter_raw(self):
        if not self.interactive:
            return
        try:
            fd = sys.stdin.fileno()
            self._saved = termios.tcgetattr(fd)
            tty.setcbreak(fd)
        except Exception:
            self._saved = None

    def exit_raw(self):
        if self._saved is None:
            return
        try:
            termios.tcsetattr(sys.stdin.fileno(), termios.TCSADRAIN, self._saved)
        except Exception:
            pass
        self._saved = None

    def w(self, text="", color=None):
        if color and self.color:
            print(color + text + C_RESET)
        else:
            print(text)

    def clear(self):
        """清屏（仅交互式终端；管道/重定向时不动，避免把转义码写进日志）。"""
        if self.interactive:
            sys.stdout.write("\033[2J\033[H")
            sys.stdout.flush()

    def hr(self, ch="─"):
        self.w(ch * max(40, min(self.width, 78)), C_CYAN)

    def paint(self, text, color):
        """按终端能力给文本上色（不可着色时原样返回）。"""
        return (color + text + C_RESET) if (color and self.color) else text

    def key(self, prompt, allowed):
        """读一个 allowed 里的按键（小写归一）；非法键提示后重读。"""
        allowed = set(k.lower() for k in allowed)
        while True:
            if not self.interactive:
                line = sys.stdin.readline()
                if line == "":
                    return "q" if "q" in allowed else None
                k = key_of_line(line)
            else:
                sys.stdout.write(prompt)
                sys.stdout.flush()
                ch = sys.stdin.read(1)
                if ch == "":
                    return "q" if "q" in allowed else None
                if ch in ("\r", "\n"):
                    k = "enter"
                elif ch == " ":
                    k = "space"
                else:
                    k = ch.lower()
            if k in allowed:
                return k
            self.w("  无效按键（可用: %s）" % " ".join(sorted(allowed)), C_YELLOW)


# ══════════════════════════════════════
#  常量（来源: hw_test_gui.py 现网已验证值；RGB 映射经实机 strace 验证）
# ══════════════════════════════════════
DDR_PASS_RATIO, DDR_WARN_RATIO = 0.50, 0.40
EMMC_READ_EXCELLENT, EMMC_READ_PASS, EMMC_WRITE_PASS, EMMC_READ_WARN = 250.0, 150.0, 100.0, 100.0
MIC_RMS_MIN, MIC_DC_MAX = 50.0, 500.0
LIDAR_HZ_MIN, LIDAR_HZ_MAX = 5.0, 15.0
CAM_RGB_HZ_MIN, CAM_DEPTH_HZ_MIN = 1.0, 5.0
SAMPLE_SECS = 1.0                     # 雷达/相机采样窗口
DDR_TEST_SECS = 10
EMMC_READ_MB, EMMC_WRITE_MB = 256, 128

DEV_STM32 = "/dev/car1_stm32"
KWS_WAV = ("/home/iflytek/ros2_ws/src/xflz_eai_car1/interaction/"
           "car1_interaction/tools/resource/kws.wav")
ROS_SETUP = "source /opt/ros/humble/setup.bash && source ~/ros2_ws/install/setup.bash"
TOPIC_RGB = "/car1_base_driver/set_rgb"
TOPIC_CMD_VEL = "/controller/cmd_vel"
TOPIC_SERVO = "/servo_controller"
TOPIC_SCAN = "/scan"
TOPIC_CAM_RGB = "/ascamera_hp60c/camera_publisher/rgb0/image"
TOPIC_CAM_DEPTH = "/ascamera_hp60c/camera_publisher/depth0/image_raw"
# 就绪判定用真实进程（ros2 node list 会被 daemon 缓存污染，实测不可信）
PROC_BASE_DRIVER = "lib/car1_base_driver/base_driver"
PROC_PERIPH = ("m1ct[_]d2", "ascamera[_]node")
# ROS encoding → (PIL 目标模式, 原始像素布局)
PIL_MODE_MAP = {"rgb8": ("RGB", "RGB"), "bgr8": ("RGB", "BGR"),
                "mono8": ("L", "L"), "8UC1": ("L", "L"), "8UC3": ("RGB", "RGB")}

RGB_INDEX_MAP = (0, 1, 8, 9, 10, 11, 12, 13, 14, 15)
ALL_LEDS = tuple(range(10))
RGB_WHITE, RGB_RED, RGB_GREEN, RGB_BLUE = (255, 255, 255), (255, 0, 0), (0, 255, 0), (0, 0, 255)
RGB_OFF = (0, 0, 0)

SERVO_CENTER = [1500] * 6
SERVO_MOTION = [1500, 1100, 2200, 1850, 1500, 1500]
SERVO_GRIPPER_OPEN, SERVO_GRIPPER_CLOSE = 1000, 1500
SERVO_HOLD_S = 3.0                    # 每个姿态保持时长

# 汇总表列宽（显示列）：#, 项, 结果, 用时；关键实测值列取剩余宽度
SUM_COLS = (3, 14, 11, 7)

CHASSIS_MOVES = [("前进", 0.1, 0.0, 0.0), ("后退", -0.1, 0.0, 0.0),
                 ("左移", 0.0, 0.1, 0.0), ("右移", 0.0, -0.1, 0.0),
                 ("左转", 0.0, 0.0, 0.5), ("右转", 0.0, 0.0, -0.5)]
CHASSIS_HOLD_S = 1.0


def rgb_band_label(logical):
    if logical < 2:
        return "2颗灯带 第%d颗" % (logical + 1)
    return "8颗灯带 第%d颗" % (logical - 1)


def rgb_pixels(logicals, color):
    """逻辑索引 → [(ROS index(1 基), r, g, b)]"""
    r, g, b = color
    return [(RGB_INDEX_MAP[i] + 1, r, g, b) for i in logicals]


def rgb_full(color):
    return rgb_pixels(ALL_LEDS, color)


# 23 步 ≈ 9.1 秒；步 = (标签, 逻辑索引, RGB, 驻留秒)
RGB_SEQUENCE = (
    [("跑马灯 逻辑%d" % i, [i], RGB_RED, 0.25) for i in ALL_LEDS]
    + [("全部 10 颗 白", list(ALL_LEDS), RGB_WHITE, 1.5),
       ("全部 10 颗 红", list(ALL_LEDS), RGB_RED, 1.0),
       ("全部 10 颗 绿", list(ALL_LEDS), RGB_GREEN, 1.0),
       ("全部 10 颗 蓝", list(ALL_LEDS), RGB_BLUE, 1.0)]
    + [s for k in range(4) for s in
       (("闪烁 %d 亮" % (k + 1), list(ALL_LEDS), RGB_WHITE, 0.2),
        ("闪烁 %d 灭" % (k + 1), list(ALL_LEDS), RGB_OFF, 0.2))]
    + [("收尾 全灭", list(ALL_LEDS), RGB_OFF, 0.5)]
)


# ══════════════════════════════════════
#  纯函数：解析与判定（可单测）
# ══════════════════════════════════════
_STRESS_A = re.compile(r"memrate\s+([\d.]+)\s+(read|write|read64pf)(\d+)\s+MB/sec")
_STRESS_B = re.compile(r"(read64pf|read|write)\s*(\d*)[^\d]*?([\d.]+)\s*MB/sec")
_DD_RE = re.compile(r"([\d.]+)\s*MB/s")


def parse_stress_ng(text):
    """解析 stress-ng --memrate 输出 → {'read': 最大读带宽, 'write': 最大写带宽}。

    stress-ng 各版本行格式不同，两种都认：
      A「数值在前」: memrate 9000.0 read64pf1024 MB/sec
      B「类型在前」: memrate (write1024)   10535.0 MB/sec
    行里带块大小且 <64B 时跳过（与小 GUI 的判定口径一致）。
    """
    bw = {}
    for line in text.splitlines():
        if "MB/sec" not in line:
            continue
        m = _STRESS_A.search(line)
        if m:
            num, kind, blk = float(m.group(1)), m.group(2), int(m.group(3))
        else:
            m = _STRESS_B.search(line)
            if not m:
                continue
            kind, blk, num = m.group(1), int(m.group(2) or 0), float(m.group(3))
        if blk and blk < 64:
            continue
        key = "read" if kind in ("read", "read64pf") else "write"
        bw[key] = max(bw.get(key, 0.0), num)
    return bw


def verdict_ddr(measured_mbs, theory_mbs):
    if theory_mbs <= 0:
        return "PASS(WARN)", "无法计算理论带宽，实测 %.0f MB/s" % measured_mbs
    pct = measured_mbs / theory_mbs
    note = "%.0f MB/s (%.0f%% 理论)" % (measured_mbs, pct * 100)
    if pct >= DDR_PASS_RATIO:
        return "PASS", note
    if pct >= DDR_WARN_RATIO:
        return "PASS(WARN)", note
    return "FAIL", note


def parse_dd_rate(text):
    m = _DD_RE.search(text or "")
    return float(m.group(1)) if m else 0.0


def verdict_emmc(read_mbs, write_mbs):
    note = "读 %.0f / 写 %.0f MB/s" % (read_mbs, write_mbs)
    if read_mbs >= EMMC_READ_EXCELLENT:
        return "PASS", note
    if read_mbs >= EMMC_READ_PASS and write_mbs >= EMMC_WRITE_PASS:
        return "PASS", note
    if read_mbs >= EMMC_READ_WARN:
        return "PASS(WARN)", note
    return "FAIL", note


def verdict_mic(rms, dc):
    note = "RMS=%.1f 直流=%.1f" % (rms, dc)
    if rms < MIC_RMS_MIN:
        return "FAIL", note + " (无信号)"
    if abs(dc) > MIC_DC_MAX:
        return "PASS(WARN)", note + " (直流偏置偏高)"
    return "PASS", note


def verdict_lidar(hz, points, min_range=None):
    note = "帧率 %.1f Hz / 有效点 %d" % (hz, points)
    if min_range:
        note += " / 最近 %.2f m" % min_range
    if points <= 0:
        return "FAIL", note + " (无数据)"
    if LIDAR_HZ_MIN <= hz <= LIDAR_HZ_MAX:
        return "PASS", note
    return "PASS(WARN)", note + " (帧率越界)"


def verdict_cam(rgb_hz, depth_hz):
    note = "彩色 %.1f Hz / 深度 %.1f Hz" % (rgb_hz, depth_hz)
    if rgb_hz <= 0 and depth_hz <= 0:
        return "FAIL", note + " (无数据)"
    if rgb_hz >= CAM_RGB_HZ_MIN and depth_hz >= CAM_DEPTH_HZ_MIN:
        return "PASS", note
    return "PASS(WARN)", note + " (帧率偏低)"


def parse_only(text, valid_ids):
    ids = [x.strip() for x in (text or "").split(",") if x.strip()]
    bad = [x for x in ids if x not in valid_ids]
    if bad:
        raise ValueError("未知测试项: %s（可用: %s）" % (", ".join(bad), ", ".join(valid_ids)))
    if not ids:
        raise ValueError("--only 未指定测试项")
    return ids


def render_summary(rows, color=False, width=78):
    """rows: [(iid, name, verdict, note, seconds)] → 汇总表（中文按显示宽度对齐）。"""
    def paint(text, col):
        return (col + text + C_RESET) if color else text

    w_num, w_name, w_res, w_sec = SUM_COLS
    w_note = max(20, width - (w_num + w_name + w_res + w_sec) - 4)
    bar = "━" * min(width, 78)
    lines = [paint(bar, C_CYAN),
             paint("  " + _pad("#", w_num) + _pad("项", w_name) + _pad("结果", w_res)
                   + _pad("关键实测值", w_note) + _pad("用时", w_sec, "right"), C_BOLD)]
    n_pass = n_fail = 0
    for i, (iid, name, verdict, note, secs) in enumerate(rows, 1):
        if verdict == "FAIL":
            n_fail += 1
        else:
            n_pass += 1
        lines.append("  " + _pad(i, w_num) + _pad(name, w_name)
                     + paint(_pad(verdict, w_res), verdict_color(verdict))
                     + _pad(note, w_note) + _pad("%.1fs" % secs, w_sec, "right"))
    lines.append(paint("─" * min(width, 78), C_CYAN))
    overall = "全部通过" if n_fail == 0 else "%d 项未通过" % n_fail
    lines.append("  " + paint("合计:  PASS %d   FAIL %d" % (n_pass, n_fail),
                              C_GREEN if n_fail == 0 else C_RED)
                 + "    结论: " + paint(overall, C_GREEN if n_fail == 0 else C_RED))
    return "\n".join(lines)


# ══════════════════════════════════════
#  命令辅助与自动项（ddr / emmc / mic）
# ══════════════════════════════════════
def _run(cmd, timeout=120, shell=False):
    """跑命令，返回 (rc, stdout+stderr)。shell=True 走 bash -lc。"""
    try:
        r = subprocess.run(cmd, shell=shell, capture_output=True, text=True,
                           timeout=timeout, errors="replace")
        return r.returncode, (r.stdout or "") + (r.stderr or "")
    except subprocess.TimeoutExpired:
        return -1, "命令超时"
    except FileNotFoundError as e:
        return -2, "命令不存在: %s" % e


def item_ddr(ctx):
    """DDR 带宽：锁 dmc 最高频 → stress-ng memrate → 恢复 governor。"""
    if _run(["sudo", "-n", "true"], timeout=5)[0] != 0:
        return "PASS(WARN)", "sudo 免密不可用，跳过锁频（未测带宽）"
    if _run(["bash", "-lc", "command -v stress-ng"], timeout=5)[0] != 0:
        return "FAIL", "未安装 stress-ng（装好后按 r 重测）"

    dmc = "/sys/class/devfreq/dmc"

    def _read(path):
        try:
            with open(path) as f:
                return f.read().strip()
        except Exception:
            return ""

    orig_gov = _read(dmc + "/governor")
    avail = [int(x) for x in _read(dmc + "/available_frequencies").split()]
    max_freq = max(avail) if avail else 0
    if max_freq:
        _run(["sudo", "sh", "-c", "echo userspace > %s/governor" % dmc], timeout=10)
        _run(["sudo", "sh", "-c", "echo %d > %s/userspace/set_freq" % (max_freq, dmc)],
             timeout=10)
    freq = int(_read(dmc + "/cur_freq") or 0)
    theory = freq / 1e9 * 8 * 1000                      # MB/s
    ctx.info("DDR 频率: %.0f MHz | 理论带宽: %.0f MB/s" % (freq / 1e6, theory))

    rc, out = _run(["sudo", "stress-ng", "--memrate", "2", "-t", "%ds" % DDR_TEST_SECS,
                    "--metrics-brief"], timeout=DDR_TEST_SECS * 4 + 30)
    if orig_gov:
        _run(["sudo", "sh", "-c", "echo %s > %s/governor" % (orig_gov, dmc)], timeout=10)

    bw = parse_stress_ng(out)
    if not bw:
        tail = " / ".join([l.strip() for l in out.splitlines() if l.strip()][-3:])
        return "FAIL", "未能解析 stress-ng 输出（rc=%d）: %s" % (rc, tail[:120])
    verdict, note = verdict_ddr(max(bw.values()), theory)
    if bw.get("read"):
        note += " | 读 %.0f" % bw["read"]
    if bw.get("write"):
        note += " | 写 %.0f" % bw["write"]
    return verdict, note


def _emmc_mount_dir():
    """挑一个位于 eMMC（非 tmpfs）的可写目录。"""
    for path in ("/home/iflytek", "/userdata", "/"):
        if not os.path.isdir(path):
            continue
        rc, out = _run(["stat", "-f", "-c", "%T", path], timeout=5)
        if rc == 0 and out.strip() not in ("tmpfs", "ramfs"):
            return path
    return "/home/iflytek"


def item_emmc(ctx):
    if _run(["sudo", "-n", "true"], timeout=5)[0] == 0:
        _run(["sudo", "sh", "-c", "echo 3 > /proc/sys/vm/drop_caches"], timeout=10)
    dev = None
    for name in sorted(os.listdir("/sys/block")):
        if name.startswith("mmcblk") and "boot" not in name:
            dev = "/dev/" + name
            break
    if not dev:
        return "FAIL", "未找到 eMMC 设备"
    rp = dev
    for name in sorted(os.listdir("/sys/block")):
        if not name.startswith("mmcblk") or "boot" in name:
            continue
        for part in sorted(os.listdir("/sys/block/" + name)):
            if not part.startswith(name + "p"):
                continue
            try:
                with open("/sys/block/%s/%s/size" % (name, part)) as f:
                    size = int(f.read().strip())
            except Exception:
                size = 0
            if size > 10000000:
                rp = "/dev/" + part
                break
        if rp != dev:
            break
    ctx.info("设备: %s（读 %s）" % (dev, rp))

    rc, out = _run(["sudo", "dd", "if=%s" % rp, "of=/dev/null", "bs=1M",
                    "count=%d" % EMMC_READ_MB, "iflag=direct"], timeout=90)
    read_mbs = parse_dd_rate(out)
    ctx.info("顺序读: %.0f MB/s" % read_mbs)

    tf = os.path.join(_emmc_mount_dir(), ".prod_test_%d.tmp" % os.getpid())
    write_mbs = 0.0
    try:
        rc, out = _run(["dd", "if=/dev/zero", "of=%s" % tf, "bs=1M",
                        "count=%d" % EMMC_WRITE_MB, "oflag=direct", "conv=fdatasync"],
                       timeout=120)
        write_mbs = parse_dd_rate(out)
    finally:
        try:
            os.unlink(tf)
        except OSError:
            pass
    ctx.info("顺序写: %.0f MB/s" % write_mbs)
    return verdict_emmc(read_mbs, write_mbs)


def item_mic(ctx):
    if np is None:
        return "FAIL", "缺少 numpy，无法分析录音"
    fd, wav = tempfile.mkstemp(suffix=".wav")
    os.close(fd)
    try:
        ctx.info("录音 2 秒（可对麦克风说话）...")
        rc, out = _run(["arecord", "-D", "hw:0,0", "-f", "S16_LE", "-r", "16000",
                        "-c", "1", "-d", "2", wav], timeout=15)
        if rc != 0:
            ctx.info("hw:0,0 失败，回退 default")
            rc, out = _run(["arecord", "-D", "default", "-f", "S16_LE", "-r", "16000",
                            "-c", "1", "-d", "2", wav], timeout=15)
        if rc != 0:
            return "FAIL", "arecord 失败（检查麦克风/声卡）"
        with wave.open(wav, "rb") as wf:
            nch, sw, fr, nf = (wf.getnchannels(), wf.getsampwidth(),
                               wf.getframerate(), wf.getnframes())
            raw = wf.readframes(nf)
        if sw != 2:
            return "FAIL", "非 16bit 录音（%d bit）" % (sw * 8)
        samples = np.frombuffer(raw, dtype="<i2").astype(np.float64)
        if samples.size == 0:
            return "FAIL", "录音为空"
        rms = float(np.sqrt(np.mean(samples ** 2)))
        dc = float(np.mean(samples))
        ctx.info("采样率 %d Hz / 声道 %d / 位深 %d" % (fr, nch, sw * 8))
        return verdict_mic(rms, dc)
    finally:
        try:
            os.unlink(wav)
        except OSError:
            pass


def _nmcli_unescape(field):
    return field.replace("\\:", ":").replace("\\\\", "\\")


def item_wifi(ctx):
    """WiFi 扫描：只用 nmcli 看能否列出 AP（不连接、不测速，离线安全）。"""
    ctx.info("执行: nmcli dev wifi list --rescan yes（最多 30 秒）")
    rc, out = _run(["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY", "dev", "wifi",
                    "list", "--rescan", "yes"], timeout=30)
    if rc != 0:
        ctx.info("rescan 失败，退回不带 rescan 的列表")
        rc, out = _run(["nmcli", "-t", "-f", "SSID,SIGNAL,SECURITY", "dev", "wifi", "list"],
                       timeout=15)
    if rc != 0:
        return "FAIL", "nmcli 执行失败（NetworkManager 未运行或 WiFi 模块异常）"
    aps = []
    for line in out.splitlines():
        parts = line.split(":")
        ssid = _nmcli_unescape(parts[0]).strip() if parts else ""
        if ssid:
            sig = parts[1].strip() if len(parts) > 1 else "?"
            aps.append((ssid, sig))
    if not aps:
        return "FAIL", "未扫到任何 AP（检查 WiFi 模块/驱动/天线）"
    aps.sort(key=lambda x: -int(x[1]) if x[1].isdigit() else 0)
    top = ", ".join("%s(%s)" % (s, sig) for s, sig in aps[:3])
    ctx.info("扫到 %d 个 AP，信号最强 3 个: %s" % (len(aps), top))
    return "PASS", "扫到 %d 个 AP（最强: %s）" % (len(aps), aps[0][0])


# ══════════════════════════════════════
#  ROS 会话（按需启动 / 复用 / 退出只清自己的进程）
# ══════════════════════════════════════
class RosSession:
    """按需启动 bringup/peripherals；已在跑则复用；退出只清理自己启动的进程组。"""

    def __init__(self, term):
        self.t = term
        self.procs = {}
        self._nodes = set()

    @staticmethod
    def _proc(pattern):
        """pattern 是否有匹配进程（pgrep -f；不经过 shell，避免自匹配）。"""
        return _run(["pgrep", "-f", pattern], timeout=5)[0] == 0

    def _base_ready(self):
        return self._proc(PROC_BASE_DRIVER)

    def _periph_ready(self):
        return any(self._proc(p) for p in PROC_PERIPH)

    def _launch(self, name, pkg_launch):
        log = "/tmp/prod_test_%s.log" % name
        f = open(log, "ab")
        cmd = ["bash", "-lc", "%s && ros2 launch %s" % (ROS_SETUP, pkg_launch)]
        self.procs[name] = subprocess.Popen(cmd, stdout=f, stderr=subprocess.STDOUT,
                                            preexec_fn=os.setsid)
        self.t.w("  已启动 %s（日志 %s）" % (name, log), C_CYAN)

    def _wait_proc(self, check, timeout=30, interval=1.0):
        t0 = time.time()
        while time.time() - t0 < timeout:
            if check():
                return True
            time.sleep(interval)
        return False

    def ensure(self, need_periph=False):
        """保证 ROS 依赖就绪；返回 True/False。"""
        if self._base_ready():
            self.t.w("  复用已在运行的 car1_base_driver 进程", C_CYAN)
        else:
            if not os.path.exists(DEV_STM32):
                self.t.w("  未发现 %s（下位机不在线？）" % DEV_STM32, C_RED)
                return False
            self._launch("bringup", "car1_bringup bringup.launch.py")
            if not self._wait_proc(self._base_ready, 30):
                self.t.w("  bringup 30 秒内未就绪（base_driver 进程未出现）", C_RED)
                return False

        if need_periph:
            if self._periph_ready():
                self.t.w("  复用已在运行的 peripherals 进程", C_CYAN)
            else:
                self._launch("peripherals", "car1_peripherals peripherals.launch.py")
                if not self._wait_proc(self._periph_ready, 30, 2.0):
                    self.t.w("  peripherals 30 秒内未就绪（相机/雷达可能缺失）", C_YELLOW)
        return True

    def cleanup(self):
        import signal as _sig
        for name, proc in list(self.procs.items()):
            if proc.poll() is not None:
                continue
            try:
                os.killpg(os.getpgid(proc.pid), _sig.SIGTERM)
                proc.wait(timeout=3)
            except Exception:
                try:
                    os.killpg(os.getpgid(proc.pid), _sig.SIGKILL)
                except Exception:
                    pass
            self.t.w("  已回收 %s" % name, C_CYAN)
        self.procs.clear()


# ══════════════════════════════════════
#  一次性 rclpy 脚本（底盘 / 舵机 / RGB / 雷达 / 相机共用）
# ══════════════════════════════════════
_ROS_PUB_TMPL = '''
import sys, time, rclpy
from rclpy.node import Node
{imports}

def main():
    rclpy.init()
    n = Node("prod_test_pub")
{setup}
    for _ in range(40):
        rclpy.spin_once(n, timeout_sec=0.05)
        if {ready}: break
{actions}
    n.destroy_node(); rclpy.shutdown()

main()
'''


def _render_ros_script(imports, setup, ready, actions):
    """把多行 setup/actions 统一缩进 4 空格后填入模板（避免生成脚本缩进错乱）。"""
    def ind(block):
        return "\n".join(("    " + line) if line.strip() else line
                         for line in block.splitlines())
    return _ROS_PUB_TMPL.format(imports=imports, setup=ind(setup),
                                ready=ready, actions=ind(actions))


def _ros_run(script_body, timeout=60):
    """把一段 rclpy 脚本写到 /tmp 并以 ROS 环境执行；返回 (rc, 输出)。"""
    path = "/tmp/prod_test_ros_%d.py" % os.getpid()
    with open(path, "w") as f:
        f.write(script_body)
    return _run(["bash", "-lc", "%s && python3 %s" % (ROS_SETUP, path)], timeout=timeout)


def _echo(ctx, out):
    for line in out.splitlines():
        if line.strip():
            ctx.info(line.strip())


def item_chassis(ctx):
    """六个方向各 1 秒，随后停车。"""
    moves = ", ".join("(%.1f, %.1f, %.1f)" % (x, y, z) for _, x, y, z in CHASSIS_MOVES)
    labels = ", ".join(name for name, _, _, _ in CHASSIS_MOVES)
    body = _render_ros_script(
        imports="from geometry_msgs.msg import Twist",
        setup='pub = n.create_publisher(Twist, "%s", 10)' % TOPIC_CMD_VEL,
        ready="pub.get_subscription_count() > 0",
        actions=(
            "moves = [%s]\n"
            'labels = "%s".split(", ")\n'
            "for i, (x, y, z) in enumerate(moves):\n"
            '    print("  动作: %%s" %% labels[i], flush=True)\n'
            "    m = Twist(); m.linear.x = x; m.linear.y = y; m.angular.z = z\n"
            "    t_end = time.time() + %s\n"
            "    while time.time() < t_end:\n"
            "        pub.publish(m); rclpy.spin_once(n, timeout_sec=0.05)\n"
            "    stop = Twist(); pub.publish(stop)\n"
            "    rclpy.spin_once(n, timeout_sec=0.05)"
            % (moves, labels, CHASSIS_HOLD_S)))
    rc, out = _ros_run(body, timeout=len(CHASSIS_MOVES) * (CHASSIS_HOLD_S + 1) + 20)
    _echo(ctx, out)
    if rc != 0:
        return "FAIL", "底盘运动脚本失败"
    return None, "六个方向依次动作（前进/后退/左移/右移/左转/右转）"


def item_servo(ctx):
    """回中位 → 运动位置 → 夹爪开 → 夹爪关。"""
    body = _render_ros_script(
        imports="from car1_servo_controller_msgs.msg import ServosPosition, ServoPosition",
        setup='pub = n.create_publisher(ServosPosition, "%s", 10)' % TOPIC_SERVO,
        ready="pub.get_subscription_count() > 0",
        actions=(
            "def send(pose):\n"
            "    msg = ServosPosition(); msg.duration = 2.0; msg.position_unit = 'pulse'\n"
            "    for sid, pwm in pose:\n"
            "        p = ServoPosition(); p.id = sid; p.position = float(pwm)\n"
            "        msg.position.append(p)\n"
            "    pub.publish(msg)\n"
            "    t_end = time.time() + %s\n"
            "    while time.time() < t_end:\n"
            "        rclpy.spin_once(n, timeout_sec=0.05)\n"
            'print("  动作: 回中位", flush=True)\n'
            "send([(i, 1500) for i in range(6)])\n"
            'print("  动作: 运动位置", flush=True)\n'
            "send([(i, p) for i, p in enumerate(%s)])\n"
            'print("  动作: 夹爪开", flush=True)\n'
            "send([(5, %d)])\n"
            'print("  动作: 夹爪关", flush=True)\n'
            "send([(5, %d)])" % (SERVO_HOLD_S, SERVO_MOTION, SERVO_GRIPPER_OPEN,
                                    SERVO_GRIPPER_CLOSE)))
    rc, out = _ros_run(body, timeout=len(SERVO_CENTER) + 4 * SERVO_HOLD_S)
    _echo(ctx, out)
    if rc != 0:
        return "FAIL", "舵机脚本失败"
    return None, "回中位/运动位置/夹爪开/夹爪关 四个动作"


def item_rgb(ctx):
    """23 步 RGB 序列；索引 = 线上索引 + 1（ROS 侧 1 基）。"""
    seq = []
    for label, logicals, color, dwell in RGB_SEQUENCE:
        wire = [RGB_INDEX_MAP[i] + 1 for i in logicals]
        seq.append((label, wire, color, dwell))
    body = _render_ros_script(
        imports="from car1_base_driver_msgs.msg import RGBStates, RGBState",
        setup='pub = n.create_publisher(RGBStates, "%s", 10)' % TOPIC_RGB,
        ready="pub.get_subscription_count() > 0",
        actions=(
            "SEQ = %r\n"
            "def send(indices, color):\n"
            "    m = RGBStates()\n"
            "    for idx in indices:\n"
            "        s = RGBState(); s.index = idx\n"
            "        s.red, s.green, s.blue = color\n"
            "        m.states.append(s)\n"
            "    pub.publish(m); rclpy.spin_once(n, timeout_sec=0.05)\n"
            "for i, (label, idxs, color, dwell) in enumerate(SEQ, 1):\n"
            '    print("  %%2d/%%d %%s -> index %%s" %% (i, len(SEQ), label, idxs), flush=True)\n'
            "    send(idxs, color)\n"
            "    time.sleep(dwell)\n"
            "send(%r, (0, 0, 0))" % (seq, list(range(1, 17)))))
    rc, out = _ros_run(body, timeout=90)
    _echo(ctx, out)
    if rc != 0:
        return "FAIL", "RGB 脚本失败"
    return None, "23 步 RGB 序列（跑马灯→换色→闪烁→全灭）"


# ══════════════════════════════════════
#  终端缩略图（相机项用）
# ══════════════════════════════════════
def render_thumb(pairs, colors, width=48):
    """pairs: [((r,g,b)上, (r,g,b)下)] → 终端一行文本（真彩半块 / 灰阶字符）。"""
    chars = " .:-=+*#%@"
    out = []
    for top, bot in pairs:
        if colors:
            out.append("\033[38;2;%d;%d;%dm\033[48;2;%d;%d;%dm▀" % (top + bot))
        else:
            lum = (0.299 * top[0] + 0.587 * top[1] + 0.114 * top[2]
                   + 0.299 * bot[0] + 0.587 * bot[1] + 0.114 * bot[2]) / 2
            out.append(chars[min(len(chars) - 1, int(lum / 256 * len(chars)))])
    if colors:
        out.append("\033[0m")
    return "".join(out)


def _print_thumb(ctx, raw_path, meta_path, depth=False):
    """把采样到的一帧渲染成终端缩略图（两行像素合并成一个半块字符）。"""
    if Image is None:
        ctx.info("（无 PIL，跳过缩略图）")
        return
    with open(meta_path) as f:
        w, h, enc = f.read().split()
    w, h = int(w), int(h)
    with open(raw_path, "rb") as f:
        buf = f.read()
    if depth:
        vals = array.array("H")
        vals.frombytes(buf[: w * h * 2])
        img = Image.new("L", (w, h))
        px = img.load()
        for y in range(h):
            base = y * w
            for x in range(w):
                v = vals[base + x]
                v = 0 if v == 0 else min(5000, max(200, v))
                px[x, y] = int((v - 200) / 4800 * 255)
    else:
        target, raw_mode = PIL_MODE_MAP.get(enc, ("RGB", "RGB"))
        img = Image.frombuffer(target, (w, h), buf, "raw", raw_mode, 0, 1)
    cols = 48
    rows = max(6, int(cols * 0.5))
    small = img.convert("RGB").resize((cols, rows * 2))
    pix = small.load()
    ctx.out("  ┌" + "─" * cols + "┐")
    for y in range(rows):
        pairs = [(pix[x, 2 * y], pix[x, 2 * y + 1]) for x in range(cols)]
        ctx.out("  │" + render_thumb(pairs, ctx.t.color) + "│")
    ctx.out("  └" + "─" * cols + "┘")


# ══════════════════════════════════════
#  雷达 / 相机项
# ══════════════════════════════════════
def item_lidar(ctx):
    body = _render_ros_script(
        imports=("from sensor_msgs.msg import LaserScan\n"
                 "from rclpy.qos import qos_profile_sensor_data"),
        setup=('msgs = []\n'
               'sub = n.create_subscription(LaserScan, "%s", lambda m: msgs.append(m), '
               'qos_profile_sensor_data)') % TOPIC_SCAN,
        ready="True",
        actions=(
            "t0 = time.time()\n"
            "while not msgs and time.time() - t0 < 5.0:\n"
            "    rclpy.spin_once(n, timeout_sec=0.1)\n"
            "t1 = time.time()\n"
            "while time.time() - t1 < %.1f:\n"
            "    rclpy.spin_once(n, timeout_sec=0.1)\n"
            "n_msg = len(msgs); pts = 0; rmin = 0.0\n"
            "if msgs:\n"
            "    m = msgs[-1]\n"
            "    valid = [r for r in m.ranges if m.range_min < r < m.range_max]\n"
            "    pts = len(valid); rmin = min(valid) if valid else 0.0\n"
            '    print("  分辨率: %%d 点 / 角度 %%.2f~%%.2f" %% (len(m.ranges), m.angle_min, m.angle_max))\n'
            'print("  帧数: %%d 帧 (帧率 %%.1f Hz) / 有效点 %%d / 最近障碍 %%.2f m" '
            '%% (n_msg, n_msg / %.1f, pts, rmin))'
            % (SAMPLE_SECS, SAMPLE_SECS)))
    rc, out = _ros_run(body, timeout=int(SAMPLE_SECS) + 40)
    _echo(ctx, out)
    m = re.search(r"帧率 ([\d.]+) Hz\) / 有效点 (\d+)(?: / 最近障碍 ([\d.]+) m)?", out)
    if rc != 0 or not m:
        return "FAIL", "雷达采样失败（节点未起或设备缺失）"
    rmin = float(m.group(3)) if m.group(3) else 0.0
    return verdict_lidar(float(m.group(1)), int(m.group(2)), rmin)


def item_camera(ctx):
    body = _render_ros_script(
        imports=("from sensor_msgs.msg import Image\n"
                 "from rclpy.qos import qos_profile_sensor_data"),
        setup=('rgb_msgs = []; dep_msgs = []\n'
               'rgb_sub = n.create_subscription(Image, "%s", lambda m: rgb_msgs.append(m), '
               'qos_profile_sensor_data)\n'
               'dep_sub = n.create_subscription(Image, "%s", lambda m: dep_msgs.append(m), '
               'qos_profile_sensor_data)') % (TOPIC_CAM_RGB, TOPIC_CAM_DEPTH),
        ready="True",
        actions=(
            "import os as _os\n"
            "for _f in (\"/tmp/prod_test_rgb.raw\", \"/tmp/prod_test_rgb.meta\", "
            "\"/tmp/prod_test_dep.raw\", \"/tmp/prod_test_dep.meta\"):\n"
            "    try: _os.unlink(_f)\n"
            "    except OSError: pass\n"
            "t0 = time.time()\n"
            "while not rgb_msgs and time.time() - t0 < 5.0:\n"
            "    rclpy.spin_once(n, timeout_sec=0.1)\n"
            "t1 = time.time()\n"
            "while time.time() - t1 < %.1f:\n"
            "    rclpy.spin_once(n, timeout_sec=0.1)\n"
            'print("彩色帧: %%d 帧 (帧率 %%.1f Hz)" %% (len(rgb_msgs), len(rgb_msgs) / %.1f))\n'
            'print("深度帧: %%d 帧 (帧率 %%.1f Hz)" %% (len(dep_msgs), len(dep_msgs) / %.1f))\n'
            "if rgb_msgs:\n"
            "    m = rgb_msgs[-1]\n"
            '    print("彩色分辨率: %%dx%%d encoding=%%s" %% (m.width, m.height, m.encoding))\n'
            '    open("/tmp/prod_test_rgb.raw", "wb").write(bytes(m.data))\n'
            '    open("/tmp/prod_test_rgb.meta", "w").write("%%d %%d %%s" %% (m.width, m.height, m.encoding))\n'
            "if dep_msgs:\n"
            "    m = dep_msgs[-1]\n"
            '    print("深度分辨率: %%dx%%d encoding=%%s" %% (m.width, m.height, m.encoding))\n'
            '    open("/tmp/prod_test_dep.raw", "wb").write(bytes(m.data))\n'
            '    open("/tmp/prod_test_dep.meta", "w").write("%%d %%d %%s" %% (m.width, m.height, m.encoding))'
            % (SAMPLE_SECS, SAMPLE_SECS, SAMPLE_SECS)))
    rc, out = _ros_run(body, timeout=int(SAMPLE_SECS) + 45)
    _echo(ctx, out)
    if rc != 0:
        return "FAIL", "相机采样失败"
    m1 = re.search(r"彩色帧:\s*\d+ 帧 \(帧率 ([\d.]+) Hz\)", out)
    m2 = re.search(r"深度帧:\s*\d+ 帧 \(帧率 ([\d.]+) Hz\)", out)
    rgb_hz = float(m1.group(1)) if m1 else 0.0
    dep_hz = float(m2.group(1)) if m2 else 0.0
    verdict, note = verdict_cam(rgb_hz, dep_hz)
    if ctx.t.width >= 60 and os.path.exists("/tmp/prod_test_rgb.meta"):
        try:
            _print_thumb(ctx, "/tmp/prod_test_rgb.raw", "/tmp/prod_test_rgb.meta")
            if os.path.exists("/tmp/prod_test_dep.meta"):
                _print_thumb(ctx, "/tmp/prod_test_dep.raw", "/tmp/prod_test_dep.meta", depth=True)
        except Exception as e:
            ctx.info("缩略图渲染失败: %s" % e)
    else:
        ctx.info("（终端过窄或未采到彩色帧，跳过缩略图）")
    return verdict, note


# ══════════════════════════════════════
#  扬声器项（预置提示音 + 现场生成 440Hz 都播放）
# ══════════════════════════════════════
def item_speaker(ctx):
    played, failed = [], []
    if os.path.exists(KWS_WAV):
        ctx.info("① 播放预置提示音: %s" % os.path.basename(KWS_WAV))
        rc, out = _run(["aplay", "-D", "plughw:0,0", KWS_WAV], timeout=20)
        (played if rc == 0 else failed).append("kws.wav")
    else:
        ctx.info("① 预置提示音不存在，跳过这一半: %s" % KWS_WAV)

    fd, wav = tempfile.mkstemp(suffix=".wav")
    os.close(fd)
    try:
        rate, dur, freq = 16000, 1.0, 440
        n = int(rate * dur)
        pcm = bytearray()
        for i in range(n):
            v = int(16000 * 0.5 * math.sin(2 * math.pi * freq * i / rate))
            pcm += int(v).to_bytes(2, "little", signed=True)
        with wave.open(wav, "wb") as wf:
            wf.setnchannels(1)
            wf.setsampwidth(2)
            wf.setframerate(rate)
            wf.writeframes(bytes(pcm))
        ctx.info("② 播放 440Hz 测试音（1 秒）")
        rc, out = _run(["aplay", "-D", "plughw:0,0", wav], timeout=20)
        (played if rc == 0 else failed).append("440Hz")
    finally:
        try:
            os.unlink(wav)
        except OSError:
            pass

    note = "已播: %s" % (" + ".join(played) if played else "无")
    if failed:
        note += " | 失败: %s" % " + ".join(failed)
    if not played:
        return "FAIL", note + "（aplay 失败，检查声卡/接线）"
    if failed:
        return "PASS(WARN)", note
    return None, note


# ══════════════════════════════════════
#  项注册表 / Runner / 命令行入口
# ══════════════════════════════════════
class Ctx:
    """项执行上下文。"""

    def __init__(self, term, ros):
        self.t = term
        self.ros = ros

    def out(self, text="", color=None):
        self.t.w(text, color)

    def info(self, text):
        self.t.w("  " + text)


class Item:
    def __init__(self, iid, name, hint, run, needs_ros=False, need_periph=False, bare=False):
        self.iid = iid
        self.name = name
        self.hint = hint
        self.run = run
        self.needs_ros = needs_ros
        self.need_periph = need_periph
        self.bare = bare              # 裸板项：默认不测，--all 才一起测


ITEMS = [
    Item("ddr", "DDR 内存带宽", "锁频后跑 stress-ng memrate，约 15 秒", item_ddr, bare=True),
    Item("emmc", "eMMC 读写", "读 256MB / 写 128MB，约 20 秒", item_emmc, bare=True),
    Item("mic", "麦克风录音", "录音 2 秒（可对麦克风说话）", item_mic, bare=True),
    Item("wifi", "WiFi 扫描", "nmcli dev wifi list 看能否扫到 AP", item_wifi),
    Item("chassis", "底盘运动", "前进/后退/左移/右移/左转/右转 各 1 秒", item_chassis,
         needs_ros=True),
    Item("lidar", "激光雷达", "采样 1 秒：帧率 / 有效点数 / 最近障碍距离", item_lidar,
         needs_ros=True, need_periph=True),
    Item("speaker", "扬声器", "先播预置提示音，再播 440Hz 测试音", item_speaker),
    Item("camera", "摄像头", "采样 1 秒 + 终端缩略图", item_camera,
         needs_ros=True, need_periph=True),
    Item("servo", "机械臂舵机", "回中位/运动位置/夹爪开/夹爪关（每个姿态 3 秒）", item_servo, needs_ros=True),
    Item("rgb", "RGB 灯带", "23 步序列（跑马灯→换色→闪烁→全灭）", item_rgb, needs_ros=True),
]
ITEM_IDS = [it.iid for it in ITEMS]
BARE_IDS = [it.iid for it in ITEMS if it.bare]
DEFAULT_ITEMS = [it for it in ITEMS if not it.bare]      # 默认流程不含裸板项
DEFAULT_IDS = [it.iid for it in DEFAULT_ITEMS]

HELP_TEXT = """按键说明
  项前: ⏎ 执行 | 空格 跳过(计 FAIL) | b 上一项 | q 中止
  项后: ⏎ PASS | 空格 FAIL | r 重测本项 | b 上一项 | q 中止
  菜单: 1..9 单项测试 | A 全流程 | l 重列清单 | q 退出
  判定: PASS / PASS(WARN) / FAIL；退出码 0=无 FAIL, 1=有 FAIL, 2=中止"""


class Runner:
    def __init__(self, term, ros, pool=None):
        self.t = term
        self.ros = ros
        self.pool = list(pool) if pool else list(DEFAULT_ITEMS)
        self.ctx = Ctx(term, ros)
        self.rows = {}                 # iid -> (verdict, note, seconds)
        self.aborted = False

    def summary(self):
        rows = []
        for it in self.pool:
            if it.iid in self.rows:
                v, note, secs = self.rows[it.iid]
                rows.append((it.iid, it.name, v, note, secs))
        if not rows:
            return "（尚无结果）"
        return render_summary(rows, color=self.t.color, width=self.t.width)

    def exit_code(self):
        if any(v[0] == "FAIL" for v in self.rows.values()):
            return 1
        if self.aborted:
            return 2
        return 0

    def _prepare_ros(self, item):
        if not item.needs_ros:
            return True
        if not self.ros.ensure(item.need_periph):
            self.t.w("  ROS 依赖未就绪", C_RED)
            return False
        return True

    def _exec(self, item):
        """跑一次项，返回 (verdict|None, note, seconds)。"""
        self._prepare_ros(item)
        t0 = time.time()
        try:
            verdict, note = item.run(self.ctx)
        except Exception as e:
            verdict, note = "FAIL", "异常: %s" % e
        return verdict, note, time.time() - t0

    def run_one(self, item):
        """返回 True=已记录 / False=用户中止 / 'back'=退到上一项。"""
        self.t.hr()
        self.t.w("[%s] %s —— %s" % (item.iid, item.name, item.hint), C_BOLD)
        while True:
            k = self.t.key("  ⏎ 执行 | 空格 跳过(计 FAIL) | b 上一项 | q 中止 > ",
                           ("enter", "space", "b", "q"))
            if k == "q":
                self.aborted = True
                return False
            if k == "b":
                return "back"
            if k == "space":
                self.rows[item.iid] = ("FAIL", "跳过", 0.0)
                self.t.w("  已记为 FAIL（跳过）", C_YELLOW)
                return True

            verdict, note, secs = self._exec(item)
            if verdict is not None:
                self.rows[item.iid] = (verdict, note, secs)
                color = C_RED if verdict == "FAIL" else (
                    C_YELLOW if "WARN" in verdict else C_GREEN)
                self.t.w("  → %s  %s  (%.1fs)" % (verdict, note, secs), color)
                return True

            while True:                                  # 人工判定
                k = self.t.key("  判定: ⏎ PASS | 空格 FAIL | r 重测 | b 上一项 | "
                               "q 中止 | ? 帮助 > ",
                               ("enter", "space", "r", "b", "q", "?"))
                if k == "?":
                    self.t.w(HELP_TEXT)
                    continue
                if k == "enter":
                    self.rows[item.iid] = ("PASS", note or "-", secs)
                    self.t.w("  → PASS", C_GREEN)
                    return True
                if k == "space":
                    self.rows[item.iid] = ("FAIL", note or "-", secs)
                    self.t.w("  → FAIL", C_RED)
                    return True
                if k == "q":
                    self.aborted = True
                    return False
                if k == "b":
                    return "back"
                if k == "r":
                    verdict, note, secs = self._exec(item)
                    if verdict is not None:
                        self.rows[item.iid] = (verdict, note, secs)
                        self.t.w("  → %s  %s  (%.1fs)" % (verdict, note, secs), C_YELLOW)
                        return True

    def run_flow(self, items):
        i = 0
        while 0 <= i < len(items):
            r = self.run_one(items[i])
            if r is False:
                return
            if r == "back":
                i = max(0, i - 1)
                continue
            i += 1

    def menu(self):
        while True:
            self.t.hr()
            self.t.w("测试项清单（顺序即产测流程）", C_BOLD)
            for n, it in enumerate(self.pool, 1):
                mark = ""
                if it.iid in self.rows:
                    v = self.rows[it.iid][0]
                    mark = "  " + self.t.paint("[" + v + "]", verdict_color(v))
                self.t.w("  %s %s %s%s" % (_pad(n, 2), _pad(it.name, 16),
                                           _pad(it.hint, 40), mark))
            if not any(it.bare for it in self.pool):
                self.t.w("   (裸板项 DDR/eMMC/麦克风 默认不测；加 --all 一起测)")
            self.t.w("")
            k = self.t.key("菜单: 1..%d 单项 | A 全流程 | l 重列 | q 退出 > " % len(self.pool),
                           tuple(str(i) for i in range(1, len(self.pool) + 1)) + ("a", "l", "q"))
            if k is None or k == "q":
                return
            if k == "l":
                continue
            if k == "a":
                self.rows.clear()
                self.run_flow(self.pool)
                self.t.hr()
                self.t.w("全流程结果", C_BOLD)
                self.t.w(self.summary())
                continue
            self.run_flow([self.pool[int(k) - 1]])
            self.t.w(self.summary())


def main(argv=None):
    ap = argparse.ArgumentParser(description="RK3576 整机产测（纯终端版）")
    ap.add_argument("--only", help="只跑指定项，逗号分隔（如 ddr,emmc）")
    ap.add_argument("--all", action="store_true",
                    help="连裸板项(DDR/eMMC/麦克风)一起测；默认不测裸板项")
    ap.add_argument("--list", action="store_true", help="列出测试项后退出")
    args = ap.parse_args(argv)

    if args.list:
        pool = ITEMS if args.all else DEFAULT_ITEMS
        for n, it in enumerate(pool, 1):
            tag = " [裸板]" if it.bare else ""
            print("%d) %-10s %s —— %s%s" % (n, it.iid, it.name, it.hint, tag))
        if not args.all:
            print("（裸板项 %s 默认不测，加 --all 一起测）" % ", ".join(BARE_IDS))
        return 0

    term = Term()
    ids = None
    if args.only:                       # 参数校验放在 try 之前，避免多打一张空汇总
        try:
            ids = parse_only(args.only, ITEM_IDS)
        except ValueError as e:
            term.w(str(e), C_RED)
            return 2

    ros = RosSession(term)
    if ids:                              # 显式 --only：允许任何项（含裸板）
        runner = Runner(term, ros, [it for it in ITEMS if it.iid in ids])
    else:
        runner = Runner(term, ros, ITEMS if args.all else DEFAULT_ITEMS)

    def _sigint(signum, frame):
        runner.aborted = True
        raise KeyboardInterrupt

    signal.signal(signal.SIGINT, _sigint)
    try:
        term.enter_raw()
        term.clear()
        term.w("RK3576 整机产测（纯终端版）", C_BOLD)
        term.w("按键: ⏎ 执行/PASS | 空格 跳过或 FAIL | r 重测 | b 上一项 | q 中止 | ? 帮助",
               C_CYAN)
        if ids:
            term.w("只跑: %s" % ", ".join(ids))
            runner.run_flow([it for it in ITEMS if it.iid in ids])
        else:
            term.w("本次流程: %s" % ("全部项(含裸板)" if args.all
                                    else "整车功能项(不含裸板: %s)" % ", ".join(BARE_IDS)))
            runner.menu()
    except KeyboardInterrupt:
        runner.aborted = True
        term.w("\n已中止", C_YELLOW)
    finally:
        term.exit_raw()
        ros.cleanup()
        term.hr()
        term.w("产测汇总", C_BOLD)
        term.w(runner.summary())
        code = runner.exit_code()
        term.w("退出码: %d" % code, C_YELLOW)
    return code


if __name__ == "__main__":
    sys.exit(main())

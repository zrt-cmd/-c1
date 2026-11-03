#!/bin/bash

################################################################################
# RK3576 Ubuntu 裸板产测自检脚本 (hardware_check_bare.sh)
#
# 用途：裸板产测环境下一键检测上位机核心硬件（板上无任何外设）
# 日期：2026-09-21
#
# 相对 hardware_check.sh 的改动：
#   1. 新增 DDR 内存带宽检测（移植 hw_test_gui.py test_ddr：锁定 dmc 最高频 + stress-ng memrate）
#   2. eMMC 检测：禁止回退到 tmpfs（否则测到的是内存），自动定位位于 eMMC 上的可写目录，
#      速率统一归一化为 MB/s，判定阈值与 hw_test_gui.py test_emmc 一致
#   3. 删除 WiFi 模块检测与网口自测（裸板产测不做网络项）
#   4. USB 仅检测 USB 子系统与驱动健康度，不再查找 ttyUSB / video 具体设备；
#      dmesg 扫描排除 rockchip-usb2phy "IRQ index not found" 等良性探测噪声
#   5. 删除深度相机检测
#   6. 删除 ALSA 声卡 / 扬声器 / 录音设备枚举
#      → 改为麦克风录音信号检测（移植 hw_test_gui.py test_mic：录音 + RMS/峰值/直流分析）
#   7. 日志、结果文件名改为 hardware_check_bare_*，不覆盖原脚本产物
#   8. 测试末尾追加两个信息块：
#      a)【硬件配置】打印内存(DDR)容量、DDR 频率、eMMC 容量与型号（与标称值比较，±10% 内属正常）
#      b)【异常项目汇总】再次列出每一条失败/警告的序号、所属检测项、现象原因，便于拍照回传
#
# 运行：sudo bash hardware_check_bare.sh
# 可选依赖：stress-ng（DDR 带宽）、alsa-utils（arecord 麦克风）、python3（波形分析）
################################################################################

# 设置 UTF-8 编码
export LANG=zh_CN.UTF-8
export LC_ALL=zh_CN.UTF-8
export LANGUAGE=zh_CN:zh

# 配置项（与 hardware_check.sh 隔离，互不覆盖）
LOG_FILE="hardware_check_bare_$(date +%Y%m%d_%H%M%S).log"
RESULT_FILE="hardware_check_bare_result.txt"

# 标称配置：仅用于末尾"硬件配置"块打印容量与偏差提示（±10% 视为正常，不参与判定）
NOMINAL_MEM_GB=8
NOMINAL_EMMC_GB=64

# 颜色定义（串口终端兼容）
if [ -t 1 ]; then
    RED='\033[0;31m'
    GREEN='\033[0;32m'
    YELLOW='\033[1;33m'
    BLUE='\033[0;34m'
    NC='\033[0m' # No Color
else
    RED=''
    GREEN=''
    YELLOW=''
    BLUE=''
    NC=''
fi

# 测试结果统计
TOTAL_TESTS=0
PASSED_TESTS=0
FAILED_TESTS=0
WARNED_TESTS=0

# 异常项目（末尾"异常项目汇总"用；元素格式 "结果|模块|现象原因"）
ISSUES=()

################################################################################
# 辅助函数
################################################################################

# 日志函数
log() {
    echo -e "$1" | iconv -f UTF-8 -t UTF-8 2>/dev/null | tee -a "$LOG_FILE"
}

# 打印标题
print_header() {
    log "\n${BLUE}========================================${NC}"
    log "${BLUE}$1${NC}"
    log "${BLUE}========================================${NC}"
}

# 打印子标题
print_subheader() {
    log "\n${YELLOW}>>> $1${NC}"
}

# 由调用者函数名反推所属检测模块（供末尾异常汇总标注出处）
current_module() {
    case "${FUNCNAME[2]}" in
        test_system_info) echo "系统信息检测" ;;
        test_cpu)         echo "CPU 检测" ;;
        test_npu)         echo "NPU 检测" ;;
        test_gpu)         echo "GPU 检测" ;;
        test_memory)      echo "内存检测" ;;
        test_ddr)         echo "DDR 内存带宽检测" ;;
        test_emmc)        echo "eMMC 存储检测" ;;
        test_usb)         echo "USB 驱动检测" ;;
        test_audio_mic)   echo "音频检测（麦克风）" ;;
        *)                echo "${FUNCNAME[2]}" ;;
    esac
}

# 测试结果（PASS / WARN / FAIL；仅 FAIL 计入失败数，WARN/FAIL 记入异常汇总）
test_result() {
    TOTAL_TESTS=$((TOTAL_TESTS + 1))
    case "$1" in
        PASS)
            log "${GREEN}[PASS]${NC} $2"
            PASSED_TESTS=$((PASSED_TESTS + 1))
            echo "PASS: $2" >> "$RESULT_FILE"
            ;;
        WARN)
            log "${YELLOW}[WARN]${NC} $2"
            WARNED_TESTS=$((WARNED_TESTS + 1))
            echo "WARN: $2" >> "$RESULT_FILE"
            ISSUES+=("WARN|$(current_module)|$2")
            ;;
        *)
            log "${RED}[FAIL]${NC} $2"
            FAILED_TESTS=$((FAILED_TESTS + 1))
            echo "FAIL: $2" >> "$RESULT_FILE"
            ISSUES+=("FAIL|$(current_module)|$2")
            ;;
    esac
}

# 检查文件/目录是否存在
check_exists() {
    if [ -e "$1" ]; then
        return 0
    else
        return 1
    fi
}

# 安全读取 sysfs/procfs 值（失败返回空串，不污染日志）
sysval() {
    cat "$1" 2>/dev/null
}

# 把 dd 输出的速率统一折算成 MB/s（dd 可能给出 kB/MB/GB 三种单位）
dd_rate_mbps() {
    local rate val unit
    rate=$(echo "$1" | grep -oE '[0-9]+([.,][0-9]+)? [kMG]B/s' | tail -1)
    [ -n "$rate" ] || return 0
    val=$(echo "$rate" | awk '{print $1}' | tr ',' '.')
    unit=$(echo "$rate" | awk '{print $2}')
    case "$unit" in
        GB/s) awk -v v="$val" 'BEGIN{printf "%.0f", v * 1000}' ;;
        MB/s) awk -v v="$val" 'BEGIN{printf "%.0f", v}' ;;
        kB/s) awk -v v="$val" 'BEGIN{printf "%.0f", v / 1000}' ;;
    esac
}

# 容量偏差提示：$1=实测值(GB/GiB) $2=标称值；±10% 视为正常（进制换算/系统保留）
capacity_note() {
    local actual="$1" nominal="$2"
    [ -n "$actual" ] || return 0
    [ -n "$nominal" ] || return 0
    [ "$nominal" -gt 0 ] 2>/dev/null || return 0
    awk -v a="$actual" -v n="$nominal" 'BEGIN{
        pct = (a - n) / n * 100;
        if (pct < 0) pct = -pct;
        if (pct <= 10) printf "（标称 %dGB，偏差 %+.1f%%，属正常换算/保留差异）", n, (a - n) / n * 100;
        else printf "（标称 %dGB，偏差 %+.1f%%，偏差过大请核对物料！）", n, (a - n) / n * 100;
    }'
}

################################################################################
# 测试模块
################################################################################

# 1. 系统信息检测
test_system_info() {
    print_header "1. 系统信息检测"

    # 内核版本
    print_subheader "内核版本"
    KERNEL_VERSION=$(uname -r)
    log "内核版本: $KERNEL_VERSION"
    if [ -n "$KERNEL_VERSION" ]; then
        test_result "PASS" "内核版本: $KERNEL_VERSION"
    else
        test_result "FAIL" "无法获取内核版本"
    fi

    # 系统架构
    ARCH=$(uname -m)
    log "系统架构: $ARCH"

    # 系统型号
    print_subheader "系统型号"
    if check_exists "/proc/device-tree/model"; then
        MODEL=$(cat /proc/device-tree/model 2>/dev/null | tr -d '\0')
        log "设备型号: $MODEL"
        if echo "$MODEL" | grep -qi "rk3576"; then
            test_result "PASS" "设备型号识别: $MODEL"
        else
            test_result "FAIL" "设备型号不匹配: $MODEL"
        fi
    else
        test_result "FAIL" "无法读取设备型号"
    fi

    # 系统版本
    print_subheader "操作系统版本"
    if check_exists "/etc/os-release"; then
        . /etc/os-release
        log "系统: $NAME $VERSION"
        test_result "PASS" "操作系统: $NAME $VERSION"
    else
        test_result "FAIL" "无法读取系统版本"
    fi

    # 启动时间
    UPTIME=$(uptime -p 2>/dev/null || uptime)
    log "运行时间: $UPTIME"
}

# 2. CPU 检测
test_cpu() {
    print_header "2. CPU 检测"

    # CPU 型号（ARM 平台 /proc/cpuinfo 通常无 model name，回退设备树）
    print_subheader "CPU 型号"
    CPU_MODEL=$(grep -m1 "model name" /proc/cpuinfo | cut -d: -f2 | xargs)
    if [ -z "$CPU_MODEL" ]; then
        CPU_MODEL=$(grep -m1 "Processor" /proc/cpuinfo | cut -d: -f2 | xargs)
    fi
    if [ -z "$CPU_MODEL" ] && [ -f "/proc/device-tree/compatible" ]; then
        CPU_MODEL=$(tr -d '\0' < /proc/device-tree/compatible | tr ',' ' ' | awk '{print $1}')
    fi
    [ -z "$CPU_MODEL" ] && CPU_MODEL="未提供（ARM 平台 /proc/cpuinfo 无型号字段）"
    log "CPU 型号: $CPU_MODEL"

    # CPU 核心数
    CPU_CORES=$(grep -c ^processor /proc/cpuinfo)
    log "CPU 核心数: $CPU_CORES"
    if [ "$CPU_CORES" -ge 4 ]; then
        test_result "PASS" "CPU 核心数: $CPU_CORES"
    else
        test_result "FAIL" "CPU 核心数异常: $CPU_CORES"
    fi

    # CPU 频率
    print_subheader "CPU 频率信息"
    for cpu in /sys/devices/system/cpu/cpu[0-9]*; do
        cpu_num=$(basename "$cpu")
        if [ -f "$cpu/cpufreq/scaling_cur_freq" ]; then
            freq=$(cat "$cpu/cpufreq/scaling_cur_freq")
            freq_mhz=$((freq / 1000))
            log "$cpu_num: ${freq_mhz} MHz"
        fi
    done

    # 可用频率范围
    if [ -f "/sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies" ]; then
        avail_freqs=$(cat /sys/devices/system/cpu/cpu0/cpufreq/scaling_available_frequencies)
        log "可用频率: $avail_freqs"
        test_result "PASS" "CPU 频率信息正常"
    else
        test_result "FAIL" "无法读取 CPU 频率信息"
    fi

    # CPU 温度
    print_subheader "CPU 温度"
    TEMP_FOUND=0
    for zone in /sys/class/thermal/thermal_zone*; do
        if [ -f "$zone/type" ] && [ -f "$zone/temp" ]; then
            zone_type=$(cat "$zone/type")
            temp=$(cat "$zone/temp")
            temp_c=$((temp / 1000))
            log "$zone_type: ${temp_c}°C"
            TEMP_FOUND=1
        fi
    done
    if [ $TEMP_FOUND -eq 1 ]; then
        test_result "PASS" "CPU 温度监控正常"
    else
        test_result "FAIL" "无法读取 CPU 温度"
    fi
}

# 3. NPU 检测
test_npu() {
    print_header "3. NPU 检测"

    # NPU 驱动状态
    print_subheader "NPU 驱动状态"
    if dmesg | grep -qi "rknpu\|rknn"; then
        log "NPU 驱动信息:"
        dmesg | grep -i "rknpu\|rknn" | tail -5 | tee -a "$LOG_FILE"
        test_result "PASS" "NPU 驱动已加载"
    else
        test_result "FAIL" "未检测到 NPU 驱动"
    fi

    # NPU 频率信息
    print_subheader "NPU 频率信息"
    NPU_FREQ_FOUND=0
    for devfreq in /sys/class/devfreq/*npu* /sys/class/devfreq/fdab0000.npu; do
        if [ -d "$devfreq" ]; then
            if [ -f "$devfreq/cur_freq" ]; then
                cur_freq=$(cat "$devfreq/cur_freq")
                freq_mhz=$((cur_freq / 1000000))
                log "当前频率: ${freq_mhz} MHz"
                NPU_FREQ_FOUND=1
            fi
            if [ -f "$devfreq/available_frequencies" ]; then
                avail_freqs=$(cat "$devfreq/available_frequencies")
                log "可用频率: $avail_freqs"
            fi
        fi
    done
    if [ $NPU_FREQ_FOUND -eq 1 ]; then
        test_result "PASS" "NPU 频率信息正常"
    else
        test_result "FAIL" "无法读取 NPU 频率信息"
    fi

    # NPU 版本信息
    print_subheader "NPU 版本信息"
    if check_exists "/proc/device-tree/npu"; then
        for npu_node in /proc/device-tree/*npu*/compatible; do
            if [ -f "$npu_node" ]; then
                npu_compat=$(cat "$npu_node" | tr -d '\0')
                log "NPU 兼容性: $npu_compat"
                test_result "PASS" "NPU 设备树信息: $npu_compat"
            fi
        done
    fi

    # 检查 RKNN 相关进程
    print_subheader "RKNN 进程状态"
    if ps aux | grep -v grep | grep -q "rknn"; then
        log "RKNN 相关进程:"
        ps aux | grep -v grep | grep "rknn" | tee -a "$LOG_FILE"
    else
        log "未发现 RKNN 相关进程"
    fi
}

# 4. GPU 检测
test_gpu() {
    print_header "4. GPU 检测"

    # Mali GPU 设备
    print_subheader "GPU 设备节点"
    GPU_DEV_FOUND=0
    for dev in /dev/mali* /dev/dri/card* /dev/dri/renderD*; do
        if [ -e "$dev" ]; then
            log "发现 GPU 设备: $dev"
            GPU_DEV_FOUND=1
        fi
    done
    if [ $GPU_DEV_FOUND -eq 1 ]; then
        test_result "PASS" "GPU 设备节点存在"
    else
        test_result "FAIL" "未找到 GPU 设备节点"
    fi

    # GPU 驱动状态
    print_subheader "GPU 驱动状态"
    if dmesg | grep -qi "mali\|gpu"; then
        log "GPU 驱动信息:"
        dmesg | grep -i "mali\|gpu" | grep -v "vpu" | tail -5 | tee -a "$LOG_FILE"
        test_result "PASS" "GPU 驱动已加载"
    else
        test_result "FAIL" "未检测到 GPU 驱动"
    fi

    # GPU 频率信息
    print_subheader "GPU 频率信息"
    GPU_FREQ_FOUND=0
    for devfreq in /sys/class/devfreq/*gpu* /sys/class/devfreq/fb000000.gpu; do
        if [ -d "$devfreq" ]; then
            if [ -f "$devfreq/cur_freq" ]; then
                cur_freq=$(cat "$devfreq/cur_freq")
                freq_mhz=$((cur_freq / 1000000))
                log "当前频率: ${freq_mhz} MHz"
                GPU_FREQ_FOUND=1
            fi
            if [ -f "$devfreq/available_frequencies" ]; then
                avail_freqs=$(cat "$devfreq/available_frequencies")
                log "可用频率: $avail_freqs"
            fi
        fi
    done
    if [ $GPU_FREQ_FOUND -eq 1 ]; then
        test_result "PASS" "GPU 频率信息正常"
    else
        test_result "FAIL" "无法读取 GPU 频率信息"
    fi
}

# 5. 内存检测
test_memory() {
    print_header "5. 内存检测"

    print_subheader "内存容量"
    if check_exists "/proc/meminfo"; then
        MEM_TOTAL=$(grep MemTotal /proc/meminfo | awk '{print $2}')
        MEM_FREE=$(grep MemAvailable /proc/meminfo | awk '{print $2}')
        MEM_TOTAL_GB=$(echo "scale=2; $MEM_TOTAL / 1024 / 1024" | bc)
        MEM_FREE_GB=$(echo "scale=2; $MEM_FREE / 1024 / 1024" | bc)
        MEM_USED_GB=$(echo "scale=2; ($MEM_TOTAL - $MEM_FREE) / 1024 / 1024" | bc)

        log "总内存: ${MEM_TOTAL_GB} GB"
        log "已用内存: ${MEM_USED_GB} GB"
        log "可用内存: ${MEM_FREE_GB} GB"

        # 检查内存是否符合预期（至少 2GB）
        MEM_TOTAL_MB=$((MEM_TOTAL / 1024))
        if [ "$MEM_TOTAL_MB" -ge 2048 ]; then
            test_result "PASS" "内存容量: ${MEM_TOTAL_GB} GB"
        else
            test_result "FAIL" "内存容量不足: ${MEM_TOTAL_GB} GB"
        fi
    else
        test_result "FAIL" "无法读取内存信息"
    fi

    # 内存详细信息
    print_subheader "内存详细信息"
    free -h | tee -a "$LOG_FILE"
}

# 6. DDR 内存带宽检测（移植 hw_test_gui.py test_ddr）
test_ddr() {
    print_header "6. DDR 内存带宽检测"

    DMC="/sys/class/devfreq/dmc"
    if [ ! -d "$DMC" ]; then
        test_result "FAIL" "未找到 DDR devfreq 节点 ($DMC)"
        return
    fi

    ORIG_GOV=$(sysval "$DMC/governor")
    CUR_FREQ=$(sysval "$DMC/cur_freq")
    CUR_FREQ=${CUR_FREQ:-0}
    AVAIL_FREQS=$(sysval "$DMC/available_frequencies")

    MAX_FREQ=0
    for f in $AVAIL_FREQS; do
        [ "$f" -gt "$MAX_FREQ" ] 2>/dev/null && MAX_FREQ=$f
    done

    print_subheader "DDR 频率信息"
    log "调度器: $ORIG_GOV"
    log "当前频率: $((CUR_FREQ / 1000000)) MHz"
    log "可用频率: $AVAIL_FREQS"

    # 锁定到最高频率（压测需要；需要 root 写 sysfs）
    if [ "$MAX_FREQ" -gt 0 ] && [ "$CUR_FREQ" -ne "$MAX_FREQ" ]; then
        if [ "$(id -u)" -eq 0 ]; then
            # 注意顺序：先切 governor，userspace 目录才会出现
            echo userspace > "$DMC/governor" 2>/dev/null
            echo "$MAX_FREQ" > "$DMC/userspace/set_freq" 2>/dev/null
            CUR_FREQ=$(sysval "$DMC/cur_freq")
            CUR_FREQ=${CUR_FREQ:-0}
            log "已锁定 DDR 到最高频率: $((MAX_FREQ / 1000000)) MHz"
        else
            log "${YELLOW}非 root，无法锁定 DDR 最高频率，带宽结果可能偏低${NC}"
        fi
    fi

    DMC_FREQ=$(sysval "$DMC/cur_freq")
    DMC_FREQ=${DMC_FREQ:-0}
    THEORY=$(awk -v f="$DMC_FREQ" 'BEGIN{printf "%.1f", f / 1e9 * 8}')
    THEORY_MB=$(awk -v f="$DMC_FREQ" 'BEGIN{printf "%.0f", f / 1e9 * 8 * 1000}')
    log "频率: $((DMC_FREQ / 1000000)) MHz | 数据率: $((DMC_FREQ / 1000000 * 2)) MT/s | 理论带宽: ${THEORY} GB/s"

    # 压测（dmesg/thermal 中的 ddr-thermal 同步记录）
    print_subheader "DDR 带宽压测 (stress-ng --memrate 2 -t 10s)"
    if ! command -v stress-ng >/dev/null 2>&1; then
        test_result "WARN" "未安装 stress-ng，跳过 DDR 带宽压测（apt install stress-ng）"
    else
        SNG_OUT=$(stress-ng --memrate 2 -t 10s --metrics-brief 2>&1)

        # 恢复原始调度器
        if [ -n "$ORIG_GOV" ] && [ "$ORIG_GOV" != "userspace" ]; then
            echo "$ORIG_GOV" > "$DMC/governor" 2>/dev/null
        fi

        # 解析汇总行: "memrate   9161.00 write128 MB/sec (average per stressor)"
        PARSED=$(mktemp)
        echo "$SNG_OUT" | sed -nE 's/.*memrate +([0-9.]+) (read|write|read64pf)([0-9]*) MB\/sec.*/\1 \2 \3/p' \
            | awk '{print $2, ($3 == "" ? 0 : $3), $1}' > "$PARSED"

        if [ ! -s "$PARSED" ]; then
            test_result "FAIL" "未获取到 DDR memrate 数据"
            log "stress-ng 输出:"
            echo "$SNG_OUT" | tail -10 | tee -a "$LOG_FILE"
        else
            awk -v tm="$THEORY_MB" '{ blk = ($2 == "" ? 0 : $2 + 0); v = $3 + 0;
                 printf "  %-9s %6s B %8.0f MB/s %6.1f%%\n", $1, (blk ? blk : "-"), v, (tm ? 100 * v / tm : 0) }' \
                "$PARSED" | tee -a "$LOG_FILE"

            BW=$(awk '$1 == "write" && $2 + 0 >= 64 && $3 + 0 > m { m = $3 + 0 } END { printf "%.0f", m }' "$PARSED")
            BR=$(awk '$1 == "read"  && $2 + 0 >= 64 && $3 + 0 > m { m = $3 + 0 } END { printf "%.0f", m }' "$PARSED")
            log "最佳写入: ${BW:-0} MB/s | 最佳读取: ${BR:-0} MB/s (理论 $THEORY_MB MB/s)"

            BW_PCT=$(awk -v a="${BW:-0}" -v b="${THEORY_MB:-1}" 'BEGIN{printf "%.0f", (b ? a / b * 100 : 0)}')
            # 阈值说明：hw_test_gui.py 取理论值的 50% 作硬门限，但 memrate 报的是压测末段瞬时值，
            # 本板 8 次采样落在 48%~60%（均值约 54%），50% 会让 PASS/WARN 反复抖动。
            # 故判失败下限取 40%（真实 DDR 故障是数倍下降），50% 仅作为日志参考。
            if [ "$BW_PCT" -ge 50 ]; then
                test_result "PASS" "DDR 带宽正常（最佳写入 ${BW} MB/s = 理论带宽 ${BW_PCT}%）"
            elif [ "$BW_PCT" -ge 40 ]; then
                test_result "PASS" "DDR 带宽可用（最佳写入 ${BW} MB/s = 理论带宽 ${BW_PCT}%，低于 GUI 参考值 50%，属测量抖动区间）"
            else
                test_result "FAIL" "DDR 带宽异常（最佳写入 ${BW} MB/s = 理论带宽 ${BW_PCT}%，低于下限 40%）"
            fi
        fi
        rm -f "$PARSED"
    fi

    # DDR 频率锁定结果
    if [ "$MAX_FREQ" -gt 0 ] && [ "$DMC_FREQ" -eq "$MAX_FREQ" ]; then
        test_result "PASS" "DDR 压测频率锁定最高 $((MAX_FREQ / 1000000)) MHz"
    else
        test_result "WARN" "DDR 未运行在最高频率 ($((DMC_FREQ / 1000000))/$((MAX_FREQ / 1000000)) MHz)，压测值可能偏低"
    fi

    # DDR 温度
    for zone in /sys/class/thermal/thermal_zone*; do
        if [ "$(sysval "$zone/type")" = "ddr-thermal" ]; then
            log "DDR 温度: $(( $(sysval "$zone/temp") / 1000 ))°C"
        fi
    done
}

# 7. eMMC 存储检测
test_emmc() {
    print_header "7. eMMC 存储检测"

    # 识别 eMMC 设备
    print_subheader "eMMC 设备识别"
    EMMC_DEV=""
    for dev in /dev/mmcblk[0-9]; do
        if [ -b "$dev" ]; then
            # 检查是否为 eMMC（通常 mmcblk0）
            dev_name=$(basename "$dev")
            if [ -f "/sys/block/$dev_name/device/type" ]; then
                dev_type=$(cat "/sys/block/$dev_name/device/type")
                log "发现存储设备: $dev (类型: $dev_type)"
                EMMC_DEV="$dev"
                break
            fi
        fi
    done

    if [ -z "$EMMC_DEV" ]; then
        test_result "FAIL" "未找到 eMMC 设备"
        return
    fi

    test_result "PASS" "eMMC 设备: $EMMC_DEV"
    EMMC_NAME=$(basename "$EMMC_DEV")

    # eMMC 容量
    print_subheader "eMMC 容量信息"
    lsblk "$EMMC_DEV" -o NAME,SIZE,TYPE,MOUNTPOINT | tee -a "$LOG_FILE"

    # eMMC 设备信息
    if [ -d "/sys/block/$EMMC_NAME/device" ]; then
        log "\neMMC 设备详情:"
        [ -f "/sys/block/$EMMC_NAME/device/name" ] && log "  名称: $(cat /sys/block/$EMMC_NAME/device/name)"
        [ -f "/sys/block/$EMMC_NAME/device/manfid" ] && log "  制造商ID: $(cat /sys/block/$EMMC_NAME/device/manfid)"
        [ -f "/sys/block/$EMMC_NAME/device/oemid" ] && log "  OEM ID: $(cat /sys/block/$EMMC_NAME/device/oemid)"
    fi

    # ── 选择位于 eMMC 上的可写目录 ──
    # 禁止使用 /tmp（RK3576 上多为 tmpfs，测出来的是内存带宽而非 eMMC）
    print_subheader "测试路径选择"
    TEST_DIR=""
    local cand majmin real_dev SEEN=" "
    for cand in "$HOME" /home/iflytek /userdata /data /root /; do
        [ -n "$cand" ] || continue
        case "$SEEN" in *" $cand "*) continue ;; esac
        SEEN="$SEEN$cand "
        [ -d "$cand" ] && [ -w "$cand" ] || continue
        # findmnt 输出带对齐空格，必须去掉
        majmin=$(findmnt -no MAJ:MIN -T "$cand" 2>/dev/null | tr -d '[:space:]')
        if [ -z "$majmin" ] || [ ! -e "/sys/dev/block/$majmin" ]; then
            log "跳过 $cand（无法定位块设备，通常是 tmpfs/内存盘）"
            continue
        fi
        real_dev=$(readlink -f "/sys/dev/block/$majmin")
        case "$real_dev" in
            *"/$EMMC_NAME"|*"/$EMMC_NAME/"*)
                TEST_DIR="$cand"
                break
                ;;
            *)
                log "跳过 $cand（块设备 $(basename "$real_dev") 不属于 $EMMC_NAME）"
                ;;
        esac
    done

    if [ -z "$TEST_DIR" ]; then
        test_result "FAIL" "未找到位于 $EMMC_NAME 上的可写目录，无法测试 eMMC 真实速率（禁止回退到 tmpfs）"
        return
    fi

    TEST_FILE="$TEST_DIR/.emmc_speedtest_$$"
    log "测试路径: $TEST_FILE"
    log "挂载信息: $(findmnt -no SOURCE,FSTYPE,OPTIONS -T "$TEST_DIR" 2>/dev/null)"

    WRITE_SPEED=""
    READ_SPEED=""
    WRITE_MODE="direct"
    READ_MODE="direct"
    local dd_out

    # 写入测试（512MB，direct 绕过页缓存获取真实速率）
    print_subheader "eMMC 速率测试（512MB）"
    log "执行写入测试..."
    dd_out=$(dd if=/dev/zero of="$TEST_FILE" bs=1M count=512 oflag=direct conv=fsync 2>&1)
    WRITE_SPEED=$(dd_rate_mbps "$dd_out")
    # 部分 dd 版本不支持 oflag=direct，降级重试（此时数值含页缓存，仅供参考）
    if [ -z "$WRITE_SPEED" ]; then
        dd_out=$(dd if=/dev/zero of="$TEST_FILE" bs=1M count=512 conv=fsync 2>&1)
        WRITE_SPEED=$(dd_rate_mbps "$dd_out")
        WRITE_MODE="cached"
    fi
    if [ -n "$WRITE_SPEED" ]; then
        log "写入速度: ${WRITE_SPEED} MB/s (${WRITE_MODE})"
    else
        log "写入速度: 测试失败"
        log "dd 输出: $dd_out"
    fi

    # 清空页缓存（若有权限），确保读测试反映真实硬件速率
    sync
    [ -w /proc/sys/vm/drop_caches ] && echo 3 > /proc/sys/vm/drop_caches 2>/dev/null

    # 读取测试
    log "执行读取测试..."
    if [ -f "$TEST_FILE" ]; then
        dd_out=$(dd if="$TEST_FILE" of=/dev/null bs=1M count=512 iflag=direct 2>&1)
        READ_SPEED=$(dd_rate_mbps "$dd_out")
        if [ -z "$READ_SPEED" ]; then
            dd_out=$(dd if="$TEST_FILE" of=/dev/null bs=1M count=512 2>&1)
            READ_SPEED=$(dd_rate_mbps "$dd_out")
            READ_MODE="cached"
        fi
        if [ -n "$READ_SPEED" ]; then
            log "读取速度: ${READ_SPEED} MB/s (${READ_MODE})"
        else
            log "读取速度: 测试失败"
            log "dd 输出: $dd_out"
        fi
        rm -f "$TEST_FILE"
    fi

    # 速率判定（阈值与 hw_test_gui.py test_emmc 一致）
    if [ -z "$WRITE_SPEED" ] || [ -z "$READ_SPEED" ]; then
        test_result "FAIL" "eMMC 读写速率测试失败"
    elif [ "$READ_SPEED" -ge 250 ]; then
        test_result "PASS" "eMMC 速度优秀（读 ${READ_SPEED} MB/s, 写 ${WRITE_SPEED} MB/s）"
    elif [ "$READ_SPEED" -ge 150 ] && [ "$WRITE_SPEED" -ge 100 ]; then
        test_result "PASS" "eMMC 速度正常（读 ${READ_SPEED} MB/s, 写 ${WRITE_SPEED} MB/s）"
    elif [ "$READ_SPEED" -ge 100 ]; then
        test_result "WARN" "eMMC 速度偏低（读 ${READ_SPEED} MB/s, 写 ${WRITE_SPEED} MB/s）"
    else
        test_result "FAIL" "eMMC 速度异常（读 ${READ_SPEED} MB/s, 写 ${WRITE_SPEED} MB/s）"
    fi
}

################################################################################
# 8. USB 驱动检测（只验证驱动，不校验外接设备）
################################################################################

test_usb() {
    print_header "8. USB 驱动检测（仅驱动/子系统，不查找外设）"

    # USB 子系统挂载
    print_subheader "USB 子系统"
    if [ -d "/sys/bus/usb/devices" ] && [ -d "/sys/bus/usb/drivers" ]; then
        log "sysfs 总线: /sys/bus/usb 已挂载"
        if [ -d "/dev/bus/usb" ]; then
            log "usbfs 节点: /dev/bus/usb 存在"
        else
            log "${YELLOW}usbfs 节点 /dev/bus/usb 不存在${NC}"
        fi
        test_result "PASS" "USB 子系统已挂载"
    else
        test_result "FAIL" "USB 子系统未挂载（/sys/bus/usb 缺失）"
        return
    fi

    # 核心驱动加载（usbcore 为内核内置模块，始终出现在 /sys/module）
    print_subheader "核心驱动加载状态"
    if [ -d "/sys/module/usbcore" ]; then
        log "usbcore: 已加载"
        test_result "PASS" "USB 核心驱动 usbcore 已加载"
    else
        test_result "FAIL" "USB 核心驱动 usbcore 未加载（内核配置异常）"
    fi

    # 主机控制器驱动绑定 + 根 Hub 枚举
    print_subheader "主机控制器（HCD）驱动"
    local hcd_bound=0 drv bound
    for drv in /sys/bus/platform/drivers/*xhci* /sys/bus/platform/drivers/*dwc3* \
               /sys/bus/platform/drivers/*ehci* /sys/bus/platform/drivers/*ohci*; do
        [ -d "$drv" ] || continue
        bound=$(ls -1 "$drv" 2>/dev/null | grep -vcE '^(bind|unbind|uevent|module|new_id|remove_id)$')
        log "驱动 $(basename "$drv"): 绑定设备 $bound 个"
        [ "$bound" -gt 0 ] && hcd_bound=1
    done

    ROOT_HUBS=$(ls -d /sys/bus/usb/devices/usb[0-9]* 2>/dev/null | wc -l)
    log "根 Hub 数量: $ROOT_HUBS"
    if [ "$hcd_bound" -eq 1 ] && [ "$ROOT_HUBS" -ge 1 ]; then
        test_result "PASS" "主机控制器驱动已绑定，根 Hub 枚举正常 ($ROOT_HUBS 个)"
    else
        test_result "FAIL" "主机控制器驱动未绑定或无根 Hub (驱动绑定=$hcd_bound, 根Hub=$ROOT_HUBS)"
    fi

    # 主机控制器内核日志
    if dmesg | grep -qiE "dwc3|xhci"; then
        log "主机控制器内核日志:"
        dmesg | grep -iE "dwc3|xhci" | tail -5 | tee -a "$LOG_FILE"
    fi

    # 类驱动注册
    print_subheader "USB 类驱动注册状态"
    local missing=() class_drv
    for class_drv in hub usbhid usb-storage uas; do
        if [ -d "/sys/bus/usb/drivers/$class_drv" ]; then
            log "已注册: $class_drv"
        else
            log "${YELLOW}未注册: $class_drv${NC}"
            missing+=("$class_drv")
        fi
    done
    if [ "${#missing[@]}" -eq 0 ]; then
        test_result "PASS" "USB 类驱动注册完整 (hub/usbhid/usb-storage/uas)"
    else
        test_result "WARN" "USB 类驱动未注册: ${missing[*]}（可能按需加载，裸板可忽略）"
    fi

    # dmesg 错误扫描（排除 rockchip-usb2phy "IRQ index not found"、dwc3 "device reset" 等良性日志）
    print_subheader "USB 内核错误扫描"
    usb_err=$(dmesg | grep -iE '(usb|xhci|dwc3)' \
        | grep -iE '(error|fail|timeout|not responding|unable|over-?current|reset failed)' \
        | grep -viE 'usb2phy|usb2-phy|inno-usb2|inno_usb2|IRQ index|device reset|EM: OPP|usb_phy|rockchip-usb|not available' \
        | tail -10)
    if [ -z "$usb_err" ]; then
        test_result "PASS" "未发现 USB 内核错误"
    else
        log "发现的错误信息:"
        echo "$usb_err" | tee -a "$LOG_FILE"
        test_result "FAIL" "USB 内核报错 $(echo "$usb_err" | wc -l) 条（检查 USB 口是否存在短路/虚焊）"
    fi

    # 根 Hub 列表（仅作证据，不校验外设）
    print_subheader "USB 控制器列表"
    if command -v lsusb >/dev/null 2>&1; then
        lsusb | tee -a "$LOG_FILE"
    else
        for dev in /sys/bus/usb/devices/usb[0-9]*; do
            [ -f "$dev/product" ] && log "  - $(cat "$dev/product")"
        done
    fi
}

################################################################################
# 9. 音频检测（麦克风录音信号，移植 hw_test_gui.py test_mic）
################################################################################

test_audio_mic() {
    print_header "9. 音频检测（麦克风录音信号）"

    if ! command -v arecord >/dev/null 2>&1; then
        test_result "FAIL" "arecord 不可用（alsa-utils 未安装）"
        return
    fi

    local WAV="/tmp/hw_mic_$$.wav"
    local rec_ok=0 dev="hw:0,0"
    print_subheader "录音 2 秒 (16000Hz / 单声道 / S16_LE)"
    log "开始录音，请保持环境安静或对着麦克风发声..."

    if arecord -D "$dev" -f S16_LE -r 16000 -c 1 -d 2 "$WAV" >/dev/null 2>&1; then
        rec_ok=1
    else
        log "${YELLOW}hw:0,0 录音失败，回退 default 设备重试...${NC}"
        if arecord -D default -f S16_LE -r 16000 -c 1 -d 2 "$WAV" >/dev/null 2>&1; then
            rec_ok=1
            dev="default"
        fi
    fi

    if [ "$rec_ok" -ne 1 ]; then
        log "音频设备诊断信息:"
        if [ -f /proc/asound/cards ]; then
            cat /proc/asound/cards | tee -a "$LOG_FILE"
        else
            log "  /proc/asound/cards 不存在"
        fi
        arecord -l 2>&1 | tee -a "$LOG_FILE"
        test_result "FAIL" "麦克风录音失败（无音频设备或设备被占用）"
        rm -f "$WAV"
        return
    fi

    # 文件大小检查（与 hw_test_gui.py 一致：<1000 字节视为未接麦克风）
    local fsize
    fsize=$(stat -c %s "$WAV" 2>/dev/null || echo 0)
    log "录音文件: $WAV ($fsize 字节)"
    if [ "$fsize" -lt 1000 ]; then
        test_result "FAIL" "录音文件过小 ($fsize 字节)，麦克风可能未连接"
        rm -f "$WAV"
        return
    fi

    if ! command -v python3 >/dev/null 2>&1; then
        test_result "WARN" "录音成功 ($fsize 字节) 但无 python3，无法分析信号强度"
        rm -f "$WAV"
        return
    fi

    # 波形分析：采样率/声道/位深 + RMS/峰值/直流（等价 hw_test_gui.py 的 numpy 计算）
    local stats
    stats=$(python3 - "$WAV" <<'PYEOF'
import math, struct, sys, wave

with wave.open(sys.argv[1], "rb") as wf:
    nch, sw, fr, nf = wf.getnchannels(), wf.getsampwidth(), wf.getframerate(), wf.getnframes()
    raw = wf.readframes(nf)

if not raw:
    print("0 0 0 0 0 0")
    raise SystemExit

fmt = "<%dh" % (len(raw) // 2) if sw == 2 else "<%db" % len(raw)
s = struct.unpack(fmt, raw)
rms = math.sqrt(sum(float(v) * v for v in s) / len(s))
peak = max(abs(v) for v in s)
dc = sum(s) / len(s)
print("%d %d %d %.1f %.0f %.1f" % (fr, nch, sw * 8, rms, peak, dc))
PYEOF
)
    rm -f "$WAV"

    local rec_rate rec_ch rec_bits rms peak dc
    read -r rec_rate rec_ch rec_bits rms peak dc <<<"$stats"
    log "采样率: ${rec_rate}Hz  声道: ${rec_ch}ch  位深: ${rec_bits}bit  录音设备: $dev"
    log "RMS: $rms  峰值: $peak  直流: $dc"

    if [ -z "$rms" ] || [ "$rec_rate" = "0" ]; then
        test_result "FAIL" "录音波形解析失败"
    elif awk "BEGIN{exit !($rms < 50)}"; then
        # 裸板上通常未装麦克风板，能录到文件即证明 I2S/Codec 通路正常，故降级为 WARN
        test_result "WARN" "麦克风无信号 (RMS=$rms，裸板未装麦克风时为预期，整机需复检)"
    elif awk "BEGIN{exit !($dc < -500 || $dc > 500)}"; then
        test_result "WARN" "麦克风直流偏置偏高 (DC=$dc)"
    else
        test_result "PASS" "麦克风信号正常 (RMS=$rms, 峰值=$peak)"
    fi
}

################################################################################
# 硬件配置信息（末尾打印容量/频率，便于产线核对物料与拍照记录）
################################################################################

print_hw_config() {
    log ""
    log "${BLUE}========================================${NC}"
    log "${BLUE}       硬件配置（请拍照记录）${NC}"
    log "${BLUE}========================================${NC}"

    # 内存 / DDR 容量
    local mem_kb mem_gib
    mem_kb=$(grep -m1 MemTotal /proc/meminfo 2>/dev/null | awk '{print $2}')
    if [ -n "$mem_kb" ]; then
        mem_gib=$(awk -v k="$mem_kb" 'BEGIN{printf "%.2f", k / 1024 / 1024}')
        log "内存(DDR): ${mem_gib} GiB $(capacity_note "$mem_gib" "$NOMINAL_MEM_GB")"
    else
        log "内存(DDR): 读取失败"
    fi

    # DDR 频率（RK3576 的 dmc devfreq 节点）
    local ddr_cur ddr_avail
    if [ -d "/sys/class/devfreq/dmc" ]; then
        ddr_cur=$(sysval /sys/class/devfreq/dmc/cur_freq)
        ddr_avail=$(sysval /sys/class/devfreq/dmc/available_frequencies)
        log "DDR 频率:  $(( ${ddr_cur:-0} / 1000000 )) MHz（可用: $ddr_avail）"
    fi

    # eMMC 容量与型号
    local blk name sectors model gb gib
    for blk in /sys/block/mmcblk[0-9]; do
        [ -f "$blk/device/type" ] || continue
        name=$(basename "$blk")
        sectors=$(sysval "$blk/size")
        model=$(sysval "$blk/device/name")
        break
    done
    if [ -n "$sectors" ]; then
        gb=$(awk -v s="$sectors" 'BEGIN{printf "%.1f", s * 512 / 1e9}')
        gib=$(awk -v s="$sectors" 'BEGIN{printf "%.1f", s * 512 / 1024 / 1024 / 1024}')
        log "eMMC:      ${gb} GB (${gib} GiB) $(capacity_note "$gb" "$NOMINAL_EMMC_GB")"
        [ -n "$model" ] && [ "$model" != "-" ] && log "eMMC 型号: ${model}"
    else
        log "eMMC:      未识别到 mmcblk 设备"
    fi
}

################################################################################
# 异常项目汇总（末尾把失败/警告再列一次，便于产线拍照/截图回传）
################################################################################

print_issues_summary() {
    if [ "${#ISSUES[@]}" -eq 0 ]; then
        log "\n${GREEN}>>> 无异常项目 <<<${NC}"
        return
    fi

    log ""
    log "${RED}========================================${NC}"
    log "${RED}       异常项目汇总（请拍照回传）${NC}"
    log "${RED}========================================${NC}"
    log "失败 ${FAILED_TESTS} 项 / 警告 ${WARNED_TESTS} 项"
    log ""

    local i=0 entry result module reason tag color
    for entry in "${ISSUES[@]}"; do
        i=$((i + 1))
        result="${entry%%|*}"
        module=$(echo "$entry" | cut -d'|' -f2)
        reason=$(echo "$entry" | cut -d'|' -f3-)
        if [ "$result" = "FAIL" ]; then
            tag="失败"
            color="$RED"
        else
            tag="警告"
            color="$YELLOW"
        fi
        log "${color}[$i] $tag${NC} | 所属检测项: $module"
        log "${color}     现象原因: $reason${NC}"
    done

    log ""
    log "排查日志: $LOG_FILE"
    log "结果清单: $RESULT_FILE"
}

################################################################################
# 主程序
################################################################################

main() {
    # 清理旧日志文件（只保留最新的）
    cleanup_old_logs() {
        local pattern="$1"
        local keep_count=1

        # 查找所有匹配的文件，按修改时间排序（新的在前）
        local files=($(ls -t ${pattern} 2>/dev/null))
        local total=${#files[@]}

        if [ $total -gt $keep_count ]; then
            log "发现 $total 个旧日志文件，保留最新 $keep_count 个，删除其余..."
            # 删除除了最新的之外的所有文件
            for ((i=$keep_count; i<$total; i++)); do
                rm -f "${files[$i]}" && log "  已删除: ${files[$i]}"
            done
        fi
    }

    # 清理旧的日志文件
    cleanup_old_logs "hardware_check_bare_*.log"

    # 清空结果文件
    > "$RESULT_FILE"

    # 打印欢迎信息
    clear
    log "${GREEN}========================================${NC}"
    log "${GREEN}   RK3576 Ubuntu 裸板产测自检脚本${NC}"
    log "${GREEN}   Hardware Check (Bare Board) v1.3${NC}"
    log "${GREEN}========================================${NC}"
    log ""
    log "开始时间: $(date '+%Y-%m-%d %H:%M:%S')"
    log "日志文件: $LOG_FILE"
    log "结果文件: $RESULT_FILE"
    log "检测项: 系统/CPU/NPU/GPU/内存/DDR带宽/eMMC/USB驱动/麦克风信号"

    # 可选依赖自检
    MISSING_TOOLS=""
    for tool in stress-ng arecord python3; do
        command -v "$tool" >/dev/null 2>&1 || MISSING_TOOLS="$MISSING_TOOLS $tool"
    done
    if [ -n "$MISSING_TOOLS" ]; then
        log "${YELLOW}缺少可选工具:$MISSING_TOOLS（对应测试项会跳过或降级）${NC}"
    fi
    if [ "$(id -u)" -ne 0 ]; then
        log "${YELLOW}当前非 root：DDR 频率锁定与 eMMC 直接写入测试可能降级${NC}"
    fi

    # 执行所有测试
    test_system_info
    test_cpu
    test_npu
    test_gpu
    test_memory
    test_ddr
    test_emmc
    test_usb
    test_audio_mic

    # 打印测试总结
    print_header "测试总结"
    log "总测试项: $TOTAL_TESTS"
    log "${GREEN}通过: $PASSED_TESTS${NC}"
    log "${YELLOW}警告: $WARNED_TESTS${NC}"
    log "${RED}失败: $FAILED_TESTS${NC}"

    if [ "$FAILED_TESTS" -eq 0 ] && [ "$WARNED_TESTS" -eq 0 ]; then
        log "\n${GREEN}>>> 所有测试通过！<<<${NC}"
        EXIT_CODE=0
    elif [ "$FAILED_TESTS" -eq 0 ]; then
        log "\n${YELLOW}>>> 无失败项，存在 $WARNED_TESTS 项警告，请确认后放行 <<<${NC}"
        EXIT_CODE=0
    else
        log "\n${RED}>>> 存在测试失败项，请检查日志 <<<${NC}"
        EXIT_CODE=1
    fi

    # 硬件配置（容量/频率，供拍照记录）
    print_hw_config

    # 异常项目汇总（失败/警告再列一次，便于拍照回传）
    print_issues_summary

    log "\n结束时间: $(date '+%Y-%m-%d %H:%M:%S')"
    log "详细日志: $LOG_FILE"
    log "测试结果: $RESULT_FILE"
    log "（有异常项时，请把上方【异常项目汇总】连同本屏拍照回传）"

    exit $EXIT_CODE
}

# 运行主程序
main

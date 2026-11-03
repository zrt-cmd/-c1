#!/bin/bash
# ==========================================================================
#  XFLZ WiFi 配网服务 · 测试监控
#
#  做什么:
#    1. 关掉旧的服务进程，用当前代码重新起一个
#    2. 打印 wlan0 当前的模块状态
#    3. 如果还有旧的 WiFi 连接配置，问你要不要删；删完盯着它是否自动回到 AP
#    4. 之后一直实时显示"当前模式"(STA / AP / 切换中) + 关键日志
#       手机端的操作你自己做 —— 脚本只负责把状态和日志摆在你面前
#
#  用法:
#      sudo bash test.sh             # 全流程
#      sudo bash test.sh --cleanup   # 只收尾（停服务 + 清掉配网热点）
#
#  前置: 请在【串口 / 本地终端】运行。第 3 步删掉配置后设备会断网，
#        走 SSH 的话会直接失联。
# ==========================================================================

set -uo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="$PROJ/bin/wifi-provision"
CONF=/etc/xflz/wifi-provision.conf
RUN=/tmp/wp-test
LOG="$RUN/service.log"

IFACE=wlan0
AP_CON=xflz-provision-ap
AP_IP=192.168.100.1
AP_SSID=""

# ---------------------------------------------------------------- 颜色
if [[ -t 1 ]]; then
  B=$'\033[1m'; R=$'\033[31m'; G=$'\033[32m'; Y=$'\033[33m'
  C=$'\033[36m'; D=$'\033[90m'; N=$'\033[0m'
else
  B=; R=; G=; Y=; C=; D=; N=
fi

hr()     { echo "${C}════════════════════════════════════════════════════════════${N}"; }
info()   { echo "  $*"; }
note()   { echo "  ${C}$*${N}"; }
ok()     { echo "  ${G}✓ $*${N}"; }
bad()    { echo "  ${R}✗ $*${N}"; }
warn()   { echo "${Y}⚠ $*${N}"; }
die()    { echo "${R}✗ $*${N}"; exit 1; }
pause()  { printf '\n  %s按回车继续...%s' "$C" "$N"; read -r _ || true; echo; }

ask() {
  local q="$1" def="${2:-n}" a
  while true; do
    if [[ "$def" == "y" ]]; then printf '%s [Y/n]: ' "$q"; else printf '%s [y/N]: ' "$q"; fi
    read -r a || return 1
    [[ -z "$a" ]] && { [[ "$def" == "y" ]] && return 0 || return 1; }
    case "$a" in
      y|Y|yes|是) return 0 ;;
      n|N|no|否)  return 1 ;;
      *) echo "  请输入 y 或 n" ;;
    esac
  done
}

# ---------------------------------------------------------------- 状态查询（全部只读）
wlan_line()  { nmcli -t -f DEVICE,STATE,CONNECTION device 2>/dev/null \
                 | awk -F: -v i="$IFACE" '$1==i {print; exit}'; }
sta_name()   { nmcli -t -f DEVICE,STATE,CONNECTION device 2>/dev/null \
                 | awk -F: -v i="$IFACE" '$1==i && $2=="connected" {print $3; exit}'; }
sta_connected() { local n; n="$(sta_name)"; [[ -n "$n" && "$n" != "$AP_CON" ]]; }
ap_active()  { nmcli -t -f NAME,DEVICE connection show --active 2>/dev/null \
                 | grep -q "^${AP_CON}:${IFACE}"; }
ap_exists()  { nmcli -t -f NAME connection show 2>/dev/null | grep -qx "$AP_CON"; }
wlan_ip()    { ip -4 addr show "$IFACE" 2>/dev/null | awk '/inet /{print $2; exit}'; }
saved_wifi() { nmcli -t -f NAME,TYPE connection show 2>/dev/null \
                 | awk -F: -v ap="$AP_CON" '$2=="802-11-wireless" && $1!=ap {print $1}'; }
svc_alive()  { pgrep -f "$BIN" >/dev/null 2>&1; }
svc_pid()    { pgrep -f "$BIN" 2>/dev/null | head -1; }

# 射频真正锁在哪个信道 —— NM 报的是"申请了什么"，只有驱动知道"实际在哪"
radio_ch()   { sed -n 's/^Current CH(fc) = //p' \
                 /sys/kernel/debug/ieee80211/*/rtw88/phy_info 2>/dev/null | head -1; }
radio_band() {
  local c; c="$(radio_ch)"
  if [[ -z "$c" ]]; then echo "读不到"
  elif (( c <= 14 )); then echo "2.4GHz"
  else echo "5GHz"; fi
}

# 门户只在 AP 模式下可访问，其余时候拿不到
portal_json() { ap_active && curl -s --max-time 2 "http://${AP_IP}/status.json" 2>/dev/null; }
jfield() { sed -n "s/.*\"$1\":[[:space:]]*\"\([^\"]*\)\".*/\1/p"; }

# ---------------------------------------------------------------- 服务控制
svc_stop() {
  pkill -f "$BIN" 2>/dev/null
  sleep 1
  pkill -9 -f "$BIN" 2>/dev/null
  sleep 1
}

svc_start() {
  mkdir -p "$RUN"
  : > "$LOG"
  ( cd "$PROJ" && exec python3 -u "$BIN" ) >>"$LOG" 2>&1 &
  sleep 2
}

# ---------------------------------------------------------------- 一次性详细状态
print_status() {
  local ip pid n st err sj rc
  ip="$(wlan_ip)"; pid="$(svc_pid)"; n="$(saved_wifi | tr '\n' ' ')"
  rc="$(radio_ch)"
  sj="$(portal_json)"
  st="$(printf '%s' "$sj" | jfield state)"
  err="$(printf '%s' "$sj" | jfield last_error)"

  hr
  printf '  %s当前状态%s   %s\n' "$B" "$N" "$(date '+%F %T')"
  hr
  if [[ -n "$pid" ]]; then
    printf '  被测服务：%s运行中%s (pid %s)\n' "$G" "$N" "$pid"
  else
    printf '  被测服务：%s未运行%s\n' "$R" "$N"
  fi
  printf '  wlan0   ：%s\n' "$(wlan_line)"
  printf '  wlan0 IP：%s\n' "${ip:-无}"
  printf '  射频信道：%s (%s)\n' "${rc:-?}" "$(radio_band)"

  if ap_active; then
    printf '  配网热点：%s已开启%s  %s\n' "$G" "$N" "$AP_SSID"
  else
    printf '  配网热点：未开启\n'
  fi
  printf '  已存WiFi：%s\n' "${n:-（无）}"
  printf '  服务状态机：%s\n' "${st:-（仅 AP 模式下可从门户读到）}"
  printf '  上次错误：%s\n' "${err:-无}"
  printf '  配置文件：%s\n' "$CONF"
  hr
}

# ---------------------------------------------------------------- 实时监控
# 需要关注的关键日志（其余 DEBUG 噪音全部不显示）
KEYPAT='state: .* -> |user requested connect to|connect to .* failed|connected to |AP up:|AP stopped|AP already active|no saved WiFi profile|retry window|retry window expired|station dropped|steady:|re-entering AP mode|web server up|web server down|scan-back|AP start failed|AP unavailable|unknown state|ap_diag'

colorize() {
  local l="$1"
  if [[ "$l" =~ failed|AP.start.failed|AP.unavailable|Traceback|ERROR ]]; then
    printf '%s%s%s' "$R" "$l" "$N"
  elif [[ "$l" =~ 不一致|没发到空中|射频停在|已掉线|读不到|未激活 ]]; then
    printf '%s%s%s' "$R" "$l" "$N"
  elif [[ "$l" =~ state:.+-\>|connected.to.|AP.up:|AP.stopped|判定：射频已在 ]]; then
    printf '%s%s%s' "$G" "$l" "$N"
  elif [[ "$l" =~ user.requested.connect.to|re-entering.AP.mode ]]; then
    printf '%s%s%s' "$Y" "$l" "$N"
  else
    printf '%s%s%s' "$D" "$l" "$N"
  fi
}

# 当前模式一句话
current_mode() {
  local ip sj st err rc
  ip="$(wlan_ip)"
  if ap_active; then
    sj="$(portal_json)"
    st="$(printf '%s' "$sj" | jfield state)"
    err="$(printf '%s' "$sj" | jfield last_error)"
    rc="$(radio_ch)"
    printf '%s模式 = AP 配网%s  热点=%s  IP=%s  状态机=%s  射频=%s(ch%s)' \
      "$G$B" "$N" "$AP_SSID" "${ip:-无}" "${st:-启动中}" \
      "$(radio_band)" "${rc:-?}"
    [[ -n "$err" ]] && printf '  上次错误=%s%s%s' "$R" "$err" "$N"
  elif sta_connected; then
    printf '%s模式 = STA 已联网%s  连接=%s  IP=%s' \
      "$G$B" "$N" "$(sta_name)" "${ip:-无}"
  elif svc_alive; then
    printf '%s模式 = 切换中 / 未连接%s  wlan0=%s' "$Y$B" "$N" "$(wlan_line)"
  else
    printf '%s模式 = 服务未运行%s  wlan0=%s' "$R$B" "$N" "$(wlan_line)"
  fi
}

monitor() {
  local mark last now total line mode
  # 日志在 svc_start 时已被清空，所以从 0 开始 = 把本次运行的日志全部吐出来。
  # 不这么做的话，启动服务到进入监控之间产生的关键日志（进 AP、切 STA）会被跳过。
  mark=0
  last=""

  echo
  hr
  printf '  %s实时监控中%s   手机端操作请随意，这里会一直跟着显示\n' "$B" "$N"
  printf '  %sCtrl-C 退出（服务继续在后台跑）%s\n' "$D" "$N"
  hr

  while true; do
    now="$(date '+%H:%M:%S')"

    # 1) 先吐新日志里的关键行
    total=$(wc -l < "$LOG" 2>/dev/null || echo 0)
    if (( total > mark )); then
      while IFS= read -r line; do
        printf '%s %s\n' "${D}${now}${N}" "$(colorize "$line")"
      done < <(tail -n +"$((mark+1))" "$LOG" 2>/dev/null | grep -E -- "$KEYPAT")
      mark=$total
    fi

    # 2) 模式变了才打状态行，避免刷屏
    mode="$(current_mode)"
    if [[ "$mode" != "$last" ]]; then
      printf '\n%s %s\n\n' "${B}${now}${N}" "$mode"
      last="$mode"
    fi

    sleep 1
  done
}

# ---------------------------------------------------------------- 收尾
cleanup() {
  echo
  hr
  printf '  %s收尾%s\n' "$B" "$N"
  hr
  systemctl stop wifi-provision 2>/dev/null
  svc_stop
  note "被测服务已停止"

  if ap_exists; then
    note "删掉测试期间创建的配网热点连接 $AP_CON"
    nmcli connection delete "$AP_CON" >/dev/null 2>&1 \
      && ok "已删除 $AP_CON" || bad "删除失败，可手工执行：nmcli connection delete $AP_CON"
  else
    note "没有残留的配网热点连接"
  fi

  echo
  note "设备当前保存的 WiFi 记录：$(saved_wifi | tr '\n' ' ' || true)"
  if ! sta_connected; then
    warn "wlan0 现在没有连上任何 WiFi —— 要联网得重新配一次网。"
  fi
  echo
}

on_int() {
  echo
  warn "已退出监控（被测服务仍在后台运行）"
  info "停服务： sudo pkill -f $BIN"
  info "收尾：   sudo bash $0 --cleanup"
  exit 0
}

# ---------------------------------------------------------------- 主流程
main() {
  [[ $EUID -eq 0 ]] || die "请用 sudo 运行：sudo bash $0"
  command -v nmcli >/dev/null || die "找不到 nmcli"
  [[ -f "$BIN" ]] || die "找不到 $BIN"

  if [[ "${1:-}" == "--cleanup" ]]; then
    cleanup
    exit 0
  fi

  mkdir -p "$RUN"
  trap on_int INT

  # 热点 SSID 预告
  local prefix hex
  prefix="$(awk -F= '/^[[:space:]]*ap_ssid_prefix/{gsub(/[[:space:]]/,"",$2); print $2}' "$CONF" 2>/dev/null)"
  prefix="${prefix:-XFLZ}"
  hex="$(tr -d ':' < "/sys/class/net/$IFACE/address" 2>/dev/null | tr '[:lower:]' '[:upper:]')"
  AP_SSID="${prefix}-${hex: -6}"

  if [[ -n "${SSH_CONNECTION:-}" ]]; then
    warn "检测到你在 SSH 会话里！删掉 WiFi 配置后设备会彻底断网，你将失去连接。"
    ask "  确定继续？" || exit 1
  fi

  # ---- 1) 关旧服务，起新服务 ----
  echo
  hr
  printf '  %s1. 重启被测服务%s\n' "$B" "$N"
  hr
  if svc_alive; then
    info "发现旧的服务进程 pid $(svc_pid)，关掉它"
  else
    info "当前没有在跑的服务进程"
  fi
  systemctl stop wifi-provision 2>/dev/null
  svc_stop
  ok "旧服务已关闭"

  svc_start
  if svc_alive; then
    ok "新服务已启动 pid $(svc_pid)"
  else
    bad "服务没起来，看日志：$LOG"
    sed 's/^/    /' "$LOG" 2>/dev/null | tail -20
    die "启动失败"
  fi
  note "日志: $LOG"

  # ---- 2) 显示当前的 wifi 模块状态 ----
  echo
  print_status

  # ---- 3) 有旧的连接配置就删掉，然后盯它回不回 AP ----
  local ps; ps="$(saved_wifi)"
  if [[ -z "$ps" ]]; then
    echo
    note "没有已保存的 WiFi 配置 —— 服务应当直接进入 AP 模式（下面监控里能看到）"
  else
    echo
    hr
    printf '  %s2. 删除旧的 WiFi 连接配置%s\n' "$B" "$N"
    hr
    info "当前已保存的 WiFi 连接："
    echo "$ps" | sed 's/^/      - /'
    echo
    warn "删掉之后设备就忘了这些 WiFi，不会自动还原；想连回来得重新配网。"
    info "外面那个 WiFi 本身没被改动，只是设备这边不再自动去连。"
    echo
    if ask "  现在删除吗？" n; then
      local p
      for p in $ps; do
        if nmcli connection delete "$p" >/dev/null 2>&1; then
          ok "已删除 $p"
        else
          bad "删除 $p 失败，可手工执行：nmcli connection delete \"$p\""
        fi
      done
      echo
      note "预期：服务约 20 秒内经「掉线容忍 → 重试 → 切 AP」回到配网模式，"
      note "      下面监控里会看到状态从 STA 变成 AP。"
    else
      warn "已跳过删除。服务会继续连着原来的 WiFi，不会进 AP 模式。"
    fi
  fi

  # ---- 4) 实时监控 ----
  pause
  monitor
}

main "$@"

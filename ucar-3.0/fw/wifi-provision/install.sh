#!/bin/bash
# ==========================================================================
#  XFLZ WiFi 配网服务 · 一键安装
#
#  做什么:
#    把 wifi-provision 装成 systemd 常驻服务，装一次之后每次开机自动跑。
#      1. 清掉手动起的野实例（防和 systemd 抢 wlan0 / 80 端口）
#      2. 装 unit -> /etc/systemd/system/wifi-provision.service
#      3. 装日志轮转 -> /etc/logrotate.d/wifi-provision
#      4. systemctl enable --now，并打印自检结果
#
#  用法:
#      sudo bash install.sh              # 安装 / 升级（可重复执行）
#      sudo bash install.sh --uninstall  # 卸载（保留配置和日志）
#
#  说明:
#      日志落在 /var/log/wifi-provision.log（本机 journald 不持久，重启即丢，
#      所以服务日志直接写盘）。卸载不会删掉这个日志，也不会删掉
#      /etc/xflz/wifi-provision.conf。
# ==========================================================================

set -uo pipefail

PROJ="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
BIN="$PROJ/bin/wifi-provision"
UNIT_SRC="$PROJ/packaging/wifi-provision.service"
ROTATE_SRC="$PROJ/packaging/wifi-provision.logrotate"
CONF_SAMPLE="$PROJ/packaging/wifi-provision.conf.sample"
REQS="$PROJ/requirements.txt"

SVC=wifi-provision
UNIT_DST="/etc/systemd/system/$SVC.service"
ROTATE_DST="/etc/logrotate.d/$SVC"
CONF=/etc/xflz/wifi-provision.conf
CONF_DIR="$(dirname "$CONF")"
LOG=/var/log/wifi-provision.log

if [[ -t 1 ]]; then
  B=$'\033[1m'; R=$'\033[31m'; G=$'\033[32m'; Y=$'\033[33m'
  C=$'\033[36m'; N=$'\033[0m'
else
  B=; R=; G=; Y=; C=; N=
fi

hr()   { printf '%s════════════════════════════════════════════════════════════%s\n' "$C" "$N"; }
info() { echo "  $*"; }
ok()   { echo "  ${G}✓ $*${N}"; }
warn() { echo "  ${Y}⚠ $*${N}"; }
die()  { echo "  ${R}✗ $*${N}"; exit 1; }

need_root() { [[ $EUID -eq 0 ]] || die "请用 sudo 运行：sudo bash $0"; }

kill_stray() {
  # Only manual instances (e.g. one started by test.sh) -- the systemd-managed
  # process is deliberately left alone, since systemctl handles that one. Killing
  # it here would race with Restart=always.
  local mainpid pids
  mainpid="$(systemctl show -p MainPID --value "$SVC" 2>/dev/null)"
  pids="$(pgrep -f "$BIN" 2>/dev/null | grep -vx "${mainpid:-0}" | tr '\n' ' ')"
  if [[ -n "${pids// /}" ]]; then
    info "发现 systemd 之外手动起的实例（pid ${pids% }），先关掉，免得和 systemd 抢 wlan0 / 80 端口"
    kill ${pids} 2>/dev/null
    sleep 1
    kill -9 ${pids} 2>/dev/null
    sleep 1
    ok "野实例已清理"
  fi
}

install_service() {
  echo
  hr
  printf '  %s安装 %s%s\n' "$B" "$SVC" "$N"
  hr

  # ---- 前置检查 ----
  command -v systemctl >/dev/null || die "没有 systemctl，这不是 systemd 系统"
  [[ -f "$BIN" ]]         || die "找不到 $BIN"
  [[ -f "$UNIT_SRC" ]]    || die "找不到 $UNIT_SRC"
  [[ -f "$ROTATE_SRC" ]]  || die "找不到 $ROTATE_SRC"
  [[ -f "$CONF_SAMPLE" ]] || die "找不到 $CONF_SAMPLE"

  # 依赖缺失就按 requirements.txt 装一次。本机是 pip 22.0.2 / Python 3.10，
  # 没有 PEP 668 限制，直接装即可，装到 /usr/local 下供系统 python3 使用。
  if ! python3 -c "import flask, werkzeug" 2>/dev/null; then
    warn "缺少 flask / werkzeug，按 $REQS 安装"
    python3 -m pip install -r "$REQS" >/dev/null 2>&1 \
      || die "装 flask / werkzeug 失败，请手动执行: python3 -m pip install -r $REQS"
    python3 -c "import flask, werkzeug" 2>/dev/null \
      || die "装完仍导入不了 flask / werkzeug"
    ok "已安装 python 依赖"
  fi
  ok "前置检查通过"

  # ---- 配置 ----
  # /etc/xflz 不一定存在（干净板子上就没有），先建出来再落配置。
  mkdir -p "$CONF_DIR"
  if [[ -f "$CONF" ]]; then
    ok "配置文件 $CONF（已存在，保留不覆盖）"
  else
    install -m 0644 "$CONF_SAMPLE" "$CONF" \
      && ok "已生成配置 $CONF" || die "写 $CONF 失败"
  fi

  # ---- 清野实例 ----
  kill_stray
  systemctl stop "$SVC" 2>/dev/null

  # ---- 装文件 ----
  # unit 是模板: 里面的 @PROJ@ 替换成本次安装所在的目录. 路径不写死, 换目录
  # 或换板子都不用改文件, 也避免装出来的 unit 指向一个不存在的旧路径.
  local unit_tmp
  unit_tmp="$(mktemp)"
  sed "s|@PROJ@|$PROJ|g" "$UNIT_SRC" > "$unit_tmp"
  if grep -q "@PROJ@" "$unit_tmp"; then
    rm -f "$unit_tmp"
    die "$UNIT_SRC 里还有未替换的 @PROJ@，模板有问题"
  fi
  if [[ -f "$UNIT_DST" ]] && cmp -s "$unit_tmp" "$UNIT_DST"; then
    info "$UNIT_DST 内容没变，跳过写入"
  else
    install -m 0644 "$unit_tmp" "$UNIT_DST" \
      && ok "已写入 $UNIT_DST" || die "写 $UNIT_DST 失败"
  fi
  rm -f "$unit_tmp"

  install -m 0644 "$ROTATE_SRC" "$ROTATE_DST" \
    && ok "已写入 $ROTATE_DST" || die "写 $ROTATE_DST 失败"

  # ---- 启用并启动 ----
  systemctl daemon-reload
  systemctl enable --now "$SVC" >/dev/null 2>&1
  ok "已 enable --now"

  check
}

check() {
  echo
  hr
  printf '  %s自检%s\n' "$B" "$N"
  hr

  local en act pid i
  # Type=simple means enable --now returns before the first log line lands, so
  # give it a moment rather than reporting a false "log is empty".
  for (( i = 0; i < 20; i++ )); do
    [[ -s "$LOG" ]] && break
    systemctl is-active --quiet "$SVC" || break
    sleep 0.5
  done

  en="$(systemctl is-enabled "$SVC" 2>&1)"
  act="$(systemctl is-active "$SVC" 2>&1)"

  [[ "$en" == "enabled" ]] && ok "开机自启：enabled" \
                           || { warn "开机自启：$en（没开起来，下面有原因）"; }

  if [[ "$act" == "active" ]]; then
    pid="$(systemctl show -p MainPID --value "$SVC" 2>/dev/null)"
    ok "服务状态：active (pid ${pid:-?})"
  else
    warn "服务状态：$act"
    info "最近日志："
    systemctl status "$SVC" --no-pager -n 20 2>&1 | sed 's/^/      /'
  fi

  echo
  info "服务日志：$LOG"
  if [[ -s "$LOG" ]]; then
    echo
    printf '  %s最近 15 行：%s\n' "$C" "$N"
    tail -n 15 "$LOG" | sed 's/^/      /'
    echo
    info "确认上面出现 \"device AP SSID will be: XFLZ-xxxxxx\"，"
    info "且不是 XFLZ-000000 —— 是 000000 说明 wlan0 还没就绪，可等 5 秒后重启服务再看。"
  else
    warn "日志文件还是空的，服务可能刚起、或启动失败"
  fi

  echo
  info "常用命令："
  info "  systemctl status $SVC"
  info "  systemctl restart $SVC"
  info "  tail -f $LOG"
  echo
}

uninstall_service() {
  echo
  hr
  printf '  %s卸载 %s%s\n' "$B" "$SVC" "$N"
  hr

  systemctl disable --now "$SVC" >/dev/null 2>&1 && ok "已停止并取消开机自启" \
                                                 || warn "停止/禁用时有报错，可能本来就没装"
  kill_stray

  [[ -f "$UNIT_DST" ]]   && rm -f "$UNIT_DST"   && ok "已删除 $UNIT_DST"
  [[ -f "$ROTATE_DST" ]] && rm -f "$ROTATE_DST" && ok "已删除 $ROTATE_DST"
  systemctl daemon-reload && ok "已 daemon-reload"

  echo
  info "以下内容已保留，需要的话自己删："
  info "  配置：$CONF"
  info "  日志：$LOG"
  echo
}

case "${1:-}" in
  "")           need_root; install_service ;;
  --uninstall)  need_root; uninstall_service ;;
  -h|--help)    sed -n '2,20p' "$0" ;;
  *)            die "不认识的参数：$1（可用：--uninstall / --help）" ;;
esac

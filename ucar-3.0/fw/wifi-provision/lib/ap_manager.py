"""nmcli wrapper for WiFi station + AP management."""

import logging
import os
import subprocess
import time

log = logging.getLogger("ap_manager")

AP_CON_NAME = "xflz-provision-ap"

# Pin the AP to an explicit channel: with band=bg and no channel, NM auto-picks
# ch1, and on this board rtw88 never radiates the beacon on ch1 (ch6 works).
AP_BAND = "2.4GHz"
AP_CHANNEL = 6


def _run(args, timeout=20, check=False):
    log.debug("exec: %s", " ".join(args))
    return subprocess.run(
        args, capture_output=True, text=True,
        timeout=timeout, check=check,
    )


def _nmcli(*args, timeout=20, check=False):
    return _run(["nmcli", *args], timeout=timeout, check=check)


def _friendly_error(raw):
    """Map the common nmcli activation failures onto something a user can act on."""
    low = raw.lower()
    if "no network with ssid" in low or "not found" in low:
        return "未找到该 WiFi，请确认名称是否完全一致（区分大小写）"
    if "secrets were required" in low or "no secrets" in low \
            or "802-11-wireless-security" in low:
        return "密码错误，请重新输入"
    if "timed out" in low or "timeout" in low:
        return "连接超时，请确认设备在信号范围内"
    if "not authorized" in low or "not allowed" in low:
        return "该 WiFi 拒绝了连接请求"
    return raw or "连接失败"


def _phy_debugfs():
    """Path to this radio's driver debug directory, or None."""
    try:
        phy = sorted(os.listdir("/sys/class/ieee80211"))[0]
    except (OSError, IndexError):
        return None
    return f"/sys/kernel/debug/ieee80211/{phy}"


def read_radio_channel():
    """Channel the RF is actually tuned to right now, or None.

    Only the driver knows this. NetworkManager reports what it *asked* for,
    which on this box can differ from where the radio ended up, so every
    "is the hotspot really up" judgement has to start here.
    """
    base = _phy_debugfs()
    if base is None:
        return None
    try:
        with open(f"{base}/rtw88/phy_info") as f:
            for line in f:
                if line.startswith("Current CH(fc)"):
                    return int(line.split("=")[-1].strip())
    except (OSError, ValueError) as e:
        log.debug("cannot read radio channel: %s", e)
    return None


def band_name(chan):
    if chan is None:
        return "读不到"
    if chan <= 14:
        return "2.4GHz"
    if chan >= 36:
        return "5GHz"
    return "未知频段"


class ApManager:
    def __init__(self, iface, ssid_prefix):
        self.iface = iface
        self.ssid_prefix = ssid_prefix
        self._ap_ssid = None

    @property
    def ap_ssid(self):
        if self._ap_ssid is None:
            self._ap_ssid = f"{self.ssid_prefix}-{self._mac_suffix()}"
        return self._ap_ssid

    def _mac_suffix(self):
        try:
            with open(f"/sys/class/net/{self.iface}/address") as f:
                mac = f.read().strip()
            return mac.replace(":", "").upper()[-6:]
        except Exception as e:
            log.warning("cannot read MAC for %s: %s", self.iface, e)
            return "000000"

    def is_station_connected(self):
        r = _nmcli("-t", "-f", "DEVICE,STATE", "device")
        for line in r.stdout.splitlines():
            parts = line.split(":", 1)
            if len(parts) == 2 and parts[0] == self.iface:
                return parts[1] == "connected" and not self.is_ap_active()
        return False

    def is_connecting(self):
        """True while NM is mid-activation on this device, any profile."""
        r = _nmcli("-t", "-f", "DEVICE,STATE", "device")
        for line in r.stdout.splitlines():
            parts = line.split(":", 1)
            if len(parts) == 2 and parts[0] == self.iface:
                return parts[1] == "connecting"
        return False

    def is_ap_active(self):
        r = _nmcli("-t", "-f", "NAME,DEVICE", "connection", "show", "--active")
        for line in r.stdout.splitlines():
            parts = line.rsplit(":", 1)
            if len(parts) == 2 and parts[0] == AP_CON_NAME:
                return True
        return False

    def has_ap_client(self):
        """True while a client holds a DHCP lease on the AP's own interface.

        NM's shared mode runs dnsmasq, which records leases here. A live lease
        means a phone is connected to the hotspot right now -- taking the AP
        down for a scan-back would cut somebody off mid-provisioning.
        """
        path = f"/var/lib/NetworkManager/dnsmasq-{self.iface}.leases"
        now = time.time()
        try:
            with open(path) as f:
                for line in f:
                    parts = line.split()
                    if not parts:
                        continue
                    try:
                        if int(parts[0]) > now:
                            return True
                    except ValueError:
                        continue
        except OSError as e:
            log.debug("cannot read %s: %s", path, e)
        return False

    def current_connection(self):
        """Name of the active station connection profile, or None."""
        r = _nmcli("-t", "-f", "DEVICE,TYPE,STATE,CONNECTION", "device")
        for line in r.stdout.splitlines():
            parts = line.split(":")
            if len(parts) >= 4 and parts[0] == self.iface \
                    and parts[2] == "connected":
                name = parts[3]
                if name and name != AP_CON_NAME:
                    return name
        return None

    def _wlan_ip(self):
        r = _run(["ip", "-4", "-o", "addr", "show", self.iface])
        for line in r.stdout.splitlines():
            for tok in line.split():
                if "/" in tok and tok[0].isdigit():
                    return tok
        return "无"

    def _recent(self, args, count=4, only=None, skip=None):
        """Last few matching log lines from a command, as a single string."""
        try:
            r = _run(args, timeout=10)
        except subprocess.SubprocessError as e:
            return f"(读取失败: {e})"
        lines = [l.strip() for l in r.stdout.splitlines() if l.strip()]
        if only:
            lines = [l for l in lines if any(k in l for k in only)]
        if skip:
            lines = [l for l in lines if not any(k in l for k in skip)]
        return " | ".join(lines[-count:]) if lines else "(无)"

    def _sample_channel(self, seconds=5, interval=1.0):
        """Watch the radio channel for a few seconds; returns every sample.

        wpa_supplicant reports AP-ENABLED as soon as the driver accepts the
        request, which can be before the radio has actually retuned. A single
        immediate read would then mistake "mid-switch" for "switch failed".
        """
        samples = []
        deadline = time.monotonic() + seconds
        while True:
            samples.append(read_radio_channel())
            if time.monotonic() >= deadline:
                return samples
            time.sleep(interval)

    def log_ap_diagnostics(self, chan_before):
        """Say out loud whether the hotspot is plausibly reachable on the air.

        The failure this exists for: NM reports the AP as activated, gets the
        192.168.100.1 address and starts dnsmasq, while the radio is still
        tuned to whatever band the station was last on. The AP parameters are
        then programmed onto a radio that is not on the AP's channel, no usable
        beacon goes out, and a phone scanning either band sees nothing -- every
        layer on the device meanwhile claims success.
        """
        samples = self._sample_channel()
        chan_after = samples[-1]
        log.info("[ap_diag] ===== AP 启动自检 =====")
        log.info("[ap_diag] 期望频段: %s (信道 %d)  期望SSID: %s",
                 AP_BAND, AP_CHANNEL, self.ap_ssid)
        log.info("[ap_diag] 启动前射频: %s (信道 %s)", band_name(chan_before),
                 chan_before if chan_before is not None else "读不到")
        log.info("[ap_diag] 启动后射频采样: %s -> %s (信道 %s)",
                 samples, band_name(chan_after),
                 chan_after if chan_after is not None else "读不到")
        log.info("[ap_diag] 接口: %s  IP=%s  NM连接=%s",
                 self.iface, self._wlan_ip(),
                 "AP 已激活" if self.is_ap_active() else "AP 未激活")

        if chan_after is None:
            log.warning("[ap_diag] 判定：读不到射频信道（debugfs 不可用），"
                        "无法确认射频是否切到 AP 信道")
        elif band_name(chan_after) != AP_BAND:
            log.warning(
                "[ap_diag] 判定：射频停在 %s，不是 AP 要求的 %s。"
                "NM 报的“热点已启动”只是配置成功，射频并没有切过去，"
                "信标大概率没发到空中 —— 手机扫不到就是这个原因。",
                band_name(chan_after), AP_BAND)
        else:
            log.info(
                "[ap_diag] 判定：射频已在 %s，配置与射频一致。"
                "若手机仍搜不到，问题不在频段，而在信标本身"
                "（rtw88 的 AP 发包路径）。", AP_BAND)

        log.info("[ap_diag] supplicant: %s",
                 self._recent(["journalctl", "-t", "wpa_supplicant",
                               "-n", "8", "--no-pager"], count=3))
        log.info("[ap_diag] 内核 AP/信标相关输出: %s",
                 self._recent(["dmesg"], count=4,
                              only=("wlan0", "rtw88", "ieee80211",
                                    "mac80211", "cfg80211", "beacon",
                                    "AP mode", "softap"),
                              skip=("rtw_debugfs", "Call trace", "WARNING:",
                                    "Modules linked in", "Hardware name",
                                    "pstate:", "CPU: ")))
        log.info("[ap_diag] ======================")

    def start_ap(self):
        """Start the provisioning AP. Idempotent."""
        if self.is_ap_active():
            log.info("AP already active: %s", self.ap_ssid)
            return True
        chan_before = read_radio_channel()
        _nmcli("connection", "delete", AP_CON_NAME, check=False)
        r = _nmcli(
            "connection", "add",
            "type", "wifi",
            "ifname", self.iface,
            "con-name", AP_CON_NAME,
            "autoconnect", "no",
            "ssid", self.ap_ssid,
            "802-11-wireless.mode", "ap",
            "802-11-wireless.band", "bg",
            "802-11-wireless.channel", str(AP_CHANNEL),
            "ipv4.addresses", "192.168.100.1/24",
            "ipv4.method", "shared",
            "ipv6.method", "disabled",
        )
        if r.returncode != 0:
            log.error("AP add failed: %s", r.stderr.strip())
            return False
        r = _nmcli("connection", "up", AP_CON_NAME, timeout=25)
        if r.returncode != 0:
            log.error("AP up failed: %s", r.stderr.strip())
            _nmcli("connection", "delete", AP_CON_NAME, check=False)
            return False
        log.info("AP up: %s (open)", self.ap_ssid)
        self.log_ap_diagnostics(chan_before)
        return True

    def stop_ap(self):
        _nmcli("connection", "down", AP_CON_NAME, check=False)
        _nmcli("connection", "delete", AP_CON_NAME, check=False)
        log.info("AP stopped")

    def _profile_name_for_ssid(self, ssid):
        """Name of the saved profile owning this SSID, or None.

        Matched on the SSID rather than on the profile name: nmcli names a new
        profile after the SSID, but appends a suffix when that name is already
        taken, so name matching alone would quietly build duplicates.
        """
        r = _nmcli("-t", "-f", "NAME,TYPE", "connection", "show")
        for line in r.stdout.splitlines():
            name, _, ctype = line.rpartition(":")
            if ctype != "802-11-wireless" or not name or name == AP_CON_NAME:
                continue
            s = _nmcli("-t", "-g", "802-11-wireless.ssid",
                       "connection", "show", name)
            if s.stdout.strip() == ssid:
                return name
        return None

    def _stored_credentials(self, name):
        """(key-mgmt, psk) as stored now, so a failure can be rolled back."""
        def prop(key):
            return _nmcli("-s", "-t", "-g", key,
                          "connection", "show", name).stdout.strip()
        return (prop("802-11-wireless-security.key-mgmt"),
                prop("802-11-wireless-security.psk"))

    def _write_credentials(self, name, password):
        if password:
            _nmcli("connection", "modify", name,
                   "wifi-sec.key-mgmt", "wpa-psk", "wifi-sec.psk", password)
        else:
            _nmcli("connection", "modify", name, "wifi-sec.key-mgmt", "none")

    def _undo_failed_connect(self, ssid, existing, saved):
        """Leave the profile list exactly as it was before the attempt."""
        if existing is None:
            # This call created the profile. Removing it stops NetworkManager
            # from auto-retrying credentials we already know are wrong.
            _nmcli("connection", "delete", ssid, check=False)
            return
        key_mgmt, psk = saved
        if not key_mgmt:
            return
        args = ["connection", "modify", existing, "wifi-sec.key-mgmt", key_mgmt]
        if psk:
            args += ["wifi-sec.psk", psk]
        _nmcli(*args, check=False)

    def connect(self, ssid, password, timeout):
        """Connect to a WiFi network. Returns (ok, error_msg).

        A failed attempt must not cost the user a working saved network, so an
        existing profile is updated in place and put back if the new password
        turns out to be wrong. Only a profile created by this call is removed.
        """
        existing = self._profile_name_for_ssid(ssid)
        saved = None
        if existing:
            saved = self._stored_credentials(existing)
            self._write_credentials(existing, password)
            # Re-activate from scratch: an already-active profile keeps its old
            # key, so a corrected password would silently not take effect.
            _nmcli("connection", "down", existing, timeout=15)
            args = ["connection", "up", existing, "ifname", self.iface]
        else:
            args = ["device", "wifi", "connect", ssid, "ifname", self.iface]
            if password:
                args += ["password", password]

        try:
            r = _nmcli("--wait", str(timeout), *args, timeout=timeout + 5)
        except subprocess.TimeoutExpired:
            log.warning("connect to %s timed out after %ds", ssid, timeout)
            self._undo_failed_connect(ssid, existing, saved)
            return False, "连接超时"

        if r.returncode == 0:
            log.info("connected to %s", ssid)
            return True, ""

        err = (r.stderr or r.stdout).strip()
        log.warning("connect to %s failed: %s", ssid, err)
        self._undo_failed_connect(ssid, existing, saved)
        return False, _friendly_error(err)

    def disconnect(self):
        """Force-disconnect wlan0 from any active connection (except AP)."""
        # Get current connection name
        r = _nmcli("-t", "-f", "DEVICE,TYPE,CONNECTION", "device")
        for line in r.stdout.splitlines():
            parts = line.split(":")
            if len(parts) >= 3 and parts[0] == self.iface:
                con_name = parts[2]
                if con_name and con_name != AP_CON_NAME:
                    log.info("disconnecting %s from %s", self.iface, con_name)
                    _nmcli("connection", "down", con_name, check=False)
                    return
        log.debug("no active connection to disconnect")

    def saved_wifi_profiles(self):
        """Names of saved WiFi connections, excluding the provisioning AP."""
        r = _nmcli("-t", "-f", "NAME,TYPE", "connection", "show")
        names = []
        for line in r.stdout.splitlines():
            # rpartition so a ':' inside the profile name cannot shift the parse
            name, _, ctype = line.rpartition(":")
            if ctype == "802-11-wireless" and name and name != AP_CON_NAME:
                names.append(name)
        return names

    def reconnect_saved(self, names, per_profile_seconds):
        """Bring saved profiles up one at a time, each within its own slice.

        Every profile gets per_profile_seconds to itself instead of sharing one
        budget. Sharing is what breaks this: the first entry in the list eats
        the whole budget whenever its network is out of range, so the profiles
        behind it are never tried at all -- the device sits next to a saved
        network it could join and still falls back to AP mode.

        Nothing is done while NM is mid-activation. Re-issuing `connection up`
        at that moment tears the activation down and starts it over, throwing
        away the scan and association progress already made.

        True as soon as a station connection is up.
        """
        wait = max(1, per_profile_seconds)
        for name in names:
            if self.is_station_connected():
                return True
            if self.is_connecting():
                log.debug("NM is mid-activation, leaving it alone")
                return False
            try:
                r = _nmcli("--wait", str(wait), "connection", "up", name,
                           timeout=wait + 5)
            except subprocess.TimeoutExpired:
                log.warning("reconnect to %s timed out after %ds", name, wait)
                continue
            if self.is_station_connected():
                return True
            log.info("reconnect to %s did not connect: %s", name,
                     (r.stderr or r.stdout).strip() or "(无输出)")
        return False

    def wait_autoconnect(self, seconds):
        """Wait up to N seconds for NM to auto-connect to any saved network."""
        deadline = time.monotonic() + seconds
        while time.monotonic() < deadline:
            if self.is_station_connected():
                return True
            time.sleep(1)
        return False

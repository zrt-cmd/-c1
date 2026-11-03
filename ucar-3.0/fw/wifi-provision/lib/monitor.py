"""WiFi provisioning state machine."""

import logging
import threading
import time

from lib.ap_manager import read_radio_channel, band_name

log = logging.getLogger("monitor")

STATE_STA_RETRY = "sta_retry"
STATE_STEADY = "steady"
STATE_AP_MODE = "ap_mode"
STATE_CONNECTED = "connected"

AP_HEALTH_INTERVAL = 15


class Monitor:
    def __init__(self, cfg, ap):
        self.cfg = cfg
        self.ap = ap
        self.web = None  # attached by main

        self._state = STATE_STA_RETRY
        self._stop = threading.Event()
        self._lock = threading.Lock()

        # Shared state (web thread reads these)
        self._last_error = ""
        self._creds = None  # (ssid, password) or None
        self._creds_evt = threading.Event()

    # ---- Public API for web thread ----

    def get_last_error(self):
        with self._lock:
            return self._last_error

    def get_status_snapshot(self):
        with self._lock:
            return {"state": self._state, "last_error": self._last_error}

    def submit_credentials(self, ssid, password):
        with self._lock:
            self._creds = (ssid, password)
            self._creds_evt.set()

    def stop(self):
        self._stop.set()

    def _take_credentials(self):
        """Consume a pending submission atomically with the wakeup event."""
        with self._lock:
            creds = self._creds
            self._creds = None
            self._creds_evt.clear()
            return creds

    # ---- Main loop ----

    def run(self):
        log.info("monitor started, state=%s", self._state)
        while not self._stop.is_set():
            try:
                if self._state == STATE_STA_RETRY:
                    self._do_sta_retry()
                elif self._state == STATE_STEADY:
                    self._do_steady()
                elif self._state == STATE_AP_MODE:
                    self._do_ap_mode()
                elif self._state == STATE_CONNECTED:
                    self._do_connected()
                else:
                    # Never fall through to a bare spin: an unknown state would
                    # otherwise loop here at 100% CPU doing nothing.
                    log.error("unknown state %r, resetting to %s",
                              self._state, STATE_STA_RETRY)
                    self._set_state(STATE_STA_RETRY)
                    time.sleep(1)
            except Exception as e:
                log.exception("state %s crashed: %s", self._state, e)
                time.sleep(2)
        log.info("monitor stopped")

    def _set_state(self, s):
        log.info("state: %s -> %s", self._state, s)
        with self._lock:
            self._state = s

    def _wait_nm_then_drive(self, profiles, grace, window):
        """Let NM autoconnect first, then work the saved profiles by hand.

        NM only considers networks it has actually seen in a scan, so in a room
        where just one saved network exists it goes straight for that one.
        Driving the profiles ourselves is a poor substitute for that: nmcli
        lists them in an order that says nothing about which are in range, and
        a single out-of-range entry can soak up the whole retry window.
        """
        if self.ap.wait_autoconnect(grace):
            log.info("connected to %s", self.ap.current_connection())
            return True
        log.info("NM did not connect within %ds, driving saved profiles", grace)
        return self._active_retry(profiles, window)

    def _active_retry(self, profiles, window):
        """Bring saved profiles up by hand for up to `window` seconds."""
        deadline = time.monotonic() + window
        while not self._stop.is_set() and time.monotonic() < deadline:
            if self.ap.is_station_connected():
                return True
            if self.ap.is_connecting():
                # NM is already working on something; restarting it would
                # throw away the scan and association progress it has made.
                time.sleep(1)
                continue
            remaining = int(deadline - time.monotonic())
            if remaining <= 1:
                break
            self.ap.reconnect_saved(
                profiles,
                min(self.cfg.per_profile_attempt_seconds, remaining))
            time.sleep(1)
        return self.ap.is_station_connected()

    def _do_sta_retry(self):
        profiles = self.ap.saved_wifi_profiles()
        if not profiles:
            log.info("no saved WiFi profile, entering AP mode directly")
            self._set_state(STATE_AP_MODE)
            return

        grace = self.cfg.boot_grace_seconds
        window = self.cfg.sta_retry_seconds
        log.info("saved WiFi %s; NM grace %ds, retry window %ds",
                 profiles, grace, window)

        if self._wait_nm_then_drive(profiles, grace, window):
            self._set_state(STATE_CONNECTED)
            return

        log.info("STA retry window expired, entering AP mode")
        self._set_state(STATE_AP_MODE)

    def _do_steady(self):
        """Connection dropped: reconnect before falling back to AP mode."""
        profiles = self.ap.saved_wifi_profiles()
        if not profiles:
            log.info("steady: no saved profile left, entering AP mode")
            self._set_state(STATE_AP_MODE)
            return

        grace = self.cfg.disconnect_threshold_seconds
        window = self.cfg.sta_retry_seconds
        log.info("steady: retrying %s (NM grace %ds, window %ds)",
                 profiles, grace, window)

        if self._wait_nm_then_drive(profiles, grace, window):
            self._set_state(STATE_CONNECTED)
            return

        log.info("steady: still down, entering AP mode")
        self._set_state(STATE_AP_MODE)

    def _start_ap_with_retry(self):
        """Bring the AP up, cleaning up wlan0 between attempts. True on success."""
        max_retries = 3
        for attempt in range(1, max_retries + 1):
            if self.ap.start_ap():
                return True
            log.error("AP start failed (attempt %d/%d), cleaning up wlan0 and retrying",
                      attempt, max_retries)
            self.ap.disconnect()
            time.sleep(2)
        log.error("AP start failed after %d attempts", max_retries)
        return False

    def _restore_ap(self):
        """Bring AP and the config web server back up. True on success."""
        if not self.ap.start_ap():
            return False
        if self.web is not None:
            self.web.start()
        return True

    def _ap_health(self):
        """(key, description) describing the AP's current on-air state.

        NetworkManager's "activated" only means the profile was applied. The
        radio channel is what decides whether a phone can actually find the
        hotspot, so watch that and flag it the moment it stops matching.
        """
        chan = read_radio_channel()
        active = self.ap.is_ap_active()
        state = "已激活" if active else "已掉线"
        return (chan, active), (
            f"[ap_diag] AP 存活: 射频={band_name(chan)}"
            f"(信道 {chan if chan is not None else '读不到'}) "
            f"NM连接={state}")

    def _do_ap_mode(self):
        if not self._start_ap_with_retry():
            log.error("AP unavailable, retrying in 10s")
            time.sleep(10)
            return

        if self.web is not None:
            self.web.start()

        last_scan_back = time.monotonic()
        last_health = 0.0
        health = None
        while not self._stop.is_set():
            creds = self._take_credentials()
            if creds:
                self._try_connect(*creds)
                return

            if self.cfg.scan_interval_seconds > 0 and \
                    time.monotonic() - last_scan_back >= \
                    self.cfg.scan_interval_seconds:
                last_scan_back = time.monotonic()
                if self._scan_back_allowed() and self._try_scan_back():
                    return

            now = time.monotonic()
            if now - last_health >= AP_HEALTH_INTERVAL:
                last_health = now
                key, desc = self._ap_health()
                if key != health:
                    if not key[1]:
                        log.warning("%s  <- 配网热点已经不在 NM 里了", desc)
                    else:
                        log.info("%s", desc)
                    health = key

            time.sleep(1)

    def _scan_back_allowed(self):
        """Whether pausing the AP to look for a known network is safe right now.

        Pausing drops the hotspot, so it must not happen while somebody is
        using it. A phone holding a DHCP lease, or a config page opened moments
        ago, both mean a person is mid-provisioning -- cutting them off there
        is worse than sitting in AP mode a while longer.
        """
        if not self.ap.saved_wifi_profiles():
            return False

        if self.ap.has_ap_client():
            log.info("scan-back skipped: a client is connected to the AP")
            return False

        if self.web is not None:
            idle = self.web.seconds_since_request()
            if idle is not None and idle < self.cfg.scan_back_idle_seconds:
                log.info("scan-back skipped: config page opened %.0fs ago", idle)
                return False

        return True

    def _try_scan_back(self):
        """Briefly leave AP so NM can reconnect to an already-saved network.

        Returning True means _do_ap_mode must exit: either we are connected, or
        the AP could not be restored and the state was reset to re-enter AP mode
        through the normal retry path. The AP is never left down on return.
        """
        log.info("scan-back: pausing AP for %ds",
                 self.cfg.scan_back_window_seconds)
        if self.web is not None:
            self.web.stop()
        self.ap.stop_ap()

        if self.ap.wait_autoconnect(self.cfg.scan_back_window_seconds):
            log.info("scan-back: reconnected to %s", self.ap.current_connection())
            self._set_state(STATE_CONNECTED)
            return True

        log.info("scan-back: no known network, restoring AP")
        if self._restore_ap():
            return False
        log.error("scan-back: could not restore AP, re-entering AP mode")
        self._set_state(STATE_AP_MODE)
        return True

    def _try_connect(self, ssid, password):
        log.info("user requested connect to %s", ssid)
        if self.web is not None:
            self.web.stop()
        self.ap.stop_ap()
        ok, err = self.ap.connect(
            ssid, password, self.cfg.connect_attempt_timeout)
        if ok:
            with self._lock:
                self._last_error = ""
            self._set_state(STATE_CONNECTED)
        else:
            with self._lock:
                self._last_error = err or "连接失败"
            # Ensure wlan0 is fully disconnected before re-entering AP mode
            log.info("forcing wlan0 disconnect before AP restart")
            self.ap.disconnect()
            time.sleep(2)  # Let NM settle
            log.info("re-entering AP mode after failure")
            self._set_state(STATE_AP_MODE)

    def _do_connected(self):
        log.info("connected to %s", self.ap.current_connection())
        threshold = self.cfg.disconnect_threshold_seconds
        drop_deadline = None
        while not self._stop.is_set():
            if self.ap.is_station_connected():
                drop_deadline = None
            else:
                now = time.monotonic()
                if drop_deadline is None:
                    drop_deadline = now + threshold
                    log.info("station dropped, holding off AP for %ds", threshold)
                elif now >= drop_deadline:
                    log.info("station down for %ds", threshold)
                    self._set_state(STATE_STEADY)
                    return
            time.sleep(2)

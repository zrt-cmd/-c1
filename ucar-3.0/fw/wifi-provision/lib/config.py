"""Configuration loader for wifi-provision."""

import configparser
import logging
import os


class Config:
    DEFAULTS = {
        "general": {
            "wifi_interface": "wlan0",
            "ap_ssid_prefix": "XFLZ",
            "log_level": "INFO",
        },
        "timing": {
            "boot_grace_seconds": "30",
            "sta_retry_seconds": "15",
            "per_profile_attempt_seconds": "5",
            "disconnect_threshold_seconds": "10",
            "connect_attempt_timeout": "20",
            "scan_interval_seconds": "180",
            "scan_back_window_seconds": "10",
            "scan_back_idle_seconds": "60",
        },
        "web": {
            "bind_host": "0.0.0.0",
            "bind_port": "80",
        },
    }

    def __init__(self, path="/etc/xflz/wifi-provision.conf"):
        cp = configparser.RawConfigParser()
        cp.read_dict(self.DEFAULTS)
        if os.path.exists(path):
            cp.read(path, encoding="utf-8")

        self.wifi_interface = cp.get("general", "wifi_interface")
        self.ap_ssid_prefix = cp.get("general", "ap_ssid_prefix")
        self.log_level = cp.get("general", "log_level").upper()

        self.boot_grace_seconds = cp.getint("timing", "boot_grace_seconds")
        self.sta_retry_seconds = cp.getint("timing", "sta_retry_seconds")
        self.per_profile_attempt_seconds = cp.getint(
            "timing", "per_profile_attempt_seconds")
        self.disconnect_threshold_seconds = cp.getint(
            "timing", "disconnect_threshold_seconds")
        self.connect_attempt_timeout = cp.getint(
            "timing", "connect_attempt_timeout")
        self.scan_interval_seconds = cp.getint(
            "timing", "scan_interval_seconds")
        self.scan_back_window_seconds = cp.getint(
            "timing", "scan_back_window_seconds")
        self.scan_back_idle_seconds = cp.getint(
            "timing", "scan_back_idle_seconds")

        self.bind_host = cp.get("web", "bind_host")
        self.bind_port = cp.getint("web", "bind_port")

    def setup_logging(self):
        logging.basicConfig(
            level=getattr(logging, self.log_level, logging.INFO),
            format="%(asctime)s %(levelname)s [%(name)s] %(message)s",
        )

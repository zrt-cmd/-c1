"""Flask captive-portal web server for WiFi provisioning."""

import logging
import os
import threading
import time

from flask import Flask, render_template, request, redirect, jsonify

log = logging.getLogger("web")


def create_app(monitor, ap_manager, root_dir):
    template_dir = os.path.join(root_dir, "web", "templates")
    static_dir = os.path.join(root_dir, "web", "static")
    app = Flask("wifi_provision",
                template_folder=template_dir,
                static_folder=static_dir)

    # Stamped on every request so the monitor can tell whether somebody has the
    # config page open right now, and therefore should not have the AP pulled
    # out from under them by a scan-back cycle.
    app.last_request = 0.0

    @app.before_request
    def _touch_last_request():
        app.last_request = time.monotonic()

    @app.route("/", methods=["GET"])
    def index():
        return render_template(
            "index.html",
            ap_ssid=ap_manager.ap_ssid,
            last_error=monitor.get_last_error(),
        )

    @app.route("/connect", methods=["POST"])
    def connect():
        ssid = (request.form.get("ssid") or "").strip()
        password = request.form.get("password", "")
        if not ssid:
            return redirect("/")
        monitor.submit_credentials(ssid, password)
        return render_template("connecting.html", ssid=ssid)

    @app.route("/status.json")
    def status_json():
        return jsonify(monitor.get_status_snapshot())

    @app.route("/<path:_path>")
    def catch_all(_path):
        # Captive-portal probes from iOS/Android/Windows land here.
        return redirect("/")

    return app


class WebServer:
    def __init__(self, app, host, port):
        self.app = app
        self.host = host
        self.port = port
        self._srv = None
        self._thread = None

    def seconds_since_request(self):
        """Seconds since the last HTTP request, or None if there was none yet.

        Monotonic, so a wrong system clock cannot skew it.
        """
        last = self.app.last_request
        if not last:
            return None
        return time.monotonic() - last

    def start(self):
        if self._thread is not None:
            return
        from werkzeug.serving import make_server
        self._srv = make_server(self.host, self.port, self.app, threaded=True)
        self._thread = threading.Thread(target=self._srv.serve_forever,
                                        daemon=True, name="web")
        self._thread.start()
        log.info("web server up on %s:%d", self.host, self.port)

    def stop(self):
        if self._srv is None and self._thread is None:
            return
        if self._srv is not None:
            try:
                self._srv.shutdown()
            except Exception as e:
                log.warning("web shutdown: %s", e)
        if self._thread is not None:
            self._thread.join(timeout=5)
        self._srv = None
        self._thread = None
        log.info("web server down")

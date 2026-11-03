#!/usr/bin/env python3
"""
STM32 串口日志查看器 - 后端服务

浏览器访问 http://<host>:8090 即可实时查看 /dev/ttyACM0 上的日志。
默认与 fw/ 目录中固件的 debug UART 参数一致：/dev/ttyACM0 @ 115200。
"""

from __future__ import annotations

import argparse
import asyncio
import base64
import glob
import json
import logging
import subprocess
import sys
import threading
import time
from collections import deque
from pathlib import Path
from typing import Any

SCRIPT_DIR = Path(__file__).parent
STATIC_DIR = SCRIPT_DIR / "log_server" / "static"

DEFAULT_HOST = "0.0.0.0"
DEFAULT_PORT = 8090
DEFAULT_SERIAL = "/dev/ttyACM0"
DEFAULT_BAUD = 115200

# 保留最近 N 个数据块作为历史（新客户端接入时补发）
HISTORY_MAXLEN = 2000
# 串口一次最多读取的字节数
READ_CHUNK = 4096
# 串口读失败时的重试间隔（秒）
RECONNECT_INTERVAL = 2.0


class Color:
    GREEN = "\033[92m"
    YELLOW = "\033[93m"
    RED = "\033[91m"
    CYAN = "\033[96m"
    END = "\033[0m"
    BOLD = "\033[1m"


def info(msg: str) -> None:
    print(f"{Color.CYAN}ℹ{Color.END} {msg}")


def ok(msg: str) -> None:
    print(f"{Color.GREEN}✓{Color.END} {msg}")


def warn(msg: str) -> None:
    print(f"{Color.YELLOW}⚠{Color.END} {msg}")


def err(msg: str) -> None:
    print(f"{Color.RED}✗{Color.END} {msg}")


def ensure_deps() -> None:
    """按需从清华源安装 aiohttp / pyserial。"""
    missing = []
    try:
        import aiohttp  # noqa: F401
    except ImportError:
        missing.append("aiohttp")
    try:
        import serial  # noqa: F401
    except ImportError:
        missing.append("pyserial")

    if not missing:
        return

    warn(f"缺少依赖: {', '.join(missing)}，正在安装...")
    cmd = [
        sys.executable, "-m", "pip", "install",
        *missing,
        "-i", "https://pypi.tuna.tsinghua.edu.cn/simple",
    ]
    try:
        subprocess.run(cmd, check=True)
        ok("依赖安装完成")
    except subprocess.CalledProcessError as e:
        err(f"依赖安装失败: {e}")
        sys.exit(1)


ensure_deps()

import serial  # noqa: E402
import serial.serialutil  # noqa: E402
from aiohttp import WSMsgType, web  # noqa: E402


def list_serial_ports() -> list[str]:
    ports = sorted(set(glob.glob("/dev/ttyACM*") + glob.glob("/dev/ttyUSB*")))
    return ports


class SerialReader:
    """在后台线程里读串口，把每一块数据交给 asyncio 侧的回调。"""

    def __init__(
        self,
        loop: asyncio.AbstractEventLoop,
        on_chunk,
        on_status,
        port: str,
        baud: int,
    ) -> None:
        self._loop = loop
        self._on_chunk = on_chunk
        self._on_status = on_status
        self._port = port
        self._baud = baud
        self._ser: serial.Serial | None = None
        self._stop = threading.Event()
        self._lock = threading.Lock()
        self._thread: threading.Thread | None = None

    @property
    def port(self) -> str:
        return self._port

    @property
    def baud(self) -> int:
        return self._baud

    @property
    def is_open(self) -> bool:
        return self._ser is not None and self._ser.is_open

    def start(self) -> None:
        self._thread = threading.Thread(target=self._run, daemon=True, name="serial-reader")
        self._thread.start()

    def stop(self) -> None:
        self._stop.set()
        with self._lock:
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None
        if self._thread is not None:
            self._thread.join(timeout=2.0)

    def reconfigure(self, port: str, baud: int) -> None:
        """线程安全地切换串口/波特率——关闭旧句柄，主循环会自动重连。"""
        with self._lock:
            self._port = port
            self._baud = baud
            if self._ser is not None:
                try:
                    self._ser.close()
                except Exception:
                    pass
                self._ser = None

    def _post_status(self, connected: bool, error: str | None) -> None:
        payload = {
            "type": "status",
            "connected": connected,
            "port": self._port,
            "baud": self._baud,
            "error": error,
        }
        self._loop.call_soon_threadsafe(self._on_status, payload)

    def _post_chunk(self, data: bytes) -> None:
        payload = {
            "type": "data",
            "ts": int(time.time() * 1000),
            "b64": base64.b64encode(data).decode("ascii"),
        }
        self._loop.call_soon_threadsafe(self._on_chunk, payload)

    def _run(self) -> None:
        while not self._stop.is_set():
            with self._lock:
                port, baud = self._port, self._baud

            try:
                ser = serial.Serial(port=port, baudrate=baud, timeout=0.1)
            except (serial.SerialException, OSError) as e:
                self._post_status(False, f"{type(e).__name__}: {e}")
                self._wait(RECONNECT_INTERVAL)
                continue

            with self._lock:
                self._ser = ser
            self._post_status(True, None)

            try:
                while not self._stop.is_set():
                    with self._lock:
                        if self._port != port or self._baud != baud:
                            break
                    try:
                        if not ser.is_open:
                            break
                        data = ser.read(READ_CHUNK)
                    except (serial.SerialException, OSError, TypeError) as e:
                        self._post_status(False, f"read error: {e}")
                        break
                    if data:
                        self._post_chunk(data)
                    else:
                        # 超时空读，让出 CPU 检查停止条件
                        continue
            finally:
                with self._lock:
                    if self._ser is ser:
                        self._ser = None
                try:
                    ser.close()
                except Exception:
                    pass
                if not self._stop.is_set():
                    self._post_status(False, "disconnected")
                    self._wait(RECONNECT_INTERVAL)

    def _wait(self, seconds: float) -> None:
        end = time.time() + seconds
        while not self._stop.is_set() and time.time() < end:
            time.sleep(0.1)


class LogHub:
    """管理串口读取器 + WebSocket 客户端广播 + 环形缓冲。"""

    def __init__(self, port: str, baud: int) -> None:
        self._loop = asyncio.get_running_loop()
        self._clients: set[web.WebSocketResponse] = set()
        self._history: deque[dict[str, Any]] = deque(maxlen=HISTORY_MAXLEN)
        self._status: dict[str, Any] = {
            "type": "status", "connected": False,
            "port": port, "baud": baud, "error": None,
        }
        self._reader = SerialReader(
            loop=self._loop,
            on_chunk=self._handle_chunk,
            on_status=self._handle_status,
            port=port, baud=baud,
        )

    def start(self) -> None:
        self._reader.start()

    def stop(self) -> None:
        self._reader.stop()

    @property
    def status(self) -> dict[str, Any]:
        return dict(self._status)

    def reconfigure(self, port: str, baud: int) -> None:
        self._history.clear()
        self._status = {
            "type": "status", "connected": False,
            "port": port, "baud": baud, "error": None,
        }
        self._reader.reconfigure(port, baud)

    async def register(self, ws: web.WebSocketResponse) -> None:
        self._clients.add(ws)
        await ws.send_json(self._status)
        if self._history:
            await ws.send_json({"type": "history", "chunks": list(self._history)})

    async def unregister(self, ws: web.WebSocketResponse) -> None:
        self._clients.discard(ws)

    def _handle_chunk(self, payload: dict[str, Any]) -> None:
        self._history.append(payload)
        self._broadcast(payload)

    def _handle_status(self, payload: dict[str, Any]) -> None:
        self._status = payload
        self._broadcast(payload)

    def _broadcast(self, payload: dict[str, Any]) -> None:
        if not self._clients:
            return
        message = json.dumps(payload)
        for ws in list(self._clients):
            if ws.closed:
                self._clients.discard(ws)
                continue
            asyncio.create_task(self._safe_send(ws, message))

    async def _safe_send(self, ws: web.WebSocketResponse, message: str) -> None:
        try:
            await ws.send_str(message)
        except (ConnectionResetError, RuntimeError):
            self._clients.discard(ws)


@web.middleware
async def no_cache(request: web.Request, handler) -> web.StreamResponse:
    """首页与静态资源强制浏览器回源校验（有 ETag，未改动时走 304，开销可忽略）。

    不设这条时响应里没有 Cache-Control，浏览器会做启发式缓存、不再回问服务器，
    改完 app.js / style.css 后用户不硬刷就永远看不到新版本。
    """
    resp = await handler(request)
    if request.path == "/" or request.path.startswith("/static/"):
        resp.headers["Cache-Control"] = "no-cache"
    return resp


def build_app(hub: LogHub) -> web.Application:
    app = web.Application(middlewares=[no_cache])

    async def index(_request: web.Request) -> web.StreamResponse:
        return web.FileResponse(STATIC_DIR / "index.html")

    async def api_ports(_request: web.Request) -> web.Response:
        return web.json_response({
            "ports": list_serial_ports(),
            "current": hub.status.get("port"),
        })

    async def api_status(_request: web.Request) -> web.Response:
        return web.json_response(hub.status)

    async def ws_handler(request: web.Request) -> web.WebSocketResponse:
        ws = web.WebSocketResponse(heartbeat=20.0, max_msg_size=1 << 20)
        await ws.prepare(request)
        await hub.register(ws)
        try:
            async for msg in ws:
                if msg.type == WSMsgType.TEXT:
                    try:
                        data = json.loads(msg.data)
                    except json.JSONDecodeError:
                        continue
                    if data.get("type") == "config":
                        port = str(data.get("port") or "").strip()
                        baud = int(data.get("baud") or 0)
                        if port and baud > 0:
                            hub.reconfigure(port, baud)
                elif msg.type == WSMsgType.ERROR:
                    break
        finally:
            await hub.unregister(ws)
        return ws

    app.router.add_get("/", index)
    app.router.add_get("/api/ports", api_ports)
    app.router.add_get("/api/status", api_status)
    app.router.add_get("/ws", ws_handler)
    app.router.add_static("/static/", STATIC_DIR, show_index=False)
    return app


def parse_args() -> argparse.Namespace:
    p = argparse.ArgumentParser(description="STM32 串口日志查看器 (Web)")
    p.add_argument("--host", default=DEFAULT_HOST, help=f"监听地址 (默认 {DEFAULT_HOST})")
    p.add_argument("--port", type=int, default=DEFAULT_PORT, help=f"HTTP 端口 (默认 {DEFAULT_PORT})")
    p.add_argument("--serial", default=DEFAULT_SERIAL, help=f"串口设备 (默认 {DEFAULT_SERIAL})")
    p.add_argument("--baud", type=int, default=DEFAULT_BAUD, help=f"波特率 (默认 {DEFAULT_BAUD})")
    return p.parse_args()


def main() -> int:
    args = parse_args()

    if not STATIC_DIR.exists():
        err(f"静态资源目录不存在: {STATIC_DIR}")
        return 1

    logging.basicConfig(level=logging.WARNING, format="%(asctime)s [%(levelname)s] %(message)s")

    info(f"启动串口日志查看器")
    info(f"  监听地址: http://{args.host}:{args.port}")
    info(f"  默认串口: {args.serial} @ {args.baud}")

    async def run() -> None:
        hub = LogHub(port=args.serial, baud=args.baud)
        hub.start()
        app = build_app(hub)

        runner = web.AppRunner(app)
        await runner.setup()
        site = web.TCPSite(runner, args.host, args.port)
        await site.start()
        ok(f"服务已就绪，浏览器打开 http://<板子IP>:{args.port}")

        stop_event = asyncio.Event()
        try:
            await stop_event.wait()
        finally:
            hub.stop()
            await runner.cleanup()

    try:
        asyncio.run(run())
    except KeyboardInterrupt:
        warn("用户中断，正在退出")
        return 130
    return 0


if __name__ == "__main__":
    sys.exit(main())

"""MJPEG 视频流服务器：统一使用 mjpeg_streamer 库（2026-08-06 用户决策）。

规则（所有任务）：启动时先 require() 检测 mjpeg_streamer；缺失 → 打印安装指引
（清华镜像）并 exit 2，不再内置 stdlib 实现。
安装：pip install -i https://pypi.tuna.tsinghua.edu.cn/simple mjpeg_streamer
接口：MjpegServer(host, port, stream_name, quality) → start()/set_frame(bgr)/port/stop()
流地址：http://<ip>:<port>/<stream_name>.mjpg（库按 name 路由；name 无 .mjpg 时补全）
"""
from __future__ import annotations

import numpy as np

INSTALL_HINT = (
    "mjpeg_streamer 未安装。请先安装（清华镜像）：\n"
    "    pip install -i https://pypi.tuna.tsinghua.edu.cn/simple mjpeg_streamer"
)


def require() -> None:
    """启动前检测：mjpeg_streamer 缺失 → 提示安装并退出（exit 2）。"""
    try:
        import mjpeg_streamer  # noqa: F401
    except ImportError:
        print(INSTALL_HINT)
        raise SystemExit(2)


class MjpegServer:
    """mjpeg_streamer 封装（vendor main.py 同款用法，接口对调用方无感）。

    注意：
    - Stream size=None：不强制缩放，保持原始分辨率；
    - stop()：库内部抛 aiohttp GracefulExit（daemon 线程随进程退出），
      wrapper 吞掉避免调用方崩溃（板上曾 Aborted core dumped）；
    - port：库不解析 port=0（返回配置值）；测试请用具体空闲端口。
    """

    def __init__(self, host: str = "0.0.0.0", port: int = 8080,
                 stream_name: str = "qrcode", quality: int = 80):
        from mjpeg_streamer import MjpegServer as _LibServer, Stream

        name = stream_name if stream_name.endswith(".mjpg") else stream_name + ".mjpg"
        self._stream = Stream(name, size=None, quality=quality, fps=30)
        self._server = _LibServer(host, port)
        self._server.add_stream(self._stream)

    def start(self) -> "MjpegServer":
        self._server.start()
        return self

    def set_frame(self, bgr: np.ndarray) -> None:
        self._stream.set_frame(bgr)

    @property
    def port(self) -> int:
        return self._server._port  # noqa: SLF001 — 库未公开实际端口（port=0 时无解）

    def stop(self) -> None:
        try:
            self._server.stop()
        except BaseException:  # GracefulExit（BaseException）→ 吞掉，线程随进程退出
            pass

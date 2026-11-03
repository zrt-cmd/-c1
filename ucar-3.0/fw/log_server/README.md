# STM32 串口日志查看器（Web）

浏览器打开 `http://<板子IP>:8090` 就能实时看 STM32 debug UART 输出。

默认串口 `/dev/ttyACM0` @ 115200，与 `fw/` 目录下固件的 debug UART 参数一致。
和 `car1_base_driver` 用的 `/dev/ttyUSB0` 是两根独立串口，**可以同时运行**。

## 目录

```
fw/
├── log_server.py            # 启动入口
└── log_server/
    ├── README.md            # 本文档
    └── static/
        ├── index.html
        ├── app.js
        └── style.css
```

## 启动

```bash
cd /home/iflytek/ros2_ws/fw
python3 log_server.py
```

首次运行会自动从清华镜像源安装 `aiohttp` 和 `pyserial`。

自定义参数：

```bash
python3 log_server.py --host 0.0.0.0 --port 8090 \
                      --serial /dev/ttyACM0 --baud 115200
```

## 功能

| 类别 | 功能 |
|------|------|
| 视图 | 文本模式（UTF-8 自动解码）/ Hex 模式（16 字节/行，带地址列和 ASCII 侧栏）|
| 过滤 | 正则过滤 + 命中行高亮 + 反选 |
| 着色 | 按行首标签上色：`[ERR]/[FATAL]` 红、`[WRN]` 黄、`[INF]/[LOG]` 正常、`[DBG]/[VRB]` 灰、`[OK]` 绿；无标签续行暗灰斜体 |
| 时间戳 | 主机时间毫秒精度，可开关 |
| 滚动 | 自动滚动开关，用户上滚自动暂停，滚到底恢复 |
| 端口 | 下拉选择 `/dev/ttyUSB* /dev/ttyACM*`，随时切换 |
| 波特率 | 9600 ~ 1000000 常用档 |
| 广播 | 一个服务多个浏览器同时看，同一串口不重复占用 |
| 缓存 | 最近 2000 数据块，新客户端接入自动补历史 |
| 断线 | 浏览器↔后端、后端↔串口 双向自动重连（拔插 USB 不用重启）|
| 导出 | 一键下载 `.log`（含时间戳）|
| 持久化 | 端口/波特率/过滤条件保存在浏览器 localStorage |

## HTTP 接口

| 路径 | 方法 | 说明 |
|------|------|------|
| `/` | GET | UI 页面 |
| `/static/*` | GET | 静态资源 |
| `/api/ports` | GET | 当前系统可用串口列表 |
| `/api/status` | GET | 当前串口连接状态（JSON）|
| `/ws` | WebSocket | 数据流；发 `{"type":"config","port":"...","baud":115200}` 可切换 |

## 常见问题

**端口显示"未连接" / `Errno 13 Permission denied`**

把当前用户加入 `dialout` 组（或改一次权限）：

```bash
sudo usermod -aG dialout $USER
# 重新登录后生效
```

**端口正在被占用 / `Errno 16 Device or resource busy`**

`/dev/ttyACM0` 只允许一个进程独占。如果你的固件在同一个物理接口上既跑业务又跑调试，请确认没有别的程序（`minicom` / `screen` / 另一个 log_server 实例）在占用。

**页面能打开但一直没数据**

- 状态栏"字节"是否在涨？如果为 0，说明串口没数据来，检查固件里的 debug 打印是否使能。
- 波特率是否匹配？在下拉里切换后点"应用"。
- 如果窗口里出现"锟斤拷"这种乱码，是波特率错了。

**想在启动时开机自启**

写一个 systemd user unit（简版）：

```ini
# ~/.config/systemd/user/stm32-log-server.service
[Unit]
Description=STM32 serial log web viewer
After=network.target

[Service]
ExecStart=/usr/bin/python3 /home/iflytek/ros2_ws/fw/log_server.py
Restart=on-failure

[Install]
WantedBy=default.target
```

```bash
systemctl --user enable --now stm32-log-server
```

#!/bin/bash
# Start a headless XFCE desktop and expose it through noVNC (browser).
# Security: VNC password + listeners bound to 127.0.0.1 only (no LAN exposure).
export DISPLAY=:1
unset WAYLAND_DISPLAY
export XDG_SESSION_TYPE=x11
VNC_PASS="${VNC_PASS:?Set VNC_PASS before starting noVNC}"

pkill -f "Xvfb :1" 2>/dev/null
pkill -f "x11vnc -display :1" 2>/dev/null
pkill -f "websockify.*6080" 2>/dev/null
sleep 1

mkdir -p ~/.vnc
x11vnc -storepasswd "$VNC_PASS" ~/.vnc/passwd > /dev/null 2>&1

setsid nohup Xvfb :1 -screen 0 1280x800x24 +extension GLX +render -noreset > /tmp/xvfb.log 2>&1 < /dev/null &
sleep 2

setsid nohup dbus-launch --exit-with-session startxfce4 > /tmp/xfce.log 2>&1 < /dev/null &
sleep 3

setsid nohup x11vnc -display :1 -rfbauth ~/.vnc/passwd -forever -shared -localhost -rfbport 5901 > /tmp/x11vnc.log 2>&1 < /dev/null &
sleep 1

setsid nohup websockify --web=/usr/share/novnc 127.0.0.1:6080 localhost:5901 > /tmp/websockify.log 2>&1 < /dev/null &
sleep 1

echo "URL: http://localhost:6080/vnc.html"
echo "VNC password: $VNC_PASS"

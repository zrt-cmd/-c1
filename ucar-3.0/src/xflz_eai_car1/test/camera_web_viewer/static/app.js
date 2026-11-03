// app.js - WebSocket handling, dual 2D canvas, tracked depth click, 3D mode detection

(function () {
  'use strict';

  const socket = io();

  let currentTab = '2d';
  let trackedPoint = null;  // {x, y} sticky tracked pixel

  const webTimestamps = [];

  // DOM
  const rgbCanvas = document.getElementById('canvas-rgb');
  const rgbCtx = rgbCanvas.getContext('2d');
  const depthCanvas = document.getElementById('canvas-depth');
  const depthCtx = depthCanvas.getContext('2d');

  const connDot = document.getElementById('conn-dot');
  const connText = document.getElementById('conn-text');
  const overlayRgb = document.getElementById('overlay-rgb');
  const overlayDepth = document.getElementById('overlay-depth');
  const overlay3d = document.getElementById('overlay-3d');
  const pickInfo = document.getElementById('pick-info');

  // Offscreen images
  const rgbImg = new Image();
  const depthImg = new Image();
  let rgbReady = false, depthReady = false;
  let latestTrackedDepth = null;

  rgbImg.onload = () => { rgbReady = true; drawRgb(); };
  depthImg.onload = () => { depthReady = true; drawDepth(); };

  // ---- Tabs (2D / 3D) ----
  document.querySelectorAll('.tab').forEach(btn => {
    btn.addEventListener('click', () => {
      document.querySelectorAll('.tab').forEach(b => b.classList.remove('active'));
      btn.classList.add('active');
      currentTab = btn.dataset.tab;
      document.getElementById('view-2d').classList.toggle('active', currentTab === '2d');
      document.getElementById('view-3d').classList.toggle('active', currentTab === '3d');

      // Enable/disable pointcloud push based on tab
      socket.emit('enable_pointcloud', { enabled: currentTab === '3d' });

      if (currentTab === '3d' && window.PointCloudViewer) {
        window.PointCloudViewer.onShow();
      } else {
        drawRgb(); drawDepth();
      }
    });
  });

  // ---- Click on EITHER canvas -> set tracked point ----
  // RGB and Depth are the same resolution (640x480) and share pixel coordinates, so a
  // click on either maps to the same pixel (x, y). The crosshair + XY is drawn on BOTH
  // canvases; depth value is queried from the depth frame and shown on both.
  function canvasClickToPixel(canvas, e) {
    const rect = canvas.getBoundingClientRect();
    const sx = canvas.width / rect.width;
    const sy = canvas.height / rect.height;
    const px = Math.round((e.clientX - rect.left) * sx);
    const py = Math.round((e.clientY - rect.top) * sy);
    return { x: px, y: py };
  }

  function setTrackedFromClick(canvas, e) {
    const p = canvasClickToPixel(canvas, e);
    trackedPoint = p;
    socket.emit('set_tracked_point', { x: p.x, y: p.y });
    // Redraw both so the marker appears immediately, before the next frame arrives.
    drawRgb();
    drawDepth();
  }

  function clearTracked() {
    trackedPoint = null;
    latestTrackedDepth = null;
    socket.emit('set_tracked_point', { x: null, y: null });
    pickInfo.innerHTML = '在 RGB 或 Depth 画面点击查询坐标';
    drawRgb();
    drawDepth();
  }

  rgbCanvas.addEventListener('click', (e) => setTrackedFromClick(rgbCanvas, e));
  depthCanvas.addEventListener('click', (e) => setTrackedFromClick(depthCanvas, e));

  // Clear tracked point on right-click (either canvas)
  rgbCanvas.addEventListener('contextmenu', (e) => { e.preventDefault(); clearTracked(); });
  depthCanvas.addEventListener('contextmenu', (e) => { e.preventDefault(); clearTracked(); });

  // ---- Drawing ----
  // Draw a crosshair + info box at the tracked pixel on a given context.
  // `showDepth`: true on the Depth canvas (label shows X/Y + depth), false on the RGB
  // canvas (label shows X/Y only). Sharing the same pixel + XY on both lets the operator
  // compare positions and judge whether the depth reading matches the RGB object.
  function drawMarker(ctx, canvasW, canvasH, showDepth) {
    if (!trackedPoint) return;
    const { x, y } = trackedPoint;

    const depthValid = (latestTrackedDepth !== null && latestTrackedDepth !== undefined);
    // RGB marker is always the accent color (no depth dependency); Depth marker turns
    // red when the depth sample is invalid.
    const accent = (showDepth && !depthValid) ? '#fa5252' : '#ffd43b';

    // Crosshair
    ctx.strokeStyle = accent;
    ctx.lineWidth = 2;
    ctx.beginPath();
    ctx.moveTo(x - 12, y); ctx.lineTo(x + 12, y);
    ctx.moveTo(x, y - 12); ctx.lineTo(x, y + 12);
    ctx.stroke();

    // Labels: XY always; depth only on the depth canvas.
    const xyLabel = `X:${x} Y:${y}`;
    const depthLabel = showDepth
      ? (depthValid ? ('深度: ' + latestTrackedDepth.toFixed(3) + 'm') : '深度: 无效')
      : null;

    ctx.font = '14px monospace';
    const tw = depthLabel
      ? Math.max(ctx.measureText(depthLabel).width, ctx.measureText(xyLabel).width)
      : ctx.measureText(xyLabel).width;
    const boxH = depthLabel ? 36 : 20;
    let tx = x + 16, ty = y - 20;
    if (tx + tw > canvasW) tx = x - tw - 16;
    if (ty < 32) ty = y + 32;

    ctx.fillStyle = 'rgba(0,0,0,0.75)';
    ctx.fillRect(tx - 4, ty - 16, tw + 8, boxH);

    ctx.fillStyle = accent;
    ctx.fillText(xyLabel, tx, ty);
    if (depthLabel) ctx.fillText(depthLabel, tx, ty + 16);
  }

  function drawRgb() {
    if (!rgbReady) return;
    if (rgbCanvas.width !== rgbImg.naturalWidth) {
      rgbCanvas.width = rgbImg.naturalWidth;
      rgbCanvas.height = rgbImg.naturalHeight;
    }
    rgbCtx.drawImage(rgbImg, 0, 0, rgbCanvas.width, rgbCanvas.height);
    drawMarker(rgbCtx, rgbCanvas.width, rgbCanvas.height, false);
  }

  function drawDepth() {
    if (!depthReady) return;
    if (depthCanvas.width !== depthImg.naturalWidth) {
      depthCanvas.width = depthImg.naturalWidth;
      depthCanvas.height = depthImg.naturalHeight;
    }
    depthCtx.drawImage(depthImg, 0, 0, depthCanvas.width, depthCanvas.height);
    drawMarker(depthCtx, depthCanvas.width, depthCanvas.height, true);
  }

  // ---- Socket events ----
  socket.on('connect', () => {
    connDot.classList.add('connected');
    connText.textContent = '已连接';
    // Re-send state on reconnect
    socket.emit('enable_pointcloud', { enabled: currentTab === '3d' });
    if (trackedPoint) {
      socket.emit('set_tracked_point', trackedPoint);
    }
  });

  socket.on('disconnect', () => {
    connDot.classList.remove('connected');
    connText.textContent = '已断开';
  });

  socket.on('frame', (msg) => {
    const now = performance.now();
    webTimestamps.push(now);
    while (webTimestamps.length > 50) webTimestamps.shift();

    if (msg.rgb) rgbImg.src = 'data:image/jpeg;base64,' + msg.rgb;
    if (msg.depth) depthImg.src = 'data:image/jpeg;base64,' + msg.depth;

    // Update tracked depth if present
    if (msg.tracked_xy) {
      if (msg.tracked_depth !== null && msg.tracked_depth !== undefined) {
        latestTrackedDepth = msg.tracked_depth;
        pickInfo.innerHTML = `X: ${msg.tracked_xy.x}, Y: ${msg.tracked_xy.y}<br>深度: ${msg.tracked_depth.toFixed(3)} m`;
      } else {
        latestTrackedDepth = null;
        pickInfo.innerHTML = `X: ${msg.tracked_xy.x}, Y: ${msg.tracked_xy.y}<br>深度: 无效`;
      }
    }

    // FPS sidebar
    document.getElementById('fps-rgb').textContent = msg.fps.rgb.toFixed(1);
    document.getElementById('fps-depth').textContent = msg.fps.depth.toFixed(1);
    document.getElementById('fps-pc').textContent = msg.fps.pointcloud.toFixed(1);
    document.getElementById('fps-web').textContent = webFps().toFixed(1);

    // Overlay FPS
    overlayRgb.textContent = `RGB: ${msg.fps.rgb.toFixed(1)} FPS`;
    overlayDepth.textContent = `Depth: ${msg.fps.depth.toFixed(1)} FPS`;
    overlay3d.textContent = `点云: ${msg.fps.pointcloud.toFixed(1)} FPS | Web: ${webFps().toFixed(1)} FPS`;

    if (msg.pointcloud && window.PointCloudViewer) {
      window.PointCloudViewer.update(msg.pointcloud);
    }
  });

  function webFps() {
    if (webTimestamps.length < 2) return 0;
    const span = (webTimestamps[webTimestamps.length - 1] - webTimestamps[0]) / 1000;
    return span > 0 ? (webTimestamps.length - 1) / span : 0;
  }
})();

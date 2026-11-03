"use strict";

const MAX_LINES = 5000;
const RECONNECT_DELAY_MS = 1500;

const el = {
  portSelect: document.getElementById("port-select"),
  portRefresh: document.getElementById("port-refresh"),
  baudSelect: document.getElementById("baud-select"),
  apply: document.getElementById("apply-config"),
  viewMode: document.getElementById("view-mode"),
  showTs: document.getElementById("show-ts"),
  autoscroll: document.getElementById("autoscroll"),
  filter: document.getElementById("filter-input"),
  filterInvert: document.getElementById("filter-invert"),
  clear: document.getElementById("clear-btn"),
  download: document.getElementById("download-btn"),
  logView: document.getElementById("log-view"),
  connDot: document.getElementById("conn-indicator"),
  connText: document.getElementById("conn-text"),
  statPort: document.getElementById("stat-port"),
  statBaud: document.getElementById("stat-baud"),
  statBytes: document.getElementById("stat-bytes"),
  statLines: document.getElementById("stat-lines"),
  statDropped: document.getElementById("stat-dropped"),
  statError: document.getElementById("stat-error"),
};

const state = {
  ws: null,
  reconnectTimer: null,
  textPending: "",           // 上一段未完成的文本行残余
  textLines: [],             // {ts, text}
  hexLines: [],              // {ts, offset, bytes: Uint8Array}
  hexOffset: 0,              // 累计字节偏移（用于 hex 视图的地址列）
  totalBytes: 0,
  droppedLines: 0,
  filterRe: null,
  filterInvert: false,
};

/* ---------- 存储 ---------- */
function loadPrefs() {
  try {
    const raw = localStorage.getItem("logviewer:prefs");
    if (!raw) return {};
    return JSON.parse(raw);
  } catch {
    return {};
  }
}
function savePrefs() {
  const prefs = {
    port: el.portSelect.value,
    baud: el.baudSelect.value,
    viewMode: el.viewMode.value,
    showTs: el.showTs.checked,
    autoscroll: el.autoscroll.checked,
    filter: el.filter.value,
    filterInvert: el.filterInvert.checked,
  };
  localStorage.setItem("logviewer:prefs", JSON.stringify(prefs));
}

/* ---------- 工具 ---------- */
function fmtTs(ms) {
  const d = new Date(ms);
  const pad = (n, w = 2) => String(n).padStart(w, "0");
  return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}.${pad(d.getMilliseconds(), 3)}`;
}

function b64ToBytes(b64) {
  const bin = atob(b64);
  const out = new Uint8Array(bin.length);
  for (let i = 0; i < bin.length; i++) out[i] = bin.charCodeAt(i);
  return out;
}

function bytesToText(bytes) {
  try {
    return new TextDecoder("utf-8", { fatal: false }).decode(bytes);
  } catch {
    let s = "";
    for (const b of bytes) s += String.fromCharCode(b);
    return s;
  }
}

function hexDumpRow(offset, bytes) {
  const hexParts = [];
  const asciiParts = [];
  for (let i = 0; i < 16; i++) {
    if (i < bytes.length) {
      const b = bytes[i];
      hexParts.push(b.toString(16).padStart(2, "0"));
      asciiParts.push(b >= 0x20 && b < 0x7f ? String.fromCharCode(b) : ".");
    } else {
      hexParts.push("  ");
      asciiParts.push(" ");
    }
    if (i === 7) hexParts.push("");
  }
  const addr = offset.toString(16).padStart(8, "0");
  return `${addr}  ${hexParts.join(" ")}  |${asciiParts.join("")}|`;
}

/* ---------- 数据接收 ---------- */
function ingestChunk(chunk) {
  const bytes = b64ToBytes(chunk.b64);
  if (bytes.length === 0) return;
  state.totalBytes += bytes.length;

  // 文本视图：按 CRLF / LF / CR 切行
  feedText(bytesToText(bytes), chunk.ts);

  // Hex 视图：按 16 字节切行
  for (let i = 0; i < bytes.length; i += 16) {
    const slice = bytes.subarray(i, Math.min(i + 16, bytes.length));
    pushHexLine({ ts: chunk.ts, offset: state.hexOffset, bytes: slice });
    state.hexOffset += slice.length;
  }

  updateStats();
}

/* 把新到的一段文本按行切分。行尾支持 \r\n / \n / \r 三种；
   末尾没遇到行尾的半行留在 textPending，等下一块数据补齐，
   这样串口在任意字节处断开都不会把一行切错或串行。 */
function feedText(text, ts) {
  let buf = state.textPending + text;
  let tail = "";
  if (buf.endsWith("\r")) {
    // 结尾的 \r 可能是被拆开的 \r\n 的前半，先扣住不切
    tail = "\r";
    buf = buf.slice(0, -1);
  }
  const parts = buf.split(/\r\n|\n|\r/);
  state.textPending = parts.pop() + tail;
  for (const line of parts) pushTextLine({ ts, text: line });
}

function pushTextLine(line) {
  state.textLines.push(line);
  if (state.textLines.length > MAX_LINES) {
    state.textLines.splice(0, state.textLines.length - MAX_LINES);
    state.droppedLines++;
  }
  if (el.viewMode.value === "text") {
    appendLineDom(renderTextLine(line));
    trimDom();
    scrollIfNeeded();
  }
}

function pushHexLine(line) {
  state.hexLines.push(line);
  if (state.hexLines.length > MAX_LINES) {
    state.hexLines.splice(0, state.hexLines.length - MAX_LINES);
    state.droppedLines++;
  }
  if (el.viewMode.value === "hex") {
    appendLineDom(renderHexLine(line));
    trimDom();
    scrollIfNeeded();
  }
}

/* ---------- 渲染 ---------- */
/* 固件日志行一律以 [TAG] 开头：INF / DBG / VRB / WRN / ERR / FATAL / OK / LOG。
   只认行首标签，不再扫描正文——否则 [HEALTH] 行里的 err=0 / fail=0 / ok= 会被误判成错误级别。 */
const LEVEL_BY_TAG = {
  ERR: "lvl-error",
  FATAL: "lvl-error",
  WRN: "lvl-warn",
  WARN: "lvl-warn",
  INF: "lvl-info",
  INFO: "lvl-info",
  LOG: "lvl-info",
  DBG: "lvl-debug",
  DEBUG: "lvl-debug",
  VRB: "lvl-debug",
  TRACE: "lvl-debug",
  OK: "lvl-ok",
  SUCCESS: "lvl-ok",
};

function classifyLevel(text) {
  const m = /^\s*\[([A-Za-z]+)\]/.exec(text);
  if (m) return LEVEL_BY_TAG[m[1].toUpperCase()] || "";
  return text ? "lvl-frag" : "";   // 没有行首标签的续行/半行
}

function renderTextLine(line) {
  const div = document.createElement("span");
  div.className = "log-line";
  const lvl = classifyLevel(line.text);
  if (lvl) div.classList.add(lvl);

  if (el.showTs.checked) {
    const ts = document.createElement("span");
    ts.className = "ts";
    ts.textContent = fmtTs(line.ts);
    div.appendChild(ts);
  }

  applyFilterToLine(div, line.text);
  div.appendChild(document.createTextNode("\n"));
  return div;
}

function renderHexLine(line) {
  const div = document.createElement("span");
  div.className = "log-line lvl-debug";
  if (el.showTs.checked) {
    const ts = document.createElement("span");
    ts.className = "ts";
    ts.textContent = fmtTs(line.ts);
    div.appendChild(ts);
  }
  const body = hexDumpRow(line.offset, line.bytes);
  applyFilterToLine(div, body);
  div.appendChild(document.createTextNode("\n"));
  return div;
}

function applyFilterToLine(node, text) {
  const re = state.filterRe;
  if (!re) {
    node.appendChild(document.createTextNode(text));
    return;
  }
  const matched = re.test(text);
  const shouldShow = state.filterInvert ? !matched : matched;
  if (!shouldShow) {
    node.style.display = "none";
    node.appendChild(document.createTextNode(text));
    return;
  }
  if (matched && !state.filterInvert) {
    node.classList.add("hit");
    // 高亮：分段插入，避免 innerHTML 注入
    let lastIdx = 0;
    re.lastIndex = 0;
    const globalRe = new RegExp(re.source, re.flags.includes("g") ? re.flags : re.flags + "g");
    let m;
    while ((m = globalRe.exec(text)) !== null) {
      if (m.index > lastIdx) node.appendChild(document.createTextNode(text.slice(lastIdx, m.index)));
      const mark = document.createElement("mark");
      mark.textContent = m[0];
      node.appendChild(mark);
      lastIdx = m.index + m[0].length;
      if (m[0].length === 0) globalRe.lastIndex++;   // 防死循环
    }
    if (lastIdx < text.length) node.appendChild(document.createTextNode(text.slice(lastIdx)));
  } else {
    node.appendChild(document.createTextNode(text));
  }
}

function appendLineDom(node) {
  el.logView.appendChild(node);
}

function trimDom() {
  const max = MAX_LINES;
  while (el.logView.childNodes.length > max) {
    el.logView.removeChild(el.logView.firstChild);
  }
}

function scrollIfNeeded() {
  if (el.autoscroll.checked) {
    el.logView.scrollTop = el.logView.scrollHeight;
  }
}

function rerenderAll() {
  el.logView.textContent = "";
  const lines = el.viewMode.value === "hex" ? state.hexLines : state.textLines;
  const frag = document.createDocumentFragment();
  for (const line of lines) {
    frag.appendChild(el.viewMode.value === "hex" ? renderHexLine(line) : renderTextLine(line));
  }
  el.logView.appendChild(frag);
  scrollIfNeeded();
}

/* ---------- 统计 & 状态栏 ---------- */
function updateStats() {
  el.statBytes.textContent = state.totalBytes.toLocaleString();
  const count = el.viewMode.value === "hex" ? state.hexLines.length : state.textLines.length;
  el.statLines.textContent = count.toLocaleString();
  el.statDropped.textContent = state.droppedLines.toLocaleString();
}

function setStatus(status) {
  el.statPort.textContent = status.port || "-";
  el.statBaud.textContent = status.baud || "-";
  if (status.connected) {
    el.connDot.className = "dot dot-on";
    el.connText.textContent = "已连接";
    el.statError.textContent = "";
  } else {
    el.connDot.className = "dot dot-err";
    el.connText.textContent = "串口断开";
    el.statError.textContent = status.error || "";
  }
}

function pushSystemLine(text) {
  const ts = Date.now();
  const div = document.createElement("span");
  div.className = "log-line system";
  if (el.showTs.checked) {
    const t = document.createElement("span");
    t.className = "ts";
    t.textContent = fmtTs(ts);
    div.appendChild(t);
  }
  div.appendChild(document.createTextNode(`--- ${text} ---`));
  div.appendChild(document.createTextNode("\n"));
  el.logView.appendChild(div);
  scrollIfNeeded();
}

/* ---------- WebSocket ---------- */
function connectWs() {
  clearTimeout(state.reconnectTimer);
  const proto = location.protocol === "https:" ? "wss:" : "ws:";
  const url = `${proto}//${location.host}/ws`;
  const ws = new WebSocket(url);
  state.ws = ws;
  el.connDot.className = "dot dot-off";
  el.connText.textContent = "连接中...";

  ws.onopen = () => {
    pushSystemLine("已连接到日志服务");
  };
  ws.onmessage = (evt) => {
    let msg;
    try { msg = JSON.parse(evt.data); } catch { return; }
    if (msg.type === "status") {
      setStatus(msg);
    } else if (msg.type === "data") {
      ingestChunk(msg);
    } else if (msg.type === "history") {
      for (const c of msg.chunks) ingestChunk(c);
    }
  };
  ws.onclose = () => {
    el.connDot.className = "dot dot-err";
    el.connText.textContent = "已断开，重连中...";
    state.reconnectTimer = setTimeout(connectWs, RECONNECT_DELAY_MS);
  };
  ws.onerror = () => {
    try { ws.close(); } catch {}
  };
}

function sendConfig() {
  const port = el.portSelect.value;
  const baud = parseInt(el.baudSelect.value, 10);
  if (!port || !baud) return;
  if (state.ws && state.ws.readyState === WebSocket.OPEN) {
    state.ws.send(JSON.stringify({ type: "config", port, baud }));
    pushSystemLine(`切换到 ${port} @ ${baud}`);
    clearLog();
  }
  savePrefs();
}

/* ---------- 操作 ---------- */
function clearLog() {
  state.textLines.length = 0;
  state.hexLines.length = 0;
  state.textPending = "";
  state.hexOffset = 0;
  state.totalBytes = 0;
  state.droppedLines = 0;
  el.logView.textContent = "";
  updateStats();
}

function downloadLog() {
  const lines = el.viewMode.value === "hex"
    ? state.hexLines.map(l => `${fmtTs(l.ts)}  ${hexDumpRow(l.offset, l.bytes)}`)
    : state.textLines.map(l => `${fmtTs(l.ts)}  ${l.text}`);
  const blob = new Blob([lines.join("\n") + "\n"], { type: "text/plain" });
  const url = URL.createObjectURL(blob);
  const a = document.createElement("a");
  const stamp = new Date().toISOString().replace(/[:.]/g, "-");
  a.href = url;
  a.download = `stm32-log-${stamp}.log`;
  document.body.appendChild(a);
  a.click();
  document.body.removeChild(a);
  URL.revokeObjectURL(url);
}

function updateFilter() {
  const src = el.filter.value.trim();
  state.filterInvert = el.filterInvert.checked;
  if (!src) {
    state.filterRe = null;
  } else {
    try {
      state.filterRe = new RegExp(src, "i");
      el.filter.style.borderColor = "";
    } catch {
      state.filterRe = null;
      el.filter.style.borderColor = "var(--error)";
    }
  }
  rerenderAll();
  savePrefs();
}

/* ---------- 端口列表 ---------- */
async function refreshPorts(preferred) {
  try {
    const resp = await fetch("/api/ports");
    const data = await resp.json();
    const current = preferred || data.current || el.portSelect.value;
    el.portSelect.innerHTML = "";
    for (const p of data.ports) {
      const opt = document.createElement("option");
      opt.value = p;
      opt.textContent = p;
      if (p === current) opt.selected = true;
      el.portSelect.appendChild(opt);
    }
    if (data.ports.length === 0) {
      const opt = document.createElement("option");
      opt.textContent = "(无可用串口)";
      opt.disabled = true;
      el.portSelect.appendChild(opt);
    } else if (!data.ports.includes(current) && current) {
      // 当前值不在扫描结果里，也加进去
      const opt = document.createElement("option");
      opt.value = current;
      opt.textContent = current + " (离线)";
      opt.selected = true;
      el.portSelect.appendChild(opt);
    }
  } catch (e) {
    console.error("refreshPorts failed", e);
  }
}

/* ---------- 事件绑定 ---------- */
function bindEvents() {
  el.portRefresh.addEventListener("click", () => refreshPorts());
  el.apply.addEventListener("click", sendConfig);
  el.viewMode.addEventListener("change", () => { rerenderAll(); updateStats(); savePrefs(); });
  el.showTs.addEventListener("change", () => { rerenderAll(); savePrefs(); });
  el.autoscroll.addEventListener("change", () => { scrollIfNeeded(); savePrefs(); });
  el.filter.addEventListener("input", updateFilter);
  el.filterInvert.addEventListener("change", updateFilter);
  el.clear.addEventListener("click", clearLog);
  el.download.addEventListener("click", downloadLog);
  el.baudSelect.addEventListener("change", savePrefs);
  el.portSelect.addEventListener("change", savePrefs);

  el.logView.addEventListener("wheel", (e) => {
    // 用户主动向上滚就关掉自动滚动，滚到底再打开
    if (e.deltaY < 0) el.autoscroll.checked = false;
    const atBottom = el.logView.scrollTop + el.logView.clientHeight >= el.logView.scrollHeight - 4;
    if (atBottom) el.autoscroll.checked = true;
  });
}

/* ---------- 启动 ---------- */
async function init() {
  const prefs = loadPrefs();
  if (prefs.baud) el.baudSelect.value = prefs.baud;
  if (prefs.viewMode) el.viewMode.value = prefs.viewMode;
  if (typeof prefs.showTs === "boolean") el.showTs.checked = prefs.showTs;
  if (typeof prefs.autoscroll === "boolean") el.autoscroll.checked = prefs.autoscroll;
  if (prefs.filter) el.filter.value = prefs.filter;
  if (typeof prefs.filterInvert === "boolean") el.filterInvert.checked = prefs.filterInvert;

  bindEvents();
  await refreshPorts(prefs.port);
  updateFilter();     // 应用过滤（若有）
  updateStats();
  connectWs();
}

init();

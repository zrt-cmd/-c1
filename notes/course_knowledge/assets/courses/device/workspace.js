/* ============================================================
   设备开发工作台 · 前端逻辑
   会话列表（SSH 终端 / 文件管理 / 串口 / 远程桌面）+ 多标签 + 分屏 + 独立窗口
   跨不过浏览器边界的协议（SSH / SFTP）统一由【本机助手】start_helper.py 转发
   链路：浏览器 → 127.0.0.1:8100（本机助手）→ 设备IP:22（sshd），设备端免安装
   串口为本机 USB 直连（浏览器 Web Serial），数据线插在电脑上，不经任何网络服务
   远程桌面为 noVNC 直连设备:6080，不经过助手
   ============================================================ */
(function () {
  'use strict';

  /* ---------------------------------------------------------- 基础工具 */
  var $ = function (id) { return document.getElementById(id); };
  var LS_SESSIONS = 'eai_device_sessions_v2';
  var LS_PREFS = 'eai_device_prefs_v2';

  var ICON = { term: '⌨', sftp: '📁', serial: '📡', vnc: '🖥' };
  var KIND_NAME = { term: 'SSH 终端', sftp: '文件管理', serial: '串口', vnc: '远程桌面' };
  var KINDS = ['term', 'sftp', 'serial', 'vnc'];
  var BAUD_LIST = [1200, 2400, 4800, 9600, 14400, 19200, 38400, 57600, 76800, 115200,
    128000, 230400, 256000, 460800, 500000, 921600, 1000000, 1500000, 2000000, 3000000];
  var DEF_BRIDGE_PORT = 8100;   // 本机助手默认端口（start_helper.py，只监听 127.0.0.1）
  var HELPER_HOST = '127.0.0.1';
  var HELPER_PROTOCOL = 'eai-helper:';   // 需一次性运行平台对应的 register 脚本注册
  // 平台判定（仅用于展示对应平台的启动/注册指引文案）。
  // 页面无法直接执行 .bat/.sh，实际拉起靠 eai-helper:// 协议——
  // 协议在哪个平台注册成功，就会唤起那个平台对应的启动方式。
  var OS = (function () {
    var ua = navigator.userAgent || '';
    // 此处描述的是【页面宿主环境】的操作系统（即本台电脑），用于生成本机引导文案。
    if (/Windows/i.test(ua)) {
      return {
        plat: 'win',
        reg: '先双击一次 register_helper.bat（一次性注册 eai-helper:// 启动协议）',
        start: '直接双击 start_helper.bat 手动启动'
      };
    }
    if (/Mac OS X|Macintosh/i.test(ua)) {
      return {   // macOS 由 register_helper_macos.sh 生成 .app 注册（start_helper.sh 首次运行已自动执行）
        plat: 'mac',
        reg: '先执行一次 ./register_helper_macos.sh（一次性生成启动器并注册 eai-helper://）',
        start: '直接在 device 目录执行 ./start_helper.sh 手动启动'
      };
    }
    if (/Linux|X11/i.test(ua)) {
      return {
        plat: 'linux',
        reg: '先执行一次 ./register_helper.sh（一次性注册 eai-helper:// 启动协议）',
        start: '直接在 device 目录执行 ./start_helper.sh 手动启动'
      };
    }
    return { plat: '', reg: '', start: '' };
  })();
  var DEF_SSH_PORT = 22;
  var DEF_VNC_PORT = 6080;
  var uid = 0;

  function esc(s) {
    return String(s == null ? '' : s).replace(/[&<>"']/g, function (c) {
      return { '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c];
    });
  }
  function lsGet(k, dflt) {
    try { var v = localStorage.getItem(k); return v ? JSON.parse(v) : dflt; } catch (e) { return dflt; }
  }
  function lsSet(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) { } }

  /* 本机助手「已初始化」记忆（与 entryflow.js 共用键）：
     1=本机协议已注册且成功拉起过 → 入口/连接可静默发 eai-helper://；
     其他值=全新环境 → 任何地方都不发协议，彻底避免系统「选择应用」弹框 */
  var KEY_HELPER_READY = 'eai_helper_ready';
  var KEY_HELPER_PENDING = 'eai_helper_pending';
  function helperReady() { return lsGet(KEY_HELPER_READY, null); }
  function markHelperReady() { lsSet(KEY_HELPER_READY, 1); }

  function b64enc(u8) {
    var s = '', CH = 0x8000;
    for (var i = 0; i < u8.length; i += CH) s += String.fromCharCode.apply(null, u8.subarray(i, i + CH));
    return btoa(s);
  }
  function b64dec(b) {
    var s = atob(b), u8 = new Uint8Array(s.length);
    for (var i = 0; i < s.length; i++) u8[i] = s.charCodeAt(i);
    return u8;
  }
  function enc(text) { return new TextEncoder().encode(text); }

  function fmtSize(n) {
    if (n == null) return '';
    if (n < 1024) return n + ' B';
    var u = ['KB', 'MB', 'GB', 'TB'], i = -1, v = n;
    do { v /= 1024; i++; } while (v >= 1024 && i < u.length - 1);
    return (v < 10 ? v.toFixed(1) : Math.round(v)) + ' ' + u[i];
  }
  function fmtTime(ts) {
    if (!ts) return '';
    var d = new Date(ts * 1000), p = function (x) { return String(x).padStart(2, '0'); };
    return d.getFullYear() + '-' + p(d.getMonth() + 1) + '-' + p(d.getDate()) +
      ' ' + p(d.getHours()) + ':' + p(d.getMinutes());
  }
  function stamp() {
    var d = new Date(), p = function (n, w) { return String(n).padStart(w || 2, '0'); };
    return '[' + p(d.getHours()) + ':' + p(d.getMinutes()) + ':' + p(d.getSeconds()) +
      '.' + p(d.getMilliseconds(), 3) + '] ';
  }
  function b64url(str) {
    return btoa(unescape(encodeURIComponent(str))).replace(/\+/g, '-').replace(/\//g, '_').replace(/=+$/, '');
  }
  function unb64url(str) {
    var s = str.replace(/-/g, '+').replace(/_/g, '/');
    while (s.length % 4) s += '=';
    return decodeURIComponent(escape(atob(s)));
  }
  function joinPath(dir, name) {
    if (!dir) dir = '/';
    return (dir === '/' ? '' : dir.replace(/\/+$/, '')) + '/' + name;
  }
  function clone(o) { return JSON.parse(JSON.stringify(o)); }
  function newId() { return 's' + Date.now().toString(36) + Math.random().toString(36).slice(2, 6); }

  /* ---------------------------------------------------------- 状态 */
  var S = {
    sessions: [],        // 已保存的会话（左侧列表）
    conns: {},           // bridgeKey -> 已登录连接 { token, user, sshHost, sshPort, homedir, base, wsBase }
    prefs: {},           // 上次用过的设备信息，用于新建时预填
    sameOrigin: null,    // 页面由本机助手托管时记录 { addr, port }
    helper: {            // 本机助手探测结果 { port, online }，连接时按需刷新
      port: lsGet('eai_helper_port', DEF_BRIDGE_PORT) || DEF_BRIDGE_PORT,
      online: false
    },
    split: false, activePane: 'L',
    panes: { L: { tabs: [], active: null }, R: { tabs: [], active: null } },
    seq: 0, popout: false
  };

  function allTabs() { return S.panes.L.tabs.concat(S.panes.R.tabs); }
  function findTab(id) {
    var t = S.panes.L.tabs.filter(function (x) { return x.id === id; })[0];
    return t || S.panes.R.tabs.filter(function (x) { return x.id === id; })[0];
  }
  function paneOf(id) {
    if (S.panes.L.tabs.some(function (x) { return x.id === id; })) return 'L';
    if (S.panes.R.tabs.some(function (x) { return x.id === id; })) return 'R';
    return null;
  }

  /* ---------------------------------------------------------- 状态栏 / 提示 */
  function setStatus(text, kind) {
    var dot = $('sbDot');
    if (dot) dot.className = 'sdot' + (kind ? ' ' + kind : '');
    if ($('sbText')) $('sbText').textContent = text;
  }
  var toastTimer = null;
  function toast(msg, kind) {
    var box = $('toast');
    box.className = 'toast' + (kind ? ' ' + kind : '');
    box.textContent = msg;
    box.hidden = false;
    clearTimeout(toastTimer);
    toastTimer = setTimeout(function () { box.hidden = true; }, kind === 'bad' ? 7000 : 3600);
  }
  function isSerialActive() {
    var p = S.activePane && S.panes[S.activePane];
    var t = (p && p.active) ? findTab(p.active) : null;
    return !!(t && t.kind === 'serial');
  }
  function setDeviceLabel(text) {
    if (!$('sbBridge')) return;
    if (text && isSerialActive()) return;   // 串口是本机 USB 直连，不显示本机助手字样
    $('sbBridge').textContent = text || '';
  }
  // 标签切换 / 开关标签时刷新右下角：串口激活则清掉助手字样，SSH/SFTP 激活则恢复其链路显示
  function refreshAnimationByTab() {
    var p = S.activePane && S.panes[S.activePane];
    var t = (p && p.active) ? findTab(p.active) : null;
    if (!t) { if (!allTabs().length) setDeviceLabel(''); return; }
    if (t.kind === 'serial') { setDeviceLabel(''); return; }
    if (t.conn) {
      setDeviceLabel('本机助手 ' + (t.conn.base || '').replace(/^https?:\/\//, '') +
        ' → ' + t.conn.addr + ':' + (t.conn.sshPort || DEF_SSH_PORT) + ' · ' + t.title);
    }
  }
  function setUserLabel(text) { if ($('wbUser')) $('wbUser').textContent = text || '未连接'; }

  /* ==========================================================
     会话仓库（localStorage）
     ========================================================== */
  function blankSession(kind) {
    return {
      id: newId(), kind: kind || 'term', name: '',
      addr: '',
      user: '', password: '', remember: true,
      sshPort: DEF_SSH_PORT, startDir: '',
      serialPort: '', serialLabel: '', baud: 115200, dataBits: 8, stopBits: 1, parity: 'none',
      vncPort: DEF_VNC_PORT, at: 0
    };
  }

  function loadSessions() {
    var list = lsGet(LS_SESSIONS, null);
    if (!Array.isArray(list)) list = [];
    S.sessions = list.filter(function (s) { return s && s.id && KINDS.indexOf(s.kind) >= 0; });
    // 迁移：桥接统一走本机助手（127.0.0.1，端口全局共用）；
    // 串口统一为本机直连（Web Serial），清掉历史的设备端串口字段
    S.sessions.forEach(function (s) {
      if (s.bridgePort) {
        S.helper.port = s.bridgePort;
        lsSet('eai_helper_port', s.bridgePort);
      }
      delete s.bridgePort; delete s.sshHost; delete s.target;
      if (s.kind === 'serial') {
        s.addr = ''; s.user = ''; s.password = '';
        if (!s.serialLabel) s.serialLabel = s.serialPort || '';
      }
      // 迁移：自动名快照与当前参数不一致的僵尸（曾编辑过 IP/波特率、名称还停在旧值）→ 清掉，标题按现值重算
      if (s.name) {
        var stale = false;
        var ipRe = /^\d{1,3}(\.\d{1,3}){3}$/;
        if (s.kind === 'serial' && s.name.indexOf('本机串口 @') === 0) {
          stale = s.name !== '本机串口 @' + (s.baud || 115200);
        } else if (ipRe.test(s.name)) {
          stale = !!s.addr && s.name !== s.addr;
        } else {
          var at = s.name.lastIndexOf('@');
          if (at > 0 && ipRe.test(s.name.slice(at + 1))) stale = s.name.slice(at + 1) !== (s.addr || '');
        }
        if (stale) s.name = '';
      }
    });
  }
  function saveSessions() {
    lsSet(LS_SESSIONS, S.sessions.map(function (s) {
      var c = clone(s);
      if (!c.remember) c.password = '';
      return c;
    }));
  }
  function findSession(id) {
    return S.sessions.filter(function (s) { return s.id === id; })[0] || null;
  }
  function upsertSession(sess) {
    var i = S.sessions.findIndex(function (s) { return s.id === sess.id; });
    if (i < 0) S.sessions.push(sess); else S.sessions[i] = sess;
    saveSessions();
  }
  function removeSession(id) {
    S.sessions = S.sessions.filter(function (s) { return s.id !== id; });
    saveSessions();
  }
  function savePrefs(s) {
    S.prefs = { addr: s.addr, user: s.user };
    lsSet(LS_PREFS, S.prefs);
  }

  /* ==========================================================
     终端字号（缩放）：全局偏好，SSH 终端与串口终端共用。
     只调 xterm 字号并重新 fit，网页其它排版不动（区别于浏览器整页缩放）
     ========================================================== */
  var TERM_FONT_KEY = 'eai_term_font_size';
  var TERM_FONT_MIN = 10, TERM_FONT_MAX = 28, TERM_FONT_DEFAULT = 15;
  var termFontSubs = [];
  function getTermFont() {
    var v = parseInt(lsGet(TERM_FONT_KEY, 0), 10);
    return (v >= TERM_FONT_MIN && v <= TERM_FONT_MAX) ? v : TERM_FONT_DEFAULT;
  }
  function setTermFont(px) {
    var v = Math.round(Number(px));
    if (!v) v = TERM_FONT_DEFAULT;
    v = Math.max(TERM_FONT_MIN, Math.min(TERM_FONT_MAX, v));
    if (v !== getTermFont()) lsSet(TERM_FONT_KEY, v);
    termFontSubs.forEach(function (fn) { try { fn(v); } catch (e) { } });
    return v;
  }
  function subTermFont(fn) {
    termFontSubs.push(fn);
    return function () { termFontSubs = termFontSubs.filter(function (f) { return f !== fn; }); };
  }
  function setXtermFont(t, px) {
    if (!t) return;
    // 写后校验：老版本 options 为只读代理时不生效，退回 setOption
    try { t.options.fontSize = px; if (t.options.fontSize === px) return; } catch (e) { }
    try { if (typeof t.setOption === 'function') t.setOption('fontSize', px); } catch (e) { }
  }

  /* 自动名形态：连接时自动生成的名称（用户名@IP / 纯IP / 本机串口@波特率）。
     编辑表单没有名称字段，这类名称不会随参数更新，判定为自动名即丢弃重算；
     按形状而非「与当前地址是否一致」判定——旧 JS 下地址被改过、名停在旧值的存量僵尸同样能清 */
  function isAutoName(nm) {
    if (!nm) return false;
    if (nm.indexOf('本机串口 @') === 0) return true;
    var ipRe = /^\d{1,3}(\.\d{1,3}){3}$/;
    if (ipRe.test(nm)) return true;
    var at = nm.lastIndexOf('@');
    return at > 0 && ipRe.test(nm.slice(at + 1));
  }

  function sessTitle(s) {
    if (s.name) return s.name;
    if (s.kind === 'serial') return '本机串口 @' + (s.baud || 115200);
    if (s.kind === 'vnc') return (s.addr || '?') + ':' + (s.vncPort || DEF_VNC_PORT);
    return (s.user ? s.user + '@' : '') + (s.addr || '?');
  }
  function sessSub(s) {
    if (s.kind === 'serial') {
      return '本机 USB 直连 · ' + (s.serialLabel || s.serialPort || '未选串口') + ' @' + (s.baud || 115200);
    }
    if (s.kind === 'vnc') {
      return 'noVNC · ' + (s.addr || '?') + ':' + (s.vncPort || DEF_VNC_PORT);
    }
    return (s.addr || '?') + ':' + (s.sshPort || DEF_SSH_PORT) + ' · ' + (s.user || '?');
  }

  /* ==========================================================
     连接：桥接一律指向本机助手 127.0.0.1（设备端免安装），
     由助手去 SSH 登录设备；token 按「设备:SSH端口」缓存，
     多台设备各自独立一条连接，共用同一个助手
     ========================================================== */
  function bridgeKey(sess) {
    return 'dev:' + ((sess && sess.addr) || '?') + ':' + ((sess && sess.sshPort) || DEF_SSH_PORT);
  }
  function bridgeBase() { return 'http://' + HELPER_HOST + ':' + helperPort(); }
  function bridgeWs() {
    var proto = (location.protocol === 'https:') ? 'wss://' : 'ws://';
    return proto + HELPER_HOST + ':' + helperPort();
  }
  function helperPort() {
    return parseInt(S.helper && S.helper.port, 10) || DEF_BRIDGE_PORT;
  }

  function login(sess) {
    var base = bridgeBase();
    return fetch(base + '/api/login', {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify({
        user: sess.user, password: sess.password || '',
        host: sess.addr, port: sess.sshPort || DEF_SSH_PORT   // 助手代连的目标 = 设备 IP
      })
    })
      .then(function (r) { return r.json(); })
      .then(function (j) {
        if (!j || !j.ok) throw new Error((j && j.error) || '登录失败');
        var conn = {
          key: bridgeKey(sess), base: base, wsBase: bridgeWs(),
          token: j.token, user: j.user, sshHost: j.host, sshPort: j.port,
          homedir: j.homedir || '/', addr: sess.addr, sessionId: sess.id
        };
        S.conns[conn.key] = conn;
        S.helper.online = true;
        return conn;
      })
      .catch(function (e) {
        if (e instanceof TypeError) {
          S.helper.online = false;
          throw new Error('无法连接本机助手 ' + base + '（点「连接」会自动尝试启动，也可手动双击 start_helper.bat）');
        }
        throw e;
      });
  }

  function ensureLogin(sess) {
    var c = S.conns[bridgeKey(sess)];
    if (c && c.token) { c.sessionId = sess.id; return Promise.resolve(c); }
    return login(sess);
  }

  function apiURL(conn, path, params) {
    var q = new URLSearchParams();
    if (conn && conn.token) q.set('token', conn.token);
    Object.keys(params || {}).forEach(function (k) {
      if (params[k] != null) q.set(k, params[k]);
    });
    var qs = q.toString();
    return (conn ? conn.base : '') + path + (qs ? '?' + qs : '');
  }
  function dropConn(conn) {
    if (!conn) return;
    delete S.conns[conn.key];
    setStatus('会话已失效，请重新连接', 'err');
    toast('会话已失效（可能被助手超时回收或服务重启），请重新打开该会话。', 'bad');
  }
  function apiGet(conn, path, params) {
    return fetch(apiURL(conn, path, params), { cache: 'no-store' })
      .then(function (r) { return r.json(); })
      .catch(function (e) { return { ok: false, error: '无法连接桥接服务：' + e.message }; });
  }
  function apiPost(conn, path, body, params) {
    return fetch(apiURL(conn, path, params), {
      method: 'POST',
      headers: { 'Content-Type': 'application/json' },
      body: JSON.stringify(body || {})
    })
      .then(function (r) { return r.json(); })
      .catch(function (e) { return { ok: false, error: '无法连接桥接服务：' + e.message }; });
  }
  function authCheck(conn, json) {
    if (json && json.relogin) { dropConn(conn); return true; }
    return false;
  }

  /* ---------------------------------------------------------- 本机助手：探测与唤起 */
  function probeHelper() {
    return fetch(bridgeBase() + '/api/ping', { cache: 'no-store', mode: 'cors' })
      .then(function (r) { return r.json(); })
      .then(function (j) {
        S.helper.online = !!(j && j.service === 'eai-device-bridge');
        return S.helper.online;
      })
      .catch(function () { S.helper.online = false; return false; });
  }

  // 通过 eai-helper:// 自定义协议唤起（需一次性运行平台对应的 register 脚本注册）。
  // 每次页面加载只自动尝试一次，避免反复弹浏览器的「外部协议」询问。
  function launchHelperOnce(force) {
    if (S.helper.launched && !force) return false;
    S.helper.launched = true;
    try {
      // 用隐藏 iframe 发起，避免未注册协议时把当前页导航走 / 弹错误打断页面
      var f = document.createElement('iframe');
      f.style.display = 'none';
      f.src = HELPER_PROTOCOL + '//start?port=' + helperPort();
      document.body.appendChild(f);
      setTimeout(function () { if (f.parentNode) f.parentNode.removeChild(f); }, 20000);
      return true;
    } catch (e) { return false; }
  }

  // 轮询等待助手就绪；waitMs=0 表示不等待，直接返回探测结果
  function waitForHelper(waitMs) {
    return new Promise(function (resolve) {
      var t0 = Date.now();
      var timer = setInterval(function () {
        probeHelper().then(function (ok) {
          if (ok || Date.now() - t0 >= waitMs) {
            clearInterval(timer);
            resolve(ok);
          }
        });
      }, 700);
    });
  }

  // 连接前确保助手在跑：先探测；不在则经协议唤起一次并轮询等待
  function ensureHelper() {
    return probeHelper().then(function (on) {
      if (on) return true;
      // 本机协议未注册（全新环境）绝不发协议 —— 免得系统弹「选择应用」框，交由初始化引导处理
      if (helperReady() !== 1) return false;
      if (!launchHelperOnce(false)) return false;
      setStatus('正在启动本机助手 …', 'busy');
      // 经协议唤起且随后探测成功 = 本机链路已打通，记忆为「已初始化」
      return waitForHelper(6000).then(function (ok) {   // 首次运行还要 pip 装依赖，多等一会
        if (ok) markHelperReady();
        return ok;
      });
    }).then(function (on) {
      if (on) S.helper.online = true;
      return on;
    });
  }

  function loadSysInfo(conn) {
    if (!conn) { $('sideSys').innerHTML = ''; return; }
    S.lastSysKey = conn.addr + '|' + (conn.key || conn.base || '');
    apiGet(conn, '/api/sys/info').then(function (j) {
      if (authCheck(conn, j) || !j || !j.ok) return;
      var rows = [
        ['设备', conn.addr + ':' + (conn.sshPort || DEF_SSH_PORT)],
        ['助手', (conn.base || '').replace(/^https?:\/\//, '')],
        ['用户', j.user + '@' + j.host],
        ['系统', j.os || j.uname || '—'],
        ['内核', j.uname || '—'],
        ['CPU', (j.cpus ? j.cpus + ' 核' : '—')],
        ['家目录', j.homedir || '—'],
        ['磁盘', j.disk || '—'],
        ['运行', j.uptime || '—']
      ];
      $('sideSys').innerHTML = rows.map(function (r) {
        return '<div><span class="k">' + esc(r[0]) + '</span> ' + esc(r[1]) + '</div>';
      }).join('');
    });
  }

  /* 切换标签时同步左上账号标签 + 左侧设备信息；同一台设备（addr|key 相同）不重复拉取 */
  function syncLabelsForTab(t) {
    if (!t) return;
    var c = t.conn || null;
    setUserLabel(c && c.user && c.addr ? (c.user + '@' + c.addr) : (t.title || '未连接'));
    if (!c) {
      if (S.lastSysKey) { S.lastSysKey = ''; loadSysInfo(null); }
      return;
    }
    var key = c.addr + '|' + (c.key || c.base || '');
    if (key === S.lastSysKey) return;
    loadSysInfo(c);
  }

  /* ---------------------------------------------------------- 通用右键菜单 */
  function showMenu(x, y, items) {
    var box = $('ctx');
    box.innerHTML = '';
    items.forEach(function (it) {
      if (it.sep) { box.appendChild(document.createElement('hr')); return; }
      var b = document.createElement('button');
      b.textContent = it.label;
      if (it.danger) b.className = 'danger';
      b.addEventListener('click', function () { hideMenu(); it.fn(); });
      box.appendChild(b);
    });
    box.hidden = false;
    var w = box.offsetWidth, h = box.offsetHeight;
    box.style.left = Math.min(x, window.innerWidth - w - 8) + 'px';
    box.style.top = Math.min(y, window.innerHeight - h - 8) + 'px';
  }
  function hideMenu() { $('ctx').hidden = true; }
  document.addEventListener('click', hideMenu);
  document.addEventListener('contextmenu', function (e) {
    var t = e.target;
    if (!t || !t.closest || (!t.closest('.tab') && !t.closest('.files') &&
      !t.closest('.side-item') && !t.closest('.side-group'))) hideMenu();
  });

  /* ==========================================================
     视图：SSH 终端
     ========================================================== */
  function TermView(cfg, conn) {
    var el = document.createElement('div');
    el.className = 'view termview';
    var host = document.createElement('div');
    host.className = 'termhost';
    var bar = document.createElement('div');
    bar.className = 'term-status';
    // 右侧缩放组：只改终端字体，不牵动网页其它排版（也可按住 Ctrl 滚滚轮）
    bar.innerHTML = '<span class="st">正在连接…</span>' +
      '<span style="flex:1"></span>' +
      '<span style="display:flex;gap:6px;align-items:center" title="只缩放终端字体，不影响页面其它部分（也可按住 Ctrl 滚动滚轮；永久生效）">' +
      '<button class="tbbtn" type="button" data-font="-1" style="padding:1px 9px">A−</button>' +
      '<span data-font-label style="min-width:42px;text-align:center"></span>' +
      '<button class="tbbtn" type="button" data-font="1" style="padding:1px 9px">A＋</button>' +
      '<button class="tbbtn" type="button" data-font="0" style="padding:1px 9px" title="恢复默认 15px">↺</button>' +
      '</span>';
    el.appendChild(host);
    el.appendChild(bar);

    var term = null, fit = null, ws = null, ro = null, closed = false, ready = false;

    function say(text, cls) {
      var st = bar.querySelector('.st');
      if (!st) return;
      st.className = cls || 'st';
      st.textContent = text;
    }
    function applyFont(px) {
      setXtermFont(term, px);
      var lab = bar.querySelector('[data-font-label]');
      if (lab) lab.textContent = px + 'px';
      if (term) doFit();
    }
    bar.addEventListener('click', function (e) {
      var t = e.target;
      var d = t && t.getAttribute && t.getAttribute('data-font');
      if (d == null) return;
      setTermFont(d === '0' ? TERM_FONT_DEFAULT : getTermFont() + (d === '-1' ? -1 : 1));
    });
    host.addEventListener('wheel', function (e) {
      if (!e.ctrlKey && !e.metaKey) return;
      e.preventDefault();
      setTermFont(getTermFont() + (e.deltaY < 0 ? 1 : -1));
    }, { passive: false });
    var unsubFont = subTermFont(applyFont);
    applyFont(getTermFont());

    function connect() {
      if (!window.Terminal) { say('未找到 xterm.js（vendor/xterm 目录缺失），终端无法工作', 'bad'); return; }
      if (!conn) { say('缺少连接信息', 'bad'); return; }
      if (term) { try { term.dispose(); } catch (e) { } term = null; }
      host.innerHTML = '';
      term = new window.Terminal({
        fontFamily: "'Cascadia Code','JetBrains Mono',Consolas,'Courier New',monospace",
        fontSize: getTermFont(), lineHeight: 1.25, cursorBlink: true, scrollback: 8000,
        allowProposedApi: true, convertEol: false,
        theme: {
          background: '#0f172a', foreground: '#e2e8f0', cursor: '#7dd3fc',
          selectionBackground: '#334155', black: '#1e293b', red: '#f87171',
          green: '#5eead4', yellow: '#fbbf24', blue: '#7dd3fc',
          magenta: '#c4b5fd', cyan: '#67e8f9', white: '#e2e8f0'
        }
      });
      fit = new window.FitAddon.FitAddon();
      term.loadAddon(fit);
      if (window.WebLinksAddon) term.loadAddon(new window.WebLinksAddon.WebLinksAddon());
      term.open(host);
      doFit();
      term.onData(function (d) {
        if (ws && ws.readyState === 1) ws.send(JSON.stringify({ t: 'in', b: b64enc(enc(d)) }));
      });
      term.onResize(function (sz) {
        if (ws && ws.readyState === 1) ws.send(JSON.stringify({ t: 'resize', c: sz.cols, r: sz.rows }));
      });
      openWS();
    }
    function doFit() {
      if (!fit || !term) return;
      var r = host.getBoundingClientRect();
      if (r.width < 20 || r.height < 20) return;
      try { fit.fit(); } catch (e) { }
    }
    function openWS() {
      var cols = (term && term.cols) || 80, rows = (term && term.rows) || 24;
      var url = conn.wsBase + '/ws/ssh?token=' + encodeURIComponent(conn.token) +
        '&cols=' + cols + '&rows=' + rows;
      try { ws = new WebSocket(url); } catch (e) {
        say('无法建立 WebSocket：' + e.message, 'bad'); return;
      }
      ws.onopen = function () { say('已连接 · ' + conn.user + '@' + conn.sshHost, 'good'); };
      ws.onmessage = function (ev) {
        var m;
        try { m = JSON.parse(ev.data); } catch (e) { return; }
        if (m.t === 'out') {
          term.write(b64dec(m.b));
        } else if (m.t === 'ready') {
          ready = true;
          say('已连接 · ' + m.user + '@' + m.host + ' · ' + m.homedir, 'good');
          if (cfg.startDir) {
            term.write('\x1b[90m# 进入 ' + cfg.startDir + '\x1b[0m\r');
            ws.send(JSON.stringify({ t: 'in', b: b64enc(enc('cd ' + cfg.startDir + '\n')) }));
          }
        } else if (m.t === 'err') {
          term.write('\r\n\x1b[31m' + m.d + '\x1b[0m\r\n');
          say(m.d, 'bad');
          if (m.relogin) dropConn(conn);
        } else if (m.t === 'exit') {
          term.write('\r\n\x1b[90m[会话已结束]\x1b[0m\r\n');
          say('会话已结束', 'bad');
        }
      };
      ws.onclose = function () {
        if (closed) return;
        if (ready) say('连接已断开 · 关闭标签可释放', 'bad');
        ready = false;
      };
      ws.onerror = function () { say('WebSocket 错误，请检查桥接服务是否在线', 'bad'); };
    }
    function disconnect() {
      closed = true;
      try { if (ws) ws.close(); } catch (e) { }
      ws = null;
      try { if (ro) ro.disconnect(); } catch (e) { }
    }

    return {
      el: el, kind: 'term',
      title: cfg.title || (conn ? conn.user + '@' + conn.addr : 'SSH 终端'),
      mount: function () {
        connect();
        if (window.ResizeObserver) {
          ro = new ResizeObserver(function () { doFit(); });
          ro.observe(host);
        }
      },
      show: function () { doFit(); if (term) term.focus(); },
      hide: function () { },
      refit: doFit,
      close: function () {
        disconnect();
        if (unsubFont) unsubFont();
        try { if (term) term.dispose(); } catch (e) { }
        term = null;
      }
    };
  }

  /* ==========================================================
     视图：文件管理（SFTP）
     ========================================================== */
  function SftpView(cfg, conn) {
    var el = document.createElement('div');
    el.className = 'view sftpview';

    el.innerHTML = [
      '<div class="tb">',
      '  <button class="tbbtn" data-a="up" title="上级目录">↑</button>',
      '  <input class="crumb" data-a="path" spellcheck="false" autocomplete="off">',
      '  <button class="tbbtn" data-a="go">跳转</button>',
      '  <button class="tbbtn" data-a="reload">⟳</button>',
      '  <span class="sep"></span>',
      '  <button class="tbbtn" data-a="mkdir">新建文件夹</button>',
      '  <button class="tbbtn primary" data-a="upload">上传文件</button>',
      '  <input type="file" multiple hidden data-a="fileinput">',
      '  <span class="flex"></span>',
      '  <button class="tbbtn" data-a="download" disabled>下载</button>',
      '  <button class="tbbtn" data-a="rename" disabled>重命名</button>',
      '  <button class="tbbtn danger" data-a="delete" disabled>删除</button>',
      '</div>',
      '<div class="sftp-body">',
      '  <div class="files">',
      '    <div class="dropmask">松开即可上传到当前目录</div>',
      '    <table><thead><tr><th>名称</th><th>大小</th><th>权限</th><th>修改时间</th></tr></thead>',
      '    <tbody></tbody></table>',
      '  </div>',
      '  <div class="editor">',
      '    <div class="ed-head">',
      '      <span class="ed-name">未打开文件</span>',
      '      <span class="ed-meta"></span>',
      '      <button class="tbbtn primary" data-a="save" disabled>保存</button>',
      '      <button class="tbbtn" data-a="closefile" disabled>关闭</button>',
      '    </div>',
      '    <div class="ed-body"><div class="ed-gutter">1</div>',
      '      <textarea class="ed-area" spellcheck="false" wrap="off" disabled></textarea></div>',
      '    <div class="ed-hint" data-a="edhint">双击左侧文件即可在此编辑，Ctrl+S 保存。</div>',
      '  </div>',
      '</div>'
    ].join('');

    var tbody = el.querySelector('tbody');
    var filesBox = el.querySelector('.files');
    var crumb = el.querySelector('[data-a="path"]');
    var body = el.querySelector('.sftp-body');
    var edName = el.querySelector('.ed-name');
    var edMeta = el.querySelector('.ed-meta');
    var edArea = el.querySelector('.ed-area');
    var edGutter = el.querySelector('.ed-gutter');
    var edHint = el.querySelector('[data-a="edhint"]');
    var fileInput = el.querySelector('[data-a="fileinput"]');

    var cwd = cfg.startDir || (conn && conn.homedir) || '/';
    var entries = [];
    var selected = null;
    var opened = null;
    var busy = false;

    function btn(a) { return el.querySelector('[data-a="' + a + '"]'); }
    function selActions() {
      var has = !!selected;
      ['download', 'rename', 'delete'].forEach(function (a) { btn(a).disabled = !has; });
    }
    function setHint(html) { edHint.innerHTML = html; }

    function load(path) {
      if (busy) return;
      busy = true;
      setStatus('读取目录 ' + path + ' …', 'busy');
      apiGet(conn, '/api/fs/list', { path: path }).then(function (j) {
        busy = false;
        if (authCheck(conn, j)) return;
        if (!j || !j.ok) { setStatus('读取失败：' + ((j && j.error) || '未知错误'), 'err'); return; }
        cwd = j.path;
        crumb.value = cwd;
        entries = j.entries || [];
        selected = null;
        render();
        selActions();
        setStatus('目录 ' + cwd + ' · ' + entries.length + ' 项', 'on');
      });
    }
    function render() {
      tbody.innerHTML = '';
      if (!entries.length) {
        tbody.innerHTML = '<tr><td colspan="4" class="empty">空目录</td></tr>';
        return;
      }
      entries.forEach(function (e) {
        var tr = document.createElement('tr');
        var ico = e.dir ? '📁' : (e.link ? '🔗' : fileIcon(e.name));
        tr.innerHTML =
          '<td class="nm"><span class="ico">' + ico + '</span>' +
          '<span class="txt' + (e.link ? ' lnk' : '') + '"></span></td>' +
          '<td class="sz">' + (e.dir ? '—' : fmtSize(e.size)) + '</td>' +
          '<td class="md">' + esc(e.mode) + '</td>' +
          '<td class="mt">' + fmtTime(e.mtime) + '</td>';
        tr.querySelector('.txt').textContent = e.name + (e.link && e.target ? ' → ' + e.target : '');
        tr.addEventListener('click', function () {
          selected = e;
          Array.prototype.forEach.call(tbody.querySelectorAll('tr'), function (x) { x.classList.remove('sel'); });
          tr.classList.add('sel');
          selActions();
        });
        tr.addEventListener('dblclick', function () {
          if (e.dir) load(joinPath(cwd, e.name));
          else openFile(joinPath(cwd, e.name));
        });
        tr.addEventListener('contextmenu', function (ev) {
          ev.preventDefault();
          ev.stopPropagation();
          selected = e;
          Array.prototype.forEach.call(tbody.querySelectorAll('tr'), function (x) { x.classList.remove('sel'); });
          tr.classList.add('sel');
          selActions();
          var full = joinPath(cwd, e.name);
          var items = [];
          if (e.dir) items.push({ label: '进入目录', fn: function () { load(full); } });
          else items.push({ label: '编辑', fn: function () { openFile(full); } });
          if (!e.dir) items.push({ label: '下载', fn: function () { download(full); } });
          items.push({ sep: true });
          items.push({ label: '重命名', fn: function () { rename(full, e.name); } });
          items.push({ label: '复制路径', fn: function () { copyText(full); } });
          items.push({ sep: true });
          items.push({ label: '删除', danger: true, fn: function () { del(full, e.name, e.dir); } });
          showMenu(ev.clientX, ev.clientY, items);
        });
        tbody.appendChild(tr);
      });
    }
    function fileIcon(name) {
      var ext = (name.split('.').pop() || '').toLowerCase();
      if (['py', 'js', 'ts', 'json', 'yaml', 'yml', 'cpp', 'c', 'h', 'hpp', 'sh'].indexOf(ext) >= 0) return '📜';
      if (['md', 'txt', 'log', 'csv', 'rst'].indexOf(ext) >= 0) return '📄';
      if (['png', 'jpg', 'jpeg', 'gif', 'svg', 'bmp', 'webp'].indexOf(ext) >= 0) return '🖼';
      if (['zip', 'tar', 'gz', 'xz', 'bz2', '7z'].indexOf(ext) >= 0) return '🗜';
      if (['stl', 'obj', 'urdf', 'dae', 'gltf', 'glb'].indexOf(ext) >= 0) return '🧩';
      return '📄';
    }

    function openFile(path) {
      setStatus('读取文件 ' + path + ' …', 'busy');
      apiGet(conn, '/api/fs/read', { path: path }).then(function (j) {
        if (authCheck(conn, j)) return;
        if (!j || !j.ok) { setStatus('打开失败：' + ((j && j.error) || '未知错误'), 'err'); return; }
        if (j.binary) {
          setStatus('二进制文件，不能在线编辑，已改为下载', 'err');
          download(path);
          return;
        }
        opened = { path: j.path, encoding: j.encoding || 'utf-8', dirty: false };
        body.classList.add('editing');
        edName.textContent = j.path;
        edMeta.textContent = fmtSize(j.size) + ' · ' + (j.encoding || 'utf-8') + ' · ' + j.mode;
        edArea.value = j.content;
        edArea.disabled = false;
        btn('save').disabled = false;
        btn('closefile').disabled = false;
        if (j.truncated) setHint('<span class="warn">文件较大，仅载入前 2 MB；保存会覆盖整个文件，请谨慎。</span>');
        else setHint('Ctrl+S 保存 · Tab 缩进 · 关闭前请先保存');
        updateGutter();
        setStatus('已打开 ' + j.path, 'on');
      });
    }
    function updateGutter() {
      var n = edArea.value.split('\n').length;
      var out = [];
      for (var i = 1; i <= n; i++) out.push(i);
      edGutter.textContent = out.join('\n');
      edGutter.scrollTop = edArea.scrollTop;
    }
    function saveFile() {
      if (!opened) return;
      var content = edArea.value;
      btn('save').disabled = true;
      setStatus('保存中…', 'busy');
      apiPost(conn, '/api/fs/write', { path: opened.path, content: content, encoding: opened.encoding })
        .then(function (j) {
          btn('save').disabled = false;
          if (authCheck(conn, j)) return;
          if (!j || !j.ok) { setStatus('保存失败：' + ((j && j.error) || '未知错误'), 'err'); return; }
          opened.dirty = false;
          setStatus('已保存 ' + j.path + '（' + fmtSize(j.bytes) + '）', 'on');
          setHint('已保存 · Ctrl+S 再次保存');
          load(cwd);
        });
    }
    function closeFile() {
      opened = null;
      body.classList.remove('editing');
      edArea.value = '';
      edArea.disabled = true;
      edName.textContent = '未打开文件';
      edMeta.textContent = '';
      btn('save').disabled = true;
      btn('closefile').disabled = true;
      setHint('双击左侧文件即可在此编辑，Ctrl+S 保存。');
    }

    function mkdir() {
      var name = prompt('新文件夹名称：');
      if (!name) return;
      apiPost(conn, '/api/fs/mkdir', { path: joinPath(cwd, name.trim()) }).then(function (j) {
        if (authCheck(conn, j)) return;
        if (!j.ok) { alert('创建失败：' + j.error); return; }
        load(cwd);
      });
    }
    function rename(path, old) {
      var name = prompt('重命名 / 移动到（可填绝对路径）：', old);
      if (!name || name === old) return;
      var target = name.indexOf('/') === 0 ? name.trim() : joinPath(cwd, name.trim());
      apiPost(conn, '/api/fs/rename', { path: path, target: target }).then(function (j) {
        if (authCheck(conn, j)) return;
        if (!j.ok) { alert('重命名失败：' + j.error); return; }
        load(cwd);
      });
    }
    function del(path, name, isDir) {
      if (!confirm('确定删除 ' + name + (isDir ? '（含其中全部内容）' : '') + ' ？此操作不可撤销。')) return;
      apiPost(conn, '/api/fs/delete', { path: path }).then(function (j) {
        if (authCheck(conn, j)) return;
        if (!j.ok) { alert('删除失败：' + j.error); return; }
        load(cwd);
      });
    }
    function download(path) {
      var a = document.createElement('a');
      a.href = apiURL(conn, '/api/fs/download', { path: path });
      a.download = '';
      document.body.appendChild(a);
      a.click();
      a.remove();
    }
    function copyText(text) {
      if (navigator.clipboard) navigator.clipboard.writeText(text).then(function () {
        setStatus('已复制：' + text, 'on');
      }, function () { prompt('手动复制：', text); });
      else prompt('手动复制：', text);
    }
    function upload(files) {
      if (!files || !files.length) return;
      var fd = new FormData();
      for (var i = 0; i < files.length; i++) fd.append('file', files[i], files[i].name);
      setStatus('上传 ' + files.length + ' 个文件…', 'busy');
      fetch(apiURL(conn, '/api/fs/upload', { path: cwd }), { method: 'POST', body: fd })
        .then(function (r) { return r.json(); })
        .then(function (j) {
          if (authCheck(conn, j)) return;
          if (!j.ok) { setStatus('上传失败：' + j.error, 'err'); return; }
          setStatus('已上传 ' + j.saved.length + ' 个文件到 ' + cwd, 'on');
          load(cwd);
        })
        .catch(function (e) { setStatus('上传失败：' + e.message, 'err'); });
    }

    el.addEventListener('click', function (e) {
      var a = e.target.getAttribute && e.target.getAttribute('data-a');
      if (!a) return;
      if (a === 'up') {
        var parent = cwd.replace(/\/+$/, '').split('/').slice(0, -1).join('/') || '/';
        load(parent);
      } else if (a === 'go') load(crumb.value.trim() || '/');
      else if (a === 'reload') load(cwd);
      else if (a === 'mkdir') mkdir();
      else if (a === 'upload') fileInput.click();
      else if (a === 'download' && selected) download(joinPath(cwd, selected.name));
      else if (a === 'rename' && selected) rename(joinPath(cwd, selected.name), selected.name);
      else if (a === 'delete' && selected) del(joinPath(cwd, selected.name), selected.name, selected.dir);
      else if (a === 'save') saveFile();
      else if (a === 'closefile') closeFile();
    });
    crumb.addEventListener('keydown', function (e) {
      if (e.key === 'Enter') { e.preventDefault(); load(crumb.value.trim() || '/'); }
    });
    fileInput.addEventListener('change', function () { upload(this.files); this.value = ''; });
    edArea.addEventListener('input', function () { updateGutter(); if (opened) opened.dirty = true; });
    edArea.addEventListener('scroll', function () { edGutter.scrollTop = edArea.scrollTop; });
    edArea.addEventListener('keydown', function (e) {
      if ((e.ctrlKey || e.metaKey) && (e.key === 's' || e.key === 'S')) { e.preventDefault(); saveFile(); return; }
      if (e.key === 'Tab') {
        e.preventDefault();
        var st = this.selectionStart, en = this.selectionEnd;
        this.value = this.value.slice(0, st) + '  ' + this.value.slice(en);
        this.selectionStart = this.selectionEnd = st + 2;
        updateGutter();
      }
    });
    ['dragenter', 'dragover'].forEach(function (t) {
      filesBox.addEventListener(t, function (e) { e.preventDefault(); filesBox.classList.add('drag'); });
    });
    ['dragleave', 'drop'].forEach(function (t) {
      filesBox.addEventListener(t, function (e) { e.preventDefault(); filesBox.classList.remove('drag'); });
    });
    filesBox.addEventListener('drop', function (e) {
      if (e.dataTransfer && e.dataTransfer.files) upload(e.dataTransfer.files);
    });

    return {
      el: el, kind: 'sftp',
      title: cfg.title || ('文件 · ' + (conn ? conn.homedir : '/')),
      mount: function () { load(cwd); },
      show: function () { },
      hide: function () { },
      refit: function () { },
      close: function () { }
    };
  }

  /* ==========================================================
     视图：本机串口（Web Serial，USB 直连，不经过网络）
     ========================================================== */
  function LocalSerialView(cfg) {
    var el = document.createElement('div');
    el.className = 'view serialview';
    var n = ++uid;
    var portListId = 'lp' + n, baudListId = 'lb' + n;

    el.innerHTML = [
      '<div class="tb">',
      '  <div class="f"><label>系统串口</label>',
      '    <input data-a="port" list="' + portListId + '" spellcheck="false" autocomplete="off"',
      '      style="min-width:200px" placeholder="serial:1（连接时选择）">',
      '    <datalist id="' + portListId + '"><option value="serial:1">连接时由系统弹窗选择</option></datalist></div>',
      '  <div class="f"><label>波特率</label>',
      '    <input data-a="baud" list="' + baudListId + '" inputmode="numeric" style="width:110px">',
      '    <datalist id="' + baudListId + '">' +
      BAUD_LIST.map(function (b) { return '<option value="' + b + '"></option>'; }).join('') +
      '</datalist></div>',
      '  <div class="f"><label>数据位</label><select data-a="dataBits"><option selected>8</option><option>7</option></select></div>',
      '  <div class="f"><label>停止位</label><select data-a="stopBits"><option selected>1</option><option>2</option></select></div>',
      '  <div class="f"><label>校验</label><select data-a="parity"><option value="none" selected>无</option><option value="even">偶</option><option value="odd">奇</option></select></div>',
      '  <div class="f"><label>流控</label><select data-a="flow"><option value="none" selected>无</option><option value="hardware">RTS/CTS</option></select></div>',
      '  <span class="sep"></span>',
      '  <button class="tbbtn primary" data-a="connect">打开串口</button>',
      '  <button class="tbbtn" data-a="disconnect" disabled>断开</button>',
      '  <button class="tbbtn" data-a="clear">清空</button>',
      '  <button class="tbbtn" data-a="scan">扫描</button>',
      '  <span class="flex"></span>',
      '  <div class="serial-state"><span class="sdot" data-a="dot"></span><span data-a="state">未连接</span>',
      '    <span class="mono" data-a="bytes" style="color:var(--text-muted)"></span></div>',
      '</div>',
      '<div class="tb">',
      '  <div class="f"><label>行尾符</label><select data-a="eol">',
      '    <option value="crlf" selected>CRLF (\\r\\n)</option><option value="lf">LF (\\n)</option>',
      '    <option value="cr">CR (\\r)</option><option value="">无</option></select></div>',
      '  <div class="f"><label>显示方式</label><select data-a="view">',
      '    <option value="term" selected>终端（可交互）</option>',
      '    <option value="text">原始文本</option>',
      '    <option value="hex">十六进制</option></select></div>',
      '  <div class="f"><label>编码</label><select data-a="enc">',
      '    <option value="utf-8" selected>UTF-8</option><option value="gbk">GBK / GB2312</option>',
      '    <option value="big5">Big5</option><option value="latin1">Latin-1</option></select></div>',
      '  <label class="chk"><input type="checkbox" data-a="ts"> 时间戳</label>',
      '  <label class="chk"><input type="checkbox" data-a="echo" checked> 本地回显</label>',
      '  <label class="chk" title="过滤 ANSI 转义序列（终端控制码），十六进制视图始终显示原始字节">',
      '    <input type="checkbox" data-a="ansi" checked> 过滤转义码</label>',
      '  <span class="flex"></span>',
      '  <span style="display:flex;gap:6px;align-items:center" title="只缩放终端字体，不影响页面其它部分（也可按住 Ctrl 滚动滚轮）">',
      '    <button class="tbbtn" type="button" data-font="-1" style="padding:1px 9px">A−</button>',
      '    <span data-font-label style="min-width:42px;text-align:center"></span>',
      '    <button class="tbbtn" type="button" data-font="1" style="padding:1px 9px">A＋</button>',
      '    <button class="tbbtn" type="button" data-font="0" style="padding:1px 9px" title="恢复默认 15px">↺</button>',
      '  </span>',
      '</div>',
      '<div class="serial-console" data-a="consoleWrap">',
      '  <div class="serial-term" data-a="termhost" hidden></div>',
      '  <div class="serial-log" data-a="console"></div>',
      '  <div class="serial-prompt" data-a="promptline">',
      '    <span class="ps">›</span>',
      '    <textarea data-a="input" rows="1" autocomplete="off" spellcheck="false"',
      '      placeholder="在此输入命令，回车发送；Tab 键直接发给设备补全" disabled></textarea>',
      '  </div>',
      '</div>'
    ].join('');

    var cons = el.querySelector('[data-a="console"]');
    var input = el.querySelector('[data-a="input"]');
    var termHost = el.querySelector('[data-a="termhost"]');
    var promptLine = el.querySelector('[data-a="promptline"]');
    var term = null, fitAddon = null, unsubFontS = null;
    // 终端缩放（同一套全局偏好）：工具条按钮 + Ctrl/Meta+滚轮，与 SSH 终端一致
    function applyFontS(px) {
      setXtermFont(term, px);
      var lab = el.querySelector('[data-font-label]');
      if (lab) lab.textContent = px + 'px';
      if (term) fitTerm();
    }
    el.addEventListener('click', function (e) {
      var t = e.target;
      var d = t && t.getAttribute && t.getAttribute('data-font');
      if (d == null) return;
      setTermFont(d === '0' ? TERM_FONT_DEFAULT : getTermFont() + (d === '-1' ? -1 : 1));
    });
    termHost.addEventListener('wheel', function (e) {
      if (!e.ctrlKey && !e.metaKey) return;
      e.preventDefault();
      setTermFont(getTermFont() + (e.deltaY < 0 ? 1 : -1));
    }, { passive: false });
    unsubFontS = subTermFont(applyFontS);
    applyFontS(getTermFont());
    var port = null, reader = null, writer = null, keepReading = false, closed = false;
    var history = [], hIdx = -1;
    var HAS = ('serial' in navigator);
    // 收发字节计数：用来区分“设备压根没回数据”和“页面没显示出来”
    var rxBytes = 0, txBytes = 0, noRxTimer = null, hintedNoRx = false;
    function paintBytes() {
      var b = q('bytes');
      if (b) b.textContent = rxBytes || txBytes ? ('已收 ' + rxBytes + ' B · 已发 ' + txBytes + ' B') : '';
    }
    // 解码器要跨 read 复用并开 stream 模式：一个多字节字符（如汉字 3 字节）
    // 很可能被拆到两次 read 里，每次新建解码器就会把它解成乱码
    var dec = null;
    function makeDecoder() {
      var name = q('enc') ? q('enc').value : 'utf-8';
      try { dec = new TextDecoder(name, { fatal: false }); }
      catch (e) { dec = new TextDecoder('utf-8', { fatal: false }); }
    }
    // 过滤 ANSI 转义序列：串口对面若是 Linux shell，会发 ESC[?2004h 这类控制码，
    // 原样显示就是 [ ?2004h 之类的“乱码”。转义序列可能被 read 截断，故用 ansiBuf 暂存
    var ansiBuf = '';
    function stripAnsi(s) {
      s = ansiBuf + s; ansiBuf = '';
      var out = '', i = 0;
      while (i < s.length) {
        if (s.charCodeAt(i) !== 0x1b) { out += s[i]; i++; continue; }
        var n = s[i + 1];
        if (n === undefined) { ansiBuf = s.slice(i); break; }
        if (n === '[') {                    // CSI：ESC [ 参数 终止符(0x40~0x7E)
          var j = i + 2, closed = false;
          while (j < s.length) {
            var cc = s.charCodeAt(j);
            if (cc >= 0x40 && cc <= 0x7e) { closed = true; j++; break; }
            j++;
          }
          if (!closed) { ansiBuf = s.slice(i); break; }
          i = j; continue;
        }
        if (n === ']') {                    // OSC：ESC ] … BEL 或 ESC \
          var k = i + 2, end = false;
          while (k < s.length) {
            if (s.charCodeAt(k) === 0x07) { k++; end = true; break; }
            if (s.charCodeAt(k) === 0x1b && s[k + 1] === '\\') { k += 2; end = true; break; }
            k++;
          }
          if (!end) { ansiBuf = s.slice(i); break; }
          i = k; continue;
        }
        // 其它转义：ESC + 若干中间字节(0x20~0x2F) + 终止字节(0x30~0x7E)
        var m = i + 1, fin = false;
        while (m < s.length) {
          var mc = s.charCodeAt(m);
          if (mc >= 0x30 && mc <= 0x7e) { fin = true; m++; break; }
          if (mc >= 0x20 && mc <= 0x2f) { m++; continue; }
          break;                            // 非法字节，按孤立 ESC 处理
        }
        if (!fin) {
          if (m >= s.length) { ansiBuf = s.slice(i); break; }   // 序列可能被截断，等下一块
          i++; continue;                                        // 孤立 ESC：丢掉它本身
        }
        i = m; continue;
      }
      return out;
    }

    function q(a) { return el.querySelector('[data-a="' + a + '"]'); }
    function setState(kind, text) {
      q('dot').className = 'sdot' + (kind ? ' ' + kind : '');
      q('state').textContent = text;
    }
    function lock(on) {
      ['port', 'baud', 'dataBits', 'stopBits', 'parity', 'flow'].forEach(function (a) { q(a).disabled = on; });
      q('connect').disabled = on;
      q('disconnect').disabled = !on;
      input.disabled = !on;
      if (on && term) term.focus();
    }
    function nearBottom() { return cons.scrollHeight - cons.scrollTop - cons.clientHeight < 40; }
    function write(text, cls) {
      if (viewMode() === 'term' && term) {
        // 终端模式下日志/提示也写进终端，否则这些信息会落在被隐藏的文本日志里
        var color = cls === 'err' ? '\x1b[31m' : (cls === 'tx' ? '\x1b[36m' : '\x1b[90m');
        term.write(color + text.replace(/\n/g, '\r\n') + '\x1b[0m');
        return;
      }
      var stick = nearBottom();
      if (q('ts').checked && cls !== 'sys') {
        var ts = document.createElement('span');
        ts.className = 'ts'; ts.textContent = stamp(); cons.appendChild(ts);
      }
      var sp = document.createElement('span');
      if (cls) sp.className = cls;
      sp.textContent = text;
      cons.appendChild(sp);
      if (stick) cons.scrollTop = cons.scrollHeight;
    }
    function sys(m) { write(m + '\n', 'sys'); }
    function viewMode() {
      var v = q('view').value;
      return (v === 'term' || v === 'hex') ? v : 'text';
    }
    // 终端模式：xterm.js 直接接管键盘，逐字符发往串口，
    // Tab / 方向键 / Ctrl+C 原样交给设备侧 shell 处理（Tab 即补全）
    function ensureTerm() {
      if (term) return true;
      if (!window.Terminal) { sys('未找到 xterm.js（vendor/xterm 缺失），已退回原始文本模式'); return false; }
      term = new window.Terminal({
        fontFamily: "'Cascadia Code','JetBrains Mono',Consolas,'Courier New',monospace",
        fontSize: getTermFont(), lineHeight: 1.25, cursorBlink: true, scrollback: 8000,
        allowProposedApi: true, convertEol: false,
        theme: {
          background: '#0f172a', foreground: '#e2e8f0', cursor: '#7dd3fc',
          selectionBackground: '#334155', black: '#1e293b', red: '#f87171',
          green: '#5eead4', yellow: '#fbbf24', blue: '#7dd3fc',
          magenta: '#c4b5fd', cyan: '#67e8f9', white: '#e2e8f0'
        }
      });
      if (window.FitAddon) { fitAddon = new window.FitAddon.FitAddon(); term.loadAddon(fitAddon); }
      // 字号读全局偏好（订阅在视图构建期已挂 applyFontS，此处创建时读一次即可）
      term.open(termHost);
      term.onData(function (d) {
        if (!writer) return;
        txBytes += d.length; paintBytes();
        try { writer.write(enc(d)).catch(function () { }); } catch (e) { }
      });
      return true;
    }
    function fitTerm() {
      if (!fitAddon || !term || termHost.hidden) return;
      var r = termHost.getBoundingClientRect();
      if (r.width < 20 || r.height < 20) return;
      try { fitAddon.fit(); } catch (e) { }
    }
    function applyMode() {
      var isTerm = (viewMode() === 'term');
      if (isTerm) {
        // 必须先让容器可见再创建终端：xterm 在 display:none 的容器里会初始化成 0 列宽
        termHost.hidden = false; cons.hidden = true; promptLine.hidden = true;
        if (!ensureTerm()) { q('view').value = 'text'; isTerm = false; }
      }
      if (!isTerm) { termHost.hidden = true; cons.hidden = false; promptLine.hidden = false; }
      if (isTerm) { setTimeout(fitTerm, 0); if (term) term.focus(); }
      else if (!input.disabled) input.focus();
    }
    function sendRaw(str) {
      if (!port || !writer) return;
      var data = enc(str);
      txBytes += data.length; paintBytes();
      try {
        writer.write(data).catch(function (e) { sys('\n发送失败：' + (e && e.message ? e.message : e)); });
      } catch (e) { sys('\n发送失败：' + (e && e.message ? e.message : e)); }
    }
    function writeBytes(u8) {
      rxBytes += u8.length;
      paintBytes();
      if (viewMode() === 'term') {
        if (term) {
          // xterm 只认 UTF-8；选了 GBK 等编码时先自行解码再写入
          var en = q('enc') ? q('enc').value : 'utf-8';
          if (en === 'utf-8') term.write(u8);
          else { if (!dec) makeDecoder(); term.write(dec.decode(u8, { stream: true })); }
        }
        return;
      }
      if (q('view').value === 'hex') {
        var a = [];
        for (var i = 0; i < u8.length; i++) a.push(u8[i].toString(16).padStart(2, '0'));
        write(a.join(' ') + ' ', '');
      } else {
        if (!dec) makeDecoder();
        var txt = dec.decode(u8, { stream: true });
        if (q('ansi').checked) txt = stripAnsi(txt);
        write(txt, '');
      }
    }
    function openOpts() {
      return {
        baudRate: parseInt(q('baud').value, 10) || 115200,
        dataBits: parseInt(q('dataBits').value, 10) || 8,
        stopBits: parseInt(q('stopBits').value, 10) || 1,
        parity: q('parity').value,
        flowControl: q('flow').value
      };
    }
    function refreshPorts() {
      if (!HAS || !navigator.serial.getPorts) return;
      Promise.resolve(navigator.serial.getPorts()).then(function (list) {
        var dl = document.getElementById(portListId);
        if (!dl) return;
        var opts = ['<option value="serial:1">连接时由系统弹窗选择</option>'];
        list.forEach(function (p, i) {
          var info = (p.getInfo && p.getInfo()) || {};
          var tag = (info.usbVendorId != null)
            ? ' · VID 0x' + info.usbVendorId.toString(16).padStart(4, '0') : '';
          opts.push('<option value="serial:' + (i + 1) + '">已授权串口 #' + (i + 1) + tag + '</option>');
        });
        dl.innerHTML = opts.join('');
      }).catch(function () { });
    }
    function resolvePort() {
      var v = (q('port').value || '').trim();
      if (!v || v === 'serial:1') return navigator.serial.requestPort();
      var m = v.match(/^serial:(\d+)$/);
      if (m) {
        var idx = parseInt(m[1], 10) - 1;
        return Promise.resolve(navigator.serial.getPorts()).then(function (list) {
          if (!list[idx]) throw new Error('未找到已授权串口 #' + m[1] + '，请选择 serial:1 后重新连接');
          return list[idx];
        });
      }
      return navigator.serial.requestPort();
    }
    async function connect() {
      if (!HAS) {
        setState('err', '不支持');
        sys('当前浏览器不支持 Web Serial：请用 Chrome / Edge，并以 file://、https 或 localhost 打开本页。');
        return;
      }
      try {
        setState('busy', '正在打开串口…');
        port = await resolvePort();
        await port.open(openOpts());
        keepReading = true;
        writer = port.writable.getWriter();
        lock(true);
        setState('on', '已连接 · ' + (q('baud').value || 115200) + 'bps');
        rxBytes = 0; txBytes = 0; hintedNoRx = false; paintBytes();
        dec = null;   // 新连接重新开始解码，丢掉上一次的残留字节
        ansiBuf = '';
        sys('本机串口已打开 · ' + (q('baud').value || 115200) + ' ' +
          q('dataBits').value + q('parity').value.charAt(0).toUpperCase() + q('stopBits').value);
        readLoop();
        refreshPorts();
      } catch (e) {
        port = null;
        setState('err', '打开失败');
        sys('打开串口失败：' + (e && e.message ? e.message : e));
      }
    }
    async function readLoop() {
      while (port && port.readable && keepReading && !closed) {
        reader = port.readable.getReader();
        try {
          while (true) {
            var r = await reader.read();
            if (r.done) break;
            if (r.value && r.value.length) writeBytes(r.value);
          }
        } catch (e) {
          if (keepReading && !closed) sys('\n读取中断：' + (e && e.message ? e.message : e));
        } finally {
          try { reader.releaseLock(); } catch (e) { }
          reader = null;
        }
      }
    }
    async function disconnect(silent) {
      keepReading = false;
      try { if (reader) await reader.cancel(); } catch (e) { }
      try { if (writer) writer.releaseLock(); } catch (e) { }
      writer = null;
      try { if (port && port.readable) await port.close(); } catch (e) { }
      port = null;
      if (closed) return;
      lock(false);
      setState('', '未连接');
      if (!silent) sys('\n串口已断开。');
      refreshPorts();
    }
    function send() {
      if (!port || !writer) return;
      var t = input.value, eol = q('eol').value;
      var tail = eol === 'crlf' ? '\r\n' : eol === 'lf' ? '\n' : eol === 'cr' ? '\r' : '';
      var data = enc(t + tail);
      txBytes += data.length;
      paintBytes();
      // 立即回显：不等 USB 写完成回调，避免设备不应答时看起来“没发出去”
      if (q('echo').checked) write('▸ ' + t + '\n', 'tx');
      if (t.trim() && history[0] !== t) { history.unshift(t); if (history.length > 60) history.pop(); }
      hIdx = -1;
      input.value = '';
      try {
        writer.write(data).catch(function (e) {
          sys('\n发送失败：' + (e && e.message ? e.message : e));
        });
      } catch (e) {
        sys('\n发送失败：' + (e && e.message ? e.message : e));
        return;
      }
      // 发完 1.5 秒内一个字节都没回来，提示一次排查方向（只提示一次，避免刷屏）
      var snap = rxBytes;
      clearTimeout(noRxTimer);
      noRxTimer = setTimeout(function () {
        if (closed || rxBytes !== snap || hintedNoRx) return;
        hintedNoRx = true;
        sys('已发出 ' + data.length + ' 字节，1.5 秒内未收到任何数据。若设备本该有回应，请检查：' +
          '① 波特率是否与设备一致；② 行尾符（多数设备需要 CRLF 或 LF）；' +
          '③ TX/RX 是否接反；④ 该设备是否需要二进制协议帧而非文本命令。');
      }, 1500);
    }

    el.addEventListener('click', function (e) {
      var a = e.target.getAttribute && e.target.getAttribute('data-a');
      if (a === 'connect') connect();
      else if (a === 'disconnect') disconnect(false);
      else if (a === 'clear') { cons.innerHTML = ''; if (term) term.clear(); }
      else if (a === 'scan') refreshPorts();
    });
    // 点黑色区域任意位置即可开始输入（MobaXterm 手感）
    q('consoleWrap').addEventListener('mousedown', function (e) {
      if (viewMode() === 'term') { if (term) term.focus(); return; }
      if (e.target === input) return;
      if (!input.disabled) setTimeout(function () { input.focus(); }, 0);
    });
    q('view').addEventListener('change', function () { lsSet('eai_serial_view', q('view').value); applyMode(); });
    q('echo').addEventListener('change', function () { lsSet('eai_serial_echo', q('echo').checked); });
    q('enc').addEventListener('change', function () { lsSet('eai_serial_enc', q('enc').value); dec = null; });
    q('ansi').addEventListener('change', function () { lsSet('eai_serial_ansi', q('ansi').checked); ansiBuf = ''; });
    input.addEventListener('keydown', function (e) {
      if (e.key === 'Enter') { e.preventDefault(); send(); return; }
      // Tab 不做焦点切换，直接发给设备（远端 shell 会用它做补全）
      if (e.key === 'Tab') { e.preventDefault(); sendRaw('\t'); return; }
      if (e.ctrlKey && (e.key === 'c' || e.key === 'C')) {
        e.preventDefault(); sendRaw('\x03'); input.value = ''; return;   // Ctrl+C：中断设备当前命令
      }
      if (e.key === 'ArrowUp') {
        e.preventDefault();
        if (history.length && hIdx < history.length - 1) { hIdx++; this.value = history[hIdx]; }
      } else if (e.key === 'ArrowDown') {
        e.preventDefault();
        if (hIdx > 0) { hIdx--; this.value = history[hIdx]; }
        else { hIdx = -1; this.value = ''; }
      }
    });
    if (HAS && navigator.serial.addEventListener) {
      navigator.serial.addEventListener('disconnect', function (e) {
        if (e.target === port) {
          sys('\n设备已被拔出，连接中断。');
          disconnect(true);
        }
      });
    }

    return {
      el: el, kind: 'serial',
      title: cfg.title || '本机串口',
      mount: function () {
        q('echo').checked = (lsGet('eai_serial_echo', true) !== false);
        q('enc').value = lsGet('eai_serial_enc', 'utf-8');
        q('ansi').checked = (lsGet('eai_serial_ansi', true) !== false);
        q('view').value = lsGet('eai_serial_view', 'term');
        applyMode();
        if (cfg.port) q('port').value = cfg.port;
        q('baud').value = cfg.baud || 115200;
        if (cfg.dataBits) q('dataBits').value = String(cfg.dataBits);
        if (cfg.stopBits) q('stopBits').value = String(cfg.stopBits);
        if (cfg.parity) q('parity').value = cfg.parity;
        if (!HAS) {
          setState('err', '不支持');
          sys('当前浏览器不支持 Web Serial（Firefox / Safari 未实现）：请用 Chrome / Edge 打开。');
          q('connect').disabled = true;
          return;
        }
        sys('提示：本机 USB 直连，数据不经过任何网络服务。首次连接请选 serial:1，由系统弹窗授权端口。');
        refreshPorts();
        if (cfg.autoconnect) connect();
      },
      show: function () {
        if (viewMode() === 'term') { setTimeout(fitTerm, 0); if (term) term.focus(); }
        else if (!input.disabled) input.focus();
        else q('connect').focus();
      },
      hide: function () { },
      refit: function () { fitTerm(); },
      close: function () {
        closed = true;
        disconnect(true);
        if (unsubFontS) unsubFontS();
        try { if (term) term.dispose(); } catch (e) { }
        term = null; fitAddon = null;
      }
    };
  }

  /* ==========================================================
     视图：远程桌面（noVNC，直连设备，不经桥接服务）
     ========================================================== */
  function VncView(cfg) {
    var el = document.createElement('div');
    el.className = 'view vncview';
    var host = cfg.host || '';
    var port = cfg.port || DEF_VNC_PORT;
    var url = 'http://' + host + ':' + port + '/vnc.html?autoconnect=1&resize=scale&reconnect=1';

    el.innerHTML = [
      '<div class="tb">',
      '  <span class="mono" style="font-size:12px;color:#64748b" data-a="url"></span>',
      '  <span class="flex"></span>',
      '  <button class="tbbtn" data-a="reload">重载</button>',
      '  <button class="tbbtn" data-a="open">在新窗口打开</button>',
      '</div>',
      '<iframe class="vnc-frame" data-a="frame" allow="clipboard-read; clipboard-write; fullscreen; picture-in-picture"></iframe>'
    ].join('');
    el.querySelector('[data-a="url"]').textContent = url;
    var frame = el.querySelector('[data-a="frame"]');
    frame.src = url;

    el.addEventListener('click', function (e) {
      var a = e.target.getAttribute && e.target.getAttribute('data-a');
      if (a === 'reload') frame.src = url;
      else if (a === 'open') window.open(url, '_blank', 'noopener');
    });

    return {
      el: el, kind: 'vnc',
      title: cfg.title || ('桌面 · ' + host + ':' + port),
      mount: function () { },
      show: function () { },
      hide: function () { },
      refit: function () { },
      close: function () { }
    };
  }

  /* ==========================================================
     标签 / 分屏
     ========================================================== */
  function makeView(kind, cfg, conn) {
    if (kind === 'term') return TermView(cfg, conn);
    if (kind === 'sftp') return SftpView(cfg, conn);
    if (kind === 'serial') return LocalSerialView(cfg);
    if (kind === 'vnc') return VncView(cfg);
    throw new Error('未知视图类型：' + kind);
  }

  function openTab(kind, cfg, conn, paneId) {
    cfg = cfg || {};
    paneId = paneId || S.activePane;
    if (paneId === 'R' && !S.split) { S.split = true; applySplit(); }
    var id = 'tab' + (++S.seq);
    var view = makeView(kind, cfg, conn);
    var tab = {
      id: id, kind: kind, title: cfg.title || view.title,
      cfg: cfg, conn: conn, view: view, sessionId: cfg.sessionId || null
    };
    view.el.classList.add('on');
    $('views' + paneId).appendChild(view.el);
    S.panes[paneId].tabs.push(tab);
    S.panes[paneId].active = id;
    S.activePane = paneId;
    try { view.mount(); } catch (e) { console.error(e); }
    renderPane(paneId);
    renderSide();
    refreshAnimationByTab();
    setStatus('已打开 ' + KIND_NAME[kind] + ' · ' + tab.title, 'on');
    return tab;
  }

  /* ---------- 标签拖拽排序（标准 HTML5 DnD 三段式：dragstart 记 id → dragover 必 preventDefault 并用中点定插位 → drop 先改数据数组再重渲染） ---------- */
  var dragTabId = null, dropRef = null;
  function clearDropHints() {
    var list = document.querySelectorAll('.tab.dropl,.tab.dropr');
    for (var i = 0; i < list.length; i++) { list[i].classList.remove('dropl'); list[i].classList.remove('dropr'); }
  }
  function reorderTab(id, toPane, refId, after) {
    var from = paneOf(id);
    if (!from || !refId || id === refId) return;
    if (from !== toPane) moveTab(id);   // 跨栏：先迁到对侧（自动开分屏）
    var arr = S.panes[toPane].tabs;
    var fi = arr.findIndex(function (x) { return x.id === id; });
    var ri = arr.findIndex(function (x) { return x.id === refId; });
    if (fi < 0 || ri < 0) { renderPane(toPane); return; }
    var item = arr.splice(fi, 1)[0];
    ri = arr.findIndex(function (x) { return x.id === refId; }); // splice 后重定位参考项
    arr.splice(ri + (after ? 1 : 0), 0, item);
    renderPane(toPane);
    setStatus(from === toPane ? '已调整标签顺序' : '已移动标签并排序');
  }

  function renderPane(paneId) {
    var p = S.panes[paneId], bar = $('tabbar' + paneId);
    bar.innerHTML = '';
    if (!p.tabs.length) {
      var ph = document.createElement('div');
      ph.className = 'tab empty';
      ph.textContent = paneId === 'L' ? '还没有标签，从左侧会话列表双击或点「＋ 新建会话」' : '右侧分屏为空';
      bar.appendChild(ph);
    }
    p.tabs.forEach(function (t) {
      var el = document.createElement('div');
      el.className = 'tab' + (p.active === t.id ? ' active' : '');
      el.innerHTML = '<span class="tico"></span><span class="tname"></span><span class="tx" title="关闭">✕</span>';
      el.querySelector('.tico').textContent = ICON[t.kind] || '•';
      el.querySelector('.tname').textContent = t.title;
      el.addEventListener('click', function (e) {
        if (e.target.classList.contains('tx')) { e.stopPropagation(); closeTab(t.id); return; }
        activate(t.id);
      });
      el.addEventListener('dblclick', function () { renameTab(t.id); });
      el.addEventListener('contextmenu', function (e) {
        e.preventDefault();
        e.stopPropagation();
        var pid = paneOf(t.id);
        var items = [
          { label: '重命名', fn: function () { renameTab(t.id); } },
          { label: '弹出为独立窗口', fn: function () { popOut(t); } },
          { label: pid === 'L' ? '移到右侧分屏' : '移到左侧', fn: function () { moveTab(t.id); } },
          { sep: true },
          { label: '关闭', fn: function () { closeTab(t.id); } },
          { label: '关闭其它', fn: function () { closeOthers(t.id); } }
        ];
        showMenu(e.clientX, e.clientY, items);
      });
      /* 标签拖拽排序：按住标签可在栏内换位，拖到另一栏则跨栏并插入该位置 */
      el.draggable = true;
      el.addEventListener('dragstart', function (e) {
        dragTabId = t.id;
        el.classList.add('dragging');
        try { e.dataTransfer.setData('text/plain', t.id); e.dataTransfer.effectAllowed = 'move'; } catch (err) { }
      });
      el.addEventListener('dragend', function () {
        el.classList.remove('dragging');
        clearDropHints();
        dragTabId = null; dropRef = null;
      });
      el.addEventListener('dragover', function (e) {
        if (!dragTabId || dragTabId === t.id) return;
        e.preventDefault();
        try { e.dataTransfer.dropEffect = 'move'; } catch (err) { }
        clearDropHints();
        var r = el.getBoundingClientRect();
        var leftSide = (e.clientX - r.left) < r.width / 2;
        el.classList.add(leftSide ? 'dropl' : 'dropr');
        dropRef = { pane: paneId, id: t.id, after: !leftSide };
      });
      el.addEventListener('dragleave', function () { el.classList.remove('dropl'); el.classList.remove('dropr'); });
      el.addEventListener('drop', function (e) {
        e.preventDefault(); e.stopPropagation();
        var dragId = '';
        try { dragId = e.dataTransfer.getData('text/plain'); } catch (err) { }
        dragId = dragId || dragTabId;
        if (dragId && dropRef) reorderTab(dragId, dropRef.pane, dropRef.id, dropRef.after);
        clearDropHints(); dragTabId = null; dropRef = null;
      });
      bar.appendChild(el);
    });
    var t = p.tabs.filter(function (x) { return x.id === p.active; })[0];
    p.tabs.forEach(function (x) { x.view.el.classList.toggle('on', !!t && x.id === t.id); });
  }

  function activate(id) {
    var paneId = paneOf(id);
    if (!paneId) return;
    S.panes[paneId].active = id;
    S.activePane = paneId;
    renderPane(paneId);
    renderSide();
    refreshAnimationByTab();
    var t = findTab(id);
    if (t) syncLabelsForTab(t);
    if (t && t.view.show) { try { t.view.show(); } catch (e) { } }
    if (t && t.view.refit) setTimeout(t.view.refit, 30);
  }

  function closeTab(id) {
    var paneId = paneOf(id);
    if (!paneId) return;
    var p = S.panes[paneId];
    var idx = p.tabs.findIndex(function (x) { return x.id === id; });
    if (idx < 0) return;
    var t = p.tabs[idx];
    try { if (t.view.close) t.view.close(); } catch (e) { }
    try { t.view.el.remove(); } catch (e) { }
    p.tabs.splice(idx, 1);
    if (p.active === id) p.active = p.tabs.length ? p.tabs[Math.max(0, idx - 1)].id : null;
    renderPane(paneId);
    renderSide();
    var nt = p.active && findTab(p.active);
    if (nt && nt.view.refit) setTimeout(nt.view.refit, 30);
    refreshAnimationByTab();
    if (!allTabs().length) setStatus('没有打开的标签', '');
  }

  function closeOthers(keepId) {
    var paneId = paneOf(keepId);
    if (!paneId) return;
    S.panes[paneId].tabs.slice().forEach(function (t) {
      if (t.id !== keepId) closeTab(t.id);
    });
  }

  function renameTab(id) {
    var t = findTab(id);
    if (!t) return;
    var name = prompt('标签名称：', t.title);
    if (name == null) return;
    t.title = name.trim() || t.title;
    renderPane(paneOf(id));
    renderSide();
  }

  function moveTab(id) {
    var from = paneOf(id), to = from === 'L' ? 'R' : 'L';
    if (!from) return;
    if (to === 'R' && !S.split) { S.split = true; applySplit(); }
    var t = findTab(id);
    var fp = S.panes[from];
    fp.tabs.splice(fp.tabs.findIndex(function (x) { return x.id === id; }), 1);
    if (fp.active === id) fp.active = fp.tabs.length ? fp.tabs[0].id : null;
    $('views' + to).appendChild(t.view.el);
    S.panes[to].tabs.push(t);
    S.panes[to].active = id;
    renderPane(from);
    renderPane(to);
    renderSide();
    refreshAnimationByTab();
    setTimeout(function () { if (t.view.refit) t.view.refit(); }, 30);
  }

  function applySplit() {
    $('split').classList.toggle('two', S.split);
    $('paneR').hidden = !S.split;
    $('btnSplit').classList.toggle('on', S.split);
    $('btnSplit').textContent = S.split ? '取消分屏' : '分屏';
    setTimeout(function () {
      allTabs().forEach(function (t) { if (t.view.refit) t.view.refit(); });
    }, 60);
  }

  function popOut(tab) {
    var payload = {
      kind: tab.kind, title: tab.title, cfg: tab.cfg,
      conn: tab.conn ? {
        key: tab.conn.key, base: tab.conn.base, wsBase: tab.conn.wsBase, token: tab.conn.token,
        user: tab.conn.user, sshHost: tab.conn.sshHost, sshPort: tab.conn.sshPort,
        homedir: tab.conn.homedir, addr: tab.conn.addr, sessionId: tab.conn.sessionId
      } : null
    };
    var url = location.href.split('#')[0] + '?popout=1#pop=' + b64url(JSON.stringify(payload));
    window.open(url, '_blank', 'width=1080,height=720');
  }

  /* ==========================================================
     打开会话
     ========================================================== */
  function touchSession(sess) {
    sess.at = Date.now() / 1000;
    upsertSession(sess);
    renderSide();
  }

  function openSession(sess, paneId) {
    if (!sess) return;
    // 本机串口直连：不走网络，无需 IP / 登录
    if (sess.kind === 'serial') {
      if (!sess.name) sess.name = '本机串口 @' + (sess.baud || 115200);
      openTab('serial', {
        port: sess.serialPort, baud: sess.baud,
        dataBits: sess.dataBits, stopBits: sess.stopBits, parity: sess.parity,
        autoconnect: !!sess.remember,
        title: sessTitle(sess), sessionId: sess.id
      }, null, paneId);
      touchSession(sess);
      return;
    }
    if (sess.kind === 'vnc') {
      // 没存 IP 时复用上次连接过的设备地址（同网段设备通常就是同一台）
      if (!sess.addr && S.prefs && S.prefs.addr) sess.addr = S.prefs.addr;
      if (!sess.addr) { openModal(sess, '请补充设备地址（IP）'); return; }
      if (!sess.name) sess.name = sess.addr;
      openTab('vnc', {
        host: sess.addr, port: sess.vncPort || DEF_VNC_PORT,
        title: sessTitle(sess), sessionId: sess.id
      }, null, paneId);
      touchSession(sess);
      return;
    }
    if (!sess.addr || !sess.user || !sess.password) {
      openModal(sess, sess.password ? '请补充连接信息' : '请输入密码后连接（未记住密码）');
      return;
    }
    var conn = S.conns[bridgeKey(sess)];
    if (conn && conn.token) { startTab(sess, conn, paneId); return; }

    setStatus('正在连接 ' + sessTitle(sess) + ' …', 'busy');
    ensureHelper().then(function (on) {
      if (!on) {
        if (helperReady() !== 1) {
          // 从未初始化过：不发协议、不弹系统框，直接给一次性分步引导
          setStatus('本机助手未初始化', 'err');
          showGuide('这台电脑还没初始化过本机助手，先按引导操作一次即可（之后全自动）。');
          throw new Error('本机助手未初始化，已打开初始化引导');
        }
        var msg = '未检测到本机助手（' + bridgeBase() + '）。' +
          (OS.reg ? OS.reg + '（只需一次，之后点连接会自动拉起助手）；' : '') +
          OS.start + '，再点「连接」。';
        setStatus('本机助手未启动', 'err');
        throw new Error(msg);
      }
      return login(sess);
    }).then(function (c) {
      savePrefs(sess);
      startTab(sess, c, paneId);
    }).catch(function (e) {
      setStatus('连接失败：' + e.message, 'err');
      toast('连接 ' + sessTitle(sess) + ' 失败：' + e.message, 'bad');
      var gm = $('guideModal');
      if (gm && !gm.hidden) return;   // 引导弹窗优先，不叠弹会话框
      openModal(sess, e.message);
    });
  }

  function startTab(sess, conn, paneId) {
    // 连接成功后若未自定义会话名称，默认用 用户名@IP
    if (!sess.name) sess.name = (conn.user ? conn.user + '@' : '') + (conn.addr || sess.addr || '?');
    var cfg = { title: sessTitle(sess), sessionId: sess.id };
    if (sess.kind === 'term') cfg.startDir = sess.startDir || '';
    if (sess.kind === 'sftp') cfg.startDir = sess.startDir || conn.homedir || '/';
    openTab(sess.kind, cfg, conn, paneId);
    touchSession(sess);
    if (conn) {
      setUserLabel(conn.user + '@' + conn.addr);
      setDeviceLabel('本机助手 ' + (conn.base || '').replace(/^https?:\/\//, '') +
        ' → ' + conn.addr + ':' + (conn.sshPort || DEF_SSH_PORT) + ' · ' + sessTitle(sess));
      loadSysInfo(conn);
    }
  }

  /* ---------------------------------------------------------- 侧栏：会话列表 */
  function renderSide() {
    var box = $('sideList');
    box.innerHTML = '';
    if (!S.sessions.length) {
      box.innerHTML = '<div class="side-empty">还没有会话。<br>' +
        '点「＋ 新建」填写设备 IP 与端口，连接成功后会自动保存在这里。</div>';
      return;
    }
    var openIds = {};
    allTabs().forEach(function (t) { if (t.sessionId) openIds[t.sessionId] = true; });

    KINDS.forEach(function (k) {
      var list = S.sessions.filter(function (s) { return s.kind === k; })
        .sort(function (a, b) { return (b.at || 0) - (a.at || 0); });
      if (!list.length) return;

      var g = document.createElement('div');
      g.className = 'side-group';
      g.innerHTML = '<span class="sg-ico"></span><span class="sg-name"></span>';
      g.querySelector('.sg-ico').textContent = ICON[k];
      g.querySelector('.sg-name').textContent = KIND_NAME[k];
      box.appendChild(g);

      list.forEach(function (s) {
        var el = document.createElement('div');
        el.className = 'side-item' + (openIds[s.id] ? ' on' : '');
        el.innerHTML = '<span class="si-ico"></span><span class="si-txt">' +
          '<span class="si-name"></span><span class="si-sub"></span></span>';
        el.querySelector('.si-ico').textContent = ICON[s.kind];
        el.querySelector('.si-name').textContent = sessTitle(s);
        el.querySelector('.si-sub').textContent = sessSub(s);
        el.title = sessTitle(s) + '\n' + sessSub(s);
        el.addEventListener('click', function () { openSession(s); });
        el.addEventListener('dblclick', function () { openSession(s, 'R'); });
        el.addEventListener('contextmenu', function (e) {
          e.preventDefault();
          e.stopPropagation();
          showMenu(e.clientX, e.clientY, [
            { label: '打开', fn: function () { openSession(s); } },
            { label: '在右侧分屏打开', fn: function () { openSession(s, 'R'); } },
            { sep: true },
            { label: '编辑', fn: function () { openModal(s); } },
            { label: '复制一份', fn: function () {
              var c = clone(s);
              c.id = newId();
              c.name = (s.name || sessTitle(s)) + ' 副本';
              c.at = Date.now() / 1000;
              upsertSession(c);
              renderSide();
            } },
            { sep: true },
            { label: '删除', danger: true, fn: function () {
              if (!confirm('删除会话「' + sessTitle(s) + '」？')) return;
              removeSession(s.id);
              renderSide();
            } }
          ]);
        });
        box.appendChild(el);
      });
    });
  }

  /* ==========================================================
     会话编辑弹窗
     ========================================================== */
  var draft = null;        // 正在编辑的会话草稿
  var editing = false;     // true = 编辑已有会话

  function defaultDraft(kind) {
    var s = blankSession(kind);
    var p = S.prefs || {};
    if (S.sameOrigin) { s.addr = S.sameOrigin.addr; }
    else if (p.addr) { s.addr = p.addr; }
    if (p.user) s.user = p.user;
    return s;
  }

  function openModal(sess, errMsg) {
    if (sess && typeof sess === 'object' && sess.id) {
      editing = true;
      draft = clone(sess);
      if (!draft.password && !draft.remember) draft.remember = true;
    } else {
      editing = false;
      draft = defaultDraft(typeof sess === 'string' ? sess : 'term');
      if (typeof sess === 'string') draft.kind = sess;
    }
    if (typeof sess === 'string' && !editing) draft.kind = sess;
    $('mTitle').textContent = editing ? '编辑会话' : '新建会话';
    $('mOk').textContent = editing ? '保存并连接' : '连接';
    $('mErr').textContent = typeof errMsg === 'string' ? errMsg : '';
    syncTypeCards();
    $('modalNew').hidden = false;
    renderModalBody();
    if (draft.kind === 'term' || draft.kind === 'sftp') refreshHelperHint();
    setTimeout(function () {
      var first = $('mBody').querySelector('[data-m="addr"]') || $('mBody').querySelector('input');
      if (first) first.focus();
    }, 30);
  }
  function closeModal() { $('modalNew').hidden = true; }
  function syncTypeCards() {
    Array.prototype.forEach.call($('mTypes').children, function (b) {
      b.classList.toggle('on', b.dataset.kind === draft.kind);
    });
  }

  function baudOptions(cur) {
    var c = String(cur == null || cur === '' ? 115200 : cur);
    var html = '';
    var found = BAUD_LIST.some(function (b) { return String(b) === c; });
    if (!found && !isNaN(parseInt(c, 10)) && String(parseInt(c, 10)) === c) {
      html += '<option value="' + esc(c) + '" selected>' + esc(c) + '（已保存）</option>';
    }
    html += BAUD_LIST.map(function (b) {
      return '<option value="' + b + '"' + (String(b) === c ? ' selected' : '') + '>' + b + '</option>';
    }).join('');
    return html;
  }

  function renderModalBody() {
    var body = $('mBody');
    var k = draft.kind;
    var html = '';

    if (k === 'term' || k === 'sftp') {
      html += '<div class="fgrid">' +
        '<div class="gf"><label>设备地址（IP）</label>' +
        '<input data-m="addr" value="' + esc(draft.addr) + '" placeholder="192.168.1.100" spellcheck="false" autocomplete="off"></div>' +
        '<div class="gf narrow"><label>SSH 端口</label>' +
        '<input data-m="sshPort" value="' + esc(draft.sshPort || DEF_SSH_PORT) + '" inputmode="numeric" placeholder="22"></div>' +
        '</div>' +
        '<div class="fgrid">' +
        '<div class="gf"><label>用户名</label>' +
        '<input data-m="user" value="' + esc(draft.user) + '" placeholder="iflytek" spellcheck="false" autocomplete="username"></div>' +
        '<div class="gf"><label>密码</label>' +
        '<input data-m="password" type="password" value="' + esc(draft.password) + '" autocomplete="current-password"></div>' +
        '</div>' +
        '<div class="fgrid one">' +
        '<div class="gf"><label>起始目录</label>' +
        '<input data-m="startDir" value="' + esc(draft.startDir) + '" placeholder="留空 = 登录后进入家目录" spellcheck="false"></div>' +
        '</div>' +
        '<label class="chk"><input type="checkbox" data-m="remember"' + (draft.remember ? ' checked' : '') +
        '> 记住密码（保存在本机浏览器，换电脑需重新输入）</label>' +
        '<details class="fadv"><summary>高级：本机助手端口</summary>' +
        '<div class="fgrid one">' +
        '<div class="gf narrow"><label>助手端口</label>' +
        '<input data-m="helperPort" value="' + helperPort() + '" inputmode="numeric" placeholder="8100"></div>' +
        '</div></details>' +
        '<div class="frow">' +
        '<button class="btn sm" id="mStartHelper" type="button">启动本机助手</button>' +
        '<span class="fhint" id="mHelperHint">检测中…</span>' +
        '</div>' +
        '<div class="fnote">浏览器只连<b>本机</b>助手 <code>start_helper.py</code>（127.0.0.1，默认端口 8100，' +
        '仅监听回环地址），再由助手 SSH 登录上面填写的设备——<b>设备端不用安装任何服务</b>。</div>';
    } else if (k === 'serial') {
      html += '<div class="fgrid">' +
        '<div class="gf"><label>串口设备</label>' +
        '<select data-m="serialPort" id="mPortSel"><option value="">— 点「选择新串口…」选择 —</option></select></div>' +
        '<div class="gf"><label>波特率</label>' +
        '<input data-m="baud" list="mBaudList" inputmode="numeric" value="' + esc(draft.baud || 115200) +
        '" placeholder="115200" spellcheck="false" autocomplete="off">' +
        '<datalist id="mBaudList">' + baudOptions(draft.baud || 115200) + '</datalist></div>' +
        '</div>';
      html += '<div class="frow">' +
        '<button class="btn sm" id="mPickPort" type="button">选择新串口…</button>' +
        '<button class="btn sm" id="mScanPorts" type="button">刷新串口列表</button>' +
        '<span class="fhint" id="mPortHint">首次使用点「选择新串口…」由系统弹窗授权，之后即可在下拉中选择；' +
        '波特率点击输入框清空后可下拉选全部档位，也可直接输入任意数值。</span>' +
        '</div>';
      html += '<label class="chk"><input type="checkbox" data-m="remember"' + (draft.remember ? ' checked' : '') +
        '> 自动连接：打开会话时直接选择上次的系统串口，不再弹窗</label>';
      html += '<div class="fnote">数据线插在<b>本机</b>：浏览器直接读写（Web Serial），仅限 Chrome / Edge；' +
        '<b>不需要</b>设备 IP、端口与账号密码。</div>';
      html += '<details class="fadv"><summary>高级：串口参数</summary>' +
        '<div class="fgrid three">' +
        '<div class="gf narrow"><label>数据位</label><select data-m="dataBits">' +
        '<option' + (String(draft.dataBits) === '8' ? ' selected' : '') + '>8</option>' +
        '<option' + (String(draft.dataBits) === '7' ? ' selected' : '') + '>7</option></select></div>' +
        '<div class="gf narrow"><label>停止位</label><select data-m="stopBits">' +
        '<option' + (String(draft.stopBits) === '1' ? ' selected' : '') + '>1</option>' +
        '<option' + (String(draft.stopBits) === '2' ? ' selected' : '') + '>2</option></select></div>' +
        '<div class="gf narrow"><label>校验</label><select data-m="parity">' +
        '<option value="none"' + (draft.parity === 'none' ? ' selected' : '') + '>无</option>' +
        '<option value="even"' + (draft.parity === 'even' ? ' selected' : '') + '>偶</option>' +
        '<option value="odd"' + (draft.parity === 'odd' ? ' selected' : '') + '>奇</option></select></div>' +
        '</div></details>';
    } else {
      html += '<div class="fgrid one">' +
        '<div class="gf narrow"><label>noVNC 端口</label>' +
        '<input data-m="vncPort" value="' + esc(draft.vncPort || DEF_VNC_PORT) + '" inputmode="numeric" placeholder="6080"></div>' +
        '</div>' +
        '<div class="fgrid one">' +
        '<div class="gf"><label>设备地址（IP）</label>' +
        '<input data-m="addr" value="' + esc(draft.addr) + '" placeholder="192.168.1.100" spellcheck="false" autocomplete="off"></div>' +
        '</div>' +
        '<div class="fnote">远程桌面走设备上已有的 VNC 服务：websockify 把 VNC 的 5900 转成 WebSocket，' +
        '再由 noVNC 网页渲染。它不经过桥接服务，因此不需要账号密码。</div>';
    }
    body.innerHTML = html;
    if (k === 'term' || k === 'sftp') {
      refreshHelperHint();
    }
    if (k === 'serial') {
      refreshPortSelect();
      var baudEl = body.querySelector('[data-m="baud"]');
      if (baudEl) {
        // Chrome 的 datalist 会按输入框当前值过滤；聚焦先清空以便展示全部档位
        baudEl.addEventListener('focus', function () {
          this.setAttribute('data-saved', this.value);
          this.value = '';
        });
        baudEl.addEventListener('blur', function () {
          if (!this.value) this.value = this.getAttribute('data-saved') || '115200';
        });
      }
    }
  }

  function readModal() {
    Array.prototype.forEach.call($('mBody').querySelectorAll('[data-m]'), function (el) {
      var k = el.dataset.m;
      draft[k] = el.type === 'checkbox' ? el.checked : el.value.trim();
      if (k === 'serialPort') draft[k] = el.value; // 串口值原样保存，不做 trim 之外处理
    });
    return draft;
  }

  function modalErr(msg) { $('mErr').textContent = msg; return false; }

  /* 本机助手：弹窗内的状态提示与一键启动 */
  function refreshHelperHint() {
    var hint = $('mHelperHint');
    if (!hint) return;
    hint.textContent = '正在检测本机助手 ' + bridgeBase() + ' …';
    probeHelper().then(function (ok) {
      var h = $('mHelperHint');
      if (!h) return;
      h.textContent = ok
        ? '✓ 本机助手运行中（' + bridgeBase() + '），直接点「连接」即可。'
        : '✗ 未检测到助手 —— 点「启动本机助手」' +
          (OS.reg ? '；若点了没反应，' + OS.reg : '') +
          '；或' + OS.start + '。';
    });
  }

  function startHelperFromModal() {
    var hint = $('mHelperHint');
    // 先应用「高级：助手端口」里手填的端口
    var pi = $('mBody').querySelector('[data-m="helperPort"]');
    var hp = pi ? parseInt(pi.value, 10) : 0;
    if (pi && pi.value.trim() !== '') {
      if (!hp || hp < 1 || hp > 65535) {
        if (hint) hint.textContent = '✗ 助手端口只能是 1~65535 的数字（如 8100）';
        return;
      }
      if (hp !== helperPort()) {
        S.helper.port = hp;
        S.helper.online = false;
        S.helper.launched = false;
        lsSet('eai_helper_port', hp);
      }
    }
    // 协议尚未在本机注册 —— 发协议只会让系统弹「选择应用」框，直接转到初始化引导
    if (helperReady() !== 1) {
      if (hint) hint.textContent = '本机协议还没注册，正在打开初始化引导 …';
      showGuide('本机助手协议还没在这台电脑注册过，先按引导操作一次即可。');
      return;
    }
    setStatus('正在启动本机助手 …', 'busy');
    launchHelperOnce(true);
    if (hint) {
      hint.textContent = '已发出启动请求 —— 浏览器若弹出「外部协议」询问请选择允许；' +
        '首次运行需安装依赖，约 1~2 分钟…';
    }
    waitForHelper(15000).then(function (ok) {
      if (ok) { S.helper.online = true; S.helper.launched = false; markHelperReady(); }
      refreshHelperHint();
      if (ok) {
        setStatus('本机助手已就绪', '');
        toast('本机助手已启动：' + bridgeBase());
      } else {
        setStatus('助手仍未启动', 'err');
        var h2 = $('mHelperHint');
        if (h2) {
          var steps = [];
          if (OS.reg) steps.push(OS.reg);
          steps.push('浏览器弹「外部协议 / 打开方式」询问时选允许、勾选始终允许（若始终没弹，说明本平台协议还没注册）');
          steps.push('首次运行装依赖需 1~2 分钟，稍等再点一次即可点亮');
          steps.push(OS.start);
          var nums = ['①', '②', '③', '④'];
          h2.textContent = '✗ 启动未成功，按顺序排查：' + steps.map(function (s, i) {
            return nums[i] + ' ' + s;
          }).join('；') + '。';
        }
        toast('助手未启动：' +
          (OS.reg ? '多数情况是还没在本平台注册启动协议；' : '') + OS.start + '。', 'bad');
        showGuide('发出的启动请求没有生效（协议多半还没注册），按引导操作一次即可。');
      }
    });
  }

  /* ==========================================================
     本机助手 · 一次性初始化引导
     全新环境绝不发 eai-helper://（未注册就发会弹系统「选择应用」
     框，普通用户无从下手）：改为 路径 + 分步操作 + 成功判据 +
     重试检测，完成一次后全程自动。
     ========================================================== */
    function localWorkspaceUrl() {
        return bridgeBase() + '/workspace.html';
    }
    /* Android（Termux）检测：平台判定目前只认 win/mac/linux，Android UA/Linux(Termux)
       会被误判为 linux 桌面，引导会给出 xdg-mime/系统注册等在 Termux 里无效的步骤。 */
    function isTermux() {
        return /\bAndroid\b/i.test(navigator.userAgent || '') ||
            /\bcom\.termux\b/i.test(navigator.userAgent || '');
    }
    /* 桥接服务是否同源托管当前页面（页面以 http://<host>:<helper端口> 打开时即在线） */
    function sameOriginHelperUp() {
        if (location.protocol !== 'http:') return false;
        var hp = parseInt(location.port || '80', 10);
        if (!hp) hp = 80;
        if (hp !== helperPort()) return false;
        return fetch('http://' + location.host + '/api/ping', { cache: 'no-store' })
            .then(function (r) { return r.json(); })
            .then(function (j) { return !!(j && j.service === 'eai-device-bridge'); })
            .catch(function () { return false; });
    }
    function guideDir() {
        // workspace.html 与助手脚本同在 device/ 目录；file: 下可推导出真实路径
        if (location.protocol !== 'file:') return '';
        var p = decodeURIComponent(location.pathname || '').replace(/\/[^/]*$/, '');
        if (/^\/[A-Za-z]:/.test(p)) p = p.slice(1);            // file:///E:/… → /E:/…
        if (OS.plat === 'win') p = p.replace(/\//g, '\\');     // → E:\…\device
        return p;
    }

    /* 远程站点（https，且不是 127.0.0.1）无法直连本机助手：
       助手只有明文 ws://（没有 TLS 证书，wss:// 握手失败），ws:// 又会被
       浏览器按混合内容拦截。所以远程页只能：下载助手包 → 启动 → 切到
       助手自托管的 http://127.0.0.1:端口/workspace.html。 */
    function isRemotePage() {
        if (location.protocol === 'file:') return false;
        // 本机调试用 http://127.0.0.1:* 页面不算远程（与 entryflow 的回环判定保持一致）
        var host = location.hostname;
        if (host === '127.0.0.1' || host === 'localhost' || host === '::1' || host === '[::1]') return false;
        if (location.protocol !== 'https:') return false;
        return !!(host && host !== '');
    }
  function helperZipUrl() { return 'eai-helper.zip'; }
  function localWorkspaceUrl() {
    return bridgeBase() + '/workspace.html';
  }

  function copyText_(t) {
    if (navigator.clipboard && navigator.clipboard.writeText) {
      return navigator.clipboard.writeText(t);
    }
    return new Promise(function (res, rej) {
      try {
        var ta = document.createElement('textarea');
        ta.value = t; ta.style.position = 'fixed'; ta.style.opacity = '0';
        document.body.appendChild(ta); ta.select();
        var ok = document.execCommand('copy');
        document.body.removeChild(ta);
        ok ? res() : rej(new Error('copy failed'));
      } catch (e) { rej(e); }
    });
  }

  function guideHtml() {
    var dir = guideDir();
    var remote = isRemotePage();
    var useHelperPackage = remote || !dir;
    var h = '';
    h += '<p class="guide-what">' + (remote
      ? '这台电脑还没启动「本机助手」。当前页面在远程网站上，浏览器不允许它直连本机 SSH 通道，' +
        '需要先下载并启动本机助手，再切到助手自带的本地页面。'
      : '这台电脑还没初始化过「本机助手」。按下面的步骤<b>操作一次</b>即可，' +
        '之后点「设备开发」会全自动静默启动，不再出现任何系统提示。') + '</p>';
    if (dir) {
      h += '<div class="guide-path">' +
        '<button class="guide-open" id="gCopy" type="button">复制路径</button>' +
        '<code id="gDir">' + esc(dir) + '</code></div>';
    }
    if (useHelperPackage) {
      h += '<div class="guide-path">' +
        '<a class="guide-open" id="gZip" href="' + esc(helperZipUrl()) + '" download>下载本机助手</a>' +
        '<code>eai-helper.zip · 解压后即用</code></div>';
    }
    h += '<ol class="guide-steps">';
    if (useHelperPackage) {
      h += '<li>点上面的「<b>下载本机助手</b>」，保存 <b>eai-helper.zip</b> 并解压到一个固定目录' +
        '<span class="fhint">压缩包里已含 start_helper.bat、device_bridge.py 和全部前端文件</span></li>';
      if (OS.plat === 'win') {
        h += '<li>双击解压出来的 <b>start_helper.bat</b>' +
          '<span class="fhint">首次运行在线安装依赖约 1~2 分钟，窗口出现横幅后保持运行</span></li>';
      } else if (OS.plat === 'mac' || OS.plat === 'linux') {
        h += '<li>打开「终端」，进入解压目录后执行：<span class="guide-cmd">./start_helper.sh</span>' +
          '<span class="fhint">首次运行自动注册 eai-helper:// 协议并安装依赖，约 1~2 分钟</span></li>';
      } else {
        h += '<li>运行解压目录里的启动脚本（' + esc(OS.reg || 'start_helper') + '）</li>';
      }
      h += '<li>等到出现横幅并保持运行：<span class="guide-cmd">具身智能课程 · 设备开发本机助手</span>' +
        '<span class="fhint">此窗口别关（关了助手就停）</span></li>';
      h += '</ol>';
      h += '<div class="guide-ok"><b>怎么算成功：</b>看到横幅后点 <b>「我已完成，重试检测」</b>，' +
        '检测通过会自动把这个页面切到本机助手地址 ' +
        '<span class="guide-cmd">' + esc(localWorkspaceUrl()) + '</span></div>';
      h += '<p class="guide-after home">往后请直接从 <code>http://127.0.0.1:' + helperPort() +
        '/index.html</code> 进入，本机页面的入口会静默拉起助手，全程自动。</p>';
      return h;
    }
    if (OS.plat === 'win') {
      h += '<li>点「复制路径」，粘贴到<b>文件资源管理器</b>地址栏后回车，进入 device 目录' +
        '<span class="fhint">也可以：按 Win+R → 粘贴路径 → 回车</span></li>';
      h += '<li>双击 <b>start_helper.bat</b>' +
        '<span class="fhint">窗口会自动完成一次性注册；首次运行在线安装依赖约 1~2 分钟</span></li>';
      h += '<li>等到出现横幅并保持运行：<span class="guide-cmd">具身智能课程 · 设备开发本机助手</span>' +
        '<span class="fhint">此窗口别关（关了助手就停）。以后不再需要手动开，点入口会自动静默拉起</span></li>';
    } else if (OS.plat === 'mac' || OS.plat === 'linux') {
      h += '<li>打开「终端」，进入 device 目录：' +
        '<span class="guide-cmd">cd ' + esc(dir || '课程包/device') + '</span></li>';
      h += '<li>执行启动：<span class="guide-cmd">./start_helper.sh</span>' +
        '<span class="fhint">首次运行自动注册 eai-helper:// 协议并安装依赖，约 1~2 分钟</span></li>';
      h += '<li>等到出现横幅并保持运行：<span class="guide-cmd">具身智能课程 · 设备开发本机助手</span></li>';
    } else {
      h += '<li>到课程包的 <b>device/</b> 目录，运行一次本平台的启动脚本（' +
        esc(OS.reg || 'start_helper') + '）</li>';
      h += '<li>等窗口出现横幅：<span class="guide-cmd">具身智能课程 · 设备开发本机助手</span></li>';
    }
    h += '</ol>';
    h += '<div class="guide-ok"><b>怎么算成功：</b>看到上面的横幅后，点右下角' +
      '<b>「我已完成，重试检测」</b>——检测通过即初始化完成，以后全程自动。</div>';
    h += '<p class="guide-after home">完成后可直接在左侧点「＋ 新建会话」开始使用；' +
      '若工作台已在其他标签打开，刷新一下即可。</p>';
    return h;
  }

  function showGuide(reason) {
    var m = $('guideModal');
    if (!m) return;
    var nm = $('modalNew');            // 引导优先，避免两个弹窗叠加
    if (nm && !nm.hidden) nm.hidden = true;
    $('gBody').innerHTML = guideHtml();
    $('gMsg').textContent = reason || '';
    m.hidden = false;
    setStatus('本机助手待初始化（一次即可）', 'busy');
  }

  function closeGuide() {
    var m = $('guideModal');
    if (m) m.hidden = true;
    var g = $('gMsg');
    if (g) g.textContent = '';
  }

  function retryGuide() {
    var g = $('gMsg');
    if (g) g.textContent = '正在检测本机助手 ' + bridgeBase() + ' …';
    probeHelper().then(function (ok) {
      if (ok) {
        S.helper.online = true;
        markHelperReady();              // 探测通过 = 注册+启动均已就绪，入口此后可静默发协议
        if (isRemotePage()) {
          // 远程页面即使助手在线也连不上明文 ws://，必须切到助手自托管的 http 页面
          setStatus('本机助手已就绪，正在切换 …', 'busy');
          location.href = localWorkspaceUrl();
          return;
        }
        closeGuide();
        setStatus('本机助手已就绪', 'on');
        toast('✓ 本机助手已就绪：' + bridgeBase());
        setDeviceLabel('本机助手 ' + bridgeBase().replace(/^https?:\/\//, '') +
          ' 就绪 · 仅监听 127.0.0.1');
        if (!S.sessions.length) openModal();
      } else if (g) {
        g.textContent = '✗ 还没检测到：请确认启动窗口出现「具身智能课程 · 设备开发本机助手」横幅' +
          '且窗口没有关闭，再点一次重试。';
      }
    });
  }

  function confirmModal() {
    var v = readModal();
    $('mErr').textContent = '';

    if (v.kind === 'serial') {
      var baudRaw = String(v.baud || '').trim();
      if (baudRaw && !/^\d+$/.test(baudRaw)) return modalErr('波特率只能填数字（如 115200）');
      v.baud = parseInt(baudRaw, 10) || 115200;
      v.addr = ''; v.user = ''; v.password = '';
      if (!v.serialPort && !v.remember) return modalErr('请从下拉中选择串口（首次请点「选择新串口…」由系统弹窗授权）');
      // 保存可读的串口显示名（芯片型号 / VID PID），侧栏与之后编辑都用它展示
      var psel = $('mPortSel');
      if (psel && psel.value && psel.selectedIndex >= 0) {
        v.serialLabel = psel.options[psel.selectedIndex].text;
      } else if (!v.serialLabel) {
        v.serialLabel = v.serialPort;
      }
    } else if (v.kind === 'vnc') {
      if (!v.addr) return modalErr('请填写设备地址（IP）');
      v.vncPort = parseInt(v.vncPort, 10) || DEF_VNC_PORT;
      v.password = ''; v.user = '';
    } else {
      if (!v.addr) return modalErr('请填写设备地址（IP）');
      if (!v.user) return modalErr('请填写用户名');
      if (!v.password) return modalErr('请填写密码');
      v.sshPort = parseInt(v.sshPort, 10) || DEF_SSH_PORT;
      // 助手端口是全局设置（所有会话共用一个本机助手）
      if (v.helperPort != null && String(v.helperPort).trim() !== '') {
        var hp = parseInt(v.helperPort, 10);
        if (!hp || hp < 1 || hp > 65535) return modalErr('助手端口只能是 1~65535 的数字（如 8100）');
        if (hp !== helperPort()) {
          S.helper.port = hp;
          S.helper.online = false;
          S.helper.launched = false;
          lsSet('eai_helper_port', hp);
        }
      }
    }
    delete v.helperPort;   // 端口存全局，不落到会话记录里

    var rec;
    if (editing) {
      rec = findSession(draft.id) || blankSession(v.kind);
      // 表单里没有名称字段，自动名（用户@IP/纯IP/本机串口@波特率）不随参数更新——
      // 按形状判定后丢弃重算；地址在旧 JS 下被改过、名停在旧值的存量僵尸同样能清
      KINDS.forEach(function (k) { if (k !== v.kind && k in v) delete v[k]; });
      Object.keys(v).forEach(function (k) { rec[k] = v[k]; });
      rec.kind = v.kind;
      if (isAutoName(rec.name)) rec.name = '';
    } else {
      rec = v;
      rec.id = rec.id || newId();
    }
    rec.at = Date.now() / 1000;
    upsertSession(rec);
    if (rec.kind !== 'vnc') savePrefs(rec);
    renderSide();
    closeModal();
    openSession(rec);
  }

  // 串口下拉：把已授权串口 + 已保存的选择合并进 #mPortSel
  var mPortErr = {
    nosupport: '当前浏览器不支持 Web Serial，请用 Chrome / Edge。'
  };
  // 浏览器不暴露 COM 口编号（如 COM9），只能拿到 USB VID/PID；按常见芯片给出可读名
  var USB_CHIPS = [
    { v: 0x1a86, p: 0x7523, n: 'CH340' },
    { v: 0x1a86, p: 0x5523, n: 'CH341' },
    { v: 0x10c4, p: 0xea60, n: 'CP2102' },
    { v: 0x0403, p: 0x6001, n: 'FT232' },
    { v: 0x0403, p: 0x6015, n: 'FT231X' },
    { v: 0x0483, p: 0x5740, n: 'STM32 VCP' },
    { v: 0x2e8a, p: 0x000a, n: 'RP2040' },
    { v: 0x2341, n: 'Arduino' },
    { v: 0x303a, n: 'ESP32' }
  ];
  function usbChipName(info) {
    if (info.usbVendorId == null) return '';
    for (var i = 0; i < USB_CHIPS.length; i++) {
      var c = USB_CHIPS[i];
      if (info.usbVendorId === c.v && (c.p == null || info.usbProductId === c.p)) return c.n;
    }
    return '';
  }
  function portLabelText(p, i) {
    var info = (p && p.getInfo && p.getInfo()) || {};
    var chip = usbChipName(info);
    var txt = '串口 serial:' + (i + 1);
    if (chip) txt += ' · ' + chip;
    if (info.usbVendorId != null) {
      txt += ' (VID 0x' + info.usbVendorId.toString(16).padStart(4, '0') +
        ' PID 0x' + (info.usbProductId != null ? info.usbProductId.toString(16).padStart(4, '0') : '----') + ')';
    }
    return txt;
  }
  function portOptionHtml(p, i) {
    return '<option value="serial:' + (i + 1) + '">' + esc(portLabelText(p, i)) + '</option>';
  }
  function ensureCurrentOption(sel) {
    var cur = sel.value || '';
    if (!cur) return;
    var has = false;
    Array.prototype.forEach.call(sel.options, function (o) { if (o.value === cur) has = true; });
    if (!has) {
      var text = (draft && draft.serialLabel && draft.serialPort === cur)
        ? draft.serialLabel
        : cur + '（已保存）';
      sel.insertAdjacentHTML('beforeend',
        '<option value="' + esc(cur) + '">' + esc(text) + '</option>');
      sel.value = cur;
    }
  }
  function renderPortOptions(list) {
    var sel = $('mPortSel');
    if (!sel) return;
    sel.innerHTML = '<option value="">— 点「选择新串口…」选择 —</option>' +
      list.map(portOptionHtml).join('');
    if (draft && draft.serialPort) sel.value = draft.serialPort;
    ensureCurrentOption(sel);
    draft.serialPort = sel.value;
    if (sel.selectedIndex >= 0) draft.serialLabel = sel.options[sel.selectedIndex].text;
  }
  // 刷新浏览器已授权的串口列表（不弹系统选择框）
  function refreshPortSelect() {
    var hint = $('mPortHint');
    if (!hint) return;
    if (!('serial' in navigator)) {
      hint.className = 'fhint bad';
      hint.textContent = mPortErr.nosupport;
      renderPortOptions([]);
      return;
    }
    hint.className = 'fhint';
    hint.textContent = '正在扫描…';
    Promise.resolve(navigator.serial.getPorts ? navigator.serial.getPorts() : []).then(function (list) {
      renderPortOptions(list);
      hint.className = list.length ? 'fhint ok' : 'fhint';
      hint.textContent = list.length
        ? '找到 ' + list.length + ' 个已授权串口，可从下拉中选择。'
        : '暂无已授权串口，点「选择新串口…」由系统弹窗授权后即可下拉选择。';
    }).catch(function (e) {
      hint.className = 'fhint bad';
      hint.textContent = '扫描失败：' + (e && e.message ? e.message : e);
    });
  }

  // 本机 Web Serial：弹出系统端口选择框，授权并选进下拉
  function pickLocalPort() {
    var hint = $('mPortHint');
    if (!hint) return;
    if (!('serial' in navigator) || !navigator.serial.requestPort) {
      hint.className = 'fhint bad';
      hint.textContent = mPortErr.nosupport;
      return;
    }
    var p;
    try { p = navigator.serial.requestPort(); } catch (e) {
      hint.className = 'fhint bad';
      hint.textContent = '无法弹出端口选择框：' + (e && e.message ? e.message : e);
      return;
    }
    p.then(function () {
      return Promise.resolve(navigator.serial.getPorts ? navigator.serial.getPorts() : []);
    }).then(function (list) {
      // 被授权的端口一定排在 getPorts() 结果里，取最后一个当作刚选中的
      var idx = list.length - 1;
      renderPortOptions(list);
      var val = 'serial:' + (idx + 1);
      var sel = $('mPortSel');
      if (sel) sel.value = val;
      if (draft) { draft.serialPort = val; draft.serialLabel = sel && sel.selectedIndex >= 0
        ? sel.options[sel.selectedIndex].text : val; }
      var labelText = (sel && sel.selectedIndex >= 0) ? sel.options[sel.selectedIndex].text : val;
      hint.className = 'fhint ok';
      hint.textContent = '已授权并选中 [' + labelText + ']，直接点「连接」即可。';
    }).catch(function (e) {
      if (e && (e.name === 'NotFoundError' || e.name === 'AbortError')) {
        hint.className = 'fhint';
        hint.textContent = '未选择端口。可再次点击「选择新串口…」重试。';
      } else {
        hint.className = 'fhint bad';
        hint.textContent = '选择失败：' + (e && e.message ? e.message : e);
      }
    });
  }

  $('mTypes').addEventListener('click', function (e) {
    var b = e.target.closest('button[data-kind]');
    if (!b) return;
    readModal();
    draft.kind = b.dataset.kind;
    syncTypeCards();
    $('mErr').textContent = '';
    renderModalBody();
  });
  $('mClose').addEventListener('click', closeModal);
  $('mCancel').addEventListener('click', closeModal);
  $('mOk').addEventListener('click', confirmModal);
  $('modalNew').addEventListener('click', function (e) { if (e.target === this) closeModal(); });
  $('mBody').addEventListener('click', function (e) {
    if (e.target && e.target.id === 'mScanPorts') {
      e.preventDefault(); refreshPortSelect();
    }
    if (e.target && e.target.id === 'mPickPort') {
      e.preventDefault(); pickLocalPort();
    }
    if (e.target && e.target.id === 'mStartHelper') {
      e.preventDefault(); startHelperFromModal();
    }
  });
  $('modalNew').addEventListener('keydown', function (e) {
    if (e.key === 'Enter' && e.target && e.target.tagName === 'INPUT') { e.preventDefault(); confirmModal(); }
    if (e.key === 'Escape') closeModal();
  });

  /* 初始化引导弹窗（一次性）：关闭 / 复制路径 / 重试检测 */
  $('gClose').addEventListener('click', closeGuide);
  $('gClose2').addEventListener('click', closeGuide);
  $('gRetry').addEventListener('click', retryGuide);
  $('guideModal').addEventListener('click', function (e) {
    if (e.target === this) closeGuide();
    if (e.target && e.target.id === 'gCopy') {
      e.preventDefault();
      var el = $('gDir');
      if (!el) return;
      copyText_(el.textContent).then(function () {
        e.target.textContent = '已复制 ✓';
        setTimeout(function () { e.target.textContent = '复制路径'; }, 1600);
      }).catch(function () {
        $('gMsg').textContent = '复制失败，请手动选中路径 Ctrl+C：' + (el.textContent || '');
      });
    }
  });
  $('guideModal').addEventListener('keydown', function (e) {
    if (e.key === 'Escape') closeGuide();
  });

  /* ==========================================================
     断开全部
     ========================================================== */
  function disconnectAll() {
    var keys = Object.keys(S.conns);
    if (!keys.length && !allTabs().length) { toast('当前没有活动连接。'); return; }
    if (!confirm('断开全部连接？本机助手会关闭这些 SSH 连接，打开着的本机串口也会一并关闭。')) return;
    keys.forEach(function (k) {
      var c = S.conns[k];
      fetch(c.base + '/api/logout?token=' + encodeURIComponent(c.token)).catch(function () { });
    });
    S.conns = {};
    allTabs().forEach(function (t) { try { t.view.close(); } catch (e) { } });
    S.panes.L.tabs = []; S.panes.L.active = null;
    S.panes.R.tabs = []; S.panes.R.active = null;
    $('viewsL').innerHTML = ''; $('viewsR').innerHTML = '';
    renderPane('L');
    renderPane('R');
    renderSide();
    $('sideSys').innerHTML = '';
    setUserLabel('未连接');
    setDeviceLabel('');
    setStatus('已断开全部连接', '');
  }

  /* ==========================================================
     初始化
     ========================================================== */
  function detectSameOrigin() {
    // 本机助手模式：先探测 127.0.0.1:助手端口；
    // 页面若由助手托管（或教学服务器在同源反代了桥接接口），再回退探测同源，
    // 两种情况都把「设备地址」预填为本机地址。
    return probeHelper().then(function (on) {
      if (on) return { addr: HELPER_HOST, port: helperPort() };
      if (location.protocol.indexOf('http') !== 0) return null;
      return fetch('/api/ping', { cache: 'no-store' })
        .then(function (r) { return r.json(); })
        .then(function (j) {
          if (j && j.service === 'eai-device-bridge') {
            return {
              addr: location.hostname,
              port: parseInt(location.port, 10) || 80
            };
          }
          return null;
        })
        .catch(function () { return null; });
    });
  }

  function bootPopout() {
    var m = /[#&]pop=([^&]+)/.exec(location.hash || '');
    if (!m) return false;
    var p;
    try { p = JSON.parse(unb64url(m[1])); } catch (e) { return false; }
    if (p.conn) S.conns[p.conn.key || p.conn.base] = p.conn;
    S.popout = true;
    $('app').hidden = false;
    renderSide();
    renderPane('L');
    renderPane('R');
    var cfg = p.cfg || {};
    cfg.title = p.title || '';
    openTab(p.kind, cfg, p.conn || null);
    if (p.conn) {
      setUserLabel(p.conn.user + '@' + p.conn.addr);
      setDeviceLabel('本机助手 ' + (p.conn.base || '').replace(/^https?:\/\//, '') +
        ' → ' + p.conn.addr + ' · ' + (p.title || ''));
      loadSysInfo(p.conn);
    }
    document.title = (p.title || KIND_NAME[p.kind] || '会话') + ' · 设备开发工作台';
    return true;
  }

  function boot() {
    loadSessions();
    S.prefs = lsGet(LS_PREFS, {}) || {};
    if (bootPopout()) return;

    $('app').hidden = false;
    renderSide();
    renderPane('L');
    renderPane('R');
    setStatus('就绪 · 从左侧会话列表打开，或点「＋ 新建会话」', '');

    detectSameOrigin().then(function (info) {
      if (isRemotePage() && info && info.addr === HELPER_HOST) {
        // 远程页面：助手就算已经在跑也连不上明文 ws://（混合内容拦截），
        // 必须切到助手自托管的 http 工作台，否则后面新建会话必然握手失败。
        location.href = localWorkspaceUrl();
        return;
      }
      if (info) {
        S.sameOrigin = info;
        if (info.addr === HELPER_HOST) {
          // 本机助手在跑：探测通过即视为「已初始化」，之后入口可静默发协议
          S.helper.port = info.port;
          S.helper.online = true;
          markHelperReady();
          lsSet('eai_helper_port', info.port);
          setDeviceLabel('本机助手 ' + info.addr + ':' + info.port + ' 就绪 · 仅监听 127.0.0.1');
        } else {
          // 页面由设备端旧版桥接托管：仅借用它的地址做预填，连接仍走本机助手
          setDeviceLabel('已从设备托管页打开 · 本机助手 ' +
            bridgeBase().replace(/^https?:\/\//, '') + '（连接前会自动探测）');
        }
      } else {
        setDeviceLabel('本机助手 ' + bridgeBase().replace(/^https?:\/\//, '') +
          ' 未启动 · 连接时会自动尝试启动');
        // 远程页面（助手没跑）与全新本地环境：都弹一次性初始化引导
        if (isRemotePage() || helperReady() !== 1) showGuide();
      }
      if (!S.sessions.length && (!$('guideModal') || $('guideModal').hidden)) openModal();
    });
  }

  $('btnNew').addEventListener('click', function () { openModal(); });
  $('btnNewTab').addEventListener('click', function () { openModal(); });
  $('btnSplit').addEventListener('click', function () {
    S.split = !S.split;
    if (!S.split && S.panes.R.tabs.length) {
      S.panes.R.tabs.slice().forEach(function (t) { moveTab(t.id); });
    }
    applySplit();
  });
  $('btnLogout').addEventListener('click', disconnectAll);

  document.addEventListener('keydown', function (e) {
    if (!(e.ctrlKey || e.metaKey) || !e.shiftKey) return;
    var k = e.key.toLowerCase();
    if (k === 't') { e.preventDefault(); openModal(); }
    else if (k === 'w') {
      e.preventDefault();
      var pid = S.activePane, p = S.panes[pid];
      if (p.active) closeTab(p.active);
    } else if (k === 'e') { e.preventDefault(); $('btnSplit').click(); }
  });

  window.addEventListener('resize', function () {
    allTabs().forEach(function (t) { if (t.view.refit) t.view.refit(); });
  });

  boot();
})();

/* ============================================================
   entryflow.js — 首页/入口页 → 工作台 的统一入口流（跨平台共用）
   点击「设备开发」时的决策顺序：
     ① 读取本机记忆 eai_helper_ready：
        =1 表示这台电脑曾成功拉起助手（协议必已注册）→ 手势内发 eai-helper://
           静默启动（各平台由操作系统翻译到对应注册的启动脚本）；
        其他（全新环境）→ 不发协议，直接进工作台，由工作台弹「一次性初始化」
           分步引导——从未注册的机器上彻底避免系统“选择应用”弹框。
     ② 同标签：协议发出后延迟跳转，避免本页卸载取消启动动作；
        新标签（target=_blank）：保持浏览器默认打开（手势内，不被弹窗拦截）。
   首次成功启动后 workspace.js 会把 ready 置 1，之后全部自动。
   ============================================================ */
(function () {
  'use strict';

  var KEY_READY = 'eai_helper_ready';        // 1=曾成功拉起（允许协议自动唤起）；0=已确认失败（跳过协议防弹框）
  var KEY_PENDING = 'eai_helper_pending';    // 最近一次发起启动的时间戳（工作台据此延长等待）
  var KEY_PORT = 'eai_helper_port';

  function lsGet(k, dflt) {
    try { var v = localStorage.getItem(k); return v == null ? dflt : JSON.parse(v); } catch (e) { return dflt; }
  }
  function lsSet(k, v) { try { localStorage.setItem(k, JSON.stringify(v)); } catch (e) { } }

  function isLocalHost() {
    var h = location.hostname;
    return h === '127.0.0.1' || h === 'localhost' || h === '::1' || h === '[::1]' || h === '';
  }

  /* 远程 https 站点必须切到本机助手托管的同源页面：
     — 本机助手只提供 ws://（没有 TLS 证书），https 页面改连 wss:// 会直接失败；
     — 退回 ws:// 又会被浏览器当作混合内容拦截。
     所以只有 file:// 与 127.0.0.1/localhost 上才原地打开工作台。 */
  function resolveTarget(href) {
    if (location.protocol !== 'https:' || isLocalHost()) return href;
    var name = href.split('#')[0].split('?')[0].split('/').pop() || 'workspace.html';
    return 'http://127.0.0.1:' + EAI.port() + '/' + name;
  }

  window.EAI = {
    KEY_READY: KEY_READY,
    KEY_PENDING: KEY_PENDING,

    /* 本机助手端口（与 workspace.js 共用 localStorage 记忆） */
    port: function () {
      var p = parseInt(lsGet(KEY_PORT, 8100), 10);
      return (p > 0 && p <= 65535) ? p : 8100;
    },

    /* 探测本机助手是否在跑（127.0.0.1:port/api/ping，失败静默返回 false） */
    probe: function (timeoutMs) {
      if (!('fetch' in window)) return Promise.resolve(false);
      var ctl = ('AbortController' in window) ? new AbortController() : null;
      var timer = null;
      if (ctl) timer = setTimeout(function () { try { ctl.abort(); } catch (e) { } }, timeoutMs || 1200);
      return fetch('http://127.0.0.1:' + EAI.port() + '/api/ping', {
        cache: 'no-store', mode: 'cors', signal: ctl ? ctl.signal : undefined
      }).then(function (r) { return r.json(); })
        .then(function (j) { return !!(j && j.service === 'eai-device-bridge'); })
        .catch(function () { return false; })
        .then(function (ok) { if (timer) clearTimeout(timer); return ok; });
    },

    /* 在用户点击手势内发起协议启动（操作系统的注册表/LaunchServices 翻译到本平台脚本） */
    fire: function () {
      lsSet(KEY_PENDING, Date.now());
      try {
        location.href = 'eai-helper://start?port=' + EAI.port();
        setTimeout(function () {
          history.replaceState(null, '', location.pathname + location.search);
        }, 0);
      } catch (e) { }
    },

    ready: function () { return lsGet(KEY_READY, null); },

    /* 点击入口统一处理。
       href   : 目标工作台地址
       newTab : 该链接是否新开标签（true 时不接管导航，交给浏览器默认行为） */
    entry: function (href, newTab) {
      var willFire = EAI.ready() === 1;
      if (willFire) EAI.fire();          // 手势内同步发协议：已注册平台会静默拉起（幂等，在跑则秒退）

      var target = resolveTarget(href);
      if (target === href) {
        if (newTab) return;              // 新标签交给浏览器默认打开（仍在手势内，不会被弹窗拦截）
        setTimeout(function () { location.href = href; }, willFire ? 450 : 60);
        return;
      }

      // 远程 https → 本地助手：先在手势内占住新标签（否则异步导航会被弹窗拦截），
      // 轮询到助手就绪后再导航过去；始终没起来就退回远程工作台，由它弹「一次性初始化」引导。
      var win = newTab ? window.open('about:blank', '_blank') : null;
      var deadline = Date.now() + 4000;
      (function wait() {
        EAI.probe(600).then(function (ok) {
          if (!ok && Date.now() < deadline) { setTimeout(wait, 250); return; }
          if (ok) {
            if (win) win.location.href = target;
            else location.href = target;
            return;
          }
          if (win) { try { win.close(); } catch (e) { } }
          location.href = href;
        });
      })();
    }
  };
})();
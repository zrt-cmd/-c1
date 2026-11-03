/* =============================================================
   beginner.js — 初学者版交互（小测判定 + 复制按钮增强）
   依赖：无。与 tutorial.js 可共存。
   ============================================================= */
(function () {
  'use strict';

  /* ── 随堂小测：点选项即时判定 ─────────────────────────────── */
  function initQuiz() {
    var quizzes = document.querySelectorAll('.quiz2');
    for (var i = 0; i < quizzes.length; i++) {
      (function (q) {
        var opts = q.querySelectorAll('.opt');
        for (var k = 0; k < opts.length; k++) {
          opts[k].addEventListener('click', function () {
            if (q.getAttribute('data-done') === '1') return;
            var right = this.getAttribute('data-right') === '1';
            this.classList.add(right ? 'ok' : 'no');
            var fb = q.querySelector('.fb');
            if (fb) {
              if (right) {
                fb.classList.add('on');
                q.setAttribute('data-done', '1');
              } else {
                this.title = '这个不对，再选一次';
              }
            }
          });
        }
      })(quizzes[i]);
    }
  }

  /* ── 参数实验台通用滑块联动（value 输出 + 自定义回调） ───── */
  window.labBind = function (id, fn) {
    var el = document.getElementById(id);
    if (!el) return;
    var out = document.getElementById(id + '_o');
    function upd() {
      if (out) out.textContent = el.value;
      if (typeof fn === 'function') fn(el.value);
    }
    el.addEventListener('input', upd);
    upd();
  };

  if (document.readyState === 'loading') {
    document.addEventListener('DOMContentLoaded', initQuiz);
  } else {
    initQuiz();
  }
})();

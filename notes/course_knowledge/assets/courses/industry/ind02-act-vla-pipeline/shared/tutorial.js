/* ROS2 教程实验页公共脚本：scrollspy + 章节 reveal */
(function(){
  const secs=[...document.querySelectorAll('section.blk')];
  const links=[...document.querySelectorAll('.side a[href^="#"]')];
  function spy(){
    if(!secs.length) return;
    let cur=secs[0].id;
    for(const s of secs){ if(s.getBoundingClientRect().top<140) cur=s.id; }
    links.forEach(a=>a.classList.toggle('now', a.getAttribute('href')==='#'+cur));
  }
  window.addEventListener('scroll',spy,{passive:true}); spy();
  const io=new IntersectionObserver(es=>es.forEach(e=>{
    if(e.isIntersecting){ e.target.classList.add('on'); io.unobserve(e.target); }
  }),{threshold:.10});
  document.querySelectorAll('.blk').forEach(el=>io.observe(el));
})();

/* 命令/代码块一键复制：给页面里所有 <pre> 加一个复制按钮 */
(function(){
  if(window.__preCopyReady) return; window.__preCopyReady=1;
  var CSS='.pre-wrap{position:relative}'
    +'.copy-btn{position:absolute;top:8px;right:8px;z-index:3;background:rgba(30,41,59,.92);color:#94a3b8;border:1px solid #334155;border-radius:7px;padding:3px 10px;font-size:11.5px;font-family:inherit;line-height:1.6;cursor:pointer;opacity:.5;transition:opacity .18s,background .18s,color .18s;user-select:none;-webkit-user-select:none}'
    +'.pre-wrap:hover .copy-btn{opacity:1}'
    +'.copy-btn:hover{background:#334155;color:#e2e8f0}'
    +'.copy-btn.done{background:#065f46;border-color:#10b981;color:#d1fae5;opacity:1}';
  function flash(btn,txt,ok){
    btn.textContent=txt;
    if(ok) btn.classList.add('done');
    setTimeout(function(){ btn.textContent='复制'; btn.classList.remove('done'); },1600);
  }
  function fallback(text,btn){
    var ta=document.createElement('textarea');
    ta.value=text; ta.setAttribute('readonly','');
    ta.style.cssText='position:fixed;top:0;left:-9999px';
    document.body.appendChild(ta); ta.select();
    try{ ta.setSelectionRange(0,ta.value.length); }catch(e){}
    try{ document.execCommand('copy'); flash(btn,'已复制 ✓',1); }
    catch(e){ flash(btn,'复制失败',0); }
    document.body.removeChild(ta);
  }
  function doCopy(pre,btn){
    var text=pre.textContent.replace(/\u00a0/g,' ').replace(/^\n+/,'').replace(/\s+$/,'');
    if(navigator.clipboard && navigator.clipboard.writeText){
      navigator.clipboard.writeText(text).then(function(){ flash(btn,'已复制 ✓',1); },
        function(){ fallback(text,btn); });
    } else { fallback(text,btn); }
  }
  function setup(){
    var st=document.createElement('style'); st.textContent=CSS; document.head.appendChild(st);
    var list=document.querySelectorAll('pre');
    for(var i=0;i<list.length;i++){
      var pre=list[i];
      if(pre.parentNode && pre.parentNode.className==='pre-wrap') continue;
      var wrap=document.createElement('div'); wrap.className='pre-wrap';
      pre.parentNode.insertBefore(wrap,pre); wrap.appendChild(pre);
      var btn=document.createElement('button');
      btn.type='button'; btn.className='copy-btn'; btn.textContent='复制';
      btn.title='复制这段命令';
      (function(p,b){ btn.addEventListener('click',function(e){ e.preventDefault(); doCopy(p,b); }); })(pre,btn);
      wrap.appendChild(btn);
    }
  }
  if(document.readyState==='loading') document.addEventListener('DOMContentLoaded',setup);
  else setup();
})();

/* 图片点击放大（lightbox） */
(function(){
  var lb=document.createElement('div');
  lb.className='lightbox';
  lb.innerHTML='<span class="lb-close">&times;</span><img alt=""><div class="lb-title"></div>';
  document.body.appendChild(lb);
  var lbImg=lb.querySelector('img'), lbTitle=lb.querySelector('.lb-title');
  function zoom(e){
    var t=e.currentTarget;
    lbImg.src=t.currentSrc||t.src;
    var cap=t.closest('figure')?t.closest('figure').querySelector('figcaption'):null;
    lbTitle.textContent=cap?cap.textContent:'';
    lb.classList.add('open');
  }
  function closeLb(){ lb.classList.remove('open'); }
  function bind(){
    document.querySelectorAll('.img-block img,.img-grid img').forEach(function(im){
      if(im.__lb) return;
      im.__lb=true; im.style.cursor='zoom-in'; im.addEventListener('click',zoom);
    });
  }
  bind();
  document.addEventListener('DOMContentLoaded',bind);
  window.addEventListener('load',bind);
  setInterval(bind,600);
  lb.addEventListener('click',closeLb);
  document.addEventListener('keydown',function(e){ if(e.key==='Escape') closeLb(); });
})();
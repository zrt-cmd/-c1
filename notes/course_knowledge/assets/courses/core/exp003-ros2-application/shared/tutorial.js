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
  }),{threshold:0, rootMargin:'0px 0px -8% 0px'});
  document.querySelectorAll('.blk').forEach(el=>io.observe(el));
  /* 兜底：超高元素（高于视口约 10 倍）永远达不到百分比阈值，按位置直接判定 */
  function revealByPos(){
    const vh=window.innerHeight;
    document.querySelectorAll('.blk:not(.on)').forEach(el=>{
      const r=el.getBoundingClientRect();
      if(r.top<vh*0.92 && r.bottom>0){ el.classList.add('on'); io.unobserve(el); }
    });
  }
  window.addEventListener('scroll',revealByPos,{passive:true});
  window.addEventListener('resize',revealByPos);
  revealByPos();
  setTimeout(revealByPos,400);
})();

/* ↓ 命令/代码块一键复制：给页面里所有 <pre> 加一个复制按钮 */
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
  /* 从 <pre> 提取待复制文本。
     data-copy="cmd" 时：逐行丢弃以 # 开头的纯注释行、以及行尾的 "  # 注释"，
     让"一个注释 + 一条命令"的排版可以只复制命令本身。 */
  function extract(pre){
    var raw=pre.textContent.replace(/\u00a0/g,' ');
    var mode=pre.getAttribute&&pre.getAttribute('data-copy');
    if(mode!=='cmd'){
      return raw.replace(/^\n+/,'').replace(/\s+$/,'');
    }
    var out=[];
    raw.split('\n').forEach(function(line){
      var t=line.replace(/\s+$/,'');
      if(/^\s*#/.test(t)) return;                 // 整行注释 → 跳过
      // 行尾注释：空白 + # 之后全部去掉（命令里出现 " # " 的概率极低）
      t=t.replace(/\s+#.*$/,'');
      if(t.trim()==='') return;                   // 去掉空行
      out.push(t);
    });
    return out.join('\n');
  }
  function doCopy(pre,btn){
    var text=extract(pre);
    if(!text){ flash(btn,'无命令可复制',0); return; }
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

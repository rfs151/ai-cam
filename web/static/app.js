// ai-cam 演示页：WebSocket 收状态/统计/预览，发控制命令
'use strict';
const $ = id => document.getElementById(id);
const esc = s => String(s ?? '').replace(/[&<>"']/g, c => ({ '&': '&amp;', '<': '&lt;', '>': '&gt;', '"': '&quot;', "'": '&#39;' }[c]));
const fmtN = n => (n ?? 0).toLocaleString('en-US');
const STATE_CN = { Idle: '空闲', Connecting: '连接中', Ready: '已就绪', Acquiring: '采集中', Error: '出错' };
const STATUS_CN = { missing_packets: '缺包', timeout: '超时', size_mismatch: '尺寸不符', aborted: '中止', other: '其它' };

let ws = null, myId = null, meta = null, info = null, stateMsg = null, cameras = [];
let wsDelay = 500;
const hist = [];          // 最近的 stats（带时间），用来算"最近 1 秒"和曲线
let lastPreviewUrl = null, lastPreviewAt = 0;

// ---------------- 连接 ----------------
function connect() {
  const proto = location.protocol === 'https:' ? 'wss' : 'ws';
  ws = new WebSocket(`${proto}://${location.host}/ws`);
  ws.binaryType = 'blob';
  ws.onopen = () => { wsDelay = 500; setConn(true); };
  ws.onclose = () => { setConn(false); setTimeout(connect, wsDelay); wsDelay = Math.min(wsDelay * 2, 8000); };
  ws.onmessage = ev => {
    if (typeof ev.data !== 'string') return onPreview(ev.data);
    let m; try { m = JSON.parse(ev.data); } catch { return; }
    (handlers[m.type] || (() => {}))(m);
  };
}
function setConn(ok) {
  $('conn').dataset.s = ok ? 'ok' : 'down';
  $('conn').querySelector('b').textContent = ok ? '已连接' : '重连中';
}
function send(op, extra = {}) {
  if (ws && ws.readyState === 1) ws.send(JSON.stringify({ op, ...extra }));
}
const iControl = () => meta && meta.controller === myId;

const handlers = {
  welcome(m) { myId = m.id; },
  meta(m) { meta = m; renderMeta(); },
  cameras(m) { cameras = m.items || []; renderCams(); },
  info(m) { info = m; renderInfo(); renderFunnel(); },
  state(m) { stateMsg = m; renderState(); },
  stats(m) { onStats(m); },
  record(m) {
    if (m.state === 'started') $('verify').innerHTML = `<p class="res run">录像中（${m.secs} 秒）…</p>`;
    else if (m.state === 'done') $('verify').innerHTML = `<p class="res run">已写入 ${fmtN(m.written)} 帧，正在校验…</p>`;
  },
  verify(m) { renderVerify(m); },
  log(m) { addLog(m.level, m.msg); },
};

// ---------------- 控制权与元信息 ----------------
function renderMeta() {
  const mine = iControl();
  $('viewers').textContent = `${meta.viewers} 人在看`;
  const b = $('btnControl');
  const left = meta.control_left, mmss = `${Math.floor(left / 60)}:${String(left % 60).padStart(2, '0')}`;
  if (mine) { b.textContent = `释放控制权（${mmss}）`; b.classList.remove('primary'); }
  else if (meta.controller) { b.textContent = '他人正在操作'; b.classList.remove('primary'); }
  else { b.textContent = '获取控制权'; b.classList.add('primary'); }
  b.disabled = !mine && !!meta.controller;
  const banner = $('banner');
  if (mine) { banner.hidden = false; banner.className = 'banner'; banner.textContent = `你正在操作。${meta.limits.control_idle_s / 60} 分钟没有操作会自动释放，其他访客实时看到你的操作结果。`; }
  else if (meta.controller) { banner.hidden = false; banner.className = 'banner warn'; banner.textContent = `另一位访客正在操作（${mmss} 后如无操作自动释放），你可以旁观。`; }
  else { banner.hidden = false; banner.className = 'banner'; banner.textContent = '点击右上角「获取控制权」后可以修改参数和注入故障。'; }
  document.querySelectorAll('[data-ctl]').forEach(el => { el.disabled = !mine; });

  // 参数回显（正在编辑的输入框不覆盖）
  const p = meta.params;
  for (const el of document.querySelectorAll('[data-key]')) {
    const v = p[el.dataset.key];
    if (v !== undefined && document.activeElement !== el) {
      if (el.tagName === 'SELECT' && ![...el.options].some(o => o.value === v)) el.add(new Option(v, v));
      el.value = v;
    }
  }
  if (document.activeElement !== $('procRange')) $('procRange').value = p.process_ms;
  if (document.activeElement !== $('diskRange')) $('diskRange').value = Math.round(p.slow_disk_us / 1000);
  $('procOut').textContent = `${p.process_ms} ms`;
  $('diskOut').textContent = `${Math.round(p.slow_disk_us / 1000)} ms`;

  const cam = meta.cams.find(c => c.serial === meta.selected);
  if (cam) {
    $('cableCam').textContent = cam.serial;
    $('btnCable').textContent = cam.plugged ? '拔掉网线' : '插回网线';
    $('btnCable').classList.toggle('on', !cam.plugged);
    $('lossOut').textContent = `${cam.loss}‰`;
    document.querySelectorAll('#lossSeg button').forEach(x => x.classList.toggle('cur', +x.dataset.loss === cam.loss));
  }
  renderCams();
}

function renderCams() {
  const known = meta ? meta.cams : [];
  // 枚举结果为准；拔掉网线的相机枚举不到，但仍列出来并标注
  const rows = known.map(k => ({ k, d: cameras.find(c => c.serial === k.serial) }));
  $('cams').innerHTML = rows.map(({ k, d }) => {
    const sel = meta && meta.selected === k.serial;
    const tags = (d ? '' : '<span class="tag bad">未发现</span>') + (k.loss ? `<span class="tag warn">丢包 ${k.loss}‰</span>` : '') + (k.plugged ? '' : '<span class="tag bad">网线已拔</span>');
    const sub = d ? `${esc(d.address)} · ${esc(d.protocol)}` : `${esc(k.iface)} · —`;
    return `<li class="${sel ? 'sel' : ''}"><span class="nm">${esc(d ? `${d.vendor} ${d.model}` : 'Aravis Fake')} · ${esc(k.serial)}${tags}</span>
      <span class="sub">${sub}</span>
      <button class="small" data-ctl data-open="${esc(k.serial)}" ${iControl() ? '' : 'disabled'}>${sel ? '重新打开' : '打开'}</button></li>`;
  }).join('');
}

function renderInfo() {
  if (!info) return;
  const sel = $('pPixfmt'), cur = meta?.params?.pixfmt || info.pixfmt;
  const allowed = ['Mono8', 'Mono16', 'RGB8', 'BayerRG8', 'BayerGR8', 'BayerGB8', 'BayerBG8'];
  sel.innerHTML = (info.pixfmts || []).filter(x => allowed.includes(x)).map(x => `<option>${esc(x)}</option>`).join('');
  sel.value = cur;
  const rows = [
    ['相机', `${info.vendor} ${info.model} · ${info.serial}`], ['地址', info.address || '—'],
    ['传感器', `${info.sensor_w}×${info.sensor_h}`], ['当前图像', `${info.width}×${info.height} ${info.pixfmt}`],
    ['每帧字节', fmtN(info.payload)], ['每帧包数', `${fmtN(info.packets_per_frame)}（包 ${info.packet_size} 字节）`],
    ['帧率上限', `${(+info.fps_max).toFixed(0)} fps`],
  ];
  $('info').innerHTML = rows.map(([k, v]) => `<dt>${k}</dt><dd>${esc(v)}</dd>`).join('');
}

// ---------------- 状态机 ----------------
function renderState() {
  const s = stateMsg?.state || 'Idle';
  document.querySelectorAll('#states li').forEach(li => li.classList.toggle('cur', li.dataset.s === s));
  const ob = $('osdState'); ob.textContent = STATE_CN[s] || s; ob.dataset.s = s;
  let note = '';
  if (s === 'Connecting' && stateMsg.retry_in_ms) note = `连接失败，${stateMsg.retry_in_ms} ms 后重试（指数退避：100 → 200 → 400 … 最长 5000 ms）`;
  else if (s === 'Connecting') note = '正在发现并打开相机…';
  else if (s === 'Acquiring') note = '相机推流中：取帧 → 检查状态 → 拷贝 → 归还 SDK 缓冲';
  else if (s === 'Ready') note = '已打开相机，参数已下发，等待开始采集';
  else if (s === 'Idle') note = '没有打开相机';
  if (stateMsg?.reconnects) note += `　已重连 ${stateMsg.reconnects} 次`;
  $('stateNote').textContent = note;
  $('nosig').hidden = s === 'Acquiring' && Date.now() - lastPreviewAt < 3000;
}

// ---------------- 预览 ----------------
function onPreview(blob) {
  const url = URL.createObjectURL(blob);
  const img = $('preview');
  img.onload = () => { if (lastPreviewUrl && lastPreviewUrl !== url) URL.revokeObjectURL(lastPreviewUrl); lastPreviewUrl = url; };
  img.src = url;
  lastPreviewAt = Date.now();
  $('nosig').hidden = true;
}
setInterval(() => { if (Date.now() - lastPreviewAt > 3000) $('nosig').hidden = false; }, 1000);

// ---------------- 统计 ----------------
function onStats(s) {
  s.t = performance.now();
  hist.push(s);
  while (hist.length > 320) hist.shift();
  $('osdFps').textContent = `${(+s.fps).toFixed(1)} fps · ${(+s.mbps).toFixed(1)} MB/s`;
  $('osdId').textContent = `frame ${s.frame_id}`;
  if (info) $('osdRes').textContent = `${info.width}×${info.height} ${info.pixfmt}`;
  renderFunnel();
  drawChart();
}

// 和约 1 秒前的统计相比的增量
function recent() {
  if (hist.length < 2) return null;
  const now = hist[hist.length - 1];
  let old = hist[0];
  for (let i = hist.length - 1; i >= 0; i--) { if (now.t - hist[i].t >= 1000) { old = hist[i]; break; } }
  const dt = (now.t - old.t) / 1000 || 1;
  const d = (f) => Math.max(0, f(now) - f(old));
  return {
    dt, now,
    ok: d(x => x.ok), bad: d(x => x.bad), gaps: d(x => x.link_gaps), lost: d(x => x.link_missing),
    pr: d(x => x.pkt.received), pm: d(x => x.pkt.missing), under: d(x => x.sdk.underruns),
    dq: now.rec && old.rec ? d(x => x.rec.drop_queue + x.rec.drop_pool) : (now.rec ? now.rec.drop_queue + now.rec.drop_pool : 0),
  };
}

function renderFunnel() {
  const s = hist[hist.length - 1];
  if (!s) return;
  const r = recent() || {};
  const pct = (a, b) => (b > 0 ? (100 * a / b) : 0);
  const lossPct = pct(r.pm, r.pr + r.pm), badPct = pct(r.bad, r.ok + r.bad);
  const lev = (bad, warn) => (bad ? 'bad' : warn ? 'warn' : 'ok');
  const badBy = Object.entries(s.bad_by || {}).map(([k, v]) => `${STATUS_CN[k] || k} ${fmtN(v)}`).join('，') || '无';
  const rec = s.rec;
  const steps = [
    { n: '网络包（GVSP）', v: `收 ${fmtN(s.pkt.received)} · 缺 ${fmtN(s.pkt.missing)}`, s: lev(lossPct > 1, r.pm > 0),
      d: `最近 1 秒缺包率 <b>${lossPct.toFixed(2)}%</b>，重发请求 ${fmtN(s.pkt.resend_requests)}（虚拟相机不支持重发）` },
    { n: '拼帧', v: `完整 ${fmtN(s.ok)} · 坏 ${fmtN(s.bad)}`, s: lev(badPct > 5, r.bad > 0),
      d: `最近 1 秒坏帧率 <b>${badPct.toFixed(1)}%</b>；坏帧原因：${badBy}` },
    { n: 'SDK 缓冲', v: `underrun ${fmtN(s.sdk.underruns)}`, s: lev(r.under > 0, false),
      d: r.under > 0 ? `SDK 没有空缓冲，到达的数据被丢弃（${info ? info.buffers : '—'} 块缓冲，取帧线程每帧多耗时 ${s.process_ms} ms）` : `缓冲周转正常（${info ? info.buffers : '—'} 块）` },
    { n: '帧号检查', v: `缺口 ${fmtN(s.link_gaps)} · 缺 ${fmtN(s.link_missing)} 帧`, s: lev(r.gaps > 0, s.link_gaps > 0),
      d: '取帧入口检查帧号连续（16 位 block ID，65535 之后回到 1 不算丢帧）' },
    { n: '程序队列（录像）', v: rec ? `${rec.queue}/${rec.qcap} · 丢 ${fmtN(rec.drop_queue + rec.drop_pool)}` : '未录像', s: rec ? lev(r.dq > 0, rec.queue > rec.qcap / 2) : 'idle',
      d: rec ? `帧池 + 有界队列，满了不等待直接丢弃计数；写盘额外耗时 ${Math.round(s.slow_disk_us / 1000)} ms/帧` : '录像时帧才进入队列和写盘线程' },
    { n: '文件', v: rec ? `写入 ${fmtN(rec.written)} · 缺口 ${fmtN(rec.file_gaps)}` : '—', s: rec ? lev(false, rec.file_gaps > 0) : 'idle',
      d: rec ? `${(rec.bytes / 1e6).toFixed(1)} MB，单帧最慢写盘 ${(rec.max_write_us / 1000).toFixed(1)} ms` : '文件缺帧 = 链路丢帧 + 程序内部丢帧' },
  ];
  $('funnel').innerHTML = steps.map(x => `<li data-s="${x.s}"><span class="fn">${x.n}</span><span class="fv">${x.v}</span><span class="fd">${x.d}</span></li>`).join('');
  renderDiag(s, r, lossPct, badPct);
}

function renderDiag(s, r, lossPct, badPct) {
  const el = $('diag');
  let cls = '', title = '', body = '';
  if (s.state === 'Connecting' || s.state === 'Error') {
    cls = 'warn'; title = '相机连接中断';
    body = '采集端发现心跳丢失或 2 秒没有图像，释放资源后按指数退避重连；相机恢复后自动重新下发参数并继续采集。';
  } else if (s.state !== 'Acquiring') {
    title = '没有在采集'; body = '打开相机后点「开始采集」。';
  } else if (r.bad > 0 && badPct > 5) {
    const n = info ? info.packets_per_frame : 0, p = lossPct / 100;
    const whole = n ? Math.pow(1 - p, n) * 100 : 0;
    cls = 'bad'; title = `网络丢包导致 ${badPct.toFixed(0)}% 的帧不完整`;
    body = `每帧切成 <b>${fmtN(n)}</b> 个包，任何一个包丢了这一帧就坏。缺包率 ${lossPct.toFixed(2)}% 时，完整帧的概率约为 ` +
      `<code>(1 − ${p.toFixed(4)})^${n} ≈ ${whole.toFixed(1)}%</code>。真实相机靠丢包重发补回；也可以把包大小改成 8000（巨帧）减少每帧包数，或降低分辨率。`;
  } else if (r.under > 0 || (r.gaps > 0 && s.process_ms > 0)) {
    cls = 'bad'; title = '取帧线程太慢，SDK 缓冲不够用';
    body = `每帧额外耗时 ${s.process_ms} ms，缓冲还给 SDK 太慢，SDK 没有空缓冲时整帧丢弃，帧号出现缺口。取帧线程里只做拷贝和归还缓冲，算法和写盘交给其它线程。`;
  } else if (r.dq > 0) {
    cls = 'warn'; title = '写盘跟不上，队列满了在丢帧';
    body = `写盘线程每帧多耗时 ${Math.round(s.slow_disk_us / 1000)} ms，64 帧的队列被填满后新帧直接丢弃并计数。取帧和 SDK 缓冲不受影响，这正是有界队列的作用。`;
  } else if (r.gaps > 0) {
    cls = 'warn'; title = '帧号不连续';
    body = '有整帧没有到达。常见原因：相机端触发太快、相机内部缓存溢出、帧率超过带宽。';
  } else {
    title = '各层都没有丢帧';
    body = '试试右上方的故障注入：把丢包率调到 2%，或把取帧线程调慢到 60 ms 并把 SDK 缓冲改成 2 块。';
  }
  el.className = `diag ${cls}`;
  el.innerHTML = `<b>${title}</b>${body}`;
}

// ---------------- 曲线 ----------------
function drawChart() {
  const c = $('chart'), dpr = window.devicePixelRatio || 1;
  const w = c.clientWidth, h = 150;
  if (c.width !== Math.round(w * dpr)) { c.width = Math.round(w * dpr); c.height = Math.round(h * dpr); }
  const g = c.getContext('2d');
  g.setTransform(dpr, 0, 0, dpr, 0, 0);
  g.clearRect(0, 0, w, h);
  const css = getComputedStyle(document.documentElement);
  const col = n => css.getPropertyValue(n).trim();
  // 每秒一个点：用相隔约 1 秒的两条统计求增量
  const pts = [];
  for (let i = hist.length - 1; i > 0 && pts.length < 60; i -= 5) {
    const a = hist[i], b = hist[Math.max(0, i - 5)];
    const dt = (a.t - b.t) / 1000;
    if (dt < 0.6) break;   // 刚连上时补发的旧统计和新统计挨得很近，算出的速率不可信
    pts.unshift({ ok: (a.ok - b.ok) / dt, bad: Math.max(0, a.bad - b.bad) / dt, gap: Math.max(0, a.link_gaps - b.link_gaps) / dt });
  }
  const L = 34, R = 8, T = 8, B = 18, W = w - L - R, H = h - T - B;
  const maxV = Math.max(10, ...pts.map(p => Math.max(p.ok, p.bad, p.gap)));
  // 刻度取 1/2/5×10^n，纵轴 4 格
  const raw = maxV / 4, mag = Math.pow(10, Math.floor(Math.log10(raw)));
  const step = [1, 2, 5, 10].map(k => k * mag).find(v => v >= raw);
  const top = step * 4;
  g.font = '11px ui-monospace, Menlo, Consolas, monospace';
  g.fillStyle = col('--muted'); g.strokeStyle = col('--chart-grid'); g.lineWidth = 1;
  for (let k = 0; k <= 4; k++) {
    const y = T + H - (H * k) / 4;
    g.beginPath(); g.moveTo(L, y + .5); g.lineTo(L + W, y + .5); g.stroke();
    g.textAlign = 'right'; g.fillText(String(Math.round((top * k) / 4)), L - 6, y + 4);
  }
  g.textAlign = 'left'; g.fillText('-60s', L, h - 4); g.textAlign = 'right'; g.fillText('现在', L + W, h - 4);
  const x = i => L + W - (pts.length - 1 - i) * (W / 59);
  const y = v => T + H - (Math.min(v, top) / top) * H;
  const line = (key, color, fill) => {
    if (pts.length < 2) return;
    g.beginPath();
    pts.forEach((p, i) => (i ? g.lineTo(x(i), y(p[key])) : g.moveTo(x(i), y(p[key]))));
    if (fill) {
      g.save(); g.lineTo(x(pts.length - 1), T + H); g.lineTo(x(0), T + H); g.closePath();
      g.globalAlpha = .12; g.fillStyle = color; g.fill(); g.restore();
      g.beginPath(); pts.forEach((p, i) => (i ? g.lineTo(x(i), y(p[key])) : g.moveTo(x(i), y(p[key]))));
    }
    g.strokeStyle = color; g.lineWidth = 1.8; g.stroke();
    const lp = pts[pts.length - 1];
    g.fillStyle = color; g.beginPath(); g.arc(x(pts.length - 1), y(lp[key]), 3, 0, 7); g.fill();
  };
  line('ok', col('--accent'), true);
  line('bad', col('--bad'));
  line('gap', col('--warn'));
}
window.addEventListener('resize', drawChart);

// ---------------- 校验结果 ----------------
function renderVerify(v) {
  if (v.error) { $('verify').innerHTML = `<p class="res bad">${esc(v.error)}</p>`; return; }
  const ok = v.crc_errors === 0;
  const r = v.record || {}, iv = v.interval_ms;
  const rows = [
    ['文件', `${v.files.join(', ')}（${(v.bytes / 1e6).toFixed(1)} MB）`],
    ['帧数', `${fmtN(v.frames)}（录像 ${(+r.secs).toFixed(1)} 秒）`],
    ['CRC 错误', fmtN(v.crc_errors)],
    ['帧号缺口', `${fmtN(v.id_gaps)}（约缺 ${fmtN(v.missing)} 帧）`],
    ['程序内丢弃', `队列满 ${fmtN(r.drop_queue)} · 帧池空 ${fmtN(r.drop_pool)}`],
    ['帧间隔 ms', iv ? `p50 ${iv.p50} · p99 ${iv.p99} · max ${iv.max}${iv.clock === 'host' ? '（主机时间）' : ''}` : '—'],
    ['最慢写盘', `${((r.max_write_us || 0) / 1000).toFixed(1)} ms`],
  ];
  $('verify').innerHTML = `<p class="res ${ok ? 'ok' : 'bad'}">${ok ? '✓ 数据完整：CRC 全部通过' : '✗ 有 CRC 错误'}${v.id_gaps ? `，但有 ${v.id_gaps} 处帧号缺口` : ''}</p>
    <table>${rows.map(([k, x]) => `<tr><td>${k}</td><td>${esc(x)}</td></tr>`).join('')}</table>`;
}

// ---------------- 日志 ----------------
function addLog(level, msg) {
  const li = document.createElement('li');
  li.dataset.l = level;
  const t = new Date().toTimeString().slice(0, 8);
  li.innerHTML = `<time>${t}</time><span>${esc(msg)}</span>`;
  const ul = $('log');
  ul.prepend(li);
  while (ul.children.length > 60) ul.lastChild.remove();
}

// ---------------- 操作 ----------------
$('btnControl').onclick = () => send(iControl() ? 'release' : 'take');
$('btnList').onclick = () => send('list');
$('btnStart').onclick = () => send('start');
$('btnStop').onclick = () => send('stop');
$('btnTrigger').onclick = () => send('trigger');
$('btnReset').onclick = () => { hist.length = 0; send('reset'); };
$('btnRecord').onclick = () => send('record', { secs: +$('recSecs').value });
$('btnDefaults').onclick = () => send('defaults');
$('btnCable').onclick = () => {
  const cam = meta?.cams.find(c => c.serial === meta.selected);
  if (cam) send('cable', { serial: cam.serial, plugged: !cam.plugged });
};
$('cams').addEventListener('click', e => {
  const b = e.target.closest('[data-open]');
  if (b) send('open', { serial: b.dataset.open });
});
$('lossSeg').addEventListener('click', e => {
  const b = e.target.closest('[data-loss]');
  if (b && meta) send('loss', { serial: meta.selected, permille: +b.dataset.loss });
});
document.querySelectorAll('[data-key]').forEach(el => el.addEventListener('change', () => {
  send('set', { key: el.dataset.key, value: el.value });
  if (el.dataset.key === 'trigger' || el.dataset.key === 'size' || el.dataset.key === 'packet_size') hist.length = 0;
}));
$('procRange').addEventListener('input', () => { $('procOut').textContent = `${$('procRange').value} ms`; });
$('procRange').addEventListener('change', () => send('set', { key: 'process_ms', value: $('procRange').value }));
$('diskRange').addEventListener('input', () => { $('diskOut').textContent = `${$('diskRange').value} ms`; });
$('diskRange').addEventListener('change', () => send('set', { key: 'slow_disk_us', value: String($('diskRange').value * 1000) }));

connect();

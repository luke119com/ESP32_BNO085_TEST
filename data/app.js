'use strict';
const $ = id => document.getElementById(id);

// ---------------- 分頁切換 ----------------
document.querySelectorAll('.tab').forEach(t => {
  t.onclick = () => {
    document.querySelectorAll('.tab').forEach(x => x.classList.remove('active'));
    document.querySelectorAll('.page').forEach(x => x.classList.remove('active'));
    t.classList.add('active');
    $(t.dataset.tab).classList.add('active');
    if (t.dataset.tab === 'wifi') loadInfo();
    if (t.dataset.tab === 'ota') loadInfo();
  };
});

// ---------------- 四元數工具 ----------------
// 四元數以 [i, j, k, r] 表示 (r = w 實部)
function qMul(a, b) {
  const [ai, aj, ak, ar] = a, [bi, bj, bk, br] = b;
  return [
    ar*bi + ai*br + aj*bk - ak*bj,
    ar*bj - ai*bk + aj*br + ak*bi,
    ar*bk + ai*bj - aj*bi + ak*br,
    ar*br - ai*bi - aj*bj - ak*bk,
  ];
}
const qConj = q => [-q[0], -q[1], -q[2], q[3]];

// 四元數 -> 尤拉角 (度)
function quatToEuler(i, j, k, r) {
  const roll  = Math.atan2(2*(r*i + j*k), 1 - 2*(i*i + j*j));
  let s = 2*(r*j - k*i); s = Math.max(-1, Math.min(1, s));
  const pitch = Math.asin(s);
  const yaw   = Math.atan2(2*(r*k + i*j), 1 - 2*(j*j + k*k));
  const d = 180/Math.PI;
  return { roll: roll*d, pitch: pitch*d, yaw: yaw*d };
}

// 四元數 -> CSS matrix3d (含 Y/Z 翻轉以符合螢幕座標)
function quatToMatrix3d(q) {
  const i = q[0], j = -q[1], k = -q[2], r = q[3]; // 轉螢幕座標
  const m00 = 1-2*(j*j+k*k), m01 = 2*(i*j-k*r),   m02 = 2*(i*k+j*r);
  const m10 = 2*(i*j+k*r),   m11 = 1-2*(i*i+k*k), m12 = 2*(j*k-i*r);
  const m20 = 2*(i*k-j*r),   m21 = 2*(j*k+i*r),   m22 = 1-2*(i*i+j*j);
  // matrix3d 為 column-major
  return `matrix3d(${m00},${m10},${m20},0,${m01},${m11},${m21},0,${m02},${m12},${m22},0,0,0,0,1)`;
}

// ---------------- 即時資料狀態 ----------------
let refQuat = null;              // 歸零視角的參考四元數
let pktCount = 0, lastHzTime = performance.now(), hzFrames = 0;

$('btnReset').onclick = () => { refQuat = null; };  // 下一筆資料設為新基準

function fmt(n, d = 2) { return Number(n).toFixed(d); }
function setCal(el, v) {
  el.textContent = v;
  el.className = 'cal c' + v;
}

// ---------------- 處理感測封包 ----------------
function onData(d) {
  pktCount++; hzFrames++;
  $('pkts').textContent = pktCount;

  // --- 3D:使用遊戲旋轉向量(無磁力漂移,較穩)---
  let q = d.grv;
  if (!refQuat) refQuat = qConj(q);          // 按下歸零時重新取基準
  const rel = qMul(refQuat, q);
  $('cube').style.transform = quatToMatrix3d(rel);

  const e = quatToEuler(q[0], q[1], q[2], q[3]);
  $('roll').textContent  = fmt(e.roll, 0) + '°';
  $('pitch').textContent = fmt(e.pitch, 0) + '°';
  $('yaw').textContent   = fmt(e.yaw, 0) + '°';

  // --- 指南針:使用地磁旋轉向量的 yaw ---
  const ge = quatToEuler(d.geo[0], d.geo[1], d.geo[2], d.geo[3]);
  let heading = (-ge.yaw + 360) % 360;       // 轉為 0~360,順時針
  drawCompass(heading);
  $('heading').textContent = fmt(heading, 0) + '°';
  $('headingDir').textContent = dirName(heading);

  // --- 校正等級 ---
  setCal($('calMag'),  d.cal[2]);
  setCal($('calMag2'), d.cal[2]);
  setCal($('calAcc'),  d.cal[0]);
  setCal($('calGyr'),  d.cal[1]);

  // --- 感測器數值 ---
  $('accX').textContent = fmt(d.acc[0]); $('accY').textContent = fmt(d.acc[1]); $('accZ').textContent = fmt(d.acc[2]);
  $('gyrX').textContent = fmt(d.gyr[0]); $('gyrY').textContent = fmt(d.gyr[1]); $('gyrZ').textContent = fmt(d.gyr[2]);
  $('magX').textContent = fmt(d.mag[0], 1); $('magY').textContent = fmt(d.mag[1], 1); $('magZ').textContent = fmt(d.mag[2], 1);
  $('linX').textContent = fmt(d.lin[0]); $('linY').textContent = fmt(d.lin[1]); $('linZ').textContent = fmt(d.lin[2]);
  $('gravX').textContent = fmt(d.grav[0]); $('gravY').textContent = fmt(d.grav[1]); $('gravZ').textContent = fmt(d.grav[2]);
  $('qi').textContent = fmt(d.rot[0], 3); $('qj').textContent = fmt(d.rot[1], 3);
  $('qk').textContent = fmt(d.rot[2], 3); $('qr').textContent = fmt(d.rot[3], 3);
  $('rotAcc').textContent = fmt(d.rotAcc * 180/Math.PI, 1);

  // --- 事件 ---
  $('steps').textContent = d.steps;
  $('stab').textContent  = stabName(d.stab);
  $('tap').textContent   = d.tap;
  $('shake').textContent = d.shake;
  $('tapAgo').textContent   = d.tap   ? (agoTxt(d.tapAgo))   : '';
  $('shakeAgo').textContent = d.shake ? (agoTxt(d.shakeAgo)) : '';
}

const agoTxt = ms => ms < 1500 ? '剛剛!' : (Math.round(ms/1000) + ' 秒前');
function stabName(s){return ['未知','置於桌面','靜止','穩定','移動中'][s] || '未知';}
function dirName(h){
  const names = ['N','NE','E','SE','S','SW','W','NW'];
  return names[Math.round(h/45)%8];
}

// ---------------- 指南針繪製 ----------------
const cc = $('compass').getContext('2d');
function drawCompass(heading) {
  const w = 260, cx = 130, cy = 130, R = 110;
  cc.clearRect(0, 0, w, w);
  // 外圈
  cc.strokeStyle = '#30363d'; cc.lineWidth = 2;
  cc.beginPath(); cc.arc(cx, cy, R, 0, 2*Math.PI); cc.stroke();

  cc.save();
  cc.translate(cx, cy);
  cc.rotate(-heading * Math.PI/180);   // 轉動整個羅盤,讓 N 指向實際北方
  // 刻度
  for (let a = 0; a < 360; a += 15) {
    const rad = a*Math.PI/180, big = (a % 90 === 0);
    cc.strokeStyle = big ? '#8b949e' : '#3d444d';
    cc.lineWidth = big ? 2 : 1;
    cc.beginPath();
    cc.moveTo(0, -R);
    cc.lineTo(0, -R + (big ? 16 : 8));
    cc.stroke();
    cc.rotate(15*Math.PI/180);
  }
  // 方位字
  const labels = [['N', '#f85149'], ['E', '#e6edf3'], ['S', '#e6edf3'], ['W', '#e6edf3']];
  cc.font = 'bold 18px sans-serif'; cc.textAlign = 'center'; cc.textBaseline = 'middle';
  labels.forEach((l, idx) => {
    const rad = idx*90*Math.PI/180;
    const x = Math.sin(rad)*(R-32), y = -Math.cos(rad)*(R-32);
    cc.fillStyle = l[1]; cc.fillText(l[0], x, y);
  });
  cc.restore();

  // 固定指針(永遠朝上,代表裝置正面方向)
  cc.fillStyle = '#2f81f7';
  cc.beginPath();
  cc.moveTo(cx, cy - 60); cc.lineTo(cx - 10, cy); cc.lineTo(cx + 10, cy); cc.closePath();
  cc.fill();
  cc.fillStyle = '#8b949e';
  cc.beginPath();
  cc.moveTo(cx, cy + 60); cc.lineTo(cx - 10, cy); cc.lineTo(cx + 10, cy); cc.closePath();
  cc.fill();
  cc.beginPath(); cc.arc(cx, cy, 6, 0, 2*Math.PI); cc.fillStyle = '#e6edf3'; cc.fill();
}
drawCompass(0);

// ---------------- 更新率統計 ----------------
setInterval(() => {
  const now = performance.now();
  const hz = hzFrames * 1000 / (now - lastHzTime);
  $('hz').textContent = hz.toFixed(0);
  hzFrames = 0; lastHzTime = now;
}, 1000);

// ---------------- 擷取率調整 ----------------
const rateSlider = $('rateSlider'), rateVal = $('rateVal');
rateSlider.oninput = () => {
  rateVal.textContent = rateSlider.value;
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ rateHz: +rateSlider.value }));
  }
};
// 韌體回報目前設定時,同步滑桿範圍與數值
function applyCfg(cfg) {
  if (cfg.min != null) rateSlider.min = cfg.min;
  if (cfg.max != null) rateSlider.max = cfg.max;
  if (cfg.rateHz != null) {
    rateSlider.value = Math.round(cfg.rateHz);
    rateVal.textContent = Math.round(cfg.rateHz);
  }
}

// ---------------- WebSocket ----------------
let ws;
function connectWS() {
  ws = new WebSocket(`ws://${location.host}/ws`);
  ws.onopen = () => { $('dot').className = 'dot on'; $('connText').textContent = '已連線'; };
  ws.onclose = () => {
    $('dot').className = 'dot off'; $('connText').textContent = '斷線,重連中…';
    setTimeout(connectWS, 1500);
  };
  ws.onmessage = ev => {
    try {
      const d = JSON.parse(ev.data);
      if (d.cfg) { applyCfg(d.cfg); return; }   // 設定封包
      onData(d);                                 // 感測資料封包
    } catch (e) {}
  };
}
connectWS();

// ---------------- 系統資訊 ----------------
async function loadInfo() {
  try {
    const r = await fetch('/api/info'); const d = await r.json();
    $('iMode').textContent = d.mode;
    $('iSsid').textContent = d.ssid;
    $('iIp').textContent   = d.ip;
    $('iRssi').textContent = d.mode === 'AP' ? '—' : d.rssi + ' dBm';
    $('iMac').textContent  = d.mac;
    $('iChip').textContent = d.chip;
    $('iHeap').textContent = (d.heap/1024).toFixed(1) + ' KB';
    $('iUp').textContent   = d.uptime + ' 秒';
  } catch (e) {}
}
loadInfo();

// ---------------- WiFi 掃描 / 儲存 ----------------
$('btnScan').onclick = async () => {
  $('btnScan').textContent = '掃描中…'; $('btnScan').disabled = true;
  const poll = async () => {
    const r = await fetch('/api/wifi/scan');
    if (r.status === 202) { setTimeout(poll, 1200); return; }
    const d = await r.json();
    const ul = $('apList'); ul.innerHTML = '';
    (d.networks || []).sort((a,b)=>b.rssi-a.rssi).forEach(n => {
      const li = document.createElement('li');
      li.innerHTML = `<span>${n.enc ? '🔒 ' : ''}${n.ssid}</span><small>${n.rssi} dBm</small>`;
      li.onclick = () => { $('ssidInput').value = n.ssid; $('passInput').focus(); };
      ul.appendChild(li);
    });
    $('btnScan').textContent = '掃描網路'; $('btnScan').disabled = false;
  };
  poll();
};

$('wifiForm').onsubmit = async e => {
  e.preventDefault();
  const fd = new FormData(e.target);
  const msg = $('wifiMsg'); msg.className = 'msg'; msg.textContent = '儲存中…';
  try {
    const r = await fetch('/api/wifi/save', { method: 'POST', body: new URLSearchParams(fd) });
    const d = await r.json();
    msg.className = 'msg ' + (d.ok ? 'ok' : 'err');
    msg.textContent = d.msg + (d.ok ? '(請稍後改連新網路)' : '');
  } catch (e) { msg.className = 'msg err'; msg.textContent = '裝置重啟中,連線已中斷(正常現象)'; }
};

// ---------------- OTA 上傳 ----------------
$('btnUpload').onclick = () => {
  const f = $('fwFile').files[0];
  const msg = $('otaMsg'), bar = $('bar');
  if (!f) { msg.className = 'msg err'; msg.textContent = '請先選擇 .bin 檔'; return; }
  msg.className = 'msg'; msg.textContent = '上傳中…';
  const xhr = new XMLHttpRequest();
  xhr.open('POST', '/api/update');
  xhr.upload.onprogress = e => {
    if (e.lengthComputable) bar.style.width = (e.loaded/e.total*100).toFixed(0) + '%';
  };
  xhr.onload = () => {
    try {
      const d = JSON.parse(xhr.responseText);
      msg.className = 'msg ' + (d.ok ? 'ok' : 'err');
      msg.textContent = d.msg;
    } catch (e) {
      msg.className = 'msg ok'; msg.textContent = '已上傳,裝置重新啟動中…';
    }
  };
  xhr.onerror = () => { msg.className = 'msg err'; msg.textContent = '上傳失敗'; };
  const fd = new FormData(); fd.append('firmware', f);
  xhr.send(fd);
};

$('btnReboot').onclick = async () => {
  if (!confirm('確定要重新開機?')) return;
  await fetch('/api/reboot', { method: 'POST' });
  $('otaMsg').textContent = '重新開機中…';
};

const users = {
  admin: { password: 'admin123', name: '系统管理员', role: '管理员' },
  doctor: { password: 'doctor123', name: '值班医生', role: '医生' },
  operator: { password: 'operator123', name: '设备操作员', role: '操作员' }
};
const permissions = {
  '管理员': { settings: true, adjust: true, operate: true, records: true, clear: true },
  '医生': { settings: true, adjust: true, operate: true, records: true, clear: false },
  '操作员': { settings: false, adjust: true, operate: true, records: true, clear: false }
};

let sessionUser = null;
const TARGET_FORCE_MIN = 0;
const TARGET_FORCE_MAX = 10;
const savedTargetText = localStorage.getItem('tractionTarget');
const savedTarget = savedTargetText === null ? NaN : Number(savedTargetText);
let currentForce = Number.isFinite(savedTarget) ? Math.max(TARGET_FORCE_MIN, Math.min(TARGET_FORCE_MAX, savedTarget)) : 10;
let forceLimit = Number(localStorage.getItem('tractionLimit') || 30);
let actualForce = 0;
let activeRecord = null;
let recordSamples = [];
let timerHandle = null;
let records = JSON.parse(localStorage.getItem('tractionRecords') || '[]');
let stateSocket = null;
let reconnectHandle = null;
let lastStateAt = 0;
let dataOnline = false;
let directionLocked = false;

const $ = id => document.getElementById(id);
const toast = message => {
  $('toast').textContent = message;
  $('toast').classList.add('show');
  clearTimeout(toast.timer);
  toast.timer = setTimeout(() => $('toast').classList.remove('show'), 2200);
};
const formatTime = date => new Intl.DateTimeFormat('zh-CN', {
  year: 'numeric', month: '2-digit', day: '2-digit',
  hour: '2-digit', minute: '2-digit', second: '2-digit', hour12: false
}).format(date).replaceAll('/', '-');

async function postJson(path, body = {}) {
  const response = await fetch(path, {
    method: 'POST',
    headers: { 'Content-Type': 'application/json' },
    body: JSON.stringify(body)
  });
  const data = await response.json();
  if (!response.ok) throw new Error(data.detail || '操作失败');
  return data;
}

function login() {
  const account = users[$('username').value.trim()];
  if (!account || account.password !== $('password').value) {
    $('loginError').textContent = '用户名或密码错误';
    return;
  }
  sessionUser = { username: $('username').value.trim(), ...account };
  sessionStorage.setItem('tractionSession', JSON.stringify(sessionUser));
  $('loginError').textContent = '';
  $('loginModal').classList.add('hidden');
  applyPermissions();
  toast('登录成功');
}

function applyPermissions() {
  if (!sessionUser) return;
  const permission = permissions[sessionUser.role];
  $('currentUser').textContent = sessionUser.name;
  $('currentRole').textContent = sessionUser.role;
  $('settingsBtn').disabled = !permission.settings;
  document.querySelectorAll('.force-adjust').forEach(button => {
    button.disabled = !permission.adjust || !!activeRecord;
  });
  $('targetForceVal').disabled = !permission.adjust || !!activeRecord;
  $('startBtn').disabled = !permission.operate || !!activeRecord || !dataOnline;
  $('stopBtn').disabled = !permission.operate || !activeRecord;
  $('recordsBtn').disabled = !permission.records;
  $('clearRecordsBtn').style.display = permission.clear ? '' : 'none';
}

function logout() {
  if (activeRecord && !confirm('牵引记录尚未结束，确定退出并将本次标记为紧急终止吗？')) return;
  if (activeRecord) emergencyStop();
  sessionUser = null;
  sessionStorage.removeItem('tractionSession');
  $('password').value = '';
  $('loginModal').classList.remove('hidden');
}

function updateForceDisplay() {
  $('targetForceVal').value = Number(currentForce.toFixed(1));
  $('targetForceVal').style.color = currentForce >= forceLimit ? '#dc2626' : '#1e3a5f';
  localStorage.setItem('tractionTarget', currentForce);
}

async function changeTarget(nextTarget) {
  if (!sessionUser || !permissions[sessionUser.role].adjust) {
    return toast('当前角色无权修改牵引参数');
  }
  if (activeRecord) return toast('请先结束当前牵引');
  const numericTarget = Number(nextTarget);
  if (!Number.isFinite(numericTarget) || numericTarget < TARGET_FORCE_MIN || numericTarget > TARGET_FORCE_MAX) {
    updateForceDisplay();
    return toast('目标牵引力请输入 0～10 N');
  }
  currentForce = Math.round(numericTarget * 10) / 10;
  updateForceDisplay();
  try {
    await postJson('/api/traction/target', { target_force_n: currentForce });
  } catch (error) {
    toast(error.message);
  }
}

async function startTraction() {
  if (!sessionUser || !permissions[sessionUser.role].operate) return;
  if (!dataOnline) return toast('真实设备数据不可用');
  try {
    await postJson('/api/traction/start', { target_force_n: currentForce });
  } catch (error) {
    return toast(error.message);
  }

  const now = new Date();
  activeRecord = {
    id: 'TR' + now.getFullYear()
      + String(now.getMonth() + 1).padStart(2, '0')
      + String(now.getDate()).padStart(2, '0')
      + '-' + String(Date.now()).slice(-6),
    start: formatTime(now),
    end: '',
    operator: sessionUser.name,
    role: sessionUser.role,
    target: currentForce,
    average: 0,
    maximum: 0,
    status: '进行中'
  };
  recordSamples = [];
  $('workStatus').textContent = '请轻拉确定方向';
  $('workStatus').classList.add('running');
  const startAt = Date.now();
  timerHandle = setInterval(() => {
    const seconds = Math.floor((Date.now() - startAt) / 1000);
    $('recordTimer').textContent =
      `记录中 ${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
  }, 1000);
  applyPermissions();
  toast('牵引已开始，请沿需要牵引的方向轻拉一次');
}

function saveFinishedRecord(status) {
  if (!activeRecord) return;
  clearInterval(timerHandle);
  activeRecord.end = formatTime(new Date());
  activeRecord.status = status;
  activeRecord.average = recordSamples.length
    ? (recordSamples.reduce((sum, value) => sum + value, 0) / recordSamples.length).toFixed(1)
    : '0.0';
  activeRecord.maximum = recordSamples.length
    ? Math.max(...recordSamples).toFixed(1)
    : '0.0';
  records.unshift(activeRecord);
  localStorage.setItem('tractionRecords', JSON.stringify(records));
  activeRecord = null;
  recordSamples = [];
  directionLocked = false;
  actualForce = 0;
  $('actualForceVal').textContent = '0.0';
  $('recordTimer').textContent = '未开始记录';
  $('workStatus').textContent = status === '已完成' ? '设备就绪' : '安全停止';
  $('workStatus').classList.remove('running');
  applyPermissions();
  renderRecords();
}

async function finishTraction(status = '已完成') {
  if (!activeRecord) return;
  try {
    await postJson('/api/traction/stop');
  } catch (error) {
    toast(error.message);
  }
  saveFinishedRecord(status);
  toast(status === '已完成' ? '牵引记录已保存' : '牵引已停止并保存记录');
}

async function emergencyStop() {
  try {
    await postJson('/api/traction/emergency-stop');
  } catch (error) {
    toast(error.message);
  }
  if (activeRecord) saveFinishedRecord('紧急终止');
  $('workStatus').textContent = '安全停止';
  toast('牵引已停止');
}

function renderRecords() {
  const keyword = $('recordSearch').value.trim().toLowerCase();
  const status = $('recordStatusFilter').value;
  const list = records.filter(record =>
    (!keyword || record.id.toLowerCase().includes(keyword)
      || record.operator.toLowerCase().includes(keyword))
    && (!status || record.status === status)
  );
  $('recordsBody').innerHTML = list.length
    ? list.map(record => `<tr><td>${record.id}</td><td>${record.start}</td><td>${record.end}</td><td>${record.operator}</td><td>${record.role}</td><td>${record.target}</td><td>${record.average}</td><td>${record.maximum}</td><td>${record.status}</td></tr>`).join('')
    : '<tr><td colspan="9" class="empty">暂无符合条件的牵引记录</td></tr>';
}

function exportRecords() {
  if (!records.length) return toast('暂无可导出的记录');
  const rows = [
    ['记录编号', '开始时间', '结束时间', '操作人员', '角色', '目标力(N)', '平均力(N)', '最大力(N)', '状态'],
    ...records.map(record => [
      record.id, record.start, record.end, record.operator, record.role,
      record.target, record.average, record.maximum, record.status
    ])
  ];
  const csv = '\uFEFF' + rows.map(row =>
    row.map(value => `"${String(value).replaceAll('"', '""')}"`).join(',')
  ).join('\r\n');
  const link = document.createElement('a');
  link.href = URL.createObjectURL(new Blob([csv], { type: 'text/csv;charset=utf-8' }));
  link.download = `牵引记录_${new Date().toISOString().slice(0, 10)}.csv`;
  link.click();
  URL.revokeObjectURL(link.href);
}

const canvas = $('forceCanvas');
const context = canvas.getContext('2d');
let dataPoints = Array(90).fill(0);

function resizeCanvas() {
  const rectangle = canvas.getBoundingClientRect();
  canvas.width = Math.max(100, rectangle.width);
  canvas.height = Math.max(100, rectangle.height);
  drawCurve();
}

function drawCurve() {
  const width = canvas.width;
  const height = canvas.height;
  context.clearRect(0, 0, width, height);
  context.fillStyle = '#f8fafc';
  context.fillRect(0, 0, width, height);
  context.strokeStyle = '#e2e8f0';
  context.lineWidth = 1;
  for (let index = 1; index < 5; index += 1) {
    const y = height * index / 5;
    context.beginPath();
    context.moveTo(0, y);
    context.lineTo(width, y);
    context.stroke();
  }
  for (let index = 1; index < 9; index += 1) {
    const x = width * index / 9;
    context.beginPath();
    context.moveTo(x, 0);
    context.lineTo(x, height);
    context.stroke();
  }
  context.strokeStyle = '#3b82f6';
  context.lineWidth = 3;
  context.beginPath();
  const maximum = Math.max(40, forceLimit + 10);
  const step = width / (dataPoints.length - 1);
  dataPoints.forEach((value, index) => {
    const x = index * step;
    const y = height - (value / maximum) * height * .82 - height * .08;
    index ? context.lineTo(x, y) : context.moveTo(x, y);
  });
  context.stroke();
  context.lineTo(width, height);
  context.lineTo(0, height);
  context.closePath();
  const gradient = context.createLinearGradient(0, 0, 0, height);
  gradient.addColorStop(0, 'rgba(59,130,246,.24)');
  gradient.addColorStop(1, 'rgba(59,130,246,.02)');
  context.fillStyle = gradient;
  context.fill();
  context.fillStyle = '#64748b';
  context.font = '12px sans-serif';
  context.fillText(`${maximum} N`, 6, 15);
  context.fillText('0 N', 6, height - 7);
  context.fillText('时间 →', width - 52, height - 7);
}

function handleState(state) {
  lastStateAt = Date.now();
  dataOnline = Boolean(
    state.gateway && state.gateway.valid
    && state.fr5 && state.fr5.valid
    && state.kwr75d && state.kwr75d.valid
  );
  $('armStatus').textContent = dataOnline ? '通信正常' : '通信中断';
  $('armStatus').classList.toggle('offline', !dataOnline);
  const joints = state.fr5 && state.fr5.joint_position_deg;
  if (Array.isArray(joints) && joints.length === 6) {
    window.latestFR5Joints = joints;
    if (window.updateFR5Joints) window.updateFR5Joints(joints);
  }
  if (state.ag95 && window.updateAG95) {
    window.updateAG95(state.ag95.position_raw);
  }

  const traction = state.traction || {};
  directionLocked = Boolean(traction.direction_locked);
  actualForce = Number(traction.measured_force_n || 0);
  $('actualForceVal').textContent = actualForce.toFixed(1);

  if (activeRecord) {
    $('workStatus').textContent = traction.message || '牵引进行中';
    $('workStatus').classList.add('running');
    if (directionLocked) recordSamples.push(actualForce);
  } else if (!dataOnline) {
    $('workStatus').textContent = '设备未就绪';
    $('workStatus').classList.remove('running');
  } else {
    $('workStatus').textContent = '设备就绪';
    $('workStatus').classList.remove('running');
  }

  dataPoints.shift();
  dataPoints.push(activeRecord && directionLocked ? actualForce : 0);
  drawCurve();
  if (sessionUser) applyPermissions();
}

function connectStateStream() {
  clearTimeout(reconnectHandle);
  const protocol = location.protocol === 'https:' ? 'wss:' : 'ws:';
  stateSocket = new WebSocket(`${protocol}//${location.host}/ws`);
  stateSocket.onmessage = event => {
    try {
      handleState(JSON.parse(event.data));
    } catch (error) {
      console.error('状态数据解析失败', error);
    }
  };
  stateSocket.onclose = () => {
    dataOnline = false;
    $('armStatus').textContent = '通信中断';
    $('armStatus').classList.add('offline');
    if (sessionUser) applyPermissions();
    reconnectHandle = setTimeout(connectStateStream, 1000);
  };
}

setInterval(() => {
  if (lastStateAt && Date.now() - lastStateAt > 1800) {
    dataOnline = false;
    $('armStatus').textContent = '通信中断';
    $('armStatus').classList.add('offline');
    $('workStatus').textContent = '设备未就绪';
    if (sessionUser) applyPermissions();
  }
}, 500);

document.querySelectorAll('.force-adjust').forEach(button => {
  button.addEventListener('click', () => {
    const nextTarget = Math.max(TARGET_FORCE_MIN, Math.min(TARGET_FORCE_MAX, currentForce + Number(button.dataset.step)));
    changeTarget(nextTarget);
  });
});
$('targetForceVal').addEventListener('change', event => changeTarget(event.target.value));
$('targetForceVal').addEventListener('keydown', event => {
  if (event.key === 'Enter') {
    event.preventDefault();
    event.target.blur();
  }
});
$('loginBtn').addEventListener('click', login);
$('password').addEventListener('keydown', event => {
  if (event.key === 'Enter') login();
});
$('logoutBtn').addEventListener('click', logout);
$('startBtn').addEventListener('click', startTraction);
$('stopBtn').addEventListener('click', () => finishTraction('已完成'));
$('emergencyBtn').addEventListener('click', emergencyStop);
$('recordsBtn').addEventListener('click', () => {
  renderRecords();
  $('recordsModal').classList.remove('hidden');
});
$('settingsBtn').addEventListener('click', () => {
  if (!sessionUser || !permissions[sessionUser.role].settings) {
    return toast('当前角色无权修改参数');
  }
  $('settingTarget').value = currentForce;
  $('settingLimit').value = forceLimit;
  $('settingsModal').classList.remove('hidden');
});
$('saveSettingsBtn').addEventListener('click', async () => {
  const limit = Number($('settingLimit').value);
  const target = Number($('settingTarget').value);
  if (limit < 5 || limit > 50 || target < TARGET_FORCE_MIN || target > TARGET_FORCE_MAX) {
    return toast('请输入有效参数，目标牵引力必须在 0～10 N');
  }
  forceLimit = limit;
  localStorage.setItem('tractionLimit', forceLimit);
  await changeTarget(target);
  $('settingsModal').classList.add('hidden');
  toast('参数已保存');
});
$('recordSearch').addEventListener('input', renderRecords);
$('recordStatusFilter').addEventListener('change', renderRecords);
$('exportBtn').addEventListener('click', exportRecords);
$('clearRecordsBtn').addEventListener('click', () => {
  if (confirm('确定清空全部牵引记录吗？该操作不可恢复。')) {
    records = [];
    localStorage.removeItem('tractionRecords');
    renderRecords();
    toast('记录已清空');
  }
});
document.querySelectorAll('[data-close]').forEach(button => {
  button.addEventListener('click', () => $(button.dataset.close).classList.add('hidden'));
});
window.addEventListener('resize', resizeCanvas);

updateForceDisplay();
renderRecords();
setTimeout(resizeCanvas, 0);
connectStateStream();
const savedSession = sessionStorage.getItem('tractionSession');
if (savedSession) {
  sessionUser = JSON.parse(savedSession);
  $('loginModal').classList.add('hidden');
  applyPermissions();
}

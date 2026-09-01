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
const TARGET_FORCE_MIN = 1;
const TARGET_FORCE_MAX = 30;
const VALIDATED_TARGET_MAX = 15;
let currentForce = 10;
const forceLimit = 25;
let actualForce = 0;
let activeRecord = null;
let timerHandle = null;
let records = [];
let stateSocket = null;
let reconnectHandle = null;
let lastStateAt = 0;
let dataOnline = false;
let directionLocked = false;
let tractionState = 0;
let pendingStart = false;

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
  $('startBtn').disabled = !permission.operate || tractionState !== 5 || !dataOnline || pendingStart;
  $('stopBtn').disabled = !permission.operate || tractionState !== 6;
  if ($('prepareBtn')) $('prepareBtn').disabled = !permission.operate || !dataOnline || ![1, 2, 8].includes(tractionState);
  if ($('calibrateBtn')) $('calibrateBtn').disabled = !permission.operate || !dataOnline || tractionState !== 2;
  if ($('emergencyBtn')) $('emergencyBtn').disabled = !permission.operate || !dataOnline || tractionState === 10;
  if ($('resetBtn')) $('resetBtn').disabled = !permission.operate || !dataOnline || ![9, 10].includes(tractionState);
  const zeroOperationDisabled = !permission.operate || !dataOnline || ![1, 2, 8].includes(tractionState);
  if ($('setZeroBtn')) $('setZeroBtn').disabled = zeroOperationDisabled;
  if ($('returnZeroBtn')) $('returnZeroBtn').disabled = zeroOperationDisabled;
  $('recordsBtn').disabled = !permission.records;
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
}

async function changeTarget(nextTarget) {
  if (!sessionUser || !permissions[sessionUser.role].adjust) {
    return toast('当前角色无权修改牵引参数');
  }
  if (activeRecord) return toast('请先结束当前牵引');
  const numericTarget = Number(nextTarget);
  if (!Number.isFinite(numericTarget) || numericTarget < TARGET_FORCE_MIN || numericTarget > TARGET_FORCE_MAX) {
    updateForceDisplay();
    return toast('目标牵引力请输入 1～30 N');
  }
  const previousForce = currentForce;
  currentForce = Math.round(numericTarget * 10) / 10;
  updateForceDisplay();
  try {
    await postJson('/api/traction/target', { target_force_n: currentForce });
    if (currentForce > VALIDATED_TARGET_MAX) {
      toast(`已设置 ${currentForce.toFixed(1)}N；当前实机验收上限为 ${VALIDATED_TARGET_MAX}N`);
    }
    return true;
  } catch (error) {
    currentForce = previousForce;
    updateForceDisplay();
    toast(error.message);
    return false;
  }
}

async function startTraction() {
  if (!sessionUser || !permissions[sessionUser.role].operate) return;
  if (pendingStart) return toast('正在等待控制器接管');
  if (!dataOnline) return toast('ROS2 牵引管理器不可用');
  try {
    if (tractionState !== 5) return toast('请先完成方向标定并锁定方向');
    await postJson('/api/traction/start');
  } catch (error) {
    return toast(error.message);
  }

  pendingStart = true;
  $('workStatus').textContent = '等待控制器接管';
  $('workStatus').classList.add('running');
  applyPermissions();
  toast('已请求接管；控制器确认后开始记录');
}

function beginLocalRecord() {
  if (activeRecord || !pendingStart) return;
  pendingStart = false;
  activeRecord = { startedAt: Date.now() };
  $('workStatus').textContent = '恒力保持中';
  $('workStatus').classList.add('running');
  const startAt = Date.now();
  timerHandle = setInterval(() => {
    const seconds = Math.floor((Date.now() - startAt) / 1000);
    $('recordTimer').textContent =
      `记录中 ${String(Math.floor(seconds / 60)).padStart(2, '0')}:${String(seconds % 60).padStart(2, '0')}`;
  }, 1000);
}

function saveFinishedRecord(status) {
  pendingStart = false;
  if (!activeRecord) return;
  clearInterval(timerHandle);
  activeRecord = null;
  $('recordTimer').textContent = '未开始记录';
  $('workStatus').textContent = status === '已完成' ? '等待 ROS2 完成释放' : '软件急停已请求';
  $('workStatus').classList.remove('running');
  applyPermissions();
  renderRecords();
  refreshHistory();
}

async function finishTraction(status = '已完成') {
  if (tractionState !== 6) return toast('当前不在恒力牵引状态');
  try {
    await postJson('/api/traction/stop');
  } catch (error) {
    return toast(error.message);
  }
  saveFinishedRecord(status);
  toast(status === '已完成' ? '牵引记录已保存' : '牵引已停止并保存记录');
}

async function emergencyStop() {
  try {
    await postJson('/api/traction/emergency-stop');
  } catch (error) {
    return toast(`${error.message}；请立即使用实体急停`);
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

function builtinTimeText(value) {
  if (!value || !Number.isFinite(Number(value.sec))) return '--';
  return formatTime(new Date((Number(value.sec) * 1000) + Number(value.nanosec || 0) / 1e6));
}

async function refreshHistory() {
  try {
    const response = await fetch('/api/traction/history', { cache: 'no-store' });
    if (!response.ok) return;
    const history = await response.json();
    records = (history.summaries || []).map(summary => ({
      id: summary.session_id || '--',
      start: builtinTimeText(summary.start_time),
      end: builtinTimeText(summary.end_time),
      operator: '--',
      role: 'ROS2',
      target: Number(summary.target_force_n || 0).toFixed(1),
      average: Number(summary.average_force_n || 0).toFixed(1),
      maximum: Number(summary.max_force_n || 0).toFixed(1),
      status: Number(summary.final_state) === 8 ? '已完成' : '异常终止'
    }));
    renderRecords();
  } catch (_) {
    // The page remains usable; the next poll retries without fabricating data.
  }
}

function exportRecords() {
  if (!records.length) return toast('暂无 ROS2 牵引历史');
  const link = document.createElement('a');
  link.href = '/api/traction/export/latest';
  link.download = `牵引记录_${new Date().toISOString().slice(0, 10)}.csv`;
  link.click();
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
  const traction = state.traction || {};
  tractionState = Number(traction.state || 0);
  if (tractionState === 6) beginLocalRecord();
  if (pendingStart && [9, 10].includes(tractionState)) {
    pendingStart = false;
    toast(traction.message || '牵引接管失败，请检查设备状态');
  }
  dataOnline = state.connected === true;
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

  directionLocked = [5, 6, 7].includes(tractionState);
  actualForce = Number(traction.actual_force_n || 0);
  $('actualForceVal').textContent = actualForce.toFixed(1);
  const tensionDetected = actualForce >= 1.0;
  $('tensionState').textContent = tensionDetected
    ? '已张紧：检测到有效牵引力' : '松弛：尚未感知到有效张力';
  $('tensionState').classList.toggle('tight', tensionDetected);

  const vector = Array.isArray(traction.force_vector_n) ? traction.force_vector_n : [];
  ['forceFx', 'forceFy', 'forceFz'].forEach((id, index) => {
    $(id).textContent = Number.isFinite(Number(vector[index])) ? Number(vector[index]).toFixed(2) : '--';
  });
  const measuredDirection = Array.isArray(traction.force_direction_base) ? traction.force_direction_base : [];
  const increaseDirection = Array.isArray(traction.increase_direction_base)
    ? traction.increase_direction_base : [];
  $('forceDirection').textContent = measuredDirection.length === 3
    ? `[${measuredDirection.map(value => Number(value).toFixed(2)).join(', ')}]` : '--';
  $('increaseDirection').textContent = increaseDirection.length === 3
    ? `[${increaseDirection.map(value => Number(value).toFixed(2)).join(', ')}]` : '--';
  const directionForModel = increaseDirection.length === 3 ? increaseDirection : [0, 0, 0];
  if (window.updateForceVector && vector.length === 3) {
    window.updateForceVector(vector, directionForModel);
  }
  $('forceVectorMessage').textContent = traction.message || traction.stop_reason || (
    tractionState === 4 ? '正在确认方向，请保持机械臂静止' :
      directionLocked ? '方向已锁定，等待开始恒力牵引' : '移动中只显示力，不判定恒力'
  );

  if (activeRecord) {
    $('workStatus').textContent = traction.fault_code || traction.stop_reason || `ROS2 状态：${tractionState}`;
    $('workStatus').classList.add('running');
  } else if (!dataOnline) {
    $('workStatus').textContent = 'ROS2 牵引管理器不可用';
    $('workStatus').classList.remove('running');
  } else {
    $('workStatus').textContent = traction.state_name || '设备就绪';
    $('workStatus').classList.remove('running');
  }

  dataPoints.shift();
  dataPoints.push(directionLocked ? actualForce : 0);
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
    $('settingLimit').value = 30;
  $('settingLimit').disabled = true;
  $('settingsModal').classList.remove('hidden');
});
$('saveSettingsBtn').addEventListener('click', async () => {
  const target = Number($('settingTarget').value);
  if (target < TARGET_FORCE_MIN || target > TARGET_FORCE_MAX) {
    return toast('目标牵引力必须在 1～30 N；当前实机验收上限为 15 N');
  }
  if (await changeTarget(target)) {
    $('settingsModal').classList.add('hidden');
    toast('参数已保存');
  }
});
$('recordSearch').addEventListener('input', renderRecords);
$('recordStatusFilter').addEventListener('change', renderRecords);
$('exportBtn').addEventListener('click', exportRecords);
document.querySelectorAll('[data-close]').forEach(button => {
  button.addEventListener('click', () => $(button.dataset.close).classList.add('hidden'));
});
window.addEventListener('resize', resizeCanvas);

updateForceDisplay();
renderRecords();
refreshHistory();
setInterval(refreshHistory, 2000);
setTimeout(resizeCanvas, 0);
connectStateStream();
const callTraction = async path => {
  try {
    const result = await postJson(path);
    const traction = result.snapshot && result.snapshot.traction;
    if (traction) {
      tractionState = Number(traction.state || 0);
      dataOnline = traction.valid === true;
      applyPermissions();
    }
    if (result.message) toast(result.message);
    return result;
  } catch (error) {
    toast(error.message);
    return null;
  }
};
if ($('prepareBtn')) $('prepareBtn').addEventListener('click', () => callTraction('/api/traction/prepare'));
if ($('calibrateBtn')) {
  $('calibrateBtn').addEventListener('click', () => callTraction('/api/traction/calibrate-direction'));
}
if ($('resetBtn')) $('resetBtn').addEventListener('click', () => callTraction('/api/traction/reset-fault'));
if ($('setZeroBtn')) $('setZeroBtn').addEventListener('click', () => callTraction('/api/traction/set-zero'));
if ($('returnZeroBtn')) $('returnZeroBtn').addEventListener('click', () => callTraction('/api/traction/return-zero'));
setInterval(() => { fetch('/api/traction/heartbeat', {method: 'POST', body: '{}'}).catch(() => {}); }, 500);
const savedSession = sessionStorage.getItem('tractionSession');
if (savedSession) {
  sessionUser = JSON.parse(savedSession);
  $('loginModal').classList.add('hidden');
  applyPermissions();
}

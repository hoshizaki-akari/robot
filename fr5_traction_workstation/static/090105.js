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
const TARGET_FORCE_MAX = 20;
let currentForce = 10;
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
let finishRequested = false;
let slackZeroAvailable = false;

const TRACTION_STATE_LABELS = {
  0: '连接中',
  1: '未初始化',
  2: '未初始化',
  3: '未初始化',
  4: '未初始化',
  5: '牵引停止',
  6: '牵引中',
  7: '正在结束',
  8: '牵引停止',
  9: '故障',
  10: '已急停'
};
const ACTION_SUCCESS_MESSAGES = {
  '/api/traction/prepare': '初始校准完成',
  '/api/traction/calibrate-direction': '方向已确定',
  '/api/traction/start': '开始牵引',
  '/api/traction/stop': '正在结束牵引',
  '/api/traction/emergency-stop': '已急停',
  '/api/traction/return-zero': '正在回零'
};
const REASON_LABELS = {
  AXIAL_TRAVEL_LIMIT: '达到行程上限',
  WRENCH_TIMEOUT: '力数据超时',
  EE_STATE_TIMEOUT: '位置数据超时',
  ROS2_CONTROL_ERROR: '运动控制异常',
  CALIBRATION_TOO_FEW_SAMPLES: '方向数据不足',
  LATERAL_FORCE_LIMIT: '横向力过大',
  LATERAL_FORCE: '横向力过大',
  UI_HEARTBEAT_TIMEOUT: '页面连接中断',
  NORMAL_RELEASE_COMPLETED: '已正常结束'
};

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

function simpleReason(value) {
  if (!value) return '';
  const text = String(value);
  return REASON_LABELS[text] || (/[一-鿿]/.test(text) ? text : '操作失败');
}

function simpleErrorMessage(error) {
  const text = String(error?.message || '');
  if (/target|1 N and 20|1～20/i.test(text)) return '目标牵引力需在1～20N';
  if (/direction|calibrat/i.test(text)) return '请先完成方向确认';
  if (/state|rejected|not available|unavailable|不可用/i.test(text)) return '当前状态不能执行';
  return /[一-鿿]/.test(text) ? text : '操作失败，请检查设备';
}

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
  const motionActive = [6, 7].includes(tractionState) || pendingStart;
  document.querySelectorAll('.force-adjust').forEach(button => {
    button.disabled = !permission.adjust || motionActive;
  });
  $('targetForceVal').disabled = !permission.adjust || motionActive;
  $('startBtn').disabled = !permission.operate || tractionState !== 5 || !dataOnline || pendingStart;
  $('stopBtn').disabled = !permission.operate || tractionState !== 6 || !dataOnline;
  if ($('prepareBtn')) $('prepareBtn').disabled = !permission.operate || !dataOnline || ![1, 2, 5, 8, 9, 10].includes(tractionState);
  if ($('calibrateBtn')) $('calibrateBtn').disabled = !permission.operate || !dataOnline || tractionState !== 2;
  if ($('emergencyBtn')) $('emergencyBtn').disabled = !permission.operate || !dataOnline || tractionState === 10;
  if ($('returnZeroBtn')) $('returnZeroBtn').disabled = !permission.operate || !dataOnline || motionActive || !slackZeroAvailable || ![2, 5, 8].includes(tractionState);
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
  $('targetForceVal').style.color = '#1e3a5f';
}

async function changeTarget(nextTarget) {
  if (!sessionUser || !permissions[sessionUser.role].adjust) {
    return toast('当前角色无权修改牵引参数');
  }
  if (activeRecord) return toast('请先结束当前牵引');
  const numericTarget = Number(nextTarget);
  if (!Number.isFinite(numericTarget) || numericTarget < TARGET_FORCE_MIN || numericTarget > TARGET_FORCE_MAX) {
    updateForceDisplay();
    return toast('目标牵引力请输入 1～20 N');
  }
  const previousForce = currentForce;
  currentForce = Math.round(numericTarget * 10) / 10;
  updateForceDisplay();
  try {
    await postJson('/api/traction/target', { target_force_n: currentForce });
    return true;
  } catch (error) {
    currentForce = previousForce;
    updateForceDisplay();
    toast(simpleErrorMessage(error));
    return false;
  }
}

async function startTraction() {
  if (!sessionUser || !permissions[sessionUser.role].operate) return;
  if (pendingStart) return toast('正在等待控制器接管');
  if (!dataOnline) return toast('设备未连接');
  const requestedTarget = Number($('targetForceVal').value);
  if (!Number.isFinite(requestedTarget) || requestedTarget < TARGET_FORCE_MIN || requestedTarget > TARGET_FORCE_MAX) {
    return toast('目标牵引力请输入 1～20 N');
  }
  if (tractionState !== 5) return toast('请先完成方向标定并锁定方向');
  // A new run must not inherit the previous run's completion request/status.
  finishRequested = false;
  pendingStart = true;
  $('workStatus').textContent = '正在开始';
  $('workStatus').classList.add('running');
  applyPermissions();
  try {
    // Always send the value currently shown in the input immediately before
    // each run. This is what makes the second and later runs independent of
    // the previous run's target.
    currentForce = Math.round(requestedTarget * 10) / 10;
    updateForceDisplay();
    await postJson('/api/traction/target', { target_force_n: currentForce });
    await postJson('/api/traction/start');
  } catch (error) {
    pendingStart = false;
    applyPermissions();
    return toast(simpleErrorMessage(error));
  }

  applyPermissions();
  toast('开始牵引');
}

function beginLocalRecord() {
  if (activeRecord || !pendingStart) return;
  pendingStart = false;
  activeRecord = { startedAt: Date.now() };
  $('workStatus').textContent = '牵引中';
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
  finishRequested = false;
  if (!activeRecord) return;
  clearInterval(timerHandle);
  activeRecord = null;
  $('recordTimer').textContent = '未开始记录';
  $('workStatus').textContent = status === '已完成' ? '已结束' : '已急停';
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
    return toast(simpleErrorMessage(error));
  }
  finishRequested = true;
  $('workStatus').textContent = '正在结束';
  $('workStatus').classList.add('running');
  applyPermissions();
  toast(status === '已完成' ? '正在结束牵引' : '正在停止');
}

async function emergencyStop() {
  try {
    await postJson('/api/traction/emergency-stop');
  } catch (error) {
    return toast(`${simpleErrorMessage(error)}；请使用实体急停`);
  }
  if (activeRecord) saveFinishedRecord('紧急终止');
  finishRequested = false;
  $('workStatus').textContent = '已急停';
  toast('已急停');
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
    ? list.map(record => `<tr><td>${record.id}</td><td>${record.start}</td><td>${record.end}</td><td>${record.operator}</td><td>${record.role}</td><td>${record.target}</td><td>${record.average}</td><td>${record.maximum}</td><td>${record.status}</td><td>${record.reason}</td></tr>`).join('')
    : '<tr><td colspan="10" class="empty">暂无符合条件的牵引记录</td></tr>';
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
    records = (history.summaries || [])
      .filter(summary => summary.stop_reason !== 'PREPARE_RESTARTED_AFTER_BASELINE_RESET')
      .map(summary => ({
        id: summary.session_id || '--',
        start: builtinTimeText(summary.start_time),
        end: builtinTimeText(summary.end_time),
        operator: '--',
        role: '系统',
        target: Number(summary.target_force_n || 0).toFixed(1),
        average: Number(summary.average_force_n || 0).toFixed(1),
        maximum: Number(summary.max_force_n || 0).toFixed(1),
        status: Number(summary.final_state) === 8 ? '已完成' : '异常终止',
        reason: simpleReason(summary.stop_reason) || '--'
      }));
    renderRecords();
  } catch (_) {
    // The page remains usable; the next poll retries without fabricating data.
  }
}

function exportRecords() {
  if (!records.length) return toast('暂无牵引记录');
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
  const maximum = 40;
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
  if ([2, 3, 4, 5, 6, 7, 8].includes(tractionState)) slackZeroAvailable = true;
  const rosTargetForce = Number(traction.target_force_n);
  if (!activeRecord && document.activeElement !== $('targetForceVal') &&
      Number.isFinite(rosTargetForce) &&
      rosTargetForce >= TARGET_FORCE_MIN && rosTargetForce <= TARGET_FORCE_MAX) {
    currentForce = Math.round(rosTargetForce * 10) / 10;
    updateForceDisplay();
  }
  if (tractionState === 6) beginLocalRecord();
  if (finishRequested && [5, 8].includes(tractionState) && activeRecord) {
    saveFinishedRecord('已完成');
  }
  if (activeRecord && [9, 10].includes(tractionState) && !pendingStart) {
    saveFinishedRecord('紧急终止');
  }
  if (pendingStart && [9, 10].includes(tractionState)) {
    pendingStart = false;
    toast(simpleReason(traction.fault_code || traction.stop_reason) || '开始失败');
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
  $('tensionState').textContent = tensionDetected ? '牵引带：紧' : '牵引带：松';
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
  if (activeRecord) {
    $('workStatus').textContent = tractionState === 7
      ? '正在结束'
      // The controller may still report the previous stop reason while a new
      // run is active. The live traction state always takes priority.
      : (tractionState === 6
        ? '牵引中'
        : (simpleReason(traction.fault_code || traction.stop_reason) || '牵引中'));
    $('workStatus').classList.add('running');
  } else if (!dataOnline) {
    $('workStatus').textContent = '设备未连接';
    $('workStatus').classList.remove('running');
  } else {
    $('workStatus').textContent = TRACTION_STATE_LABELS[tractionState] || '设备状态';
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
  $('settingsModal').classList.remove('hidden');
});
$('saveSettingsBtn').addEventListener('click', async () => {
  const target = Number($('settingTarget').value);
  if (target < TARGET_FORCE_MIN || target > TARGET_FORCE_MAX) {
    return toast('目标牵引力必须在 1～20 N');
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
      if (path === '/api/traction/prepare') slackZeroAvailable = true;
      applyPermissions();
    }
    toast(ACTION_SUCCESS_MESSAGES[path] || '操作完成');
    return result;
  } catch (error) {
    toast(simpleErrorMessage(error));
    return null;
  }
};
if ($('prepareBtn')) $('prepareBtn').addEventListener('click', () => callTraction('/api/traction/prepare'));
if ($('calibrateBtn')) {
  $('calibrateBtn').addEventListener('click', () => callTraction('/api/traction/calibrate-direction'));
}
if ($('returnZeroBtn')) $('returnZeroBtn').addEventListener('click', () => callTraction('/api/traction/return-zero'));
setInterval(() => { fetch('/api/traction/heartbeat', {method: 'POST', body: '{}'}).catch(() => {}); }, 500);
const savedSession = sessionStorage.getItem('tractionSession');
if (savedSession) {
  sessionUser = JSON.parse(savedSession);
  $('loginModal').classList.add('hidden');
  applyPermissions();
}

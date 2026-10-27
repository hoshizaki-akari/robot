const permissions = {
  '本地操作': { settings: true, adjust: true, operate: true, records: true, clear: true },
  '管理员': { settings: true, adjust: true, operate: true, records: true, clear: true },
  '医生': { settings: true, adjust: true, operate: true, records: true, clear: false },
  '操作员': { settings: false, adjust: true, operate: true, records: true, clear: false }
};

let sessionUser = { name: '本机操作员', role: '本地操作' };
const TARGET_FORCE_MIN = 1;
let tractionForceLimit = 100;
let currentForce = 10;
let confirmedForce = 10;
let targetUpdatePending = false;
let queuedTarget = null;
let targetUpdateTimer = null;
let operationMode = 2;
let actualForce = 0;
let activeRecord = null;
let timerHandle = null;
let records = [];
let stateSocket = null;
let reconnectHandle = null;
let lastStateAt = 0;
let dataOnline = false;
let calibrated = false;
let directionAvailable = false;
let directionLocked = false;
let tractionState = 0;
let pendingStart = false;
let finishRequested = false;
let emergencyPending = false;
let unloadPending = false;
const MODE_LABELS = { 0: '恒力牵引（历史）', 1: '目标牵引', 2: '省力拖拽', 3: '卸力' };
const ACTIVE_STATES = [3, 6, 7, 11, 12];
const IDLE_STATES = [1, 2, 5, 8];

const TRACTION_STATE_LABELS = {
  0: '连接中',
  1: '等待校准',
  2: '准备就绪',
  3: '正在获取方向',
  4: '正在校准',
  5: '牵引停止',
  6: '牵引中',
  7: '卸力中',
  8: '牵引停止',
  9: '故障',
  10: '已急停',
  11: '拖拽中',
  12: '已到达'
};
const ACTION_SUCCESS_MESSAGES = {
  '/api/traction/prepare': '重新校准完成',
  '/api/traction/start': '开始牵引',
  '/api/traction/stop': '正在结束任务',
  '/api/traction/unload': '开始卸力',
  '/api/traction/emergency-stop': '已急停',
  '/api/traction/emergency-recover': '急停已恢复'
};
const REASON_LABELS = {
  WRENCH_TIMEOUT: '力数据超时',
  EE_STATE_TIMEOUT: '位置数据超时',
  ROS2_CONTROL_ERROR: '运动控制异常',
  CARTESIAN_STOP_FAILED: '伺服停止未确认，请检查控制器后恢复',
  CALIBRATION_TOO_FEW_SAMPLES: '方向数据不足',
  LATERAL_FORCE_LIMIT: '横向力过大',
  LATERAL_FORCE: '横向力过大',
  UI_HEARTBEAT_TIMEOUT: '页面连接中断',
  NORMAL_RELEASE_COMPLETED: '已正常结束',
  CONSTANT_FORCE_STOPPED: '恒力牵引已结束',
  DRAG_COMPLETED: '拖拽已完成',
  POSITION_TRACTION_COMPLETED: '目标牵引已完成',
  POSITION_TRACTION_STOPPED: '目标牵引已结束',
  UNLOAD_COMPLETED: '卸力完成',
  UNLOAD_BELOW_5N: '卸力完成',
  UNLOAD_TIMEOUT: '卸力超时已停止',
  TARGET_TRACTION_STOPPED: '目标牵引已结束',
  SWITCHED_TO_UNLOADING: '已切换卸力',
  DIRECTION_SEARCH_NO_FORCE: '未检测到有效受力，请调整绳带后重试',
  DIRECTION_SEARCH_CONTACT_UNSTABLE: '搜索受力未保持稳定，已停止；请张紧绳带后重新开始',
  DIRECTION_SEARCH_CANCELLED: '已取消方向获取',
  DIRECTION_SEARCH_FEEDBACK_INTERRUPTED: '方向获取因反馈中断而停止',
  UNLOAD_STOPPED: '卸力已结束',
  DIRECTION_SEARCH_EXHAUSTED: '未检测到有效受力，请调整绳带后重试'
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
  if (/target|目标牵引力|1 N and 100|1～100/i.test(text)) return `目标牵引力需在1～${tractionForceLimit}N`;
  if (/mode|模式/i.test(text)) return '当前状态不能切换模式';
  if (/direction/i.test(text)) return '暂无有效方向，请选择目标牵引开始获取方向';
  if (/calibrat/i.test(text)) return '自动校准尚未完成，请检查设备状态';
  if (/unload|release/i.test(text)) return '卸力需已有有效方向且当前受力至少5N';
  if (/state|rejected|not available|unavailable|不可用/i.test(text)) return '当前状态不能执行';
  return /[一-鿿]/.test(text) ? text : '操作失败，请检查设备';
}

function recoveryErrorMessage(error) {
  const text = String(error?.message || '');
  if (/牵引服务超时|timeout/i.test(text)) return '急停恢复等待设备超时，请检查连接后重试';
  if (/emergency stop input/i.test(text)) return '请先释放实体急停，再点击急停恢复';
  if (/Cartesian servo session|ServoMoveEnd/i.test(text)) return '机械臂运动状态尚未清理，请检查设备';
  if (/force sensor/i.test(text)) return '力传感器未恢复，请检查设备';
  if (/did not enter manual|Mode\(1\)/i.test(text)) return '机械臂未切换到手动模式，请检查示教器';
  if (/robot enable|manual mode could not be enabled/i.test(text)) return '机械臂未成功上使能，请检查设备';
  return /[一-鿿]/.test(text) ? text : '急停恢复失败，请检查设备状态';
}

async function postJson(path, body = {}) {
  const headers = { 'Content-Type': 'application/json' };
  if (sessionUser) {
    headers['X-Operator'] = encodeURIComponent(sessionUser.name);
    headers['X-Role'] = encodeURIComponent(sessionUser.role);
  }
  headers['X-Operation-Detail'] = encodeURIComponent(JSON.stringify(body));
  const response = await fetch(path, {
    method: 'POST',
    headers,
    body: JSON.stringify(body)
  });
  const data = await response.json();
  if (!response.ok) throw new Error(data.detail || '操作失败');
  return data;
}

function motionStateActive() {
  return ACTIVE_STATES.includes(tractionState) || (tractionState === 5 && directionLocked);
}

function canUnload() {
  return calibrated && directionAvailable && actualForce >= 5 &&
    [1, 2, 5, 6, 8, 11, 12].includes(tractionState) && !pendingStart && !unloadPending;
}

function applyPermissions() {
  if (!sessionUser) return;
  const permission = permissions[sessionUser.role];
  $('currentUser').textContent = sessionUser.name;
  $('currentRole').textContent = sessionUser.role;
  const motionActive = motionStateActive() || pendingStart || unloadPending;
  const liveTractionAdjustment = operationMode === 1 && [6, 12].includes(tractionState);
  $('settingsBtn').disabled = !permission.settings || !dataOnline || motionActive;
  $('targetForceVal').disabled = !permission.adjust || operationMode !== 1 ||
    (motionActive && !liveTractionAdjustment);
  const startStateReady = calibrated && !directionLocked && IDLE_STATES.includes(tractionState) &&
    (operationMode !== 3 || canUnload());
  $('startBtn').disabled = !permission.operate || !startStateReady || !dataOnline || pendingStart;
  $('stopBtn').disabled = !permission.operate || !motionStateActive() || !dataOnline;
  $('prepareBtn').disabled = !permission.operate || !dataOnline || motionActive ||
    !IDLE_STATES.includes(tractionState);
  $('unloadBtn').disabled = !permission.operate || !dataOnline || !canUnload();
  const showStopAction = motionStateActive() || pendingStart || unloadPending;
  $('startBtn').classList.toggle('slot-hidden', showStopAction);
  $('stopBtn').classList.toggle('slot-hidden', !showStopAction);
  const recovering = tractionState === 10 || tractionState === 9;
  $('emergencyBtn').disabled = !permission.operate || !dataOnline || emergencyPending;
  $('emergencyLabel').textContent = tractionState === 9 ? '故障恢复' : (recovering ? '急停恢复' : '急停');
  $('emergencyBtn').classList.toggle('recover', recovering);
  document.querySelectorAll('.mode-btn').forEach(button => {
    button.disabled = !permission.operate || !dataOnline || motionActive ||
      !IDLE_STATES.includes(tractionState);
    button.classList.toggle('active', Number(button.dataset.mode) === operationMode);
  });
  $('startLabel').textContent = operationMode === 2 ? '开始拖拽' :
    (operationMode === 3 ? '开始卸力' : '开始牵引');
  $('stopLabel').textContent = tractionState === 7 ? '停止卸力' :
    (operationMode === 2 ? '结束拖拽' : '结束牵引');
  $('recordsBtn').disabled = !permission.records;
}

async function shutdownProgram() {
  const confirmButton = $('confirmShutdownBtn');
  confirmButton.disabled = true;
  confirmButton.textContent = '正在关闭…';
  try {
    const response = await fetch('/api/system/shutdown', { method: 'POST' });
    if (!response.ok) throw new Error('关闭请求失败');
    $('shutdownPrompt').textContent = '正在停止机械臂控制服务并关闭程序，请稍候。';
  } catch (error) {
    confirmButton.disabled = false;
    confirmButton.textContent = '确认关闭';
    $('shutdownModal').classList.add('hidden');
    toast(simpleErrorMessage(error));
  }
}

function updateForceDisplay() {
  currentForce = Math.round(currentForce);
  $('targetForceVal').min = TARGET_FORCE_MIN;
  $('targetForceVal').max = tractionForceLimit;
  $('targetForceVal').value = currentForce;
  $('targetForceDisplay').textContent = String(currentForce);
  $('sliderMaximum').textContent = String(tractionForceLimit);
  const span = Math.max(1, tractionForceLimit - TARGET_FORCE_MIN);
  const progress = Math.max(0, Math.min(100,
    ((currentForce - TARGET_FORCE_MIN) / span) * 100));
  $('targetForceVal').style.setProperty('--slider-progress', `${progress}%`);
}

async function changeTarget(nextTarget) {
  if (!sessionUser || !permissions[sessionUser.role].adjust) {
    return toast('当前角色无权修改牵引参数');
  }
  const liveUpdateAllowed = activeRecord && operationMode === 1 && [6, 12].includes(tractionState);
  if (activeRecord && !liveUpdateAllowed) return toast('当前模式运行中不能修改目标');
  const numericTarget = Number(nextTarget);
  if (!Number.isFinite(numericTarget) || numericTarget < TARGET_FORCE_MIN || numericTarget > tractionForceLimit) {
    updateForceDisplay();
    return toast(`目标牵引力请输入 1～${tractionForceLimit} N`);
  }
  currentForce = Math.round(numericTarget);
  updateForceDisplay();
  targetUpdatePending = true;
  applyPermissions();
  try {
    await postJson('/api/traction/target', { target_force_n: currentForce });
    confirmedForce = currentForce;
    targetUpdatePending = false;
    applyPermissions();
    return true;
  } catch (error) {
    targetUpdatePending = false;
    if (queuedTarget === null) currentForce = confirmedForce;
    updateForceDisplay();
    applyPermissions();
    toast(simpleErrorMessage(error));
    return false;
  } finally {
    if (queuedTarget !== null) flushQueuedTarget();
  }
}

async function flushQueuedTarget() {
  if (targetUpdatePending || queuedTarget === null) return;
  const target = queuedTarget;
  queuedTarget = null;
  await changeTarget(target);
}

function queueSliderTarget(rawTarget, immediate = false) {
  const target = Math.max(TARGET_FORCE_MIN,
    Math.min(tractionForceLimit, Math.round(Number(rawTarget))));
  currentForce = target;
  queuedTarget = target;
  updateForceDisplay();
  if (immediate) {
    clearTimeout(targetUpdateTimer);
    targetUpdateTimer = setTimeout(() => {
      targetUpdateTimer = null;
      flushQueuedTarget();
    }, 0);
  } else if (targetUpdateTimer === null) {
    targetUpdateTimer = setTimeout(() => {
      targetUpdateTimer = null;
      flushQueuedTarget();
    }, 50);
  }
}

function applyForceLimit(limit) {
  const numericLimit = Number(limit);
  tractionForceLimit = Number.isFinite(numericLimit) && numericLimit >= TARGET_FORCE_MIN
    ? Math.floor(numericLimit) : 100;
  if (currentForce > tractionForceLimit) currentForce = tractionForceLimit;
  updateForceDisplay();
}

async function loadSettings() {
  try {
    const response = await fetch('/api/settings', { cache: 'no-store' });
    if (!response.ok) return;
    const settings = await response.json();
    applyForceLimit(settings.traction_force_limit_n);
  } catch (_) {
    applyForceLimit(100);
  }
}

async function startTraction() {
  if (!sessionUser || !permissions[sessionUser.role].operate) return;
  if (pendingStart) return toast('正在等待控制器接管');
  if (!dataOnline) return toast('设备未连接');
  if (!calibrated || !IDLE_STATES.includes(tractionState)) {
    return toast('设备尚未就绪，请等待自动校准或检查设备状态');
  }
  if (operationMode === 3) return unloadTraction();
  if (![1, 2].includes(operationMode)) return toast('请选择当前支持的牵引模式');
  const requestedTarget = Number($('targetForceVal').value);
  if (operationMode === 1 && (!Number.isFinite(requestedTarget) ||
      requestedTarget < TARGET_FORCE_MIN || requestedTarget > tractionForceLimit)) {
    return toast(`目标牵引力请输入 1～${tractionForceLimit} N`);
  }
  // The manager locks an available direction, or searches along tool -Z.
  // Direction discovery never requires a separate browser confirmation.
  finishRequested = false;
  pendingStart = true;
  $('workStatus').textContent = '正在开始';
  $('workStatus').classList.add('running');
  applyPermissions();
  try {
    if (operationMode === 1) {
      // Always send the value currently shown before each new run.
      currentForce = Math.round(requestedTarget);
      updateForceDisplay();
      await postJson('/api/traction/target', { target_force_n: currentForce });
    }
    await postJson('/api/traction/start');
  } catch (error) {
    pendingStart = false;
    applyPermissions();
    return toast(simpleErrorMessage(error));
  }
  applyPermissions();
  toast(operationMode === 2 ? '开始拖拽' : '开始牵引；无有效方向时将自动获取');
}

async function unloadTraction() {
  if (!sessionUser || !permissions[sessionUser.role].operate || !dataOnline) return;
  if (!canUnload()) return toast('卸力需已有有效方向且当前受力至少5N');
  unloadPending = true;
  finishRequested = false;
  applyPermissions();
  try {
    await postJson('/api/traction/unload');
    toast('开始卸力，低于5N自动停止');
  } catch (error) {
    toast(simpleErrorMessage(error));
  } finally {
    unloadPending = false;
    applyPermissions();
  }
}

function beginLocalRecord() {
  if (activeRecord) return;
  pendingStart = false;
  activeRecord = { startedAt: Date.now() };
  $('workStatus').textContent = tractionState === 3 ? '正在获取方向' :
    (tractionState === 7 ? '卸力中' : (operationMode === 2 ? '拖拽中' : '牵引中'));
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
  if (!motionStateActive()) return toast('当前没有正在执行的任务');
  try {
    await postJson('/api/traction/stop');
  } catch (error) {
    return toast(simpleErrorMessage(error));
  }
  finishRequested = true;
  $('workStatus').textContent = '正在结束';
  $('workStatus').classList.add('running');
  applyPermissions();
  toast(status === '已完成' ? (operationMode === 2 ? '结束拖拽' : '正在结束牵引') : '正在停止');
}

async function emergencyStop() {
  const recovering = tractionState === 10 || tractionState === 9;
  emergencyPending = true;
  applyPermissions();
  try {
    await postJson(recovering
      ? '/api/traction/emergency-recover'
      : '/api/traction/emergency-stop');
  } catch (error) {
    emergencyPending = false;
    applyPermissions();
    return toast(recovering
      ? recoveryErrorMessage(error)
      : `${simpleErrorMessage(error)}；请使用实体急停`);
  }
  emergencyPending = false;
  if (recovering) {
    finishRequested = false;
    $('workStatus').textContent = '设备恢复中';
    applyPermissions();
    return toast('急停已恢复');
  }
  if (activeRecord) saveFinishedRecord('紧急终止');
  finishRequested = false;
  $('workStatus').textContent = '已急停';
  applyPermissions();
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
    ? list.map(record => `<tr><td>${record.id}</td><td>${record.mode}</td><td>${record.start}</td><td>${record.end}</td><td>${record.operator}</td><td>${record.role}</td><td>${record.target}</td><td>${record.average}</td><td>${record.maximum}</td><td>${record.status}</td><td>${record.reason}</td><td><button class="secondary-btn row-export-btn" data-export-session="${record.id}">导出日志</button></td></tr>`).join('')
    : '<tr><td colspan="12" class="empty">暂无符合条件的牵引记录</td></tr>';
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
        mode: MODE_LABELS[Number(summary.operation_mode ?? 0)] || '未知模式',
        start: builtinTimeText(summary.start_time),
        end: builtinTimeText(summary.end_time),
        operator: summary.operator || '--',
        role: summary.role || '系统',
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

function exportSession(sessionId) {
  if (!sessionId || !sessionId.startsWith('session_')) return toast('记录编号无效');
  const link = document.createElement('a');
  link.href = `/api/traction/export/session/${encodeURIComponent(sessionId)}`;
  link.download = `${sessionId}_完整日志.zip`;
  link.click();
}

const canvas = $('forceCanvas');
const context = canvas.getContext('2d');
let forceHistory = [];
const FORCE_HISTORY_WINDOW_MS = 60000;

function resizeCanvas() {
  const rectangle = canvas.getBoundingClientRect();
  canvas.width = Math.max(100, rectangle.width);
  canvas.height = Math.max(100, rectangle.height);
  drawCurve();
}

function drawCurve() {
  const width = canvas.width;
  const height = canvas.height;
  const plot = { left: 36, top: 12, right: width - 10, bottom: height - 24 };
  const plotWidth = Math.max(1, plot.right - plot.left);
  const plotHeight = Math.max(1, plot.bottom - plot.top);
  context.clearRect(0, 0, width, height);
  context.fillStyle = '#f8fafc';
  context.fillRect(0, 0, width, height);
  context.strokeStyle = '#e2e8f0';
  context.lineWidth = 1;
  for (let index = 0; index <= 4; index += 1) {
    const y = plot.top + plotHeight * index / 4;
    context.beginPath();
    context.moveTo(plot.left, y);
    context.lineTo(plot.right, y);
    context.stroke();
  }
  for (let index = 0; index <= 6; index += 1) {
    const x = plot.left + plotWidth * index / 6;
    context.beginPath();
    context.moveTo(x, plot.top);
    context.lineTo(x, plot.bottom);
    context.stroke();
  }

  let peak = Math.max(currentForce, actualForce, 0);
  forceHistory.forEach(point => {
    peak = Math.max(peak, point.actual, point.target);
  });
  const scaleSteps = [20, 40, 60, 80, 100, 120, 150];
  const desiredMaximum = peak * 1.1;
  const maximum = scaleSteps.find(value => value >= desiredMaximum) ||
    Math.ceil(desiredMaximum / 50) * 50;
  const now = Date.now();
  const start = now - FORCE_HISTORY_WINDOW_MS;
  const xFor = timestamp => plot.left +
    Math.max(0, Math.min(1, (timestamp - start) / FORCE_HISTORY_WINDOW_MS)) * plotWidth;
  const yFor = value => plot.bottom -
    Math.max(0, Math.min(1, value / maximum)) * plotHeight;

  if (forceHistory.length) {
    context.save();
    context.beginPath();
    context.rect(plot.left, plot.top, plotWidth, plotHeight);
    context.clip();

    context.strokeStyle = '#16a34a';
    context.lineWidth = 2.5;
    context.setLineDash([9, 7]);
    context.beginPath();
    forceHistory.forEach((point, index) => {
      const x = xFor(point.time);
      const y = yFor(point.target);
      if (!index) {
        context.moveTo(x, y);
      } else {
        const previousY = yFor(forceHistory[index - 1].target);
        context.lineTo(x, previousY);
        context.lineTo(x, y);
      }
    });
    context.stroke();

    context.strokeStyle = '#2680eb';
    context.lineWidth = 3;
    context.setLineDash([]);
    context.beginPath();
    forceHistory.forEach((point, index) => {
      const x = xFor(point.time);
      const y = yFor(point.actual);
      index ? context.lineTo(x, y) : context.moveTo(x, y);
    });
    context.stroke();
    context.restore();
  }

  context.fillStyle = '#64748b';
  context.font = '12px "Microsoft YaHei", sans-serif';
  context.textAlign = 'right';
  context.fillText(String(maximum), plot.left - 6, plot.top + 4);
  context.fillText(String(Math.round(maximum / 2)), plot.left - 6, plot.top + plotHeight / 2 + 4);
  context.fillText('0', plot.left - 6, plot.bottom + 4);
  context.textAlign = 'left';
  context.fillText('−60秒', plot.left, height - 6);
  context.textAlign = 'right';
  context.fillText('现在', plot.right, height - 6);
}

function appendForceSample(actual, target) {
  const now = Date.now();
  forceHistory.push({ time: now, actual: Number(actual) || 0, target: Number(target) || 0 });
  const cutoff = now - FORCE_HISTORY_WINDOW_MS;
  while (forceHistory.length && forceHistory[0].time < cutoff) forceHistory.shift();
}

function handleState(state) {
  lastStateAt = Date.now();
  const traction = state.traction || {};
  tractionState = Number(traction.state || 0);
  operationMode = Number(traction.operation_mode ?? 2);
  calibrated = traction.calibrated === true;
  directionAvailable = traction.direction_available === true;
  directionLocked = traction.direction_locked === true;
  const rosTargetForce = Number(traction.target_force_n);
  if (!targetUpdatePending && queuedTarget === null && document.activeElement !== $('targetForceVal') &&
      Number.isFinite(rosTargetForce) &&
      rosTargetForce >= TARGET_FORCE_MIN && rosTargetForce <= tractionForceLimit) {
    currentForce = Math.round(rosTargetForce);
    confirmedForce = currentForce;
    updateForceDisplay();
  }
  if (motionStateActive()) beginLocalRecord();
  if (IDLE_STATES.includes(tractionState) && !directionLocked && activeRecord) {
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

  const lockedDirection = Array.isArray(traction.locked_direction_base)
    ? traction.locked_direction_base : null;
  if (window.updateTractionDirection) {
    // The displayed arrow follows the manager's two-second direction sample,
    // then remains fixed for the complete run, including the hold phase.
    window.updateTractionDirection(directionAvailable ? lockedDirection : null);
  }
  actualForce = Number(traction.actual_force_n || 0);
  $('actualForceVal').textContent = actualForce.toFixed(1);
  const tensionDetected = actualForce >= 1.0;
  $('tensionState').textContent = tensionDetected ? '张紧' : '松弛';
  $('tensionState').classList.toggle('tight', tensionDetected);
  $('directionState').textContent = tractionState === 3 ? '正在获取方向' :
    (directionLocked ? '方向已锁定' : (directionAvailable ? '方向已更新' : '等待施力'));
  $('directionState').classList.toggle('following', tractionState === 3);
  $('prepareBtn').title = calibrated
    ? '已校准；仅在牵引带松弛且机械臂静止时重新校准'
    : '等待后台完成自动校准；牵引带应保持松弛';

  if (activeRecord) {
    $('workStatus').textContent = tractionState === 7
      ? '卸力中'
      // The controller may still report the previous stop reason while a new
      // run is active. The live traction state always takes priority.
      : (ACTIVE_STATES.includes(tractionState)
        ? TRACTION_STATE_LABELS[tractionState]
        : (simpleReason(traction.fault_code || traction.stop_reason) || '牵引中'));
    $('workStatus').classList.add('running');
  } else if (!dataOnline) {
    $('workStatus').textContent = '设备未连接';
    $('workStatus').classList.remove('running');
  } else {
    $('workStatus').textContent = TRACTION_STATE_LABELS[tractionState] || '设备状态';
    $('workStatus').classList.remove('running');
  }

  appendForceSample(actualForce, Number.isFinite(rosTargetForce) ? rosTargetForce : currentForce);
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

$('targetForceVal').addEventListener('input', event => queueSliderTarget(event.target.value));
$('targetForceVal').addEventListener('change', event => queueSliderTarget(event.target.value, true));
$('shutdownBtn').addEventListener('click', () => {
  $('shutdownPrompt').textContent = '确认关闭工作站程序和机器人控制服务吗？';
  $('confirmShutdownBtn').disabled = false;
  $('confirmShutdownBtn').textContent = '确认关闭';
  $('shutdownModal').classList.remove('hidden');
});
$('cancelShutdownBtn').addEventListener('click', () => {
  $('shutdownModal').classList.add('hidden');
});
$('confirmShutdownBtn').addEventListener('click', shutdownProgram);
$('startBtn').addEventListener('click', startTraction);
$('stopBtn').addEventListener('click', () => finishTraction('已完成'));
$('emergencyBtn').addEventListener('click', emergencyStop);
$('unloadBtn').addEventListener('click', unloadTraction);
$('recordsBtn').addEventListener('click', () => {
  renderRecords();
  $('recordsModal').classList.remove('hidden');
});
$('settingsBtn').addEventListener('click', () => {
  if (!sessionUser || !permissions[sessionUser.role].settings) {
    return toast('当前角色无权修改参数');
  }
  $('settingTarget').value = currentForce;
  $('settingTarget').max = tractionForceLimit;
  $('settingForceLimit').value = tractionForceLimit;
  $('settingsModal').classList.remove('hidden');
});
$('saveSettingsBtn').addEventListener('click', async () => {
  const target = Number($('settingTarget').value);
  const limit = Number($('settingForceLimit').value);
  if (!Number.isSafeInteger(limit) || limit < TARGET_FORCE_MIN) {
    return toast('目标力调节上限必须是大于等于1N的整数');
  }
  if (!Number.isInteger(target) || target < TARGET_FORCE_MIN || target > limit) {
    return toast(`默认目标牵引力必须是1～${limit}N的整数`);
  }
  try {
    await postJson('/api/settings', { traction_force_limit_n: limit });
  } catch (error) {
    return toast(simpleErrorMessage(error));
  }
  applyForceLimit(limit);
  if (!(await changeTarget(target))) {
    return;
  }
  $('settingsModal').classList.add('hidden');
  toast('参数已保存');
});
$('recordSearch').addEventListener('input', renderRecords);
$('recordStatusFilter').addEventListener('change', renderRecords);
$('exportBtn').addEventListener('click', exportRecords);
$('recordsBody').addEventListener('click', event => {
  const button = event.target.closest('[data-export-session]');
  if (button) exportSession(button.dataset.exportSession);
});
document.querySelectorAll('[data-close]').forEach(button => {
  button.addEventListener('click', () => $(button.dataset.close).classList.add('hidden'));
});
window.addEventListener('resize', resizeCanvas);

updateForceDisplay();
loadSettings();
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
      handleState(result.snapshot);
    }
    toast(ACTION_SUCCESS_MESSAGES[path] || '操作完成');
    return result;
  } catch (error) {
    toast(simpleErrorMessage(error));
    return null;
  }
};
if ($('prepareBtn')) $('prepareBtn').addEventListener('click', () => callTraction('/api/traction/prepare'));
document.querySelectorAll('.mode-btn').forEach(button => {
  button.addEventListener('click', async () => {
    const requestedMode = Number(button.dataset.mode);
    try {
      await postJson('/api/traction/mode', { mode: requestedMode });
      operationMode = requestedMode;
      pendingStart = false;
      finishRequested = false;
      applyPermissions();
      toast(`已切换为${MODE_LABELS[operationMode]}`);
    } catch (error) {
      toast(simpleErrorMessage(error));
    }
  });
});
setInterval(() => { fetch('/api/traction/heartbeat', {method: 'POST', body: '{}'}).catch(() => {}); }, 500);
applyPermissions();

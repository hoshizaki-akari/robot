"use strict";

const $ = (id) => document.getElementById(id);
const history = [];
let latest = null;
let toastTimer = null;
let drawQueued = false;

const phasePresentation = {
  disconnected: ["数据断开", "×"], waiting_data: ["等待数据", "…"],
  need_baseline: ["等待基线", "○"], baseline_capturing: ["采集基线", "◎"],
  moving: ["机械臂移动中", "↔"], settling: ["正在停稳", "≈"],
  unstable: ["受力波动", "∿"], confirming: ["正在确认", "…"],
  orientation_changed: ["法兰朝向变化", "⟳"],
  slack: ["绳带松动", "✓"], transition: ["松紧过渡", "◐"],
  tension: ["绳带已张紧", "↗"],
};

function fmt(value, digits = 2) {
  return Number.isFinite(Number(value)) ? Number(value).toFixed(digits) : "--";
}

function vectorText(vector) {
  return Array.isArray(vector) ? `[${vector.map((v) => fmt(v, 3)).join(", ")}]` : "[--, --, --]";
}

function showToast(message, error = false) {
  const toast = $("toast");
  toast.textContent = message;
  toast.className = `toast show${error ? " error" : ""}`;
  clearTimeout(toastTimer);
  toastTimer = setTimeout(() => { toast.className = "toast"; }, 3600);
}

async function apiAction(url, options = {}) {
  const response = await fetch(url, { ...options, headers: { "Content-Type": "application/json" } });
  const body = await response.json().catch(() => ({}));
  if (!response.ok) throw new Error(body.detail || `请求失败 (${response.status})`);
  if (body.state) update(body.state, false);
  return body;
}

function update(state, appendHistory = true) {
  latest = state;
  const connected = Boolean(state.connected);
  $("connectionBadge").classList.toggle("online", connected);
  $("connectionText").textContent = connected ? "真实传感器已连接" : "传感器未连接";

  const phase = state.warning ? "warning-state" : state.phase;
  const [title, icon] = phasePresentation[state.phase] || [state.phase_text || "未知状态", "?"];
  $("statePanel").className = `state-panel panel ${phase}`;
  $("stateTitle").textContent = state.warning ? "牵引力超过 10 N" : title;
  $("stateIcon").textContent = state.warning ? "!" : icon;
  $("stateExplanation").textContent = state.warning ? state.warning_text : state.phase_text;
  $("forceValue").textContent = fmt(state.resultant_force_n);
  $("forceMeter").style.width = `${Math.min(100, Math.max(0, Number(state.resultant_force_n || 0) / 10 * 100))}%`;

  const force = state.relative_force_n || [0, 0, 0];
  ["fxValue", "fyValue", "fzValue"].forEach((id, i) => { $(id).textContent = fmt(force[i]); });
  $("forceAxis").textContent = state.actual_force_axis || "--";
  $("forceDirection").textContent = vectorText(state.actual_force_direction);
  $("increaseAxis").textContent = state.increase_motion_axis || "--";
  $("increaseDirection").textContent = vectorText(state.increase_motion_direction);

  const movingText = !state.motion_available
    ? "不可用（只按受力稳定性判定）"
    : state.moving
      ? `正在移动（最大关节速度 ${fmt(state.max_joint_speed_deg_s, 3)}°/s）`
      : `静止（最大关节速度 ${fmt(state.max_joint_speed_deg_s, 3)}°/s）`;
  $("motionValue").textContent = movingText;
  $("orientationValue").textContent = !state.orientation_available
    ? "当前数据源不提供姿态（请人工保持不变）"
    : !state.baseline_ready
      ? "等待设置基线"
      : `相对基线变化 ${fmt(state.orientation_change_deg, 3)}° / 上限 ${fmt(state.thresholds?.max_orientation_change_deg, 1)}°`;
  $("stabilityTag").textContent = state.moving ? "移动期间暂缓结论" : (state.phase_text || "--");
  $("sourceValue").textContent = state.source_detail || state.source || "等待发现";
  $("frameValue").textContent = state.frame_id || "--";
  $("rateValue").textContent = `${fmt(state.sample_rate_hz, 1)} Hz / ${state.data_age_ms ?? "--"} ms`;
  $("baselineValue").textContent = state.baseline_ready
    ? `已设置 ${vectorText(state.baseline_force_n)} N`
    : state.baseline_message;
  const limits = state.thresholds || {};
  $("thresholdValue").textContent = `≤${fmt(limits.slack_n, 1)} N 松动 · ≥${fmt(limits.tension_n, 1)} N 张紧 · >${fmt(limits.warning_n, 0)} N 警戒`;

  $("warningBanner").classList.toggle("hidden", !state.warning);
  $("warningText").textContent = state.warning_text || "";
  $("baselineButton").disabled = !connected || state.moving || state.baseline_capturing;
  $("clearBaselineButton").disabled = !state.baseline_ready && !state.baseline_capturing;
  $("reverseButton").disabled = !state.baseline_ready;
  $("baselineProgressWrap").classList.toggle("hidden", !state.baseline_capturing);
  $("baselineProgress").style.width = `${Math.round((state.baseline_progress || 0) * 100)}%`;
  $("baselineProgressText").textContent = state.phase_text || "正在采集基线";

  if (connected) {
    $("diagnosticText").textContent = `已通过 ${state.source_detail || state.source} 接收真实 KWR75D 数据；程序未发送运动或校零命令。${state.last_error ? ` 最近错误：${state.last_error}` : ""}`;
  } else {
    const diagnosticParts = [];
    Object.entries(state.diagnostics || {}).forEach(([key, value]) => diagnosticParts.push(`${key}: ${value}`));
    if (state.last_error) diagnosticParts.unshift(`错误: ${state.last_error}`);
    $("diagnosticText").textContent = diagnosticParts.join("　|　") || "程序正在自动寻找真实数据源。";
  }

  if (appendHistory && connected && state.baseline_ready) {
    const now = performance.now() / 1000;
    history.push({ t: now, value: Number(state.resultant_force_n || 0) });
    while (history.length && history[0].t < now - 30) history.shift();
  }
  queueDraw();
}

function canvasContext(canvas) {
  const ratio = window.devicePixelRatio || 1;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(1, Math.round(rect.width * ratio));
  const height = Math.max(1, Math.round(rect.height * ratio));
  if (canvas.width !== width || canvas.height !== height) {
    canvas.width = width; canvas.height = height;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(ratio, 0, 0, ratio, 0, 0);
  return { ctx, width: rect.width, height: rect.height };
}

function arrow(ctx, from, to, color, width = 2) {
  const angle = Math.atan2(to.y - from.y, to.x - from.x);
  ctx.strokeStyle = color; ctx.fillStyle = color; ctx.lineWidth = width;
  ctx.beginPath(); ctx.moveTo(from.x, from.y); ctx.lineTo(to.x, to.y); ctx.stroke();
  ctx.beginPath(); ctx.moveTo(to.x, to.y);
  ctx.lineTo(to.x - 9 * Math.cos(angle - .45), to.y - 9 * Math.sin(angle - .45));
  ctx.lineTo(to.x - 9 * Math.cos(angle + .45), to.y - 9 * Math.sin(angle + .45));
  ctx.closePath(); ctx.fill();
}

function project(vector, length, origin) {
  const [x, y, z] = vector;
  return { x: origin.x + length * (.84 * x - .56 * y), y: origin.y + length * (.34 * x + .38 * y - .92 * z) };
}

function drawVectors() {
  const { ctx, width, height } = canvasContext($("vectorCanvas"));
  ctx.clearRect(0, 0, width, height);
  const origin = { x: width * .48, y: height * .6 };
  const axes = [
    [[1, 0, 0], "#ff6a75", "X+"], [[0, 1, 0], "#54e79f", "Y+"], [[0, 0, 1], "#53b7ff", "Z+"],
  ];
  ctx.font = "11px system-ui";
  axes.forEach(([vector, color, label]) => {
    const end = project(vector, Math.min(width, height) * .34, origin);
    arrow(ctx, origin, end, color, 1.3); ctx.fillStyle = color; ctx.fillText(label, end.x + 5, end.y);
  });
  ctx.fillStyle = "#668096"; ctx.beginPath(); ctx.arc(origin.x, origin.y, 3, 0, Math.PI * 2); ctx.fill();
  if (!latest) return;
  const vectors = [
    [latest.actual_force_direction, "#38d9ff", 4],
    [latest.increase_motion_direction, "#ffac5e", 3],
  ];
  vectors.forEach(([vector, color, lineWidth]) => {
    if (!Array.isArray(vector)) return;
    arrow(ctx, origin, project(vector, Math.min(width, height) * .42, origin), color, lineWidth);
  });
}

function drawChart() {
  const { ctx, width, height } = canvasContext($("forceChart"));
  ctx.clearRect(0, 0, width, height);
  const pad = { left: 38, right: 12, top: 10, bottom: 24 };
  const plotW = Math.max(1, width - pad.left - pad.right), plotH = Math.max(1, height - pad.top - pad.bottom);
  const values = history.map((p) => p.value);
  const yMax = Math.max(2, ...values.map((v) => v * 1.2));
  ctx.font = "10px system-ui"; ctx.lineWidth = 1;
  for (let i = 0; i <= 4; i += 1) {
    const y = pad.top + plotH * i / 4;
    ctx.strokeStyle = "rgba(142,173,195,.12)"; ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(width - pad.right, y); ctx.stroke();
    ctx.fillStyle = "#6e8799"; ctx.textAlign = "right"; ctx.fillText(`${fmt(yMax * (1 - i / 4), 1)}`, pad.left - 7, y + 3);
  }
  const thresholds = latest?.thresholds || { slack_n: .5, tension_n: 1, warning_n: 10 };
  [[thresholds.slack_n, "#39e59d"], [thresholds.tension_n, "#ffac5e"], [thresholds.warning_n, "#ff5f6d"]].forEach(([value, color]) => {
    if (value > yMax) return;
    const y = pad.top + plotH * (1 - value / yMax);
    ctx.setLineDash([4, 4]); ctx.strokeStyle = color; ctx.globalAlpha = .62;
    ctx.beginPath(); ctx.moveTo(pad.left, y); ctx.lineTo(width - pad.right, y); ctx.stroke();
    ctx.setLineDash([]); ctx.globalAlpha = 1;
  });
  ctx.fillStyle = "#6e8799"; ctx.textAlign = "center";
  [0, 10, 20, 30].forEach((seconds) => ctx.fillText(`${30 - seconds}s`, pad.left + plotW * seconds / 30, height - 6));
  if (history.length < 2) return;
  const endTime = history[history.length - 1].t, startTime = endTime - 30;
  ctx.beginPath();
  history.forEach((point, i) => {
    const x = pad.left + plotW * Math.max(0, (point.t - startTime) / 30);
    const y = pad.top + plotH * (1 - Math.min(1, point.value / yMax));
    if (i === 0) ctx.moveTo(x, y); else ctx.lineTo(x, y);
  });
  ctx.strokeStyle = "#38d9ff"; ctx.lineWidth = 2; ctx.shadowColor = "rgba(56,217,255,.5)"; ctx.shadowBlur = 7; ctx.stroke(); ctx.shadowBlur = 0;
}

function queueDraw() {
  if (drawQueued) return;
  drawQueued = true;
  requestAnimationFrame(() => { drawQueued = false; drawVectors(); drawChart(); });
}

async function loadInitial() {
  try {
    const response = await fetch("/api/state?history=true");
    const state = await response.json();
    const old = state.history || [];
    const now = performance.now() / 1000;
    old.slice(-600).forEach((point, i, array) => history.push({ t: now - (array.length - i) * .05, value: Number(point.force_n || 0) }));
    update(state, false);
  } catch (error) { showToast(`读取初始状态失败：${error.message}`, true); }
}

function connectWebSocket() {
  const protocol = location.protocol === "https:" ? "wss" : "ws";
  const socket = new WebSocket(`${protocol}://${location.host}/ws`);
  socket.onmessage = (event) => update(JSON.parse(event.data));
  socket.onclose = () => setTimeout(connectWebSocket, 1000);
  socket.onerror = () => socket.close();
}

$("baselineButton").addEventListener("click", async () => {
  try { const body = await apiAction("/api/baseline", { method: "POST" }); showToast(body.message); }
  catch (error) { showToast(error.message, true); }
});
$("clearBaselineButton").addEventListener("click", async () => {
  try { await apiAction("/api/baseline", { method: "DELETE" }); history.length = 0; showToast("松绳基线已清除"); }
  catch (error) { showToast(error.message, true); }
});
$("reverseButton").addEventListener("click", async () => {
  try { await apiAction("/api/direction/reverse", { method: "POST" }); showToast("增力运动方向已反转，请再次用已知轴向小力确认"); }
  catch (error) { showToast(error.message, true); }
});
window.addEventListener("resize", queueDraw);
loadInitial();
connectWebSocket();

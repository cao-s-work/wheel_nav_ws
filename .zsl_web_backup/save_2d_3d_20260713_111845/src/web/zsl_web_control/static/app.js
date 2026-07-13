"use strict";

const $ = (s) => document.querySelector(s);
const $$ = (s) => [...document.querySelectorAll(s)];
const tokenFromUrl = new URLSearchParams(location.search).get("token");
if (tokenFromUrl) localStorage.setItem("zsl_api_token", tokenFromUrl);
const API_TOKEN = localStorage.getItem("zsl_api_token") || "";

const app = {
  page: "dashboard",
  socket: null,
  socketOnline: false,
  reconnect: null,
  heartbeat: null,
  teleopLoop: null,
  poll: null,
  clientId: null,
  state: null,
  maps: [],
  manual: false,
  pointerActive: false,
  speed: 0.65,
  command: { vx: 0, vy: 0, wz: 0 },
  keys: new Set(),
  localEvents: [],
  live: {
    data: null,
    loading: false,
    poll: null,
    image: null,
    imageVersion: -1,
    requestedVersion: -1,
    imageReady: false,
    zoom: 1,
    panX: 0,
    panY: 0,
    follow: true,
    showScan: true,
    showPath: true,
    dragging: false,
    dragX: 0,
    dragY: 0,
    transform: null,
  },
};

const pageTitles = {
  dashboard: "机器人运行总览",
  manual: "人工控制与安全",
  mapping: "建图作业管理",
  navigation: "导航与重定位",
  maps: "场景地图库",
  diagnostics: "系统诊断中心",
};

function authHeaders() {
  const headers = { "Content-Type": "application/json" };
  if (API_TOKEN) headers.Authorization = `Bearer ${API_TOKEN}`;
  return headers;
}

async function api(method, path, body) {
  const controller = new AbortController();
  const timeout = setTimeout(() => controller.abort(), 25000);
  try {
    const response = await fetch(path, {
      method,
      headers: authHeaders(),
      body: body === undefined ? undefined : JSON.stringify(body),
      signal: controller.signal,
    });
    const data = await response.json().catch(() => ({ success: false, message: `HTTP ${response.status}` }));
    if (!response.ok && data.success !== false) data.success = false;
    return data;
  } catch (error) {
    return { success: false, message: error.name === "AbortError" ? "请求超时" : error.message };
  } finally {
    clearTimeout(timeout);
  }
}

function toast(title, message = "", level = "info") {
  const node = document.createElement("div");
  node.className = `toast ${level}`;
  node.innerHTML = "<b></b><span></span>";
  node.querySelector("b").textContent = title;
  node.querySelector("span").textContent = message;
  $("#toast-region").appendChild(node);
  setTimeout(() => node.remove(), 4200);
}

function localEvent(message, level = "info") {
  app.localEvents.push({ id: `local-${Date.now()}-${Math.random()}`, timestamp: Date.now() / 1000, message, level, source: "browser" });
  if (app.localEvents.length > 40) app.localEvents.shift();
  renderEvents();
}

function formatTime(seconds) {
  return new Date(seconds * 1000).toLocaleTimeString("zh-CN", { hour12: false });
}

function confirmAction(title, message) {
  const dialog = $("#confirm-dialog");
  $("#dialog-title").textContent = title;
  $("#dialog-message").textContent = message;
  dialog.showModal();
  return new Promise((resolve) => {
    dialog.addEventListener("close", () => resolve(dialog.returnValue === "default"), { once: true });
  });
}

function navigate(page) {
  app.page = page;
  $$(".page").forEach((node) => node.classList.toggle("active", node.id === `page-${page}`));
  $$(".nav-item").forEach((node) => node.classList.toggle("active", node.dataset.page === page));
  $("#page-title").textContent = pageTitles[page] || "ZSL-1W 控制台";
  if (page === "maps") refreshMaps();
  if (page === "diagnostics") refreshDiagnostics();
  if (page === "mapping") refreshLiveMapping();
}

function wsUrl() {
  const scheme = location.protocol === "https:" ? "wss" : "ws";
  const query = API_TOKEN ? `?token=${encodeURIComponent(API_TOKEN)}` : "";
  return `${scheme}://${location.host}/ws${query}`;
}

function connectSocket() {
  if (app.socket && [WebSocket.OPEN, WebSocket.CONNECTING].includes(app.socket.readyState)) return;
  const socket = new WebSocket(wsUrl());
  app.socket = socket;
  socket.addEventListener("open", () => {
    app.socketOnline = true;
    updateConnectionIndicators();
    clearInterval(app.heartbeat);
    app.heartbeat = setInterval(() => wsSend({ type: "heartbeat" }), 5000);
    clearInterval(app.teleopLoop);
    app.teleopLoop = setInterval(() => {
      if (app.manual && app.socketOnline) wsSend({ type: "teleop", ...app.command });
    }, 80);
    localEvent("WebSocket 已连接", "success");
  });
  socket.addEventListener("message", (event) => {
    try {
      const message = JSON.parse(event.data);
      if (message.type === "hello") {
        app.clientId = message.client_id;
        if (message.state) applyState(message.state);
      } else if (message.type === "state") {
        applyState(message.data);
      } else if (message.type === "control_mode_result") {
        toast(message.success ? "控制模式已切换" : "切换失败", message.message, message.success ? "success" : "error");
      } else if (message.type === "error") {
        toast("控制请求被拒绝", message.message, "error");
      }
    } catch (error) {
      console.warn("Invalid WS message", error);
    }
  });
  socket.addEventListener("close", () => {
    app.socketOnline = false;
    app.pointerActive = false;
    zeroCommand(true);
    updateConnectionIndicators();
    clearInterval(app.heartbeat);
    localEvent("WebSocket 已断开，机器人保持停车", "warning");
    clearTimeout(app.reconnect);
    app.reconnect = setTimeout(connectSocket, 1800);
  });
  socket.addEventListener("error", () => socket.close());
}

function wsSend(payload) {
  if (!app.socket || app.socket.readyState !== WebSocket.OPEN) return false;
  app.socket.send(JSON.stringify(payload));
  return true;
}

function updateConnectionIndicators() {
  $("#side-web").textContent = app.socketOnline ? "Web 已连接" : "Web 离线";
  $("#side-web-dot").classList.toggle("online", app.socketOnline);
}

function applyState(state) {
  app.state = state;
  const newManual = state?.control?.mode === "manual";
  const hasController = state?.control?.controller_present;
  app.manual = newManual;
  // 自动获取控制租约：手动模式但未持有租约时自动发起
  if (newManual && !hasController && app.socketOnline) {
    wsSend({ type: "control_mode", mode: "manual" });
  }
  renderState();
}

function sensor(nodeId, dotId, info) {
  $(nodeId).textContent = `${Number(info?.hz || 0).toFixed(1)} Hz`;
  $(dotId).classList.toggle("good", Boolean(info?.alive));
}

function ctrlModeName(mode) {
  return ({ 0: "阻尼 / 趴下", 1: "站立", 2: "姿态", 3: "运动", 18: "运动" })[mode] || `模式 ${mode ?? "--"}`;
}

function renderState() {
  const state = app.state || {};
  const robot = state.robot || {};
  const mapping = state.mapping || {};
  const nav = state.navigation || {};
  const maps = state.maps || {};
  const control = state.control || {};
  const sdk = Boolean(robot.connected);
  const battery = robot.battery_percent;

  $("#side-sdk").textContent = sdk ? "SDK 已连接" : "SDK 未连接";
  $("#side-sdk-dot").classList.toggle("online", sdk);
  $("#dash-sdk").textContent = sdk ? "在线" : "离线";
  $("#dash-watchdog").textContent = robot.cmd_watchdog_s >= 0 ? `指令 ${robot.cmd_watchdog_s.toFixed(2)} s` : "无指令";
  $("#dash-battery").textContent = battery == null ? "--%" : `${battery.toFixed(1)}%`;
  $("#dash-power-hint").textContent = battery == null ? "等待数据" : battery < 20 ? "建议尽快充电" : battery < 40 ? "电量偏低" : "电量正常";
  $("#dash-nav").textContent = mapping.navigation_active ? "已启动" : "未启动";
  $("#dash-nav-state").textContent = nav.message || "无任务";
  $("#dash-map").textContent = maps.active || "未选择";
  $("#dash-map-count").textContent = `${maps.count || 0} 张地图`;

  $("#ready-driver").textContent = sdk ? "正常" : "异常";
  $("#ready-lock").textContent = robot.read_only ? "已锁定" : "已解锁";
  $("#ready-estop").textContent = robot.estop_latched ? "已锁存" : "正常";
  const pose = robot.pose;
  $("#ready-pose").textContent = pose ? `${pose.source} / ${pose.frame_id}` : "无数据";
  const ready = sdk && !robot.estop_latched && mapping.lidar?.alive && mapping.odometry?.alive;
  const readyTag = $("#ready-tag");
  readyTag.textContent = ready ? "系统可用" : "存在异常";
  readyTag.className = `tag ${ready ? "good" : "warn"}`;

  sensor("#sensor-lidar", "#sensor-lidar-dot", mapping.lidar);
  sensor("#sensor-scan", "#sensor-scan-dot", mapping.scan);
  sensor("#sensor-odom", "#sensor-odom-dot", mapping.odometry);
  sensor("#sensor-map", "#sensor-map-dot", mapping.map_topic);
  $("#pose-x").textContent = pose ? pose.x.toFixed(2) : "--";
  $("#pose-y").textContent = pose ? pose.y.toFixed(2) : "--";
  $("#pose-yaw").textContent = pose ? `${pose.yaw_deg.toFixed(1)}°` : "--";

  $("#manual-mode").classList.toggle("active", app.manual);
  $("#auto-mode").classList.toggle("active", !app.manual);
  $("#top-hold").textContent = app.manual ? "人工保持中" : "人工保持";
  $("#manual-banner").innerHTML = app.manual
    ? `<b>${control.controller_present ? "人工控制权已获取" : "人工保持，无控制者"}</b><span>当前阻断 Nav2。遥控失联只会停车，不会自动恢复导航。</span>`
    : "<b>当前由 Nav2 控制</b><span>进入人工接管会取消当前导航任务；失联后保持停车。</span>";
  $("#manual-lock-state").textContent = robot.read_only ? "已锁定" : "已解锁";

  const mappingActive = Boolean(mapping.slam_active);
  const mappingJob = mapping.mapping_job || {};
  const mappingState = mappingJob.state || (mappingActive ? "running" : "idle");
  const mappingLabels = { idle: "未运行", starting: "启动中", running: "建图运行中", degraded: "部分就绪", failed: "启动失败" };
  const mappingClasses = { idle: "neutral", starting: "warn", running: "good", degraded: "warn", failed: "bad" };
  $("#mapping-tag").textContent = mappingLabels[mappingState] || mappingState;
  $("#mapping-tag").className = `tag ${mappingClasses[mappingState] || "neutral"}`;
  const jobNotice = $("#mapping-job-notice");
  jobNotice.className = `notice ${mappingState === "failed" ? "danger" : mappingState === "running" ? "success" : mappingState === "starting" || mappingState === "degraded" ? "warning" : "info"}`;
  jobNotice.querySelector("b").textContent = mappingLabels[mappingState] || "建图状态";
  $("#mapping-job-message").textContent = mappingJob.message || "点击“开始建图”后，系统将自动启动完整建图链路。";
  $("#mapping-start").disabled = mappingState === "starting" || mappingState === "running";
  $("#mapping-stop").disabled = mappingState === "idle";
  $("#map-lidar-hz").textContent = `${Number(mapping.lidar?.hz || 0).toFixed(1)} Hz`;
  $("#map-scan-hz").textContent = `${Number(mapping.scan?.hz || 0).toFixed(1)} Hz`;
  $("#map-odom-hz").textContent = `${Number(mapping.odometry?.hz || 0).toFixed(1)} Hz`;
  $("#map-map-hz").textContent = `${Number(mapping.map_topic?.hz || 0).toFixed(1)} Hz`;
  $("#map-size").textContent = mapping.map_info ? `${mapping.map_info.width} × ${mapping.map_info.height}` : "--";
  $("#map-resolution").textContent = mapping.map_info ? `${mapping.map_info.resolution} m` : "--";
  $("#map-root").textContent = mapping.map_root || "--";
  $("#mapping-process").textContent = mappingJob.script_enabled
    ? `脚本模式 · ${mappingLabels[mappingState] || mappingState}`
    : (mapping.processes?.mapping?.running ? `PID ${mapping.processes.mapping.pid}` : "未启动");

  $("#nav-stack-tag").textContent = mapping.navigation_active ? "已启动" : "未启动";
  $("#nav-stack-tag").className = `tag ${mapping.navigation_active ? "good" : "neutral"}`;
  const navClass = ["succeeded"].includes(nav.state) ? "good" : ["aborted", "error", "rejected"].includes(nav.state) ? "bad" : ["active", "sending", "canceling"].includes(nav.state) ? "warn" : "neutral";
  $("#nav-task-tag").textContent = nav.state || "idle";
  $("#nav-task-tag").className = `tag ${navClass}`;
  $("#nav-message").textContent = nav.message || "无导航任务";
  $("#nav-distance").textContent = nav.distance_remaining == null ? "-- m" : `${Number(nav.distance_remaining).toFixed(2)} m`;
  $("#nav-eta").textContent = nav.estimated_time_remaining_s == null ? "-- s" : `${Number(nav.estimated_time_remaining_s).toFixed(1)} s`;
  $("#nav-recoveries").textContent = nav.recoveries ?? 0;
  $("#maps-root-label").textContent = maps.root || "--";

  renderEvents();
}

function renderEvents() {
  const remote = app.state?.events || [];
  const merged = [...remote, ...app.localEvents].sort((a, b) => b.timestamp - a.timestamp).slice(0, 40);
  const list = $("#event-list");
  list.innerHTML = "";
  if (!merged.length) {
    list.innerHTML = "<li><time>--</time><i></i><span>暂无事件</span></li>";
    return;
  }
  for (const item of merged) {
    const li = document.createElement("li");
    li.innerHTML = `<time>${formatTime(item.timestamp)}</time><i class="${item.level || "info"}"></i><span></span>`;
    li.querySelector("span").textContent = `[${item.source || "system"}] ${item.message}`;
    list.appendChild(li);
  }
}

async function refreshState() {
  const result = await api("GET", "/api/v1/state");
  if (result.success && result.data) applyState(result.data);
}

async function refreshMaps() {
  const result = await api("GET", "/api/v1/maps");
  if (!result.success) return toast("地图库刷新失败", result.message, "error");
  app.maps = result.data || [];
  renderMapSelect();
  renderMapLibrary();
}

function renderMapSelect() {
  const select = $("#nav-map-select");
  const previous = select.value || app.state?.maps?.active || "";
  select.innerHTML = '<option value="">请选择地图</option>';
  for (const map of app.maps.filter((m) => m.valid !== false)) {
    const option = document.createElement("option");
    option.value = map.name;
    option.textContent = `${map.name}${map.active ? "（当前）" : ""}`;
    select.appendChild(option);
  }
  if (app.maps.some((m) => m.name === previous)) select.value = previous;
  updateMapPreview();
}

function updateMapPreview() {
  const name = $("#nav-map-select").value;
  const map = app.maps.find((item) => item.name === name);
  const image = $("#nav-map-preview");
  const empty = $("#nav-map-empty");
  if (map?.preview_url) {
    image.src = `${map.preview_url}${API_TOKEN ? `${map.preview_url.includes("?") ? "&" : "?"}token=${encodeURIComponent(API_TOKEN)}` : ""}`;
    image.style.display = "block";
    empty.style.display = "none";
    image.onerror = () => { image.style.display = "none"; empty.style.display = "block"; };
  } else {
    image.style.display = "none";
    empty.style.display = "block";
  }
}

function renderMapLibrary() {
  const root = $("#map-library");
  root.innerHTML = "";
  if (!app.maps.length) {
    root.innerHTML = '<div class="notice info"><b>暂无地图</b><span>完成建图后输入名称并保存，地图会出现在这里。</span></div>';
    return;
  }
  for (const map of app.maps) {
    const card = document.createElement("div");
    card.className = "map-card";
    const preview = map.preview_url ? `${map.preview_url}${API_TOKEN ? `&token=${encodeURIComponent(API_TOKEN)}` : ""}` : "";
    card.innerHTML = `
      <div class="map-card-image">${preview ? `<img src="${preview}" alt="${map.name}">` : "<span>无预览</span>"}</div>
      <div class="map-card-body">
        <div class="map-card-head"><h3></h3><span class="tag ${map.active ? "good" : "neutral"}">${map.active ? "当前地图" : "可用"}</span></div>
        <div class="map-card-meta"><span>分辨率 ${map.resolution ?? "--"}</span><span>${formatBytes(map.size_bytes || 0)}</span><span>${new Date((map.modified_at || 0) * 1000).toLocaleString("zh-CN")}</span><span>${map.mode || "trinary"}</span></div>
        <div class="map-card-actions"><button class="btn primary" data-map-load>加载地图</button><button class="btn secondary" data-map-nav>启动导航</button><button class="btn danger-soft" data-map-delete ${map.active ? "disabled" : ""}>删除</button></div>
      </div>`;
    card.querySelector("h3").textContent = map.name;
    card.querySelector("[data-map-load]").onclick = () => loadMap(map.name);
    card.querySelector("[data-map-nav]").onclick = () => startNavigation(map.name);
    card.querySelector("[data-map-delete]").onclick = () => deleteMap(map.name);
    root.appendChild(card);
  }
}

function formatBytes(value) {
  if (value < 1024) return `${value} B`;
  if (value < 1024 * 1024) return `${(value / 1024).toFixed(1)} KB`;
  return `${(value / 1024 / 1024).toFixed(1)} MB`;
}

async function action(path, label, body) {
  const result = await api("POST", path, body);
  toast(result.success ? `${label}成功` : `${label}失败`, result.message || "", result.success ? "success" : "error");
  localEvent(`${label}: ${result.message || (result.success ? "成功" : "失败")}`, result.success ? "success" : "error");
  await refreshState();
  return result;
}

async function loadMap(name) {
  if (!name) return toast("请选择地图", "", "warning");
  const ok = await confirmAction("切换地图", `切换到“${name}”会取消当前导航并保持停车，是否继续？`);
  if (!ok) return;
  const result = await action("/api/v1/maps/load", "切换地图", { name });
  if (result.success) await refreshMaps();
}

async function startNavigation(name) {
  if (!name) return toast("请选择地图", "", "warning");
  const result = await action("/api/v1/navigation/start", "启动导航", { map: name });
  if (result.success) navigate("navigation");
}

async function deleteMap(name) {
  const ok = await confirmAction("删除地图", `将永久删除地图“${name}”及对应图像文件，是否继续？`);
  if (!ok) return;
  const result = await api("DELETE", `/api/v1/maps/${encodeURIComponent(name)}`);
  toast(result.success ? "地图已删除" : "删除失败", result.message || "", result.success ? "success" : "error");
  if (result.success) refreshMaps();
}

async function refreshDiagnostics() {
  const result = await api("GET", "/api/v1/diagnostics");
  if (!result.success) return;
  const data = result.data || {};
  $("#node-count").textContent = data.nodes?.length || 0;
  $("#service-count").textContent = data.services?.length || 0;
  $("#topic-count").textContent = data.topics?.length || 0;
  $("#ros-nodes").textContent = (data.nodes || []).join("\n") || "没有发现节点";
  const processRoot = $("#process-list");
  processRoot.innerHTML = "";
  const processes = Object.entries(data.processes || {});
  if (!processes.length) processRoot.innerHTML = '<div class="diag-item"><b>暂无受控进程</b><small>通过网页启动建图或导航后会显示在这里。</small></div>';
  for (const [name, item] of processes) {
    const node = document.createElement("div");
    node.className = "diag-item";
    node.innerHTML = `<div><b>${name}</b><span class="tag ${item.running ? "good" : "neutral"}">${item.running ? "运行中" : `已退出 ${item.exit_code ?? ""}`}</span></div><small>PID ${item.pid} · ${item.command}</small><small>${item.log_path}</small>`;
    processRoot.appendChild(node);
  }
  const diagRoot = $("#driver-diagnostics");
  diagRoot.innerHTML = "";
  const diagnostics = data.driver_diagnostics || [];
  if (!diagnostics.length) diagRoot.innerHTML = '<div class="diag-item"><b>暂无诊断消息</b><small>等待 /diagnostics 数据。</small></div>';
  for (const item of diagnostics) {
    const node = document.createElement("div");
    node.className = "diag-item";
    const tag = item.level >= 2 ? "bad" : item.level === 1 ? "warn" : "good";
    node.innerHTML = `<div><b></b><span class="tag ${tag}">${item.message || "OK"}</span></div><small></small>`;
    node.querySelector("b").textContent = item.name;
    node.querySelector("small").textContent = item.hardware_id || "";
    diagRoot.appendChild(node);
  }
}

function setCommand(vx, vy, wz) {
  app.command = { vx, vy, wz };
  $("#cmd-vx").textContent = vx.toFixed(2);
  $("#cmd-vy").textContent = vy.toFixed(2);
  $("#cmd-wz").textContent = wz.toFixed(2);
  $("#bar-vx").style.width = `${Math.min(100, Math.abs(vx) * 100)}%`;
  $("#bar-vy").style.width = `${Math.min(100, Math.abs(vy) * 100)}%`;
  $("#bar-wz").style.width = `${Math.min(100, Math.abs(wz) * 100)}%`;
}

function zeroCommand(send = true) {
  setCommand(0, 0, 0);
  $("#joystick-knob").style.transform = "translate(0px,0px)";
  if (send && app.manual) wsSend({ type: "teleop", ...app.command });
}

function setupJoystick() {
  const joystick = $("#joystick");
  const knob = $("#joystick-knob");
  function update(event) {
    const rect = joystick.getBoundingClientRect();
    const point = event.touches ? event.touches[0] : event;
    const dx = point.clientX - (rect.left + rect.width / 2);
    const dy = point.clientY - (rect.top + rect.height / 2);
    const radius = rect.width * 0.32;
    const length = Math.hypot(dx, dy);
    const scale = length > radius ? radius / length : 1;
    const x = dx * scale;
    const y = dy * scale;
    knob.style.transform = `translate(${x}px,${y}px)`;
    setCommand((-y / radius) * app.speed, 0, (-x / radius) * app.speed);
  }
  joystick.addEventListener("pointerdown", (event) => { app.pointerActive = true; joystick.setPointerCapture(event.pointerId); update(event); });
  joystick.addEventListener("pointermove", (event) => { if (app.pointerActive) update(event); });
  const release = () => { app.pointerActive = false; zeroCommand(); };
  joystick.addEventListener("pointerup", release);
  joystick.addEventListener("pointercancel", release);
}

function updateKeyboardCommand() {
  if (!app.manual) return zeroCommand(false);
  const vx = (app.keys.has("w") ? 1 : 0) - (app.keys.has("s") ? 1 : 0);
  const wz = (app.keys.has("a") ? 1 : 0) - (app.keys.has("d") ? 1 : 0);
  setCommand(vx * app.speed, 0, wz * app.speed);
}

function setupHoldUnlock() {
  const button = $("#unlock-btn");
  let timer = null;
  const start = () => {
    if (timer) return;
    button.classList.add("holding");
    timer = setTimeout(async () => {
      timer = null;
      button.classList.remove("holding");
      await action("/api/v1/robot/read_only", "运动解锁", { read_only: false });
    }, 1200);
  };
  const stop = () => { if (timer) clearTimeout(timer); timer = null; button.classList.remove("holding"); };
  button.addEventListener("pointerdown", start);
  button.addEventListener("pointerup", stop);
  button.addEventListener("pointerleave", stop);
  button.addEventListener("pointercancel", stop);
}


function liveImageUrl(path) {
  if (!path) return "";
  if (!API_TOKEN) return path;
  return `${path}${path.includes("?") ? "&" : "?"}token=${encodeURIComponent(API_TOKEN)}`;
}

function worldToMapPixel(x, y, map) {
  const origin = map.origin || { x: 0, y: 0, yaw_deg: 0 };
  const yaw = Number(origin.yaw_deg || 0) * Math.PI / 180;
  const dx = x - Number(origin.x || 0);
  const dy = y - Number(origin.y || 0);
  const localX = Math.cos(yaw) * dx + Math.sin(yaw) * dy;
  const localY = -Math.sin(yaw) * dx + Math.cos(yaw) * dy;
  return {
    x: localX / map.resolution,
    y: map.height - 1 - localY / map.resolution,
  };
}

function mapPixelToWorld(px, py, map) {
  const origin = map.origin || { x: 0, y: 0, yaw_deg: 0 };
  const yaw = Number(origin.yaw_deg || 0) * Math.PI / 180;
  const localX = px * map.resolution;
  const localY = (map.height - 1 - py) * map.resolution;
  return {
    x: Number(origin.x || 0) + Math.cos(yaw) * localX - Math.sin(yaw) * localY,
    y: Number(origin.y || 0) + Math.sin(yaw) * localX + Math.cos(yaw) * localY,
  };
}

function resizeLiveCanvas() {
  const canvas = $("#live-map-canvas");
  if (!canvas) return null;
  const rect = canvas.getBoundingClientRect();
  const width = Math.max(320, rect.width || 320);
  const height = Math.max(320, rect.height || 320);
  const dpr = Math.min(2, window.devicePixelRatio || 1);
  const targetWidth = Math.round(width * dpr);
  const targetHeight = Math.round(height * dpr);
  if (canvas.width !== targetWidth || canvas.height !== targetHeight) {
    canvas.width = targetWidth;
    canvas.height = targetHeight;
  }
  const ctx = canvas.getContext("2d");
  ctx.setTransform(dpr, 0, 0, dpr, 0, 0);
  return { canvas, ctx, width, height, dpr };
}

function liveScreenPoint(worldX, worldY, map, transform) {
  const pixel = worldToMapPixel(worldX, worldY, map);
  return {
    x: transform.offsetX + pixel.x * transform.scale,
    y: transform.offsetY + pixel.y * transform.scale,
  };
}

function renderLiveMapping() {
  const surface = resizeLiveCanvas();
  if (!surface) return;
  const { ctx, width, height } = surface;
  ctx.clearRect(0, 0, width, height);
  ctx.fillStyle = "#07111d";
  ctx.fillRect(0, 0, width, height);

  const data = app.live.data;
  const map = data?.map;
  const image = app.live.image;
  const available = Boolean(data?.available && map && image && app.live.imageReady);
  $("#live-map-empty").classList.toggle("hidden", available);
  if (!available) {
    app.live.transform = null;
    return;
  }

  const fitScale = Math.max(0.001, Math.min((width - 28) / map.width, (height - 28) / map.height));
  const scale = fitScale * app.live.zoom;
  let offsetX = (width - map.width * scale) / 2 + app.live.panX;
  let offsetY = (height - map.height * scale) / 2 + app.live.panY;

  if (app.live.follow && data.pose) {
    const robotPixel = worldToMapPixel(data.pose.x, data.pose.y, map);
    offsetX = width / 2 - robotPixel.x * scale;
    offsetY = height / 2 - robotPixel.y * scale;
  }

  const transform = { offsetX, offsetY, scale, width, height };
  app.live.transform = transform;

  ctx.save();
  ctx.imageSmoothingEnabled = false;
  ctx.shadowColor = "rgba(71, 226, 255, .18)";
  ctx.shadowBlur = 14;
  ctx.drawImage(image, offsetX, offsetY, map.width * scale, map.height * scale);
  ctx.shadowBlur = 0;
  ctx.strokeStyle = "rgba(122, 227, 255, .35)";
  ctx.lineWidth = 1;
  ctx.strokeRect(offsetX, offsetY, map.width * scale, map.height * scale);
  ctx.restore();

  if (app.live.showPath && Array.isArray(data.path) && data.path.length > 1) {
    ctx.save();
    ctx.strokeStyle = "#42d8ff";
    ctx.lineWidth = 2.4;
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.shadowColor = "rgba(66, 216, 255, .7)";
    ctx.shadowBlur = 8;
    ctx.beginPath();
    data.path.forEach((point, index) => {
      const screen = liveScreenPoint(point[0], point[1], map, transform);
      if (index === 0) ctx.moveTo(screen.x, screen.y);
      else ctx.lineTo(screen.x, screen.y);
    });
    ctx.stroke();
    ctx.restore();
  }

  if (app.live.showScan && Array.isArray(data.scan?.points)) {
    ctx.save();
    ctx.fillStyle = "rgba(255, 186, 73, .92)";
    ctx.shadowColor = "rgba(255, 174, 42, .7)";
    ctx.shadowBlur = 4;
    const radius = Math.max(1.2, Math.min(2.4, scale * 0.9));
    for (const point of data.scan.points) {
      const screen = liveScreenPoint(point[0], point[1], map, transform);
      if (screen.x < -4 || screen.y < -4 || screen.x > width + 4 || screen.y > height + 4) continue;
      ctx.beginPath();
      ctx.arc(screen.x, screen.y, radius, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  if (data.pose) {
    const center = liveScreenPoint(data.pose.x, data.pose.y, map, transform);
    const yaw = Number(data.pose.yaw_deg || 0) * Math.PI / 180;
    const forward = liveScreenPoint(
      data.pose.x + 0.45 * Math.cos(yaw),
      data.pose.y + 0.45 * Math.sin(yaw),
      map,
      transform,
    );
    const angle = Math.atan2(forward.y - center.y, forward.x - center.x);
    ctx.save();
    ctx.translate(center.x, center.y);
    ctx.rotate(angle);
    ctx.shadowColor = "rgba(80, 255, 178, .9)";
    ctx.shadowBlur = 12;
    ctx.fillStyle = "#50ffb2";
    ctx.beginPath();
    ctx.moveTo(15, 0);
    ctx.lineTo(-10, -9);
    ctx.lineTo(-6, 0);
    ctx.lineTo(-10, 9);
    ctx.closePath();
    ctx.fill();
    ctx.fillStyle = "#09141e";
    ctx.beginPath();
    ctx.arc(0, 0, 3.2, 0, Math.PI * 2);
    ctx.fill();
    ctx.restore();
  }

  $("#live-map-scale").textContent = `${app.live.zoom.toFixed(2)}×`;
}

function updateLiveMapStatus(data) {
  const map = data?.map;
  const pose = data?.pose;
  const scan = data?.scan || {};
  const poseError = data?.errors?.pose || "";
  const scanError = data?.errors?.scan || "";

  $("#live-map-state").textContent = !data?.enabled
    ? "功能已关闭"
    : data?.available
      ? (pose ? "地图与位姿正常" : "地图正常，等待 TF")
      : "等待 /map";
  $("#live-map-pose").textContent = pose
    ? `X ${Number(pose.x).toFixed(2)} · Y ${Number(pose.y).toFixed(2)} · ${Number(pose.yaw_deg).toFixed(1)}°`
    : "--";
  $("#live-map-scan-count").textContent = String(scan.count || 0);
  $("#live-map-age").textContent = map?.age_s == null ? "--" : `${Number(map.age_s).toFixed(2)} s`;
  const empty = $("#live-map-empty");
  if (!data?.available) {
    empty.querySelector("b").textContent = "等待实时地图";
    empty.querySelector("span").textContent = "点击“开始建图”，并确认 /map 正常发布。";
  } else if (!pose) {
    empty.querySelector("b").textContent = "等待机器人 TF";
    empty.querySelector("span").textContent = poseError || "请检查 map → base_link TF。";
  } else if (!scan.points?.length && scanError) {
    $("#live-map-state").textContent = "地图正常，激光叠加等待 TF";
  }
}

function requestLiveMapImage(map) {
  if (!map || !map.image_url) return;
  const version = Number(map.version || 0);
  if (app.live.imageVersion === version || app.live.requestedVersion === version) return;
  app.live.requestedVersion = version;
  const image = new Image();
  image.decoding = "async";
  image.onload = () => {
    app.live.image = image;
    app.live.imageVersion = version;
    app.live.requestedVersion = -1;
    app.live.imageReady = true;
    renderLiveMapping();
  };
  image.onerror = () => {
    app.live.requestedVersion = -1;
    app.live.imageReady = false;
  };
  image.src = liveImageUrl(`${map.image_url}${map.image_url.includes("?") ? "&" : "?"}_=${Date.now()}`);
}

async function refreshLiveMapping() {
  if (app.live.loading) return;
  app.live.loading = true;
  try {
    const result = await api("GET", "/api/v1/live_mapping");
    if (!result.success) return;
    app.live.data = result.data || null;
    updateLiveMapStatus(app.live.data);
    if (app.live.data?.map) requestLiveMapImage(app.live.data.map);
    renderLiveMapping();
  } finally {
    app.live.loading = false;
  }
}

function setupLiveMap() {
  const canvas = $("#live-map-canvas");
  if (!canvas) return;

  const updateButtons = () => {
    $("#live-follow").classList.toggle("active", app.live.follow);
    $("#live-toggle-scan").classList.toggle("active", app.live.showScan);
    $("#live-toggle-path").classList.toggle("active", app.live.showPath);
  };

  $("#live-fit").onclick = () => {
    app.live.follow = false;
    app.live.zoom = 1;
    app.live.panX = 0;
    app.live.panY = 0;
    updateButtons();
    renderLiveMapping();
  };
  $("#live-follow").onclick = () => {
    app.live.follow = !app.live.follow;
    if (app.live.follow) {
      app.live.panX = 0;
      app.live.panY = 0;
    }
    updateButtons();
    renderLiveMapping();
  };
  $("#live-toggle-scan").onclick = () => {
    app.live.showScan = !app.live.showScan;
    updateButtons();
    renderLiveMapping();
  };
  $("#live-toggle-path").onclick = () => {
    app.live.showPath = !app.live.showPath;
    updateButtons();
    renderLiveMapping();
  };
  $("#live-reset-path").onclick = async () => {
    const result = await action("/api/v1/live_mapping/reset_path", "清空建图轨迹");
    if (result.success) refreshLiveMapping();
  };

  canvas.addEventListener("wheel", (event) => {
    event.preventDefault();
    app.live.follow = false;
    const factor = event.deltaY < 0 ? 1.12 : 0.89;
    app.live.zoom = Math.max(0.2, Math.min(8, app.live.zoom * factor));
    updateButtons();
    renderLiveMapping();
  }, { passive: false });

  canvas.addEventListener("pointerdown", (event) => {
    app.live.dragging = true;
    app.live.follow = false;
    app.live.dragX = event.clientX;
    app.live.dragY = event.clientY;
    canvas.setPointerCapture(event.pointerId);
    updateButtons();
  });
  canvas.addEventListener("pointermove", (event) => {
    if (app.live.dragging) {
      app.live.panX += event.clientX - app.live.dragX;
      app.live.panY += event.clientY - app.live.dragY;
      app.live.dragX = event.clientX;
      app.live.dragY = event.clientY;
      renderLiveMapping();
    }
    const transform = app.live.transform;
    const map = app.live.data?.map;
    if (transform && map) {
      const rect = canvas.getBoundingClientRect();
      const px = (event.clientX - rect.left - transform.offsetX) / transform.scale;
      const py = (event.clientY - rect.top - transform.offsetY) / transform.scale;
      const world = mapPixelToWorld(px, py, map);
      $("#live-map-coordinate").textContent = `X ${world.x.toFixed(2)} · Y ${world.y.toFixed(2)}`;
    }
  });
  const release = () => { app.live.dragging = false; };
  canvas.addEventListener("pointerup", release);
  canvas.addEventListener("pointercancel", release);
  canvas.addEventListener("dblclick", () => $("#live-fit").click());

  if (window.ResizeObserver) {
    new ResizeObserver(() => renderLiveMapping()).observe($("#live-map-stage"));
  } else {
    window.addEventListener("resize", renderLiveMapping);
  }
  updateButtons();
}

function bindEvents() {
  $$(".nav-item").forEach((button) => button.onclick = () => navigate(button.dataset.page));
  $$('[data-go]').forEach((button) => button.onclick = () => navigate(button.dataset.go));
  $$('[data-refresh]').forEach((button) => button.onclick = refreshState);
  $("#clear-local-log").onclick = () => { app.localEvents = []; renderEvents(); };
  $("#nav-map-select").onchange = updateMapPreview;
  $("#refresh-maps").onclick = refreshMaps;
  $("#refresh-diagnostics").onclick = refreshDiagnostics;

  $("#manual-mode").onclick = () => wsSend({ type: "control_mode", mode: "manual" });
  $("#auto-mode").onclick = () => wsSend({ type: "control_mode", mode: "auto" });
  $("#top-hold").onclick = () => wsSend({ type: "control_mode", mode: "manual" });
  $$("[data-speed]").forEach((button) => button.onclick = () => {
    app.speed = Number(button.dataset.speed);
    $$("[data-speed]").forEach((item) => item.classList.toggle("active", item === button));
  });
  $("#zero-command").onclick = () => zeroCommand();

  $$('[data-action]').forEach((button) => button.onclick = () => action(`/api/v1/robot/${button.dataset.action}`, button.querySelector("b").textContent));
  $("#lock-btn").onclick = () => action("/api/v1/robot/read_only", "安全上锁", { read_only: true });
  $("#reset-estop").onclick = async () => {
    if (await confirmAction("复位急停", "复位后机器人仍保持 read_only 锁定，需要再次长按解锁。")) action("/api/v1/robot/reset_estop", "复位急停");
  };
  const estop = async () => action("/api/v1/robot/estop", "紧急停止");
  $("#top-estop").onclick = estop;
  $("#manual-estop").onclick = estop;

  $("#mapping-start").onclick = async () => {
    const button = $("#mapping-start");
    button.disabled = true;
    button.textContent = "正在启动…";
    const result = await action("/api/v1/mapping/start", "开始建图");
    if (result.success) navigate("mapping");
    button.textContent = "开始建图";
    await refreshState();
  };
  $("#mapping-stop").onclick = async () => {
    if (await confirmAction("停止建图", "停止由网页启动的建图进程。尚未保存的地图不会自动保存。")) action("/api/v1/mapping/stop", "停止建图");
  };
  $("#map-save").onclick = async () => {
    const name = $("#map-save-name").value.trim();
    if (!name) return toast("请输入地图名称", "可使用字母、数字、中文、下划线和短横线", "warning");
    const result = await action("/api/v1/maps/save", "保存地图", { name });
    if (result.success) refreshMaps();
  };

  $("#load-map").onclick = () => loadMap($("#nav-map-select").value);
  $("#start-navigation").onclick = () => startNavigation($("#nav-map-select").value);
  $("#stop-navigation").onclick = async () => {
    if (await confirmAction("停止导航栈", "将取消当前任务并停止由网页启动的导航进程。")) action("/api/v1/navigation/stop", "停止导航");
  };
  $("#send-goal").onclick = () => action("/api/v1/navigation/goal", "发送导航目标", {
    x: Number($("#goal-x").value), y: Number($("#goal-y").value), yaw_deg: Number($("#goal-yaw").value),
  });
  $("#cancel-goal").onclick = () => action("/api/v1/navigation/cancel", "取消导航");
  $("#clear-costmaps").onclick = () => action("/api/v1/navigation/clear_costmaps", "清除代价地图");
  $("#use-current-pose").onclick = () => {
    const pose = app.state?.robot?.pose;
    if (!pose) return toast("当前无定位数据", "", "warning");
    $("#loc-x").value = pose.x; $("#loc-y").value = pose.y; $("#loc-yaw").value = pose.yaw_deg;
  };
  $("#set-initial-pose").onclick = async () => {
    if (!(await confirmAction("执行重定位", "系统将取消导航、保持停车、发布初始位姿并清除代价地图。"))) return;
    action("/api/v1/localization/initial_pose", "设置初始位姿", {
      x: Number($("#loc-x").value), y: Number($("#loc-y").value), yaw_deg: Number($("#loc-yaw").value),
    });
  };
  $("#global-localization").onclick = async () => {
    if (await confirmAction("全局重定位", "AMCL 将扩大搜索范围。机器人会保持停车，请随后观察定位是否收敛。")) action("/api/v1/localization/global", "全局重定位");
  };
  $("#nomotion-update").onclick = () => action("/api/v1/localization/nomotion_update", "静止更新");

  window.addEventListener("keydown", (event) => {
    if (["INPUT", "SELECT", "TEXTAREA"].includes(document.activeElement?.tagName)) return;
    const key = event.key.toLowerCase();
    if (["w", "a", "s", "d"].includes(key)) { app.keys.add(key); updateKeyboardCommand(); event.preventDefault(); }
    if (event.code === "Space") { zeroCommand(); event.preventDefault(); }
  });
  window.addEventListener("keyup", (event) => { app.keys.delete(event.key.toLowerCase()); updateKeyboardCommand(); });
  window.addEventListener("blur", () => { app.keys.clear(); zeroCommand(); });
  document.addEventListener("visibilitychange", () => { if (document.hidden) { app.keys.clear(); zeroCommand(); } });
  setupJoystick();
  setupHoldUnlock();
  setupLiveMap();
}

function start() {
  bindEvents();
  connectSocket();
  refreshState();
  refreshMaps();
  setInterval(() => $("#clock").textContent = new Date().toLocaleTimeString("zh-CN", { hour12: false }), 1000);
  app.poll = setInterval(() => { if (!app.socketOnline) refreshState(); }, 1500);
  setInterval(() => { if (["maps", "navigation"].includes(app.page)) refreshMaps(); }, 10000);
  app.live.poll = setInterval(() => { if (app.page === "mapping") refreshLiveMapping(); }, 300);
}

document.addEventListener("DOMContentLoaded", start);

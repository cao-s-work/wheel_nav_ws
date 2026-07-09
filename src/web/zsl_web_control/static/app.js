// ZSL-1W Web 控制台 — 前端逻辑
const WS_URL = `ws://${location.host}/ws`;
let ws = null;
let heartbeatTimer = null;
let teleopTimer = null;
let teleopActive = false;

// =========================================================================
// WebSocket
// =========================================================================
function connectWS() {
  ws = new WebSocket(WS_URL);
  ws.onopen = () => {
    document.getElementById("ws-status").textContent = "在线";
    document.getElementById("ws-status").className = "online";
    startHeartbeat();
  };
  ws.onclose = () => {
    document.getElementById("ws-status").textContent = "离线";
    document.getElementById("ws-status").className = "offline";
    stopHeartbeat();
    setTimeout(connectWS, 2000);
  };
  ws.onerror = () => ws.close();
}

function startHeartbeat() {
  stopHeartbeat();
  heartbeatTimer = setInterval(() => {
    if (ws && ws.readyState === WebSocket.OPEN) {
      ws.send(JSON.stringify({ type: "heartbeat" }));
    }
  }, 100);
}

function stopHeartbeat() {
  if (heartbeatTimer) { clearInterval(heartbeatTimer); heartbeatTimer = null; }
  // 断连归零
  sendTeleop(0, 0, 0);
}

// =========================================================================
// REST API
// =========================================================================
async function api(method, path, body) {
  const opts = { method, headers: { "Content-Type": "application/json" } };
  if (body) opts.body = JSON.stringify(body);
  try {
    const resp = await fetch(path, opts);
    return await resp.json();
  } catch (e) {
    console.error("API error:", e);
    return null;
  }
}

// =========================================================================
// 导航
// =========================================================================
function sendNavGoal() {
  const x = parseFloat(document.getElementById("nav-x").value);
  const y = parseFloat(document.getElementById("nav-y").value);
  const yaw = parseFloat(document.getElementById("nav-yaw").value);
  api("POST", "/api/nav/goal", { x, y, yaw });
}

// =========================================================================
// 虚拟摇杆
// =========================================================================
let joystickState = { vx: 0, vy: 0, wz: 0 };

function initJoystick() {
  const canvas = document.getElementById("joystick");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");
  const cx = 100, cy = 100, r = 70;

  function draw(knobX, knobY) {
    ctx.clearRect(0, 0, 200, 200);
    ctx.beginPath(); ctx.arc(cx, cy, r, 0, Math.PI * 2);
    ctx.strokeStyle = "#666"; ctx.lineWidth = 2; ctx.stroke();
    ctx.beginPath(); ctx.arc(knobX, knobY, 15, 0, Math.PI * 2);
    ctx.fillStyle = "#2196F3"; ctx.fill();
  }

  function getPos(e) {
    const rect = canvas.getBoundingClientRect();
    return { x: e.clientX - rect.left, y: e.clientY - rect.top };
  }

  function update(knobX, knobY) {
    const dx = (knobX - cx) / r;
    const dy = (knobY - cy) / r;
    const dist = Math.sqrt(dx * dx + dy * dy);
    let vx = dx, vy = dy;
    if (dist > 1) { vx /= dist; vy /= dist; }
    joystickState.vx = Math.round(vx * 100) / 100;
    joystickState.vy = 0;  // 横移第一阶段关闭
    joystickState.wz = 0;  // 旋转需额外控件
    const clampX = cx + vx * r;
    const clampY = cy + vy * r;
    draw(clampX, clampY);
    sendTeleop(joystickState.vx, joystickState.vy, joystickState.wz);
    document.getElementById("teleop-values").textContent =
      `vx: ${joystickState.vx.toFixed(2)} vy: ${joystickState.vy.toFixed(2)} wz: ${joystickState.wz.toFixed(2)}`;
  }

  canvas.addEventListener("mousedown", (e) => {
    teleopActive = true;
    const p = getPos(e); update(p.x, p.y);
    if (!teleopTimer) {
      teleopTimer = setInterval(() => {
        if (teleopActive) sendTeleop(joystickState.vx, joystickState.vy, joystickState.wz);
      }, 50);
    }
  });
  canvas.addEventListener("mousemove", (e) => {
    if (!teleopActive) return;
    const p = getPos(e); update(p.x, p.y);
  });
  canvas.addEventListener("mouseup", () => {
    teleopActive = false;
    joystickState.vx = joystickState.vy = joystickState.wz = 0;
    draw(cx, cy);
    sendTeleop(0, 0, 0);
    if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; }
    document.getElementById("teleop-values").textContent = "vx: 0.00 vy: 0.00 wz: 0.00";
  });
  canvas.addEventListener("mouseleave", () => {
    teleopActive = false;
    joystickState.vx = joystickState.vy = joystickState.wz = 0;
    draw(cx, cy);
    sendTeleop(0, 0, 0);
    if (teleopTimer) { clearInterval(teleopTimer); teleopTimer = null; }
  });

  draw(cx, cy);
}

function sendTeleop(vx, vy, wz) {
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "teleop", vx, vy, wz }));
  }
}

function setSpeedLevel(level) {
  // 通过 WebSocket 发送档位切换
  if (ws && ws.readyState === WebSocket.OPEN) {
    ws.send(JSON.stringify({ type: "speed", level }));
  }
}

// =========================================================================
// 状态轮询刷新 Dashboard
// =========================================================================
async function pollState() {
  const state = await api("GET", "/api/state");
  if (state) {
    document.getElementById("sdk-connected").textContent = state.connected ? "已连接" : "断开";
    document.getElementById("battery").textContent = state.battery + "%";
    document.getElementById("ctrl-mode").textContent = state.ctrl_mode;
    document.getElementById("read-only").textContent = state.read_only ? "锁定" : "解锁";
  }
}

// =========================================================================
// 启动
// =========================================================================
connectWS();
initJoystick();
setInterval(pollState, 1000);

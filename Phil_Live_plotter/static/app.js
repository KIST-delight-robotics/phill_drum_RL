/*
 * Drum Robot Live CAN Plotter - frontend
 *
 * 백엔드(backend/can_bridge.py)가 붙으면 LIVE, 못 붙으면 목업 데이터로 돈다.
 *
 * 백엔드 -> GUI
 *   { type: "frame", motor: "R_arm1", dir: "tx" | "rx", mode: "servo" | "mit",
 *     ts: <epoch seconds>, decoded: { pos?, vel?, tor?, kp?, kd?, packet? },
 *     raw: { id: "0x101", dlc: 8, data: [0x12, 0x34, ...] } }
 *   { type: "status", motor: "R_arm1", connected: true }
 *   { type: "log_status", logging: true, file: "logs/candump-....log" }
 *   { type: "log_list", files: [{path, name, size, mtime}, ...] }
 *   { type: "file_series", file, tStart, tEnd, stats, series: {motorId: {posCmd: [[t,y],...]}} }
 *
 * GUI -> 백엔드
 *   { type: "control", action: "set_modes", globalMode: "mit", overrides: {...} }
 *   { type: "control", action: "start_logging" | "stop_logging" }
 *   { type: "control", action: "list_logs" }
 *   { type: "control", action: "load_log", path: "logs/....log", buckets: 800 }
 *
 * 모터 매핑은 config/motors.json 에서 읽는다 (백엔드도 같은 파일을 본다).
 *
 * ingestFrame() / setMotorConnected() 가 목업과 실데이터의 공통 진입점이다.
 *
 * 데이터 포인트는 {x: epochSeconds, y: value} 형태로 저장하고 Chart.js의
 * linear x축을 쓴다. command 프레임과 feedback 프레임은 서로 다른 배열에
 * 들어가므로(공유 인덱스를 쓰지 않으므로) 한쪽이 비어 있어도 다른 쪽 선이
 * 끊기지 않는다.
 */

// config/motors.json 을 못 읽었을 때만 쓰는 최소 기본값
const FALLBACK_CONFIG = {
  defaultMode: "servo",
  motors: [
    { id: "waist", label: "Waist", nodeId: 0, type: "TMotor" },
  ],
};

// 모터별 고정 색상 (전체 보기에서 모터 구분용)
const MOTOR_COLORS = [
  "#4C9AFF", "#36B37E", "#FF8B00", "#FF5C5C",
  "#B968C7", "#00C7E6", "#FFD23F", "#FF6EC7",
  "#82C91E", "#748FFC", "#E8590C", "#63E6BE",
];

let MOTORS = [];
const SERIES_KEYS = ["posCmd", "posFb", "velCmd", "velFb", "torCmd", "torFb"];

function motorColor(motorId) {
  const idx = MOTORS.findIndex((m) => m.id === motorId);
  return MOTOR_COLORS[idx % MOTOR_COLORS.length];
}

const MAX_POINTS = 400; // 라이브 모드에서 라인 하나당 유지할 점 개수
// 실제 제어 주기는 5ms(모터당 200Hz)라 들어오는 대로 다 찍으면 브라우저가 못 버틴다.
// 프레임 로그와 현재값은 전부 반영하고, 그래프에 찍는 점만 이 간격으로 솎아낸다.
const PLOT_MIN_INTERVAL_SEC = 0.02; // 50Hz -> 400점이면 약 8초 창

const state = {
  globalMode: "servo",
  selectedMotorId: null,
  view: "single", // "single" | "overview"
  dataSource: "mock", // "mock" | "live" | "file"
  ws: null,
  logging: false,
  loggingFile: null,
  motors: {},
  allFrames: [], // 전체 모터 프레임 로그 (최신이 앞)
  fileMeta: null, // 로그 파일 분석 모드일 때 파일 정보
  pendingFile: null, // URL 해시로 지정한, 연결되면 바로 열 로그 파일
};

function initMotorState(motors) {
  MOTORS = motors;
  state.motors = {};
  for (const m of MOTORS) {
    state.motors[m.id] = {
      ...m,
      modeOverride: "global", // "global" | "servo" | "mit"
      connected: false,
      posCmd: [], posFb: [],
      velCmd: [], velFb: [],
      torCmd: [], torFb: [],
      last: { pos: 0, vel: 0, tor: 0 },
      lastPlotted: { tx: 0, rx: 0 },
    };
  }
  state.selectedMotorId = MOTORS.length ? MOTORS[0].id : null;
}

async function loadConfig() {
  try {
    const res = await fetch("config/motors.json", { cache: "no-store" });
    if (!res.ok) throw new Error("HTTP " + res.status);
    const cfg = await res.json();
    // 정본은 로봇 설정이고 백엔드가 붙으면 그걸로 갈아끼운다.
    // 여기서 읽는 건 백엔드가 없을 때(목업) 쓰는 예비 목록이다.
    initMotorState(cfg.fallbackMotors || cfg.motors || []);
    state.globalMode = cfg.defaultMode || "servo";
    return true;
  } catch (err) {
    console.error("config/motors.json 로드 실패 - 기본값으로 동작합니다:", err);
    initMotorState(FALLBACK_CONFIG.motors);
    state.globalMode = FALLBACK_CONFIG.defaultMode;
    return false;
  }
}

function effectiveMode(motor) {
  if (motor.type === "Maxon") return "maxon";
  return motor.modeOverride === "global" ? state.globalMode : motor.modeOverride;
}

// ---------- Chart.js setup ----------

Chart.defaults.color = "#8b95a5";
Chart.defaults.borderColor = "#262d38";
Chart.defaults.font.family = "'JetBrains Mono', monospace";
Chart.defaults.font.size = 10;

function timeTickCallback(value) {
  // 라이브(몇 초 창)면 초 단위로만 찍으면 눈금이 전부 같은 값이 되고,
  // 로그 파일 전체 구간(분~시간)이면 반대로 시:분:초 가 필요하다.
  const span = this.max - this.min;
  const d = new Date(value * 1000);
  const pad = (n) => String(n).padStart(2, "0");
  if (span > 120) return `${pad(d.getHours())}:${pad(d.getMinutes())}:${pad(d.getSeconds())}`;
  return `${pad(d.getMinutes())}:${pad(d.getSeconds())}.${Math.floor(d.getMilliseconds() / 100)}`;
}

function xAxisOptions() {
  return {
    type: "linear",
    ticks: { maxTicksLimit: 6, callback: timeTickCallback },
    grid: { color: "#1c222b" },
  };
}

function makeLineChart(canvasId, datasets) {
  const ctx = document.getElementById(canvasId).getContext("2d");
  return new Chart(ctx, {
    type: "line",
    data: { datasets },
    options: {
      animation: false,
      responsive: true,
      maintainAspectRatio: false,
      parsing: false, // datasets already provide {x, y} points
      interaction: { mode: "nearest", intersect: false },
      scales: {
        x: xAxisOptions(),
        y: { grid: { color: "#1c222b" } },
      },
      plugins: { legend: { display: false } },
    },
  });
}

function lineDataset(label, data, color, dashed) {
  return {
    label,
    data,
    borderColor: color,
    borderWidth: 1.5,
    borderDash: dashed ? [5, 3] : [],
    pointRadius: 0,
    tension: 0.15,
    spanGaps: true,
  };
}

// 단일 모터 뷰: 필드별로 command/feedback 두 라인
const chartPos = makeLineChart("chartPos", [
  lineDataset("Command", [], "#4c9aff", false),
  lineDataset("Feedback", [], "#a6d4ff", true),
]);
const chartVel = makeLineChart("chartVel", [
  lineDataset("Command", [], "#36b37e", false),
  lineDataset("Feedback", [], "#b3f1d8", true),
]);
const chartTor = makeLineChart("chartTor", [
  lineDataset("Command", [], "#ff8b00", false),
  lineDataset("Feedback", [], "#ffd9a6", true),
]);

function refreshChart(chart) {
  chart.update("none");
  const empty = chart.data.datasets.every((d) => d.data.length === 0);
  chart.canvas.closest(".chart-card").classList.toggle("empty", empty);
}

function renderSelectedMotorCharts() {
  const m = state.motors[state.selectedMotorId];
  chartPos.data.datasets[0].data = m.posCmd;
  chartPos.data.datasets[1].data = m.posFb;
  chartVel.data.datasets[0].data = m.velCmd;
  chartVel.data.datasets[1].data = m.velFb;
  chartTor.data.datasets[0].data = m.torCmd;
  chartTor.data.datasets[1].data = m.torFb;

  refreshChart(chartPos);
  refreshChart(chartVel);
  refreshChart(chartTor);
}

// 전체 모터 보기: 필드 x {command, feedback} 6개 차트, 모터별 1라인씩
function motorDatasets(bufferKey) {
  return MOTORS.map((m) => lineDataset(m.label, state.motors[m.id][bufferKey], motorColor(m.id), false));
}

// 모터 목록을 config 에서 읽은 뒤에 만든다
let overviewCharts = {};

function createOverviewCharts() {
  // 모터 목록이 바뀌어 다시 만들 때는 기존 차트를 먼저 버려야 한다
  // (같은 canvas 에 두 번 만들면 Chart.js 가 거부한다)
  for (const chart of Object.values(overviewCharts)) chart.destroy();
  overviewCharts = {
    posCmd: makeLineChart("chartPosCmdAll", motorDatasets("posCmd")),
    posFb: makeLineChart("chartPosFbAll", motorDatasets("posFb")),
    velCmd: makeLineChart("chartVelCmdAll", motorDatasets("velCmd")),
    velFb: makeLineChart("chartVelFbAll", motorDatasets("velFb")),
    torCmd: makeLineChart("chartTorCmdAll", motorDatasets("torCmd")),
    torFb: makeLineChart("chartTorFbAll", motorDatasets("torFb")),
  };
}

function renderOverviewCharts() {
  // 데이터셋이 모터 버퍼 배열을 직접 참조하고 있고, ingestFrame은 push/shift로
  // 그 배열을 제자리에서 변경하므로 update()만 호출하면 된다.
  for (const chart of Object.values(overviewCharts)) refreshChart(chart);
}

function renderOverviewLegend() {
  const el = document.getElementById("overviewLegend");
  el.innerHTML = MOTORS.map(
    (m) => `<span class="overview-legend-item"><i class="swatch-dot" style="background:${motorColor(m.id)}"></i>${m.label}</span>`
  ).join("");
}

// ---------- Sidebar / motor list ----------

const motorListEl = document.getElementById("motorList");

function renderMotorList() {
  motorListEl.innerHTML = "";
  for (const m of MOTORS) {
    const mo = state.motors[m.id];
    const li = document.createElement("li");
    li.className = "motor-item" + (m.id === state.selectedMotorId ? " selected" : "");
    li.dataset.motorId = m.id;

    const modeOptionsHtml = m.type === "TMotor"
      ? `<select data-motor-id="${m.id}" class="mode-override">
           <option value="global" ${mo.modeOverride === "global" ? "selected" : ""}>Global (${state.globalMode.toUpperCase()})</option>
           <option value="servo" ${mo.modeOverride === "servo" ? "selected" : ""}>Servo</option>
           <option value="mit" ${mo.modeOverride === "mit" ? "selected" : ""}>MIT</option>
         </select>`
      : `<select disabled title="Maxon 모드(CSP/CSV/CST)는 이 도구 범위 밖"><option>CSP/CSV/CST</option></select>`;

    li.innerHTML = `
      <div class="motor-item-top">
        <div class="motor-name-wrap">
          <span class="status-dot ${mo.connected ? "on" : ""}"></span>
          <span class="motor-name">${m.label}</span>
          <span class="motor-id">#${m.nodeId}</span>
        </div>
        <span class="type-badge ${m.type.toLowerCase()}">${m.type}</span>
      </div>
      <div class="motor-item-mode">${modeOptionsHtml}</div>
      <div class="motor-mini-values">
        <span>P <b>${mo.last.pos.toFixed(2)}</b></span>
        <span>V <b>${mo.last.vel.toFixed(2)}</b></span>
        <span>T <b>${mo.last.tor.toFixed(2)}</b></span>
      </div>
    `;

    li.addEventListener("click", (e) => {
      if (e.target.tagName === "SELECT") return;
      selectMotor(m.id);
    });

    const select = li.querySelector("select.mode-override");
    if (select) {
      select.addEventListener("click", (e) => e.stopPropagation());
      select.addEventListener("change", (e) => {
        mo.modeOverride = e.target.value;
        if (m.id === state.selectedMotorId) renderMotorHeader();
        sendModeConfig();
      });
    }

    motorListEl.appendChild(li);
  }
}

function updateMotorMiniValues() {
  // 값만 바꿔서 리스트 전체를 다시 그리지 않는다 (선택 상태 등 유지)
  for (const m of MOTORS) {
    const li = motorListEl.querySelector(`li[data-motor-id="${m.id}"]`);
    if (!li) continue;
    const mo = state.motors[m.id];
    const dot = li.querySelector(".status-dot");
    dot.classList.toggle("on", mo.connected);
    const vals = li.querySelectorAll(".motor-mini-values b");
    vals[0].textContent = mo.last.pos.toFixed(2);
    vals[1].textContent = mo.last.vel.toFixed(2);
    vals[2].textContent = mo.last.tor.toFixed(2);
  }
}

function selectMotor(motorId) {
  state.selectedMotorId = motorId;
  renderMotorList();
  renderMotorHeader();
  renderSelectedMotorCharts();
  renderFrameLog();
}

// ---------- View switch (단일 모터 / 전체 보기) ----------

const singleViewEl = document.getElementById("singleView");
const overviewViewEl = document.getElementById("overviewView");
const frameLogHint = document.getElementById("frameLogHint");

function setView(view) {
  state.view = view;
  singleViewEl.hidden = view !== "single";
  overviewViewEl.hidden = view !== "overview";
  document.querySelectorAll(".view-tab").forEach((btn) => {
    btn.classList.toggle("active", btn.dataset.view === view);
  });
  frameLogHint.textContent = view === "single" ? "선택된 모터의 최근 프레임" : "전체 모터의 최근 프레임";
  if (view === "single") renderSelectedMotorCharts();
  else renderOverviewCharts();
  renderFrameLog();
}

document.querySelectorAll(".view-tab").forEach((btn) => {
  btn.addEventListener("click", () => setView(btn.dataset.view));
});

// ---------- Motor header ----------

const motorHeaderEl = document.getElementById("motorHeader");

function renderMotorHeader() {
  const m = state.motors[state.selectedMotorId];
  const mode = effectiveMode(m);
  const modeLabel = mode === "maxon" ? "Maxon (CSP/CSV/CST)" : mode.toUpperCase();
  motorHeaderEl.innerHTML = `
    <div>
      <h2>${m.label} <span class="motor-id">#${m.nodeId}</span></h2>
      <div class="sub">${m.type} &middot; ${m.connected ? "연결됨" : "신호 없음"}</div>
    </div>
    <div class="motor-header-badges">
      <span class="effective-mode-badge ${mode === "maxon" ? "maxon" : ""}">${modeLabel}</span>
    </div>
  `;
}

// ---------- Frame log ----------

const frameLogBody = document.getElementById("frameLogBody");

function renderFrameLog() {
  if (state.dataSource === "file") {
    frameLogHint.textContent = "파일 분석 모드에서는 프레임 목록을 표시하지 않습니다";
    frameLogBody.innerHTML = "";
    return;
  }

  const rows = state.allFrames
    .filter((f) => state.view === "overview" || f.motorId === state.selectedMotorId)
    .slice(0, 20);

  frameLogBody.innerHTML = rows
    .map(
      (f) => `
      <tr>
        <td>${f.time}</td>
        <td>${state.motors[f.motorId].label}</td>
        <td class="dir-${f.dir}">${f.dir.toUpperCase()}</td>
        <td>${f.canId}</td>
        <td>${f.dlc}</td>
        <td>${f.dataHex}</td>
        <td>${f.decoded}</td>
      </tr>`
    )
    .join("");
}

function fmtValue(v, digits) {
  return v === undefined || v === null ? "-" : v.toFixed(digits);
}

function describeDecoded(decoded) {
  let text = `pos=${fmtValue(decoded.pos, 3)} vel=${fmtValue(decoded.vel, 2)} tor=${fmtValue(decoded.tor, 2)}`;
  if (decoded.kp !== undefined) text += ` kp=${fmtValue(decoded.kp, 1)} kd=${fmtValue(decoded.kd, 2)}`;
  if (decoded.packet) text += ` [${decoded.packet}]`;
  return text;
}

function pushFrame(motorId, dir, decoded, raw) {
  const now = new Date();
  const time = now.toTimeString().split(" ")[0] + "." + String(now.getMilliseconds()).padStart(3, "0");
  state.allFrames.unshift({
    motorId,
    time,
    dir,
    canId: raw.id,
    dlc: raw.dlc,
    dataHex: raw.data.map((b) => b.toString(16).padStart(2, "0")).join(" "),
    decoded: describeDecoded(decoded),
  });
  if (state.allFrames.length > 200) state.allFrames.length = 200;
}

// ---------- Common ingestion path (mock + live share this) ----------
// dir "tx" = 명령(제어 프로세스 -> 모터), "rx" = 피드백(모터 -> 제어 프로세스)

function appendPoint(arr, x, y) {
  // 실제 CAN 프레임은 한 프레임에 세 값이 다 들어있지 않다
  // (예: 서보 SET_POS 명령은 위치만). 없는 값은 그냥 건너뛴다.
  if (y === undefined || y === null) return false;
  arr.push({ x, y });
  if (arr.length > MAX_POINTS) arr.shift();
  return true;
}

function ingestFrame(motorId, dir, mode, decoded, raw, ts) {
  const motor = state.motors[motorId];
  if (!motor) return;

  const x = ts !== undefined ? ts : Date.now() / 1000;
  const isCmd = dir === "tx";

  if (decoded.pos !== undefined && decoded.pos !== null) motor.last.pos = decoded.pos;
  if (decoded.vel !== undefined && decoded.vel !== null) motor.last.vel = decoded.vel;
  if (decoded.tor !== undefined && decoded.tor !== null) motor.last.tor = decoded.tor;

  if (x - motor.lastPlotted[dir] >= PLOT_MIN_INTERVAL_SEC) {
    motor.lastPlotted[dir] = x;
    appendPoint(isCmd ? motor.posCmd : motor.posFb, x, decoded.pos);
    appendPoint(isCmd ? motor.velCmd : motor.velFb, x, decoded.vel);
    appendPoint(isCmd ? motor.torCmd : motor.torFb, x, decoded.tor);
  }

  motor.connected = true;

  pushFrame(motorId, dir, decoded, raw);
}

function setMotorConnected(motorId, connected) {
  const motor = state.motors[motorId];
  if (!motor) return;
  motor.connected = connected;
}

// 백엔드는 모터별로 어떤 프로토콜(Servo/MIT)로 디코딩할지 알아야 하므로
// GUI에서 모드가 바뀔 때마다 전체 설정을 보내준다.
function sendModeConfig() {
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN) return;
  const overrides = {};
  for (const m of MOTORS) overrides[m.id] = state.motors[m.id].modeOverride;
  state.ws.send(JSON.stringify({
    type: "control",
    action: "set_modes",
    globalMode: state.globalMode,
    overrides,
  }));

  // 파일 분석 결과는 불러올 때의 모드로 디코딩된 것이라, 모드를 바꾸면 다시 디코딩해야 한다.
  // (WS 는 순서가 보장되므로 백엔드는 set_modes 를 먼저 반영하고 load_log 를 처리한다)
  if (state.dataSource === "file" && state.fileMeta && state.fileMeta.file) {
    fileInfoEl.textContent = "모드 변경 - 다시 디코딩 중...";
    state.ws.send(JSON.stringify({ type: "control", action: "load_log", path: state.fileMeta.file, buckets: 800 }));
  }
}

// ---------- Render loop (데이터 유입 속도와 화면 갱신 속도를 분리) ----------

setInterval(() => {
  // 파일 분석 모드는 데이터가 더 이상 변하지 않으므로 다시 그리지 않는다
  // (수십만 점을 150ms마다 다시 그리면 브라우저가 멈춘다)
  if (state.dataSource === "file") return;
  updateMotorMiniValues();
  if (state.view === "single") {
    renderMotorHeader();
    renderSelectedMotorCharts();
  } else {
    renderOverviewCharts();
  }
  renderFrameLog();
}, 150);

// ---------- Mock data generator ----------

let mockTimer = null;

function startMock() {
  if (mockTimer) return;
  const t0 = performance.now();
  mockTimer = setInterval(() => {
    const t = (performance.now() - t0) / 1000;
    const now = Date.now() / 1000;
    MOTORS.forEach((m, idx) => {
      const freq = 0.15 + idx * 0.03;
      const phase = idx * 0.6;
      const amp = m.type === "TMotor" ? 1.2 : 0.8;

      const cmdPos = amp * Math.sin(freq * t + phase);
      const fbPos = cmdPos - 0.03 * Math.sin(freq * t * 3) + (Math.random() - 0.5) * 0.02;

      const cmdVel = amp * freq * Math.cos(freq * t + phase);
      const fbVel = cmdVel + (Math.random() - 0.5) * 0.15;

      const cmdTor = 0.5 * Math.sign(cmdVel) + (Math.random() - 0.5) * 0.1;
      const fbTor = cmdTor + (Math.random() - 0.5) * 0.2;

      const mode = effectiveMode(state.motors[m.id]);
      const rawData = new Array(8).fill(0).map(() => Math.floor(Math.random() * 256));

      ingestFrame(m.id, "tx", mode, { pos: cmdPos, vel: cmdVel, tor: cmdTor }, {
        id: "0x" + m.nodeId.toString(16).padStart(3, "0"),
        dlc: 8,
        data: rawData,
      }, now);

      ingestFrame(m.id, "rx", mode, { pos: fbPos, vel: fbVel, tor: fbTor }, {
        id: "0x" + (0x100 + m.nodeId).toString(16).padStart(3, "0"),
        dlc: 8,
        data: rawData.slice().reverse(),
      }, now + 0.002);
    });
  }, 100);
}

function stopMock() {
  clearInterval(mockTimer);
  mockTimer = null;
}

// ---------- Live WebSocket (stub, falls back to mock) ----------

const DATA_SOURCE_LABELS = { live: "LIVE", mock: "MOCK DATA", file: "LOG FILE" };

function setDataSourceBadge(mode) {
  const badge = document.getElementById("dataSourceBadge");
  const label = document.getElementById("dataSourceLabel");
  badge.classList.remove("live", "mock", "file");
  badge.classList.add(mode);
  label.textContent = DATA_SOURCE_LABELS[mode] || mode;
  // 목업은 진짜 데이터와 구분이 안 되면 위험하므로 배너로 크게 알린다
  document.getElementById("mockBanner").hidden = mode !== "mock";
}

function handleServerMessage(msg) {
  if (msg.type === "config") {
    applyBackendConfig(msg);
  } else if (msg.type === "frame") {
    // 파일 분석 중에는 라이브 프레임이 그래프를 덮어쓰지 않도록 무시한다
    if (state.dataSource === "file") return;
    ingestFrame(msg.motor, msg.dir, msg.mode, msg.decoded, msg.raw, msg.ts);
  } else if (msg.type === "status") {
    if (state.dataSource !== "file") setMotorConnected(msg.motor, msg.connected);
  } else if (msg.type === "log_status") {
    applyLogStatus(msg);
  } else if (msg.type === "log_list") {
    applyLogList(msg.files);
  } else if (msg.type === "file_series") {
    applyFileSeries(msg);
  } else if (msg.type === "file_error") {
    fileInfoEl.textContent = "오류: " + msg.message;
    logLoadBtn.disabled = false;
  } else if (msg.type === "bus_info") {
    applyBusInfo(msg);
  }
}

// 모터 목록의 정본은 로봇 설정(Phil)이고, 그건 백엔드만 읽을 수 있다.
// 그래서 백엔드가 붙으면 그쪽 목록으로 갈아끼운다.
function applyBackendConfig(msg) {
  if (!Array.isArray(msg.motors) || msg.motors.length === 0) return;

  const sameAsNow =
    msg.motors.length === MOTORS.length &&
    msg.motors.every((m, i) => MOTORS[i].id === m.id && MOTORS[i].nodeId === m.nodeId && MOTORS[i].type === m.type);
  if (sameAsNow) return;

  console.info("모터 설정을 백엔드에서 받아왔습니다:", msg.source);
  initMotorState(msg.motors);
  if (msg.defaultMode) state.globalMode = msg.defaultMode;
  document.getElementById("globalMode").value = state.globalMode;
  // 접속 직후 보낸 set_modes 와 화면이 어긋나지 않도록 초기화된 모드를 다시 알려준다
  sendModeConfig();

  createOverviewCharts();
  renderMotorList();
  renderMotorHeader();
  renderOverviewLegend();
  renderSelectedMotorCharts();
  renderOverviewCharts();
}

// 브링업할 때 "왜 아무것도 안 나오지" 를 바로 알 수 있게 버스 상태를 보여준다
function applyBusInfo(msg) {
  const ifaces = msg.interfaces.length
    ? msg.interfaces.map((i) => `${i.name}(${i.state})`).join(" ")
    : "CAN 인터페이스 없음";
  busInfoEl.textContent = `${ifaces} · ${msg.framesDecoded.toLocaleString()}/${msg.framesSeen.toLocaleString()} 프레임 디코딩`;

  const down = msg.interfaces.filter((i) => i.state === "DOWN" || i.state === "없음");
  if (down.length) {
    busWarnEl.textContent = `${down.map((i) => i.name).join(", ")} 링크 DOWN`;
  } else if (msg.framesSeen && !msg.framesDecoded) {
    const top = msg.unmapped.slice(0, 3).map((u) => u.id).join(", ");
    busWarnEl.textContent = `매핑 안 되는 ID: ${top} — nodeId/모드 확인`;
  } else if (msg.unmapped.length) {
    busWarnEl.textContent = `미매핑 ID ${msg.unmapped.length}종`;
  } else {
    busWarnEl.textContent = "";
  }
}

function connectLive(url) {
  let settled = false;
  try {
    const ws = new WebSocket(url);
    state.ws = ws;

    const timeout = setTimeout(() => {
      if (!settled) {
        ws.close();
        fallbackToMock();
      }
    }, 1500);

    ws.addEventListener("open", () => {
      settled = true;
      clearTimeout(timeout);
      stopMock();
      sendModeConfig();
      ws.send(JSON.stringify({ type: "control", action: "list_logs" }));

      if (state.pendingFile) {
        const path = state.pendingFile;
        state.pendingFile = null;
        fileInfoEl.textContent = "디코딩 중...";
        logLoadBtn.disabled = true;
        ws.send(JSON.stringify({ type: "control", action: "load_log", path, buckets: 800 }));
      }

      if (state.dataSource === "file") return; // 파일 분석 중이면 화면은 그대로 둔다
      state.dataSource = "live";
      setDataSourceBadge("live");
      MOTORS.forEach((m) => setMotorConnected(m.id, false));
    });

    ws.addEventListener("message", (ev) => {
      try {
        const msg = JSON.parse(ev.data);
        if (msg.type === "batch") msg.items.forEach(handleServerMessage);
        else handleServerMessage(msg);
      } catch (err) {
        console.warn("잘못된 메시지 형식:", err);
      }
    });

    ws.addEventListener("close", () => {
      if (state.dataSource === "live") fallbackToMock();
    });
    ws.addEventListener("error", () => {
      if (!settled) {
        settled = true;
        clearTimeout(timeout);
        fallbackToMock();
      }
    });
  } catch (err) {
    fallbackToMock();
  }
}

function fallbackToMock() {
  if (state.dataSource === "file") return; // 파일 분석 결과를 목업으로 덮지 않는다
  state.dataSource = "mock";
  setDataSourceBadge("mock");
  startMock();
}

// ---------- 로그 파일 전체 구간 분석 모드 ----------

const busInfoEl = document.getElementById("busInfo");
const busWarnEl = document.getElementById("busWarn");
const logFileSelect = document.getElementById("logFileSelect");
const logLoadBtn = document.getElementById("logLoadBtn");
const logLiveBtn = document.getElementById("logLiveBtn");
const fileInfoEl = document.getElementById("fileInfo");

function formatBytes(bytes) {
  if (bytes < 1024) return bytes + " B";
  if (bytes < 1024 * 1024) return (bytes / 1024).toFixed(0) + " KB";
  return (bytes / 1024 / 1024).toFixed(1) + " MB";
}

function applyLogList(files) {
  logFileSelect.innerHTML = files.length
    ? files.map((f) => `<option value="${f.path}">${f.name} (${formatBytes(f.size)})</option>`).join("")
    : `<option value="">logs/ 에 로그 파일 없음</option>`;
}

function clearMotorBuffers() {
  for (const m of MOTORS) {
    const mo = state.motors[m.id];
    for (const key of SERIES_KEYS) mo[key].length = 0;
    mo.lastPlotted = { tx: 0, rx: 0 };
    mo.last = { pos: 0, vel: 0, tor: 0 };
    mo.connected = false;
  }
  state.allFrames.length = 0;
}

function applyFileSeries(msg) {
  stopMock();
  state.dataSource = "file";
  state.fileMeta = msg;
  clearMotorBuffers();

  for (const [motorId, series] of Object.entries(msg.series)) {
    const mo = state.motors[motorId];
    if (!mo) continue; // config 에 없는 모터 (설정이 바뀐 로그)
    for (const key of SERIES_KEYS) {
      const points = series[key];
      if (!points) continue;
      for (const [x, y] of points) mo[key].push({ x, y });
      const lastPoint = points[points.length - 1];
      if (key.startsWith("pos")) mo.last.pos = lastPoint[1];
      else if (key.startsWith("vel")) mo.last.vel = lastPoint[1];
      else mo.last.tor = lastPoint[1];
    }
    const stat = msg.stats[motorId];
    mo.connected = Boolean(stat && (stat.tx || stat.rx));
  }

  const duration = msg.tEnd - msg.tStart;
  const started = new Date(msg.tStart * 1000).toLocaleString();
  fileInfoEl.textContent =
    `${msg.file} · ${started} 부터 ${duration.toFixed(1)}초 · ` +
    `${msg.totalLines.toLocaleString()}줄 중 ${msg.decodedFrames.toLocaleString()}프레임 디코딩`;

  setDataSourceBadge("file");
  logLoadBtn.disabled = false;
  renderMotorList();
  renderMotorHeader();
  renderSelectedMotorCharts();
  renderOverviewCharts();
  renderFrameLog();
}

logLoadBtn.addEventListener("click", () => {
  const path = logFileSelect.value;
  if (!path) {
    fileInfoEl.textContent = "불러올 파일이 없습니다";
    return;
  }
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
    fileInfoEl.textContent = "백엔드 미연결 (로그 디코딩은 백엔드가 함)";
    return;
  }
  fileInfoEl.textContent = "디코딩 중...";
  logLoadBtn.disabled = true;
  state.ws.send(JSON.stringify({ type: "control", action: "load_log", path, buckets: 800 }));
});

logLiveBtn.addEventListener("click", () => {
  if (state.dataSource !== "file") return;
  state.fileMeta = null;
  clearMotorBuffers();
  fileInfoEl.textContent = "";
  const live = state.ws && state.ws.readyState === WebSocket.OPEN;
  state.dataSource = live ? "live" : "mock";
  setDataSourceBadge(state.dataSource);
  if (!live) startMock();
  renderMotorList();
  renderMotorHeader();
});

// ---------- Logging (candump -l) control ----------
// 실제 candump 프로세스는 백엔드가 띄운다. 백엔드가 보내주는 log_status 로
// 버튼 상태와 파일명을 실제 상태에 맞춘다.

const loggingBtn = document.getElementById("loggingBtn");
const loggingInfo = document.getElementById("loggingInfo");

function applyLogStatus(msg) {
  state.logging = msg.logging;
  state.loggingFile = msg.file || null;
  loggingBtn.textContent = msg.logging ? "candump 로깅 중지" : "candump 로깅 시작";
  loggingBtn.classList.toggle("active", msg.logging);
  loggingInfo.textContent = msg.logging ? (msg.file || "기록 중") : (msg.error || "중지됨");
}

loggingBtn.addEventListener("click", () => {
  if (!state.ws || state.ws.readyState !== WebSocket.OPEN) {
    loggingInfo.textContent = "백엔드 미연결";
    return;
  }
  const action = state.logging ? "stop_logging" : "start_logging";
  state.ws.send(JSON.stringify({ type: "control", action }));
});

// ---------- Top bar wiring ----------

document.getElementById("globalMode").addEventListener("change", (e) => {
  state.globalMode = e.target.value;
  renderMotorList();
  renderMotorHeader();
  sendModeConfig();
});

document.getElementById("wsRetry").addEventListener("click", () => {
  if (state.ws) {
    state.ws.close();
  }
  stopMock();
  connectLive(document.getElementById("wsUrl").value.trim());
});

// ---------- Init ----------

// #overview            -> 전체 보기로 시작
// #view=overview&file=logs/candump-....log -> 특정 로그를 바로 분석
function parseHash() {
  const raw = location.hash.replace(/^#/, "");
  if (!raw) return {};
  if (!raw.includes("=")) return { view: raw };
  const params = new URLSearchParams(raw);
  return { view: params.get("view"), file: params.get("file") };
}

async function init() {
  const configOk = await loadConfig();
  if (!configOk) {
    fileInfoEl.textContent = "config/motors.json 을 읽지 못했습니다 (http 로 열었는지 확인)";
  }

  document.getElementById("globalMode").value = state.globalMode;

  createOverviewCharts();
  renderMotorList();
  renderMotorHeader();
  renderOverviewLegend();
  renderSelectedMotorCharts();
  renderOverviewCharts();
  renderFrameLog();
  const opts = parseHash();
  if (opts.view === "overview") setView("overview");
  state.pendingFile = opts.file || null;

  setDataSourceBadge("mock");
  connectLive(document.getElementById("wsUrl").value.trim());
}

init();

"use strict";
const $ = id => document.getElementById(id);
let token = "";
let hasROS = false;
let lastTask = "";
let polling = false;
let activeTab = "current";
let pendingDraw = false;
let lastDraw = "";
let workflowAvailable = false;
const labels = {running: "运行中", stopping: "正在停止", succeeded: "成功", failed: "失败", cancelled: "已取消"};
const phases = {checking: "正在检查", preparing: "正在准备线稿", connecting: "正在连接机械臂",
  planning: "正在规划", enabling: "正在准备运动", starting: "正在核对起点", drawing: "正在画", returning_to_idle: "正在返回待机位",
  done: "已画完", stopping: "正在停止", stopped: "已停止", interrupted: "已中断", failed: "失败",
  prepared: "已生成任务", stop_unconfirmed: "停止状态未确认"};
const activePhases = new Set(["checking", "preparing", "connecting", "planning", "enabling", "starting", "drawing", "returning_to_idle", "stopping"]);

function message(text, error = false) {
  $("message").textContent = text;
  $("message").classList.toggle("error", error);
}

async function request(path, data) {
  const options = data === undefined ? {} : {
    method: "POST", headers: {"Content-Type": "application/json", "X-Piper-Token": token},
    body: JSON.stringify(data)
  };
  const response = await fetch(path, options);
  const result = await response.json();
  if (!response.ok) throw new Error(result.error || `HTTP ${response.status}`);
  return result;
}

async function perform(action) {
  try { await action(); } catch (error) { message(error.message, true); }
}

function fileURL(path) { return `/files?path=${encodeURIComponent(path)}`; }

function populate(id, paths, preferred) {
  const select = $(id);
  const previous = preferred || select.value;
  select.replaceChildren();
  paths.forEach(path => {
    const option = document.createElement("option");
    option.value = path;
    option.textContent = path;
    select.append(option);
  });
  if (paths.includes(previous)) select.value = previous;
}

function imagePreview() {
  const path = $("image").value;
  $("input-preview").hidden = !path;
  if (path) $("input-preview").src = fileURL(path);
}

function jobPreview() {
  const path = $("job").value;
  $("drawing-preview").hidden = !path;
  $("job-download").hidden = !path;
  if (path) {
    $("drawing-preview").src = fileURL(path.replace(/_job\.yaml$/, "_preview.svg"));
    $("job-download").href = fileURL(path);
  }
}

async function refreshFiles(preferredImage, newestJob = false) {
  const data = await request("/api/files");
  populate("image", data.images, preferredImage);
  populate("job", data.jobs, newestJob ? data.jobs[0] : undefined);
  populate("config-source", data.configs);
  imagePreview();
  jobPreview();
}

async function refreshConfig() {
  const kind = $("config-kind").value;
  $("config-import").hidden = kind !== "drawing";
  $("config-text").value = (await request(`/api/config?kind=${kind}`)).text;
}

async function selectTab(id) {
  activeTab = id;
  document.querySelectorAll(".tab-panel").forEach(panel => { panel.hidden = panel.id !== id; });
  document.querySelectorAll("[data-tab]").forEach(button => {
    if (button.dataset.tab === id) button.setAttribute("aria-current", "page");
    else button.removeAttribute("aria-current");
  });
  if (id === "history") await refreshHistory();
}

async function refreshHistory() {
  if (!workflowAvailable) {
    $("history-empty").textContent = "重载服务后可查看共用历史。";
    return;
  }
  const records = await request("/api/history");
  const list = $("history-list");
  $("history-empty").hidden = records.length > 0;
  list.replaceChildren();
  for (const record of records) {
    const row = document.createElement("div");
    row.className = "history-item row";
    const checked = document.createElement("input");
    checked.type = "checkbox";
    checked.value = record.id;
    checked.setAttribute("aria-label", `选中 ${record.time}`);
    row.append(checked);
    if (record.preview) {
      const image = document.createElement("img");
      image.src = fileURL(record.preview);
      image.alt = "任务预览";
      image.loading = "lazy";
      row.append(image);
    }
    const title = document.createElement("span");
    title.textContent = `${record.time} · ${record.title || "绘画任务"} · ${phases[record.phase] || "未知"}`;
    row.append(title);
    const source = document.createElement("select");
    source.setAttribute("aria-label", "重画的线稿来源");
    for (const [value, text, available] of [
      ["", "选择重画方式", true], ["line_art", "复用已有线稿", record.line_art],
      ["photo", "从原照片重新生成", record.photo], ["path", "复用已有路径", !record.line_art && record.job]
    ]) {
      if (available) source.add(new Option(text, value));
    }
    const redraw = document.createElement("button");
    redraw.className = "operation ros";
    redraw.textContent = "再画一次";
    redraw.onclick = () => perform(() => startDraw({history: record.id, source: source.value}));
    row.append(source, redraw);
    list.append(row);
  }
}

async function startDraw(input, mode = "real") {
  if (pendingDraw) return;
  if (!input.source) throw new Error("请先选择线稿来源");
  if (!input.image && !input.history) throw new Error("请先选择图片");
  pendingDraw = true;
  try {
    const requestId = crypto.randomUUID?.() || `${Date.now()}-${Math.random()}`;
    await request("/api/action", {operation: "draw", mode, request_id: requestId, ...input});
    await selectTab("current");
    message("已开始处理绘画任务。");
    await refreshState();
  } finally { pendingDraw = false; }
}

function sampleList(kind, samples) {
  const list = $(`${kind}-samples`);
  list.replaceChildren();
  samples.forEach(sample => {
    const item = document.createElement("li");
    item.textContent = `法兰 mm: ${sample.position_mm.map(x => x.toFixed(2)).join(", ")}`;
    list.append(item);
  });
}

async function refreshState() {
  if (polling) return;
  polling = true;
  try {
    const state = await request("/api/state");
    workflowAvailable = Object.hasOwn(state, "drawing");
    token = state.token;
    hasROS = state.ros_available;
    $("environment").textContent = hasROS ? (state.calibrated ? "已标定，可以选图绘画" : "请先完成 TCP 和纸面标定") : "当前环境只能处理图片和标定";
    $("source").options[2].disabled = !hasROS;
    $("config-path").textContent = `当前工作配置：${state.config_path}`;
    $("calibrated").textContent = state.calibrated ? "TCP 与纸面已标定" : "未完成 TCP 或纸面标定";
    $("calibrated").className = `badge ${state.calibrated ? "good" : ""}`;
    const drawing = state.drawing;
    const busy = (state.task && ["running", "stopping"].includes(state.task.status)) ||
      activePhases.has(drawing?.phase);
    document.querySelectorAll(".operation").forEach(button => { button.disabled = !!busy; });
    document.querySelectorAll(".ros").forEach(button => {
      button.disabled = !hasROS || (button.classList.contains("operation") && !!busy);
    });
    $("legacy-run").hidden = workflowAvailable;
    $("config-kind").disabled = !workflowAvailable;
    if (!workflowAvailable) {
      $("draw-start").disabled = true;
      $("simulate").disabled = true;
      $("environment").textContent = "一键流程待重载。高级操作仍可使用。";
    }
    $("stop").disabled = !busy && !state.device_connected && state.backend?.status !== "running";
    $("draw-status").textContent = phases[drawing?.phase] || "空闲";
    const progress = [];
    if (drawing?.message) progress.push(drawing.message);
    if (drawing?.total_strokes) progress.push(
      `已完成 ${drawing.completed_strokes || 0} / ${drawing.total_strokes} 笔`);
    if (drawing?.estimated_seconds && drawing.phase !== "done") progress.push(
      `预计绘画约 ${Math.max(1, Math.ceil(drawing.estimated_seconds / 60))} 分钟`);
    $("draw-progress").textContent = progress.join(" · ");
    $("current-preview").hidden = !drawing?.preview;
    if (drawing?.preview && $("current-preview").getAttribute("src") !== fileURL(drawing.preview)) {
      $("current-preview").src = fileURL(drawing.preview);
    }
    const drawKey = drawing ? drawing.id + drawing.phase : "";
    if (drawKey !== lastDraw && activeTab === "history") await refreshHistory();
    lastDraw = drawKey;
    const tcp = state.samples.tcp;
    const paper = state.samples.paper;
    $("tcp-count").textContent = `${tcp.length} 个样本`;
    $("paper-count").textContent = `${paper.length} / 4 个角点`;
    $("capture-paper").textContent = ["采集左下角", "采集右下角", "采集右上角", "采集左上角"][paper.length] || "已采齐四个角点";
    $("capture-paper").disabled = !!busy || paper.length >= 4;
    $("locate-paper").disabled = !!busy || paper.length !== 4;
    $("park").disabled = !!busy || !state.parking;
    $("parking-status").textContent = state.parking ?
      `待机位 (°)：${state.parking.positions_rad.map(x => (x * 180 / Math.PI).toFixed(2)).join(", ")}` : "尚未保存待机位。";
    sampleList("tcp", tcp);
    sampleList("paper", paper);
    const measurement = state.calibration.tcp;
    $("tcp-result").textContent = measurement ?
      `TCP (mm)：${measurement.tcp_offset_mm.map(x => x.toFixed(3)).join(", ")}；最大残差 ${measurement.max_error_mm.toFixed(3)} mm` : "尚未计算有效 TCP。";
    if (state.calibration.paper) {
      const p = state.calibration.paper.paper;
      $("paper-result").textContent = `已在实测范围内选取 A5 ${p.drawing_area.width_mm} × ${p.drawing_area.height_mm} mm。`;
    } else {
      $("paper-result").textContent = "尚未定位纸面。";
    }
    if (state.device) {
      const d = state.device;
      const feedback = d.after || d;
      const positions = feedback.positions || [];
      $("device-status").textContent = `上次设备操作：${d.operation || "查询"} / ${d.status || "未知"}\n` +
        `时间：${d.finished_at || d.checked_at || "未记录"}\n` +
        `固件：${feedback.firmware || "未知"}；关节角 (°)：${positions.map(x => (x * 180 / Math.PI).toFixed(2)).join(", ")}` +
        (d.error ? `\n错误：${d.error}` : "");
    }
    if (state.task) {
      const task = state.task;
      $("task-status").textContent = `${task.operation} · ${labels[task.status]}`;
      $("task-status").className = `badge ${task.status === "failed" ? "bad" : task.status === "succeeded" ? "good" : ""}`;
      $("task-log").textContent = task.log || "等待命令输出…";
      const key = task.id + task.status;
      if (key !== lastTask && !busy && task.operation !== "draw") {
        await refreshFiles(undefined, task.operation === "prepare" && task.status === "succeeded");
        if (["tcp", "paper", "capture"].includes(task.operation)) await refreshConfig();
        message(`${task.operation}：${labels[task.status]}。${task.status === "failed" ? "具体原因见任务日志。" : ""}`, task.status === "failed");
      }
      lastTask = key;
    }
    if (state.backend) {
      $("backend-status").textContent = `后端：${state.backend.mode} / ${labels[state.backend.status]}（服务就绪情况见日志）`;
      $("backend-log").textContent = state.backend.log || "正在启动…";
    }
  } finally { polling = false; }
}

async function start(operation, extra = {}) {
  const data = Object.assign({operation, mode: $("mode").value}, extra);
  if (operation === "park") {
    if (!confirm("使能并移动到已保存的待机位？请确认移动路径无障碍。")) return;
    data.speed_percent = Number($("speed").value);
  }
  if (operation === "resume" && !confirm("解除软件急停可能导致电机失能和机械臂下落，继续？")) return;
  if (operation === "enable" && !confirm("使能机械臂并保持当前反馈位置？")) return;
  if (operation === "paper") {
    data.orientation = $("orientation").value;
  }
  if (operation === "prepare") {
    data.image = $("image").value;
    data.source = $("image-source").value;
  }
  if (["plan", "run"].includes(operation)) data.job = $("job").value;
  if (operation === "run" && data.mode === "real" && !confirm("将实际驱动机械臂执行所选任务。确认现场已准备好？")) return;
  await request("/api/action", data);
  message(`${operation} 已提交，进度见日志。`);
  await refreshState();
}

document.querySelectorAll("[data-operation]").forEach(button => {
  button.addEventListener("click", () => perform(() => start(button.dataset.operation)));
});
document.querySelectorAll("[data-capture]").forEach(button => {
  button.addEventListener("click", () => perform(() => {
    const source = $("source").value;
    return start("capture", {kind: button.dataset.capture, source: source === "auto" && !workflowAvailable ? "sdk" : source});
  }));
});
document.querySelectorAll("[data-samples]").forEach(button => {
  button.addEventListener("click", () => perform(async () => {
    if (button.dataset.edit === "clear" && !confirm("清空这组采样并撤销相关标定？")) return;
    const result = await request("/api/samples", {kind: button.dataset.samples, action: button.dataset.edit});
    message(result.message);
    await refreshState(); await refreshConfig();
  }));
});
$("read-status").onclick = () => perform(() => start($("source").value === "ros" ? "status" : "doctor"));
$("stop").onclick = () => perform(async () => {
  const result = await request("/api/stop", {source: $("source").value === "ros" ? "ros" : "sdk"});
  message(result.message); await refreshState();
});
$("backend-stop").onclick = () => perform(async () => {
  const result = await request("/api/stop", {source: $("source").value === "ros" ? "ros" : "sdk", backend: true});
  message(result.message); await refreshState();
});
$("refresh-files").onclick = () => perform(() => refreshFiles());
$("reload-config").onclick = () => perform(refreshConfig);
$("load-config").onclick = () => perform(async () => {
  const result = await request("/api/config", {path: $("config-source").value});
  message(result.message); await refreshConfig(); await refreshState();
});
$("save-config").onclick = () => perform(async () => {
  const result = await request("/api/config", {kind: $("config-kind").value, text: $("config-text").value});
  message(result.message); await refreshConfig(); await refreshState();
});
$("config-kind").onchange = () => perform(refreshConfig);
$("image").onchange = imagePreview;
$("job").onchange = jobPreview;
document.querySelectorAll("[data-tab]").forEach(button => {
  button.onclick = () => perform(() => selectTab(button.dataset.tab));
});
$("draw-start").onclick = () => perform(() => startDraw({image: $("image").value, source: $("image-source").value}));
$("simulate").onclick = () => perform(() => startDraw({image: $("image").value, source: $("image-source").value}, "mock"));
$("history-refresh").onclick = () => perform(refreshHistory);
$("history-delete").onclick = () => perform(async () => {
  const ids = Array.from($("history-list").querySelectorAll("input:checked"), input => input.value);
  if (!ids.length) throw new Error("请勾选要删除的历史任务");
  if (!confirm(`删除所选的 ${ids.length} 个历史任务及其生成文件？`)) return;
  await request("/api/history/delete", {ids});
  await refreshHistory();
  await refreshFiles();
  message(`已删除 ${ids.length} 个历史任务。`);
});
$("upload").onchange = () => perform(async () => {
  const file = $("upload").files[0];
  if (!file) return;
  if (file.size > 30 * 1024 * 1024) throw new Error("图片不能超过 30 MB");
  message("正在上传图片…");
  const response = await fetch(`/api/upload?name=${encodeURIComponent(file.name)}`, {
    method: "POST", headers: {"X-Piper-Token": token, "Content-Type": file.type}, body: file
  });
  const result = await response.json();
  if (!response.ok) throw new Error(result.error);
  await refreshFiles(result.path); message("图片已上传。");
  $("upload").value = "";
});

perform(async () => {
  await refreshState(); await refreshFiles(); await refreshConfig();
  message("选择图片和线稿来源即可开始。");
});
setInterval(() => perform(refreshState), 1500);

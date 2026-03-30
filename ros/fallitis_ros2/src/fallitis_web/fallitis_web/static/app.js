const connectionState = document.getElementById("connection-state");
const modeState = document.getElementById("mode-state");
const snapshotState = document.getElementById("snapshot-state");
const poseReadout = document.getElementById("pose-readout");
const mapReadout = document.getElementById("map-readout");
const lidarReadout = document.getElementById("lidar-readout");
const imuReadout = document.getElementById("imu-readout");
const perceptionReadout = document.getElementById("perception-readout");
const actuationReadout = document.getElementById("actuation-readout");
const debugDump = document.getElementById("debug-dump");
const mapCanvas = document.getElementById("map-canvas");
const mapCtx = mapCanvas.getContext("2d");
const lidarCanvas = document.getElementById("lidar-canvas");
const lidarCtx = lidarCanvas.getContext("2d");
const imuCanvas = document.getElementById("imu-canvas");
const imuCtx = imuCanvas.getContext("2d");
const manualModeButton = document.getElementById("mode-manual");
const autonomousModeButton = document.getElementById("mode-autonomous");
const recognitionOffButton = document.getElementById("recognition-off");
const recognitionOnButton = document.getElementById("recognition-on");

const keys = { w: false, a: false, s: false, d: false };
const keyButtons = new Map(
  Array.from(document.querySelectorAll(".key")).map((button) => [button.dataset.key, button]),
);
const overlayControls = {
  rawTrail: document.getElementById("toggle-raw-trail"),
  target: document.getElementById("toggle-target"),
  marker: document.getElementById("toggle-marker"),
  lidarPoints: document.getElementById("toggle-lidar-points"),
};

const ROBOT_MARKER_YAW_OFFSET = Math.PI * 1.5;
const LIDAR_DIRECTIONS = [
  { key: "front", short: "F", x: 0, y: 1 },
  { key: "front_right", short: "FR", x: 0.74, y: 0.74 },
  { key: "right", short: "R", x: 1, y: 0 },
  { key: "rear_right", short: "RR", x: 0.74, y: -0.74 },
  { key: "rear", short: "B", x: 0, y: -1 },
  { key: "rear_left", short: "RL", x: -0.74, y: -0.74 },
  { key: "left", short: "L", x: -1, y: 0 },
  { key: "front_left", short: "FL", x: -0.74, y: 0.74 },
];

let socket;
let mapImage = new Image();
let mapMeta = null;
let pose = null;
let imu = null;
let lidarScan = {};
let mapVersion = -1;
let mappingDebug = {};
let perceptionStatus = {};
let actuationStatus = {};
let requestedMode = "manual";
let requestedRecognitionEnabled = false;
let view = { scale: 1, offsetX: 0, offsetY: 0, fitted: false };
let mapDrag = null;
let lidarView = { scale: 1 };
let imuView = { rotation: 0 };
let imuDrag = null;

function currentMode() {
  const liveMode = String(actuationStatus.mode || requestedMode || "autonomous").toLowerCase();
  return liveMode === "manual" ? "manual" : "autonomous";
}

function updateModeButtons() {
  const mode = currentMode();
  modeState.textContent = mode;
  manualModeButton.classList.toggle("active", mode === "manual");
  autonomousModeButton.classList.toggle("active", mode === "autonomous");
}

function currentRecognitionEnabled() {
  if (typeof perceptionStatus.object_recognition_enabled === "boolean") {
    return perceptionStatus.object_recognition_enabled;
  }
  return requestedRecognitionEnabled;
}

function updateRecognitionButtons() {
  const enabled = currentRecognitionEnabled();
  recognitionOffButton.classList.toggle("active", !enabled);
  recognitionOnButton.classList.toggle("active", enabled);
}

function formatSnapshot(payload) {
  if (!payload) {
    return "None";
  }
  const parts = [];
  if (payload.frames?.left) {
    parts.push(`L ${payload.frames.left.width}x${payload.frames.left.height}`);
  }
  if (payload.frames?.right) {
    parts.push(`R ${payload.frames.right.width}x${payload.frames.right.height}`);
  }
  if (payload.directory) {
    parts.push(payload.directory);
  }
  return parts.length > 0 ? parts.join(" | ") : payload.path || "Snapshot saved";
}

function requestSnapshot() {
  fetch("/api/snapshot", { method: "POST" })
    .then((response) => response.json())
    .then((payload) => {
      snapshotState.textContent = formatSnapshot(payload);
    })
    .catch(() => {
      snapshotState.textContent = "Snapshot failed";
    });
}

function requestLoP() {
  fetch("/api/lop", { method: "POST" }).catch(() => {});
}

function requestMode(mode) {
  requestedMode = mode;
  updateModeButtons();
  fetch("/api/mode", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ value: mode }),
  })
    .then((response) => response.json())
    .then((payload) => {
      if (typeof payload.value === "string") {
        requestedMode = payload.value;
        actuationStatus = { ...actuationStatus, mode: payload.value };
        updateModeButtons();
      }
      if (mode !== "manual") {
        clearMovementKeys();
      }
    })
    .catch(() => {});
}

function requestObjectRecognition(enabled) {
  requestedRecognitionEnabled = Boolean(enabled);
  updateRecognitionButtons();
  fetch("/api/object_recognition", {
    method: "POST",
    headers: { "Content-Type": "application/json" },
    body: JSON.stringify({ value: requestedRecognitionEnabled }),
  })
    .then((response) => response.json())
    .then((payload) => {
      if (typeof payload.value === "boolean") {
        requestedRecognitionEnabled = payload.value;
        perceptionStatus = {
          ...perceptionStatus,
          object_recognition_enabled: payload.value,
        };
        updateRecognitionButtons();
      }
    })
    .catch(() => {});
}

function connect() {
  const scheme = window.location.protocol === "https:" ? "wss" : "ws";
  socket = new WebSocket(`${scheme}://${window.location.host}/ws`);
  connectionState.textContent = "Connecting";

  socket.addEventListener("open", () => {
    connectionState.textContent = "Connected";
  });

  socket.addEventListener("close", () => {
    connectionState.textContent = "Disconnected";
    window.setTimeout(connect, 1000);
  });

  socket.addEventListener("message", (event) => {
    const payload = JSON.parse(event.data);
    if (payload.type === "state") {
      handleState(payload);
      return;
    }
    if (payload.type === "snapshot") {
      snapshotState.textContent = formatSnapshot(payload);
      return;
    }
    if (payload.type === "mode" && typeof payload.value === "string") {
      requestedMode = payload.value;
      actuationStatus = { ...actuationStatus, mode: payload.value };
      updateModeButtons();
      return;
    }
    if (payload.type === "object_recognition" && typeof payload.value === "boolean") {
      requestedRecognitionEnabled = payload.value;
      perceptionStatus = {
        ...perceptionStatus,
        object_recognition_enabled: payload.value,
      };
      updateRecognitionButtons();
    }
  });
}

function sendTeleop() {
  if (!socket || socket.readyState !== WebSocket.OPEN || currentMode() !== "manual") {
    return;
  }
  socket.send(JSON.stringify({ type: "teleop", keys }));
}

function clearMovementKeys() {
  Object.keys(keys).forEach((key) => {
    keys[key] = false;
  });
  updateKeyButtons();
  sendTeleop();
}

function updateKeyButtons() {
  const manual = currentMode() === "manual";
  for (const [key, button] of keyButtons.entries()) {
    if (key === "space" || key === "l") {
      continue;
    }
    button.classList.toggle("active", manual && Boolean(keys[key]));
  }
}

function setKeyState(key, active) {
  if (!(key in keys)) {
    return;
  }
  keys[key] = active;
  updateKeyButtons();
}

function yawFromPose(currentPose) {
  return Math.atan2(
    2 * currentPose.yaw_qw * currentPose.yaw_qz,
    1 - 2 * currentPose.yaw_qz * currentPose.yaw_qz,
  );
}

function formatNumber(value, decimals = 4) {
  return Number.isFinite(value) ? Number(value).toFixed(decimals) : "--";
}

function formatShape(value) {
  if (!Array.isArray(value) || value.length !== 2) {
    return "--";
  }
  return `${value[0]}x${value[1]}`;
}

function formatPoint(value, decimals = 4) {
  if (!Array.isArray(value) || value.length !== 2) {
    return "--";
  }
  return `${formatNumber(value[0], decimals)}, ${formatNumber(value[1], decimals)}`;
}

function formatDirectionalRanges(ranges) {
  return LIDAR_DIRECTIONS.map((direction) => {
    const value = Object.prototype.hasOwnProperty.call(ranges, direction.key) ? ranges[direction.key] : null;
    return `${direction.short} ${formatNumber(value, 5)}`;
  }).join(" | ");
}

function formatAngleRadians(value) {
  return Number.isFinite(value) ? `${formatNumber(value, 6)} rad` : "--";
}

function formatAngleDegrees(value) {
  return Number.isFinite(value) ? `${(Number(value) * 180 / Math.PI).toFixed(4)} deg` : "--";
}

function formatImuReadout() {
  if (!imu) {
    return "waiting";
  }
  return [
    `roll  ${formatAngleRadians(imu.roll)} | ${formatAngleDegrees(imu.roll)}`,
    `pitch ${formatAngleRadians(imu.pitch)} | ${formatAngleDegrees(imu.pitch)}`,
    `yaw   ${formatAngleRadians(imu.yaw)} | ${formatAngleDegrees(imu.yaw)}`,
  ].join("\n");
}

function buildDebugView() {
  const mappingDebugView = { ...mappingDebug };
  delete mappingDebugView.trail_raw;
  delete mappingDebugView.lidar_points;
  return {
    lidarScan,
    mappingDebug: mappingDebugView,
    perceptionStatus,
    actuationStatus,
  };
}

function handleState(payload) {
  mapMeta = payload.map_meta && Object.keys(payload.map_meta).length > 0 ? payload.map_meta : mapMeta;
  pose = payload.pose && Object.keys(payload.pose).length > 0 ? payload.pose : pose;
  imu = payload.imu && Object.keys(payload.imu).length > 0 ? payload.imu : imu;
  lidarScan = payload.lidar_scan && Object.keys(payload.lidar_scan).length > 0 ? payload.lidar_scan : lidarScan;
  mappingDebug = payload.mapping_debug || mappingDebug;
  perceptionStatus = payload.perception_status || perceptionStatus;
  actuationStatus = payload.actuation_status || actuationStatus;
  if (typeof actuationStatus.mode === "string") {
    requestedMode = actuationStatus.mode;
  }
  updateModeButtons();
  if (typeof perceptionStatus.object_recognition_enabled === "boolean") {
    requestedRecognitionEnabled = perceptionStatus.object_recognition_enabled;
  }
  updateRecognitionButtons();
  snapshotState.textContent = payload.last_snapshot || snapshotState.textContent || "None";

  if (pose) {
    poseReadout.textContent = [
      `x ${formatNumber(pose.x, 3)} | y ${formatNumber(pose.y, 3)}`,
      `v ${formatNumber(pose.linear_x, 3)} | w ${formatNumber(pose.angular_z, 3)}`,
    ].join(" | ");
  }

  mapReadout.textContent = [
    mapMeta ? `map ${mapMeta.width}x${mapMeta.height} @ ${formatNumber(mapMeta.resolution, 4)} m/cell` : "map waiting",
    `mode ${currentMode()}`,
    `trail ${mappingDebug.trail_raw?.length || 0}`,
    `target ${formatPoint(mappingDebug.target_world)}`,
    `score ${mappingDebug.game_score ?? perceptionStatus.game_score ?? "--"} | time ${mappingDebug.game_time ?? perceptionStatus.game_time ?? "--"}`,
  ].join("\n");

  lidarReadout.textContent = [
    `front ${formatNumber(perceptionStatus.lidar_front_range, 5)}`,
    `valid pts ${perceptionStatus.lidar_valid_count ?? "--"}`,
    `scan pts ${lidarScan.sample_count ?? "--"}`,
    `range max ${formatNumber(lidarScan.range_max ?? mappingDebug.lidar_range_max_m, 4)}`,
    formatDirectionalRanges(mappingDebug.lidar_directional_ranges || {}),
  ].join("\n");

  imuReadout.textContent = formatImuReadout();

  perceptionReadout.textContent = [
    `left ${formatShape(perceptionStatus.left_camera_shape)}`,
    `right ${formatShape(perceptionStatus.right_camera_shape)}`,
    `score ${perceptionStatus.game_score ?? "--"}`,
    `time ${perceptionStatus.game_time ?? "--"}`,
    `recognition ${currentRecognitionEnabled() ? "on" : "off"} | det ${perceptionStatus.object_recognition_detection_count ?? 0}`,
    `${perceptionStatus.yolo_status || "recognition status --"}`,
  ].join("\n");

  actuationReadout.textContent = [
    `mode ${currentMode()}`,
    `cmd linear ${formatNumber(actuationStatus.linear_x)}`,
    `cmd angular ${formatNumber(actuationStatus.angular_z)}`,
    `autonomous ${actuationStatus.autonomous ?? (currentMode() !== "manual")}`,
  ].join("\n");

  debugDump.textContent = JSON.stringify(buildDebugView(), null, 2);

  if (typeof payload.map_version === "number" && payload.map_version !== mapVersion) {
    mapVersion = payload.map_version;
    mapImage = new Image();
    mapImage.onload = () => {
      fitMapIfNeeded();
      drawMap();
    };
    mapImage.src = `/map/latest.png?v=${mapVersion}`;
  } else {
    drawMap();
  }
  drawLidar();
  drawImu();
}

function fitMapIfNeeded(force = false) {
  if (!mapImage.width || !mapImage.height) {
    return;
  }
  if (!view.fitted || force) {
    const scaleX = mapCanvas.width / mapImage.width;
    const scaleY = mapCanvas.height / mapImage.height;
    view.scale = Math.min(scaleX, scaleY) * 0.95;
    view.offsetX = (mapCanvas.width - mapImage.width * view.scale) / 2;
    view.offsetY = (mapCanvas.height - mapImage.height * view.scale) / 2;
    view.fitted = true;
  }
}

function worldToCanvas(x, y) {
  if (!mapMeta) {
    return null;
  }
  const px = (x - mapMeta.origin_x) / mapMeta.resolution;
  const py = mapMeta.height - (y - mapMeta.origin_y) / mapMeta.resolution;
  return {
    x: view.offsetX + px * view.scale,
    y: view.offsetY + py * view.scale,
  };
}

function drawTrail(points, color, width) {
  if (!Array.isArray(points) || points.length < 2) {
    return;
  }
  mapCtx.beginPath();
  let started = false;
  for (const point of points) {
    if (!Array.isArray(point) || point.length !== 2) {
      continue;
    }
    const canvasPoint = worldToCanvas(point[0], point[1]);
    if (!canvasPoint) {
      continue;
    }
    if (!started) {
      mapCtx.moveTo(canvasPoint.x, canvasPoint.y);
      started = true;
    } else {
      mapCtx.lineTo(canvasPoint.x, canvasPoint.y);
    }
  }
  mapCtx.strokeStyle = color;
  mapCtx.lineWidth = width;
  mapCtx.stroke();
}

function drawTarget() {
  if (!overlayControls.target.checked) {
    return;
  }
  const target = Array.isArray(mappingDebug.target_world) ? mappingDebug.target_world : null;
  if (!target) {
    return;
  }
  const canvasTarget = worldToCanvas(target[0], target[1]);
  if (!canvasTarget) {
    return;
  }
  if (pose) {
    const canvasPose = worldToCanvas(pose.x, pose.y);
    if (canvasPose) {
      mapCtx.setLineDash([8, 8]);
      mapCtx.strokeStyle = "#7cc0ff";
      mapCtx.lineWidth = 1.3;
      mapCtx.beginPath();
      mapCtx.moveTo(canvasPose.x, canvasPose.y);
      mapCtx.lineTo(canvasTarget.x, canvasTarget.y);
      mapCtx.stroke();
      mapCtx.setLineDash([]);
    }
  }
  mapCtx.fillStyle = "#7dff9b";
  mapCtx.beginPath();
  mapCtx.arc(canvasTarget.x, canvasTarget.y, 5, 0, Math.PI * 2);
  mapCtx.fill();
  mapCtx.strokeStyle = "#091018";
  mapCtx.lineWidth = 2;
  mapCtx.stroke();
}

function drawRobotMarker() {
  if (!overlayControls.marker.checked || !mapMeta || !pose || !mapImage.width) {
    return;
  }
  const marker = worldToCanvas(pose.x, pose.y);
  if (!marker) {
    return;
  }
  const yaw = yawFromPose(pose);
  mapCtx.save();
  mapCtx.translate(marker.x, marker.y);
  mapCtx.rotate(yaw + ROBOT_MARKER_YAW_OFFSET);
  mapCtx.scale(1, -1);
  mapCtx.fillStyle = "#ffd166";
  mapCtx.beginPath();
  mapCtx.moveTo(15, 0);
  mapCtx.lineTo(-8, 9);
  mapCtx.lineTo(-3.5, 2);
  mapCtx.lineTo(-10, -1.5);
  mapCtx.lineTo(-7, -9);
  mapCtx.closePath();
  mapCtx.fill();
  mapCtx.strokeStyle = "#11161d";
  mapCtx.lineWidth = 2;
  mapCtx.stroke();
  mapCtx.restore();
}

function drawMap() {
  mapCtx.clearRect(0, 0, mapCanvas.width, mapCanvas.height);
  mapCtx.fillStyle = "#091018";
  mapCtx.fillRect(0, 0, mapCanvas.width, mapCanvas.height);

  if (!mapImage.width) {
    mapCtx.fillStyle = "#9fb0c3";
    mapCtx.font = "14px IBM Plex Mono";
    mapCtx.fillText("Waiting for occupancy map...", 24, 28);
    return;
  }

  mapCtx.drawImage(
    mapImage,
    view.offsetX,
    view.offsetY,
    mapImage.width * view.scale,
    mapImage.height * view.scale,
  );

  if (overlayControls.rawTrail.checked) {
    drawTrail(mappingDebug.trail_raw, "#ff7f7f", 1.3);
  }
  drawTarget();
  drawRobotMarker();
}

function drawLidar() {
  lidarCtx.clearRect(0, 0, lidarCanvas.width, lidarCanvas.height);
  lidarCtx.fillStyle = "#091018";
  lidarCtx.fillRect(0, 0, lidarCanvas.width, lidarCanvas.height);

  const points = overlayControls.lidarPoints.checked ? lidarScan.points || [] : [];
  const directionalRanges = mappingDebug.lidar_directional_ranges || {};
  const centerX = lidarCanvas.width / 2;
  const centerY = lidarCanvas.height / 2;
  const radiusPx = Math.min(lidarCanvas.width, lidarCanvas.height) * 0.38;
  const rangeMax = Math.max(0.05, Number(lidarScan.range_max ?? mappingDebug.lidar_range_max_m) || 0.36);
  const scale = (radiusPx / rangeMax) * lidarView.scale;

  lidarCtx.strokeStyle = "#253241";
  lidarCtx.lineWidth = 1;
  lidarCtx.beginPath();
  lidarCtx.moveTo(centerX, centerY - radiusPx * lidarView.scale);
  lidarCtx.lineTo(centerX, centerY + radiusPx * lidarView.scale);
  lidarCtx.moveTo(centerX - radiusPx * lidarView.scale, centerY);
  lidarCtx.lineTo(centerX + radiusPx * lidarView.scale, centerY);
  lidarCtx.stroke();

  for (const ratio of [0.25, 0.5, 0.75, 1.0]) {
    lidarCtx.beginPath();
    lidarCtx.arc(centerX, centerY, radiusPx * ratio * lidarView.scale, 0, Math.PI * 2);
    lidarCtx.stroke();
  }

  lidarCtx.setLineDash([6, 6]);
  for (const direction of LIDAR_DIRECTIONS) {
    const value = directionalRanges[direction.key];
    const distance = Number.isFinite(value) ? value : rangeMax;
    const endX = centerX + direction.x * distance * scale;
    const endY = centerY - direction.y * distance * scale;
    lidarCtx.strokeStyle = Number.isFinite(value) ? "#4ea8de" : "#405364";
    lidarCtx.beginPath();
    lidarCtx.moveTo(centerX, centerY);
    lidarCtx.lineTo(endX, endY);
    lidarCtx.stroke();
  }
  lidarCtx.setLineDash([]);

  lidarCtx.fillStyle = "#7cc0ff";
  for (const point of points) {
    if (!Array.isArray(point) || point.length !== 2) {
      continue;
    }
    const x = centerX - point[1] * scale;
    const y = centerY - point[0] * scale;
    lidarCtx.fillRect(x - 1, y - 1, 3, 3);
  }

  lidarCtx.font = "12px IBM Plex Mono";
  lidarCtx.fillStyle = "#cfd9e3";
  lidarCtx.textAlign = "center";
  lidarCtx.textBaseline = "middle";
  for (const direction of LIDAR_DIRECTIONS) {
    const value = directionalRanges[direction.key];
    const labelX = lidarCanvas.width / 2 + direction.x * (radiusPx + 24);
    const labelY = lidarCanvas.height / 2 - direction.y * (radiusPx + 20);
    lidarCtx.fillText(`${direction.short} ${formatNumber(value, 5)}`, labelX, labelY);
  }

  lidarCtx.fillStyle = "#ffd166";
  lidarCtx.beginPath();
  lidarCtx.arc(centerX, centerY, 5, 0, Math.PI * 2);
  lidarCtx.fill();
}

function drawImu() {
  imuCtx.clearRect(0, 0, imuCanvas.width, imuCanvas.height);
  imuCtx.fillStyle = "#091018";
  imuCtx.fillRect(0, 0, imuCanvas.width, imuCanvas.height);

  const centerX = imuCanvas.width / 2;
  const centerY = imuCanvas.height / 2;
  const radius = Math.min(imuCanvas.width, imuCanvas.height) * 0.32;

  imuCtx.strokeStyle = "#253241";
  imuCtx.lineWidth = 1;
  imuCtx.beginPath();
  imuCtx.arc(centerX, centerY, radius, 0, Math.PI * 2);
  imuCtx.stroke();

  if (!imu) {
    imuCtx.fillStyle = "#9fb0c3";
    imuCtx.font = "14px IBM Plex Mono";
    imuCtx.textAlign = "center";
    imuCtx.fillText("Waiting for IMU...", centerX, centerY);
    return;
  }

  const topRotation = (imu.yaw || 0) + imuView.rotation;
  const bodyAxisForward = [Math.cos(topRotation), -Math.sin(topRotation)];
  const bodyAxisLeft = [Math.sin(topRotation), Math.cos(topRotation)];
  const tiltForward = Math.max(-1, Math.min(1, (imu.pitch || 0) / 0.7));
  const tiltLeft = Math.max(-1, Math.min(1, (imu.roll || 0) / 0.7));
  const tiltX = centerX + (bodyAxisForward[0] * tiltForward + bodyAxisLeft[0] * tiltLeft) * radius * 0.45;
  const tiltY = centerY + (bodyAxisForward[1] * tiltForward + bodyAxisLeft[1] * tiltLeft) * radius * 0.45;

  imuCtx.strokeStyle = "#253241";
  imuCtx.lineWidth = 1;
  imuCtx.beginPath();
  imuCtx.moveTo(centerX - radius, centerY);
  imuCtx.lineTo(centerX + radius, centerY);
  imuCtx.moveTo(centerX, centerY - radius);
  imuCtx.lineTo(centerX, centerY + radius);
  imuCtx.stroke();

  imuCtx.save();
  imuCtx.translate(centerX, centerY);
  imuCtx.rotate(-topRotation);
  imuCtx.strokeStyle = "#7cc0ff";
  imuCtx.lineWidth = 2;
  imuCtx.beginPath();
  imuCtx.moveTo(0, -radius * 0.70);
  imuCtx.lineTo(radius * 0.26, radius * 0.34);
  imuCtx.lineTo(0, radius * 0.16);
  imuCtx.lineTo(-radius * 0.26, radius * 0.34);
  imuCtx.closePath();
  imuCtx.stroke();
  imuCtx.restore();

  imuCtx.strokeStyle = "#9fe870";
  imuCtx.lineWidth = 2;
  imuCtx.beginPath();
  imuCtx.moveTo(centerX, centerY);
  imuCtx.lineTo(centerX + bodyAxisForward[0] * radius * 0.8, centerY + bodyAxisForward[1] * radius * 0.8);
  imuCtx.stroke();

  imuCtx.fillStyle = "#ffb86b";
  imuCtx.beginPath();
  imuCtx.arc(tiltX, tiltY, 5, 0, Math.PI * 2);
  imuCtx.fill();

  imuCtx.fillStyle = "#ffd166";
  imuCtx.beginPath();
  imuCtx.arc(centerX, centerY, 4, 0, Math.PI * 2);
  imuCtx.fill();

  imuCtx.font = "11px IBM Plex Mono";
  imuCtx.textAlign = "left";
  imuCtx.textBaseline = "top";
  imuCtx.fillStyle = "#cfd9e3";
  imuCtx.fillText(`R ${formatNumber(imu.roll, 6)}`, 10, 10);
  imuCtx.fillText(`P ${formatNumber(imu.pitch, 6)}`, 10, 24);
  imuCtx.fillText(`Y ${formatNumber(imu.yaw, 6)}`, 10, 38);
  imuCtx.fillText(`view ${formatNumber(imuView.rotation, 5)}`, 10, 52);
}

function zoomLidar(factor) {
  lidarView.scale = Math.min(12, Math.max(0.35, lidarView.scale * factor));
  drawLidar();
}

function zoom(factor, centerX = mapCanvas.width / 2, centerY = mapCanvas.height / 2) {
  const oldScale = view.scale;
  view.scale = Math.min(40, Math.max(0.05, view.scale * factor));
  const ratio = view.scale / oldScale;
  view.offsetX = centerX - (centerX - view.offsetX) * ratio;
  view.offsetY = centerY - (centerY - view.offsetY) * ratio;
  drawMap();
}

mapCanvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const factor = event.deltaY < 0 ? 1.12 : 0.89;
  const rect = mapCanvas.getBoundingClientRect();
  zoom(factor, event.clientX - rect.left, event.clientY - rect.top);
});

mapCanvas.addEventListener("pointerdown", (event) => {
  mapDrag = { x: event.clientX, y: event.clientY };
  mapCanvas.classList.add("dragging");
});

lidarCanvas.addEventListener("wheel", (event) => {
  event.preventDefault();
  const factor = event.deltaY < 0 ? 1.12 : 0.89;
  zoomLidar(factor);
});

imuCanvas.addEventListener("pointerdown", (event) => {
  imuDrag = { x: event.clientX, rotation: imuView.rotation };
  imuCanvas.classList.add("dragging");
});

window.addEventListener("pointermove", (event) => {
  if (mapDrag) {
    view.offsetX += event.clientX - mapDrag.x;
    view.offsetY += event.clientY - mapDrag.y;
    mapDrag = { x: event.clientX, y: event.clientY };
    drawMap();
  }
  if (!imuDrag) {
    return;
  }
  imuView.rotation = imuDrag.rotation + (event.clientX - imuDrag.x) * 0.01;
  drawImu();
});

window.addEventListener("pointerup", () => {
  mapDrag = null;
  imuDrag = null;
  mapCanvas.classList.remove("dragging");
  imuCanvas.classList.remove("dragging");
});

document.getElementById("zoom-in").addEventListener("click", () => zoom(1.18));
document.getElementById("zoom-out").addEventListener("click", () => zoom(0.84));
document.getElementById("zoom-reset").addEventListener("click", () => {
  fitMapIfNeeded(true);
  drawMap();
});
document.getElementById("lidar-zoom-in").addEventListener("click", () => zoomLidar(1.18));
document.getElementById("lidar-zoom-out").addEventListener("click", () => zoomLidar(0.84));
document.getElementById("lidar-zoom-reset").addEventListener("click", () => {
  lidarView = { scale: 1 };
  drawLidar();
});
document.getElementById("imu-view-reset").addEventListener("click", () => {
  imuView = { rotation: 0 };
  drawImu();
});

manualModeButton.addEventListener("click", () => requestMode("manual"));
autonomousModeButton.addEventListener("click", () => requestMode("autonomous"));
recognitionOffButton.addEventListener("click", () => requestObjectRecognition(false));
recognitionOnButton.addEventListener("click", () => requestObjectRecognition(true));

for (const control of Object.values(overlayControls)) {
  control.addEventListener("change", () => {
    drawMap();
    drawLidar();
  });
}

window.addEventListener("keydown", (event) => {
  const key = event.key.toLowerCase();
  if (["w", "a", "s", "d", "l", " "].includes(key)) {
    event.preventDefault();
  }
  if (event.repeat) {
    return;
  }
  if (key === " ") {
    keyButtons.get("space")?.classList.add("active");
    requestSnapshot();
    return;
  }
  if (key === "l") {
    keyButtons.get("l")?.classList.add("active");
    requestLoP();
    return;
  }
  if (currentMode() !== "manual") {
    return;
  }
  setKeyState(key, true);
  sendTeleop();
});

window.addEventListener("keyup", (event) => {
  const key = event.key.toLowerCase();
  if (key === " ") {
    keyButtons.get("space")?.classList.remove("active");
    return;
  }
  if (key === "l") {
    keyButtons.get("l")?.classList.remove("active");
    return;
  }
  setKeyState(key, false);
  sendTeleop();
});

window.addEventListener("blur", () => {
  clearMovementKeys();
});

for (const [key, button] of keyButtons.entries()) {
  if (key === "space") {
    button.addEventListener("click", requestSnapshot);
    continue;
  }
  if (key === "l") {
    button.addEventListener("click", () => {
      button.classList.add("active");
      requestLoP();
      window.setTimeout(() => button.classList.remove("active"), 140);
    });
    continue;
  }
  button.addEventListener("pointerdown", () => {
    if (currentMode() !== "manual") {
      return;
    }
    setKeyState(key, true);
    sendTeleop();
  });
  button.addEventListener("pointerup", () => {
    setKeyState(key, false);
    sendTeleop();
  });
  button.addEventListener("pointerleave", () => {
    setKeyState(key, false);
    sendTeleop();
  });
}

window.setInterval(sendTeleop, 100);
updateModeButtons();
updateRecognitionButtons();
connect();
drawMap();
drawLidar();
drawImu();

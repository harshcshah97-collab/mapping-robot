"use strict";

const state = {
  ros: null,
  viewer: null,
  gridClient: null,
  roomOverlay: null,
  commandTopic: null,
  assistantQueryTopic: null,
  mode: "unknown",
  activeMap: null,
  rooms: [],
  interaction: null,
  dragStart: null,
  roomPoints: [],
  tracking: null,
  movingTimer: null,
  toastTimer: null,
  pendingQuestions: new Map(),
};

const element = (id) => document.getElementById(id);

function showToast(message, isError = false) {
  const toast = element("toast");
  toast.textContent = message;
  toast.classList.toggle("error", isError);
  toast.classList.add("show");
  clearTimeout(state.toastTimer);
  state.toastTimer = setTimeout(() => toast.classList.remove("show"), 4200);
}

async function api(path, options = {}) {
  const response = await fetch(path, {
    ...options,
    headers: {"Content-Type": "application/json", ...(options.headers || {})},
  });
  let payload;
  try { payload = await response.json(); } catch (_error) { payload = {}; }
  if (!response.ok || payload.status === "error") {
    throw new Error(payload.message || `Request failed (${response.status})`);
  }
  return payload;
}

function openPanel(name) {
  document.querySelectorAll(".panel").forEach((panel) => panel.classList.toggle("active", panel.id === `panel-${name}`));
  document.querySelectorAll(".bottom-nav [data-panel]").forEach((button) => button.classList.toggle("active", button.dataset.panel === name));
  if (name === "map") setTimeout(() => state.viewer && state.viewer.scene.update(), 50);
  window.scrollTo({top: 0, behavior: "smooth"});
}

function hotOverride() { return Boolean(element("hot-override")?.checked); }

async function modeAction(action) {
  const payload = (action === "start_tracking" || action === "start_follow") ? {allow_hot_operation: hotOverride()} : {};
  try {
    const result = await api(`/api/${action}`, {method: "POST", body: JSON.stringify(payload)});
    if (result.mode) state.mode = result.mode;
    updateModeDisplay();
    showToast(result.message || "Command accepted.");
    if (action === "start_tracking" || action === "start_follow") openPanel("vision");
    if (action === "start_mapping" || action === "start_navigation") openPanel("map");
    if (action === "start_teleop") openPanel("drive");
    setTimeout(refreshStatus, 900);
  } catch (error) { showToast(error.message, true); }
}

async function trackingCommand(command, startIfNeeded = false) {
  try {
    const result = await api("/api/tracking_command", {
      method: "POST",
      body: JSON.stringify({command, start_if_needed: startIfNeeded, allow_hot_operation: hotOverride()}),
    });
    showToast(result.message || "Tracking command queued.");
    if (startIfNeeded) { state.mode = "tracking"; updateModeDisplay(); }
  } catch (error) { showToast(error.message, true); }
}

function updateModeDisplay() {
  const pill = element("mode-pill");
  pill.textContent = state.mode || "unknown";
  pill.className = `pill ${state.mode === "stopped" ? "neutral" : "good"}`;
}

async function refreshStatus() {
  try {
    const data = await api("/api/status");
    state.mode = data.mode;
    state.activeMap = data.map;
    updateModeDisplay();
    const tempPill = element("temp-pill");
    tempPill.textContent = data.pi_temp_c == null ? "-- °C" : `${data.pi_temp_c.toFixed(1)} °C`;
    tempPill.className = `pill ${data.pi_temp_c >= 80 ? "bad" : data.pi_temp_c >= 70 ? "warning" : "good"}`;
    element("system-status").textContent = JSON.stringify({mode: data.mode, map: data.map, rooms: data.room_count, temperature_c: data.pi_temp_c, throttled: data.throttled, operations: data.operations}, null, 2);
    if (!state.tracking) {
      element("identity-dot").classList.toggle("good", Boolean(data.subject_enrolled));
      element("identity-summary").textContent = data.subject_enrolled ? "An appearance profile is stored. Start Vision to verify and follow it." : "No profile is stored. Follow and framing are blocked until you enroll.";
    }
    if (state.activeMap && element("map-select").value !== state.activeMap) {
      element("map-select").value = state.activeMap;
      await loadRooms();
    }
  } catch (error) {
    element("system-status").textContent = `App server unavailable: ${error.message}`;
  }
}

function currentRosTime() {
  const milliseconds = Date.now();
  return {sec: Math.floor(milliseconds / 1000), nanosec: (milliseconds % 1000) * 1000000};
}

function setConnectionStatus(text, style) {
  const pill = element("connection-pill");
  pill.textContent = text;
  pill.className = `pill ${style}`;
}

function connectRos() {
  if (typeof ROSLIB === "undefined") {
    setConnectionStatus("ROS library unavailable", "bad");
    showToast("The ROS browser libraries could not load. Check internet access or vendor the CDN files.", true);
    return;
  }
  const host = element("robot-host").value.trim() || window.location.hostname || "127.0.0.1";
  if (state.ros) state.ros.close();
  setConnectionStatus("connecting…", "warning");
  const protocol = window.location.protocol === "https:" ? "wss" : "ws";
  state.ros = new ROSLIB.Ros({url: `${protocol}://${host}:9090`});
  state.ros.on("connection", () => { setConnectionStatus("ROS connected", "good"); setupRosInterfaces(); });
  state.ros.on("error", () => setConnectionStatus("ROS error", "bad"));
  state.ros.on("close", () => setConnectionStatus("ROS offline", "neutral"));
}

function setupRosInterfaces() {
  state.commandTopic = new ROSLIB.Topic({ros: state.ros, name: "/cmd_vel", messageType: "geometry_msgs/msg/Twist"});
  state.assistantQueryTopic = new ROSLIB.Topic({ros: state.ros, name: "/assistant/text_query", messageType: "std_msgs/msg/String"});
  new ROSLIB.Topic({ros: state.ros, name: "/assistant/text_response", messageType: "std_msgs/msg/String"}).subscribe(handleAssistantResponse);
  new ROSLIB.Topic({ros: state.ros, name: "/tracking/status", messageType: "std_msgs/msg/String"}).subscribe((message) => {
    try { renderTrackingStatus(JSON.parse(message.data)); } catch (_error) { element("tracking-event").textContent = message.data; }
  });
  new ROSLIB.Topic({ros: state.ros, name: "/tracking/event", messageType: "std_msgs/msg/String"}).subscribe((message) => { element("tracking-event").textContent = message.data; });
  setupMap();
}

function setupMap() {
  if (typeof ROS2D === "undefined" || typeof createjs === "undefined") return;
  element("map").replaceChildren();
  const width = Math.max(300, Math.min(850, element("map-shell").clientWidth - 24));
  const height = Math.max(340, Math.min(560, Math.round(width * 0.68)));
  state.viewer = new ROS2D.Viewer({divID: "map", width, height});
  state.roomOverlay = new createjs.Container();
  state.viewer.scene.addChild(state.roomOverlay);
  attachMapInteraction();
  state.gridClient = new ROS2D.OccupancyGridClient({ros: state.ros, rootObject: state.viewer.scene, continuous: true});
  state.viewer.scene.setChildIndex(state.roomOverlay, state.viewer.scene.numChildren - 1);
  state.gridClient.on("change", () => {
    const grid = state.gridClient.currentGrid;
    state.viewer.scaleToDimensions(grid.width, grid.height);
    state.viewer.shift(grid.pose.position.x, grid.pose.position.y);
    state.viewer.scene.setChildIndex(
      state.roomOverlay,
      state.viewer.scene.numChildren - 1
    );
    element("map-empty").style.display = "none";
    drawRoomOverlay();
  });
}

function setMapInteraction(mode) {
  if ((mode === "pose" || mode === "goal") && state.mode !== "navigation") {
    showToast("Start Navigation before setting a pose or goal.", true); return;
  }
  if (mode === "room" && !element("map-select").value) {
    showToast("Select a saved map before drawing a room.", true); return;
  }
  state.interaction = mode;
  state.dragStart = null;
  const messages = {pose: "Drag from the robot position toward its forward direction.", goal: "Drag from the destination toward the desired arrival direction.", room: "Tap each room corner, then press Save room."};
  element("map-hint").textContent = messages[mode] || "Drag on the map to set the robot pose or a navigation goal.";
  element("map-hint").classList.toggle("active", Boolean(mode));
}

function attachMapInteraction() {
  state.viewer.scene.addEventListener("stagemousedown", (event) => {
    if (!state.interaction) return;
    const point = state.viewer.scene.globalToRos(event.stageX, event.stageY);
    if (state.interaction === "room") {
      state.roomPoints.push({x: Number(point.x.toFixed(4)), y: Number(point.y.toFixed(4))});
      element("save-room").disabled = state.roomPoints.length < 3;
      drawRoomOverlay();
      return;
    }
    state.dragStart = point;
  });
  state.viewer.scene.addEventListener("stagemouseup", async (event) => {
    if (!state.dragStart || !["pose", "goal"].includes(state.interaction)) return;
    const end = state.viewer.scene.globalToRos(event.stageX, event.stageY);
    const yaw = Math.atan2(end.y - state.dragStart.y, end.x - state.dragStart.x);
    if (state.interaction === "pose") publishInitialPose(state.dragStart, yaw);
    else await sendNavigationGoal(state.dragStart, yaw);
    setMapInteraction(null);
  });
}

function publishInitialPose(point, yaw) {
  if (!state.ros?.isConnected) { showToast("ROS is not connected.", true); return; }
  const topic = new ROSLIB.Topic({ros: state.ros, name: "/initialpose", messageType: "geometry_msgs/msg/PoseWithCovarianceStamped"});
  const covariance = Array(36).fill(0); covariance[0] = 0.25; covariance[7] = 0.25; covariance[35] = 0.068;
  topic.publish(new ROSLIB.Message({header: {stamp: currentRosTime(), frame_id: "map"}, pose: {pose: {position: {x: point.x, y: point.y, z: 0}, orientation: {x: 0, y: 0, z: Math.sin(yaw / 2), w: Math.cos(yaw / 2)}}, covariance}}));
  showToast("Initial pose published.");
}

async function sendNavigationGoal(point, yaw) {
  try {
    const result = await api("/api/navigation_goal", {method: "POST", body: JSON.stringify({x: point.x, y: point.y, yaw})});
    showToast(result.message);
  } catch (error) { showToast(error.message, true); }
}

function polygonCentroid(points) {
  let twiceArea = 0, x = 0, y = 0;
  points.forEach((point, index) => {
    const next = points[(index + 1) % points.length];
    const cross = point.x * next.y - next.x * point.y;
    twiceArea += cross; x += (point.x + next.x) * cross; y += (point.y + next.y) * cross;
  });
  if (Math.abs(twiceArea) < 1e-8) return {x: points.reduce((sum, p) => sum + p.x, 0) / points.length, y: points.reduce((sum, p) => sum + p.y, 0) / points.length};
  return {x: x / (3 * twiceArea), y: y / (3 * twiceArea)};
}

function drawPolygon(graphics, points, color, fill = true) {
  if (!points.length) return;
  const scale = Math.max(Math.abs(state.viewer.scene.scaleX || 1), 1);
  graphics.setStrokeStyle(2 / scale).beginStroke(color);
  if (fill) graphics.beginFill(color);
  graphics.moveTo(points[0].x, points[0].y);
  points.slice(1).forEach((point) => graphics.lineTo(point.x, point.y));
  if (points.length > 2) graphics.closePath();
  if (fill) graphics.endFill();
  graphics.endStroke();
}

function drawRoomOverlay() {
  if (!state.roomOverlay || !state.viewer) return;
  state.roomOverlay.removeAllChildren();
  const colors = ["#2da58b", "#4c82c5", "#a66fd1", "#bf863c", "#bd5660"];
  state.rooms.forEach((room, index) => {
    const shape = new createjs.Shape(); shape.alpha = 0.23;
    drawPolygon(shape.graphics, room.polygon, colors[index % colors.length]);
    state.roomOverlay.addChild(shape);
    const goal = new createjs.Shape();
    const radius = 5 / Math.max(Math.abs(state.viewer.scene.scaleX || 1), 1);
    goal.graphics.beginFill(colors[index % colors.length]).drawCircle(room.goal.x, room.goal.y, radius);
    state.roomOverlay.addChild(goal);
  });
  if (state.roomPoints.length) {
    const draft = new createjs.Shape(); draft.alpha = 0.8;
    drawPolygon(draft.graphics, state.roomPoints, "#f4bd68", false);
    const radius = 4 / Math.max(Math.abs(state.viewer.scene.scaleX || 1), 1);
    state.roomPoints.forEach((point) => draft.graphics.beginFill("#f4bd68").drawCircle(point.x, point.y, radius));
    state.roomOverlay.addChild(draft);
  }
  state.viewer.scene.update();
}

function clearRoomDraft() {
  state.roomPoints = []; element("save-room").disabled = true; setMapInteraction(null); drawRoomOverlay();
}

async function saveRoom() {
  const name = element("room-name").value.trim();
  const selectedMap = element("map-select").value;
  if (state.roomPoints.length < 3) { showToast("Tap at least three room corners.", true); return; }
  const center = polygonCentroid(state.roomPoints);
  try {
    const result = await api("/api/rooms", {method: "POST", body: JSON.stringify({name, map: selectedMap, polygon: state.roomPoints, goal: {...center, yaw: Number(element("room-yaw").value)}})});
    showToast(`${result.room.name} saved.`); element("room-name").value = ""; clearRoomDraft(); await loadRooms();
  } catch (error) { showToast(error.message, true); }
}

async function loadMaps() {
  try {
    const data = await api("/api/maps");
    const select = element("map-select"); select.replaceChildren();
    if (!data.maps.length) select.append(new Option("No saved maps found", ""));
    data.maps.forEach((name) => select.append(new Option(name.replace(/\.yaml$/, ""), name)));
    if (state.activeMap && data.maps.includes(state.activeMap)) select.value = state.activeMap;
    await loadRooms();
  } catch (error) { showToast(error.message, true); }
}

async function loadRooms() {
  const mapName = element("map-select").value;
  if (!mapName) { state.rooms = []; renderRoomList(); drawRoomOverlay(); return; }
  try { state.rooms = (await api(`/api/rooms?map=${encodeURIComponent(mapName)}`)).rooms; renderRoomList(); drawRoomOverlay(); }
  catch (error) { showToast(error.message, true); }
}

function renderRoomList() {
  const list = element("room-list"); list.replaceChildren();
  if (!state.rooms.length) { const empty = document.createElement("p"); empty.className = "muted"; empty.textContent = "No rooms on this map yet."; list.append(empty); return; }
  state.rooms.forEach((room) => {
    const item = document.createElement("div"); item.className = "room-item";
    const name = document.createElement("span"); name.textContent = room.name;
    const go = document.createElement("button"); go.className = "primary"; go.textContent = "Go"; go.addEventListener("click", () => navigateRoom(room));
    const remove = document.createElement("button"); remove.textContent = "Delete"; remove.addEventListener("click", () => deleteRoom(room));
    item.append(name, go, remove); list.append(item);
  });
}

async function navigateRoom(room) {
  try { const result = await api("/api/navigate_room", {method: "POST", body: JSON.stringify({room: room.name, map: room.map})}); state.mode = "navigation"; updateModeDisplay(); showToast(result.message); }
  catch (error) { showToast(error.message, true); }
}

async function deleteRoom(room) {
  if (!window.confirm(`Delete ${room.name}?`)) return;
  try { await api(`/api/rooms/${encodeURIComponent(room.name)}?map=${encodeURIComponent(room.map)}`, {method: "DELETE"}); await loadRooms(); showToast("Room deleted."); }
  catch (error) { showToast(error.message, true); }
}

function renderTrackingStatus(status) {
  state.tracking = status;
  element("track-mode").textContent = status.mode || "unknown";
  element("track-enrolled").textContent = status.subject_enrolled ? "enrolled" : "not enrolled";
  element("track-target").textContent = status.target_visible ? `visible${status.target_id != null ? ` · ID ${status.target_id}` : ""}` : "not visible";
  element("track-distance").textContent = Number.isFinite(status.target_distance_m) ? `${status.target_distance_m.toFixed(2)} m` : "--";
  element("track-block").textContent = status.motion_blocked_by || "clear";
  element("track-camera").textContent = status.vision_runtime || "streaming";
  element("identity-dot").classList.toggle("good", Boolean(status.subject_enrolled));
  element("identity-summary").textContent = status.subject_enrolled ? "An appearance profile is stored. Bob will reject people who do not match it." : "No profile is stored. Follow and framing are blocked until you enroll.";
}

function appendMessage(text, role) {
  const message = document.createElement("div"); message.className = `message ${role}`; message.textContent = text;
  element("chat-log").append(message); element("chat-log").scrollTop = element("chat-log").scrollHeight;
}

function askBob(text) {
  if (!state.assistantQueryTopic || !state.ros?.isConnected) { showToast("Connect ROS before asking Bob.", true); return; }
  const id = `${Date.now()}-${Math.random().toString(16).slice(2)}`;
  appendMessage(text, "user"); appendMessage("Thinking…", "bob");
  const waiting = element("chat-log").lastElementChild; state.pendingQuestions.set(id, waiting);
  state.assistantQueryTopic.publish(new ROSLIB.Message({data: JSON.stringify({id, text})}));
  setTimeout(() => {
    if (!state.pendingQuestions.has(id)) return;
    waiting.textContent = "Bob did not answer in time. Check the assistant service and internet connection.";
    state.pendingQuestions.delete(id);
  }, 45000);
}

function handleAssistantResponse(message) {
  try {
    const response = JSON.parse(message.data); const waiting = state.pendingQuestions.get(response.id);
    if (waiting) waiting.textContent = response.text; else appendMessage(response.text, "bob");
    state.pendingQuestions.delete(response.id);
  } catch (_error) { appendMessage(message.data, "bob"); }
}

function zeroTwist() { return new ROSLIB.Message({linear: {x: 0, y: 0, z: 0}, angular: {x: 0, y: 0, z: 0}}); }
function stopMoving() {
  clearInterval(state.movingTimer); state.movingTimer = null;
  if (state.commandTopic && state.ros?.isConnected) state.commandTopic.publish(zeroTwist());
}
function startMoving(linear, angular) {
  if (state.mode !== "teleop") { showToast("Start Teleop before using the drive pad.", true); return; }
  if (!state.commandTopic || !state.ros?.isConnected) { showToast("ROS is not connected.", true); return; }
  stopMoving();
  const command = new ROSLIB.Message({linear: {x: linear, y: 0, z: 0}, angular: {x: 0, y: 0, z: angular}});
  state.commandTopic.publish(command); state.movingTimer = setInterval(() => state.commandTopic.publish(command), 100);
}

function bindDriveButton(id, velocity) {
  const button = element(id);
  button.addEventListener("pointerdown", (event) => { event.preventDefault(); button.setPointerCapture(event.pointerId); velocity(); });
  ["pointerup", "pointercancel", "lostpointercapture", "pointerleave"].forEach((name) => button.addEventListener(name, stopMoving));
}

async function emergencyStop() {
  stopMoving();
  try { const result = await api("/api/stop", {method: "POST", body: "{}"}); state.mode = "stopped"; updateModeDisplay(); showToast(result.message); }
  catch (error) { showToast(`Stop request failed: ${error.message}`, true); }
}

async function scanWifi() {
  const select = element("wifi-ssid"); select.innerHTML = "<option>Scanning…</option>";
  try { const data = await api("/api/wifi_scan"); select.replaceChildren(new Option("Select a network…", "")); data.networks.forEach((network) => select.append(new Option(network, network))); }
  catch (error) { select.innerHTML = "<option>Scan failed</option>"; showToast(error.message, true); }
}

async function connectWifi() {
  try { const result = await api("/api/wifi", {method: "POST", body: JSON.stringify({ssid: element("wifi-ssid").value, password: element("wifi-password").value})}); element("wifi-status").textContent = result.message; }
  catch (error) { element("wifi-status").textContent = error.message; }
}

async function powerAction(action) {
  if (!window.confirm(`${action} Bob now? The active mode will stop.`)) return;
  try { const result = await api(`/api/${action}`, {method: "POST", body: "{}"}); showToast(result.message); }
  catch (error) { showToast(error.message, true); }
}

function bindEvents() {
  document.querySelectorAll("[data-panel]").forEach((button) => button.addEventListener("click", () => openPanel(button.dataset.panel)));
  document.querySelectorAll("[data-open-panel]").forEach((button) => button.addEventListener("click", () => openPanel(button.dataset.openPanel)));
  document.querySelectorAll("[data-action]").forEach((button) => button.addEventListener("click", () => modeAction(button.dataset.action)));
  document.querySelectorAll("[data-tracking]").forEach((button) => button.addEventListener("click", () => trackingCommand(button.dataset.tracking, button.dataset.startIfNeeded === "true")));
  document.querySelectorAll("[data-power]").forEach((button) => button.addEventListener("click", () => powerAction(button.dataset.power)));
  element("start-navigation").addEventListener("click", async () => {
    try { const result = await api("/api/start_navigation", {method: "POST", body: JSON.stringify({map: element("map-select").value})}); state.mode = "navigation"; state.activeMap = result.map; updateModeDisplay(); showToast(result.message); }
    catch (error) { showToast(error.message, true); }
  });
  element("start-mapping").addEventListener("click", () => modeAction("start_mapping"));
  element("enable-teleop").addEventListener("click", () => modeAction("start_teleop"));
  element("set-pose").addEventListener("click", () => setMapInteraction("pose"));
  element("set-goal").addEventListener("click", () => setMapInteraction("goal"));
  element("draw-room").addEventListener("click", () => { clearRoomDraft(); setMapInteraction("room"); });
  element("save-room").addEventListener("click", saveRoom); element("clear-room").addEventListener("click", clearRoomDraft);
  element("map-select").addEventListener("change", loadRooms);
  element("save-map").addEventListener("click", async () => { try { const result = await api("/api/save_map", {method: "POST", body: JSON.stringify({name: element("map-name").value.trim()})}); showToast(result.message); setTimeout(loadMaps, 3500); } catch (error) { showToast(error.message, true); } });
  element("assistant-form").addEventListener("submit", (event) => { event.preventDefault(); const input = element("assistant-query"); const text = input.value.trim(); if (text) { askBob(text); input.value = ""; } });
  element("connect-ros").addEventListener("click", connectRos); element("check-imu").addEventListener("click", async () => { try { showToast((await api("/api/check_imu")).message); } catch (error) { showToast(error.message, true); } });
  element("wifi-scan").addEventListener("click", scanWifi); element("wifi-connect").addEventListener("click", connectWifi); element("emergency-stop").addEventListener("click", emergencyStop);
  const linear = () => Number(element("linear-speed").value), angular = () => Number(element("angular-speed").value);
  bindDriveButton("drive-forward", () => startMoving(linear(), 0)); bindDriveButton("drive-back", () => startMoving(-linear(), 0)); bindDriveButton("drive-left", () => startMoving(0, angular())); bindDriveButton("drive-right", () => startMoving(0, -angular()));
  element("drive-stop").addEventListener("click", stopMoving);
  element("linear-speed").addEventListener("input", () => element("linear-output").textContent = `${linear().toFixed(2)} m/s`);
  element("angular-speed").addEventListener("input", () => element("angular-output").textContent = `${angular().toFixed(2)} rad/s`);
  window.addEventListener("blur", stopMoving); document.addEventListener("visibilitychange", () => { if (document.hidden) stopMoving(); }); window.addEventListener("pagehide", stopMoving);
}

async function initialize() {
  element("robot-host").value = window.location.hostname || "127.0.0.1";
  bindEvents(); await refreshStatus(); await loadMaps(); connectRos();
  setInterval(refreshStatus, 2500);
  if ("serviceWorker" in navigator && window.isSecureContext) navigator.serviceWorker.register("service-worker.js").catch(() => {});
}

window.addEventListener("DOMContentLoaded", initialize);

/**
 * app.js – Frontend Controller cho Hệ Thống Quản Lý Kho & Điều Phối Robot (Bài 8.4)
 * Hoàn toàn chạy offline qua mạng LAN, không phụ thuộc thư viện CDN.
 */

// ── Biến Toàn Cục ──────────────────────────────────────────────────────────
let ws = null;
let currentMode = "navigation"; // "navigation" | "mapping" (Bài 8b.1)
let currentRobot = {
  x: -2.40,
  y: 0.00,
  yaw: 0.00,
  state: "IDLE",
  task_id: 0,
  slot_id: "",
  battery_pct: 100.0,
  holding_box: false,
  held_box_tag: 0,
  last_error: "",
  linear_speed: 0.0,
  angular_speed: 0.0,
};

let inventoryData = null;
let itemsList = [];

// Siêu dữ liệu bản đồ động (Bài 8b.2 & 8b.6)
let mapMetadata = {
  version: 0,
  resolution: 0.025,
  origin: [-4.3, -2.5, 0.0],
  origin_x: -4.3,
  origin_y: -2.5,
  width: 312,
  height: 200,
};
let mapImg = new Image();
let mapLoaded = false;

// Các lớp hiển thị và dữ liệu cảm biến thời gian thực (Bài 8b.3 & 8b.6)
let activeLayers = {
  lidar: true,
  trail: true,
  path: true,
  racks: true,
  robot: true,
};
let scanPoints = [];
let trailPoints = [];
let pathPoints = [];
let currentObstacle = null;

// Pan & Zoom View Transform (Bài 8b.6)
let viewTransform = {
  scale: 1.0,
  panX: 0,
  panY: 0,
};
let isDragging = false;
let dragStart = { x: 0, y: 0 };
let hasDragged = false;

// Teleop Timer & Command (Bài 8b.5)
let teleopTimer = null;
let activeTeleopCmd = { vx: 0.0, vy: 0.0, wz: 0.0 };
let isTeleopPressed = false;
let isRobotEnabled = false; // Công tắc kích hoạt di chuyển robot (Enable switch)
let isAutoMapperRunning = false;
let pendingNavGoal = null;

const FSM_STEPS_OUT = [
  { key: "OUT_RESERVE", label: "Đặt ô" },
  { key: "OUT_NAV_TO_SLOT", label: "Đến kệ" },
  { key: "OUT_DOCK_SLOT", label: "Dock ô" },
  { key: "OUT_PICK_FROM_SLOT", label: "Gắp hộp" },
  { key: "OUT_NAV_TO_OUT", label: "Đến xuất" },
  { key: "OUT_DOCK_OUT", label: "Dock bàn" },
  { key: "OUT_PLACE_TO_STATION", label: "Đặt bàn" },
  { key: "OUT_COMMIT", label: "Commit" },
  { key: "OUT_NAV_HOME", label: "Về Home" },
];

const FSM_STEPS_IN = [
  { key: "IN_RESERVE", label: "Đặt ô" },
  { key: "IN_NAV_TO_IN", label: "Đến nhập" },
  { key: "IN_DOCK_IN", label: "Dock bàn" },
  { key: "IN_PICK_FROM_STATION", label: "Gắp hộp" },
  { key: "IN_NAV_TO_SLOT", label: "Đến kệ" },
  { key: "IN_DOCK_SLOT", label: "Dock ô" },
  { key: "IN_PLACE_TO_SLOT", label: "Đặt kệ" },
  { key: "IN_COMMIT", label: "Commit" },
  { key: "IN_NAV_HOME", label: "Về Home" },
];

// ── Khởi Tạo Ứng Dụng ──────────────────────────────────────────────────────
document.addEventListener("DOMContentLoaded", () => {
  initMap();
  initCanvasInteractions();
  initTeleopControls();
  updateRobotEnableUI(false);
  fetchMode();
  connectWebSocket();
  fetchInventory();
  fetchItems();
  fetchTasks();
  fetchOrderHistory();
  fetchTransactions();
  fetchEvents();
  setInterval(pollVisionStatus, 1000);

  // Polling dự phòng tự động làm mới dữ liệu
  setInterval(() => {
    fetchMode();
    fetchInventory();
    fetchTasks();
    fetchItems();
    const activeTab = document.querySelector(".tab-pane.active");
    if (activeTab && activeTab.id === "tab-logs") {
      fetchOrderHistory();
      fetchTransactions();
    }
  }, 3000);
});

// ── 1. WEBSOCKET CLIENT (Tự Động Kết Nối Lại) ────────────────────────────────
function connectWebSocket() {
  const wsDot = document.getElementById("ws-dot");
  const wsText = document.getElementById("ws-text");
  const protocol = window.location.protocol === "https:" ? "wss:" : "ws:";
  const wsUrl = `${protocol}//${window.location.host}/ws`;

  ws = new WebSocket(wsUrl);

  ws.onopen = () => {
    wsDot.className = "status-dot online";
    wsText.textContent = "Trực tuyến";
    console.log("[WS] Đã kết nối thành công tới WebSocket.");
  };

  ws.onmessage = (event) => {
    try {
      const msg = JSON.parse(event.data);
      handleWsMessage(msg);
    } catch (err) {
      console.error("[WS] Lỗi giải mã JSON:", err);
    }
  };

  ws.onclose = () => {
    wsDot.className = "status-dot offline";
    wsText.textContent = "Mất kết nối (Đang thử lại...)";
    // Thử kết nối lại sau 2s
    setTimeout(connectWebSocket, 2000);
  };

  ws.onerror = (err) => {
    console.warn("[WS] Lỗi kết nối WebSocket:", err);
    ws.close();
  };
}

function handleWsMessage(msg) {
  const { type, data } = msg;

  if (type === "mode") {
    currentMode = (data.mode || "navigation").toLowerCase();
    applyModeUI(currentMode);
  } else if (type === "map_update") {
    // Tải ảnh mới và CHỈ cập nhật metadata khi ảnh đã nạp xong (Bài 8b.2, 8b.6)
    if (data.version && data.version > mapMetadata.version) {
      const nextImg = new Image();
      nextImg.src = `/api/map/live.png?v=${data.version}&t=${Date.now()}`;
      nextImg.onload = () => {
        mapImg = nextImg;
        mapMetadata = {
          version: data.version,
          width: data.width,
          height: data.height,
          resolution: data.resolution,
          origin_x: data.origin_x,
          origin_y: data.origin_y,
          origin: [data.origin_x, data.origin_y, 0.0],
        };
        mapLoaded = true;
        updateMapMetaUI();
        drawMap();
      };
    }
  } else if (type === "scan") {
    scanPoints = data.points || [];
    requestAnimationFrame(drawMap);
  } else if (type === "trail") {
    trailPoints = data.points || [];
    requestAnimationFrame(drawMap);
  } else if (type === "path") {
    pathPoints = data.points || [];
    requestAnimationFrame(drawMap);
  } else if (type === "robot") {
    currentRobot = { ...currentRobot, ...data };
    if (data.is_enabled !== undefined) {
      updateRobotEnableUI(data.is_enabled);
    }
    updateRobotUI();
    drawMap();
  } else if (type === "robot_enable") {
    if (data && data.enabled !== undefined) {
      updateRobotEnableUI(data.enabled);
    }
  } else if (type === "history_reset") {
    fetchOrderHistory();
    fetchTransactions();
    fetchEvents();
    fetchTasks();
    fetchInventory();
    fetchItems();
    trailPoints = [];
    pathPoints = [];
    requestAnimationFrame(drawMap);
  } else if (type === "inventory") {
    inventoryData = data;
    renderInventoryView();
    fetchOrderHistory();
    fetchTransactions();
    fetchItems();
  } else if (type === "task") {
    updateActiveTaskView(data);
    fetchTasks();
    fetchOrderHistory();
    fetchTransactions();
    fetchItems();
  } else if (type === "notification") {
    addNotification(data.level, data.text);
    showToast(data.text, data.level);
  } else if (type === "dynamic_obstacle") {
    currentObstacle = data;
    requestAnimationFrame(drawMap);
  }
}

// ── 2. CẬP NHẬT GIAO DIỆN ROBOT ─────────────────────────────────────────────
function updateRobotUI() {
  // Trạng thái Robot
  const stateBadge = document.getElementById("robot-state-badge");
  const st = currentRobot.state || "IDLE";
  stateBadge.textContent = st === "NAV_HOME" ? "VỀ HOME" : st;
  stateBadge.className = "status-pill " + (
    st === "ESTOP" ? "estop" :
    st === "NAV_HOME" ? "nav-home" :
    st === "IDLE" ? "idle" : "running"
  );

  document.getElementById("robot-task-id").textContent = currentRobot.task_id > 0 ? `#${currentRobot.task_id}` : "--";
  document.getElementById("robot-slot-id").textContent = currentRobot.slot_id || "--";
  document.getElementById("robot-holding-box").textContent = currentRobot.holding_box ? `Đang kẹp hộp #${currentRobot.held_box_tag}` : "Không";

  // Cập nhật trạng thái tay kẹp servo ở tab Vision và tab Nhập/Xuất
  const graspStatus = document.getElementById("cam-grasp-status");
  if (graspStatus) {
    graspStatus.textContent = currentRobot.holding_box ? `Đang kẹp chặt hộp (Tag #${currentRobot.held_box_tag})` : "Đang mở (Chưa kẹp)";
    graspStatus.style.color = currentRobot.holding_box ? "var(--color-success)" : "";
  }
  const manualGripperBadge = document.getElementById("manual-gripper-badge");
  if (manualGripperBadge) {
    manualGripperBadge.textContent = currentRobot.holding_box ? "Tay kẹp Servo: ĐÓNG (KẸP CHẶT)" : "Tay kẹp Servo: MỞ";
    manualGripperBadge.className = "status-pill " + (currentRobot.holding_box ? "running" : "idle");
  }

  // Góc quay yaw sang độ
  const deg = (currentRobot.yaw * 180 / Math.PI).toFixed(1);
  document.getElementById("robot-yaw").textContent = `${deg}°`;

  // Tọa độ trên bản đồ
  document.getElementById("map-coords").textContent = `(${currentRobot.x.toFixed(2)}, ${currentRobot.y.toFixed(2)})`;

  // Vận tốc
  document.getElementById("robot-speed-linear").textContent = `${(currentRobot.linear_speed || 0).toFixed(2)} m/s`;
  document.getElementById("robot-speed-angular").textContent = `${(currentRobot.angular_speed || 0).toFixed(2)} rad/s`;

  // Mức pin
  const pct = Math.max(0, Math.min(100, currentRobot.battery_pct || 100));
  const fill = document.getElementById("battery-fill");
  fill.style.width = `${pct}%`;
  fill.style.background = pct > 40 ? "var(--color-success)" : pct > 20 ? "var(--color-warning)" : "var(--color-danger)";
  document.getElementById("battery-text").textContent = `${Math.round(pct)}%`;

  // Lỗi cảnh báo
  const errBox = document.getElementById("robot-error-box");
  const errText = document.getElementById("robot-error-text");
  if (currentRobot.last_error) {
    errBox.classList.remove("hidden");
    errText.textContent = currentRobot.last_error;
  } else {
    errBox.classList.add("hidden");
  }

  // Banner E-Stop
  const estopBanner = document.getElementById("estop-banner");
  if (currentRobot.state === "ESTOP") {
    estopBanner.classList.remove("hidden");
  } else {
    estopBanner.classList.add("hidden");
  }
}

// ── 3. CHẾ ĐỘ VẬN HÀNH & BẢN ĐỒ 2D CANVAS VỚI CÁC LỚP (Bài 8b) ─────────────
async function fetchMode() {
  try {
    const res = await fetch("/api/mode");
    if (!res.ok) return;
    const data = await res.json();
    currentMode = (data.mode || "navigation").toLowerCase();
    applyModeUI(currentMode);
  } catch (err) {
    console.warn("[Mode] Không thể nạp chế độ vận hành:", err);
  }
}

function applyModeUI(mode) {
  const badge = document.getElementById("mode-badge");
  const mapModeLabel = document.getElementById("map-mode-label");
  const warnBanner = document.getElementById("mapping-order-warning");
  const inboundForm = document.getElementById("inbound-form");
  const outboundForm = document.getElementById("outbound-form");
  const slamBadge = document.getElementById("slam-status-badge");

  const setFormsDisabled = (disabled) => {
    [inboundForm, outboundForm].forEach((f) => {
      if (f) {
        f.querySelectorAll("button, select, input").forEach((el) => {
          el.disabled = disabled;
        });
      }
    });
  };

  if (mode === "mapping") {
    if (badge) {
      badge.textContent = "MAPPING MODE";
      badge.className = "mode-badge mapping";
    }
    if (mapModeLabel) {
      mapModeLabel.textContent = "MAPPING";
      mapModeLabel.className = "badge";
      mapModeLabel.style.background = "#f59e0b";
      mapModeLabel.style.color = "#000";
    }
    if (warnBanner) warnBanner.classList.remove("hidden");
    setFormsDisabled(true);
    if (slamBadge) {
      slamBadge.textContent = "Đang dựng SLAM";
      slamBadge.style.background = "#10b981";
    }
  } else {
    if (badge) {
      badge.textContent = "NAV MODE";
      badge.className = "mode-badge navigation";
    }
    if (mapModeLabel) {
      mapModeLabel.textContent = "NAV";
      mapModeLabel.className = "badge";
      mapModeLabel.style.background = "#10b981";
      mapModeLabel.style.color = "#fff";
    }
    if (warnBanner) warnBanner.classList.add("hidden");
    setFormsDisabled(false);
    if (typeof updateInboundPreview === "function") updateInboundPreview();
    if (typeof updateOutboundPreview === "function") updateOutboundPreview();
    if (slamBadge) {
      slamBadge.textContent = "Navigation";
      slamBadge.style.background = "rgba(255,255,255,0.1)";
    }
  }
}

function updateMapMetaUI() {
  const vText = document.getElementById("map-version-text");
  const sText = document.getElementById("map-size-text");
  const rText = document.getElementById("map-res-text");
  if (vText) vText.textContent = `Bản đồ: v${mapMetadata.version || 0}`;
  if (sText) sText.textContent = `${mapMetadata.width || 280}×${mapMetadata.height || 200}`;
  if (rText) rText.textContent = `${(mapMetadata.resolution || 0.025).toFixed(3)}m`;
}

function initMap() {
  // Thử nạp metadata của bản đồ live trước
  fetch("/api/map/live/meta")
    .then((r) => r.json())
    .then((data) => {
      if (data && data.version > 0) {
        mapMetadata = {
          version: data.version,
          width: data.width,
          height: data.height,
          resolution: data.resolution,
          origin_x: data.origin_x,
          origin_y: data.origin_y,
          origin: [data.origin_x, data.origin_y, 0.0],
        };
        // version đếm lại từ 1 mỗi lần chạy -> thêm timestamp để không bao giờ dùng lại ảnh cache cũ
        mapImg.src = `/api/map/live.png?v=${data.version}&t=${Date.now()}`;
      } else {
        return fetch("/api/map").then((r) => r.json()).then((d) => {
          mapMetadata = {
            version: 0,
            width: d.width,
            height: d.height,
            resolution: d.resolution,
            origin_x: d.origin[0],
            origin_y: d.origin[1],
            origin: d.origin,
          };
          mapImg.src = `/api/map/image?t=${Date.now()}`;
        });
      }
    })
    .catch(() => {
      mapImg.src = "map.png";
    })
    .finally(() => {
      mapImg.onload = () => {
        mapLoaded = true;
        updateMapMetaUI();
        drawMap();
      };
    });
}

function toggleLayer(layerName) {
  const chk = document.getElementById(`layer-${layerName}`);
  if (chk) {
    activeLayers[layerName] = chk.checked;
    drawMap();
  }
}

function zoomMap(factor) {
  viewTransform.scale = Math.max(0.4, Math.min(6.0, viewTransform.scale * factor));
  drawMap();
}

function resetMapView() {
  viewTransform.scale = 1.0;
  viewTransform.panX = 0;
  viewTransform.panY = 0;
  drawMap();
}

/**
 * Công thức biến đổi tọa độ thế giới (x, y) sang pixel trong ảnh bản đồ:
 * px = (x - ox) / resolution
 * py = H - (y - oy) / resolution
 * (Gốc tọa độ ảnh ở trên-trái, ROS ở dưới-trái, đã lật qua np.flipud)
 */
function worldToCanvas(x, y) {
  const ox = mapMetadata.origin_x !== undefined ? mapMetadata.origin_x : mapMetadata.origin[0];
  const oy = mapMetadata.origin_y !== undefined ? mapMetadata.origin_y : mapMetadata.origin[1];
  const res = mapMetadata.resolution || 0.025;
  const H = mapMetadata.height || 200;

  const px = (x - ox) / res;
  const py = H - (y - oy) / res;
  return { px, py };
}

/**
 * Biến đổi ngược pixel ảnh sang tọa độ thế giới (x, y).
 */
function canvasToWorld(px, py) {
  const ox = mapMetadata.origin_x !== undefined ? mapMetadata.origin_x : mapMetadata.origin[0];
  const oy = mapMetadata.origin_y !== undefined ? mapMetadata.origin_y : mapMetadata.origin[1];
  const res = mapMetadata.resolution || 0.025;
  const H = mapMetadata.height || 200;

  const x = ox + px * res;
  const y = oy + (H - py) * res;
  return { x, y };
}

function drawMap() {
  const canvas = document.getElementById("map-canvas");
  if (!canvas) return;
  const ctx = canvas.getContext("2d");

  // Đồng bộ kích thước canvas với map metadata
  if (mapMetadata.width && mapMetadata.height) {
    if (canvas.width !== mapMetadata.width || canvas.height !== mapMetadata.height) {
      canvas.width = mapMetadata.width;
      canvas.height = mapMetadata.height;
    }
  }

  const W = canvas.width;
  const H = canvas.height;

  ctx.clearRect(0, 0, W, H);

  // Áp dụng Ma Trận Phóng to / Thu nhỏ & Kéo (Pan & Zoom)
  ctx.save();
  ctx.translate(viewTransform.panX, viewTransform.panY);
  ctx.scale(viewTransform.scale, viewTransform.scale);

  // 1. Ảnh bản đồ nền (PGM / Live SLAM PNG)
  if (mapLoaded) {
    ctx.drawImage(mapImg, 0, 0, W, H);
  } else {
    ctx.fillStyle = "#111827";
    ctx.fillRect(0, 0, W, H);
  }

  // 2. Lớp Quỹ đạo di chuyển (Trail - Bài 8b.3, xanh lam)
  if (activeLayers.trail && trailPoints.length > 1) {
    ctx.save();
    ctx.strokeStyle = "#38bdf8";
    ctx.lineWidth = Math.max(1.0, 1.8 / viewTransform.scale);
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    const p0 = worldToCanvas(trailPoints[0][0], trailPoints[0][1]);
    ctx.moveTo(p0.px, p0.py);
    for (let i = 1; i < trailPoints.length; i++) {
      const p = worldToCanvas(trailPoints[i][0], trailPoints[i][1]);
      ctx.lineTo(p.px, p.py);
    }
    ctx.stroke();
    ctx.restore();
  }

  // 3. Lớp Đường đi Nav2 (Path - Bài 8b.3, xanh lá)
  if (activeLayers.path && pathPoints.length > 1) {
    ctx.save();
    ctx.strokeStyle = "#22c55e";
    ctx.lineWidth = Math.max(1.2, 2.2 / viewTransform.scale);
    ctx.lineJoin = "round";
    ctx.lineCap = "round";
    ctx.beginPath();
    const p0 = worldToCanvas(pathPoints[0][0], pathPoints[0][1]);
    ctx.moveTo(p0.px, p0.py);
    for (let i = 1; i < pathPoints.length; i++) {
      const p = worldToCanvas(pathPoints[i][0], pathPoints[i][1]);
      ctx.lineTo(p.px, p.py);
    }
    ctx.stroke();
    ctx.restore();
  }

  // 4. Lớp Điểm quét LiDAR (LiDAR Scan - Bài 8b.3, đỏ)
  if (activeLayers.lidar && scanPoints.length > 0) {
    ctx.save();
    ctx.fillStyle = "#ef4444";
    const r = Math.max(0.9, 1.3 / viewTransform.scale);
    for (let i = 0; i < scanPoints.length; i++) {
      const p = worldToCanvas(scanPoints[i][0], scanPoints[i][1]);
      ctx.beginPath();
      ctx.arc(p.px, p.py, r, 0, Math.PI * 2);
      ctx.fill();
    }
    ctx.restore();
  }

  // 5. Lớp Ô kệ kho & Trạm dịch vụ (Racks & Stations)
  if (activeLayers.racks) {
    drawPickupTable(ctx, -2.40, 1.25, "TRẠM IN #30", "#10b981", "in");
    drawPickupTable(ctx, -2.40, -1.25, "TRẠM OUT #31", "#f59e0b", "out");
    drawStation(ctx, -2.40, 0.45, "IN Staging", "#34d399");
    drawStation(ctx, -2.40, -0.45, "OUT Staging", "#fbbf24");
    drawStation(ctx, -2.40, 0.00, "HOME", "#6366f1");
    drawRacksOnMap(ctx);
  }

  // 6. Lớp Robot AGV-SCARA
  if (activeLayers.robot) {
    drawRobotOnMap(ctx);
  }

  // 7. Lớp Vật cản động (Dynamic Obstacle - Ô vuông nhỏ tuần tra)
  if (currentObstacle && currentObstacle.x !== undefined) {
    drawDynamicObstacleOnMap(ctx, currentObstacle.x, currentObstacle.y, currentObstacle.size || 0.20);
  }

  ctx.restore();
}

function drawDynamicObstacleOnMap(ctx, ox, oy, size) {
  const half = size / 2;
  const p1 = worldToCanvas(ox - half, oy + half);
  const p2 = worldToCanvas(ox + half, oy - half);
  const w = Math.abs(p2.x - p1.x);
  const h = Math.abs(p2.y - p1.y);

  ctx.save();
  ctx.shadowColor = "#f97316";
  ctx.shadowBlur = 8;
  ctx.fillStyle = "#ea580c";
  ctx.fillRect(p1.x, p1.y, w, h);

  ctx.shadowBlur = 0;
  ctx.strokeStyle = "#1c1917";
  ctx.lineWidth = Math.max(1, 1.5 / viewTransform.scale);
  ctx.strokeRect(p1.x, p1.y, w, h);

  ctx.fillStyle = "#ffffff";
  ctx.font = `bold ${Math.max(8, 10 / viewTransform.scale)}px sans-serif`;
  ctx.textAlign = "center";
  ctx.textBaseline = "bottom";
  ctx.fillText("VẬT CẢN", p1.x + w / 2, p1.y - 3);
  ctx.restore();
}

function drawPickupTable(ctx, cx, cy, label, color, type) {
  const w = 0.85;
  const d = 0.40;
  const topLeft = worldToCanvas(cx - w / 2, cy + d / 2);
  const botRight = worldToCanvas(cx + w / 2, cy - d / 2);

  const rx = Math.min(topLeft.px, botRight.px);
  const ry = Math.min(topLeft.py, botRight.py);
  const rw = Math.abs(botRight.px - topLeft.px);
  const rh = Math.abs(botRight.py - topLeft.py);

  ctx.save();
  // Khung bàn công nghiệp
  ctx.fillStyle = "rgba(15, 23, 42, 0.85)";
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.2;
  ctx.fillRect(rx, ry, rw, rh);
  ctx.strokeRect(rx, ry, rw, rh);

  // 6 vị trí đặt hộp phôi trên bàn: [-2.70, -2.58, -2.46, -2.34, -2.22, -2.10]
  const boxXList = [-2.70, -2.58, -2.46, -2.34, -2.22, -2.10];
  const boxY = cy > 0 ? 1.15 : -1.15;
  const inTags = [26, 27, 28, 29, 32, 33];

  boxXList.forEach((bx, idx) => {
    const pt = worldToCanvas(bx, boxY);
    const bW = 6.0;
    const bH = 6.0;

    if (type === "in") {
      // Trạm IN: 6 phôi hàng chờ nhập kho
      ctx.fillStyle = color;
      ctx.fillRect(pt.px - bW / 2, pt.py - bH / 2, bW, bH);
      ctx.strokeStyle = "#ffffff";
      ctx.lineWidth = 0.5;
      ctx.strokeRect(pt.px - bW / 2, pt.py - bH / 2, bW, bH);
      
      // Số tag mini
      ctx.font = "bold 5px sans-serif";
      ctx.fillStyle = "#ffffff";
      ctx.textAlign = "center";
      ctx.fillText(`${inTags[idx]}`, pt.px, pt.py + 2);
    } else {
      // Trạm OUT: 6 vị trí trả hàng riêng biệt chống đè
      ctx.strokeStyle = color;
      ctx.lineWidth = 0.8;
      ctx.strokeRect(pt.px - bW / 2, pt.py - bH / 2, bW, bH);
      ctx.fillStyle = "rgba(245, 158, 11, 0.25)";
      ctx.fillRect(pt.px - bW / 2, pt.py - bH / 2, bW, bH);

      ctx.font = "bold 5px sans-serif";
      ctx.fillStyle = "#fbbf24";
      ctx.textAlign = "center";
      ctx.fillText(`O${idx}`, pt.px, pt.py + 2);
    }
  });

  // Tên trạm
  ctx.font = "bold 7px sans-serif";
  ctx.fillStyle = "#ffffff";
  ctx.textAlign = "center";
  const labelY = cy > 0 ? ry - 4 : ry + rh + 8;
  ctx.fillText(label, rx + rw / 2, labelY);
  ctx.restore();
}

function drawStation(ctx, x, y, label, color) {
  const { px, py } = worldToCanvas(x, y);
  ctx.save();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(px, py, 3.5, 0, Math.PI * 2);
  ctx.fill();

  ctx.font = "7.5px sans-serif";
  ctx.fillStyle = "#cbd5e1";
  ctx.textAlign = "center";
  ctx.fillText(label, px, py - 5);
  ctx.restore();
}

function drawRacksOnMap(ctx) {
  const cols = [0.75, 1.25, 1.75];
  const allSlots = (inventoryData && inventoryData.slots_detail) ? inventoryData.slots_detail : [];

  // Định nghĩa vị trí thật trong Gazebo:
  // Kệ A: Mặt trước y=1.76, ván sàn y=1.87, bay đỗ xe y=1.325, điểm chờ staging y=1.125
  // Kệ B: Mặt trước y=-1.76, ván sàn y=-1.87, bay đỗ xe y=-1.325, điểm chờ staging y=-1.125
  const racks = [
    { name: "A", rackKey: "rack_a", shelfY: 1.87, bayY: 1.325, stagingY: 1.125, baseColor: "#00d2ff" },
    { name: "B", rackKey: "rack_b", shelfY: -1.87, bayY: -1.325, stagingY: -1.125, baseColor: "#f59e0b" },
  ];

  ctx.save();
  racks.forEach((rack) => {
    cols.forEach((colX, colIdx) => {
      // 1. Điểm chờ an toàn ngoài hành lang (Staging Stand-off Point - y = 1.125 / -1.125)
      const stgPt = worldToCanvas(colX, rack.stagingY);
      ctx.strokeStyle = "rgba(52, 211, 153, 0.5)";
      ctx.fillStyle = "rgba(52, 211, 153, 0.15)";
      ctx.lineWidth = 0.7;
      ctx.beginPath();
      ctx.arc(stgPt.px, stgPt.py, 3.0, 0, Math.PI * 2);
      ctx.fill();
      ctx.stroke();

      // 2. Ô vạch đỗ xe vàng (Docking Bay) trước kệ đúng thực tế Gazebo (y = 1.325 / -1.325)
      const bayPt = worldToCanvas(colX, rack.bayY);
      const bayW = 14;  // 0.36m / 0.025
      const bayH = 19;  // 0.48m / 0.025
      ctx.fillStyle = "rgba(234, 179, 8, 0.12)";
      ctx.fillRect(bayPt.px - bayW / 2, bayPt.py - bayH / 2, bayW, bayH);
      ctx.strokeStyle = "rgba(234, 179, 8, 0.75)";
      ctx.lineWidth = 0.9;
      ctx.setLineDash([3, 2]);
      ctx.strokeRect(bayPt.px - bayW / 2, bayPt.py - bayH / 2, bayW, bayH);
      ctx.setLineDash([]);

      ctx.font = "bold 4.5px sans-serif";
      ctx.fillStyle = "#eab308";
      ctx.textAlign = "center";
      ctx.fillText(`DOCK`, bayPt.px, bayPt.py + 1.5);

      // 3. Thân ô kệ tại vị trí thực tế sát tường (y = 1.87 / -1.87)
      const shelfPt = worldToCanvas(colX, rack.shelfY);
      const sW = 16;
      const sH = 11;
      
      // Lấy trạng thái của Tầng 1 và Tầng 2
      const slotT1 = allSlots.find(s => (s.rack === rack.rackKey || (s.slot_id && s.slot_id.startsWith(rack.name))) && s.col === colIdx && s.tier === 1);
      const slotT2 = allSlots.find(s => (s.rack === rack.rackKey || (s.slot_id && s.slot_id.startsWith(rack.name))) && s.col === colIdx && s.tier === 2);

      // Khung cột kệ
      ctx.fillStyle = "rgba(15, 23, 42, 0.92)";
      ctx.strokeStyle = rack.baseColor;
      ctx.lineWidth = 1.0;
      ctx.fillRect(shelfPt.px - sW / 2, shelfPt.py - sH / 2, sW, sH);
      ctx.strokeRect(shelfPt.px - sW / 2, shelfPt.py - sH / 2, sW, sH);

      // Vẽ 2 tầng (T1 và T2) bên trong cột
      const tW = 6;
      const tH = 7;
      const t1X = shelfPt.px - sW / 2 + 1.2;
      const t2X = shelfPt.px + sW / 2 - tW - 1.2;
      const tY = shelfPt.py - tH / 2;

      // Màu Tầng 1
      const isFullT1 = slotT1 && slotT1.state === "FULL";
      ctx.fillStyle = isFullT1 ? rack.baseColor : "rgba(255, 255, 255, 0.08)";
      ctx.fillRect(t1X, tY, tW, tH);
      ctx.strokeStyle = isFullT1 ? "#fff" : "rgba(255, 255, 255, 0.25)";
      ctx.lineWidth = 0.5;
      ctx.strokeRect(t1X, tY, tW, tH);

      // Màu Tầng 2
      const isFullT2 = slotT2 && slotT2.state === "FULL";
      ctx.fillStyle = isFullT2 ? rack.baseColor : "rgba(255, 255, 255, 0.08)";
      ctx.fillRect(t2X, tY, tW, tH);
      ctx.strokeStyle = isFullT2 ? "#fff" : "rgba(255, 255, 255, 0.25)";
      ctx.lineWidth = 0.5;
      ctx.strokeRect(t2X, tY, tW, tH);

      // Nhãn mã cột (A0, A1, A2, B0, B1, B2)
      ctx.font = "bold 6.5px sans-serif";
      ctx.fillStyle = "#ffffff";
      ctx.textAlign = "center";
      const textY = rack.shelfY > 0 ? shelfPt.py - 8 : shelfPt.py + 14;
      ctx.fillText(`${rack.name}${colIdx}`, shelfPt.px, textY);
    });
  });
  ctx.restore();
}

function drawRobotOnMap(ctx) {
  const { px, py } = worldToCanvas(currentRobot.x, currentRobot.y);
  // Lưu ý: Đảo dấu yaw vì trục y canvas hướng xuống dưới
  const canvasYaw = -currentRobot.yaw;

  ctx.save();
  ctx.translate(px, py);
  ctx.rotate(canvasYaw);

  // Vòng sáng hào quang (Aura) quanh robot
  ctx.beginPath();
  ctx.arc(0, 0, 9, 0, Math.PI * 2);
  ctx.fillStyle = currentRobot.state === "ESTOP"
    ? "rgba(239, 68, 68, 0.3)"
    : currentRobot.state === "NAV_HOME"
    ? "rgba(99, 102, 241, 0.4)"
    : "rgba(0, 210, 255, 0.25)";
  ctx.fill();

  // Thân robot dạng tam giác mũi tên
  ctx.beginPath();
  ctx.moveTo(8, 0);       // Mũi trước
  ctx.lineTo(-6, -5);     // Góc sau trái
  ctx.lineTo(-4, 0);      // Lõm đuôi
  ctx.lineTo(-6, 5);      // Góc sau phải
  ctx.closePath();

  ctx.fillStyle = currentRobot.state === "ESTOP" ? "#ef4444" : (currentRobot.state === "NAV_HOME" ? "#818cf8" : "#00f0ff");
  ctx.fill();
  ctx.strokeStyle = "#ffffff";
  ctx.lineWidth = 1;
  ctx.stroke();

  // Vẽ khối hộp trên khay xe nếu robot đang cầm hộp
  if (currentRobot.holding_box) {
    ctx.fillStyle = "#10b981";
    ctx.fillRect(-3, -3, 6, 6);
    ctx.strokeStyle = "#fff";
    ctx.lineWidth = 0.5;
    ctx.strokeRect(-3, -3, 6, 6);
  }

  ctx.restore();
}

// ── 4. QUẢN LÝ TỒN KHO & Ô KỆ (Mục 8.4 Tab 2) ──────────────────────────────
async function fetchInventory() {
  try {
    const res = await fetch("/api/inventory");
    if (!res.ok) return;
    inventoryData = await res.json();
    renderInventoryView();
  } catch (err) {
    console.warn("Không thể tải tồn kho:", err);
  }
}

function renderInventoryView() {
  if (!inventoryData) return;

  // 1. Thống kê tổng số ô
  const stats = inventoryData.slot_stats || {};
  document.getElementById("rack-stats-text").textContent =
    `${stats.full_count || 0}/${stats.total || 12} Có hàng (${stats.empty_count || 0} Trống)`;

  // 2. Phân loại ô theo Kệ và Tầng
  const slots = inventoryData.slots_detail || [];
  const rackA_T1 = [];
  const rackA_T2 = [];
  const rackB_T1 = [];
  const rackB_T2 = [];

  slots.forEach((s) => {
    const isA = s.rack.toLowerCase().includes("a") || s.slot_id.startsWith("A");
    if (isA) {
      if (s.tier === 2) rackA_T2.push(s);
      else rackA_T1.push(s);
    } else {
      if (s.tier === 2) rackB_T2.push(s);
      else rackB_T1.push(s);
    }
  });

  renderSlotRow("slots-rack-a-tier-2", rackA_T2);
  renderSlotRow("slots-rack-a-tier-1", rackA_T1);
  renderSlotRow("slots-rack-b-tier-2", rackB_T2);
  renderSlotRow("slots-rack-b-tier-1", rackB_T1);

  // 3. Render bảng mặt hàng
  renderItemsTable(inventoryData.items || []);

  // 4. Đồng bộ danh mục tồn kho tab Nhập/Xuất ngay lập tức
  // 4. Đồng bộ danh mục tồn kho tab Nhập/Xuất ngay lập tức
  if (inventoryData.items && itemsList && itemsList.length > 0) {
    inventoryData.items.forEach((invIt) => {
      const match = itemsList.find((it) => it.item_code === invIt.item_code);
      if (match) {
        match.current_stock = (invIt.in_stock !== undefined ? invIt.in_stock : invIt.current_stock) || 0;
        match.is_low_stock = Boolean(invIt.is_low_stock);
      }
    });
    populateItemDropdowns();
  }

  // 5. Cập nhật danh sách ô trống vào dropdown ô đích nhập hàng
  updateInboundSlotOptions(slots);

  // 6. Cập nhật lại bản đồ kho 2D để phản ánh trạng thái mới của ô kệ
  drawMap();
}

function updateInboundSlotOptions(slots) {
  const slotSelect = document.getElementById("inbound-slot-id");
  if (!slotSelect) return;
  const currentVal = slotSelect.value;
  const emptySlots = (slots || []).filter((s) => s.state === "EMPTY");
  
  let html = `<option value="">-- Tự động chọn ô tối ưu (chi phí thấp nhất) --</option>`;
  emptySlots.forEach((s) => {
    const rackName = (s.rack || "").toLowerCase().includes("a") ? "Kệ A" : "Kệ B";
    const locationCode = s.location_code || s.slot_id;
    html += `<option value="${s.slot_id}">${locationCode} (${rackName}, Cột ${s.col + 1}, Tầng ${s.tier}, ID ${s.slot_id})</option>`;
  });
  slotSelect.innerHTML = html;
  if (currentVal && emptySlots.some((s) => s.slot_id === currentVal)) {
    slotSelect.value = currentVal;
  }
}

function renderSlotRow(containerId, slotList) {
  const container = document.getElementById(containerId);
  if (!container) return;
  // Sắp xếp theo cột tăng dần (col 0, 1, 2)
  slotList.sort((a, b) => a.col - b.col);

  container.innerHTML = slotList.map((s) => {
    const stateClass = s.state.toLowerCase();
    const itemText = s.item_code ? `${s.item_code}` : "Trống";
    return `
      <div class="slot-card ${stateClass}" onclick="showSlotDetail('${s.slot_id}', '${s.location_code || s.slot_id}', '${s.state}', '${s.item_code || ''}', ${s.quantity})">
        <span class="slot-name">${s.location_code || s.slot_id}</span>
        <span class="slot-tag">ID ${s.slot_id} • Tag #${s.slot_tag_id}</span>
        <span class="slot-item">${itemText}</span>
        <span class="slot-state-badge ${stateClass}">${s.state}</span>
      </div>
    `;
  }).join("");
}

function renderItemsTable(items) {
  const tbody = document.getElementById("items-table-body");
  if (!tbody) return;

  if (items.length === 0) {
    tbody.innerHTML = `<tr><td colspan="6" class="text-center">Chưa có mặt hàng trong CSDL.</td></tr>`;
    return;
  }

  tbody.innerHTML = items.map((it) => {
    const isLow = it.is_low_stock;
    const badge = isLow
      ? `<span class="badge-low-stock">Sắp hết hàng</span>`
      : `<span class="badge-ok-stock">Ổn định</span>`;
    return `
      <tr>
        <td><strong>${it.item_code}</strong></td>
        <td>${it.name}</td>
        <td>#${it.box_tag_id}</td>
        <td><strong>${it.in_stock}</strong> hộp</td>
        <td>${it.low_stock_threshold} hộp</td>
        <td>${badge}</td>
      </tr>
    `;
  }).join("");
}

function showSlotDetail(slotId, locationCode, state, itemCode, qty) {
  if (state === "EMPTY") {
    const slotSelect = document.getElementById("inbound-slot-id");
    if (slotSelect) {
      slotSelect.value = slotId;
    }
    showToast(`Đã chọn ${locationCode} (${slotId}) làm ô đích cho Trục Nhập Kho!`, "info");
  } else {
    if (itemCode) {
      const outSelect = document.getElementById("outbound-item-code");
      if (outSelect) {
        outSelect.value = itemCode;
        updateOutboundPreview();
      }
    }
    showToast(`Ô ${locationCode}: Trạng thái ${state}${itemCode ? ` (Chứa ${qty} hộp ${itemCode} - Đã chọn sang Trục Xuất Kho)` : ""}`, "info");
  }
}

// ── 5. FORM ĐIỀU PHỐI 2 TRỤC: NHẬP KHO & XUẤT KHO (Tab 3) ──────────────────
async function fetchItems() {
  try {
    const res = await fetch("/api/items");
    if (!res.ok) return;
    itemsList = await res.json();
    populateItemDropdowns();
  } catch (err) {
    console.warn("Không thể tải danh mục mặt hàng:", err);
  }
}

function populateItemDropdowns() {
  if (!itemsList || itemsList.length === 0) return;

  const inSelect = document.getElementById("inbound-item-code");
  const outSelect = document.getElementById("outbound-item-code");

  // 1. Trục Nhập Kho: Ưu tiên các mặt hàng chưa có hàng (current_stock === 0)
  if (inSelect) {
    const prevInVal = inSelect.value;
    const sortedForInbound = [...itemsList].sort((a, b) => {
      const stockA = a.current_stock || 0;
      const stockB = b.current_stock || 0;
      if (stockA === 0 && stockB > 0) return -1;
      if (stockA > 0 && stockB === 0) return 1;
      return a.item_code.localeCompare(b.item_code);
    });

    inSelect.innerHTML = sortedForInbound.map((it) => {
      const stock = it.current_stock || 0;
      const badge = stock === 0
        ? `[✓ Sẵn sàng nhập: 0 hộp]`
        : `[⚠️ Đã có ${stock} hộp - Đầy kho]`;
      return `<option value="${it.item_code}">${it.item_code} – ${it.name} ${badge}</option>`;
    }).join("");

    if (prevInVal && itemsList.some((it) => it.item_code === prevInVal)) {
      inSelect.value = prevInVal;
    } else {
      const firstEligible = sortedForInbound.find((it) => (it.current_stock || 0) === 0);
      if (firstEligible) inSelect.value = firstEligible.item_code;
    }
    updateInboundPreview();
  }

  // 2. Trục Xuất Kho: Ưu tiên các mặt hàng đã có hàng trên kệ (current_stock > 0)
  if (outSelect) {
    const prevOutVal = outSelect.value;
    const sortedForOutbound = [...itemsList].sort((a, b) => {
      const stockA = a.current_stock || 0;
      const stockB = b.current_stock || 0;
      if (stockA > 0 && stockB === 0) return -1;
      if (stockA === 0 && stockB > 0) return 1;
      return a.item_code.localeCompare(b.item_code);
    });

    outSelect.innerHTML = sortedForOutbound.map((it) => {
      const stock = it.current_stock || 0;
      const badge = stock > 0
        ? `[✓ Có sẵn: ${stock} hộp]`
        : `[Hết hàng: 0 hộp]`;
      return `<option value="${it.item_code}">${it.item_code} – ${it.name} ${badge}</option>`;
    }).join("");

    if (prevOutVal && itemsList.some((it) => it.item_code === prevOutVal)) {
      outSelect.value = prevOutVal;
    } else {
      const firstEligible = sortedForOutbound.find((it) => (it.current_stock || 0) > 0);
      if (firstEligible) outSelect.value = firstEligible.item_code;
    }
    updateOutboundPreview();
  }
}

function updateInboundPreview() {
  const select = document.getElementById("inbound-item-code");
  const itemCode = select ? select.value : "";
  const found = itemsList.find((it) => it.item_code === itemCode);
  const stock = found ? (found.current_stock || 0) : 0;

  const stockEl = document.getElementById("inbound-preview-stock");
  if (stockEl) stockEl.textContent = `${stock} hộp`;

  const freeSlots = inventoryData?.slot_stats?.empty_count ?? 0;
  const freeSlotsEl = document.getElementById("inbound-free-slots");
  if (freeSlotsEl) freeSlotsEl.textContent = `${freeSlots} ô`;

  const banner = document.getElementById("inbound-eligibility-banner");
  const btnInbound = document.getElementById("btn-submit-inbound");

  if (banner) {
    if (!itemCode) {
      banner.innerHTML = "";
      banner.style.display = "none";
      if (btnInbound) {
        btnInbound.disabled = true;
        btnInbound.style.opacity = "0.5";
      }
    } else if (stock > 0) {
      banner.innerHTML = `<span>⚠️ Mặt hàng <strong>${itemCode}</strong> hiện <strong>ĐÃ CÓ HÀNG (${stock} hộp)</strong> trong kho → <strong>KHÔNG ĐƯỢC NHẬP THÊM</strong> theo quy tắc WMS!</span>`;
      banner.className = "eligibility-banner banner-in-stock";
      banner.style.display = "block";
      if (btnInbound) {
        btnInbound.disabled = true;
        btnInbound.style.opacity = "0.45";
        btnInbound.title = "Mặt hàng này đã có hàng trên kệ! Chỉ mặt hàng chưa có hàng mới được nhập.";
      }
    } else {
      banner.innerHTML = `<span>✅ Mặt hàng <strong>${itemCode}</strong> hiện <strong>CHƯA CÓ TRÊN KỆ (0 hộp)</strong> → <strong>HỢP LỆ ĐỂ NHẬP KHO</strong> từ trạm IN lên kệ.</span>`;
      banner.className = "eligibility-banner banner-no-stock";
      banner.style.display = "block";
      if (btnInbound) {
        btnInbound.disabled = false;
        btnInbound.style.opacity = "1.0";
        btnInbound.title = "Nhấn để tạo nhiệm vụ nhập hàng";
      }
    }
  }
}

function updateOutboundPreview() {
  const select = document.getElementById("outbound-item-code");
  const itemCode = select ? select.value : "";
  const found = itemsList.find((it) => it.item_code === itemCode);
  const stock = found ? (found.current_stock || 0) : 0;

  const stockEl = document.getElementById("outbound-preview-stock");
  if (stockEl) stockEl.textContent = `${stock} hộp`;

  // Tìm ô kệ đang chứa mặt hàng này
  const occupiedSlot = (inventoryData?.slots || []).find(
    (s) => s.item_code === itemCode && s.state === "OCCUPIED"
  );
  const slotEl = document.getElementById("outbound-preview-slot");
  if (slotEl) {
    if (occupiedSlot) {
      const rackName = (occupiedSlot.rack || "").toLowerCase().includes("a") ? "Kệ A" : "Kệ B";
      const loc = occupiedSlot.location_code || occupiedSlot.slot_id;
      slotEl.textContent = `${loc} (${rackName}, T${occupiedSlot.tier})`;
    } else {
      slotEl.textContent = stock > 0 ? "Chưa xác định ô" : "Không có trên kệ";
    }
  }

  const banner = document.getElementById("outbound-eligibility-banner");
  const btnOutbound = document.getElementById("btn-submit-outbound");

  if (banner) {
    if (!itemCode) {
      banner.innerHTML = "";
      banner.style.display = "none";
      if (btnOutbound) {
        btnOutbound.disabled = true;
        btnOutbound.style.opacity = "0.5";
      }
    } else if (stock <= 0) {
      banner.innerHTML = `<span>⚠️ Mặt hàng <strong>${itemCode}</strong> hiện <strong>HẾT HÀNG (0 hộp)</strong> trên kệ → <strong>KHÔNG THỂ XUẤT KHO</strong>!</span>`;
      banner.className = "eligibility-banner banner-in-stock";
      banner.style.display = "block";
      if (btnOutbound) {
        btnOutbound.disabled = true;
        btnOutbound.style.opacity = "0.45";
        btnOutbound.title = "Mặt hàng này không có tồn kho để xuất!";
      }
    } else {
      const locStr = occupiedSlot ? (occupiedSlot.location_code || occupiedSlot.slot_id) : "kệ";
      banner.innerHTML = `<span>✅ Mặt hàng <strong>${itemCode}</strong> hiện <strong>CÓ SẴN (${stock} hộp)</strong> tại ô <strong>${locStr}</strong> → <strong>SẴN SÀNG XUẤT KHO</strong> ra trạm OUT.</span>`;
      banner.className = "eligibility-banner banner-no-stock";
      banner.style.display = "block";
      if (btnOutbound) {
        btnOutbound.disabled = false;
        btnOutbound.style.opacity = "1.0";
        btnOutbound.title = "Nhấn để tạo nhiệm vụ xuất hàng";
      }
    }
  }
}

function changeInboundQty(delta) {
  const input = document.getElementById("inbound-quantity");
  if (!input) return;
  let val = parseInt(input.value, 10) + delta;
  val = Math.max(1, Math.min(10, val));
  input.value = val;
}

function changeOutboundQty(delta) {
  const input = document.getElementById("outbound-quantity");
  if (!input) return;
  let val = parseInt(input.value, 10) + delta;
  val = Math.max(1, Math.min(10, val));
  input.value = val;
}

function getInboundPriority() {
  const radios = document.getElementsByName("inbound-priority");
  for (let r of radios) {
    if (r.checked) return parseInt(r.value, 10);
  }
  return 1;
}

function getOutboundPriority() {
  const radios = document.getElementsByName("outbound-priority");
  for (let r of radios) {
    if (r.checked) return parseInt(r.value, 10);
  }
  return 1;
}

async function submitInboundOrder() {
  if (!isRobotEnabled) {
    showToast("⚠️ Hệ thống đang KHÓA. Vui lòng bật công tắc ENABLE để lập lệnh Nhập kho!", "warning");
    return;
  }

  const itemSelect = document.getElementById("inbound-item-code");
  const itemCode = itemSelect ? itemSelect.value : "";
  const qtyInput = document.getElementById("inbound-quantity");
  const quantity = qtyInput ? (parseInt(qtyInput.value, 10) || 1) : 1;
  const slotSelect = document.getElementById("inbound-slot-id");
  const slotId = slotSelect && slotSelect.value ? slotSelect.value : null;
  const priority = getInboundPriority();

  if (!itemCode) {
    showToast("Vui lòng chọn mặt hàng cần nhập!", "warn");
    return;
  }

  const found = itemsList.find((it) => it.item_code === itemCode);
  if (found && found.current_stock > 0) {
    showToast(`Mặt hàng '${itemCode}' đã có ${found.current_stock} hộp trong kho! Theo quy định, chỉ mặt hàng chưa có trong kho mới được nhập.`, "warn");
    return;
  }

  const payload = { item_code: itemCode, quantity: quantity, priority: priority };
  if (slotId) {
    payload.slot_id = slotId;
  }

  try {
    const res = await fetch("/api/tasks/inbound", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify(payload),
    });
    const data = await res.json();
    if (res.ok) {
      showToast(`Đã tạo lệnh nhập kho thành công: ${data.message}`, "info");
      fetchInventory();
      fetchItems();
      fetchTasks();
      switchPage("page-robot");
    } else {
      showToast(`Lỗi ${res.status}: ${data.detail || data.message}`, "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để nhập hàng!", "error");
  }
}

async function submitOutboundOrder() {
  if (!isRobotEnabled) {
    showToast("⚠️ Hệ thống đang KHÓA. Vui lòng bật công tắc ENABLE để lập lệnh Xuất kho!", "warning");
    return;
  }

  const itemSelect = document.getElementById("outbound-item-code");
  const itemCode = itemSelect ? itemSelect.value : "";
  const qtyInput = document.getElementById("outbound-quantity");
  const quantity = qtyInput ? (parseInt(qtyInput.value, 10) || 1) : 1;
  const priority = getOutboundPriority();

  if (!itemCode) {
    showToast("Vui lòng chọn mặt hàng cần xuất!", "warn");
    return;
  }

  const found = itemsList.find((it) => it.item_code === itemCode);
  if (found && found.current_stock <= 0) {
    showToast(`Mặt hàng '${itemCode}' chưa có hàng trên kệ (Tồn: 0)! Không thể xuất kho.`, "warn");
    return;
  }

  try {
    const res = await fetch("/api/tasks/outbound", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ item_code: itemCode, quantity: quantity, priority: priority }),
    });
    const data = await res.json();
    if (res.ok) {
      showToast(`Đã tạo lệnh xuất kho thành công: ${data.message}`, "info");
      fetchInventory();
      fetchItems();
      fetchTasks();
      switchPage("page-robot");
    } else {
      showToast(`Lỗi ${res.status}: ${data.detail || data.message}`, "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để xuất hàng!", "error");
  }
}

// Fallback hàm tương thích ngược
function updateItemPreview() {
  updateInboundPreview();
  updateOutboundPreview();
}
function changeQty(delta) {
  changeInboundQty(delta);
  changeOutboundQty(delta);
}

// ── 6. TIẾN ĐỘ FSM & HÀNG ĐỢI NHIỆM VỤ (Mục 8.4 Tab 4) ──────────────────────
async function fetchTasks() {
  try {
    const res = await fetch("/api/tasks?limit=30");
    if (!res.ok) return;
    const tasks = await res.json();

    // 1. Phân loại nhiệm vụ đang chạy và hàng đợi
    const runningTask = tasks.find((t) => t.status === "RUNNING");
    const queuedTasks = tasks.filter((t) => t.status === "QUEUED");

    renderActiveTask(runningTask);
    renderQueuedTasks(queuedTasks);
  } catch (err) {
    console.warn("Không thể tải danh sách nhiệm vụ:", err);
  }
}

function renderActiveTask(task) {
  const body = document.getElementById("active-task-body");
  const badge = document.getElementById("active-task-badge");

  if (!task) {
    badge.textContent = "IDLE";
    badge.className = "badge";
    body.innerHTML = `<div class="empty-state">Hiện không có nhiệm vụ nào đang chạy.</div>`;
    return;
  }

  badge.textContent = `#${task.task_id} – ${task.type}`;
  badge.className = "badge highlight";

  const isOutbound = task.type === "OUTBOUND";
  const steps = isOutbound ? FSM_STEPS_OUT : FSM_STEPS_IN;
  const currState = currentRobot.state;

  // Xác định vị trí bước hiện tại
  let activeIndex = steps.findIndex((s) => s.key === currState);
  if (activeIndex === -1 && currState.includes(isOutbound ? "OUT" : "IN")) {
    activeIndex = 0;
  }

  const stepperHtml = steps.map((s, idx) => {
    let stateClass = "";
    let icon = idx + 1;
    if (idx < activeIndex) {
      stateClass = "completed";
      icon = "✓";
    } else if (idx === activeIndex) {
      stateClass = "active";
    }
    return `
      <div class="fsm-step ${stateClass}">
        <div class="step-circle">${icon}</div>
        <span class="step-label">${s.label}</span>
      </div>
    `;
  }).join("");

  body.innerHTML = `
    <div class="fsm-stepper">${stepperHtml}</div>
    <div class="active-task-meta">
      <div>
        <span>Mặt hàng: <strong>${task.item_code}</strong></span> |
        <span>Ô kệ: <strong>${task.slot_id || currentRobot.slot_id || "--"}</strong></span>
      </div>
      <button class="btn-cancel" onclick="cancelTask(${task.task_id})">Hủy nhiệm vụ</button>
    </div>
  `;
}

function updateActiveTaskView(taskEvent) {
  fetchTasks();
}

function renderQueuedTasks(queue) {
  const tbody = document.getElementById("tasks-queue-body");
  if (!tbody) return;

  if (queue.length === 0) {
    tbody.innerHTML = `<tr><td colspan="7" class="text-center">Hàng đợi trống.</td></tr>`;
    return;
  }

  tbody.innerHTML = queue.map((t) => `
    <tr>
      <td><strong>#${t.task_id}</strong></td>
      <td><span class="status-pill ${t.type.toLowerCase()}">${t.type}</span></td>
      <td>${t.item_code}</td>
      <td>Ưu tiên ${t.priority}</td>
      <td>${t.status}</td>
      <td>${(t.created_at || "").slice(11, 19)}</td>
      <td>
        <button class="btn-sm btn-danger" onclick="cancelTask(${t.task_id})">Hủy</button>
      </td>
    </tr>
  `).join("");
}

async function cancelTask(taskId) {
  if (!confirm(`Bạn có chắc chắn muốn hủy nhiệm vụ #${taskId}?`)) return;

  try {
    const res = await fetch(`/api/tasks/${taskId}/cancel`, { method: "POST" });
    const data = await res.json();
    if (res.ok) {
      showToast(data.message || `Đã hủy nhiệm vụ #${taskId}`, "info");
      fetchTasks();
      fetchInventory();
    } else {
      showToast(`Không thể hủy: ${data.detail || data.message}`, "error");
    }
  } catch (err) {
    showToast("Lỗi mạng khi gửi yêu cầu hủy nhiệm vụ", "error");
  }
}

// ── 7. LỊCH SỬ ĐƠN HÀNG, GIAO DỊCH & THÔNG BÁO (Mục 8.4 Tab 5) ──────────────
let currentHistoryTab = "orders";

function switchHistoryTab(tabName) {
  currentHistoryTab = tabName;
  const btnOrders = document.getElementById("btn-tab-orders-history");
  const btnTx = document.getElementById("btn-tab-tx-history");
  const ordersContainer = document.getElementById("order-history-table-container");
  const txContainer = document.getElementById("tx-table-container");

  if (tabName === "orders") {
    if (btnOrders) btnOrders.classList.add("active");
    if (btnTx) btnTx.classList.remove("active");
    if (ordersContainer) ordersContainer.classList.remove("hidden");
    if (txContainer) txContainer.classList.add("hidden");
    fetchOrderHistory();
  } else {
    if (btnOrders) btnOrders.classList.remove("active");
    if (btnTx) btnTx.classList.add("active");
    if (ordersContainer) ordersContainer.classList.add("hidden");
    if (txContainer) txContainer.classList.remove("hidden");
    fetchTransactions();
  }
}

function refreshHistoryData() {
  fetchOrderHistory();
  fetchTransactions();
  fetchEvents();
}

async function fetchOrderHistory() {
  try {
    const res = await fetch("/api/tasks?limit=50");
    if (!res.ok) return;
    const tasks = await res.json();
    const tbody = document.getElementById("order-history-body");
    if (!tbody) return;

    if (tasks.length === 0) {
      tbody.innerHTML = `<tr><td colspan="6" class="text-center">Chưa có đơn hàng nào trong hệ thống.</td></tr>`;
      return;
    }

    tbody.innerHTML = tasks.map((t) => {
      const isOutbound = t.type === "OUTBOUND";
      const typeClass = isOutbound ? "outbound" : "inbound";
      const typeLabel = isOutbound ? "XUẤT" : "NHẬP";

      let statusBadge = "";
      const s = (t.status || "QUEUED").toUpperCase();
      if (s === "SUCCEEDED") {
        statusBadge = `<span class="status-badge succeeded">✓ Hoàn thành</span>`;
      } else if (s === "RUNNING") {
        statusBadge = `<span class="status-badge running">⚙ Đang chạy</span>`;
      } else if (s === "QUEUED") {
        statusBadge = `<span class="status-badge queued">⏳ Chờ xử lý</span>`;
      } else if (s === "FAILED") {
        statusBadge = `<span class="status-badge failed" title="${t.error || ''}">✖ Thất bại</span>`;
      } else if (s === "CANCELLED") {
        statusBadge = `<span class="status-badge cancelled">🚫 Đã hủy</span>`;
      } else {
        statusBadge = `<span class="status-badge">${s}</span>`;
      }

      const timeStr = t.finished_at ? t.finished_at.slice(11, 19) : (t.started_at ? t.started_at.slice(11, 19) : (t.created_at || "").slice(11, 19));
      const itemName = t.item_name || t.item_code;

      return `
        <tr>
          <td><strong>#${t.task_id}</strong></td>
          <td><span class="type-pill ${typeClass}">${typeLabel}</span></td>
          <td><strong>${itemName}</strong> <small style="color:var(--text-muted)">(${t.item_code})</small></td>
          <td><code>${t.slot_id || "--"}</code></td>
          <td>${statusBadge}</td>
          <td>${timeStr}</td>
        </tr>
      `;
    }).join("");
  } catch (err) {
    console.warn("Không thể tải lịch sử đơn hàng:", err);
  }
}

async function fetchTransactions() {
  try {
    const res = await fetch("/api/transactions?limit=50");
    if (!res.ok) return;
    const txs = await res.json();
    const tbody = document.getElementById("tx-table-body");
    if (!tbody) return;

    if (txs.length === 0) {
      tbody.innerHTML = `<tr><td colspan="6" class="text-center">Chưa có giao dịch biến động kho.</td></tr>`;
      return;
    }

    tbody.innerHTML = txs.map((tx) => {
      const isAdd = tx.delta > 0;
      const deltaClass = isAdd ? "text-success" : "text-danger";
      const typeClass = tx.type === "INBOUND" ? "inbound" : "outbound";
      const typeLabel = tx.type === "INBOUND" ? "NHẬP" : "XUẤT";
      const itemName = tx.item_name || tx.item_code;
      return `
        <tr>
          <td>#${tx.task_id}</td>
          <td><span class="type-pill ${typeClass}">${typeLabel}</span></td>
          <td><strong>${itemName}</strong> <small style="color:var(--text-muted)">(${tx.item_code})</small></td>
          <td><code>${tx.slot_id || "--"}</code></td>
          <td class="${deltaClass}"><strong>${isAdd ? "+" : ""}${tx.delta}</strong></td>
          <td>${(tx.ts || "").slice(11, 19)}</td>
        </tr>
      `;
    }).join("");
  } catch (err) {
    console.warn("Không thể tải lịch sử giao dịch:", err);
  }
}

async function fetchEvents() {
  try {
    const res = await fetch("/api/events?limit=30");
    if (!res.ok) return;
    const events = await res.json();
    const feed = document.getElementById("notifications-feed");
    if (!feed || events.length === 0) return;

    feed.innerHTML = events.map((ev) => {
      const timeStr = (ev.ts || "").slice(11, 19) || new Date().toLocaleTimeString();
      const lvl = (ev.level || "info").toLowerCase();
      return `
        <div class="notif-item ${lvl}">
          <div>${ev.message}</div>
          <div class="notif-time">${timeStr}</div>
        </div>
      `;
    }).join("");
  } catch (err) {
    console.warn("Không thể tải lịch sử sự kiện:", err);
  }
}

function addNotification(level, text) {
  const feed = document.getElementById("notifications-feed");
  if (!feed) return;

  const empty = feed.querySelector(".notif-empty");
  if (empty) empty.remove();

  const now = new Date().toLocaleTimeString();
  const div = document.createElement("div");
  div.className = `notif-item ${level || "info"}`;
  div.innerHTML = `
    <div>${text}</div>
    <div class="notif-time">${now}</div>
  `;
  feed.prepend(div);
}

function clearNotifications() {
  const feed = document.getElementById("notifications-feed");
  if (feed) {
    feed.innerHTML = `<div class="notif-empty">Chưa có thông báo mới.</div>`;
  }
}

// ── 8. E-STOP & XÁC NHẬN HAI BƯỚC KHÔI PHỤC ─────────────────────────────────
async function triggerEStop() {
  try {
    const res = await fetch("/api/estop", { method: "POST" });
    if (res.ok) {
      showToast("ĐÃ KÍCH HOẠT DỪNG KHẨN CẤP (E-STOP)!", "error");
      currentRobot.state = "ESTOP";
      updateRobotUI();
      drawMap();
    }
  } catch (err) {
    showToast("Không thể gửi lệnh E-Stop tới máy chủ!", "error");
  }
}

function openResetModal() {
  const modal = document.getElementById("reset-modal");
  const chk = document.getElementById("chk-confirm-safety");
  const btn = document.getElementById("btn-do-reset");
  if (modal) modal.classList.remove("hidden");
  if (chk) chk.checked = false;
  if (btn) btn.disabled = true;
}

function closeResetModal() {
  const modal = document.getElementById("reset-modal");
  if (modal) modal.classList.add("hidden");
}

function toggleResetBtn() {
  const chk = document.getElementById("chk-confirm-safety");
  const btn = document.getElementById("btn-do-reset");
  if (btn && chk) {
    btn.disabled = !chk.checked;
  }
}

async function executeResetEStop() {
  try {
    const res = await fetch("/api/estop/reset", { method: "POST" });
    if (res.ok) {
      closeResetModal();
      showToast("Đã giải phóng E-Stop! Robot đang tự động di chuyển về trạm HOME...", "info");
      currentRobot.state = "NAV_HOME";
      currentRobot.is_navigating_home = true;
      updateRobotUI();
      updateRobotEnableUI(isRobotEnabled);
      drawMap();
    } else {
      showToast("Lỗi khi khôi phục E-Stop!", "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để khôi phục!", "error");
  }
}

// ── 9. TIỆN ÍCH CHUYỂN 2 TRANG & TOAST ──────────────────────────────────────
function switchPage(pageId, navBtn) {
  // Ẩn tất cả panes
  document.querySelectorAll(".tab-pane").forEach((pane) => {
    pane.classList.remove("active");
  });
  // Hiện trang được chọn
  const activePane = document.getElementById(pageId);
  if (activePane) activePane.classList.add("active");

  // Đồng bộ trạng thái active cho cả Header Tabs và Bottom Nav
  document.querySelectorAll(".header-tab-btn, .nav-item").forEach((btn) => {
    if (btn.getAttribute("onclick")?.includes(pageId)) {
      btn.classList.add("active");
    } else {
      btn.classList.remove("active");
    }
  });

  // Kích hoạt render lại các thành phần tương ứng
  if (pageId === "page-robot") {
    setTimeout(drawMap, 50);
    fetchTasks();
  } else if (pageId === "page-warehouse") {
    fetchInventory();
    fetchItems();
    refreshHistoryData();
  }
}

function switchTabById(tabId) {
  if (tabId === "tab-orders" || tabId === "tab-inventory" || tabId === "page-warehouse") {
    switchPage("page-warehouse");
  } else {
    switchPage("page-robot");
  }
}

function switchTab(tabId, navBtn) {
  switchTabById(tabId);
}

function showToast(message, type = "info") {
  const container = document.getElementById("toast-container");
  if (!container) return;

  const toast = document.createElement("div");
  toast.className = `toast ${type}`;
  toast.textContent = message;

  container.appendChild(toast);

  setTimeout(() => {
    toast.style.opacity = "0";
    toast.style.transform = "translateX(100%)";
    toast.style.transition = "all 0.3s";
    setTimeout(() => toast.remove(), 300);
  }, 4000);
}

// ── 10. TƯƠNG TÁC CANVAS: PAN & ZOOM VÀ CHỌN ĐIỂM NAV2 (Bài 8b.6) ───────────
function initCanvasInteractions() {
  const container = document.getElementById("canvas-container");
  const canvas = document.getElementById("map-canvas");
  if (!container || !canvas) return;

  // 1. Kéo bản đồ (Pan Drag)
  container.addEventListener("pointerdown", (e) => {
    if (e.button !== 0 && e.pointerType === "mouse") return;
    isDragging = true;
    hasDragged = false;
    dragStart = { x: e.clientX - viewTransform.panX, y: e.clientY - viewTransform.panY };
    container.setPointerCapture(e.pointerId);
  });

  container.addEventListener("pointermove", (e) => {
    if (!isDragging) return;
    const dx = Math.abs(e.clientX - (dragStart.x + viewTransform.panX));
    const dy = Math.abs(e.clientY - (dragStart.y + viewTransform.panY));
    if (dx > 3 || dy > 3) {
      hasDragged = true;
    }
    viewTransform.panX = e.clientX - dragStart.x;
    viewTransform.panY = e.clientY - dragStart.y;
    drawMap();
  });

  const stopDrag = (e) => {
    if (!isDragging) return;
    isDragging = false;
    try {
      container.releasePointerCapture(e.pointerId);
    } catch (_) {}

    // Nếu chỉ chạm click (không kéo) trong chế độ Navigation & Robot rảnh -> Mở hộp thoại gửi mục tiêu
    if (!hasDragged && currentMode === "navigation" && currentRobot.state === "IDLE") {
      const rect = canvas.getBoundingClientRect();
      const clickClientX = e.clientX - rect.left;
      const clickClientY = e.clientY - rect.top;

      // Tính pixel ảnh trước khi áp ma trận viewTransform
      const imgPx = (clickClientX - viewTransform.panX) / viewTransform.scale;
      const imgPy = (clickClientY - viewTransform.panY) / viewTransform.scale;

      const world = canvasToWorld(imgPx, imgPy);
      openNavGoalModal(world.x, world.y);
    }
  };

  container.addEventListener("pointerup", stopDrag);
  container.addEventListener("pointercancel", stopDrag);

  // 2. Cuộn chuột để Phóng to / Thu nhỏ (Wheel Zoom)
  container.addEventListener("wheel", (e) => {
    e.preventDefault();
    const zoomFactor = e.deltaY < 0 ? 1.15 : 0.85;
    const rect = canvas.getBoundingClientRect();
    const mouseX = e.clientX - rect.left;
    const mouseY = e.clientY - rect.top;

    const newScale = Math.max(0.4, Math.min(6.0, viewTransform.scale * zoomFactor));
    viewTransform.panX = mouseX - (mouseX - viewTransform.panX) * (newScale / viewTransform.scale);
    viewTransform.panY = mouseY - (mouseY - viewTransform.panY) * (newScale / viewTransform.scale);
    viewTransform.scale = newScale;

    drawMap();
  }, { passive: false });
}

// ── 11. ĐIỀU KHIỂN TELEOP VỚI WATCHDOG 0.3s (Bài 8b.5) ──────────────────────
function initTeleopControls() {
  const buttons = [
    { id: "btn-teleop-fwd", cmd: { vx: 0.22, vy: 0.0, wz: 0.0 } },
    { id: "btn-teleop-bwd", cmd: { vx: -0.22, vy: 0.0, wz: 0.0 } },
    { id: "btn-teleop-left", cmd: { vx: 0.0, vy: 0.18, wz: 0.0 } },
    { id: "btn-teleop-right", cmd: { vx: 0.0, vy: -0.18, wz: 0.0 } },
    { id: "btn-teleop-rot-left", cmd: { vx: 0.0, vy: 0.0, wz: 0.50 } },
    { id: "btn-teleop-rot-right", cmd: { vx: 0.0, vy: 0.0, wz: -0.50 } },
  ];

  buttons.forEach(({ id, cmd }) => {
    const btn = document.getElementById(id);
    if (!btn) return;

    const startHandler = (e) => {
      e.preventDefault();
      btn.classList.add("pressed");
      startTeleop(cmd.vx, cmd.vy, cmd.wz);
    };

    const stopHandler = (e) => {
      e.preventDefault();
      btn.classList.remove("pressed");
      stopTeleop();
    };

    btn.addEventListener("pointerdown", startHandler);
    btn.addEventListener("pointerup", stopHandler);
    btn.addEventListener("pointerleave", stopHandler);
    btn.addEventListener("pointercancel", stopHandler);
  });

  const stopBtn = document.getElementById("btn-teleop-stop");
  if (stopBtn) {
    stopBtn.addEventListener("click", () => {
      stopTeleop();
    });
  }
}

function startTeleop(vx, vy, wz) {
  if (!isRobotEnabled) {
    showToast("⚠️ Robot đang bị KHÓA! Vui lòng bật công tắc Enable để di chuyển.", "warning");
    return;
  }
  stopTeleop();
  isTeleopPressed = true;
  activeTeleopCmd = { vx, vy, wz };
  sendTeleop(vx, vy, wz);
  // Gửi lặp lại 10 Hz (mỗi 100ms) để nuôi Watchdog phía server (Bài 8b.5)
  teleopTimer = setInterval(() => {
    if (!isRobotEnabled) {
      stopTeleop();
      return;
    }
    sendTeleop(activeTeleopCmd.vx, activeTeleopCmd.vy, activeTeleopCmd.wz);
  }, 100);
}

function stopTeleop() {
  isTeleopPressed = false;
  if (teleopTimer) {
    clearInterval(teleopTimer);
    teleopTimer = null;
  }
  activeTeleopCmd = { vx: 0.0, vy: 0.0, wz: 0.0 };
  sendTeleop(0.0, 0.0, 0.0);
}

async function sendTeleop(vx, vy, wz) {
  try {
    await fetch("/api/teleop", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ vx, vy, wz }),
    });
  } catch (err) {
    // Không làm gián đoạn UI nếu có lỗi mạng
  }
}

// Kiểm tra xem robot có đang di chuyển / bận hoạt động hay không
function isRobotRunning() {
  const lin = Math.abs(currentRobot.linear_speed || 0);
  const ang = Math.abs(currentRobot.angular_speed || 0);
  const st = (currentRobot.state || "IDLE").toUpperCase();
  const isBusyState = st !== "IDLE" && st !== "OFFLINE" && st !== "ESTOP";
  return lin > 0.01 || ang > 0.01 || isBusyState || isTeleopPressed || Boolean(currentRobot.is_navigating_home);
}

// ── CÔNG TẮC KÍCH HOẠT ROBOT (ROBOT MOTION ENABLE) ─────────────────────────
async function toggleRobotEnable(enabled) {
  const willEnable = Boolean(enabled);

  // RÀNG BUỘC AN TOÀN: Không được phép tắt Enable khi robot đang chạy!
  if (!willEnable && isRobotRunning()) {
    // Hoàn tác lại trạng thái switch về ON ngay lập tức
    const cardSwitch = document.getElementById("robot-enable-switch");
    if (cardSwitch) cardSwitch.checked = true;
    const headerSwitch = document.getElementById("header-robot-enable");
    if (headerSwitch) headerSwitch.checked = true;

    // Rung lắc cảnh báo
    [cardSwitch, headerSwitch].forEach((sw) => {
      const parent = sw?.closest(".switch") || sw?.parentElement;
      if (parent) {
        parent.classList.add("shake-locked");
        setTimeout(() => parent.classList.remove("shake-locked"), 500);
      }
    });

    const stateDesc = currentRobot.state === "NAV_HOME" ? "đang tự động di chuyển về trạm HOME" :
                      currentRobot.state ? `đang ở trạng thái ${currentRobot.state}` : "đang di chuyển";
    showToast(`⚠️ Không được phép tắt ENABLE khi robot đang hoạt động (${stateDesc})!`, "warning");
    return;
  }

  try {
    const res = await fetch("/api/robot/enable", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ enabled: willEnable }),
    });
    const data = await res.json();
    if (res.ok && data.success) {
      updateRobotEnableUI(data.is_enabled);
      if (data.is_enabled) {
        showToast("⚡ Công tắc Enable ĐÃ BẬT. Robot sẵn sàng di chuyển!", "success");
      } else {
        stopTeleop();
        showToast("🔒 Công tắc Enable ĐÃ TẮT. Đã khóa di chuyển robot.", "warning");
      }
    } else {
      // Revert lại trạng thái cũ nếu server từ chối
      updateRobotEnableUI(isRobotEnabled);
      showToast(data.detail || "Không thể chuyển trạng thái Enable!", "error");
    }
  } catch (err) {
    console.error("[Enable] Lỗi gửi lệnh toggle enable:", err);
    updateRobotEnableUI(isRobotEnabled);
    showToast("Không thể gửi lệnh chuyển trạng thái Enable!", "error");
  }
}

function updateRobotEnableUI(enabled) {
  isRobotEnabled = Boolean(enabled);

  // 0. Khóa / Mở Khóa Toàn Bộ Hệ Thống trên Body
  document.body.classList.toggle("robot-disabled", !isRobotEnabled);
  document.body.classList.toggle("robot-enabled", isRobotEnabled);

  // 1. Đồng bộ Switch ở Card SLAM & D-Pad
  const cardSwitch = document.getElementById("robot-enable-switch");
  if (cardSwitch && cardSwitch.checked !== isRobotEnabled) {
    cardSwitch.checked = isRobotEnabled;
  }

  // 2. Đồng bộ Switch ở Header
  const headerSwitch = document.getElementById("header-robot-enable");
  if (headerSwitch && headerSwitch.checked !== isRobotEnabled) {
    headerSwitch.checked = isRobotEnabled;
  }

  // 3. Khóa trạng thái công tắc khi robot đang chạy (không cho phép tắt)
  const running = isRobotRunning();
  [cardSwitch, headerSwitch].forEach((sw) => {
    const parent = sw?.closest(".switch") || sw?.parentElement;
    if (parent) {
      if (isRobotEnabled && running) {
        parent.classList.add("switch-running-lock");
        parent.setAttribute("title", "Robot đang hoạt động / di chuyển - Không được phép tắt ENABLE!");
      } else {
        parent.classList.remove("switch-running-lock");
        parent.setAttribute("title", "Bật/Tắt công tắc kích hoạt robot");
      }
    }
  });

  // 4. Header Wrap Class
  const headerWrap = document.getElementById("header-enable-wrap");
  if (headerWrap) {
    headerWrap.classList.toggle("active", isRobotEnabled);
  }

  // 5. Card Box Class
  const cardBox = document.getElementById("robot-enable-card-box");
  if (cardBox) {
    cardBox.classList.toggle("active", isRobotEnabled);
  }

  // 6. Card Badge
  const badge = document.getElementById("robot-enable-badge");
  if (badge) {
    if (isRobotEnabled) {
      badge.textContent = running ? "⚡ ĐANG CHẠY (KHÓA TẮT)" : "⚡ ĐÃ BẬT (ON)";
      badge.className = "badge enable-status-badge enabled" + (running ? " running-active" : "");
    } else {
      badge.textContent = "🔒 ĐÃ KHÓA (OFF)";
      badge.className = "badge enable-status-badge disabled";
    }
  }

  // 7. Hint text
  const hint = document.getElementById("enable-hint-text");
  if (hint) {
    if (isRobotEnabled) {
      hint.textContent = running
        ? "⚡ Robot đang hoạt động / di chuyển. Công tắc Enable được giữ cố định an toàn!"
        : "✅ Động cơ đã kích hoạt. Sẵn sàng nhận lệnh di chuyển!";
      hint.classList.add("active");
    } else {
      hint.textContent = "⚠️ Động cơ đang khóa. Bật công tắc Enable để cho phép robot di chuyển.";
      hint.classList.remove("active");
    }
  }

  // 8. Teleop D-Pad Container Lock
  const teleopSection = document.getElementById("teleop-section");
  if (teleopSection) {
    teleopSection.classList.toggle("locked", !isRobotEnabled);
  }
}

function onTeleopLockClicked() {
  showToast("⚠️ Robot đang khóa di chuyển! Vui lòng gạt công tắc Enable phía trên sang BẬT để mở khóa.", "warning");
}

// ── 12. QUẢN LÝ VÀ ĐIỀU KHIỂN SLAM (Bài 8b.5) ────────────────────────────────
function openSaveMapModal() {
  const modal = document.getElementById("save-map-modal");
  if (modal) modal.classList.remove("hidden");
}

function closeSaveMapModal() {
  const modal = document.getElementById("save-map-modal");
  if (modal) modal.classList.add("hidden");
}

async function confirmSaveMap() {
  const input = document.getElementById("input-map-name");
  const name = input ? input.value.trim() : "warehouse_map";
  if (!name) {
    showToast("Vui lòng nhập tên bản đồ!", "warn");
    return;
  }

  try {
    const res = await fetch("/api/slam/save", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name }),
    });
    const data = await res.json();
    if (res.ok) {
      closeSaveMapModal();
      showToast(data.message || `Đã lưu bản đồ '${name}' thành công!`, "info");
    } else {
      showToast(data.detail || "Lỗi lưu bản đồ!", "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để lưu bản đồ!", "error");
  }
}

async function serializeSlamMap() {
  const name = prompt("Nhập tên file serialize pose graph:", "slam_pose_graph");
  if (!name) return;

  try {
    const res = await fetch("/api/slam/serialize", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ name }),
    });
    const data = await res.json();
    if (res.ok) {
      showToast(data.message || "Đã lưu pose graph thành công!", "info");
    } else {
      showToast(data.detail || "Lỗi serialize pose graph!", "error");
    }
  } catch (err) {
    showToast("Không thể gửi lệnh serialize!", "error");
  }
}

function openClearSlamModal() {
  const modal = document.getElementById("clear-slam-modal");
  const chk = document.getElementById("chk-confirm-clear-slam");
  const btn = document.getElementById("btn-do-clear-slam");
  if (modal) modal.classList.remove("hidden");
  if (chk) chk.checked = false;
  if (btn) btn.disabled = true;
}

function closeClearSlamModal() {
  const modal = document.getElementById("clear-slam-modal");
  if (modal) modal.classList.add("hidden");
}

function toggleClearSlamBtn() {
  const chk = document.getElementById("chk-confirm-clear-slam");
  const btn = document.getElementById("btn-do-clear-slam");
  if (btn && chk) {
    btn.disabled = !chk.checked;
  }
}

async function confirmClearSlam() {
  try {
    const res = await fetch("/api/slam/clear", { method: "POST" });
    const data = await res.json();
    if (res.ok) {
      closeClearSlamModal();
      trailPoints = [];
      scanPoints = [];
      drawMap();
      showToast(data.message || "Đã đặt lại phiên SLAM!", "info");
    } else {
      showToast(data.detail || "Lỗi đặt lại phiên SLAM!", "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để xóa phiên SLAM!", "error");
  }
}

async function toggleAutoMapper() {
  const btn = document.getElementById("btn-auto-mapper");
  if (!isAutoMapperRunning) {
    try {
      const res = await fetch("/api/mapping/auto/start", { method: "POST" });
      if (res.ok) {
        isAutoMapperRunning = true;
        if (btn) {
          btn.textContent = "⏹️ Dừng Quét";
          btn.style.background = "var(--color-danger)";
        }
        showToast("Đã bắt đầu tự động quét kho (AutoMapper)!", "info");
      }
    } catch (err) {
      showToast("Lỗi khởi chạy AutoMapper!", "error");
    }
  } else {
    try {
      const res = await fetch("/api/mapping/auto/stop", { method: "POST" });
      if (res.ok) {
        isAutoMapperRunning = false;
        if (btn) {
          btn.textContent = "🤖 Tự Động Quét";
          btn.style.background = "";
        }
        showToast("Đã dừng tự động quét kho!", "info");
      }
    } catch (err) {
      showToast("Lỗi dừng AutoMapper!", "error");
    }
  }
}

// ── 13. ĐIỀU HƯỚNG TỚI ĐIỂM CHỌN TRÊN BẢN ĐỒ (NAV2 GOAL) ────────────────────
function openNavGoalModal(x, y) {
  if (!isRobotEnabled) {
    showToast("⚠️ Bản đồ đang KHÓA. Vui lòng bật công tắc ENABLE để gửi mục tiêu điều hướng!", "warning");
    return;
  }
  pendingNavGoal = { x, y };
  const modal = document.getElementById("nav-goal-modal");
  const text = document.getElementById("nav-goal-text");
  if (text) {
    text.textContent = `Bạn có muốn robot điều hướng tới tọa độ (x: ${x.toFixed(2)}, y: ${y.toFixed(2)})?`;
  }
  if (modal) modal.classList.remove("hidden");
}

function closeNavGoalModal() {
  const modal = document.getElementById("nav-goal-modal");
  if (modal) modal.classList.add("hidden");
  pendingNavGoal = null;
}

async function confirmNavGoal() {
  if (!pendingNavGoal) return;
  const { x, y } = pendingNavGoal;
  closeNavGoalModal();
  showToast(`Mục tiêu (x: ${x.toFixed(2)}, y: ${y.toFixed(2)}) đã được ghi nhận.`, "info");
}

// ── 14. THỊ GIÁC (CAMERA TRỰC TIẾP TRÊN CÙNG KHUNG HÌNH) ────────────────────
const TAG_TYPE_LABEL = { shelf: "Ô kệ", station: "Trạm IN/OUT", box: "Hộp hàng" };
let currentFrameViewMode = "pip"; // 'pip' | 'split' | 'map' | 'cam'
let currentPipSource = "dock";     // 'dock' | 'hand'
let currentActiveCamFeed = "dock"; // 'dock' | 'hand'

function setFrameViewMode(mode) {
  currentFrameViewMode = mode;
  const viewport = document.getElementById("dual-frame-viewport");
  if (viewport) {
    viewport.className = `dual-frame-viewport mode-${mode}`;
  }

  // Cập nhật trạng thái nút bấm
  document.querySelectorAll(".view-mode-btn").forEach((btn) => {
    btn.classList.remove("active");
  });
  const activeBtn = document.getElementById(`btn-mode-${mode}`);
  if (activeBtn) activeBtn.classList.add("active");

  const pip = document.getElementById("map-pip-camera");
  if (pip) {
    if (mode === "pip") {
      pip.classList.remove("hidden");
    } else {
      pip.classList.add("hidden");
    }
  }

  // Vẽ lại canvas map để fit kích thước mới
  setTimeout(drawMap, 60);

  const modeNames = {
    pip: "Camera PiP (Nổi góc bản đồ)",
    split: "Chia đôi song song (Bản đồ & Camera)",
    map: "Toàn màn hình Bản đồ",
    cam: "Toàn màn hình Camera",
  };
  showToast(`Chế độ hiển thị: ${modeNames[mode] || mode}`, "info");
}

function togglePipCamSource() {
  currentPipSource = currentPipSource === "dock" ? "hand" : "dock";
  const pipImg = document.getElementById("pip-stream-img");
  const pipTitle = document.getElementById("pip-cam-title");
  if (pipImg) {
    pipImg.src = `/api/vision/${currentPipSource}/stream?t=${Date.now()}`;
  }
  if (pipTitle) {
    pipTitle.textContent = currentPipSource === "dock" ? "● LIVE DOCK CAM" : "● LIVE HAND CAM";
  }
  showToast(`Camera PiP: Đã đổi sang ${currentPipSource === 'dock' ? 'Camera Gầm (Dock)' : 'Camera Tay Gắp (Hand)'}`, "info");
}

function togglePipSize() {
  const pip = document.getElementById("map-pip-camera");
  if (!pip) return;
  pip.classList.toggle("pip-enlarged");
}

function togglePipVisibility() {
  const pip = document.getElementById("map-pip-camera");
  if (!pip) return;
  pip.classList.toggle("hidden");
}

function switchCameraFeed(which) {
  currentActiveCamFeed = which;
  const feedDock = document.getElementById("feed-dock-box");
  const feedHand = document.getElementById("feed-hand-box");
  const btnDock = document.getElementById("btn-cam-dock");
  const btnHand = document.getElementById("btn-cam-hand");
  const badge = document.getElementById("camera-current-badge");
  const fpsDock = document.getElementById("dock-cam-fps");
  const fpsHand = document.getElementById("hand-cam-fps");

  if (which === "dock") {
    if (feedDock) feedDock.classList.remove("hidden");
    if (feedHand) feedHand.classList.add("hidden");
    if (btnDock) btnDock.classList.add("active");
    if (btnHand) btnHand.classList.remove("active");
    if (badge) badge.textContent = "DOCK CAMERA (GẦM XE)";
    if (fpsDock) fpsDock.classList.remove("hidden");
    if (fpsHand) fpsHand.classList.add("hidden");
  } else {
    if (feedDock) feedDock.classList.add("hidden");
    if (feedHand) feedHand.classList.remove("hidden");
    if (btnDock) btnDock.classList.remove("active");
    if (btnHand) btnHand.classList.add("active");
    if (badge) badge.textContent = "HAND CAMERA (TAY GẮP SCARA)";
    if (fpsDock) fpsDock.classList.add("hidden");
    if (fpsHand) fpsHand.classList.remove("hidden");
  }
}

function snapshotCurrentCam() {
  snapshotCamera(currentActiveCamFeed);
}

async function pollVisionStatus() {
  const pane = document.getElementById("page-robot") || document.getElementById("tab-vision");
  if (!pane || !pane.classList.contains("active")) return;
  try {
    const res = await fetch("/api/vision/status");
    if (!res.ok) return;
    renderVisionStatus(await res.json());
  } catch (err) {
    /* bỏ qua lỗi mạng tạm thời */
  }
}

function tagHudText(tags, emptyText) {
  if (!tags || tags.length === 0) return emptyText;
  return tags.map((t) => `#${t.id} (${t.dist.toFixed(2)}m)`).join("  ·  ");
}

function renderVisionStatus(v) {
  const dockHud = document.getElementById("dock-tags-hud");
  const handHud = document.getElementById("hand-tags-hud");
  if (dockHud) dockHud.textContent = v.has_dock_feed
    ? tagHudText(v.dock_tags, "Không thấy tag nào")
    : "Chưa có ảnh từ Dock Camera";
  if (handHud) handHud.textContent = v.has_hand_feed
    ? tagHudText(v.hand_tags, "Không thấy tag nào")
    : "Chưa có ảnh từ Hand Camera";

  const pipTagBadge = document.getElementById("pip-tag-badge");
  if (pipTagBadge) {
    const activeTags = currentPipSource === "dock" ? v.dock_tags : v.hand_tags;
    pipTagBadge.textContent = activeTags && activeTags.length > 0
      ? activeTags.map((t) => `#${t.id}`).join(", ")
      : "Quét ArUco...";
  }

  const grasp = document.getElementById("cam-grasp-status");
  if (grasp) grasp.textContent = currentRobot.holding_box ? `Đang giữ hộp #${currentRobot.held_box_tag}` : "Chưa gắp";

  const rows = [];
  (v.dock_tags || []).forEach((t) => rows.push({ ...t, cam: "Dock" }));
  (v.hand_tags || []).forEach((t) => rows.push({ ...t, cam: "Hand" }));

  const badge = document.getElementById("tags-count-badge");
  if (badge) badge.textContent = `${rows.length} thẻ nhìn thấy`;

  const tbody = document.getElementById("vision-tags-table-body");
  if (!tbody) return;
  if (rows.length === 0) {
    tbody.innerHTML = `<tr><td colspan="5" class="text-center">Chưa phát hiện mã ArUco.</td></tr>`;
    return;
  }
  const now = new Date().toLocaleTimeString("vi-VN");
  tbody.innerHTML = rows.map((t) => `
    <tr>
      <td><strong>#${t.id}</strong></td>
      <td>${TAG_TYPE_LABEL[t.type] || t.type}</td>
      <td>${t.cam}</td>
      <td>${t.dist.toFixed(3)} m</td>
      <td>${now}</td>
    </tr>`).join("");
}

function refreshVisionStreams() {
  const ts = Date.now();
  const dock = document.getElementById("dock-stream-img");
  const hand = document.getElementById("hand-stream-img");
  const pip = document.getElementById("pip-stream-img");
  if (dock) dock.src = `/api/vision/dock/stream?t=${ts}`;
  if (hand) hand.src = `/api/vision/hand/stream?t=${ts}`;
  if (pip) pip.src = `/api/vision/${currentPipSource}/stream?t=${ts}`;
  showToast("Đã làm mới các luồng camera", "info");
}

function onStreamError(img, which) {
  setTimeout(() => {
    img.src = `/api/vision/${which}/stream?t=${Date.now()}`;
  }, 3000);
}

function snapshotCamera(which) {
  window.open(`/api/vision/${which}/live.jpg?t=${Date.now()}`, "_blank");
}

// Mở nhanh Trang 2 tạo lệnh Nhập / Xuất từ thẻ Trạm IN / OUT
function quickNavToInbound() {
  if (!isRobotEnabled) {
    showToast("⚠️ Hệ thống đang KHÓA. Vui lòng bật công tắc ENABLE để thao tác trạm!", "warning");
    return;
  }
  switchPage("page-warehouse");
  showToast("Đã chuyển sang Trang 2: Chọn mặt hàng và bấm 'XÁC NHẬN NHẬP KHO'", "info");
}

function quickNavToOutbound() {
  if (!isRobotEnabled) {
    showToast("⚠️ Hệ thống đang KHÓA. Vui lòng bật công tắc ENABLE để thao tác trạm!", "warning");
    return;
  }
  switchPage("page-warehouse");
  showToast("Đã chuyển sang Trang 2: Chọn mặt hàng và bấm 'XÁC NHẬN XUẤT KHO'", "info");
}

async function triggerWebGripper(state) {
  try {
    const res = await fetch("/api/arm/gripper", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ state }),
    });
    const data = await res.json();
    const badge = document.getElementById("manual-gripper-badge");
    if (res.ok && data.success) {
      if (badge) {
        badge.textContent = state ? "Tay kẹp Servo: ĐÓNG (KẸP CHẶT)" : "Tay kẹp Servo: MỞ";
        badge.className = "status-pill " + (state ? "running" : "idle");
      }
      showToast(state ? "Động cơ servo đã đóng kẹp chặt hộp!" : "Động cơ servo đã mở má kẹp nhả hàng!", "info");
    } else {
      showToast(data.message || data.detail || "Lỗi điều khiển tay kẹp", "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để điều khiển tay kẹp!", "error");
  }
}

async function triggerWebPose(pose) {
  try {
    const res = await fetch("/api/arm/pose", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ pose }),
    });
    const data = await res.json();
    if (res.ok) {
      showToast(data.message || `Tay máy → ${pose}`, "info");
    } else {
      showToast(data.detail || "Lỗi điều khiển tay máy", "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để điều khiển tay máy!", "error");
  }
}

async function replenishInboundStation() {
  if (!isRobotEnabled) {
    showToast("⚠️ Hệ thống đang KHÓA. Vui lòng bật công tắc ENABLE để tiếp phôi!", "warning");
    return;
  }
  try {
    const res = await fetch("/api/station/inbound/replenish", { method: "POST" });
    const data = await res.json();
    if (res.ok && data.success) {
      showToast(data.message || "Đã tiếp thêm phôi vào trạm IN!", "success");
      fetchInventory();
    } else {
      showToast(data.message || "Không thể tiếp thêm phôi", "error");
    }
  } catch (err) {
    showToast("Lỗi kết nối tới máy chủ: " + err, "error");
  }
}

async function clearOutboundStation() {
  if (!isRobotEnabled) {
    showToast("⚠️ Hệ thống đang KHÓA. Vui lòng bật công tắc ENABLE để dọn bàn!", "warning");
    return;
  }
  try {
    const res = await fetch("/api/station/outbound/clear", { method: "POST" });
    const data = await res.json();
    if (res.ok && data.success) {
      showToast(data.message || "Đã dọn sạch 6 vị trí trạm OUT!", "success");
      fetchInventory();
    } else {
      showToast(data.message || "Không thể dọn bàn trạm OUT", "error");
    }
  } catch (err) {
    showToast("Lỗi kết nối tới máy chủ: " + err, "error");
  }
}


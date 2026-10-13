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
  origin: [-3.5, -2.5, 0.0],
  origin_x: -3.5,
  origin_y: -2.5,
  width: 280,
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
  fetchMode();
  connectWebSocket();
  fetchInventory();
  fetchItems();
  fetchTasks();
  fetchTransactions();
  setInterval(pollVisionStatus, 1000);

  // Polling dự phòng nhẹ (mỗi 10s làm mới bảng nếu websocket có độ trễ)
  setInterval(() => {
    fetchMode();
    fetchInventory();
    fetchTasks();
  }, 10000);
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
      nextImg.src = `/api/map/live.png?v=${data.version}`;
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
    updateRobotUI();
    drawMap();
  } else if (type === "inventory") {
    inventoryData = data;
    renderInventoryView();
  } else if (type === "task") {
    updateActiveTaskView(data);
    fetchTasks();
  } else if (type === "notification") {
    addNotification(data.level, data.text);
    showToast(data.text, data.level);
  }
}

// ── 2. CẬP NHẬT GIAO DIỆN ROBOT ─────────────────────────────────────────────
function updateRobotUI() {
  // Trạng thái Robot
  const stateBadge = document.getElementById("robot-state-badge");
  stateBadge.textContent = currentRobot.state || "IDLE";
  stateBadge.className = "status-pill " + (
    currentRobot.state === "ESTOP" ? "estop" :
    currentRobot.state === "IDLE" ? "idle" : "running"
  );

  document.getElementById("robot-task-id").textContent = currentRobot.task_id > 0 ? `#${currentRobot.task_id}` : "--";
  document.getElementById("robot-slot-id").textContent = currentRobot.slot_id || "--";
  document.getElementById("robot-holding-box").textContent = currentRobot.holding_box ? `Hộp tag #${currentRobot.held_box_tag}` : "Không";

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
  const orderForm = document.getElementById("order-form");
  const slamBadge = document.getElementById("slam-status-badge");

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
    if (orderForm) {
      orderForm.querySelectorAll("button, select, input").forEach((el) => {
        el.disabled = true;
      });
    }
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
    if (orderForm) {
      orderForm.querySelectorAll("button, select, input").forEach((el) => {
        el.disabled = false;
      });
    }
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
        mapImg.src = data.image_url || `/api/map/live.png?v=${data.version}`;
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
          mapImg.src = d.image_url || "/api/map/image";
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
    drawPickupTable(ctx, -2.40, 1.20, "TRẠM IN #30", "#10b981");
    drawPickupTable(ctx, -2.40, -1.20, "TRẠM OUT #31", "#f59e0b");
    drawStation(ctx, -2.40, 0.88, "work IN", "#34d399");
    drawStation(ctx, -2.40, -0.88, "work OUT", "#fbbf24");
    drawStation(ctx, -2.40, 0.00, "HOME", "#6366f1");
    drawRacksOnMap(ctx);
  }

  // 6. Lớp Robot AGV-SCARA
  if (activeLayers.robot) {
    drawRobotOnMap(ctx);
  }

  ctx.restore();
}

function drawPickupTable(ctx, x, y, label, color) {
  const { px, py } = worldToCanvas(x, y);
  ctx.save();
  ctx.fillStyle = color + "55";
  ctx.strokeStyle = color;
  ctx.lineWidth = 1.2;
  ctx.fillRect(px - 12, py - 7, 24, 14);
  ctx.strokeRect(px - 12, py - 7, 24, 14);
  // Hộp hàng trên bàn
  ctx.fillStyle = color;
  ctx.fillRect(px - 4, py - 4, 8, 8);
  ctx.font = "bold 7px sans-serif";
  ctx.fillStyle = "#fff";
  ctx.textAlign = "center";
  ctx.fillText(label, px, py + (y > 0 ? -10 : 17));
  ctx.restore();
}

function drawStation(ctx, x, y, label, color) {
  const { px, py } = worldToCanvas(x, y);
  ctx.save();
  ctx.fillStyle = color;
  ctx.beginPath();
  ctx.arc(px, py, 4, 0, Math.PI * 2);
  ctx.fill();

  ctx.font = "8px sans-serif";
  ctx.fillStyle = "#fff";
  ctx.textAlign = "center";
  ctx.fillText(label, px, py - 6);
  ctx.restore();
}

function drawRacksOnMap(ctx) {
  const cols = [0.75, 1.25, 1.75];
  const racks = [
    { name: "A", y: 0.60, color: "rgba(0, 210, 255, 0.4)" },
    { name: "B", y: -0.60, color: "rgba(245, 158, 11, 0.4)" },
  ];

  ctx.save();
  racks.forEach((rack) => {
    cols.forEach((colX, colIdx) => {
      const { px, py } = worldToCanvas(colX, rack.y);
      ctx.fillStyle = rack.color;
      ctx.strokeStyle = "#fff";
      ctx.lineWidth = 0.5;

      // Vẽ hình chữ nhật ô kệ
      ctx.fillRect(px - 6, py - 4, 12, 8);
      ctx.strokeRect(px - 6, py - 4, 12, 8);

      ctx.font = "7px sans-serif";
      ctx.fillStyle = "#cbd5e1";
      ctx.textAlign = "center";
      ctx.fillText(`${rack.name}${colIdx}`, px, py + 12);
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
    : "rgba(0, 210, 255, 0.25)";
  ctx.fill();

  // Thân robot dạng tam giác mũi tên
  ctx.beginPath();
  ctx.moveTo(8, 0);       // Mũi trước
  ctx.lineTo(-6, -5);     // Góc sau trái
  ctx.lineTo(-4, 0);      // Lõm đuôi
  ctx.lineTo(-6, 5);      // Góc sau phải
  ctx.closePath();

  ctx.fillStyle = currentRobot.state === "ESTOP" ? "#ef4444" : "#00f0ff";
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

  // 4. Cập nhật danh sách ô trống vào dropdown ô đích nhập hàng
  updateInboundSlotOptions(slots);
}

function updateInboundSlotOptions(slots) {
  const slotSelect = document.getElementById("order-slot-id");
  if (!slotSelect) return;
  const currentVal = slotSelect.value;
  const emptySlots = (slots || []).filter((s) => s.state === "EMPTY");
  
  let html = `<option value="">-- Tự động chọn ô tối ưu (gần trạm nhất) --</option>`;
  emptySlots.forEach((s) => {
    const rackName = (s.rack || "").toLowerCase().includes("a") ? "Kệ A" : "Kệ B";
    html += `<option value="${s.slot_id}">${s.slot_id} (${rackName}, Cột ${s.col + 1}, Tầng ${s.tier})</option>`;
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
      <div class="slot-card ${stateClass}" onclick="showSlotDetail('${s.slot_id}', '${s.state}', '${s.item_code || ''}', ${s.quantity})">
        <span class="slot-name">${s.slot_id}</span>
        <span class="slot-tag">Tag #${s.slot_tag_id}</span>
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

function showSlotDetail(slotId, state, itemCode, qty) {
  if (state === "EMPTY") {
    const slotSelect = document.getElementById("order-slot-id");
    if (slotSelect) {
      slotSelect.value = slotId;
    }
    showToast(`Đã chọn ô trống ${slotId} làm ô đích nhập hàng trong form tạo đơn!`, "info");
  } else {
    showToast(`Ô ${slotId}: Trạng thái ${state}${itemCode ? ` (Chứa ${qty} hộp ${itemCode})` : ""}`, "info");
  }
}

// ── 5. FORM NHẬP / XUẤT HÀNG (Mục 8.4 Tab 3) ───────────────────────────────
async function fetchItems() {
  try {
    const res = await fetch("/api/items");
    if (!res.ok) return;
    itemsList = await res.json();
    const select = document.getElementById("order-item-code");
    if (!select) return;

    select.innerHTML = itemsList.map((it) =>
      `<option value="${it.item_code}">${it.item_code} – ${it.name} (Tồn: ${it.current_stock})</option>`
    ).join("");

    updateItemPreview();
  } catch (err) {
    console.warn("Không thể tải danh mục mặt hàng:", err);
  }
}

function updateItemPreview() {
  const select = document.getElementById("order-item-code");
  const itemCode = select ? select.value : "";
  const found = itemsList.find((it) => it.item_code === itemCode);

  document.getElementById("preview-stock").textContent = found ? `${found.current_stock} hộp` : "0";

  const freeSlots = inventoryData?.slot_stats?.empty_count ?? 0;
  document.getElementById("preview-free-slots").textContent = `${freeSlots} ô`;
}

function changeQty(delta) {
  const input = document.getElementById("order-quantity");
  let val = parseInt(input.value, 10) + delta;
  val = Math.max(1, Math.min(10, val));
  input.value = val;
}

function getSelectedPriority() {
  const radios = document.getElementsByName("order-priority");
  for (let r of radios) {
    if (r.checked) return parseInt(r.value, 10);
  }
  return 1;
}

async function submitInboundOrder() {
  const itemCode = document.getElementById("order-item-code").value;
  const quantity = parseInt(document.getElementById("order-quantity").value, 10);
  const slotSelect = document.getElementById("order-slot-id");
  const slotId = slotSelect && slotSelect.value ? slotSelect.value : null;

  if (!itemCode) {
    showToast("Vui lòng chọn mặt hàng nhập!", "warn");
    return;
  }

  const payload = { item_code: itemCode, quantity: quantity };
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
      showToast(`Đã tạo đơn nhập thành công: ${data.message}`, "info");
      fetchInventory();
      fetchItems();
      fetchTasks();
      switchTabById("tab-tasks");
    } else {
      showToast(`Lỗi ${res.status}: ${data.detail || data.message}`, "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để nhập hàng!", "error");
  }
}

async function submitOutboundOrder() {
  const itemCode = document.getElementById("order-item-code").value;
  const quantity = parseInt(document.getElementById("order-quantity").value, 10);
  if (!itemCode) {
    showToast("Vui lòng chọn mặt hàng xuất!", "warn");
    return;
  }

  try {
    const res = await fetch("/api/tasks/outbound", {
      method: "POST",
      headers: { "Content-Type": "application/json" },
      body: JSON.stringify({ item_code: itemCode, quantity: quantity }),
    });
    const data = await res.json();
    if (res.ok) {
      showToast(`Đã tạo đơn xuất thành công: ${data.message}`, "info");
      fetchInventory();
      fetchItems();
      fetchTasks();
      switchTabById("tab-tasks");
    } else {
      showToast(`Lỗi ${res.status}: ${data.detail || data.message}`, "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để xuất hàng!", "error");
  }
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

// ── 7. LỊCH SỬ GIAO DỊCH & THÔNG BÁO (Mục 8.4 Tab 5) ────────────────────────
async function fetchTransactions() {
  try {
    const res = await fetch("/api/transactions?limit=30");
    if (!res.ok) return;
    const txs = await res.json();
    const tbody = document.getElementById("tx-table-body");
    if (!tbody) return;

    if (txs.length === 0) {
      tbody.innerHTML = `<tr><td colspan="6" class="text-center">Chưa có giao dịch kho.</td></tr>`;
      return;
    }

    tbody.innerHTML = txs.map((tx) => {
      const isAdd = tx.delta > 0;
      const deltaClass = isAdd ? "text-success" : "text-danger";
      return `
        <tr>
          <td>#${tx.task_id}</td>
          <td>${tx.type}</td>
          <td><strong>${tx.item_code}</strong></td>
          <td>${tx.slot_id}</td>
          <td class="${deltaClass}"><strong>${isAdd ? "+" : ""}${tx.delta}</strong></td>
          <td>${(tx.ts || "").slice(11, 19)}</td>
        </tr>
      `;
    }).join("");
  } catch (err) {
    console.warn("Không thể tải lịch sử giao dịch:", err);
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
      showToast("Khôi phục thành công! Robot sẵn sàng.", "info");
      currentRobot.state = "IDLE";
      updateRobotUI();
      drawMap();
    } else {
      showToast("Lỗi khi khôi phục E-Stop!", "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để khôi phục!", "error");
  }
}

// ── 9. TIỆN ÍCH CHUYỂN TAB & TOAST ──────────────────────────────────────────
function switchTabById(tabId) {
  const btn = document.querySelector(`.nav-item[onclick*="'${tabId}'"]`);
  switchTab(tabId, btn);
}

function switchTab(tabId, navBtn) {
  // Ẩn tất cả panes
  document.querySelectorAll(".tab-pane").forEach((pane) => {
    pane.classList.remove("active");
  });
  // Hiện pane được chọn
  const activePane = document.getElementById(tabId);
  if (activePane) activePane.classList.add("active");

  // Đổi active nav button
  document.querySelectorAll(".nav-item").forEach((btn) => {
    btn.classList.remove("active");
  });
  if (navBtn) navBtn.classList.add("active");

  // Nếu chuyển về tab Overview thì vẽ lại canvas cho chuẩn kích thước
  if (tabId === "tab-overview") {
    setTimeout(drawMap, 50);
  }
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
  stopTeleop();
  activeTeleopCmd = { vx, vy, wz };
  sendTeleop(vx, vy, wz);
  // Gửi lặp lại 10 Hz (mỗi 100ms) để nuôi Watchdog phía server (Bài 8b.5)
  teleopTimer = setInterval(() => {
    sendTeleop(activeTeleopCmd.vx, activeTeleopCmd.vy, activeTeleopCmd.wz);
  }, 100);
}

function stopTeleop() {
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

// ── 14. THỊ GIÁC (CAMERA STREAMS) & TRẠM IN/OUT & TAY MÁY THỦ CÔNG ───────────
const TAG_TYPE_LABEL = { shelf: "Ô kệ", station: "Trạm IN/OUT", box: "Hộp hàng" };

async function pollVisionStatus() {
  // Chỉ gọi API khi đang mở tab Thị giác để tiết kiệm băng thông
  const pane = document.getElementById("tab-vision");
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
    tbody.innerHTML = `<tr><td colspan="5" class="text-center">Chưa phát hiện mã ArUco nào trong tầm nhìn hiện tại.</td></tr>`;
    return;
  }
  const now = new Date().toLocaleTimeString("vi-VN");
  tbody.innerHTML = rows.map((t) => `
    <tr>
      <td><strong>#${t.id}</strong></td>
      <td>${TAG_TYPE_LABEL[t.type] || t.type}</td>
      <td>${t.cam}</td>
      <td>${t.dist.toFixed(3)} m</td>
      <td>${t.reproj_error} px • ${now}</td>
    </tr>`).join("");
}

function refreshVisionStreams() {
  const ts = Date.now();
  const dock = document.getElementById("dock-stream-img");
  const hand = document.getElementById("hand-stream-img");
  if (dock) dock.src = `/api/vision/dock/stream?t=${ts}`;
  if (hand) hand.src = `/api/vision/hand/stream?t=${ts}`;
  showToast("Đã làm mới luồng camera", "info");
}

function onStreamError(img, which) {
  // Thử nối lại luồng MJPEG sau 3 giây
  setTimeout(() => {
    img.src = `/api/vision/${which}/stream?t=${Date.now()}`;
  }, 3000);
}

function snapshotCamera(which) {
  window.open(`/api/vision/${which}/live.jpg?t=${Date.now()}`, "_blank");
}

// Mở nhanh tab tạo lệnh Nhập / Xuất từ thẻ Trạm IN / OUT
function quickNavToInbound() {
  switchTabById("tab-orders");
  showToast("Chọn mặt hàng rồi bấm 'LẤY HÀNG TRẠM IN BỎ LÊN KỆ'", "info");
}

function quickNavToOutbound() {
  switchTabById("tab-orders");
  showToast("Chọn mặt hàng rồi bấm 'LẤY HÀNG TỪ KỆ XUẤT RA TRẠM OUT'", "info");
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
        badge.textContent = state ? "Tay gắp: ON" : "Tay gắp: OFF";
        badge.className = "status-pill " + (state ? "running" : "idle");
      }
      showToast(data.message, "info");
    } else {
      showToast(data.message || data.detail || "Lỗi điều khiển tay gắp", "error");
    }
  } catch (err) {
    showToast("Không thể kết nối máy chủ để điều khiển tay gắp!", "error");
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

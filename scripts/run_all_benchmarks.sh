#!/usr/bin/env bash
# ==============================================================================
# scripts/run_all_benchmarks.sh
# Kịch bản chạy toàn bộ bộ đo chỉ tiêu hệ thống Warehouse AGV-SCARA (Mục 9.3)
# Hỗ trợ: --quick (thử nhanh), --full (chạy đủ số lượt mặc định), hoặc số lần tùy chọn.
# ==============================================================================

set -e

WS_DIR="/home/lehuythien/warehouse_ws"
RESULTS_DIR="${WS_DIR}/docs/results"
cd "${WS_DIR}"

mkdir -p "${RESULTS_DIR}"

# Source ROS 2 environment
if [ -f "/opt/ros/humble/setup.bash" ]; then
    source /opt/ros/humble/setup.bash
fi
if [ -f "${WS_DIR}/install/setup.bash" ]; then
    source "${WS_DIR}/install/setup.bash"
fi

# Parsing arguments
MODE="full"
NAV_TRIALS=20
DOCK_RUNS=30
PICK_RUNS=30

while [[ $# -gt 0 ]]; do
    case "$1" in
        --quick|--fast)
            MODE="quick"
            NAV_TRIALS=2
            DOCK_RUNS=2
            PICK_RUNS=1
            shift
            ;;
        --full)
            MODE="full"
            NAV_TRIALS=20
            DOCK_RUNS=30
            PICK_RUNS=30
            shift
            ;;
        --nav-trials)
            NAV_TRIALS="$2"
            shift 2
            ;;
        --dock-runs)
            DOCK_RUNS="$2"
            shift 2
            ;;
        --pick-runs)
            PICK_RUNS="$2"
            shift 2
            ;;
        *)
            echo "Tuỳ chọn không hợp lệ: $1"
            echo "Sử dụng: $0 [--quick|--full] [--nav-trials N] [--dock-runs N] [--pick-runs N]"
            exit 1
            ;;
    esac
done

echo "======================================================================"
echo "          CHẠY BỘ ĐO CHỈ TIÊU TOÀN DIỆN KHO HÀNG (MỤC 9.3)           "
echo "======================================================================"
echo "Chế độ:         ${MODE}"
echo "Nav trials:     ${NAV_TRIALS}"
echo "Docking runs:   ${DOCK_RUNS}"
echo "Pick/tier runs: ${PICK_RUNS}"
echo "Thư mục xuất:   ${RESULTS_DIR}"
echo "======================================================================"

# 1. Chạy các benchmark tính toán & phân tích tài nguyên (độc lập hoặc phân tích dữ liệu)
echo ""
echo ">>> [1/5] Chạy benchmark Odometry Drift (Odom thô vs EKF vs AMCL vs Ground Truth)..."
python3 scripts/benchmark_odometry_drift.py

echo ""
echo ">>> [2/5] Chạy benchmark FSM Timing Breakdown & System Resources..."
python3 scripts/benchmark_fsm_and_resources.py

# Kiểm tra xem Gazebo mô phỏng có đang chạy hay không để chạy các bài test tương tác
if pgrep -f gzserver > /dev/null 2>&1; then
    echo ""
    echo ">>> Phát hiện mô phỏng Gazebo đang hoạt động. Tiếp tục chạy các bài benchmark tương tác..."

    echo ""
    echo ">>> [3/5] Chạy benchmark Điều hướng Nav2 (${NAV_TRIALS} lượt)..."
    python3 scripts/benchmark_nav.py --trials "${NAV_TRIALS}" || echo "[CẢNH BÁO] benchmark_nav.py có cảnh báo/lỗi nhưng tiếp tục."

    echo ""
    echo ">>> [4/5] Chạy benchmark Docking Visual Servoing (${DOCK_RUNS} lượt)..."
    python3 scripts/benchmark_docking.py --runs "${DOCK_RUNS}" || echo "[CẢNH BÁO] benchmark_docking.py có cảnh báo/lỗi nhưng tiếp tục."

    echo ""
    echo ">>> [5/5] Chạy benchmark SCARA Pick & Place (${PICK_RUNS} lượt/tầng)..."
    python3 scripts/benchmark_pick.py --runs-per-tier "${PICK_RUNS}" || echo "[CẢNH BÁO] benchmark_pick.py có cảnh báo/lỗi nhưng tiếp tục."

else
    echo ""
    echo "======================================================================"
    echo "[LƯU Ý] Hệ thống ROS 2/Gazebo chưa được khởi động trước lệnh này."
    echo "Các biểu đồ phân tích tĩnh, FSM, tài nguyên và sai lệch Odom đã được tạo."
    echo "Để chạy đầy đủ tương tác Nav2/Docking/Pick-Place trong mô phỏng:"
    echo "  Terminal 1: ros2 launch wh_bringup full_system.launch.py"
    echo "  Terminal 2: ./scripts/wait_ready.py"
    echo "  Terminal 3: ./scripts/run_all_benchmarks.sh [chế độ]"
    echo "======================================================================"
fi

echo ""
echo "======================================================================"
echo "          TỔNG KẾT DANH MỤC KẾT QUẢ ĐÃ TẠO TẠI docs/results/          "
echo "======================================================================"
ls -lh "${RESULTS_DIR}"
echo "======================================================================"
echo "HOÀN TẤT BỘ ĐO CHỈ TIÊU!"

#!/usr/bin/env bash
# Cài toàn bộ phụ thuộc để build + chạy dự án diff_robot_v3 (Ubuntu 22.04 + ROS 2 Humble).
#
#   ./setup/install_dependencies.sh                  cài đủ để build và chạy robot + web
#   ./setup/install_dependencies.sh --with-platformio   thêm PlatformIO (nạp firmware ESP32 từ máy này)
#   ./setup/install_dependencies.sh --with-mesh-tools   thêm open3d cho a3_description/scripts (tuỳ chọn)
#   ./setup/install_dependencies.sh --dry-run           chỉ in các lệnh sẽ chạy
#
# Chạy lại nhiều lần được (apt bỏ qua gói đã có). Không cần chạy bằng sudo - script tự gọi sudo khi cần.
set -euo pipefail

ROS_DISTRO_NAME=humble
WITH_PIO=0
WITH_MESH=0
DRY=0

for arg in "$@"; do
    case "$arg" in
        --with-platformio) WITH_PIO=1 ;;
        --with-mesh-tools) WITH_MESH=1 ;;
        --dry-run) DRY=1 ;;
        -h|--help) sed -n '2,9p' "$0" | sed 's/^# \{0,1\}//'; exit 0 ;;
        *) echo "Tham số không hợp lệ: $arg (xem --help)" >&2; exit 2 ;;
    esac
done

WS_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"   # thư mục gốc workspace (chứa src/)
log()  { printf '\n\033[1;34m==> %s\033[0m\n' "$*"; }
warn() { printf '\033[1;33m[!] %s\033[0m\n' "$*" >&2; }
run()  { if [ "$DRY" = 1 ]; then echo "+ $*"; else "$@"; fi; }
SUDO=""; [ "$(id -u)" -ne 0 ] && SUDO="sudo"

# ---------------------------------------------------------------- kiểm tra hệ điều hành
. /etc/os-release
if [ "${VERSION_CODENAME:-}" != "jammy" ]; then
    warn "Dự án được thử trên Ubuntu 22.04 (jammy) + ROS 2 Humble; bạn đang dùng ${PRETTY_NAME:-không rõ}."
    read -r -p "Vẫn tiếp tục? [y/N] " ans; [[ "$ans" =~ ^[Yy]$ ]] || exit 1
fi

# ---------------------------------------------------------------- 1. ROS 2 Humble (nếu chưa có)
if [ ! -f "/opt/ros/$ROS_DISTRO_NAME/setup.bash" ]; then
    log "Chưa có ROS 2 $ROS_DISTRO_NAME -> thêm kho apt và cài ros-$ROS_DISTRO_NAME-desktop"
    run $SUDO apt-get update
    run $SUDO apt-get install -y software-properties-common curl gnupg lsb-release
    run $SUDO add-apt-repository -y universe
    run $SUDO curl -sSL https://raw.githubusercontent.com/ros/rosdistro/master/ros.key \
        -o /usr/share/keyrings/ros-archive-keyring.gpg
    if [ "$DRY" = 0 ]; then
        echo "deb [arch=$(dpkg --print-architecture) signed-by=/usr/share/keyrings/ros-archive-keyring.gpg] http://packages.ros.org/ros2/ubuntu $(. /etc/os-release && echo "$UBUNTU_CODENAME") main" \
            | $SUDO tee /etc/apt/sources.list.d/ros2.list >/dev/null
    fi
    run $SUDO apt-get update
    run $SUDO apt-get install -y "ros-$ROS_DISTRO_NAME-desktop"
else
    log "Đã có ROS 2 $ROS_DISTRO_NAME"
fi

# ---------------------------------------------------------------- 2. công cụ build
log "Công cụ build (colcon, rosdep, git, python)"
run $SUDO apt-get update
run $SUDO apt-get install -y \
    build-essential cmake git python3-pip python3-colcon-common-extensions python3-rosdep python3-vcstool

# ---------------------------------------------------------------- 3. gói ROS + Python dùng trong dự án
# Liệt kê tường minh (khớp với package.xml) để không phụ thuộc rosdep tìm đúng; rosdep ở bước 4 chỉ để bắt phần sót.
log "Gói ROS 2 + Python cho robot, SLAM, Nav2 và web"
ROS_PKGS=(
    # a3_bringup / a3_driver / a3_description
    rplidar-ros laser-filters robot-localization
    robot-state-publisher joint-state-publisher joint-state-publisher-gui xacro urdf rviz2
    teleop-twist-keyboard gazebo-ros-pkgs
    # atlas_slam + a3_web: SLAM Toolbox + Nav2 (đủ controller DWB / RPP / MPPI, map_server, AMCL)
    slam-toolbox navigation2 nav2-bringup nav2-common nav2-map-server
    nav2-regulated-pure-pursuit-controller nav2-mppi-controller dwb-core
    # rf2o_laser_odometry (build từ source trong src/)
    eigen3-cmake-module tf2-geometry-msgs
    # message / interface
    rosidl-default-generators rosidl-default-runtime
)
APT_PKGS=("${ROS_PKGS[@]/#/ros-$ROS_DISTRO_NAME-}")
APT_PKGS+=(
    libeigen3-dev
    python3-serial      # a3_driver: cổng USB tới ESP32
    python3-aiohttp     # a3_web: server REST + WebSocket
    python3-numpy python3-yaml python3-pytest
)
run $SUDO apt-get install -y "${APT_PKGS[@]}"

# ---------------------------------------------------------------- 4. rosdep (bắt các phụ thuộc còn sót theo package.xml)
log "rosdep: đối chiếu với package.xml trong src/"
if [ ! -f /etc/ros/rosdep/sources.list.d/20-default.list ]; then
    run $SUDO rosdep init || warn "rosdep init lỗi (có thể đã init) - bỏ qua"
fi
run rosdep update || warn "rosdep update lỗi (mạng?) - bỏ qua"
if [ "$DRY" = 1 ]; then
    echo "+ rosdep install --from-paths $WS_DIR/src --ignore-src -r -y --rosdistro $ROS_DISTRO_NAME"
else
    # shellcheck disable=SC1090
    (. "/opt/ros/$ROS_DISTRO_NAME/setup.bash"
     rosdep install --from-paths "$WS_DIR/src" --ignore-src -r -y --rosdistro "$ROS_DISTRO_NAME") \
        || warn "rosdep còn báo thiếu vài khoá (xem log ở trên) - thường vô hại nếu các gói ở bước 3 đã cài"
fi

# ---------------------------------------------------------------- 5. quyền truy cập cổng serial (ESP32, RPLidar)
if ! id -nG "${SUDO_USER:-$USER}" | tr ' ' '\n' | grep -qx dialout; then
    log "Thêm ${SUDO_USER:-$USER} vào nhóm dialout (đọc /dev/ttyUSB*) - cần đăng xuất/đăng nhập lại"
    run $SUDO usermod -aG dialout "${SUDO_USER:-$USER}"
else
    log "Đã có quyền dialout"
fi

# ---------------------------------------------------------------- 6. tuỳ chọn
if [ "$WITH_PIO" = 1 ]; then
    log "PlatformIO (nạp firmware src/diff_drive_ros)"
    run python3 -m pip install --user --upgrade platformio
    warn "Đảm bảo ~/.local/bin nằm trong PATH để dùng lệnh 'pio'"
fi
if [ "$WITH_MESH" = 1 ]; then
    log "open3d (scripts giảm mesh trong a3_description/scripts)"
    run python3 -m pip install --user --upgrade open3d
fi

# ---------------------------------------------------------------- xong
cat <<EOF

Xong. Bước tiếp theo:

    source /opt/ros/$ROS_DISTRO_NAME/setup.bash
    cd $WS_DIR && colcon build --symlink-install
    source install/setup.bash
    ros2 launch a3_web web.launch.py          # rồi mở http://localhost:8080

Lưu ý:
  - Nếu vừa được thêm vào nhóm dialout: đăng xuất/đăng nhập lại (hoặc khởi động lại) mới có hiệu lực.
  - a3_driver mặc định mở /dev/esp32: tạo udev symlink cho ESP32 hoặc chạy với tham số serial_port:=/dev/ttyUSB0
    (xem src/diff_drive_ros/README.md).
EOF

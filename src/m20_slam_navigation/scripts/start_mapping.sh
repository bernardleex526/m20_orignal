#!/usr/bin/env bash
set -euo pipefail

SCRIPT_PATH="$(readlink -f -- "${BASH_SOURCE[0]}")"
SCRIPT_DIR="$(cd -- "$(dirname -- "${SCRIPT_PATH}")" && pwd)"
WORKSPACE="$(cd -- "${SCRIPT_DIR}/../../.." && pwd)"

MAP_NAME="m20_map"
OUTPUT_DIR=""
BAG_PATH=""
USE_RVIZ="true"
FORCE_BUILD="false"
SKIP_BUILD="false"
DOMAIN_ID="${ROS_DOMAIN_ID:-0}"
LAUNCH_PID=""
BAG_PID=""
DRDDS_PID=""
CLEANED_UP="false"

usage() {
  printf '%s\n' \
    "Usage: $0 [options]" \
    "  --map-name NAME       map directory prefix (default: m20_map)" \
    "  --output-dir DIR      exact output directory" \
    "  --bag PATH            replay a ROS 2 bag instead of waiting for live topics" \
    "  --domain ID           ROS_DOMAIN_ID (default: current or 0)" \
    "  --no-rviz             do not start RViz" \
    "  --build               force a clean package rebuild" \
    "  --skip-build          use the existing install space" \
    "  -h, --help            show this help"
}

while (($#)); do
  case "$1" in
    --map-name)
      MAP_NAME="${2:?missing map name}"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="${2:?missing output directory}"
      shift 2
      ;;
    --bag)
      BAG_PATH="${2:?missing bag path}"
      shift 2
      ;;
    --domain)
      DOMAIN_ID="${2:?missing domain ID}"
      shift 2
      ;;
    --no-rviz)
      USE_RVIZ="false"
      shift
      ;;
    --build)
      FORCE_BUILD="true"
      shift
      ;;
    --skip-build)
      SKIP_BUILD="true"
      shift
      ;;
    -h|--help)
      usage
      exit 0
      ;;
    *)
      printf 'Unknown option: %s\n' "$1" >&2
      usage >&2
      exit 2
      ;;
  esac
done

source_ros() {
  set +u
  if [[ -f /opt/robot/scripts/setup_ros2.sh ]]; then
    source /opt/robot/scripts/setup_ros2.sh
  elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
    source "/opt/ros/${ROS_DISTRO}/setup.bash"
  elif [[ -f /opt/ros/foxy/setup.bash ]]; then
    source /opt/ros/foxy/setup.bash
  elif [[ -f /opt/ros/humble/setup.bash ]]; then
    source /opt/ros/humble/setup.bash
  else
    printf 'ROS 2 setup was not found.\n' >&2
    exit 1
  fi
  set -u
}

source_ros
export ROS_DOMAIN_ID="${DOMAIN_ID}"
export RMW_IMPLEMENTATION="${RMW_IMPLEMENTATION:-rmw_fastrtps_cpp}"

if [[ ! -d "${WORKSPACE}/src/m20_slam_navigation" ]]; then
  printf 'Cannot locate workspace from script path: %s\n' "${WORKSPACE}" >&2
  exit 1
fi
if [[ -n "${BAG_PATH}" && ! -e "${BAG_PATH}" ]]; then
  printf 'Bag path does not exist: %s\n' "${BAG_PATH}" >&2
  exit 1
fi

if [[ -z "${OUTPUT_DIR}" ]]; then
  OUTPUT_DIR="${WORKSPACE}/maps/${MAP_NAME}-$(date +%Y%m%d-%H%M%S)"
fi
mkdir -p "${OUTPUT_DIR}"
MAP_PATH="${OUTPUT_DIR}/full_cloud.pcd"

expected_slam_binary="${WORKSPACE}/install/m20_slam_navigation/lib/m20_slam_navigation/slam_node"
running_slam_pids=()
while read -r pid; do
  [[ -z "${pid}" ]] && continue
  if [[ "$(readlink -f "/proc/${pid}/exe" 2>/dev/null || true)" == "${expected_slam_binary}" ]]; then
    running_slam_pids+=("${pid}")
  fi
done < <(pgrep -x slam_node 2>/dev/null || true)
if ((${#running_slam_pids[@]} > 0)); then
  printf 'A slam_node process is already running. Stop it before starting another mapper.\n' >&2
  ps -o pid,ppid,stat,args -p "${running_slam_pids[@]}" >&2 || true
  exit 1
fi

if pgrep -x slam_ddsnode >/dev/null 2>&1; then
  printf 'The vendor slam_ddsnode is running; stop its mapping service before using the same topics and TF.\n' >&2
  pgrep -af '(^|/)slam_ddsnode( |$)' >&2 || true
  exit 1
fi

if [[ "${SKIP_BUILD}" != "true" ]] && {
  [[ "${FORCE_BUILD}" == "true" ]] ||
  [[ ! -f "${WORKSPACE}/install/m20_slam_navigation/share/m20_slam_navigation/package.sh" ]];
}; then
  printf 'Building m20_slam_navigation in %s\n' "${WORKSPACE}"
  (
    cd "${WORKSPACE}"
    colcon build --symlink-install --packages-select m20_slam_navigation \
      --cmake-args -DCMAKE_BUILD_TYPE=Release
  )
fi

if [[ ! -f "${WORKSPACE}/install/setup.bash" ]]; then
  printf 'Missing install/setup.bash; run with --build first.\n' >&2
  exit 1
fi
set +u
source "${WORKSPACE}/install/setup.bash"
set -u

wait_for_topic() {
  local topic="$1"
  local expected_type="$2"
  local attempts=30
  while ((attempts > 0)); do
    local actual_type
    actual_type="$(ROS2CLI_NO_DAEMON=1 ros2 topic type "${topic}" 2>/dev/null || true)"
    if [[ "${actual_type}" == "${expected_type}" ]]; then
      return 0
    fi
    attempts=$((attempts - 1))
    sleep 1
  done
  printf 'Required topic unavailable or wrong type: %s (expected %s)\n' \
    "${topic}" "${expected_type}" >&2
  return 1
}

wait_for_subscriber() {
  local topic="$1"
  local attempts=30
  while ((attempts > 0)); do
    local subscribers
    subscribers="$(ROS2CLI_NO_DAEMON=1 ros2 topic info -v "${topic}" 2>/dev/null \
      | awk -F': ' '/Subscription count:/ {print $2; exit}' || true)"
    if [[ "${subscribers:-0}" =~ ^[0-9]+$ ]] && (( subscribers >= 1 )); then
      return 0
    fi
    attempts=$((attempts - 1))
    sleep 1
  done
  printf 'Mapper did not subscribe to %s before bag playback\n' "${topic}" >&2
  return 1
}

cleanup() {
  if [[ "${CLEANED_UP}" == "true" ]]; then
    return
  fi
  CLEANED_UP="true"
  # An SSH client can close the PTY immediately after forwarding Ctrl+C.  Do
  # not let the resulting HUP (or a repeated INT/TERM) interrupt map saving
  # and exact child cleanup halfway through.
  trap '' HUP INT TERM
  # Also arm a detached fallback before doing any foreground cleanup.  Some
  # M20Pro SSH sessions are reaped by sshd after the save response, regardless
  # of the shell's HUP trap.  Capture only the mapper PIDs that exist now and
  # verify /proc/<pid>/exe before signalling them, so a later mapper instance
  # and the native /opt/robot slam_ddsnode cannot be touched.
  local expected_slam_binary
  expected_slam_binary="${WORKSPACE}/install/m20_slam_navigation/lib/m20_slam_navigation/slam_node"
  local watchdog_slam_pids=()
  mapfile -t watchdog_slam_pids < <(
    pgrep -f "^${expected_slam_binary}( |$)" 2>/dev/null || true)
  if ((${#watchdog_slam_pids[@]} > 0)); then
    setsid /bin/bash -c '
      trap "" HUP INT TERM
      expected="$1"
      shift
      sleep 2
      timeout 180 ros2 service call /m20_slam/save_map std_srvs/srv/Trigger "{}" \
        >/dev/null 2>&1 || true
      for pid in "$@"; do
        if [[ "$(readlink -f "/proc/${pid}/exe" 2>/dev/null || true)" == "${expected}" ]]; then
          kill -INT "${pid}" 2>/dev/null || true
        fi
      done
      sleep 3
      for pid in "$@"; do
        if [[ "$(readlink -f "/proc/${pid}/exe" 2>/dev/null || true)" == "${expected}" ]]; then
          kill -TERM "${pid}" 2>/dev/null || true
        fi
      done
    ' m20-cleanup "${expected_slam_binary}" "${watchdog_slam_pids[@]}" \
      </dev/null >/dev/null 2>&1 &
  fi
  printf '\nSaving map to %s\n' "${MAP_PATH}"
  if [[ -n "${BAG_PATH}" ]]; then
    sleep 2
  fi
  timeout 180 ros2 service call /m20_slam/save_map std_srvs/srv/Trigger '{}' || true
  stop_group() {
    local process_group="$1"
    [[ -z "${process_group}" ]] && return
    kill -INT -- "-${process_group}" 2>/dev/null || true
    for _ in {1..20}; do
      if ! kill -0 -- "-${process_group}" 2>/dev/null; then
        return
      fi
      sleep 0.25
    done
    kill -TERM -- "-${process_group}" 2>/dev/null || true
  }
  stop_workspace_slam_nodes() {
    # ros2 launch may place a lifecycle component in a process group different
    # from its own.  Match only this workspace binary and never /opt/robot.
    local slam_pattern
    slam_pattern="^${WORKSPACE}/install/m20_slam_navigation/lib/m20_slam_navigation/slam_node( |$)"
    local slam_pids=()
    mapfile -t slam_pids < <(pgrep -f "${slam_pattern}" 2>/dev/null || true)
    ((${#slam_pids[@]} == 0)) && return
    kill -INT "${slam_pids[@]}" 2>/dev/null || true
    for _ in {1..20}; do
      local remaining=false
      for pid in "${slam_pids[@]}"; do
        if kill -0 "${pid}" 2>/dev/null; then
          remaining=true
          break
        fi
      done
      [[ "${remaining}" == "false" ]] && return
      sleep 0.25
    done
    for pid in "${slam_pids[@]}"; do
      kill -TERM "${pid}" 2>/dev/null || true
    done
  }
  stop_group "${BAG_PID}"
  # Stop the component while ros2 launch is still alive.  On M20Pro, stopping
  # launch first can close the SSH command before an orphaned component is
  # reached by the remainder of this trap.
  stop_workspace_slam_nodes
  stop_group "${LAUNCH_PID}"
  stop_workspace_slam_nodes
  if [[ -n "${DRDDS_PID}" ]]; then
    sudo -n kill -INT -- "-${DRDDS_PID}" 2>/dev/null || true
    sleep 1
    sudo -n kill -TERM -- "-${DRDDS_PID}" 2>/dev/null || true
  fi
  wait "${BAG_PID}" 2>/dev/null || true
  wait "${LAUNCH_PID}" 2>/dev/null || true
  if [[ -f "${MAP_PATH}" ]]; then
    printf 'Mapping complete: %s\n' "${MAP_PATH}"
  else
    printf 'No map file was produced. Check sensor contract and mapper logs.\n' >&2
  fi
}
trap cleanup EXIT HUP INT TERM

if [[ -z "${BAG_PATH}" ]]; then
  DRDDS_RECEIVER="${WORKSPACE}/install/m20_slam_navigation/lib/m20_slam_navigation/m20_drdds_receiver"
  if [[ ! -x "${DRDDS_RECEIVER}" ]]; then
    printf 'Missing DrDDS receiver: %s\n' "${DRDDS_RECEIVER}" >&2
    exit 1
  fi
  printf 'DrDDS shared memory requires root access; validating sudo once.\n'
  sudo -v
  # Authenticate while still attached to this terminal, then create the
  # receiver's detached process group as root.  Running `setsid sudo -n`
  # first loses a tty-scoped sudo timestamp on M20Pro and immediately fails.
  sudo -n setsid env LD_LIBRARY_PATH="${LD_LIBRARY_PATH:-}" \
    "${DRDDS_RECEIVER}" \
    --lidar-topic /LIDAR/POINTS \
    --imu-topic /IMU \
    --domain 0 \
    --prefix rt \
    --network eth0/eth1 \
    --lidar-socket /tmp/m20_drdds_lidar.sock \
    --imu-socket /tmp/m20_drdds_imu.sock \
    --shm &
  DRDDS_PID=$!
  for _ in {1..20}; do
    [[ -S /tmp/m20_drdds_lidar.sock && -S /tmp/m20_drdds_imu.sock ]] && break
    if ! kill -0 "${DRDDS_PID}" 2>/dev/null; then
      printf 'DrDDS receiver exited before creating its socket.\n' >&2
      exit 1
    fi
    sleep 0.25
  done
  if [[ ! -S /tmp/m20_drdds_lidar.sock || ! -S /tmp/m20_drdds_imu.sock ]]; then
    printf 'Timed out waiting for vendor LiDAR/IMU sockets\n' >&2
    exit 1
  fi
fi

printf 'Starting M20 mapping with the OEM topic/TF contract. Ctrl+C saves and exits.\n'
LIDAR_TRANSPORT="drdds"
IMU_TRANSPORT="drdds"
MAX_LIDAR_QUEUE_SIZE="3"
CHECKPOINT_SAVE_PERIOD_S="10.0"
if [[ -n "${BAG_PATH}" ]]; then
  LIDAR_TRANSPORT="ros2"
  IMU_TRANSPORT="ros2"
  # Offline A/B must process the complete official bag even if loop
  # verification temporarily consumes more CPU than real time.
  MAX_LIDAR_QUEUE_SIZE="512"
  CHECKPOINT_SAVE_PERIOD_S="0.0"
fi
setsid ros2 launch m20_slam_navigation slam_system.launch.py \
  use_rviz:="${USE_RVIZ}" \
  use_sim_time:="false" \
  lidar_topic:=/LIDAR/POINTS \
  lidar_transport:="${LIDAR_TRANSPORT}" \
  imu_topic:=/IMU \
  imu_transport:="${IMU_TRANSPORT}" \
  drdds_socket_path:=/tmp/m20_drdds_lidar.sock \
  drdds_imu_socket_path:=/tmp/m20_drdds_imu.sock \
  max_lidar_queue_size:="${MAX_LIDAR_QUEUE_SIZE}" \
  checkpoint_save_period_s:="${CHECKPOINT_SAVE_PERIOD_S}" \
  map_save_path:="${MAP_PATH}" &
LAUNCH_PID=$!

if [[ -n "${BAG_PATH}" ]]; then
  # Do not start the finite test bag until both mapper subscriptions have
  # appeared in DDS.  Otherwise the first seconds are silently missed and the
  # offline A/B result is not comparable with the vendor run.
  wait_for_subscriber /LIDAR/POINTS
  wait_for_subscriber /IMU
  # Slower wall-time playback changes no ROS timestamps, but gives the
  # geometry verifier enough CPU to keep every scan during offline A/B.
  setsid ros2 bag play "${BAG_PATH}" --clock --rate 0.75 &
  BAG_PID=$!
  wait "${BAG_PID}"
  # Loop/submap verification can temporarily run slower than bag wall time.
  # Keep the mapper alive long enough to consume the complete 452-frame
  # official bag before the EXIT trap requests a map snapshot.
  sleep 20
else
  wait "${LAUNCH_PID}"
fi

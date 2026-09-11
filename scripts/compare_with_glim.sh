#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
WS_ROOT="$(cd "${REPO_ROOT}/../.." && pwd)"

usage() {
  cat <<'EOF'
Compare lidarslam_ros2 vs GLIM on the same rosbag2 and export trajectories.

This script:
  1) runs lidarslam on a bag (with optional TF fallback)
  2) logs TF as a TUM trajectory
  3) runs GLIM (glim_rosbag) on the same bag
  4) collects GLIM dump trajectories
  5) optionally runs evo_ape if installed

Usage:
  bash scripts/compare_with_glim.sh --bag /path/to/bag [options]
  bash scripts/compare_with_glim.sh --official --download [options]

Bag selection:
  --bag PATH                   rosbag2 directory (must contain metadata.yaml)
  --official                   Use the official open dataset used by this repo
  --variant livox|ouster       Official dataset variant (default: livox)
  --dest DIR                   Official dataset directory (default: ./demo_data/koide_lidar_camera_calib)
  --download                   Download+extract official dataset if needed
  --bag-dir DIR                Use a specific extracted rosbag2 directory for --official (must contain metadata.yaml)

Common options:
  --out-dir DIR                Output directory (default: ./output/compare_YYYYMMDD_HHMMSS)
  --param FILE                 lidarslam parameter YAML passed to run_bag_demo.sh
  --points-topic TOPIC         PointCloud2 topic (auto-detects if omitted)
  --imu-topic TOPIC            Imu topic (auto-detects if omitted)
  --no-imu                     Disable IMU for lidarslam and GLIM config patching
  --use-sim-time true|false    Use /clock time for lidarslam TF logging (default: true)
  --rviz                       Start RViz for lidarslam (default: off)
  --no-graph-based-slam        Run lidarslam frontend only (skip graph_based_slam backend)

Frames (lidarslam):
  --global-frame-id FRAME      (default: map)
  --odom-frame-id FRAME        (default: odom)
  --robot-frame-id FRAME       (default: base_link)
  --base-frame FRAME           Static TF base frame (default: inferred from points_frame_id)
  --lidar-frame FRAME          Static TF lidar frame (default: inferred from points_frame_id)
  --points-frame-id FRAME      Force PointCloud2 frame_id for trajectory logging

TF fallback (lidarslam):
  --auto-static-tf             If bag has no /tf(/tf_static), enable identity static TF (default: on)
  --auto-static-tf-timeout SEC Timeout for lidar frame_id detection (default: 5)

GLIM options:
  --skip-glim                  Run only lidarslam and export its TUM trajectory
  --glim-preset cpu|gpu        Which GLIM preset to copy (default: cpu)
  --glim-mode lidar-only|lidar-imu
                               Whether to disable IMU in GLIM configs (default: lidar-only)
  --glim-config-path DIR       Use a GLIM config directory as-is (skip preset copy+patch)
  --glim-timeout-sec SEC       Wall-clock timeout for GLIM run (0 disables, default: 180)
  --glim-cache-dir DIR         Content-bound cache directory for verified GLIM trajectories
                               (default: ./output/glim_reference_cache)
  --no-glim-cache              Disable verified cache lookup and storage
  --glim-viewer                Enable GLIM viewer modules (default: off)
  --no-glim-viewer             Keep GLIM viewer modules disabled (default)

Notes:
  - This script treats GLIM as a reference trajectory (not ground-truth).
  - GLIM outputs are dumped under /tmp/dump; this script copies traj_lidar.txt into --out-dir.
  - A cache fallback is used only when bag bytes, effective config, GLIM runtime,
    request options, harness, manifest, and TUM trajectory digest all match.
  - Legacy path/topic-only cache files are ignored and never reused.
EOF
}

die() {
  echo "error: $*" >&2
  exit 1
}

is_ntu_viral_tnp01_bag() {
  local bag_path="${1:-}"
  [[ -n "${bag_path}" ]] || return 1
  [[ "${bag_path}" == *"/tnp_01"* || "${bag_path}" == *"tnp_01_rosbag2"* || "${bag_path}" == *"tnp_01_points_restamped_vn100_rosbag2"* ]]
}

detect_frame_id() {
  local topic="$1"
  local timeout_sec="${2:-5}"
  local raw=""
  local frame=""
  local line=""

  if [[ -z "${topic}" ]]; then
    echo ""
    return 0
  fi

  if [[ ! -f "${BAG_PATH}/metadata.yaml" ]]; then
    echo ""
    return 0
  fi

  if command -v timeout >/dev/null 2>&1; then
    raw="$(timeout "${timeout_sec}" ros2 topic echo --once --qos-profile sensor_data --field header.frame_id "${topic}" 2>/dev/null || true)"
    if [[ -z "${raw}" ]]; then
      raw="$(timeout "${timeout_sec}" ros2 topic echo --once --qos-profile sensor_data "${topic}" 2>/dev/null | awk '/frame_id:/ {print $2; exit}' || true)"
    fi
  else
    echo "warn: timeout command not found; topic frame detection may hang" >&2
    raw="$(ros2 topic echo --once --qos-profile sensor_data --field header.frame_id "${topic}" 2>/dev/null || true)"
    if [[ -z "${raw}" ]]; then
      raw="$(ros2 topic echo --once --qos-profile sensor_data "${topic}" 2>/dev/null | awk '/frame_id:/ {print $2; exit}' || true)"
    fi
  fi

  while IFS= read -r line; do
    line="$(echo "${line}" | tr -d '\r' | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
    [[ -z "${line}" ]] && continue
    if [[ "${line}" == *"message was lost"* || "${line}" == *"does not appear to be published yet"* ]]; then
      continue
    fi
    if [[ "${line}" == *":"* ]]; then
      line="${line#*:}"
      line="$(echo "${line}" | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
    fi
    if [[ "${line}" == *" "* ]]; then
      continue
    fi
    frame="$(sanitize_frame_id "${line}")"
    break
  done <<< "${raw}"

  echo "${frame}"
}

now_monotonic() {
  python3 - <<'PY'
import time
print(time.monotonic())
PY
}

pick_ros_domain_id() {
  python3 - <<'PY'
import os
import time

base = ((os.getpid() * 131) + (time.monotonic_ns() // 1_000_000)) % 180
print(base + 20)
PY
}

offset_ros_domain_id() {
  python3 - "$@" <<'PY'
import sys

base = int(sys.argv[1]) if len(sys.argv) > 1 else 20
offset = int(sys.argv[2]) if len(sys.argv) > 2 else 0
span = 180
start = 20
print(((base - start + offset) % span) + start)
PY
}

find_recent_glim_traj() {
  local start_sec="${1:-0}"
  python3 - "${start_sec}" <<'PY'
import glob
import os
import sys

start = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
best_path = ""
best_mtime = -1.0
for path in glob.glob("/tmp/**/traj_lidar.txt", recursive=True):
    if not os.path.isfile(path):
        continue
    try:
        mtime = os.path.getmtime(path)
    except Exception:
        continue
    if mtime >= start - 5.0 and mtime > best_mtime:
        best_path = path
        best_mtime = mtime
print(best_path)
PY
}

detect_glim_failure_reason() {
  local log_path="${1:-}"
  python3 - "${log_path}" <<'PY'
import os
import sys

path = sys.argv[1] if len(sys.argv) > 1 else ""
if not path or not os.path.isfile(path):
    print("")
    raise SystemExit(0)

with open(path, "r", encoding="utf-8", errors="replace") as f:
    text = f.read().lower()

patterns = [
    ("dds_permission", [
        "getifaddrs: operation not permitted",
        "failed to create segment",
        "failed to create shared memory manager",
        "error creating socket: operation not permitted",
        "unable to register shm transport",
    ]),
    ("rcl_init_failed", [
        "failed to initialize rcl node",
        "cannot create participant due to initialization error",
        "problem creating rtpsparticipant",
    ]),
    ("gpu_module_missing", [
        "failed to load odometry estimation module",
        "failed to open libodometry_estimation_gpu.so",
    ]),
]

for name, needles in patterns:
    if any(needle in text for needle in needles):
        print(name)
        break
else:
    print("")
PY
}

prepare_glim_cache_identity() {
  if [[ "${GLIM_CACHE_ENABLED:-true}" != "true" ]]; then
    GLIM_CACHE_STATUS="DISABLED"
    return 1
  fi
  if [[ "${GLIM_CACHE_IDENTITY_READY:-false}" == "true" && -f "${GLIM_CACHE_IDENTITY_PATH:-}" ]]; then
    GLIM_CACHE_KEY="$(python3 - "${GLIM_CACHE_IDENTITY_PATH}" <<'PY'
import json
import sys

with open(sys.argv[1], "r", encoding="utf-8") as stream:
    print(json.load(stream)["cache_key_sha256"])
PY
)" || return 1
    GLIM_CACHE_TRAJ="${GLIM_CACHE_DIR}/${GLIM_CACHE_KEY}.traj_lidar.txt"
    GLIM_CACHE_MANIFEST="${GLIM_CACHE_DIR}/${GLIM_CACHE_KEY}.manifest.json"
    if [[ "${GLIM_CACHE_STATUS:-}" != "HIT_VERIFIED" && "${GLIM_CACHE_STATUS:-}" != "STORED" ]]; then
      GLIM_CACHE_STATUS="IDENTITY_READY"
    fi
    return 0
  fi
  if [[ -z "${GLIM_CONFIG_REAL:-}" || ! -d "${GLIM_CONFIG_REAL}" ]]; then
    GLIM_CACHE_STATUS="IDENTITY_UNAVAILABLE_CONFIG"
    return 1
  fi

  local runtime_kind=""
  local runtime_token=""
  local runtime_path=""
  local resolved_path=""
  local glim_prefix=""
  local glim_ros_prefix=""
  local -a runtime_args=()
  local -A seen_runtime_paths=()

  if [[ -n "${GLIM_DOCKER_IMAGE:-}" ]]; then
    if ! command -v docker >/dev/null 2>&1; then
      GLIM_CACHE_STATUS="IDENTITY_UNAVAILABLE_RUNTIME"
      return 1
    fi
    runtime_token="$(docker image inspect --format '{{.Id}}' "${GLIM_DOCKER_IMAGE}" 2>/dev/null || true)"
    if [[ -z "${runtime_token}" ]]; then
      GLIM_CACHE_STATUS="IDENTITY_UNAVAILABLE_RUNTIME"
      return 1
    fi
    runtime_kind="docker-image"
    runtime_args+=(--runtime-token "image=${GLIM_DOCKER_IMAGE};id=${runtime_token}")
  else
    glim_ros_prefix="$(ros2 pkg prefix glim_ros 2>/dev/null || true)"
    glim_prefix="$(ros2 pkg prefix glim 2>/dev/null || true)"
    if [[ -z "${glim_ros_prefix}" || -z "${glim_prefix}" ]]; then
      GLIM_CACHE_STATUS="IDENTITY_UNAVAILABLE_RUNTIME"
      return 1
    fi
    runtime_kind="local-install"
    runtime_args+=(--runtime-token "ros=${ROS_DISTRO:-unknown};arch=$(uname -m)")
    while IFS= read -r -d '' runtime_path; do
      resolved_path="$(realpath "${runtime_path}" 2>/dev/null || true)"
      if [[ -n "${resolved_path}" && -z "${seen_runtime_paths[${resolved_path}]:-}" ]]; then
        seen_runtime_paths["${resolved_path}"]=1
        runtime_args+=(--runtime-path "${resolved_path}")
      fi
    done < <(
      find -L "${glim_prefix}/lib" -maxdepth 1 -type f -name 'libglim*.so*' -print0 2>/dev/null
      for runtime_path in \
        "${glim_ros_prefix}/lib/glim_ros/glim_rosbag" \
        "${glim_ros_prefix}/share/glim_ros/package.xml" \
        "${glim_prefix}/share/glim/package.xml"; do
        [[ -e "${runtime_path}" ]] && printf '%s\0' "${runtime_path}"
      done
    )
    if [[ "${#seen_runtime_paths[@]}" -eq 0 ]]; then
      GLIM_CACHE_STATUS="IDENTITY_UNAVAILABLE_RUNTIME"
      return 1
    fi
  fi

  echo "building verified GLIM cache identity (hashing bag/config once)..."
  local -a identity_args=(
    identity
    --bag-dir "${BAG_PATH}"
    --config-dir "${GLIM_CONFIG_REAL}"
    --runtime-kind "${runtime_kind}"
    --harness "${REPO_ROOT}/scripts/compare_with_glim.sh"
    --points-topic "${POINTS_TOPIC}"
    --imu-topic "${IMU_TOPIC}"
    --mode "${GLIM_MODE}"
    --preset "${GLIM_PRESET}"
    --no-imu "${NO_IMU}"
    --viewer "${GLIM_VIEWER}"
    --output "${GLIM_CACHE_IDENTITY_PATH}"
  )
  if [[ -n "${GLIM_OMP_NUM_THREADS:-}" ]]; then
    identity_args+=(--omp-threads "${GLIM_OMP_NUM_THREADS}")
  fi
  identity_args+=("${runtime_args[@]}")
  if GLIM_CACHE_KEY="$(
    python3 "${REPO_ROOT}/scripts/glim_reference_cache.py" \
      "${identity_args[@]}" 2>"${GLIM_CACHE_LOG}"
  )"; then
    GLIM_CACHE_TRAJ="${GLIM_CACHE_DIR}/${GLIM_CACHE_KEY}.traj_lidar.txt"
    GLIM_CACHE_MANIFEST="${GLIM_CACHE_DIR}/${GLIM_CACHE_KEY}.manifest.json"
    GLIM_CACHE_STATUS="IDENTITY_READY"
    GLIM_CACHE_IDENTITY_READY="true"
    echo "verified GLIM cache key: ${GLIM_CACHE_KEY}"
    return 0
  fi
  GLIM_CACHE_STATUS="IDENTITY_ERROR"
  echo "warn: verified GLIM cache identity could not be built" >&2
  sed -n '1,3p' "${GLIM_CACHE_LOG}" >&2 || true
  return 1
}

use_verified_glim_traj() {
  local reason="${1:-fresh_glim_unavailable}"
  local cache_path=""
  prepare_glim_cache_identity || {
    echo "warn: no verified GLIM cache identity is available; stale/legacy cache is ignored" >&2
    return 1
  }
  if ! cache_path="$(
    python3 "${REPO_ROOT}/scripts/glim_reference_cache.py" lookup \
      --identity "${GLIM_CACHE_IDENTITY_PATH}" \
      --cache-dir "${GLIM_CACHE_DIR}" \
      2>"${GLIM_CACHE_LOG}"
  )"; then
    GLIM_CACHE_STATUS="MISS"
    echo "warn: no verified GLIM cache entry matches this exact run" >&2
    sed -n '1,3p' "${GLIM_CACHE_LOG}" >&2 || true
    return 1
  fi

  mkdir -p "${GLIM_OUT}"
  cp -f "${cache_path}" "${GLIM_OUT}/traj_lidar.txt"
  GLIM_TRAJ="${GLIM_OUT}/traj_lidar.txt"
  GLIM_TRAJ_LINES="$(wc -l < "${GLIM_TRAJ}" | tr -d ' ')"
  GLIM_REFERENCE_SOURCE="cache"
  GLIM_CACHE_STATUS="HIT_VERIFIED"
  if [[ -z "${GLIM_FAILURE_REASON}" ]]; then
    GLIM_FAILURE_REASON="${reason}"
  fi
  echo "warn: using content-verified GLIM trajectory (${reason}): ${cache_path}" >&2
  return 0
}

persist_verified_glim_cache() {
  local src="${1:-}"
  [[ "${GLIM_CACHE_ENABLED:-true}" == "true" ]] || return 0
  [[ -n "${src}" ]] || return 1
  [[ -f "${src}" ]] || return 1
  prepare_glim_cache_identity || return 1
  if python3 "${REPO_ROOT}/scripts/glim_reference_cache.py" store \
    --identity "${GLIM_CACHE_IDENTITY_PATH}" \
    --cache-dir "${GLIM_CACHE_DIR}" \
    --trajectory "${src}" \
    >"${GLIM_CACHE_LOG}" 2>&1; then
    GLIM_CACHE_STATUS="STORED"
    echo "verified GLIM cache stored: ${GLIM_CACHE_MANIFEST}"
    return 0
  fi
  GLIM_CACHE_STATUS="STORE_REJECTED"
  echo "warn: GLIM trajectory was used fresh but not cached" >&2
  sed -n '1,3p' "${GLIM_CACHE_LOG}" >&2 || true
  return 1
}

write_metrics_json() {
  [[ -n "${METRICS_JSON:-}" ]] || return 0

  METRICS_JSON="${METRICS_JSON}" \
  STARTED_AT="${STARTED_AT:-}" \
  STARTED_AT_UNIX="${STARTED_AT_UNIX:-}" \
  OUT_DIR="${OUT_DIR:-}" \
  BAG_PATH="${BAG_PATH:-}" \
  OFFICIAL="${OFFICIAL:-}" \
  VARIANT="${VARIANT:-}" \
  DEST_DIR="${DEST_DIR:-}" \
  OFFICIAL_BAG_DIR="${OFFICIAL_BAG_DIR:-}" \
  BAG_DURATION_NS="${BAG_DURATION_NS:-}" \
  BAG_DURATION_SEC="${BAG_DURATION_SEC:-}" \
  POINTS_TOPIC="${POINTS_TOPIC:-}" \
  IMU_TOPIC="${IMU_TOPIC:-}" \
  NO_IMU="${NO_IMU:-}" \
  USE_SIM_TIME="${USE_SIM_TIME:-}" \
  USE_RVIZ="${USE_RVIZ:-}" \
  GLOBAL_FRAME_ID="${GLOBAL_FRAME_ID:-}" \
  ODOM_FRAME_ID="${ODOM_FRAME_ID:-}" \
  ROBOT_FRAME_ID="${ROBOT_FRAME_ID:-}" \
  AUTO_STATIC_TF="${AUTO_STATIC_TF:-}" \
  AUTO_STATIC_TF_TIMEOUT="${AUTO_STATIC_TF_TIMEOUT:-}" \
  POINTS_FRAME_ID="${POINTS_FRAME_ID:-}" \
  LIDARSLAM_DIR="${LIDARSLAM_DIR:-}" \
  LIDARSLAM_TUM="${LIDARSLAM_TUM:-}" \
  LIDARSLAM_LOG="${LIDARSLAM_LOG:-}" \
  LIDARSLAM_PARAM_FILE="${LIDARSLAM_PARAM_FILE:-}" \
  LIDARSLAM_RC="${LIDARSLAM_RC:-}" \
  LIDARSLAM_WALL_SEC="${LIDARSLAM_WALL_SEC:-}" \
  LIDARSLAM_RTF="${LIDARSLAM_RTF:-}" \
  LIDARSLAM_TUM_LINES="${LIDARSLAM_TUM_LINES:-}" \
  LIDARSLAM_SUCCESS="${LIDARSLAM_SUCCESS:-}" \
  GLIM_AVAILABLE="${GLIM_AVAILABLE:-}" \
  GLIM_OUT="${GLIM_OUT:-}" \
  GLIM_LOG="${GLIM_LOG:-}" \
  GLIM_RC="${GLIM_RC:-}" \
  GLIM_WALL_SEC="${GLIM_WALL_SEC:-}" \
  GLIM_RTF="${GLIM_RTF:-}" \
  GLIM_DUMP_DIR="${GLIM_DUMP_DIR:-}" \
  GLIM_TRAJ="${GLIM_TRAJ:-}" \
  GLIM_TRAJ_LINES="${GLIM_TRAJ_LINES:-}" \
  GLIM_SUCCESS="${GLIM_SUCCESS:-}" \
  GLIM_CONFIG_REAL="${GLIM_CONFIG_REAL:-}" \
  GLIM_REFERENCE_SOURCE="${GLIM_REFERENCE_SOURCE:-}" \
  GLIM_FAILURE_REASON="${GLIM_FAILURE_REASON:-}" \
  GLIM_CACHE_ENABLED="${GLIM_CACHE_ENABLED:-}" \
  GLIM_CACHE_STATUS="${GLIM_CACHE_STATUS:-}" \
  GLIM_CACHE_KEY="${GLIM_CACHE_KEY:-}" \
  GLIM_CACHE_IDENTITY_PATH="${GLIM_CACHE_IDENTITY_PATH:-}" \
  GLIM_CACHE_MANIFEST="${GLIM_CACHE_MANIFEST:-}" \
  GLIM_CACHE_TRAJ="${GLIM_CACHE_TRAJ:-}" \
  EVO_APE_LOG="${EVO_APE_LOG:-}" \
  python3 - <<'PY'
import json
import os
from typing import Any, Optional

def get(name: str, default: Optional[str] = None) -> Optional[str]:
    v = os.environ.get(name, "")
    return v if v != "" else default

def get_bool(name: str) -> Optional[bool]:
    v = get(name)
    if v is None:
        return None
    v = v.strip().lower()
    if v in ("1", "true", "t", "yes", "y", "on"):
        return True
    if v in ("0", "false", "f", "no", "n", "off"):
        return False
    return None

def get_int(name: str) -> Optional[int]:
    v = get(name)
    if v is None:
        return None
    try:
        return int(v)
    except Exception:
        try:
            return int(float(v))
        except Exception:
            return None

def get_float(name: str) -> Optional[float]:
    v = get(name)
    if v is None:
        return None
    try:
        return float(v)
    except Exception:
        return None

data: dict[str, Any] = {
    "started_at": get("STARTED_AT"),
    "started_at_unix": get_int("STARTED_AT_UNIX"),
    "out_dir": get("OUT_DIR"),
    "bag_path": get("BAG_PATH"),
    "official": get_bool("OFFICIAL"),
    "variant": get("VARIANT"),
    "dest_dir": get("DEST_DIR"),
    "official_bag_dir": get("OFFICIAL_BAG_DIR"),
    "bag_duration_ns": get_int("BAG_DURATION_NS"),
    "bag_duration_sec": get_float("BAG_DURATION_SEC"),
    "points_topic": get("POINTS_TOPIC"),
    "imu_topic": get("IMU_TOPIC"),
    "no_imu": get_bool("NO_IMU"),
    "use_sim_time": get_bool("USE_SIM_TIME"),
    "use_rviz": get_bool("USE_RVIZ"),
    "frames": {
        "global_frame_id": get("GLOBAL_FRAME_ID"),
        "odom_frame_id": get("ODOM_FRAME_ID"),
        "robot_frame_id": get("ROBOT_FRAME_ID"),
        "points_frame_id": get("POINTS_FRAME_ID"),
    },
    "auto_static_tf": {
        "enabled": get_bool("AUTO_STATIC_TF"),
        "timeout_sec": get_float("AUTO_STATIC_TF_TIMEOUT"),
    },
    "lidarslam": {
        "rc": get_int("LIDARSLAM_RC"),
        "wall_sec": get_float("LIDARSLAM_WALL_SEC"),
        "rtf": get_float("LIDARSLAM_RTF"),
        "success": get_bool("LIDARSLAM_SUCCESS"),
        "tum_path": get("LIDARSLAM_TUM"),
        "tum_lines": get_int("LIDARSLAM_TUM_LINES"),
        "log_path": get("LIDARSLAM_LOG"),
        "param_path": get("LIDARSLAM_PARAM_FILE"),
        "out_dir": get("LIDARSLAM_DIR"),
    },
    "glim": {
        "available": get_bool("GLIM_AVAILABLE"),
        "rc": get_int("GLIM_RC"),
        "wall_sec": get_float("GLIM_WALL_SEC"),
        "rtf": get_float("GLIM_RTF"),
        "success": get_bool("GLIM_SUCCESS"),
        "dump_dir": get("GLIM_DUMP_DIR"),
        "traj_path": get("GLIM_TRAJ"),
        "traj_lines": get_int("GLIM_TRAJ_LINES"),
        "reference_source": get("GLIM_REFERENCE_SOURCE"),
        "failure_reason": get("GLIM_FAILURE_REASON"),
        "cache_traj_path": get("GLIM_CACHE_TRAJ"),
        "cache": {
            "enabled": get_bool("GLIM_CACHE_ENABLED"),
            "status": get("GLIM_CACHE_STATUS"),
            "key_sha256": get("GLIM_CACHE_KEY"),
            "identity_path": get("GLIM_CACHE_IDENTITY_PATH"),
            "manifest_path": get("GLIM_CACHE_MANIFEST"),
        },
        "log_path": get("GLIM_LOG"),
        "out_dir": get("GLIM_OUT"),
        "config_path": get("GLIM_CONFIG_REAL"),
    },
    "evo": {
        "ape_log_path": get("EVO_APE_LOG"),
    },
}

ape_log = data.get("evo", {}).get("ape_log_path")
if ape_log and os.path.isfile(ape_log):
    try:
        import re

        stats = {}
        units = None
        with open(ape_log, "r", encoding="utf-8", errors="replace") as f:
            for line in f:
                if "translation" in line and "(m)" in line:
                    units = "m"
                m = re.match(
                    r"^\s*(rmse|mean|median|min|max|std)\s*[:\t ]\s*([-+0-9.eE]+)\s*$",
                    line,
                )
                if m:
                    stats[m.group(1)] = float(m.group(2))
        if stats:
            data["evo"]["ape"] = {"units": units, **stats} if units else stats
    except Exception:
        pass

path = get("METRICS_JSON")
if not path:
    raise SystemExit(0)

os.makedirs(os.path.dirname(os.path.abspath(path)), exist_ok=True)
tmp = path + ".tmp"
with open(tmp, "w", encoding="utf-8") as f:
    json.dump(data, f, indent=2, sort_keys=True)
    f.write("\n")
os.replace(tmp, path)
PY
}

patch_glim_config() {
  local cfg_dir="$1"
  local mode="$2"
  local points_topic="$3"
  local imu_topic="$4"
  local viewer_enabled="0"

  if [[ "${GLIM_VIEWER}" == "true" ]]; then
    viewer_enabled="1"
  fi

  [[ -d "${cfg_dir}" ]] || return 1

  GLIM_CONFIG_DIR="${cfg_dir}" \
  GLIM_MODE="${mode}" \
  GLIM_PRESET_NAME="${GLIM_PRESET}" \
  GLIM_BAG_PATH="${BAG_PATH}" \
  POINTS_TOPIC="${points_topic}" \
  IMU_TOPIC="${imu_topic}" \
  GLIM_VIEWER_ENABLED="${viewer_enabled}" \
  python3 - <<'PY'
import json
import os
import re
from pathlib import Path

cfg_dir = Path(os.environ["GLIM_CONFIG_DIR"])
mode = os.environ.get("GLIM_MODE", "lidar-only").strip().lower()
preset = os.environ.get("GLIM_PRESET_NAME", "cpu").strip().lower()
bag_path = os.environ.get("GLIM_BAG_PATH", "")
points_topic = os.environ.get("POINTS_TOPIC", "/points_raw").strip() or "/points_raw"
imu_topic = os.environ.get("IMU_TOPIC", "/imu").strip() or "/imu"
glim_viewer = os.environ.get("GLIM_VIEWER_ENABLED", "0").strip() == "1"
is_tnp01 = "tnp_01" in bag_path


def _strip_json_comments(text: str) -> str:
    # Remove block comments.
    no_block = []
    in_block = False
    i = 0
    while i < len(text):
        if not in_block and text[i : i + 2] == "/*":
            in_block = True
            i += 2
            continue
        if in_block and text[i : i + 2] == "*/":
            in_block = False
            i += 2
            continue
        if not in_block:
            no_block.append(text[i])
        i += 1

    lines = []
    for line in "".join(no_block).splitlines():
        if "//" in line:
            line = line.split("//", 1)[0]
        line = line.rstrip()
        if line.strip():
            lines.append(line)
    text = "\n".join(lines)
    # Remove trailing commas before closing brackets.
    text = re.sub(r",(?=\s*[}\]])", "", text)
    return text


def load_json(path: Path):
    with path.open("r", encoding="utf-8", errors="replace") as f:
        raw = f.read()
    return json.loads(_strip_json_comments(raw))


def save_json(path: Path, obj):
    with path.open("w", encoding="utf-8") as f:
        json.dump(obj, f, indent=2, sort_keys=True)
        f.write("\n")


cfg_json = cfg_dir / "config.json"
cfg = load_json(cfg_json)
global_cfg = cfg.get("global", {})
if isinstance(global_cfg, dict):
    if mode == "lidar-only":
        global_cfg["config_odometry"] = "config_odometry_ct.json"
        global_cfg["config_sub_mapping"] = "config_sub_mapping_cpu.json"
        global_cfg["config_global_mapping"] = "config_global_mapping_cpu.json"
    elif preset == "cpu":
        global_cfg["config_odometry"] = "config_odometry_cpu.json"
        global_cfg["config_sub_mapping"] = "config_sub_mapping_cpu.json"
        global_cfg["config_global_mapping"] = "config_global_mapping_cpu.json"
    if not glim_viewer:
        global_cfg.pop("config_viewer", None)
    cfg["global"] = global_cfg
save_json(cfg_json, cfg)

ros_cfg_json = cfg_dir / "config_ros.json"
if ros_cfg_json.is_file():
    ros_cfg = load_json(ros_cfg_json)
    ros = ros_cfg.get("glim_ros", {})
    if isinstance(ros, dict):
        ros["points_topic"] = points_topic
        ros["imu_topic"] = imu_topic
        ros["points_topics"] = [points_topic]
        ros["imu_topics"] = [imu_topic]
        if is_tnp01:
            ros["imu_time_offset"] = 0.12
            ros["acc_scale"] = 0.0
        if not glim_viewer:
            ext = ros.get("extension_modules")
            if isinstance(ext, list):
                ros["extension_modules"] = [
                    x
                    for x in ext
                    if isinstance(x, str)
                    and "standard_viewer" not in x
                    and "rviz_viewer" not in x
                ]
        ros_cfg["glim_ros"] = ros
        save_json(ros_cfg_json, ros_cfg)

sensors_cfg_json = cfg_dir / "config_sensors.json"
if sensors_cfg_json.is_file() and is_tnp01:
    sensors_cfg = load_json(sensors_cfg_json)
    sensors = sensors_cfg.get("sensors", {})
    if isinstance(sensors, dict):
        sensors["T_lidar_imu"] = [-0.07, 0.0, 0.035, 0.0, 0.0, 0.0, 1.0]
        sensors["ring_field"] = "ring"
        sensors["autoconf_perpoint_times"] = True
        sensors["autoconf_prefer_frame_time"] = False
        sensors["perpoint_relative_time"] = True
        sensors["perpoint_time_scale"] = 1e-9
        sensors_cfg["sensors"] = sensors
        save_json(sensors_cfg_json, sensors_cfg)

for fname, section in (
    ("config_preprocess.json", "preprocess"),
    ("config_odometry_gpu.json", "odometry_estimation"),
    ("config_odometry_cpu.json", "odometry_estimation"),
):
    p = cfg_dir / fname
    if not (is_tnp01 and p.is_file()):
        continue
    data = load_json(p)
    cfg_section = data.get(section)
    if isinstance(cfg_section, dict):
        cfg_section["num_threads"] = 1
        data[section] = cfg_section
        save_json(p, data)

if not glim_viewer:
    def strip_viewers(obj):
        changed = False
        if isinstance(obj, dict):
            for key, value in obj.items():
                if key == "extension_modules" and isinstance(value, list):
                    filtered = [
                        x
                        for x in value
                        if not (
                            isinstance(x, str)
                            and ("standard_viewer" in x or "rviz_viewer" in x)
                        )
                    ]
                    if filtered != value:
                        obj[key] = filtered
                        changed = True
                elif strip_viewers(value):
                    changed = True
        elif isinstance(obj, list):
            for value in obj:
                if strip_viewers(value):
                    changed = True
        return changed

    for p in cfg_dir.glob("*.json"):
        try:
            data = load_json(p)
        except Exception:
            continue
        changed = strip_viewers(data)
        if changed:
            save_json(p, data)

if mode == "lidar-only":
    for fname in (
        "config_sub_mapping_cpu.json",
        "config_global_mapping_cpu.json",
        "config_odometry_ct.json",
    ):
        p = cfg_dir / fname
        if not p.is_file():
            continue
        try:
            data = load_json(p)
        except Exception:
            continue
        changed = False
        for key in ("sub_mapping", "global_mapping", "odometry_estimation"):
            section = data.get(key)
            if isinstance(section, dict) and isinstance(section.get("enable_imu"), bool):
                section["enable_imu"] = False
                changed = True
        if changed:
            save_json(p, data)
PY
}

OFFICIAL="false"
VARIANT="livox"
DEST_DIR="${REPO_ROOT}/demo_data/koide_lidar_camera_calib"
DO_DOWNLOAD="false"
OFFICIAL_BAG_DIR=""

BAG_PATH=""
OUT_DIR=""
PARAM_FILE=""
LIDARSLAM_FORCE_NO_IMU="false"

POINTS_TOPIC=""
IMU_TOPIC=""
NO_IMU="false"
USE_SIM_TIME="true"
USE_RVIZ="false"

GLOBAL_FRAME_ID="map"
ODOM_FRAME_ID="odom"
ROBOT_FRAME_ID="base_link"
ROBOT_FRAME_ID_USER_SPECIFIED="false"
BASE_FRAME=""
LIDAR_FRAME=""
POINTS_FRAME_ID=""
POINTS_FRAME_ID_USER_SPECIFIED="false"
BASE_FRAME_USER_SPECIFIED="false"
LIDAR_FRAME_USER_SPECIFIED="false"

AUTO_STATIC_TF="true"
AUTO_STATIC_TF_TIMEOUT="5"

SKIP_GLIM="false"
GLIM_PRESET="cpu"
GLIM_MODE="lidar-only"
GLIM_CONFIG_PATH=""
GLIM_TIMEOUT_SEC="180"
GLIM_CACHE_DIR="${REPO_ROOT}/output/glim_reference_cache"
GLIM_CACHE_ENABLED="true"
GLIM_VIEWER="false"
USE_GRAPH_BASED_SLAM="true"
GLIM_OMP_NUM_THREADS=""
GLIM_DOCKER_IMAGE=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    -h|--help)
      usage
      exit 0
      ;;
    --official)
      OFFICIAL="true"; shift ;;
    --variant)
      VARIANT="${2:-}"; shift 2 ;;
    --dest)
      DEST_DIR="${2:-}"; shift 2 ;;
    --download)
      DO_DOWNLOAD="true"; shift ;;
    --bag-dir)
      OFFICIAL_BAG_DIR="${2:-}"; shift 2 ;;
    --bag)
      BAG_PATH="${2:-}"; shift 2 ;;
    --out-dir)
      OUT_DIR="${2:-}"; shift 2 ;;
    --param)
      PARAM_FILE="${2:-}"; shift 2 ;;
    --points-topic)
      POINTS_TOPIC="${2:-}"; shift 2 ;;
    --imu-topic)
      IMU_TOPIC="${2:-}"; shift 2 ;;
    --no-imu)
      NO_IMU="true"; shift ;;
    --use-sim-time)
      USE_SIM_TIME="${2:-}"; shift 2 ;;
    --rviz)
      USE_RVIZ="true"; shift ;;
    --no-graph-based-slam)
      USE_GRAPH_BASED_SLAM="false"; shift ;;
    --global-frame-id)
      GLOBAL_FRAME_ID="${2:-}"; shift 2 ;;
    --odom-frame-id)
      ODOM_FRAME_ID="${2:-}"; shift 2 ;;
    --robot-frame-id)
      ROBOT_FRAME_ID="${2:-}"; ROBOT_FRAME_ID_USER_SPECIFIED="true"; shift 2 ;;
    --base-frame)
      BASE_FRAME="${2:-}"; BASE_FRAME_USER_SPECIFIED="true"; shift 2 ;;
    --lidar-frame)
      LIDAR_FRAME="${2:-}"; LIDAR_FRAME_USER_SPECIFIED="true"; shift 2 ;;
    --points-frame-id)
      POINTS_FRAME_ID="${2:-}"; POINTS_FRAME_ID_USER_SPECIFIED="true"; shift 2 ;;
    --auto-static-tf)
      AUTO_STATIC_TF="true"; shift ;;
    --no-auto-static-tf)
      AUTO_STATIC_TF="false"; shift ;;
    --auto-static-tf-timeout)
      AUTO_STATIC_TF_TIMEOUT="${2:-}"; shift 2 ;;
    --skip-glim)
      SKIP_GLIM="true"; shift ;;
    --glim-preset)
      GLIM_PRESET="${2:-}"; shift 2 ;;
    --glim-mode)
      GLIM_MODE="${2:-}"; shift 2 ;;
    --glim-config-path)
      GLIM_CONFIG_PATH="${2:-}"; shift 2 ;;
    --glim-timeout-sec)
      GLIM_TIMEOUT_SEC="${2:-}"; shift 2 ;;
    --glim-cache-dir)
      GLIM_CACHE_DIR="${2:-}"; shift 2 ;;
    --no-glim-cache)
      GLIM_CACHE_ENABLED="false"; shift ;;
    --glim-viewer)
      GLIM_VIEWER="true"; shift ;;
    --no-glim-viewer)
      GLIM_VIEWER="false"; shift ;;
    *)
      die "unknown arg: $1 (use --help)"
    ;;
  esac
done

case "${GLIM_PRESET}" in
  cpu|gpu) ;;
  *) die "--glim-preset must be cpu or gpu" ;;
esac
case "${GLIM_MODE}" in
  lidar-only|lidar-imu) ;;
  *) die "--glim-mode must be lidar-only or lidar-imu" ;;
esac
if ! [[ "${GLIM_TIMEOUT_SEC}" =~ ^[0-9]+$ ]]; then
  die "--glim-timeout-sec must be a non-negative integer"
fi
if [[ -n "${GLIM_OMP_NUM_THREADS}" ]] && ! [[ "${GLIM_OMP_NUM_THREADS}" =~ ^[1-9][0-9]*$ ]]; then
  die "GLIM_OMP_NUM_THREADS must be a positive integer when set"
fi

sanitize_frame_id() {
  local value="${1:-}"
  value="$(echo "${value}" | tr -d '\r' | sed 's/^[[:space:]]*//; s/[[:space:]]*$//')"
  value="${value#\"}"
  value="${value%\"}"
  value="${value#\'}"
  value="${value%\'}"
  if [[ "${value}" == *" "* ]]; then
    echo ""
    return
  fi
  if [[ ! "${value}" =~ ^[A-Za-z_/.][A-Za-z0-9_./:-]*$ ]]; then
    echo ""
    return
  fi
  echo "${value}"
}

if [[ "${OFFICIAL}" == "true" ]]; then
  official_args=(--variant "${VARIANT}" --dest "${DEST_DIR}" --print-bag-dir)
  if [[ "${DO_DOWNLOAD}" == "true" ]]; then
    official_args+=(--download)
  fi
  if [[ -n "${OFFICIAL_BAG_DIR}" ]]; then
    official_args+=(--bag-dir "${OFFICIAL_BAG_DIR}")
  fi
  BAG_PATH="$(bash "${REPO_ROOT}/scripts/run_official_demo.sh" "${official_args[@]}")"
fi

[[ -n "${BAG_PATH}" ]] || { usage; die "--bag is required (or use --official)"; }
[[ -f "${BAG_PATH}/metadata.yaml" ]] || die "bag must contain metadata.yaml: ${BAG_PATH}"

if [[ -z "${OUT_DIR}" ]]; then
  stamp="$(date +%Y%m%d_%H%M%S)"
  OUT_DIR="${REPO_ROOT}/output/compare_${stamp}"
fi
mkdir -p "${OUT_DIR}"
ROS_LOG_DIR="${OUT_DIR}/.ros_log"
mkdir -p "${ROS_LOG_DIR}"
export ROS_LOG_DIR

STARTED_AT_UNIX="$(date +%s)"
STARTED_AT="$(date -Iseconds)"
METRICS_JSON="${OUT_DIR}/metrics.json"

BAG_DURATION_NS="$(
  awk '
    /^[[:space:]]*duration:/ {f=1; next}
    f && /^[[:space:]]*nanoseconds:/ {print $2; exit}
  ' "${BAG_PATH}/metadata.yaml" 2>/dev/null || true
)"
BAG_DURATION_SEC=""

LIDARSLAM_RC=""
LIDARSLAM_WALL_SEC=""
LIDARSLAM_RTF=""
LIDARSLAM_TUM_LINES="0"
LIDARSLAM_SUCCESS="false"
LIDARSLAM_LOG=""

GLIM_AVAILABLE="false"
GLIM_RC=""
GLIM_WALL_SEC=""
GLIM_RTF=""
GLIM_SUCCESS="false"
GLIM_LOG=""
GLIM_DUMP_DIR=""
GLIM_TRAJ=""
GLIM_TRAJ_LINES="0"
GLIM_CONFIG_REAL=""
GLIM_OUT=""
GLIM_REFERENCE_SOURCE="none"
GLIM_FAILURE_REASON=""
GLIM_CACHE_TRAJ=""
GLIM_CACHE_MANIFEST=""
GLIM_CACHE_KEY=""
GLIM_CACHE_STATUS="ENABLED_UNRESOLVED"
if [[ "${GLIM_CACHE_ENABLED}" != "true" ]]; then
  GLIM_CACHE_STATUS="DISABLED"
fi
GLIM_CACHE_IDENTITY_PATH="${OUT_DIR}/glim_cache_identity.json"
GLIM_CACHE_LOG="${OUT_DIR}/glim_cache.log"
GLIM_CACHE_IDENTITY_READY="false"
RUN_FRESH_GLIM="true"

EVO_APE_LOG="${OUT_DIR}/evo_ape.txt"

# Best-effort environment setup (won't override an already-sourced environment).
# Avoid `set -u` expansion failures from ROS setup scripts that reference
# unset AMENT_TRACE_SETUP_FILES.
set +u
if [[ -f "${WS_ROOT}/install/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "${WS_ROOT}/install/setup.bash"
elif [[ -n "${ROS_DISTRO:-}" && -f "/opt/ros/${ROS_DISTRO}/setup.bash" ]]; then
  # shellcheck source=/dev/null
  source "/opt/ros/${ROS_DISTRO}/setup.bash"
fi
set -u

command -v ros2 >/dev/null 2>&1 || die "ros2 not found in PATH (source your ROS 2 environment first)"
command -v python3 >/dev/null 2>&1 || die "python3 not found in PATH"

if [[ -n "${BAG_DURATION_NS}" ]]; then
  BAG_DURATION_SEC="$(BAG_DURATION_NS="${BAG_DURATION_NS}" python3 - <<'PY'
import os
ns = int(os.environ["BAG_DURATION_NS"])
print(ns / 1e9)
PY
)"
else
  BAG_DURATION_SEC="0"
fi

write_metrics_json

bag_info="$(ros2 bag info "${BAG_PATH}" 2>/dev/null || true)"

if [[ -z "${POINTS_TOPIC}" ]]; then
  POINTS_TOPIC="$(
    echo "${bag_info}" | awk -v type="sensor_msgs/msg/PointCloud2" -F'[|]' '
      $0 ~ ("Type: " type) && $0 ~ /Topic:/ && $0 ~ /Count:/ {
        topic=""; count=0;
        for (i=1; i<=NF; i++) {
          seg=$i;
          gsub(/^[ \t]+|[ \t]+$/, "", seg);
          if (seg ~ /^Topic:/) { sub(/^Topic:[ \t]*/, "", seg); topic=seg; }
          if (seg ~ /^Count:/) { sub(/^Count:[ \t]*/, "", seg); count=seg+0; }
        }
        if (topic != "" && count > best_count) { best_count=count; best_topic=topic; }
      }
      END { if (best_topic != "") print best_topic; }
    '
  )"
fi

HAS_IMU="false"
if [[ "${NO_IMU}" != "true" ]]; then
  if [[ -n "${IMU_TOPIC}" ]]; then
    HAS_IMU="true"
  else
    IMU_TOPIC="$(
      echo "${bag_info}" | awk -v type="sensor_msgs/msg/Imu" -F'[|]' '
        $0 ~ ("Type: " type) && $0 ~ /Topic:/ && $0 ~ /Count:/ {
          topic=""; count=0;
          for (i=1; i<=NF; i++) {
            seg=$i;
            gsub(/^[ \t]+|[ \t]+$/, "", seg);
            if (seg ~ /^Topic:/) { sub(/^Topic:[ \t]*/, "", seg); topic=seg; }
            if (seg ~ /^Count:/) { sub(/^Count:[ \t]*/, "", seg); count=seg+0; }
          }
          if (topic != "" && count > best_count) { best_count=count; best_topic=topic; }
        }
        END { if (best_topic != "") print best_topic; }
      '
    )"
    if [[ -n "${IMU_TOPIC}" ]]; then
      HAS_IMU="true"
    fi
  fi
fi

if [[ -z "${POINTS_TOPIC}" ]]; then
  POINTS_TOPIC="/points_raw"
  echo "warn: failed to auto-detect PointCloud2 topic; using default: ${POINTS_TOPIC}" >&2
fi
if [[ -z "${IMU_TOPIC}" ]]; then
  IMU_TOPIC="/imu"
fi

if [[ -z "${PARAM_FILE}" && "${NO_IMU}" != "true" ]]; then
  if is_ntu_viral_tnp01_bag "${BAG_PATH}"; then
    tuned_param="${REPO_ROOT}/lidarslam/param/lidarslam_ouster_aggressive_noimu.yaml"
    if [[ -f "${tuned_param}" ]]; then
      PARAM_FILE="${tuned_param}"
      LIDARSLAM_FORCE_NO_IMU="true"
      USE_GRAPH_BASED_SLAM="false"
      if [[ "${GLIM_MODE}" == "lidar-only" ]]; then
        GLIM_MODE="lidar-imu"
      fi
      if [[ "${GLIM_PRESET}" == "cpu" ]]; then
        GLIM_PRESET="gpu"
      fi
      GLIM_OMP_NUM_THREADS="1"
      GLIM_DOCKER_IMAGE="koide3/glim_ros2:jazzy_cuda12.5"
      echo "note: using NTU VIRAL tnp_01 lidarslam no-IMU params: ${PARAM_FILE}"
      echo "note: forcing lidarslam no-IMU + no-graph, and GLIM lidar-imu threads=1 via ${GLIM_DOCKER_IMAGE}"
    fi
  elif [[ "${BAG_PATH}" == *"/glim_mid360/"* || "${BAG_PATH}" == *"rosbag2_2024_04_16-14_17_01"* ]]; then
    tuned_param="${REPO_ROOT}/lidarslam/param/lidarslam_mid360_noimu.yaml"
    if [[ -f "${tuned_param}" ]]; then
      PARAM_FILE="${tuned_param}"
      USE_GRAPH_BASED_SLAM="false"
      echo "note: using MID360 no-IMU params: ${PARAM_FILE}"
      echo "note: disabling graph_based_slam for MID360 sample"
    fi
  elif [[ "${POINTS_TOPIC}" == *"livox"* ]]; then
    tuned_param="${REPO_ROOT}/lidarslam/param/lidarslam_solid_state_imu_tight.yaml"
    if [[ -f "${tuned_param}" ]]; then
      PARAM_FILE="${tuned_param}"
      echo "note: using tuned Livox IMU params: ${PARAM_FILE}"
    fi
  fi
fi

USE_PHASE_DOMAINS="false"
if [[ -z "${ROS_DOMAIN_ID:-}" ]]; then
  ROS_DOMAIN_BASE="$(pick_ros_domain_id)"
  PROBE_DOMAIN_ID="$(offset_ros_domain_id "${ROS_DOMAIN_BASE}" 0)"
  LIDARSLAM_DOMAIN_ID="$(offset_ros_domain_id "${ROS_DOMAIN_BASE}" 1)"
  GLIM_DOMAIN_ID="$(offset_ros_domain_id "${ROS_DOMAIN_BASE}" 2)"
  USE_PHASE_DOMAINS="true"
else
  PROBE_DOMAIN_ID="${ROS_DOMAIN_ID}"
  LIDARSLAM_DOMAIN_ID="${ROS_DOMAIN_ID}"
  GLIM_DOMAIN_ID="${ROS_DOMAIN_ID}"
fi

LIDARSLAM_CMD_PREFIX=()
GLIM_CMD_PREFIX=()
if [[ "${USE_PHASE_DOMAINS}" == "true" ]]; then
  LIDARSLAM_CMD_PREFIX=(env "ROS_DOMAIN_ID=${LIDARSLAM_DOMAIN_ID}")
  GLIM_CMD_PREFIX=(env "ROS_DOMAIN_ID=${GLIM_DOMAIN_ID}")
fi
if [[ -n "${GLIM_OMP_NUM_THREADS}" ]]; then
  GLIM_CMD_PREFIX+=("OMP_NUM_THREADS=${GLIM_OMP_NUM_THREADS}")
fi

echo "bag:          ${BAG_PATH}"
echo "points topic: ${POINTS_TOPIC}"
echo "imu topic:    ${IMU_TOPIC} (detected=${HAS_IMU}, no_imu=${NO_IMU})"
echo "out dir:      ${OUT_DIR}"
echo "bag duration: ${BAG_DURATION_SEC}s"
if [[ "${USE_PHASE_DOMAINS}" == "true" ]]; then
  echo "ros domain:   probe=${PROBE_DOMAIN_ID} lidarslam=${LIDARSLAM_DOMAIN_ID} glim=${GLIM_DOMAIN_ID}"
else
  echo "ros domain:   inherited=${ROS_DOMAIN_ID}"
fi

if command -v timeout >/dev/null 2>&1; then
  :
else
  echo "warn: timeout command not found; lidar frame_id detection may hang" >&2
fi

if [[ "${USE_PHASE_DOMAINS}" == "true" ]]; then
  export ROS_DOMAIN_ID="${PROBE_DOMAIN_ID}"
fi
play_pid=""
ros2 bag play "${BAG_PATH}" --topics "${POINTS_TOPIC}" --rate 10.0 >/dev/null 2>&1 &
play_pid="$!"

if [[ "${POINTS_FRAME_ID_USER_SPECIFIED}" == "true" ]]; then
  POINTS_FRAME_ID="$(sanitize_frame_id "${POINTS_FRAME_ID}")"
else
  POINTS_FRAME_ID="$(detect_frame_id "${POINTS_TOPIC}" "${AUTO_STATIC_TF_TIMEOUT}")"
  POINTS_FRAME_ID="$(sanitize_frame_id "${POINTS_FRAME_ID}")"
fi

kill "${play_pid}" 2>/dev/null || true
wait "${play_pid}" 2>/dev/null || true
if [[ "${USE_PHASE_DOMAINS}" == "true" ]]; then
  unset ROS_DOMAIN_ID || true
fi

if [[ -z "${POINTS_FRAME_ID}" ]]; then
  if is_ntu_viral_tnp01_bag "${BAG_PATH}"; then
    POINTS_FRAME_ID="sensor1/os_sensor"
    echo "warn: frame detection failed; using sensor1/os_sensor fallback for tnp_01" >&2
  elif [[ "${POINTS_TOPIC}" == *"livox"* ]]; then
    POINTS_FRAME_ID="livox_frame"
    echo "warn: frame detection failed; using livox_frame fallback for topic ${POINTS_TOPIC}" >&2
  else
    POINTS_FRAME_ID="${ROBOT_FRAME_ID}"
  fi
  echo "warn: failed to detect PointCloud2 frame_id; using ${POINTS_FRAME_ID}" >&2
else
  POINTS_FRAME_ID="${POINTS_FRAME_ID}"
fi
if [[ -z "${BASE_FRAME}" ]]; then
  BASE_FRAME="${POINTS_FRAME_ID}"
fi
if [[ -z "${LIDAR_FRAME}" ]]; then
  LIDAR_FRAME="${POINTS_FRAME_ID}"
fi
if [[ "${ROBOT_FRAME_ID_USER_SPECIFIED}" != "true" ]] && is_ntu_viral_tnp01_bag "${BAG_PATH}"; then
  ROBOT_FRAME_ID="${POINTS_FRAME_ID}"
fi
echo "points frame: ${POINTS_FRAME_ID}"
echo "base frame:   ${BASE_FRAME}"
echo "lidar frame:  ${LIDAR_FRAME}"
echo "points frame user specified: ${POINTS_FRAME_ID_USER_SPECIFIED}"

write_metrics_json

LIDARSLAM_DIR="${OUT_DIR}/lidarslam"
mkdir -p "${LIDARSLAM_DIR}"
LIDARSLAM_TUM="${LIDARSLAM_DIR}/traj_${GLOBAL_FRAME_ID}_${POINTS_FRAME_ID}.tum"
LIDARSLAM_PARAM_FILE="${PARAM_FILE}"

lidarslam_args=(
  --bag "${BAG_PATH}"
  --save-dir "${LIDARSLAM_DIR}"
  --points-topic "${POINTS_TOPIC}"
  --use-sim-time "${USE_SIM_TIME}"
  --global-frame-id "${GLOBAL_FRAME_ID}"
  --odom-frame-id "${ODOM_FRAME_ID}"
  --robot-frame-id "${ROBOT_FRAME_ID}"
  --base-frame "${BASE_FRAME}"
  --lidar-frame "${LIDAR_FRAME}"
  --points-frame-id "${POINTS_FRAME_ID}"
  --tum-out "${LIDARSLAM_TUM}"
  --tum-parent-frame "${GLOBAL_FRAME_ID}"
  --tum-child-frame "${POINTS_FRAME_ID}"
)
if [[ -n "${PARAM_FILE}" ]]; then
  lidarslam_args+=(--param "${PARAM_FILE}")
fi
if [[ "${NO_IMU}" == "true" || "${LIDARSLAM_FORCE_NO_IMU}" == "true" ]]; then
  lidarslam_args+=(--no-imu)
else
  lidarslam_args+=(--imu-topic "${IMU_TOPIC}")
fi
if [[ "${USE_RVIZ}" == "true" ]]; then
  lidarslam_args+=(--rviz)
fi
if [[ "${USE_GRAPH_BASED_SLAM}" == "false" ]]; then
  lidarslam_args+=(--no-graph-based-slam)
fi
if [[ "${AUTO_STATIC_TF}" == "true" ]]; then
  lidarslam_args+=(--auto-static-tf --auto-static-tf-timeout "${AUTO_STATIC_TF_TIMEOUT}")
fi

echo
echo "running lidarslam..."
LIDARSLAM_LOG="${LIDARSLAM_DIR}/run_bag_demo.log"
lidarslam_t0="$(now_monotonic)"
set +e
"${LIDARSLAM_CMD_PREFIX[@]}" bash "${REPO_ROOT}/scripts/run_bag_demo.sh" "${lidarslam_args[@]}" 2>&1 | tee "${LIDARSLAM_LOG}"
LIDARSLAM_RC="${PIPESTATUS[0]}"
set -e
lidarslam_t1="$(now_monotonic)"
LIDARSLAM_WALL_SEC="$(T0="${lidarslam_t0}" T1="${lidarslam_t1}" python3 - <<'PY'
import os
t0 = float(os.environ["T0"])
t1 = float(os.environ["T1"])
print(max(0.0, t1 - t0))
PY
)"
if [[ -f "${LIDARSLAM_TUM}" ]]; then
  LIDARSLAM_TUM_LINES="$(wc -l < "${LIDARSLAM_TUM}" | tr -d ' ')"
fi
if [[ "${LIDARSLAM_RC}" -eq 0 && "${LIDARSLAM_TUM_LINES}" -ge 2 ]]; then
  LIDARSLAM_SUCCESS="true"
fi
if [[ "${BAG_DURATION_SEC}" != "0" && "${LIDARSLAM_WALL_SEC}" != "" ]]; then
  LIDARSLAM_RTF="$(D="${BAG_DURATION_SEC}" W="${LIDARSLAM_WALL_SEC}" python3 - <<'PY'
import os
d = float(os.environ.get("D", "0") or "0")
w = float(os.environ.get("W", "0") or "0")
if d > 0.0:
    print(w / d)
PY
)"
fi

write_metrics_json

if [[ "${SKIP_GLIM}" == "true" ]]; then
  echo
  echo "skip_glim=true"
  echo "lidarslam tum: ${LIDARSLAM_TUM}"
  write_metrics_json
  exit 0
fi

GLIM_OUT="${OUT_DIR}/glim"
mkdir -p "${GLIM_OUT}"
GLIM_LOG="${GLIM_OUT}/glim_rosbag.log"
GLIM_DUMP_DIR="${GLIM_OUT}/dump"
mkdir -p "${GLIM_DUMP_DIR}"

if [[ -z "${GLIM_DOCKER_IMAGE}" ]] && ! ros2 pkg prefix glim_ros >/dev/null 2>&1; then
  echo
  echo "warn: glim_ros not found. Install GLIM (ROS 2) to run the comparison step." >&2
  GLIM_FAILURE_REASON="glim_ros_missing"
  GLIM_CACHE_STATUS="IDENTITY_UNAVAILABLE_RUNTIME"
  echo "warn: verified cache reuse requires the current GLIM runtime identity" >&2
  echo "lidarslam tum: ${LIDARSLAM_TUM}"
  write_metrics_json
  exit 0
fi

if [[ "${RUN_FRESH_GLIM}" == "true" ]]; then
  GLIM_AVAILABLE="true"

  glim_config_dir=""
  if [[ -n "${GLIM_CONFIG_PATH}" ]]; then
    glim_config_dir="${GLIM_CONFIG_PATH}"
  else
    GLIM_PREFIX="$(ros2 pkg prefix glim 2>/dev/null || true)"
    [[ -n "${GLIM_PREFIX}" ]] || die "glim package not found (needed for preset configs). Install GLIM or pass --glim-config-path"

    preset_src="${GLIM_PREFIX}/share/glim/config/presets/${GLIM_PRESET}"
    if [[ -d "${preset_src}" ]] && [[ -f "${preset_src}/config.json" ]]; then
      :
    else
      preset_root="${GLIM_PREFIX}/share/glim/config"
      [[ -f "${preset_root}/config.json" ]] || die "GLIM config not found: ${preset_root}"
      echo "warn: GLIM preset not found: ${preset_src}; using fallback config directory"
      preset_src="${preset_root}"
    fi

    glim_config_dir="${OUT_DIR}/glim_config_${GLIM_PRESET}"
    rm -rf "${glim_config_dir}"
    cp -R "${preset_src}" "${glim_config_dir}"

    echo
    if [[ -z "${GLIM_CONFIG_PATH}" ]]; then
      echo "preparing GLIM config: ${glim_config_dir}"
      patch_glim_config "${glim_config_dir}" "${GLIM_MODE}" "${POINTS_TOPIC}" "${IMU_TOPIC}" || true
    fi
  fi

  GLIM_CONFIG_REAL="$(realpath "${glim_config_dir}")"
  prepare_glim_cache_identity || true

  echo
  echo "running GLIM (preset=${GLIM_PRESET}, mode=${GLIM_MODE})..."
  GLIM_RUN_START_SEC="$(date +%s)"
  pre_dump="$(find /tmp -maxdepth 4 -type d \( -name 'dump*' -o -name 'dump' \) -print 2>/dev/null | sort -u || true)"
  glim_t0="$(now_monotonic)"
  set +e
  if [[ -n "${GLIM_DOCKER_IMAGE}" ]]; then
    GLIM_CONFIG_MOUNT="$(realpath "${glim_config_dir}")"
    GLIM_BAG_MOUNT="$(realpath "${BAG_PATH}")"
    GLIM_DUMP_MOUNT="$(realpath "${GLIM_DUMP_DIR}")"
    if [[ "${GLIM_TIMEOUT_SEC}" != "0" ]] && command -v timeout >/dev/null 2>&1; then
      timeout "${GLIM_TIMEOUT_SEC}" docker run --rm --gpus all \
        ${GLIM_OMP_NUM_THREADS:+-e OMP_NUM_THREADS=${GLIM_OMP_NUM_THREADS}} \
        -v "${GLIM_CONFIG_MOUNT}:/config:ro" \
        -v "${GLIM_BAG_MOUNT}:/bag:ro" \
        -v "${GLIM_DUMP_MOUNT}:/dump" \
        "${GLIM_DOCKER_IMAGE}" \
        bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run glim_ros glim_rosbag /bag --ros-args -p config_path:=/config -p auto_quit:=true -p dump_path:=/dump" \
        >"${GLIM_LOG}" 2>&1
    else
      docker run --rm --gpus all \
        ${GLIM_OMP_NUM_THREADS:+-e OMP_NUM_THREADS=${GLIM_OMP_NUM_THREADS}} \
        -v "${GLIM_CONFIG_MOUNT}:/config:ro" \
        -v "${GLIM_BAG_MOUNT}:/bag:ro" \
        -v "${GLIM_DUMP_MOUNT}:/dump" \
        "${GLIM_DOCKER_IMAGE}" \
        bash -lc "source /opt/ros/jazzy/setup.bash && source /root/ros2_ws/install/setup.bash && ros2 run glim_ros glim_rosbag /bag --ros-args -p config_path:=/config -p auto_quit:=true -p dump_path:=/dump" \
        >"${GLIM_LOG}" 2>&1
    fi
    GLIM_RC="$?"
  elif [[ "${GLIM_TIMEOUT_SEC}" != "0" ]] && command -v timeout >/dev/null 2>&1; then
    if [[ "${GLIM_VIEWER}" == "true" ]]; then
      "${GLIM_CMD_PREFIX[@]}" timeout "${GLIM_TIMEOUT_SEC}" \
        ros2 run glim_ros glim_rosbag "${BAG_PATH}" \
        --ros-args -p config_path:="${GLIM_CONFIG_REAL}" -p auto_quit:=true -p dump_path:="${GLIM_DUMP_DIR}" \
        >"${GLIM_LOG}" 2>&1
    else
      "${GLIM_CMD_PREFIX[@]}" env DISPLAY= QT_QPA_PLATFORM=offscreen timeout "${GLIM_TIMEOUT_SEC}" \
        ros2 run glim_ros glim_rosbag "${BAG_PATH}" \
        --ros-args -p config_path:="${GLIM_CONFIG_REAL}" -p auto_quit:=true -p dump_path:="${GLIM_DUMP_DIR}" \
        >"${GLIM_LOG}" 2>&1
    fi
    GLIM_RC="$?"
  else
    if [[ "${GLIM_VIEWER}" == "true" ]]; then
      "${GLIM_CMD_PREFIX[@]}" ros2 run glim_ros glim_rosbag "${BAG_PATH}" \
        --ros-args -p config_path:="${GLIM_CONFIG_REAL}" -p auto_quit:=true -p dump_path:="${GLIM_DUMP_DIR}" \
        >"${GLIM_LOG}" 2>&1
    else
      "${GLIM_CMD_PREFIX[@]}" env DISPLAY= QT_QPA_PLATFORM=offscreen \
        ros2 run glim_ros glim_rosbag "${BAG_PATH}" \
        --ros-args -p config_path:="${GLIM_CONFIG_REAL}" -p auto_quit:=true -p dump_path:="${GLIM_DUMP_DIR}" \
        >"${GLIM_LOG}" 2>&1
    fi
    GLIM_RC="$?"
  fi
  set -e
  glim_t1="$(now_monotonic)"
  GLIM_WALL_SEC="$(T0="${glim_t0}" T1="${glim_t1}" python3 - <<'PY'
import os
t0 = float(os.environ["T0"])
t1 = float(os.environ["T1"])
print(max(0.0, t1 - t0))
PY
)"
  if [[ "${BAG_DURATION_SEC}" != "0" && "${GLIM_WALL_SEC}" != "" ]]; then
    GLIM_RTF="$(D="${BAG_DURATION_SEC}" W="${GLIM_WALL_SEC}" python3 - <<'PY'
import os
d = float(os.environ.get("D", "0") or "0")
w = float(os.environ.get("W", "0") or "0")
if d > 0.0:
    print(w / d)
PY
)"
  fi
  post_dump="$(find /tmp -maxdepth 4 -type d \( -name 'dump*' -o -name 'dump' \) -print 2>/dev/null | sort -u || true)"
  recent_traj="$(find_recent_glim_traj "${GLIM_RUN_START_SEC}")"
  if [[ -f "${GLIM_DUMP_DIR}/traj_lidar.txt" ]]; then
    recent_traj="${GLIM_DUMP_DIR}/traj_lidar.txt"
  fi

  new_dumps="${post_dump}"
  if [[ -n "${pre_dump}" ]]; then
    new_dumps="$(comm -13 <(printf '%s\n' ${pre_dump}) <(printf '%s\n' ${post_dump}) 2>/dev/null || true)"
  fi
  if [[ -z "${new_dumps}" ]]; then
    # fallback: infer from glim logs
    GLIM_HINT_DUMP="$(python3 - "${GLIM_LOG}" <<'PY'
import os
import re
import sys

log_path = sys.argv[1]
best = ""
if os.path.isfile(log_path):
    with open(log_path, "r", encoding="utf-8", errors="replace") as f:
        for line in f:
            m = re.search(r"saving to\s+([^\s]+)", line)
            if m:
                p = m.group(1).strip().strip('"')
                if p.endswith(".txt"):
                    p = os.path.dirname(p)
                if os.path.isdir(p):
                    best = p
print(best)
PY
)"
    if [[ -n "${GLIM_HINT_DUMP}" ]]; then
      new_dumps="${GLIM_HINT_DUMP}"
    fi
  fi
  if [[ -z "${new_dumps}" ]]; then
    # fallback: choose newest dump-like directory touched after run start
    new_dumps="$(python3 - "${GLIM_RUN_START_SEC}" <<'PY'
import os
import sys
import glob

start = float(sys.argv[1]) if len(sys.argv) > 1 else 0.0
best = ""
best_mtime = -1.0
for p in glob.glob("/tmp/**/dump_*", recursive=True) + glob.glob("/tmp/**/dump", recursive=True):
    if not os.path.isdir(p):
        continue
    try:
        m = os.path.getmtime(p)
    except Exception:
        continue
    if m >= start - 5 and m > best_mtime:
        best_mtime = m
        best = p
print(best)
PY
)"
  fi

  GLIM_TRAJ_SRC="${recent_traj}"
  post_dump=""
  if [[ -n "${GLIM_TRAJ_SRC}" ]]; then
    post_dump="$(dirname "${GLIM_TRAJ_SRC}")"
  else
    post_dump="$(printf '%s\n' ${new_dumps} | head -n 1 | tr -d '[:space:]')"
  fi

  if [[ -z "${post_dump}" && -z "${GLIM_TRAJ_SRC}" ]]; then
    if [[ "${GLIM_RC}" -eq 124 ]]; then
      GLIM_FAILURE_REASON="timeout"
    else
      GLIM_FAILURE_REASON="$(detect_glim_failure_reason "${GLIM_LOG}")"
      if [[ -z "${GLIM_FAILURE_REASON}" ]]; then
        GLIM_FAILURE_REASON="no_dump_detected"
      fi
    fi
    use_verified_glim_traj "${GLIM_FAILURE_REASON}" || true
  fi

  if [[ -z "${post_dump}" && -z "${GLIM_TRAJ}" ]]; then
    echo "warn: failed to detect a new GLIM dump directory" >&2
    echo "hint: check ${GLIM_LOG}" >&2
    echo "lidarslam tum: ${LIDARSLAM_TUM}"
    write_metrics_json
    exit 0
  fi

  if [[ -n "${post_dump}" ]]; then
    echo "glim dump:    ${post_dump}"
    GLIM_DUMP_DIR="${post_dump}"
  fi
  if [[ -z "${GLIM_TRAJ}" ]]; then
    if [[ -z "${GLIM_TRAJ_SRC}" && -n "${GLIM_DUMP_DIR}" ]]; then
      GLIM_TRAJ_SRC="$(find "${GLIM_DUMP_DIR}" -type f -name 'traj_lidar.txt' -print 2>/dev/null | sort | head -n 1 || true)"
    fi
    if [[ -n "${GLIM_DUMP_DIR}" ]]; then
      GLIM_ODO_SRC="$(find "${GLIM_DUMP_DIR}" -type f -name 'odom_lidar.txt' -print 2>/dev/null | sort | head -n 1 || true)"
      cp -f "${GLIM_ODO_SRC}" "${GLIM_OUT}/odom_lidar.txt" 2>/dev/null || true
    fi
    cp -f "${GLIM_TRAJ_SRC}" "${GLIM_OUT}/traj_lidar.txt" 2>/dev/null || true
    GLIM_TRAJ="${GLIM_OUT}/traj_lidar.txt"
    if [[ -f "${GLIM_TRAJ}" ]]; then
      GLIM_REFERENCE_SOURCE="fresh"
      persist_verified_glim_cache "${GLIM_TRAJ}" || true
    else
      if [[ "${GLIM_RC}" -eq 124 ]]; then
        GLIM_FAILURE_REASON="timeout"
      else
        GLIM_FAILURE_REASON="$(detect_glim_failure_reason "${GLIM_LOG}")"
        if [[ -z "${GLIM_FAILURE_REASON}" ]]; then
          GLIM_FAILURE_REASON="no_traj_in_dump"
        fi
      fi
      use_verified_glim_traj "${GLIM_FAILURE_REASON}" || true
    fi
  fi

  if [[ ! -f "${GLIM_TRAJ}" ]]; then
    if [[ -z "${GLIM_FAILURE_REASON}" ]]; then
      GLIM_FAILURE_REASON="no_glim_reference"
    fi
    echo "warn: GLIM traj_lidar.txt not available" >&2
    echo "hint: check ${GLIM_LOG}" >&2
    echo "lidarslam tum: ${LIDARSLAM_TUM}"
    write_metrics_json
    exit 0
  fi

  GLIM_TRAJ_LINES="$(wc -l < "${GLIM_TRAJ}" | tr -d ' ')"
  if [[ "${GLIM_TRAJ_LINES}" -ge 2 ]] && [[ "${GLIM_RC}" -eq 0 || "${GLIM_RC}" -eq 124 ]]; then
    GLIM_SUCCESS="true"
  fi
fi

write_metrics_json

echo
echo "lidarslam tum: ${LIDARSLAM_TUM}"
echo "glim traj:     ${GLIM_TRAJ}"

if command -v evo_ape >/dev/null 2>&1; then
  echo
  echo "running evo_ape (GLIM as reference)..."
  evo_ape tum "${GLIM_TRAJ}" "${LIDARSLAM_TUM}" -a --no_plot >"${EVO_APE_LOG}" 2>&1 || true
  echo "evo_ape log:   ${EVO_APE_LOG}"
else
  echo
  echo "evo_ape not found; using builtin APE fallback"
  python3 "${REPO_ROOT}/scripts/ape_from_tum.py" \
    --ref "${GLIM_TRAJ}" \
    --est "${LIDARSLAM_TUM}" \
    --out "${EVO_APE_LOG}" \
    || true
  echo "ape log:       ${EVO_APE_LOG}"
fi

write_metrics_json

#!/usr/bin/env bash
set -euo pipefail

# Phase 2 hard gate (docs/roadmap/v0.6.md): run the offline deterministic
# backend runner N times on the same recorded backend-input bag and require
# the loop-edge sets AND the optimized trajectories to be byte-identical
# across runs.
#
# Usage:
#   bash scripts/run_offline_determinism_check.sh \
#     --bag output/backend_replay_x/backend_input \
#     [--params lidarslam/param/lidarslam_mid360_rko_graph.yaml] \
#     [--setup /path/to/workspace/install/setup.bash] \
#     [--runs 3] [--output-dir output/offline_determinism_<timestamp>] \
#     [--ros-domain-base 140] [--resume] \
#     [--reference-tum output/glim_mid360_reference.tum] \
#     [--ape-interpolate] [--ape-max-time-diff 0.05] [--require-ape] \
#     [--save-maps] [--max-rtf 1.0] [--max-peak-rss-mib 4096] \
#     [--max-wall-cv-percent 5.0] [--param name:=value ...]
#
# --save-maps forwards refine_save_maps:=true to the runner so each run
# writes map_optimized.pcd / map_refined.pcd (used by the release gate to
# run the map-quality profile check on the gate-produced refined map).
# Repeatable --param name:=value arguments are appended after the parameter
# file and make one-knob ablations explicit without copying the full YAML.
#
# When --reference-tum is given, each run's trajectory_optimized.tum is also
# scored with scripts/ape_from_tum.py (report only by default; pass
# --require-ape to make scorer success and finite RMSE fail-closed). The
# default gate is byte identity, not an APE threshold. For sparse checkpoint references (e.g.
# total-station checkpoints recorded while the platform is stationary,
# which fall between submap-rate poses) pass --ape-interpolate together
# with a generous --ape-max-time-diff (e.g. 2.0). A markdown summary in the
# same shape as the legacy backend_replay_summary.md is always written.  The
# M4a receipt and JSON/YAML summaries additionally freeze the complete input
# tree, toolchain/environment, per-run /usr/bin/time -v measurements, and the
# registration-plugin receipt.  Thresholds are optional report-only gates;
# when supplied, every threshold is fail-closed.

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

BAG=""
PARAMS="${REPO_ROOT}/lidarslam/param/lidarslam_mid360_rko_graph.yaml"
SETUP_FILE="${REPO_ROOT}/../install/setup.bash"
if [[ ! -f "${SETUP_FILE}" ]]; then
  SETUP_FILE="${REPO_ROOT}/install/setup.bash"
fi
RUNS=3
OUTPUT_DIR="${REPO_ROOT}/output/offline_determinism_$(date +%Y%m%d_%H%M%S)"
REFERENCE_TUM=""
APE_INTERPOLATE=false
APE_MAX_TIME_DIFF="0.05"
REQUIRE_APE=false
SAVE_MAPS=false
PARAM_OVERRIDES=()
ROS_DOMAIN_BASE=140
RESUME=false
MAX_RTF=""
MAX_PEAK_RSS_MIB=""
MAX_WALL_CV_PERCENT=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --bag) BAG="$2"; shift 2 ;;
    --params) PARAMS="$2"; shift 2 ;;
    --setup) SETUP_FILE="$2"; shift 2 ;;
    --runs) RUNS="$2"; shift 2 ;;
    --output-dir) OUTPUT_DIR="$2"; shift 2 ;;
    --reference-tum) REFERENCE_TUM="$2"; shift 2 ;;
    --ape-interpolate) APE_INTERPOLATE=true; shift ;;
    --ape-max-time-diff) APE_MAX_TIME_DIFF="$2"; shift 2 ;;
    --require-ape) REQUIRE_APE=true; shift ;;
    --save-maps) SAVE_MAPS=true; shift ;;
    --ros-domain-base) ROS_DOMAIN_BASE="$2"; shift 2 ;;
    --resume) RESUME=true; shift ;;
    --max-rtf) MAX_RTF="$2"; shift 2 ;;
    --max-peak-rss-mib) MAX_PEAK_RSS_MIB="$2"; shift 2 ;;
    --max-wall-cv-percent) MAX_WALL_CV_PERCENT="$2"; shift 2 ;;
    --param)
      if [[ "${2:-}" != *:=* || "${2%%:=*}" == "" || "${2#*:=}" == "" ]]; then
        echo "--param expects name:=value" >&2
        exit 2
      fi
      PARAM_OVERRIDES+=("$2")
      shift 2
      ;;
    *) echo "unknown option: $1" >&2; exit 2 ;;
  esac
done

is_finite_nonnegative() {
  local value="$1"
  # awk converts arbitrary strings such as "abc" to zero.  Validate the
  # lexical form first so malformed measurements/thresholds cannot pass a
  # fail-closed gate through that coercion.
  if [[ ! "${value}" =~ ^\+?([0-9]+([.][0-9]*)?|[.][0-9]+)([eE][+-]?[0-9]+)?$ ]]; then
    return 1
  fi
  awk -v numeric_value="${value}" 'BEGIN {
    if (numeric_value != numeric_value || numeric_value < 0 || numeric_value >= 1e308) exit 1;
    exit 0;
  }'
}

validate_optional_threshold() {
  local option_name="$1"
  local value="$2"
  if ! is_finite_nonnegative "${value}"; then
    echo "${option_name} must be a finite non-negative number: ${value}" >&2
    exit 2
  fi
}

sha256_file() {
  sha256sum "$1" | awk '{print $1}'
}

sha256_tree() {
  local tree_root="$1"
  (
    cd "${tree_root}"
    while IFS= read -r -d '' relative_path; do
      printf '%s  %s\n' "$(sha256_file "${relative_path}")" "${relative_path}"
    done < <(find . -type f -printf '%P\0' | LC_ALL=C sort -z)
  ) | sha256sum | awk '{print $1}'
}

git_status_filtered() {
  if (( ${#GIT_OUTPUT_EXCLUDE_PATHSPEC[@]} )); then
    git -C "${REPO_ROOT}" status --porcelain=v1 --untracked-files=all -- . \
      "${GIT_OUTPUT_EXCLUDE_PATHSPEC[@]}"
  else
    git -C "${REPO_ROOT}" status --porcelain=v1 --untracked-files=all
  fi
}

git_diff_filtered() {
  if (( ${#GIT_OUTPUT_EXCLUDE_PATHSPEC[@]} )); then
    git -C "${REPO_ROOT}" diff --binary --no-ext-diff -- . \
      "${GIT_OUTPUT_EXCLUDE_PATHSPEC[@]}"
  else
    git -C "${REPO_ROOT}" diff --binary --no-ext-diff
  fi
}

git_cached_diff_filtered() {
  if (( ${#GIT_OUTPUT_EXCLUDE_PATHSPEC[@]} )); then
    git -C "${REPO_ROOT}" diff --cached --binary --no-ext-diff -- . \
      "${GIT_OUTPUT_EXCLUDE_PATHSPEC[@]}"
  else
    git -C "${REPO_ROOT}" diff --cached --binary --no-ext-diff
  fi
}

git_untracked_paths_filtered() {
  if (( ${#GIT_OUTPUT_EXCLUDE_PATHSPEC[@]} )); then
    git -C "${REPO_ROOT}" ls-files --others --exclude-standard -z -- . \
      "${GIT_OUTPUT_EXCLUDE_PATHSPEC[@]}"
  else
    git -C "${REPO_ROOT}" ls-files --others --exclude-standard -z
  fi
}

sha256_untracked_content() {
  local relative_path absolute_path
  {
    while IFS= read -r -d '' relative_path; do
      absolute_path="${REPO_ROOT}/${relative_path}"
      if [[ -f "${absolute_path}" ]]; then
        printf '%s  %s\n' "$(sha256_file "${absolute_path}")" "${relative_path}"
      elif [[ -L "${absolute_path}" ]]; then
        printf 'symlink:%s  %s\n' "$(readlink "${absolute_path}")" "${relative_path}"
      else
        printf 'other  %s\n' "${relative_path}"
      fi
    done < <(git_untracked_paths_filtered)
  } | LC_ALL=C sort | sha256sum | awk '{print $1}'
}

sha256_param_overrides() {
  if (( ${#PARAM_OVERRIDES[@]} == 0 )); then
    printf '' | sha256sum | awk '{print $1}'
  else
    printf '%s\0' "${PARAM_OVERRIDES[@]}" | sha256sum | awk '{print $1}'
  fi
}

parse_bag_duration_seconds() {
  local metadata_path="$1"
  local duration_ns
  duration_ns="$(awk '
    /^[^[:space:]]/ {in_duration = 0}
    /^[[:space:]]+duration:[[:space:]]*$/ {in_duration = 1; next}
    in_duration && /^[[:space:]]+nanoseconds:[[:space:]]*/ {
      print $2;
      exit
    }
  ' "${metadata_path}")"
  if [[ ! "${duration_ns}" =~ ^[0-9]+$ ]] || [[ "${duration_ns}" == "0" ]]; then
    echo "bag metadata has no positive duration.nanoseconds: ${metadata_path}" >&2
    return 1
  fi
  awk -v nanoseconds="${duration_ns}" 'BEGIN {printf "%.9f", nanoseconds / 1e9}'
}

parse_elapsed_seconds() {
  local elapsed="$1"
  local -a fields
  if [[ ! "${elapsed}" =~ ^[0-9]+:[0-5][0-9]([.][0-9]+)?$ &&
    ! "${elapsed}" =~ ^[0-9]+:[0-5][0-9]:[0-5][0-9]([.][0-9]+)?$ ]]; then
    return 1
  fi
  IFS=: read -r -a fields <<< "${elapsed}"
  case "${#fields[@]}" in
    2)
      awk -v minutes="${fields[0]}" -v seconds="${fields[1]}" \
        'BEGIN {printf "%.9f", minutes * 60 + seconds}'
      ;;
    3)
      awk -v hours="${fields[0]}" -v minutes="${fields[1]}" -v seconds="${fields[2]}" \
        'BEGIN {printf "%.9f", hours * 3600 + minutes * 60 + seconds}'
      ;;
    *)
      return 1
      ;;
  esac
}

time_metric() {
  local time_path="$1"
  local metric="$2"
  case "${metric}" in
    wall)
      sed -n 's/^[[:space:]]*Elapsed (wall clock) time (h:mm:ss or m:ss):[[:space:]]*//p' \
        "${time_path}" | head -n 1
      ;;
    peak_rss_kib)
      sed -n 's/^[[:space:]]*Maximum resident set size (kbytes):[[:space:]]*//p' \
        "${time_path}" | head -n 1
      ;;
    *)
      return 1
      ;;
  esac
}

receipt_field() {
  local receipt_path="$1"
  local field="$2"
  awk -F': ' -v field="${field}" '
    $1 == field {
      value = $2;
      sub(/^"/, "", value);
      sub(/"$/, "", value);
      print value;
      exit
    }
  ' "${receipt_path}"
}

registration_parameter_field() {
  local receipt_path="$1"
  local parameter="$2"
  local field="$3"
  awk -v parameter="${parameter}" -v field="${field}" '
    $0 == "  \"" parameter "\":" {in_parameter = 1; next}
    in_parameter && $0 ~ /^  \"/ {exit}
    in_parameter && index($0, "    " field ": ") == 1 {
      value = substr($0, length("    " field ": ") + 1)
      sub(/^\"/, "", value)
      sub(/\"$/, "", value)
      print value
      exit
    }
  ' "${receipt_path}"
}

ape_rmse_value() {
  local ape_path="$1"
  awk '/^[[:space:]]*rmse:[[:space:]]*/ {print $2; exit}' "${ape_path}"
}

validate_ape_artifact() {
  local run_dir="$1"
  local ape_path="${run_dir}/ape.txt"
  local rmse
  if [[ ! -s "${ape_path}" ]]; then
    echo "required APE report is missing: ${ape_path}" >&2
    return 1
  fi
  rmse="$(ape_rmse_value "${ape_path}")"
  if ! is_finite_nonnegative "${rmse}"; then
    echo "required APE RMSE is missing or non-finite: ${ape_path}" >&2
    return 1
  fi
}

validate_registration_receipt() {
  local run_dir="$1"
  local receipt_path="${run_dir}/registration_plugin_receipt.yaml"
  local field value
  if [[ ! -s "${receipt_path}" ]]; then
    echo "missing registration receipt: ${receipt_path}" >&2
    return 1
  fi
  for field in schema role backend_kind requested_class resolved_class \
    metadata_class_id api_major license target_policy correspondence_metric \
    capabilities_bits library_path plugin_manifest_path; do
    if ! grep -Eq "^${field}:[[:space:]]" "${receipt_path}"; then
      echo "registration receipt missing field '${field}': ${receipt_path}" >&2
      return 1
    fi
    value="$(receipt_field "${receipt_path}" "${field}")"
    if [[ -z "${value}" && "${field}" != "library_path" && \
          "${field}" != "plugin_manifest_path" ]]; then
      echo "registration receipt missing field '${field}': ${receipt_path}" >&2
      return 1
    fi
  done
  [[ "$(receipt_field "${receipt_path}" schema)" == "1" ]] || return 1
  [[ "$(receipt_field "${receipt_path}" role)" == "backend_loop" ]] || return 1
  [[ "$(receipt_field "${receipt_path}" backend_kind)" == "host_builtin" ]] || return 1
  [[ "$(receipt_field "${receipt_path}" requested_class)" == "lidarslam_builtin/NdtOmp" ]] || return 1
  [[ "$(receipt_field "${receipt_path}" resolved_class)" == "lidarslam_builtin/NdtOmp" ]] || return 1
  [[ "$(receipt_field "${receipt_path}" metadata_class_id)" == \
    "lidarslam_builtin/NdtOmp" ]] || return 1
  [[ "$(receipt_field "${receipt_path}" api_major)" == "1" ]] || return 1
  [[ "$(receipt_field "${receipt_path}" license)" == "BSD-2-Clause" ]] || return 1
  [[ "$(receipt_field "${receipt_path}" target_policy)" == "raw_target" ]] || return 1
  [[ "$(receipt_field "${receipt_path}" correspondence_metric)" == \
    "mean_distance" ]] || return 1
  [[ -z "$(receipt_field "${receipt_path}" library_path)" ]] || return 1
  [[ -z "$(receipt_field "${receipt_path}" plugin_manifest_path)" ]] || return 1
  for parameter in resolution transformation_epsilon maximum_iterations step_size \
    outlier_ratio num_threads target_cell_cache_capacity neighborhood_search_method; do
    if ! grep -Eq "^[[:space:]]+\"${parameter}\":[[:space:]]*$" "${receipt_path}"; then
      echo "registration receipt missing typed parameter '${parameter}': ${receipt_path}" >&2
      return 1
    fi
    if [[ -z "$(registration_parameter_field "${receipt_path}" "${parameter}" type)" ||
      -z "$(registration_parameter_field "${receipt_path}" "${parameter}" value)" ]]; then
      echo "registration receipt has incomplete typed parameter '${parameter}': ${receipt_path}" >&2
      return 1
    fi
  done
  [[ "$(registration_parameter_field "${receipt_path}" neighborhood_search_method type)" == "string" ]] || return 1
  [[ "$(registration_parameter_field "${receipt_path}" neighborhood_search_method value)" == "DIRECT7" ]] || return 1
  [[ "$(registration_parameter_field "${receipt_path}" maximum_iterations type)" == "integer" ]] || return 1
  [[ "$(registration_parameter_field "${receipt_path}" num_threads type)" == "integer" ]] || return 1
  [[ "$(registration_parameter_field "${receipt_path}" target_cell_cache_capacity type)" == "integer" ]] || return 1
  [[ "$(registration_parameter_field "${receipt_path}" target_cell_cache_capacity value)" == "3" ]] || return 1
  [[ "$(registration_parameter_field "${receipt_path}" resolution type)" == "double" ]] || return 1
  [[ "$(registration_parameter_field "${receipt_path}" transformation_epsilon type)" == "double" ]] || return 1
  [[ "$(registration_parameter_field "${receipt_path}" step_size type)" == "double" ]] || return 1
  [[ "$(registration_parameter_field "${receipt_path}" outlier_ratio type)" == "double" ]] || return 1
  for requirement in initial_guess aligned_source mean_correspondence_distance; do
    if ! grep -Eq "^[[:space:]]+${requirement}:[[:space:]]+true[[:space:]]*$" "${receipt_path}"; then
      echo "registration receipt missing required capability '${requirement}': ${receipt_path}" >&2
      return 1
    fi
  done
}

validate_completion_marker() {
  local run_dir="$1"
  local expected_run_index="$2"
  local marker_path="${run_dir}/.complete"
  local field expected actual
  if [[ ! -s "${marker_path}" ]]; then
    echo "run completion marker is missing: ${marker_path}" >&2
    return 1
  fi
  for field in schema run_index complete execution_identity_sha256 \
    script_sha256 runner_sha256 bag_tree_sha256 bag_metadata_sha256 \
    params_sha256 setup_sha256 reference_tum_sha256 fixed_loop_edges_sha256 \
    parameter_overrides_sha256 git_worktree_fingerprint \
    dynamic_dependency_fingerprint; do
    if ! grep -Eq "^${field}:[[:space:]]" "${marker_path}"; then
      echo "completion marker missing field '${field}': ${marker_path}" >&2
      return 1
    fi
    case "${field}" in
      schema) expected=2 ;;
      run_index) expected="${expected_run_index}" ;;
      complete) expected=true ;;
      execution_identity_sha256) expected="${EXECUTION_IDENTITY_SHA256}" ;;
      script_sha256) expected="${SCRIPT_SHA256}" ;;
      runner_sha256) expected="${RUNNER_SHA256}" ;;
      bag_tree_sha256) expected="${BAG_TREE_SHA256}" ;;
      bag_metadata_sha256) expected="${BAG_METADATA_SHA256}" ;;
      params_sha256) expected="${PARAMS_SHA256}" ;;
      setup_sha256) expected="${SETUP_SHA256}" ;;
      reference_tum_sha256) expected="${REFERENCE_TUM_SHA256:-none}" ;;
      fixed_loop_edges_sha256) expected="${FIXED_LOOP_EDGES_SHA256:-none}" ;;
      parameter_overrides_sha256) expected="${PARAM_OVERRIDES_SHA256}" ;;
      git_worktree_fingerprint) expected="${GIT_WORKTREE_FINGERPRINT}" ;;
      dynamic_dependency_fingerprint) expected="${DYNAMIC_DEPENDENCY_FINGERPRINT}" ;;
      *) return 1 ;;
    esac
    actual="$(receipt_field "${marker_path}" "${field}")"
    if [[ "${actual}" != "${expected}" ]]; then
      echo "completion marker identity mismatch for ${field}: ${marker_path}" >&2
      return 1
    fi
  done
}

validate_run_artifacts() {
  local run_dir="$1"
  local require_complete="${2:-true}"
  local expected_run_index="${3:-}"
  local required_artifact
  for required_artifact in loop_edges.csv trajectory_optimized.tum \
    registration_plugin_receipt.yaml run_time_v.txt run_metrics.yaml; do
    if [[ ! -s "${run_dir}/${required_artifact}" ]]; then
      echo "run is incomplete; missing artifact ${run_dir}/${required_artifact}" >&2
      return 1
    fi
  done
  if [[ "${require_complete}" == true ]]; then
    validate_completion_marker "${run_dir}" "${expected_run_index}" || return 1
  fi
  validate_registration_receipt "${run_dir}" || return 1
  if [[ "${REQUIRE_APE}" == true ]]; then
    validate_ape_artifact "${run_dir}" || return 1
  fi
}

load_run_metrics() {
  local run_dir="$1"
  local metrics_path="${run_dir}/run_metrics.yaml"
  local complete wall peak rtf
  complete="$(receipt_field "${metrics_path}" complete)"
  wall="$(receipt_field "${metrics_path}" wall_sec)"
  peak="$(receipt_field "${metrics_path}" peak_rss_mib)"
  rtf="$(receipt_field "${metrics_path}" rtf)"
  if [[ "${complete}" != true ]] || ! is_finite_nonnegative "${wall}" || \
    ! is_finite_nonnegative "${peak}" || \
    ! is_finite_nonnegative "${rtf}"; then
    echo "run metrics are missing or non-finite: ${metrics_path}" >&2
    return 1
  fi
  RUN_WALL_SEC+=("${wall}")
  RUN_PEAK_RSS_MIB+=("${peak}")
  RUN_RTF+=("${rtf}")
}

yaml_quote() {
  local value="$1"
  value=${value//\\/\\\\}
  value=${value//"/\\"}
  value=${value//$'\n'/\\n}
  printf '"%s"' "${value}"
}

if ! [[ "${RUNS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "--runs must be a positive integer: ${RUNS}" >&2
  exit 2
fi
if (( ROS_DOMAIN_BASE < 0 || ROS_DOMAIN_BASE + RUNS > 233 )); then
  echo "--ros-domain-base must leave one valid ROS domain (0..232) per run" >&2
  exit 2
fi

if [[ -z "${BAG}" ]]; then
  echo "--bag <backend_input bag dir> is required" >&2
  exit 2
fi
PARAMS="$(realpath -m "${PARAMS}")"
SETUP_FILE="$(realpath -m "${SETUP_FILE}")"
BAG="$(realpath -m "${BAG}")"
OUTPUT_DIR="$(realpath -m "${OUTPUT_DIR}")"
if [[ "${OUTPUT_DIR}" == "${REPO_ROOT}" ]]; then
  echo "--output-dir may not be the repository root" >&2
  exit 2
fi
GIT_OUTPUT_EXCLUDE_PATHSPEC=()
if [[ "${OUTPUT_DIR}" == "${REPO_ROOT}/"* ]]; then
  GIT_OUTPUT_EXCLUDE_PATHSPEC=(":(exclude)${OUTPUT_DIR#"${REPO_ROOT}/"}")
fi
if [[ ! -f "${SETUP_FILE}" ]]; then
  if [[ -n "${ROS_DISTRO:-}" ]] && command -v ros2 >/dev/null 2>&1; then
    SETUP_FILE="active_environment"
  else
    echo "setup file not found and no active ROS environment is available: ${SETUP_FILE}" >&2
    exit 2
  fi
fi
if [[ ! -f "${PARAMS}" ]]; then
  echo "params file not found: ${PARAMS}" >&2
  exit 2
fi
if [[ ! -d "${BAG}" ]]; then
  echo "bag directory not found: ${BAG}" >&2
  exit 2
fi
if [[ ! -x /usr/bin/time ]]; then
  echo "/usr/bin/time is required for fail-closed runtime measurement" >&2
  exit 2
fi
if [[ -n "${MAX_RTF}" ]]; then
  validate_optional_threshold "--max-rtf" "${MAX_RTF}"
fi
if [[ -n "${MAX_PEAK_RSS_MIB}" ]]; then
  validate_optional_threshold "--max-peak-rss-mib" "${MAX_PEAK_RSS_MIB}"
fi
if [[ -n "${MAX_WALL_CV_PERCENT}" ]]; then
  validate_optional_threshold "--max-wall-cv-percent" "${MAX_WALL_CV_PERCENT}"
  if (( RUNS < 2 )); then
    echo "--max-wall-cv-percent requires at least two runs" >&2
    exit 2
  fi
fi
if ! is_finite_nonnegative "${APE_MAX_TIME_DIFF}"; then
  echo "--ape-max-time-diff must be a finite non-negative number: ${APE_MAX_TIME_DIFF}" >&2
  exit 2
fi
if [[ ! -f "${BAG}/metadata.yaml" ]]; then
  echo "bag metadata.yaml is required for RTF measurement: ${BAG}" >&2
  exit 2
fi
BAG_DURATION_SEC="$(parse_bag_duration_seconds "${BAG}/metadata.yaml")" || exit 2
if ! is_finite_nonnegative "${BAG_DURATION_SEC}" || \
  ! awk -v duration="${BAG_DURATION_SEC}" 'BEGIN {exit !(duration > 0)}'; then
  echo "bag duration is invalid: ${BAG_DURATION_SEC}" >&2
  exit 2
fi
if [[ -n "${REFERENCE_TUM}" ]]; then
  REFERENCE_TUM="$(realpath -m "${REFERENCE_TUM}")"
  if [[ ! -f "${REFERENCE_TUM}" ]]; then
    echo "reference TUM file not found: ${REFERENCE_TUM}" >&2
    exit 2
  fi
fi
if [[ "${REQUIRE_APE}" == true && -z "${REFERENCE_TUM}" ]]; then
  echo "--require-ape requires --reference-tum" >&2
  exit 2
fi

if [[ "${SETUP_FILE}" != "active_environment" ]]; then
  # shellcheck disable=SC1091
  set +u
  source "${SETUP_FILE}"
  set -u
fi

GRAPH_PREFIX="$(ros2 pkg prefix graph_based_slam)"
RUNNER_EXECUTABLE="${GRAPH_PREFIX}/lib/graph_based_slam/graph_slam_offline_runner"
if [[ ! -x "${RUNNER_EXECUTABLE}" ]]; then
  echo "graph_slam_offline_runner not found under selected setup: ${RUNNER_EXECUTABLE}" >&2
  exit 2
fi
RUNNER_SHA256="$(sha256sum "${RUNNER_EXECUTABLE}" | awk '{print $1}')"
PARAMS_SHA256="$(sha256sum "${PARAMS}" | awk '{print $1}')"
BAG_METADATA_SHA256="$(sha256sum "${BAG}/metadata.yaml" | awk '{print $1}')"
BAG_TREE_SHA256="$(sha256_tree "${BAG}")"
SETUP_SHA256="$(sha256sum "${SETUP_FILE}" | awk '{print $1}')"
GIT_REVISION="$(git -C "${REPO_ROOT}" rev-parse HEAD 2>/dev/null || echo unknown)"
ROS_DISTRO_RECORDED="${ROS_DISTRO:-unknown}"
COMPILER_COMMAND="${CXX:-c++}"
COMPILER_BINARY="${COMPILER_COMMAND%% *}"
COMPILER_PATH="$(command -v "${COMPILER_BINARY}" 2>/dev/null || true)"
if [[ -n "${COMPILER_PATH}" ]]; then
  COMPILER_VERSION="$(${COMPILER_PATH} --version 2>/dev/null | head -n 1 || true)"
else
  COMPILER_VERSION="unknown"
fi
CPU_MODEL="$(awk -F: '/^model name[[:space:]]*:/ {gsub(/^[[:space:]]+/, "", $2); print $2; exit}' /proc/cpuinfo 2>/dev/null || true)"
CPU_MODEL="${CPU_MODEL:-unknown}"
HARDWARE_THREADS="$(nproc 2>/dev/null || getconf _NPROCESSORS_ONLN 2>/dev/null || echo unknown)"
MEMORY_TOTAL_MIB="$(awk '/^MemTotal:/ {printf "%.3f", $2 / 1024; exit}' /proc/meminfo 2>/dev/null || true)"
MEMORY_TOTAL_MIB="${MEMORY_TOTAL_MIB:-unknown}"
OPENMP_RUNTIME="$(ldd "${RUNNER_EXECUTABLE}" 2>/dev/null | awk '/lib(gomp|omp|iomp)/ {print $1; exit}' || true)"
OPENMP_RUNTIME="${OPENMP_RUNTIME:-unknown}"
DYNAMIC_DEPENDENCIES="$(
  {
    ldd "${RUNNER_EXECUTABLE}" 2>/dev/null |
      awk '/=>[[:space:]]+\// {print $3} /^[[:space:]]*\// {print $1}' |
      LC_ALL=C sort -u |
      while IFS= read -r dependency_path; do
        if [[ -f "${dependency_path}" ]]; then
          printf '%s  %s\n' "$(sha256_file "${dependency_path}")" "${dependency_path}"
        else
          printf 'unresolved  %s\n' "${dependency_path}"
        fi
      done
  } || true
)"
DYNAMIC_DEPENDENCY_FINGERPRINT="$(printf '%s\n' "${DYNAMIC_DEPENDENCIES}" | sha256sum | awk '{print $1}')"
OMP_NUM_THREADS_RECORDED="${OMP_NUM_THREADS:-}"
OMP_DYNAMIC_RECORDED="${OMP_DYNAMIC:-}"
OMP_PROC_BIND_RECORDED="${OMP_PROC_BIND:-}"
OMP_PLACES_RECORDED="${OMP_PLACES:-}"
REFERENCE_TUM_SHA256=""
if [[ -n "${REFERENCE_TUM}" ]]; then
  REFERENCE_TUM_SHA256="$(sha256sum "${REFERENCE_TUM}" | awk '{print $1}')"
fi
FIXED_LOOP_EDGES_PATH=""
for override in "${PARAM_OVERRIDES[@]}"; do
  if [[ "${override}" == fixed_loop_edges_path:=* ]]; then
    FIXED_LOOP_EDGES_PATH="${override#*:=}"
  fi
done
FIXED_LOOP_EDGES_SHA256=""
if [[ -n "${FIXED_LOOP_EDGES_PATH}" ]]; then
  FIXED_LOOP_EDGES_PATH="$(realpath -m "${FIXED_LOOP_EDGES_PATH}")"
  if [[ ! -f "${FIXED_LOOP_EDGES_PATH}" ]]; then
    echo "fixed loop edge CSV not found: ${FIXED_LOOP_EDGES_PATH}" >&2
    exit 2
  fi
  FIXED_LOOP_EDGES_SHA256="$(sha256sum "${FIXED_LOOP_EDGES_PATH}" | awk '{print $1}')"
fi
SCRIPT_PATH="${SCRIPT_DIR}/run_offline_determinism_check.sh"
SCRIPT_SHA256="$(sha256sum "${SCRIPT_PATH}" | awk '{print $1}')"
GIT_STATUS_TEXT="$(git_status_filtered 2>/dev/null || true)"
if [[ -n "${GIT_STATUS_TEXT}" ]]; then
  GIT_DIRTY=true
else
  GIT_DIRTY=false
fi
GIT_STATUS_SHA256="$(printf '%s\n' "${GIT_STATUS_TEXT}" | sha256sum | awk '{print $1}')"
GIT_TRACKED_DIFF_SHA256="$({ git_diff_filtered 2>/dev/null || true; git_cached_diff_filtered 2>/dev/null || true; } | sha256sum | awk '{print $1}')"
GIT_UNTRACKED_CONTENT_SHA256="$(sha256_untracked_content)"
GIT_WORKTREE_FINGERPRINT="$({
  printf 'status_sha256=%s\n' "${GIT_STATUS_SHA256}"
  printf 'tracked_diff_sha256=%s\n' "${GIT_TRACKED_DIFF_SHA256}"
  printf 'untracked_content_sha256=%s\n' "${GIT_UNTRACKED_CONTENT_SHA256}"
} | sha256sum | awk '{print $1}')"
PARAM_OVERRIDES_SHA256="$(sha256_param_overrides)"
REFERENCE_TUM_MARKER_SHA256="${REFERENCE_TUM_SHA256:-none}"
FIXED_LOOP_EDGES_MARKER_SHA256="${FIXED_LOOP_EDGES_SHA256:-none}"
EXECUTION_IDENTITY_SHA256="$({
  printf 'script_sha256=%s\n' "${SCRIPT_SHA256}"
  printf 'git_worktree_fingerprint=%s\n' "${GIT_WORKTREE_FINGERPRINT}"
  printf 'runner_sha256=%s\n' "${RUNNER_SHA256}"
  printf 'params_sha256=%s\n' "${PARAMS_SHA256}"
  printf 'bag_tree_sha256=%s\n' "${BAG_TREE_SHA256}"
  printf 'bag_metadata_sha256=%s\n' "${BAG_METADATA_SHA256}"
  printf 'bag_duration_sec=%s\n' "${BAG_DURATION_SEC}"
  printf 'setup_sha256=%s\n' "${SETUP_SHA256}"
  printf 'ros_distro=%s\n' "${ROS_DISTRO_RECORDED}"
  printf 'compiler_command=%s\n' "${COMPILER_COMMAND}"
  printf 'compiler_path=%s\n' "${COMPILER_PATH}"
  printf 'compiler_version=%s\n' "${COMPILER_VERSION}"
  printf 'cpu_model=%s\n' "${CPU_MODEL}"
  printf 'hardware_threads=%s\n' "${HARDWARE_THREADS}"
  printf 'memory_total_mib=%s\n' "${MEMORY_TOTAL_MIB}"
  printf 'omp_runtime=%s\n' "${OPENMP_RUNTIME}"
  printf 'omp_num_threads=%s\n' "${OMP_NUM_THREADS_RECORDED}"
  printf 'omp_dynamic=%s\n' "${OMP_DYNAMIC_RECORDED}"
  printf 'omp_proc_bind=%s\n' "${OMP_PROC_BIND_RECORDED}"
  printf 'omp_places=%s\n' "${OMP_PLACES_RECORDED}"
  printf 'dynamic_dependency_fingerprint=%s\n' "${DYNAMIC_DEPENDENCY_FINGERPRINT}"
  printf 'reference_tum_sha256=%s\n' "${REFERENCE_TUM_MARKER_SHA256}"
  printf 'fixed_loop_edges_sha256=%s\n' "${FIXED_LOOP_EDGES_MARKER_SHA256}"
  printf 'parameter_overrides_sha256=%s\n' "${PARAM_OVERRIDES_SHA256}"
  printf 'save_maps=%s\n' "${SAVE_MAPS}"
  printf 'require_ape=%s\n' "${REQUIRE_APE}"
  printf 'ape_interpolate=%s\n' "${APE_INTERPOLATE}"
  printf 'ape_max_time_diff=%s\n' "${APE_MAX_TIME_DIFF}"
  printf 'ros_domain_base=%s\n' "${ROS_DOMAIN_BASE}"
} | sha256sum | awk '{print $1}')"

RUN_WALL_SEC=()
RUN_PEAK_RSS_MIB=()
RUN_RTF=()

mkdir -p "${OUTPUT_DIR}"
echo "bag:    ${BAG}"
echo "params: ${PARAMS}"
echo "setup:  ${SETUP_FILE}"
echo "runner: ${RUNNER_EXECUTABLE} (${RUNNER_SHA256})"
echo "runs:   ${RUNS}"
echo "out:    ${OUTPUT_DIR}"

# Validate every existing run directory before starting any new process.  A
# later stale directory must not allow run1 to execute and then leave a mixed
# or partially measured output tree behind.
for i in $(seq 1 "${RUNS}"); do
  run_dir="${OUTPUT_DIR}/run${i}"
  if [[ ! -d "${run_dir}" ]]; then
    continue
  fi
  if [[ "${RESUME}" == true && -f "${run_dir}/.complete" ]]; then
    if ! validate_run_artifacts "${run_dir}" true "${i}"; then
      echo "refusing to resume invalid, partial, or identity-mismatched run ${i}: ${run_dir}" >&2
      exit 1
    fi
  elif [[ -n "$(find "${run_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
    if [[ "${RESUME}" == true ]]; then
      echo "refusing to resume partial run without a valid completion marker ${i}: ${run_dir}" >&2
      exit 1
    fi
    echo "run directory is non-empty; choose a new --output-dir or pass --resume: ${run_dir}" >&2
    exit 2
  fi
done

for i in $(seq 1 "${RUNS}"); do
  run_dir="${OUTPUT_DIR}/run${i}"
  echo "--- run ${i}/${RUNS}"
  if [[ -d "${run_dir}" ]]; then
    if [[ "${RESUME}" == true ]]; then
      if [[ -f "${run_dir}/.complete" ]]; then
        if ! validate_run_artifacts "${run_dir}" true "${i}" || \
          ! load_run_metrics "${run_dir}"; then
          echo "refusing to resume invalid, partial, or identity-mismatched run ${i}: ${run_dir}" >&2
          exit 1
        fi
        echo "reuse complete run ${i}: ${run_dir}"
        continue
      fi
      if [[ -n "$(find "${run_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
        echo "refusing to resume partial run without a valid completion marker ${i}: ${run_dir}" >&2
        exit 1
      fi
    elif [[ -n "$(find "${run_dir}" -mindepth 1 -maxdepth 1 -print -quit)" ]]; then
      echo "run directory is non-empty; choose a new --output-dir or pass --resume: ${run_dir}" >&2
      exit 2
    fi
  fi
  mkdir -p "${run_dir}"
  RUNNER_CMD=(
    "${RUNNER_EXECUTABLE}" --ros-args
    --disable-rosout-logs
    --params-file "${PARAMS}"
    -p bag_path:="${BAG}"
    -p output_dir:="${run_dir}"
  )
  if [[ "${SAVE_MAPS}" == "true" ]]; then
    RUNNER_CMD+=(-p refine_save_maps:=true)
  fi
  for override in "${PARAM_OVERRIDES[@]}"; do
    RUNNER_CMD+=(-p "${override}")
  done
  if LC_ALL=C /usr/bin/time -v -o "${run_dir}/run_time_v.txt" \
    env ROS_DOMAIN_ID=$((ROS_DOMAIN_BASE + i - 1)) ROS_LOCALHOST_ONLY=1 \
    "${RUNNER_CMD[@]}" > "${run_dir}/runner.log" 2>&1; then
    runner_status=0
  else
    runner_status=$?
  fi
  if (( runner_status != 0 )); then
    echo "offline runner failed in run ${i} (status=${runner_status}); refusing completion" >&2
    exit 1
  fi
  if ! validate_registration_receipt "${run_dir}"; then
    echo "registration receipt validation failed in run ${i}" >&2
    exit 1
  fi
  elapsed_raw="$(time_metric "${run_dir}/run_time_v.txt" wall)"
  peak_rss_kib="$(time_metric "${run_dir}/run_time_v.txt" peak_rss_kib)"
  wall_sec="$(parse_elapsed_seconds "${elapsed_raw}")" || {
    echo "missing or invalid /usr/bin/time wall measurement in run ${i}" >&2
    exit 1
  }
  if ! [[ "${peak_rss_kib}" =~ ^[0-9]+$ ]] || [[ "${peak_rss_kib}" == "0" ]]; then
    echo "missing or invalid /usr/bin/time peak RSS measurement in run ${i}" >&2
    exit 1
  fi
  peak_rss_mib="$(awk -v kib="${peak_rss_kib}" 'BEGIN {printf "%.9f", kib / 1024}')"
  rtf="$(awk -v wall="${wall_sec}" -v duration="${BAG_DURATION_SEC}" \
    'BEGIN {printf "%.9f", wall / duration}')"
  if ! is_finite_nonnegative "${wall_sec}" || ! is_finite_nonnegative "${peak_rss_mib}" || \
    ! is_finite_nonnegative "${rtf}"; then
    echo "non-finite runtime measurement in run ${i}" >&2
    exit 1
  fi
  printf 'schema: 1\nrun_index: %s\ncomplete: true\nwall_sec: %s\npeak_rss_kib: %s\npeak_rss_mib: %s\nbag_duration_sec: %s\nrtf: %s\ntime_source: /usr/bin/time -v\n' \
    "${i}" "${wall_sec}" "${peak_rss_kib}" "${peak_rss_mib}" \
    "${BAG_DURATION_SEC}" "${rtf}" > "${run_dir}/run_metrics.yaml"
  RUN_WALL_SEC+=("${wall_sec}")
  RUN_PEAK_RSS_MIB+=("${peak_rss_mib}")
  RUN_RTF+=("${rtf}")
  md5sum "${run_dir}/loop_edges.csv" "${run_dir}/trajectory_optimized.tum"
  if [[ -n "${REFERENCE_TUM}" ]]; then
    APE_CMD=(
      python3 "${SCRIPT_DIR}/ape_from_tum.py"
      --ref "${REFERENCE_TUM}"
      --est "${run_dir}/trajectory_optimized.tum"
      --out "${run_dir}/ape.txt"
      --max-time-diff "${APE_MAX_TIME_DIFF}"
    )
    if [[ "${APE_INTERPOLATE}" == "true" ]]; then
      APE_CMD+=(--interpolate)
    fi
    if "${APE_CMD[@]}" > "${run_dir}/ape_postprocess.log" 2>&1; then
      if [[ "${REQUIRE_APE}" == true ]] && ! validate_ape_artifact "${run_dir}"; then
        echo "required APE gate failed in run ${i}" >&2
        exit 1
      fi
    elif [[ "${REQUIRE_APE}" == true ]]; then
      echo "required APE post-processing failed in run ${i}" >&2
      exit 1
    else
      echo "WARN: APE post-processing failed in run ${i}; continuing" >&2
    fi
  fi
  if ! validate_run_artifacts "${run_dir}" false; then
    echo "required artifacts failed validation in run ${i}" >&2
    exit 1
  fi
  printf 'schema: 2\nrun_index: %s\ncomplete: true\nexecution_identity_sha256: %s\nscript_sha256: %s\nrunner_sha256: %s\nbag_tree_sha256: %s\nbag_metadata_sha256: %s\nparams_sha256: %s\nsetup_sha256: %s\nreference_tum_sha256: %s\nfixed_loop_edges_sha256: %s\nparameter_overrides_sha256: %s\ngit_worktree_fingerprint: %s\ndynamic_dependency_fingerprint: %s\n' \
    "${i}" "${EXECUTION_IDENTITY_SHA256}" "${SCRIPT_SHA256}" \
    "${RUNNER_SHA256}" "${BAG_TREE_SHA256}" "${BAG_METADATA_SHA256}" \
    "${PARAMS_SHA256}" "${SETUP_SHA256}" "${REFERENCE_TUM_MARKER_SHA256}" \
    "${FIXED_LOOP_EDGES_MARKER_SHA256}" "${PARAM_OVERRIDES_SHA256}" \
    "${GIT_WORKTREE_FINGERPRINT}" "${DYNAMIC_DEPENDENCY_FINGERPRINT}" \
    > "${run_dir}/.complete"
  if ! validate_run_artifacts "${run_dir}" true "${i}"; then
    echo "completion marker validation failed in run ${i}" >&2
    exit 1
  fi
done

ape_rmse_for_run() {
  local run_dir="$1"
  if [[ -f "${run_dir}/ape.txt" ]]; then
    ape_rmse_value "${run_dir}/ape.txt"
  fi
}

echo "--- verdict"
edges_ref="${OUTPUT_DIR}/run1/loop_edges.csv"
traj_ref="${OUTPUT_DIR}/run1/trajectory_optimized.tum"
edge_count=$(($(wc -l < "${edges_ref}") - 1))
status=0
loop_edges_identical=true
optimized_trajectories_identical=true
registration_receipts_identical=true
for i in $(seq 2 "${RUNS}"); do
  if ! diff -q "${edges_ref}" "${OUTPUT_DIR}/run${i}/loop_edges.csv" > /dev/null; then
    echo "MISMATCH (loop edges): run1 vs run${i}"
    diff "${edges_ref}" "${OUTPUT_DIR}/run${i}/loop_edges.csv" | head -10 || true
    loop_edges_identical=false
    status=1
  fi
  if ! diff -q "${traj_ref}" "${OUTPUT_DIR}/run${i}/trajectory_optimized.tum" > /dev/null; then
    echo "MISMATCH (optimized trajectory): run1 vs run${i}"
    optimized_trajectories_identical=false
    status=1
  fi
  if ! diff -q "${OUTPUT_DIR}/run1/registration_plugin_receipt.yaml" \
    "${OUTPUT_DIR}/run${i}/registration_plugin_receipt.yaml" > /dev/null; then
    echo "MISMATCH (registration receipt): run1 vs run${i}"
    registration_receipts_identical=false
    status=1
  fi
  # v0.7 refinement artifacts (present only when the runner ran with
  # refine:=true) join the byte-identity contract.
  for refined_artifact in trajectory_refined.tum map_refinement_report.yaml plane_revisit_report.yaml; do
    if [[ -f "${OUTPUT_DIR}/run1/${refined_artifact}" ]]; then
      if [[ ! -f "${OUTPUT_DIR}/run${i}/${refined_artifact}" ]] || \
        ! diff -q "${OUTPUT_DIR}/run1/${refined_artifact}" "${OUTPUT_DIR}/run${i}/${refined_artifact}" > /dev/null; then
        echo "MISMATCH (${refined_artifact}): run1 vs run${i}"
        status=1
      fi
    elif [[ -f "${OUTPUT_DIR}/run${i}/${refined_artifact}" ]]; then
      echo "MISMATCH (${refined_artifact}): run1 vs run${i}"
      status=1
    fi
  done
done

max_rtf_observed="$(printf '%s\n' "${RUN_RTF[@]}" | awk '
  BEGIN {max = -1}
  NF {if ($1 > max) max = $1}
  END {if (max < 0) exit 1; printf "%.9f", max}
')" || {
  echo "runtime factor measurements are missing" >&2
  status=1
  max_rtf_observed="nan"
}
max_peak_rss_mib_observed="$(printf '%s\n' "${RUN_PEAK_RSS_MIB[@]}" | awk '
  BEGIN {max = -1}
  NF {if ($1 > max) max = $1}
  END {if (max < 0) exit 1; printf "%.9f", max}
')" || {
  echo "peak RSS measurements are missing" >&2
  status=1
  max_peak_rss_mib_observed="nan"
}
wall_stats="$(printf '%s\n' "${RUN_WALL_SEC[@]}" | awk '
  NF {sum += $1; values[n++] = $1}
  END {
    if (n == 0) exit 1;
    mean = sum / n;
    for (i = 0; i < n; ++i) variance += (values[i] - mean) * (values[i] - mean);
    printf "%.9f %.9f", mean, (mean > 0 ? sqrt(variance / n) / mean * 100 : 0);
  }
')" || {
  echo "wall-time measurements are missing" >&2
  status=1
  wall_stats="nan nan"
}
read -r wall_mean_sec wall_cv_percent <<< "${wall_stats}"
for aggregate in "${max_rtf_observed}" "${max_peak_rss_mib_observed}" \
  "${wall_mean_sec}" "${wall_cv_percent}"; do
  if ! is_finite_nonnegative "${aggregate}"; then
    echo "runtime aggregate measurement is missing or non-finite" >&2
    status=1
  fi
done
if [[ -n "${MAX_RTF}" ]] && ! awk -v actual="${max_rtf_observed}" -v limit="${MAX_RTF}" \
  'BEGIN {exit !(actual == actual && actual <= limit)}'; then
  echo "FAIL: max RTF ${max_rtf_observed} exceeds --max-rtf ${MAX_RTF}" >&2
  status=1
fi
if [[ -n "${MAX_PEAK_RSS_MIB}" ]] && \
  ! awk -v actual="${max_peak_rss_mib_observed}" -v limit="${MAX_PEAK_RSS_MIB}" \
    'BEGIN {exit !(actual == actual && actual <= limit)}'; then
  echo "FAIL: peak RSS ${max_peak_rss_mib_observed} MiB exceeds --max-peak-rss-mib ${MAX_PEAK_RSS_MIB}" >&2
  status=1
fi
if [[ -n "${MAX_WALL_CV_PERCENT}" ]] && \
  ! awk -v actual="${wall_cv_percent}" -v limit="${MAX_WALL_CV_PERCENT}" \
    'BEGIN {exit !(actual == actual && actual <= limit)}'; then
  echo "FAIL: wall-time CV ${wall_cv_percent}% exceeds --max-wall-cv-percent ${MAX_WALL_CV_PERCENT}" >&2
  status=1
fi

# The YAML receipt is the source for the JSON summary.  Check the converter
# before writing the receipt so a missing parser cannot leave a PASS YAML next
# to a missing/failed JSON artifact.
JSON_CONVERTER_AVAILABLE=true
if ! python3 -c 'import yaml' >/dev/null 2>&1; then
  echo "PyYAML is required for the machine-readable JSON summary" >&2
  JSON_CONVERTER_AVAILABLE=false
  status=1
fi

if (( status == 0 )); then
  receipt_status=PASS
else
  receipt_status=FAIL
fi
machine_receipt="${OUTPUT_DIR}/offline_determinism_receipt.yaml"
{
  echo "schema: 2"
  echo "receipt_kind: offline_determinism"
  echo "status: ${receipt_status}"
  echo "script:"
  echo "  path: $(yaml_quote "${SCRIPT_PATH}")"
  echo "  sha256: $(yaml_quote "${SCRIPT_SHA256}")"
  echo "execution_identity_sha256: $(yaml_quote "${EXECUTION_IDENTITY_SHA256}")"
  echo "runs: ${RUNS}"
  echo "runner:"
  echo "  path: $(yaml_quote "${RUNNER_EXECUTABLE}")"
  echo "  sha256: $(yaml_quote "${RUNNER_SHA256}")"
  echo "  setup_path: $(yaml_quote "${SETUP_FILE}")"
  echo "  setup_sha256: $(yaml_quote "${SETUP_SHA256}")"
  echo "inputs:"
  echo "  bag_path: $(yaml_quote "${BAG}")"
  echo "  bag_tree_sha256: $(yaml_quote "${BAG_TREE_SHA256}")"
  echo "  bag_metadata_sha256: $(yaml_quote "${BAG_METADATA_SHA256}")"
  echo "  bag_duration_sec: ${BAG_DURATION_SEC}"
  echo "  params_path: $(yaml_quote "${PARAMS}")"
  echo "  params_sha256: $(yaml_quote "${PARAMS_SHA256}")"
  echo "  optional:"
  if [[ -n "${REFERENCE_TUM}" ]]; then
    echo "    reference_tum_path: $(yaml_quote "${REFERENCE_TUM}")"
    echo "    reference_tum_sha256: $(yaml_quote "${REFERENCE_TUM_SHA256}")"
  else
    echo "    reference_tum_path: null"
    echo "    reference_tum_sha256: null"
  fi
  if [[ -n "${FIXED_LOOP_EDGES_PATH}" ]]; then
    echo "    fixed_loop_edges_path: $(yaml_quote "${FIXED_LOOP_EDGES_PATH}")"
    echo "    fixed_loop_edges_sha256: $(yaml_quote "${FIXED_LOOP_EDGES_SHA256}")"
  else
    echo "    fixed_loop_edges_path: null"
    echo "    fixed_loop_edges_sha256: null"
  fi
  if [[ ${#PARAM_OVERRIDES[@]} -eq 0 ]]; then
    echo "  parameter_overrides: []"
  else
    echo "  parameter_overrides:"
    for override in "${PARAM_OVERRIDES[@]}"; do
      echo "    - $(yaml_quote "${override}")"
    done
  fi
  echo "  parameter_overrides_sha256: $(yaml_quote "${PARAM_OVERRIDES_SHA256}")"
  echo "  save_maps: ${SAVE_MAPS}"
  echo "  require_ape: ${REQUIRE_APE}"
  echo "  ape_interpolate: ${APE_INTERPOLATE}"
  echo "  ape_max_time_diff: ${APE_MAX_TIME_DIFF}"
  echo "environment:"
  echo "  git_revision: $(yaml_quote "${GIT_REVISION}")"
  echo "  git_dirty: ${GIT_DIRTY}"
  echo "  git_status_sha256: $(yaml_quote "${GIT_STATUS_SHA256}")"
  echo "  git_tracked_diff_sha256: $(yaml_quote "${GIT_TRACKED_DIFF_SHA256}")"
  echo "  git_untracked_content_sha256: $(yaml_quote "${GIT_UNTRACKED_CONTENT_SHA256}")"
  echo "  git_worktree_fingerprint: $(yaml_quote "${GIT_WORKTREE_FINGERPRINT}")"
  echo "  ros_distro: $(yaml_quote "${ROS_DISTRO_RECORDED}")"
  echo "  compiler:"
  echo "    command: $(yaml_quote "${COMPILER_COMMAND}")"
  echo "    path: $(yaml_quote "${COMPILER_PATH}")"
  echo "    version: $(yaml_quote "${COMPILER_VERSION}")"
  echo "  cpu:"
  echo "    model: $(yaml_quote "${CPU_MODEL}")"
  echo "    hardware_threads: $(yaml_quote "${HARDWARE_THREADS}")"
  echo "  memory_total_mib: $(yaml_quote "${MEMORY_TOTAL_MIB}")"
  echo "  openmp:"
  echo "    runtime: $(yaml_quote "${OPENMP_RUNTIME}")"
  echo "    dynamic_dependency_fingerprint: $(yaml_quote "${DYNAMIC_DEPENDENCY_FINGERPRINT}")"
  if [[ -n "${DYNAMIC_DEPENDENCIES}" ]]; then
    echo "    dynamic_dependencies:"
    while IFS= read -r dependency_record; do
      echo "      - $(yaml_quote "${dependency_record}")"
    done <<< "${DYNAMIC_DEPENDENCIES}"
  else
    echo "    dynamic_dependencies: []"
  fi
  echo "    OMP_NUM_THREADS: $(yaml_quote "${OMP_NUM_THREADS_RECORDED}")"
  echo "    OMP_DYNAMIC: $(yaml_quote "${OMP_DYNAMIC_RECORDED}")"
  echo "    OMP_PROC_BIND: $(yaml_quote "${OMP_PROC_BIND_RECORDED}")"
  echo "    OMP_PLACES: $(yaml_quote "${OMP_PLACES_RECORDED}")"
  echo "gates:"
  if [[ -n "${MAX_RTF}" ]]; then echo "  max_rtf: ${MAX_RTF}"; else echo "  max_rtf: null"; fi
  if [[ -n "${MAX_PEAK_RSS_MIB}" ]]; then
    echo "  max_peak_rss_mib: ${MAX_PEAK_RSS_MIB}"
  else
    echo "  max_peak_rss_mib: null"
  fi
  if [[ -n "${MAX_WALL_CV_PERCENT}" ]]; then
    echo "  max_wall_cv_percent: ${MAX_WALL_CV_PERCENT}"
  else
    echo "  max_wall_cv_percent: null"
  fi
  echo "  observed_max_rtf: ${max_rtf_observed}"
  echo "  observed_max_peak_rss_mib: ${max_peak_rss_mib_observed}"
  echo "  observed_wall_mean_sec: ${wall_mean_sec}"
  echo "  observed_wall_cv_percent: ${wall_cv_percent}"
  echo "determinism:"
  echo "  loop_edges_identical: ${loop_edges_identical}"
  echo "  optimized_trajectories_identical: ${optimized_trajectories_identical}"
  echo "  registration_receipts_identical: ${registration_receipts_identical}"
  echo "runs_detail:"
  for i in $(seq 1 "${RUNS}"); do
    run_dir="${OUTPUT_DIR}/run${i}"
    run_index=$((i - 1))
    echo "  - run_index: ${i}"
    echo "    complete: true"
    echo "    wall_sec: ${RUN_WALL_SEC[run_index]}"
    echo "    peak_rss_mib: ${RUN_PEAK_RSS_MIB[run_index]}"
    echo "    rtf: ${RUN_RTF[run_index]}"
    echo "    loop_edges_sha256: $(yaml_quote "$(sha256sum "${run_dir}/loop_edges.csv" | awk '{print $1}')")"
    echo "    trajectory_sha256: $(yaml_quote "$(sha256sum "${run_dir}/trajectory_optimized.tum" | awk '{print $1}')")"
    echo "    registration_receipt_sha256: $(yaml_quote "$(sha256sum "${run_dir}/registration_plugin_receipt.yaml" | awk '{print $1}')")"
  done
} > "${machine_receipt}"

machine_summary_json="${OUTPUT_DIR}/offline_determinism_summary.json"
if [[ "${JSON_CONVERTER_AVAILABLE}" == true ]] && ! python3 - "${machine_receipt}" "${machine_summary_json}" <<'PY'
import json
import sys
from pathlib import Path

try:
    import yaml
except ImportError as error:
    print(f"PyYAML is required to write the JSON receipt: {error}", file=sys.stderr)
    raise SystemExit(1)

source = Path(sys.argv[1])
destination = Path(sys.argv[2])
data = yaml.safe_load(source.read_text(encoding="utf-8"))
if not isinstance(data, dict) or data.get("receipt_kind") != "offline_determinism":
    print(f"invalid determinism receipt: {source}", file=sys.stderr)
    raise SystemExit(1)
destination.write_text(json.dumps(data, indent=2, sort_keys=True) + "\n", encoding="utf-8")
PY
then
  echo "failed to generate machine-readable JSON summary" >&2
  status=1
  JSON_CONVERTER_AVAILABLE=false
fi
if [[ "${JSON_CONVERTER_AVAILABLE}" != true ]]; then
  # Keep all machine-readable artifacts present and consistently FAIL even
  # when the optional YAML parser is unavailable or rejects the receipt.
  sed -i 's/^status: PASS$/status: FAIL/' "${machine_receipt}"
  printf '{\n  "receipt_kind": "offline_determinism",\n  "status": "FAIL",\n  "error": "JSON summary conversion failed"\n}\n' \
    > "${machine_summary_json}"
fi
machine_summary_yaml="${OUTPUT_DIR}/offline_determinism_summary.yaml"
cp "${machine_receipt}" "${machine_summary_yaml}"

summary="${OUTPUT_DIR}/offline_determinism_summary.md"
{
  echo "runner_setup: \`${SETUP_FILE}\`"
  echo "runner_executable: \`${RUNNER_EXECUTABLE}\`"
  echo "runner_sha256: \`${RUNNER_SHA256}\`"
  echo "script_sha256: \`${SCRIPT_SHA256}\`"
  echo "execution_identity_sha256: \`${EXECUTION_IDENTITY_SHA256}\`"
  echo "setup_sha256: \`${SETUP_SHA256}\`"
  echo "params_file: \`${PARAMS}\`"
  echo "params_sha256: \`${PARAMS_SHA256}\`"
  echo "bag_tree_sha256: \`${BAG_TREE_SHA256}\`"
  echo "bag_metadata_sha256: \`${BAG_METADATA_SHA256}\`"
  echo "bag_duration_sec: ${BAG_DURATION_SEC}"
  echo "git_revision: \`${GIT_REVISION}\`"
  echo "git_dirty: ${GIT_DIRTY}"
  echo "git_worktree_fingerprint: \`${GIT_WORKTREE_FINGERPRINT}\`"
  echo "ros_distro: \`${ROS_DISTRO_RECORDED}\`"
  echo "compiler: \`${COMPILER_VERSION}\`"
  echo "cpu: \`${CPU_MODEL}\` (${HARDWARE_THREADS} hardware threads)"
  echo "memory_total_mib: ${MEMORY_TOTAL_MIB}"
  echo "openmp_runtime: \`${OPENMP_RUNTIME}\`"
  echo "dynamic_dependency_fingerprint: \`${DYNAMIC_DEPENDENCY_FINGERPRINT}\`"
  echo "openmp_threads: \`${OMP_NUM_THREADS_RECORDED:-default}\`"
  echo "machine_receipt_yaml: \`${machine_receipt}\`"
  echo "machine_summary_yaml: \`${machine_summary_yaml}\`"
  echo "machine_summary_json: \`${machine_summary_json}\`"
  echo "parameter_overrides_sha256: \`${PARAM_OVERRIDES_SHA256}\`"
  echo "require_ape: ${REQUIRE_APE}"
  if [[ -n "${FIXED_LOOP_EDGES_PATH}" ]]; then
    echo "fixed_loop_edges_path: \`${FIXED_LOOP_EDGES_PATH}\`"
    echo "fixed_loop_edges_sha256: \`${FIXED_LOOP_EDGES_SHA256}\`"
  fi
  echo "parameter_overrides:"
  if [[ ${#PARAM_OVERRIDES[@]} -eq 0 ]]; then
    echo "- none"
  else
    for override in "${PARAM_OVERRIDES[@]}"; do
      echo "- \`${override}\`"
    done
  fi
  echo ""
  echo "| run | wall_sec | peak_rss_mib | rtf | ape_rmse | n_loop_edges | loop_edges_md5 | trajectory_md5 | receipt_md5 |"
  echo "| --- | ---: | ---: | ---: | ---: | ---: | --- | --- | --- |"
  for i in $(seq 1 "${RUNS}"); do
    run_dir="${OUTPUT_DIR}/run${i}"
    run_index=$((i - 1))
    n_edges=$(($(wc -l < "${run_dir}/loop_edges.csv") - 1))
    edges_md5=$(md5sum "${run_dir}/loop_edges.csv" | cut -c1-12)
    traj_md5=$(md5sum "${run_dir}/trajectory_optimized.tum" | cut -c1-12)
    receipt_md5=$(md5sum "${run_dir}/registration_plugin_receipt.yaml" | cut -c1-12)
    rmse=$(ape_rmse_for_run "${run_dir}")
    echo "| run${i} | ${RUN_WALL_SEC[run_index]} | ${RUN_PEAK_RSS_MIB[run_index]} | ${RUN_RTF[run_index]} | ${rmse:-n/a} | ${n_edges} | \`${edges_md5}\` | \`${traj_md5}\` | \`${receipt_md5}\` |"
  done
  echo ""
  echo "edge_sets_identical: ${loop_edges_identical}"
  echo "optimized_trajectories_identical: ${optimized_trajectories_identical}"
  echo "registration_receipts_identical: ${registration_receipts_identical}"
  echo "wall_mean_sec: ${wall_mean_sec}"
  echo "wall_cv_percent: ${wall_cv_percent}"
  echo "max_rtf: ${max_rtf_observed}"
  echo "max_peak_rss_mib: ${max_peak_rss_mib_observed}"
  if [[ ${status} -eq 0 ]]; then
    echo "gate_status: PASS"
  else
    echo "gate_status: FAIL"
  fi
} | tee "${summary}"

if [[ ${status} -eq 0 ]]; then
  echo "DETERMINISM_OK: ${RUNS} runs produced byte-identical loop_edges.csv, trajectory_optimized.tum, and registration_plugin_receipt.yaml (${edge_count} edges); receipt=${machine_receipt} yaml=${machine_summary_yaml} json=${machine_summary_json}"
else
  echo "DETERMINISM_FAILED: see ${summary} and ${machine_receipt}"
fi
exit ${status}

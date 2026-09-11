#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat <<'EOF' >&2
Usage:
  bash scripts/run_release_readiness_checks.sh [options]

Options:
  --out-dir <dir>               Output directory for logs and summaries
  --benchmark-root <dir>        Root directory to scan for metrics.json (default: ./output)
  --ape-threshold <m>           Optional APE threshold passed to benchmark_summary.py
  --ape-threshold-reference-kind <kind>
                                Reference kind to gate on (default: ground_truth)
  --release-profile <path>      Release-profile YAML (per-dataset pass/target)
                                Default: scripts/release_profiles.yaml when omitted
  --no-release-profile          Disable the release-profile gate
  --fail-on-profiles            Fail if a blocking profile FAILs or has NO_DATA;
                                evidence is bound to the current Git commit
  --skip-default-ci             Skip scripts/run_default_ci_checks.sh
  --skip-benchmark-summary      Skip benchmark summary generation
  --public-mid360-completion    Run the public MID-360 segment-reset completion gate
  --public-mid360-completion-output-dir <dir>
                                Output directory for the public MID-360 gate
                                (default: <out-dir>/mid360_public_completion_gate)
  --public-mid360-loop-cloud <json>
                                Override public loop-cloud analysis JSON
  --public-mid360-segment-reset-plan <json>
                                Override segment-reset plan JSON
  --public-mid360-start-run-dir <dir>
                                Override start segment RKO run directory
  --public-mid360-end-run-dir <dir>
                                Override end segment RKO run directory
  --public-mid360-segment-map-alignment <json>
                                Override segment map alignment JSON
  --public-mid360-adoption-gate <json>
                                Override public RKO adoption-gate JSON
  --public-mid360-dashboard-html <html>
                                Override segment-reset dashboard HTML
  --public-mid360-min-segment-rko-poses <n>
                                Minimum TUM poses for each reset segment
  --offline-determinism-bag <dir>
                                Run the offline backend determinism hard gate
                                (Phase 2, docs/roadmap/v0.6.md) on this
                                recorded backend-input bag; any byte-level
                                mismatch across runs fails the gate
  --offline-determinism-runs <n>
                                Run count for the determinism gate (default: 3)
  --offline-determinism-params <yaml>
                                Parameter file for the determinism gate
                                (default: lidarslam/param/lidarslam_mid360_rko_graph.yaml)
  --offline-determinism-reference-tum <tum>
                                Optional reference trajectory; adds per-run APE
                                to the determinism report (report only)
  --offline-determinism-map-quality-profile <yaml>
                                Run the map-quality threshold profile on the
                                refined map produced BY the determinism gate
                                itself (v0.7 Phase 3): forwards --save-maps to
                                the determinism runs and then checks
                                run1/map_refined.pcd against this profile
                                (blocking profiles fail the gate on violation)
  --map-quality-pcd <path>[@<profile>]
                                Run the map-quality metrics stage (v0.7,
                                docs/roadmap/v0.7.md) on this map PCD file
                                or pointcloud_map directory. Repeatable.
                                A byte-level mismatch across the 3 runs
                                always fails the gate (determinism
                                enforcement). With @<profile> the metric
                                values are additionally checked against
                                that threshold profile YAML (Phase 3); a
                                blocking profile fails the gate on
                                violation, report_only only reports.
                                Without a profile the values stay
                                report-only
  --map-quality-downsample <m>  Downsample for the map-quality stage
                                (default: 0.1)
  --degeneracy-report <csv>     Run the degeneracy diagnostics report stage
                                (v0.8 Phase 1, docs/roadmap/v0.8.md §5) on a
                                per-scan diagnostics CSV produced by
                                graph_based_slam / graph_slam_offline_runner
                                with degeneracy_diagnostics_csv_path set.
                                Repeatable. Report-only: the stage writes a
                                Markdown/JSON summary and never fails the
                                gate, even on a missing/malformed CSV (same
                                rollout shape as v0.7's report-only stages)
  --frontend-determinism-bag <dir>
                                Run the offline frontend determinism hard gate
                                (Phase 4, docs/roadmap/v0.6.md) on this raw
                                sensor bag via the lockstep scanmatcher
                                runner; any byte-level mismatch fails the gate
  --frontend-determinism-cloud-topic <topic>
                                Cloud topic in the raw bag (required with
                                --frontend-determinism-bag)
  --frontend-determinism-imu-topic <topic>
                                Optional IMU topic in the raw bag
  --frontend-determinism-runs <n>
                                Run count for the frontend gate (default: 3)
  --frontend-determinism-params <yaml>
                                Parameter file for the frontend gate
                                (default: lidarslam/param/lidarslam.yaml)
  --frontend-determinism-max-clouds <n>
                                Cap processed clouds (0 = whole bag)
  --frontend-determinism-reference-tum <tum>
                                Optional reference trajectory; adds per-run APE
                                to the frontend report (report only)
  --dogfood                     Run the Autoware pointcloud-map dogfood flow
  --autoware-core-dir <dir>     autoware_core checkout for dogfood
  --work-dir <dir>              Runtime workspace directory for dogfood
  --viewer-run-dir <dir>        Reuse an existing viewer run directory for dogfood
  --wait-for-offline-completion Wait for full rosbag completion during dogfood
  --auto-exit-secs <sec>        Auto-close RViz after N seconds during dogfood
  --help                        Show this help

This script is intended as a release/readiness gate for the default workflow.
It can run:
  1. local build/test verification
  2. benchmark summary and HTML report generation from existing metrics.json runs
  3. optional public MID-360 segment-reset completion gate
  4. optional offline backend determinism hard gate (byte-identical loop edges
     and optimized trajectory across N runs on a recorded backend-input bag)
  5. optional Autoware map dogfood

When --ape-threshold is provided, the benchmark summary becomes a hard gate and
the script exits non-zero if any selected run is missing APE or exceeds the
threshold. By default this gate is scoped to `ground_truth` runs so
cross-validation artifacts can appear in reports without blocking release.
Hard benchmark gates also exit non-zero when the benchmark root contains no
metrics.json evidence. --skip-benchmark-summary cannot be combined with
--ape-threshold or --fail-on-profiles.

The release-profile gate runs in addition to (or instead of) --ape-threshold:
each profile in the YAML scores its own pass/target threshold against the best
matching run, with optional report_only_until semantics so hard datasets
can soak without blocking release. A blocking profile with no matching run
fails closed. With --fail-on-profiles, blocking evidence must also have clean,
complete provenance for the exact current repository commit. Report-only
profiles remain historical comparisons and are not commit-bound.
EOF
}

fail() {
  echo "error: $*" >&2
  echo "hint: run 'bash scripts/run_release_readiness_checks.sh --help' for valid options." >&2
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  if [[ -z "${value}" || "${value}" == -* ]]; then
    fail "option requires a value: ${option}"
  fi
}

SCRIPT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)
REPO_ROOT=$(cd "${SCRIPT_DIR}/.." && pwd)

OUT_DIR="${REPO_ROOT}/output/release_readiness_$(date +%Y%m%d_%H%M%S)"
BENCHMARK_ROOT="${REPO_ROOT}/output"
APE_THRESHOLD=""
APE_THRESHOLD_REFERENCE_KIND="ground_truth"
RELEASE_PROFILE="${REPO_ROOT}/scripts/release_profiles.yaml"
FAIL_ON_PROFILES=false
RUN_DEFAULT_CI=true
RUN_BENCHMARK_SUMMARY=true
RUN_PUBLIC_MID360_COMPLETION=false
RUN_DOGFOOD=false

PUBLIC_MID360_COMPLETION_OUTPUT_DIR=""
PUBLIC_MID360_LOOP_CLOUD=""
PUBLIC_MID360_SEGMENT_RESET_PLAN=""
PUBLIC_MID360_START_RUN_DIR=""
PUBLIC_MID360_END_RUN_DIR=""
PUBLIC_MID360_SEGMENT_MAP_ALIGNMENT=""
PUBLIC_MID360_ADOPTION_GATE=""
PUBLIC_MID360_DASHBOARD_HTML=""
PUBLIC_MID360_MIN_SEGMENT_RKO_POSES=""

OFFLINE_DETERMINISM_BAG=""
OFFLINE_DETERMINISM_RUNS=""
OFFLINE_DETERMINISM_PARAMS=""
OFFLINE_DETERMINISM_REFERENCE_TUM=""
OFFLINE_DETERMINISM_MAP_QUALITY_PROFILE=""
MAP_QUALITY_PCDS=()
MAP_QUALITY_PROFILES=()
MAP_QUALITY_DOWNSAMPLE=""
DEGENERACY_REPORT_CSVS=()
FRONTEND_DETERMINISM_BAG=""
FRONTEND_DETERMINISM_CLOUD_TOPIC=""
FRONTEND_DETERMINISM_IMU_TOPIC=""
FRONTEND_DETERMINISM_RUNS=""
FRONTEND_DETERMINISM_PARAMS=""
FRONTEND_DETERMINISM_MAX_CLOUDS=""
FRONTEND_DETERMINISM_REFERENCE_TUM=""

AUTOWARE_CORE_DIR=""
WORK_DIR=""
VIEWER_RUN_DIR=""
WAIT_FOR_OFFLINE_COMPLETION=false
AUTO_EXIT_SECS=""

while [[ $# -gt 0 ]]; do
  case "$1" in
    --out-dir)
      require_value "$1" "${2:-}"
      OUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --benchmark-root)
      require_value "$1" "${2:-}"
      BENCHMARK_ROOT=$(realpath -m "$2")
      shift 2
      ;;
    --ape-threshold)
      require_value "$1" "${2:-}"
      APE_THRESHOLD="$2"
      shift 2
      ;;
    --ape-threshold-reference-kind)
      require_value "$1" "${2:-}"
      APE_THRESHOLD_REFERENCE_KIND="$2"
      shift 2
      ;;
    --release-profile)
      require_value "$1" "${2:-}"
      RELEASE_PROFILE=$(realpath -m "$2")
      shift 2
      ;;
    --no-release-profile)
      RELEASE_PROFILE=""
      shift
      ;;
    --fail-on-profiles)
      FAIL_ON_PROFILES=true
      shift
      ;;
    --skip-default-ci)
      RUN_DEFAULT_CI=false
      shift
      ;;
    --skip-benchmark-summary)
      RUN_BENCHMARK_SUMMARY=false
      shift
      ;;
    --public-mid360-completion)
      RUN_PUBLIC_MID360_COMPLETION=true
      shift
      ;;
    --public-mid360-completion-output-dir)
      require_value "$1" "${2:-}"
      PUBLIC_MID360_COMPLETION_OUTPUT_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-loop-cloud)
      require_value "$1" "${2:-}"
      PUBLIC_MID360_LOOP_CLOUD=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-segment-reset-plan)
      require_value "$1" "${2:-}"
      PUBLIC_MID360_SEGMENT_RESET_PLAN=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-start-run-dir)
      require_value "$1" "${2:-}"
      PUBLIC_MID360_START_RUN_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-end-run-dir)
      require_value "$1" "${2:-}"
      PUBLIC_MID360_END_RUN_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-segment-map-alignment)
      require_value "$1" "${2:-}"
      PUBLIC_MID360_SEGMENT_MAP_ALIGNMENT=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-adoption-gate)
      require_value "$1" "${2:-}"
      PUBLIC_MID360_ADOPTION_GATE=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-dashboard-html)
      require_value "$1" "${2:-}"
      PUBLIC_MID360_DASHBOARD_HTML=$(realpath -m "$2")
      shift 2
      ;;
    --public-mid360-min-segment-rko-poses)
      require_value "$1" "${2:-}"
      PUBLIC_MID360_MIN_SEGMENT_RKO_POSES="$2"
      shift 2
      ;;
    --offline-determinism-bag)
      require_value "$1" "${2:-}"
      OFFLINE_DETERMINISM_BAG=$(realpath -m "$2")
      shift 2
      ;;
    --offline-determinism-runs)
      require_value "$1" "${2:-}"
      OFFLINE_DETERMINISM_RUNS="$2"
      shift 2
      ;;
    --offline-determinism-params)
      require_value "$1" "${2:-}"
      OFFLINE_DETERMINISM_PARAMS=$(realpath -m "$2")
      shift 2
      ;;
    --offline-determinism-reference-tum)
      require_value "$1" "${2:-}"
      OFFLINE_DETERMINISM_REFERENCE_TUM=$(realpath -m "$2")
      shift 2
      ;;
    --offline-determinism-map-quality-profile)
      require_value "$1" "${2:-}"
      OFFLINE_DETERMINISM_MAP_QUALITY_PROFILE=$(realpath -m "$2")
      shift 2
      ;;
    --map-quality-pcd)
      require_value "$1" "${2:-}"
      if [[ "$2" == *@* ]]; then
        MAP_QUALITY_PCDS+=("$(realpath -m "${2%@*}")")
        MAP_QUALITY_PROFILES+=("$(realpath -m "${2##*@}")")
      else
        MAP_QUALITY_PCDS+=("$(realpath -m "$2")")
        MAP_QUALITY_PROFILES+=("")
      fi
      shift 2
      ;;
    --map-quality-downsample)
      require_value "$1" "${2:-}"
      MAP_QUALITY_DOWNSAMPLE="$2"
      shift 2
      ;;
    --degeneracy-report)
      require_value "$1" "${2:-}"
      DEGENERACY_REPORT_CSVS+=("$(realpath -m "$2")")
      shift 2
      ;;
    --frontend-determinism-bag)
      require_value "$1" "${2:-}"
      FRONTEND_DETERMINISM_BAG=$(realpath -m "$2")
      shift 2
      ;;
    --frontend-determinism-cloud-topic)
      require_value "$1" "${2:-}"
      FRONTEND_DETERMINISM_CLOUD_TOPIC="$2"
      shift 2
      ;;
    --frontend-determinism-imu-topic)
      require_value "$1" "${2:-}"
      FRONTEND_DETERMINISM_IMU_TOPIC="$2"
      shift 2
      ;;
    --frontend-determinism-runs)
      require_value "$1" "${2:-}"
      FRONTEND_DETERMINISM_RUNS="$2"
      shift 2
      ;;
    --frontend-determinism-params)
      require_value "$1" "${2:-}"
      FRONTEND_DETERMINISM_PARAMS=$(realpath -m "$2")
      shift 2
      ;;
    --frontend-determinism-max-clouds)
      require_value "$1" "${2:-}"
      FRONTEND_DETERMINISM_MAX_CLOUDS="$2"
      shift 2
      ;;
    --frontend-determinism-reference-tum)
      require_value "$1" "${2:-}"
      FRONTEND_DETERMINISM_REFERENCE_TUM=$(realpath -m "$2")
      shift 2
      ;;
    --dogfood)
      RUN_DOGFOOD=true
      shift
      ;;
    --autoware-core-dir)
      require_value "$1" "${2:-}"
      AUTOWARE_CORE_DIR=$(realpath "$2")
      shift 2
      ;;
    --work-dir)
      require_value "$1" "${2:-}"
      WORK_DIR=$(realpath -m "$2")
      shift 2
      ;;
    --viewer-run-dir)
      require_value "$1" "${2:-}"
      VIEWER_RUN_DIR=$(realpath "$2")
      shift 2
      ;;
    --wait-for-offline-completion)
      WAIT_FOR_OFFLINE_COMPLETION=true
      shift
      ;;
    --auto-exit-secs)
      require_value "$1" "${2:-}"
      AUTO_EXIT_SECS="$2"
      shift 2
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      fail "unknown option: $1"
      ;;
  esac
done

if [[ "${RUN_BENCHMARK_SUMMARY}" != "true" \
  && ( -n "${APE_THRESHOLD}" || "${FAIL_ON_PROFILES}" == "true" ) ]]; then
  fail "--skip-benchmark-summary cannot disable a requested benchmark gate"
fi
if [[ "${FAIL_ON_PROFILES}" == "true" && -z "${RELEASE_PROFILE}" ]]; then
  fail "--fail-on-profiles requires an active --release-profile"
fi
if [[ "${FAIL_ON_PROFILES}" == "true" && ! -f "${RELEASE_PROFILE}" ]]; then
  fail "release profile not found: ${RELEASE_PROFILE}"
fi

RELEASE_CANDIDATE_COMMIT=""
if [[ "${FAIL_ON_PROFILES}" == "true" ]]; then
  if ! RELEASE_CANDIDATE_COMMIT="$(
    git -c "safe.directory=${REPO_ROOT}" \
      -C "${REPO_ROOT}" rev-parse --verify HEAD^{commit}
  )"; then
    fail "cannot resolve the release-candidate Git commit from ${REPO_ROOT}"
  fi
  if [[ ! "${RELEASE_CANDIDATE_COMMIT}" =~ ^[0-9a-f]{40}$ ]]; then
    fail "release-candidate Git commit is not an exact 40-character object ID"
  fi
fi

mkdir -p "${OUT_DIR}"

echo "Release readiness output: ${OUT_DIR}"
if [[ -n "${RELEASE_CANDIDATE_COMMIT}" ]]; then
  echo "Release candidate commit: ${RELEASE_CANDIDATE_COMMIT}"
fi

if [[ "${RUN_DEFAULT_CI}" == "true" ]]; then
  echo "==> Running default workflow checks"
  bash "${REPO_ROOT}/scripts/run_default_ci_checks.sh" \
    2>&1 | tee "${OUT_DIR}/default_ci.log"
fi

if [[ "${RUN_BENCHMARK_SUMMARY}" == "true" ]]; then
  METRICS_FOUND="$(find "${BENCHMARK_ROOT}" -name metrics.json -print -quit 2>/dev/null || true)"
  SUMMARY_CMD=(
    python3
    "${REPO_ROOT}/scripts/benchmark_summary.py"
    --root "${BENCHMARK_ROOT}"
    --write-md "${OUT_DIR}/benchmark_summary.md"
    --write-csv "${OUT_DIR}/benchmark_summary.csv"
  )
  if [[ -n "${APE_THRESHOLD}" ]]; then
    SUMMARY_CMD+=(
      --ape-threshold "${APE_THRESHOLD}"
      --ape-threshold-reference-kind "${APE_THRESHOLD_REFERENCE_KIND}"
      --fail-on-ape-threshold
    )
  fi
  if [[ -n "${RELEASE_PROFILE}" && -f "${RELEASE_PROFILE}" ]]; then
    SUMMARY_CMD+=(--release-profile "${RELEASE_PROFILE}")
    if [[ "${FAIL_ON_PROFILES}" == "true" ]]; then
      SUMMARY_CMD+=(
        --fail-on-profiles
        --required-git-commit "${RELEASE_CANDIDATE_COMMIT}"
      )
    fi
  elif [[ -n "${RELEASE_PROFILE}" ]]; then
    echo "warning: release profile not found at ${RELEASE_PROFILE}; continuing without profile gate" >&2
  fi
  if [[ -n "${METRICS_FOUND}" ]]; then
    echo "==> Generating benchmark summary from ${BENCHMARK_ROOT}"
    "${SUMMARY_CMD[@]}" 2>&1 | tee "${OUT_DIR}/benchmark_summary.log"
    echo "==> Generating benchmark HTML report from ${BENCHMARK_ROOT}"
    python3 "${REPO_ROOT}/scripts/generate_html_report.py" \
      --root "${BENCHMARK_ROOT}" \
      --out "${OUT_DIR}/benchmark_report.html" \
      2>&1 | tee "${OUT_DIR}/benchmark_report.log"
  else
    if [[ "${FAIL_ON_PROFILES}" == "true" ]]; then
      echo "==> No metrics.json found under ${BENCHMARK_ROOT}; evaluating every release profile as missing evidence"
      echo "==> No metrics.json found under ${BENCHMARK_ROOT}; skipping benchmark HTML report" \
        | tee "${OUT_DIR}/benchmark_report.log"
      "${SUMMARY_CMD[@]}" 2>&1 | tee "${OUT_DIR}/benchmark_summary.log"
    elif [[ -n "${APE_THRESHOLD}" ]]; then
      echo "error: no metrics.json found under ${BENCHMARK_ROOT}; requested benchmark gate has no evidence" \
        | tee "${OUT_DIR}/benchmark_summary.log" >&2
      exit 2
    fi
    echo "==> No metrics.json found under ${BENCHMARK_ROOT}; benchmark reporting is skipped" \
      | tee "${OUT_DIR}/benchmark_summary.log"
    echo "==> No metrics.json found under ${BENCHMARK_ROOT}; skipping benchmark HTML report" \
      | tee "${OUT_DIR}/benchmark_report.log"
  fi
fi

if [[ "${RUN_PUBLIC_MID360_COMPLETION}" == "true" ]]; then
  if [[ -z "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}" ]]; then
    PUBLIC_MID360_COMPLETION_OUTPUT_DIR="${OUT_DIR}/mid360_public_completion_gate"
  fi
  echo "==> Running public MID-360 completion gate"
  PUBLIC_MID360_CMD=(
    python3
    "${REPO_ROOT}/scripts/run_mid360_robot_public_completion_gate.py"
    --json
    --output-dir "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}"
  )
  if [[ -n "${PUBLIC_MID360_LOOP_CLOUD}" ]]; then
    PUBLIC_MID360_CMD+=(--loop-cloud "${PUBLIC_MID360_LOOP_CLOUD}")
  fi
  if [[ -n "${PUBLIC_MID360_SEGMENT_RESET_PLAN}" ]]; then
    PUBLIC_MID360_CMD+=(--segment-reset-plan "${PUBLIC_MID360_SEGMENT_RESET_PLAN}")
  fi
  if [[ -n "${PUBLIC_MID360_START_RUN_DIR}" ]]; then
    PUBLIC_MID360_CMD+=(--start-run-dir "${PUBLIC_MID360_START_RUN_DIR}")
  fi
  if [[ -n "${PUBLIC_MID360_END_RUN_DIR}" ]]; then
    PUBLIC_MID360_CMD+=(--end-run-dir "${PUBLIC_MID360_END_RUN_DIR}")
  fi
  if [[ -n "${PUBLIC_MID360_SEGMENT_MAP_ALIGNMENT}" ]]; then
    PUBLIC_MID360_CMD+=(--segment-map-alignment "${PUBLIC_MID360_SEGMENT_MAP_ALIGNMENT}")
  fi
  if [[ -n "${PUBLIC_MID360_ADOPTION_GATE}" ]]; then
    PUBLIC_MID360_CMD+=(--adoption-gate "${PUBLIC_MID360_ADOPTION_GATE}")
  fi
  if [[ -n "${PUBLIC_MID360_DASHBOARD_HTML}" ]]; then
    PUBLIC_MID360_CMD+=(--dashboard-html "${PUBLIC_MID360_DASHBOARD_HTML}")
  fi
  if [[ -n "${PUBLIC_MID360_MIN_SEGMENT_RKO_POSES}" ]]; then
    PUBLIC_MID360_CMD+=(--min-segment-rko-poses "${PUBLIC_MID360_MIN_SEGMENT_RKO_POSES}")
  fi
  "${PUBLIC_MID360_CMD[@]}" 2>&1 | tee "${OUT_DIR}/public_mid360_completion_gate.log"
fi

if [[ -n "${OFFLINE_DETERMINISM_BAG}" ]]; then
  echo "==> Running offline backend determinism hard gate"
  OFFLINE_DETERMINISM_CMD=(
    bash
    "${REPO_ROOT}/scripts/run_offline_determinism_check.sh"
    --bag "${OFFLINE_DETERMINISM_BAG}"
    --output-dir "${OUT_DIR}/offline_determinism"
  )
  if [[ -n "${OFFLINE_DETERMINISM_RUNS}" ]]; then
    OFFLINE_DETERMINISM_CMD+=(--runs "${OFFLINE_DETERMINISM_RUNS}")
  fi
  if [[ -n "${OFFLINE_DETERMINISM_PARAMS}" ]]; then
    OFFLINE_DETERMINISM_CMD+=(--params "${OFFLINE_DETERMINISM_PARAMS}")
  fi
  if [[ -n "${OFFLINE_DETERMINISM_REFERENCE_TUM}" ]]; then
    OFFLINE_DETERMINISM_CMD+=(--reference-tum "${OFFLINE_DETERMINISM_REFERENCE_TUM}")
  fi
  if [[ -n "${OFFLINE_DETERMINISM_MAP_QUALITY_PROFILE}" ]]; then
    OFFLINE_DETERMINISM_CMD+=(--save-maps)
  fi
  "${OFFLINE_DETERMINISM_CMD[@]}" 2>&1 | tee "${OUT_DIR}/offline_determinism.log"

  if [[ -n "${OFFLINE_DETERMINISM_MAP_QUALITY_PROFILE}" ]]; then
    echo "==> Checking the gate-produced refined map against the map-quality profile"
    REFINED_MAP="${OUT_DIR}/offline_determinism/run1/map_refined.pcd"
    if [[ ! -f "${REFINED_MAP}" ]]; then
      echo "refined map not found: ${REFINED_MAP}" >&2
      echo "(the determinism stage must run with refine enabled to produce it)" >&2
      exit 1
    fi
    bash "${REPO_ROOT}/scripts/run_map_quality_check.sh" \
      --input "${REFINED_MAP}" \
      --output-dir "${OUT_DIR}/offline_determinism_map_quality" \
      --profile "${OFFLINE_DETERMINISM_MAP_QUALITY_PROFILE}" \
      2>&1 | tee "${OUT_DIR}/offline_determinism_map_quality.log"
  fi
fi

if [[ -n "${FRONTEND_DETERMINISM_BAG}" ]]; then
  echo "==> Running offline frontend determinism hard gate"
  if [[ -z "${FRONTEND_DETERMINISM_CLOUD_TOPIC}" ]]; then
    echo "--frontend-determinism-cloud-topic is required with --frontend-determinism-bag" >&2
    exit 2
  fi
  FRONTEND_DETERMINISM_CMD=(
    bash
    "${REPO_ROOT}/scripts/run_frontend_determinism_check.sh"
    --bag "${FRONTEND_DETERMINISM_BAG}"
    --cloud-topic "${FRONTEND_DETERMINISM_CLOUD_TOPIC}"
    --output-dir "${OUT_DIR}/frontend_determinism"
  )
  if [[ -n "${FRONTEND_DETERMINISM_IMU_TOPIC}" ]]; then
    FRONTEND_DETERMINISM_CMD+=(--imu-topic "${FRONTEND_DETERMINISM_IMU_TOPIC}")
  fi
  if [[ -n "${FRONTEND_DETERMINISM_RUNS}" ]]; then
    FRONTEND_DETERMINISM_CMD+=(--runs "${FRONTEND_DETERMINISM_RUNS}")
  fi
  if [[ -n "${FRONTEND_DETERMINISM_PARAMS}" ]]; then
    FRONTEND_DETERMINISM_CMD+=(--params "${FRONTEND_DETERMINISM_PARAMS}")
  fi
  if [[ -n "${FRONTEND_DETERMINISM_MAX_CLOUDS}" ]]; then
    FRONTEND_DETERMINISM_CMD+=(--max-clouds "${FRONTEND_DETERMINISM_MAX_CLOUDS}")
  fi
  if [[ -n "${FRONTEND_DETERMINISM_REFERENCE_TUM}" ]]; then
    FRONTEND_DETERMINISM_CMD+=(--reference-tum "${FRONTEND_DETERMINISM_REFERENCE_TUM}")
  fi
  "${FRONTEND_DETERMINISM_CMD[@]}" 2>&1 | tee "${OUT_DIR}/frontend_determinism.log"
fi

if [[ "${RUN_DOGFOOD}" == "true" ]]; then
  echo "==> Running Autoware pointcloud-map dogfood"
  DOGFOOD_CMD=(
    bash
    "${REPO_ROOT}/scripts/run_rko_lio_graph_autoware_dogfood.sh"
  )
  if [[ -n "${AUTOWARE_CORE_DIR}" ]]; then
    DOGFOOD_CMD+=(--autoware-core-dir "${AUTOWARE_CORE_DIR}")
  fi
  if [[ -n "${WORK_DIR}" ]]; then
    DOGFOOD_CMD+=(--work-dir "${WORK_DIR}")
  fi
  if [[ -n "${VIEWER_RUN_DIR}" ]]; then
    DOGFOOD_CMD+=(--viewer-run-dir "${VIEWER_RUN_DIR}")
  fi
  if [[ "${WAIT_FOR_OFFLINE_COMPLETION}" == "true" ]]; then
    DOGFOOD_CMD+=(--wait-for-offline-completion)
  fi
  if [[ -n "${AUTO_EXIT_SECS}" ]]; then
    DOGFOOD_CMD+=(--auto-exit-secs "${AUTO_EXIT_SECS}")
  fi
  "${DOGFOOD_CMD[@]}" 2>&1 | tee "${OUT_DIR}/dogfood.log"
fi

if [[ ${#MAP_QUALITY_PCDS[@]} -gt 0 ]]; then
  echo "==> Running map-quality metrics stage (3-run byte identity + optional threshold profiles)"
  MAP_QUALITY_INDEX=0
  for MAP_QUALITY_PCD in "${MAP_QUALITY_PCDS[@]}"; do
    MAP_QUALITY_PROFILE="${MAP_QUALITY_PROFILES[$MAP_QUALITY_INDEX]}"
    MAP_QUALITY_INDEX=$((MAP_QUALITY_INDEX + 1))
    MAP_QUALITY_NAME=$(basename "$(dirname "${MAP_QUALITY_PCD}")")_$(basename "${MAP_QUALITY_PCD%.*}")
    MAP_QUALITY_CMD=(
      bash
      "${REPO_ROOT}/scripts/run_map_quality_check.sh"
      --input "${MAP_QUALITY_PCD}"
      --output-dir "${OUT_DIR}/map_quality/${MAP_QUALITY_INDEX}_${MAP_QUALITY_NAME}"
    )
    if [[ -n "${MAP_QUALITY_DOWNSAMPLE}" ]]; then
      MAP_QUALITY_CMD+=(--downsample "${MAP_QUALITY_DOWNSAMPLE}")
    fi
    if [[ -n "${MAP_QUALITY_PROFILE}" ]]; then
      MAP_QUALITY_CMD+=(--profile "${MAP_QUALITY_PROFILE}")
    fi
    "${MAP_QUALITY_CMD[@]}" 2>&1 | tee -a "${OUT_DIR}/map_quality.log"
  done
fi

if [[ ${#DEGENERACY_REPORT_CSVS[@]} -gt 0 ]]; then
  echo "==> Running degeneracy diagnostics report stage (v0.8 Phase 1, report-only)"
  DEGENERACY_INDEX=0
  for DEGENERACY_CSV in "${DEGENERACY_REPORT_CSVS[@]}"; do
    DEGENERACY_INDEX=$((DEGENERACY_INDEX + 1))
    DEGENERACY_NAME=$(basename "$(dirname "${DEGENERACY_CSV}")")_$(basename "${DEGENERACY_CSV%.*}")
    DEGENERACY_OUT="${OUT_DIR}/degeneracy_report/${DEGENERACY_INDEX}_${DEGENERACY_NAME}"
    mkdir -p "${DEGENERACY_OUT}"
    # Report-only (docs/roadmap/v0.8.md §5 Phase 1): a summarizer failure is
    # logged but never fails the readiness gate.
    if ! python3 "${REPO_ROOT}/scripts/summarize_degeneracy_csv.py" \
      --csv "${DEGENERACY_CSV}" \
      --write-md "${DEGENERACY_OUT}/degeneracy_summary.md" \
      --write-json "${DEGENERACY_OUT}/degeneracy_summary.json" \
      2>&1 | tee -a "${OUT_DIR}/degeneracy_report.log"; then
      echo "warning: degeneracy report stage failed for ${DEGENERACY_CSV} (report-only, not gating)" \
        | tee -a "${OUT_DIR}/degeneracy_report.log"
    fi
  done
fi

echo "==> Release readiness checks completed"
echo "  output_dir: ${OUT_DIR}"
if [[ -f "${OUT_DIR}/benchmark_summary.md" ]]; then
  echo "  benchmark_summary_md: ${OUT_DIR}/benchmark_summary.md"
fi
if [[ -f "${OUT_DIR}/benchmark_summary.csv" ]]; then
  echo "  benchmark_summary_csv: ${OUT_DIR}/benchmark_summary.csv"
fi
if [[ -f "${OUT_DIR}/benchmark_report.html" ]]; then
  echo "  benchmark_report_html: ${OUT_DIR}/benchmark_report.html"
fi
if [[ -n "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}" \
  && -f "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}/mid360_robot_public_completion_gate.json" ]]; then
  echo "  public_mid360_completion_gate_json: ${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}/mid360_robot_public_completion_gate.json"
fi
if [[ -n "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}" \
  && -f "${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}/mid360_robot_public_completion_gate.md" ]]; then
  echo "  public_mid360_completion_gate_md: ${PUBLIC_MID360_COMPLETION_OUTPUT_DIR}/mid360_robot_public_completion_gate.md"
fi
if [[ -f "${OUT_DIR}/offline_determinism/offline_determinism_summary.md" ]]; then
  echo "  offline_determinism_summary_md: ${OUT_DIR}/offline_determinism/offline_determinism_summary.md"
fi
if [[ -d "${OUT_DIR}/degeneracy_report" ]]; then
  echo "  degeneracy_report_dir: ${OUT_DIR}/degeneracy_report (report-only)"
fi

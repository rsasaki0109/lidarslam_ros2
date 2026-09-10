#!/usr/bin/env bash
# Exactly one input-free v17 correction gate.  This gate never mounts a host
# path into the container and never starts a formal replay.
set -Eeuo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd -P)"
IMAGE_TAG="${FAST_LIVO2_V17_IMAGE_TAG:-m6a10-v2c-v17-retryable-abort-correction-20260823t224017z-fast-livo2-benchmark:ros1-pinned}"
BUILD_RECEIPT="${FAST_LIVO2_V17_BUILD_RECEIPT_PATH:-/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_build_correction_20260823T224017Z_agentv17/build_identity.receipt.json}"
GATE_RECEIPT="${FAST_LIVO2_V17_GATE_RECEIPT_PATH:-/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_no_input_correction_20260823T224017Z_agentv17/no_input.receipt.json}"
ROOT_RECEIPT="$(dirname "${GATE_RECEIPT}")"
LOG_PATH="${FAST_LIVO2_V17_GATE_LOG_PATH:-${ROOT_RECEIPT}/no_input.log}"
INSPECT_PATH="${FAST_LIVO2_V17_GATE_INSPECT_PATH:-${ROOT_RECEIPT}/container.inspect.json}"
CONTAINER_NAME="${FAST_LIVO2_V17_CONTAINER_NAME:-m6a10-v17-no-input-correction-20260823t224017z}"
PAYLOAD="/runner/scripts/fast_livo2_m6a10_v17_no_input_container_payload.sh"

PATCH_SHA="c9254497b21846b2de6a028aa5fd60927a80da641c2fb47943c49ff3636ca001"
HEADER_SHA="e45aace02cf153e1eeb00981628e545b8cc5fd279c185a627ce6b3ba18a16778"
MAPPER_SHA="c2a62cf0a6943a8c68084e70585955212bbe944b8788d285e4ca41845492ba94"
SELFTEST_SHA="f81bde7cabe54ebfa702908afa9ef071f2436ce5fee2f50bf67e3c494972bc84"
WRAPPER_SHA="506c6a8c72bca9fa66cf49abaead79eabc5b177fa9b1b30f868383a98a5ab67e"
PAYLOAD_SHA="a8043f751db0638f34e79254133ac5974527c609f7574c616ee219a685cd8ed5"
FEEDER_SHA="6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7"
PROFILE_SHA="1803ba255adfac96a61e1f677be53e84b196ad70bb367a10ebdb9791cb94128b"
BASE_ID="sha256:f103a0f61f7ec9b19cbae02da6c6b8a28f17475dbc3f51fba3206b954ca4ca7a"
PHASE_CONTRACT="m6a10-online-compute-v5-terminal-nonlidar-at-or-after-boundary"
TRANSPORT_CONTRACT="m6a10-v12-callback-ack-transport-outstanding-v1"

sha() { sha256sum "$1" | awk '{print $1}'; }
fail() { echo "FAST-LIVO2 v17 no-input gate failed: $*" >&2; exit 1; }
GATE_SOURCE_SHA="$(sha "${ROOT_DIR}/scripts/run_fast_livo2_m6a10_v17_no_input_gate.sh")"
json_field() { python3 - "$1" "$2" <<'PY'
import json, sys
value = json.loads(open(sys.argv[1], encoding="utf-8").read())
current = value
for part in sys.argv[2].split('.'):
    current = current[part]
print(current)
PY
}

test -z "${BAG_PATH:-}" && test -z "${GROUND_TRUTH_PATH:-}" && test -z "${SCORER_PATH:-}"
test -f "${BUILD_RECEIPT}" && test ! -L "${BUILD_RECEIPT}" || fail 'build receipt missing or symlink'
test -f "${BUILD_RECEIPT}.sha256" && test ! -L "${BUILD_RECEIPT}.sha256" || fail 'build sidecar missing or symlink'
BUILD_SHA="$(sha "${BUILD_RECEIPT}")"
BUILD_SIDE_SHA="$(sha "${BUILD_RECEIPT}.sha256")"
test "$(basename "${BUILD_RECEIPT}") = build_identity.receipt.json"
grep -Fq "${BUILD_SHA}  build_identity.receipt.json" "${BUILD_RECEIPT}.sha256" || fail 'build sidecar mismatch'
test "$(json_field "${BUILD_RECEIPT}" status)" = PASS || fail 'build status'
test "$(json_field "${BUILD_RECEIPT}" build.command_count)" = 1 || fail 'build count'
test "$(json_field "${BUILD_RECEIPT}" safety.input_opened)" = False
test "$(json_field "${BUILD_RECEIPT}" safety.formal_replay_started)" = False
test "$(json_field "${BUILD_RECEIPT}" source.v17_mapper_sha256)" = "${MAPPER_SHA}"
test "$(json_field "${BUILD_RECEIPT}" source.v17_patch_sha256)" = "${PATCH_SHA}"
test "$(json_field "${BUILD_RECEIPT}" source.v17_payload_sha256)" = "${PAYLOAD_SHA}"

IMAGE_ID="$(json_field "${BUILD_RECEIPT}" image.id)"
[[ "${IMAGE_ID}" == sha256:* ]] || fail 'build image id'
test "$(json_field "${BUILD_RECEIPT}" image.base_id)" = "${BASE_ID}"
test ! -e "${ROOT_RECEIPT}" && test ! -L "${ROOT_RECEIPT}" || fail 'gate root already exists'
mkdir -p "${ROOT_RECEIPT}"
for path in "${GATE_RECEIPT}" "${GATE_RECEIPT}.sha256" "${LOG_PATH}" "${INSPECT_PATH}"; do
  test ! -e "${path}" && test ! -L "${path}" || fail "evidence exists: ${path}"
done
docker image inspect "${IMAGE_ID}" >/dev/null || fail 'exact image id unavailable'
docker image inspect "${IMAGE_ID}" --format '{{.Id}}' | grep -Fxq "${IMAGE_ID}" || fail 'image identity drift'

LABELS="$(docker image inspect "${IMAGE_ID}" --format '{{json .Config.Labels}}')"
python3 - "${LABELS}" "${IMAGE_TAG}" "${BASE_ID}" "${PATCH_SHA}" "${HEADER_SHA}" "${MAPPER_SHA}" "${SELFTEST_SHA}" "${WRAPPER_SHA}" "${PAYLOAD_SHA}" "${PROFILE_SHA}" "${FEEDER_SHA}" "${PHASE_CONTRACT}" "${TRANSPORT_CONTRACT}" <<'PY'
import json, sys
labels = json.loads(sys.argv[1])
expected = {
    'benchmark.fast_livo2.m6a10_v15_base_id': sys.argv[3],
    'benchmark.fast_livo2.m6a10_v17_patch_sha256': sys.argv[4],
    'benchmark.fast_livo2.m6a10_v17_header_sha256': sys.argv[5],
    'benchmark.fast_livo2.m6a10_v17_mapper_sha256': sys.argv[6],
    'benchmark.fast_livo2.m6a10_v17_selftest_sha256': sys.argv[7],
    'benchmark.fast_livo2.m6a10_v17_wrapper_sha256': sys.argv[8],
    'benchmark.fast_livo2.m6a10_v17_payload_sha256': sys.argv[9],
    'benchmark.fast_livo2.m6a10_v17_profile_sha256': sys.argv[10],
    'benchmark.fast_livo2.m6a10_v15_feeder_sha256': sys.argv[11],
    'benchmark.fast_livo2.m6a10_phase_contract': sys.argv[12],
    'benchmark.fast_livo2.m6a10_transport_contract': sys.argv[13],
    'benchmark.fast_livo2.m6a10_network_expectation': 'none',
    'benchmark.fast_livo2.m6a10_rootfs_expectation': 'read_only_runtime',
    'benchmark.fast_livo2.m6a10_input_present': 'false',
    'benchmark.fast_livo2.m6a10_ground_truth_present': 'false',
    'benchmark.fast_livo2.m6a10_scorer_present': 'false',
    'benchmark.fast_livo2.m6a10_map_present': 'false',
    'benchmark.fast_livo2.m6a10_formal_replay_forbidden': 'true',
}
if labels.get('benchmark.fast_livo2.m6a10_v15_base_id') != sys.argv[3]:
    raise SystemExit('base label mismatch')
for key, expected_value in expected.items():
    if labels.get(key) != expected_value:
        raise SystemExit(f'label mismatch: {key}')
PY

set +e
docker run --name "${CONTAINER_NAME}" --init --pull=never --network none --read-only \
  --env ROS_MASTER_URI=http://127.0.0.1:11311 \
  --env ROS_IP=127.0.0.1 \
  --env ROS_HOSTNAME=127.0.0.1 \
  --env ROS_HOME=/root/.ros \
  --env ROS_LOG_DIR=/out/ros_logs \
  --tmpfs /tmp:rw,nosuid,nodev \
  --tmpfs /root/.ros:rw,nosuid,nodev \
  --tmpfs /out:rw,nosuid,nodev \
  "${IMAGE_ID}" "${PAYLOAD}" >"${LOG_PATH}" 2>&1
RUN_RC=$?
set -e
docker inspect "${CONTAINER_NAME}" >"${INSPECT_PATH}" 2>/dev/null || true
CONTAINER_EXISTS=1
docker inspect "${CONTAINER_NAME}" >/dev/null 2>&1 || CONTAINER_EXISTS=0

python3 - "${INSPECT_PATH}" "${CONTAINER_NAME}" "${IMAGE_ID}" "${RUN_RC}" "${LOG_PATH}" <<'PY'
import json, sys
from pathlib import Path
inspect_path, name, image, run_rc, log_path = sys.argv[1:]
if not Path(inspect_path).is_file():
    raise SystemExit('container inspect missing')
records = json.loads(Path(inspect_path).read_text(encoding='utf-8'))
if not isinstance(records, list) or len(records) != 1:
    raise SystemExit('inspect shape')
value = records[0]
if value.get('Name') != '/' + name:
    raise SystemExit('container name')
if value.get('Image') != image:
    raise SystemExit('container image')
state = value.get('State') or {}
if state.get('Status') != 'exited' or state.get('OOMKilled') is not False:
    raise SystemExit('container did not naturally exit cleanly')
if state.get('ExitCode') != 0 or int(run_rc) != 0:
    raise SystemExit('payload failure')
host = value.get('HostConfig') or {}
if host.get('NetworkMode') != 'none' or host.get('ReadonlyRootfs') is not True:
    raise SystemExit('runtime isolation')
if host.get('Binds') not in (None, []):
    raise SystemExit('host bind present')
mounts = value.get('Mounts') or []
if mounts:
    raise SystemExit('docker mount present')
tmpfs = host.get('Tmpfs') or {}
destinations = sorted(tmpfs)
if destinations != ['/out', '/root/.ros', '/tmp']:
    raise SystemExit('tmpfs destinations')
if any(value != 'rw,nosuid,nodev' for value in tmpfs.values()):
    raise SystemExit('tmpfs mode')
env = (value.get('Config') or {}).get('Env') or []
for expected in ('ROS_MASTER_URI=http://127.0.0.1:11311', 'ROS_IP=127.0.0.1',
                 'ROS_HOSTNAME=127.0.0.1', 'ROS_HOME=/root/.ros',
                 'ROS_LOG_DIR=/out/ros_logs'):
    if expected not in env:
        raise SystemExit('ROS environment')
if Path(log_path).is_symlink() or not Path(log_path).is_file():
    raise SystemExit('runtime log')
PY

grep -Fq 'M6A10_V17_PRODUCTION_ALL PASS' "${LOG_PATH}"
grep -Fq 'M6A10_V17_NO_INPUT_SCHEMA3_PASS' "${LOG_PATH}"
grep -Fq 'M6A10_V17_NO_INPUT_PASS' "${LOG_PATH}"

test "${CONTAINER_EXISTS}" = 1
STATE="$(docker inspect "${CONTAINER_NAME}" --format '{{.State.Status}}')"
test "${STATE}" = exited || fail 'cleanup attempted before stopped state'
docker rm "${CONTAINER_NAME}" >/dev/null
test -z "$(docker ps -a --format '{{.Names}}' | awk -v n="${CONTAINER_NAME}" '$0 == n {print}')" || fail 'container not removed'

LOG_SHA="$(sha "${LOG_PATH}")"
INSPECT_SHA="$(sha "${INSPECT_PATH}")"
ROOT_RECEIPT="${ROOT_RECEIPT}" GATE_RECEIPT="${GATE_RECEIPT}" GATE_SOURCE_SHA="${GATE_SOURCE_SHA}" LOG_SHA="${LOG_SHA}" \
INSPECT_SHA="${INSPECT_SHA}" BUILD_SHA="${BUILD_SHA}" BUILD_SIDE_SHA="${BUILD_SIDE_SHA}" \
IMAGE_TAG="${IMAGE_TAG}" IMAGE_ID="${IMAGE_ID}" CONTAINER_NAME="${CONTAINER_NAME}" \
python3 - <<'PY'
import hashlib, json, os
from pathlib import Path

target = Path(os.environ['GATE_RECEIPT'])
raw_value = {
    'schema_version': 1,
    'receipt_kind': 'v17_no_input_correction_gate',
    'status': 'PASS',
    'candidate': 'v17-retryable-empty-synchronization-abort',
    'image': {'tag': os.environ['IMAGE_TAG'], 'id': os.environ['IMAGE_ID']},
    'source': {'v17_no_input_gate_sha256': os.environ['GATE_SOURCE_SHA']},
    'build_receipt': {
        'path': '/media/sasaki/aiueo1/benchmarks/m6a10_training_20260824/fast_livo2_v2c_v17_build_correction_20260823T224017Z_agentv17/build_identity.receipt.json',
        'sha256': os.environ['BUILD_SHA'],
        'sidecar_sha256': os.environ['BUILD_SIDE_SHA'],
    },
    'runtime': {
        'container_name': os.environ['CONTAINER_NAME'], 'start_count': 1,
        'network': 'none', 'rootfs': 'read_only', 'host_mount_count': 0,
        'tmpfs_destinations': ['/tmp', '/root/.ros', '/out'],
        'explicit_ros_env': True, 'natural_exit': True, 'exit_code': 0,
        'oom_killed': False, 'cleanup': 'stopped_only_remove_success',
        'log_sha256': os.environ['LOG_SHA'], 'inspect_sha256': os.environ['INSPECT_SHA'],
    },
    'production_selftest': {
        'empty_retry_then_complete': 'PASS', 'strict_after_nonlidar': 'PASS',
        'equal_boundary_nonlidar': 'PASS', 'partial_retry_after_record': 'FAIL_CLOSED',
        'explicit_discard': 'FAIL_CLOSED', 'processing_failure': 'FAIL_CLOSED',
    },
    'feeder': {'schema_version': 3, 'status': 'pass', 'sha256': '6921d159ca4c45bcecfaf9db7fbd6f8a3d92783d100a51ba3ca76abbaf477bb7'},
    'safety': {'input_opened': False, 'ground_truth_content_opened': False,
               'scorer_invoked': False, 'map_saved': False, 'formal_replay_started': False},
}
if target.exists() or target.is_symlink():
    raise SystemExit('receipt overwrite')
raw = (json.dumps(raw_value, sort_keys=True, indent=2) + '\n').encode()
part = target.with_name('.' + target.name + '.part')
if part.exists() or part.is_symlink():
    raise SystemExit('staging overwrite')
fd = os.open(str(part), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
try:
    os.write(fd, raw); os.fsync(fd)
finally:
    os.close(fd)
os.link(str(part), str(target)); os.unlink(str(part)); os.chmod(str(target), 0o444)
digest = hashlib.sha256(raw).hexdigest()
sidecar = Path(str(target) + '.sha256')
if sidecar.exists() or sidecar.is_symlink():
    raise SystemExit('sidecar overwrite')
side_raw = (digest + '  ' + target.name + '\n').encode('ascii')
fd = os.open(str(sidecar), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
try:
    os.write(fd, side_raw); os.fsync(fd)
finally:
    os.close(fd)
os.chmod(str(sidecar), 0o444)
print(str(target), digest)
PY
printf 'v17_no_input_gate_receipt=%s\n' "${GATE_RECEIPT}"

#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)"
BASE_IMAGE="${FAST_LIVO2_V10_BASE_IMAGE:-m6a10-v2c-v9wallrate-20260823t040642-fast-livo2-benchmark:ros1-pinned}"
IMAGE_TAG="${FAST_LIVO2_V10_IMAGE_TAG:-m6a10-v2c-v10-terminal-20260823-fast-livo2-benchmark:ros1-pinned}"
PATCH_PATH="${ROOT_DIR}/docker/patches/fast_livo2.m6a10-v2c-v10-terminal-support-context.patch"
EXPECTED_BASE_ID="sha256:0686d85da9c789e5d0edf6c0c13921ce7fb87730239356537928a7a2e1d070d0"

test -f "${PATCH_PATH}"
docker image inspect "${BASE_IMAGE}" >/dev/null
BASE_ID="$(docker image inspect "${BASE_IMAGE}" --format '{{.Id}}')"
test "${BASE_ID}" = "${EXPECTED_BASE_ID}"
PATCH_SHA256="$(sha256sum "${PATCH_PATH}" | awk '{print $1}')"

docker build --network none --pull=false \
  --build-arg "FAST_LIVO2_V9_BASE_IMAGE=${BASE_IMAGE}" \
  --build-arg "FAST_LIVO2_V9_BASE_ID=${BASE_ID}" \
  --build-arg "FAST_LIVO2_M6A10_V10_PATCH_SHA256=${PATCH_SHA256}" \
  -f "${ROOT_DIR}/docker/fast_livo2_m6a10_v10.Dockerfile" \
  -t "${IMAGE_TAG}" "${ROOT_DIR}"

docker image inspect "${IMAGE_TAG}" --format \
  'v10_image={{.Id}} base={{index .Config.Labels "benchmark.fast_livo2.m6a10_v9_base_id"}} patch={{index .Config.Labels "benchmark.fast_livo2.m6a10_v10_patch_sha256"}}'

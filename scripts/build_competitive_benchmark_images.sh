#!/usr/bin/env bash
set -euo pipefail

# Build only pinned benchmark recipes. This is a provenance preflight, not a
# benchmark runner: it never downloads bags/GT and never changes the identity
# receipt. The ours context contains only its Dockerfile; the image clones the
# public repository at the declared revision and initializes only the
# build-required submodule.
ROOT=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
OURS_REVISION=866f733677e92ecb08d67126e463da99dd140d46
OURS_REPOSITORY=https://github.com/rsasaki0109/lidar_slam_ros2.git
GLIM_REVISION=faa264a1bce1bda406f73457e35511f56cdc2eaa
GLIM_ROS2_REVISION=4a9e7a4cb084967c8525a1be529ad3ba2a118ae7
GLIM_BUILD_WITH_CV_BRIDGE=ON
GLIM_CORE_PATCH_SHA256=f3e7549ee1730df37125f17c4be00b5643a6b9db9eacbc96c9ecb2f682b08867
GLIM_ROS2_PATCH_SHA256=ede64c7a00f409a19138f41c19685f08d58b5c8c8d9d9ffa47b6c7dcb774c0f2
FAST_REVISION=0d2c0346107b75b59934975adec9a6eeeb913c64
RPG_VIKIT_REVISION=6c886c8e5d83997806e00294826d528cea3581dd
SOPHUS_REVISION=a621ff2e56c56c839a6c40418d42c3c254424b5c
FAST_LIVO2_M6A10_PATCH_SHA256=33f30a40ad54db5eea331a87b8b86f32410aac2980e37e4cb6d5027100095297
FAST_LIVO2_M6A10_FEEDER_SHA256=1ea1c9bb9c625795c51c4b8c4b6aa81364370da17f8cac562f46d00af9691831
FAST_LIVO2_M6A10_RUNNER_SHA256=cf1120f79c0cf2c10eab309ec56e6edc93f6045ec2d80c5821263c8af2f2cf30
FAST_LIVO2_M6A10_CONSUMER_CONTRACT=m6a10-v2c-fast-livo2-single-inflight-v2-watchdog-v1
FAST_LIVO2_M6A10_WATCHDOG_CONTRACT=m6a10-fast-livo2-host-watchdog-v2
FAST_LIVO2_CPU_FLAGS_POLICY=portable_x86_64_v1
OURS_CONTEXT=
RKO_LIO_ARCHIVE_SHA256=95783dca0cdac394052fbdb03cf1636cf51ee65a2316fbaef3a67bfe4c88d09f
OURS_PATCH_SHA256=67a7b0f9c0118e51604fd89690f682b40f1c24fcd5b39ed216e04bc47e8ee503
OURS_LAUNCH_SHA256=d45545717f90f6877b5f281fc5623df04b824f7a236d2fe73b91c2dd3714371c

cleanup() {
  if [[ -n "$OURS_CONTEXT" && -d "$OURS_CONTEXT" ]]; then
    rm -rf "$OURS_CONTEXT"
  fi
}
trap cleanup EXIT

usage() {
  cat <<'EOF'
Usage: build_competitive_benchmark_images.sh [--system ours|glim|fast_livo2|all]
                                             [--tag-prefix PREFIX]

Builds pinned recipes with --pull=false. The default is --system all.
The ours build clones the declared public repository revision and initializes
only the build-required gitlink-pinned submodule inside the image.
EOF
}

SYSTEM=all
TAG_PREFIX=
while (($#)); do
  case "$1" in
    --system) SYSTEM=${2:?--system requires a value}; shift 2 ;;
    --tag-prefix) TAG_PREFIX=${2:?--tag-prefix requires a value}; shift 2 ;;
    -h|--help) usage; exit 0 ;;
    *) echo "unknown argument: $1" >&2; usage >&2; exit 2 ;;
  esac
done

case "$SYSTEM" in
  ours|glim|fast_livo2|all) ;;
  *) echo "invalid --system: $SYSTEM" >&2; exit 2 ;;
esac

if [[ "$SYSTEM" == ours || "$SYSTEM" == all ]]; then
  OURS_CONTEXT=$(mktemp -d "${TMPDIR:-/tmp}/lidarslam-ours-context.XXXXXX")
  cp "$ROOT/docker/ours_competitive_benchmark.Dockerfile" \
    "$OURS_CONTEXT/Dockerfile"
  cp "$ROOT/lidarslam/launch/rko_lio_slam.launch.py" \
    "$OURS_CONTEXT/rko_lio_slam.launch.py"
  test "$(sha256sum "$OURS_CONTEXT/rko_lio_slam.launch.py" | awk '{ print $1 }')" = \
    "$OURS_LAUNCH_SHA256"
  git -C "$ROOT/Thirdparty/rko_lio" diff --binary > \
    "$OURS_CONTEXT/rko_lio.m6a10-v2a.patch"
  test "$(sha256sum "$OURS_CONTEXT/rko_lio.m6a10-v2a.patch" | awk '{ print $1 }')" = \
    "$OURS_PATCH_SHA256"
  test "$(git -C "$ROOT/Thirdparty/rko_lio" rev-parse HEAD)" = \
    "622b74778a41f753d47aa5918043755ebcbd4c75"
  git -C "$ROOT/Thirdparty/rko_lio" archive --format=tar \
    --prefix=rko_lio/ HEAD > "$OURS_CONTEXT/rko_lio.tar"
  test "$(sha256sum "$OURS_CONTEXT/rko_lio.tar" | awk '{ print $1 }')" = \
    "$RKO_LIO_ARCHIVE_SHA256"
fi

build() {
  local system=$1 recipe tag
  case "$system" in
    ours)
      recipe=docker/ours_competitive_benchmark.Dockerfile
      tag="${TAG_PREFIX}lidarslam-ours:jazzy" ;;
    glim)
      recipe=docker/glim_cpu_benchmark.Dockerfile
      tag="${TAG_PREFIX}glim-cpu-benchmark:competitive-v1" ;;
    fast_livo2)
      recipe=docker/fast_livo2_benchmark.Dockerfile
      tag="${TAG_PREFIX}fast-livo2-benchmark:ros1-pinned" ;;
  esac
  echo "building ${system}: ${tag} (${recipe})"
  case "$system" in
    ours)
      docker build --pull=false --file "$OURS_CONTEXT/Dockerfile" --tag "$tag" \
        --build-arg "OURS_REPOSITORY=$OURS_REPOSITORY" \
        --build-arg "OURS_REVISION=$OURS_REVISION" \
        --build-arg "RKO_LIO_ARCHIVE_SHA256=$RKO_LIO_ARCHIVE_SHA256" \
        --build-arg "RKO_LIO_PATCH_SHA256=$OURS_PATCH_SHA256" \
        --build-arg "OURS_LAUNCH_SHA256=$OURS_LAUNCH_SHA256" \
        "$OURS_CONTEXT" ;;
    glim)
      docker build --pull=false --file "$ROOT/$recipe" --tag "$tag" \
        --build-arg "GLIM_REVISION=$GLIM_REVISION" \
        --build-arg "GLIM_ROS2_REVISION=$GLIM_ROS2_REVISION" \
        --build-arg "GLIM_BUILD_WITH_CV_BRIDGE=$GLIM_BUILD_WITH_CV_BRIDGE" \
        --build-arg "GLIM_M6A10_CORE_PATCH_SHA256=$GLIM_CORE_PATCH_SHA256" \
        --build-arg "GLIM_M6A10_ROS2_PATCH_SHA256=$GLIM_ROS2_PATCH_SHA256" "$ROOT" ;;
    fast_livo2)
      docker build --pull=false --file "$ROOT/$recipe" --tag "$tag" \
        --build-arg "FAST_LIVO2_REVISION=$FAST_REVISION" \
        --build-arg "RPG_VIKIT_REVISION=$RPG_VIKIT_REVISION" \
        --build-arg "SOPHUS_REVISION=$SOPHUS_REVISION" \
        --build-arg "FAST_LIVO2_M6A10_PATCH_SHA256=$FAST_LIVO2_M6A10_PATCH_SHA256" \
        --build-arg "FAST_LIVO2_M6A10_FEEDER_SHA256=$FAST_LIVO2_M6A10_FEEDER_SHA256" \
        --build-arg "FAST_LIVO2_M6A10_RUNNER_SHA256=$FAST_LIVO2_M6A10_RUNNER_SHA256" \
        --build-arg "FAST_LIVO2_M6A10_CONSUMER_CONTRACT=$FAST_LIVO2_M6A10_CONSUMER_CONTRACT" \
        --build-arg "FAST_LIVO2_M6A10_WATCHDOG_CONTRACT=$FAST_LIVO2_M6A10_WATCHDOG_CONTRACT" \
        --build-arg "FAST_LIVO2_CPU_FLAGS_POLICY=$FAST_LIVO2_CPU_FLAGS_POLICY" "$ROOT" ;;
  esac
}

if [[ "$SYSTEM" == all ]]; then
  build ours
  build glim
  build fast_livo2
else
  build "$SYSTEM"
fi

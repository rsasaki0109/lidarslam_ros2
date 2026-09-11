#!/usr/bin/env bash
# Backward-compatible Docker image entrypoint. The first-map implementation is
# shared with sourced source workspaces in run_first_map_demo.sh.
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
exec "${SCRIPT_DIR}/run_first_map_demo.sh" "$@"

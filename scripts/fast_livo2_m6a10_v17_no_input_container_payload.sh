#!/usr/bin/env bash
# Input-free v17 image selftest payload.  The host gate invokes this fixed
# file directly; it never receives a host mount or an authority document.
set -Eeuo pipefail

OUT_DIR=/out
SELFTEST=/opt/fast_livo_v17/m6a10_terminal_support_context_v17_production_selftest
FIXTURE=/opt/fast_livo_v17/m6a10_v17_consumer_status_pass.json
FEEDER=/runner/scripts/fast_livo2_m6a10_feeder.py

test -x "${SELFTEST}"
test -f "${FIXTURE}" && test ! -L "${FIXTURE}"
test -f "${FEEDER}" && test ! -L "${FEEDER}"
mkdir -p "${OUT_DIR}/v17_selftest"
M6A10_SELFTEST_OUTPUT_DIR="${OUT_DIR}/v17_selftest" "${SELFTEST}"

PYTHONPATH=/runner/scripts python3 - "${FIXTURE}" "${FEEDER}" <<'PY'
import hashlib
import json
import sys
from pathlib import Path

fixture = Path(sys.argv[1])
feeder = Path(sys.argv[2])
value = json.loads(fixture.read_bytes().decode("utf-8"))
if feeder.is_symlink() or not feeder.is_file():
    raise SystemExit("feeder regular-file gate")
from fast_livo2_m6a10_v15_feeder import validate_consumer_status
validated = validate_consumer_status(value)
if validated.get("schema_version") != 3:
    raise SystemExit("schema3 feeder gate")
if value.get("status") != "pass":
    raise SystemExit("feeder status gate")
print("M6A10_V17_NO_INPUT_SCHEMA3_PASS", hashlib.sha256(feeder.read_bytes()).hexdigest())
PY

printf '%s\n' 'M6A10_V17_NO_INPUT_PASS'

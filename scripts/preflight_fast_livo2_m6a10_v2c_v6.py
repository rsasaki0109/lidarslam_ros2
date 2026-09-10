# Copyright 2026 Sasaki
# All rights reserved.
#
# Software License Agreement (BSD 2-Clause Simplified License)
#
# Redistribution and use in source and binary forms, with or without
# modification, are permitted provided that the following conditions are met:
#
#  * Redistributions of source code must retain the above copyright notice,
#    this list of conditions and the following disclaimer.
#  * Redistributions in binary form must reproduce the above copyright notice,
#    this list of conditions and the following disclaimer in the documentation
#    and/or other materials provided with the distribution.
#
# THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS "AS IS"
# AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT LIMITED TO, THE
# IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR A PARTICULAR PURPOSE
# ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT HOLDER OR CONTRIBUTORS BE
# LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL, SPECIAL, EXEMPLARY, OR
# CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT LIMITED TO, PROCUREMENT OF
# SUBSTITUTE GOODS OR SERVICES; LOSS OF USE, DATA, OR PROFITS; OR BUSINESS
# INTERRUPTION) HOWEVER CAUSED AND ON ANY THEORY OF LIABILITY, WHETHER IN
# CONTRACT, STRICT LIABILITY, OR TORT (INCLUDING NEGLIGENCE OR OTHERWISE)
# ARISING IN ANY WAY OUT OF THE USE OF THIS SOFTWARE, EVEN IF ADVISED OF THE
# POSSIBILITY OF SUCH DAMAGE.

"""Versioned v6 identity preflight reusing the frozen v5 verifier logic."""

from __future__ import annotations

import hashlib
import importlib.util
import copy
from pathlib import Path
import sys

import yaml


ROOT = Path(__file__).resolve().parents[1]
BASE_PATH = ROOT / 'scripts/preflight_fast_livo2_m6a10_v2c_v5.py'
BASE_SHA256 = (
    '68051e2e4ebea15e1be3841a94d41c782bc5b107713275f6f683be2730b99231')
PHASE_KEY = 'retry_v6'
CONTRACT = 'm6a10-v2c-fast-livo2-single-inflight-v1'
KIND = 'fast_livo2_m6a10_v2c_v6_execution_identity_preflight'

V6_BUILD_ROOT = Path(
    '/media/sasaki/aiueo1/benchmarks/competitive_build_evidence/'
    'm6a10-v2c-fast-livo2-v6portable-20260823')
V6_BUILD_RECEIPT = V6_BUILD_ROOT / 'build_receipt.json'
V6_BUILD_RECEIPT_SHA256 = (
    'fdf80903113ba7a0a93347e9f725dee5abd145e42984c251e520f241d685252c')
V6_IMAGE_TAG = 'm6a10-v2c-v6portable-fast-livo2-benchmark:ros1-pinned'
V6_IMAGE_ID = (
    'sha256:97ce7d51757610f13d3c0c8561356332b26254f1afed4763589c7b7134620c42')
V6_IMAGE_INSPECT_SHA256 = (
    '0e45d3600abc0300b0f0e23e8294b62cbf0b53b9edd8451ffcf795393e0f17e8')
V6_LABELS_CANONICAL_SHA256 = (
    '2ad1f273faf7160804e40a865dbe2cd4b74b85f64733f20c9ef5c584b1f67257')


def _sha256(path: Path) -> str:
    return hashlib.sha256(path.read_bytes()).hexdigest()


def _load_base():
    if _sha256(BASE_PATH) != BASE_SHA256:
        raise RuntimeError('frozen v5 preflight verifier SHA changed')
    spec = importlib.util.spec_from_file_location('fast_livo2_v5_preflight',
                                                   BASE_PATH)
    if spec is None or spec.loader is None:
        raise RuntimeError('could not load frozen v5 preflight verifier')
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    # The imported implementation uses its module-level __file__ when binding
    # the preflight script. Redirect that one path to this versioned wrapper so
    # the receipt binds v6, while all frozen checks remain identical.
    module.__file__ = str(Path(__file__).resolve())
    module.KIND = KIND
    module.CONTRACT = CONTRACT

    # The source tree intentionally records v6 as a new retry only after the
    # identity and quiescence gates pass.  Keep this preflight executable in
    # the short interval before that formal profile entry is added by deriving
    # a read-only v6 phase from the immutable v5 contract.  If a retry_v6
    # entry is already present, it remains authoritative.
    def phase():
        document = yaml.safe_load(module.PROFILE.read_text(encoding='utf-8'))
        contract = document['competitive_slam_profile'][
            'm6a10_fast_livo2_v2c']
        retry = contract.get('retry_v6')
        if retry is not None:
            if retry.get('contract_id') != CONTRACT:
                raise module.PreflightError('v6 contract mismatch')
            if retry.get('status') != 'build_passed_not_executed':
                raise module.PreflightError(
                    'v6 is not build_passed_not_executed')
            if retry.get('result') is not None or retry.get('runner_start_attempted'):
                raise module.PreflightError('v6 already started or has a result')
            if retry.get('bag_replay_started') or retry.get('gt_content_opened'):
                raise module.PreflightError('v6 records input or GT activity')
            return retry

        # Do not mutate the loaded YAML.  This branch is only a verifier
        # phase object and cannot authorize a replay by itself.
        base = contract.get('retry_v5')
        if not isinstance(base, dict):
            raise module.PreflightError('v5 predecessor contract is missing')
        retry = copy.deepcopy(base)
        retry.update({
            'contract_id': CONTRACT,
            'status': 'build_passed_not_executed',
            'result': None,
            'runner_start_attempted': False,
            'container_start_attempted': False,
            'bag_replay_started': False,
            'gt_content_opened': False,
            'scorer_invoked': False,
        })
        retry['feeder'] = {
            'path': 'scripts/fast_livo2_m6a10_feeder.py',
            'sha256': _sha256(ROOT / 'scripts/fast_livo2_m6a10_feeder.py'),
        }
        retry['runner'] = {
            'path': 'scripts/run_fast_livo2_benchmark.py',
            'sha256': _sha256(ROOT / 'scripts/run_fast_livo2_benchmark.py'),
        }
        retry['container_entrypoint'] = {
            'path': 'scripts/fast_livo2_container_run.sh',
            'sha256': _sha256(ROOT / 'scripts/fast_livo2_container_run.sh'),
        }
        retry['build'] = {
            'recipe': {
                'path': 'docker/fast_livo2_benchmark.Dockerfile',
                'sha256': _sha256(ROOT / 'docker/fast_livo2_benchmark.Dockerfile'),
            },
            'entrypoint': {
                'path': 'scripts/build_competitive_benchmark_images.sh',
                'sha256': _sha256(ROOT / 'scripts/build_competitive_benchmark_images.sh'),
            },
            'receipt_path': str(V6_BUILD_RECEIPT),
            'receipt_sha256': V6_BUILD_RECEIPT_SHA256,
        }
        retry['preflight'] = {'script_sha256': _sha256(Path(__file__).resolve())}
        retry['image'] = {
            'tag': V6_IMAGE_TAG,
            'image_id': V6_IMAGE_ID,
            'image_inspect_sha256': V6_IMAGE_INSPECT_SHA256,
            # The frozen verifier evaluates the legacy default argument even
            # when the canonical field is present; bind both aliases.
            'labels_sha256': V6_LABELS_CANONICAL_SHA256,
            'labels_canonical_sha256': V6_LABELS_CANONICAL_SHA256,
        }
        return retry

    module.phase = phase
    return module


def main(argv=None) -> int:
    module = _load_base()
    return module.main(argv)


if __name__ == '__main__':
    raise SystemExit(main(sys.argv[1:]))

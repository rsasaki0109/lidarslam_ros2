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

"""Synthetic adversarial tests for the README claim publication guard."""

from __future__ import annotations

import hashlib
import importlib.util
import json
from pathlib import Path

import pytest


ROOT = Path(__file__).resolve().parents[2]
SCRIPT = ROOT / 'scripts' / 'verify_competitive_claim_readme.py'
SPEC = importlib.util.spec_from_file_location('competitive_claim_readme_guard', SCRIPT)
MODULE = importlib.util.module_from_spec(SPEC)
assert SPEC.loader is not None
SPEC.loader.exec_module(MODULE)

CONFIG = (ROOT / 'configs/slam_benchmark_profiles/'
          'competitive_claim_readme_publication_v1.json')
SCHEMA = (ROOT / 'configs/slam_benchmark_profiles/'
          'competitive_claim_readme_publication_v1.schema.json')
BEGIN = '<!-- BEGIN GENERATED COMPETITIVE CLAIM PUBLICATION -->'
END = '<!-- END GENERATED COMPETITIVE CLAIM PUBLICATION -->'


def _sha(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _write_json(path: Path, value: object) -> None:
    path.parent.mkdir(parents=True, exist_ok=True)
    path.write_text(json.dumps(value, sort_keys=True, indent=2) + '\n', encoding='utf-8')


def _fixture(tmp_path: Path, *, ready: bool = False,
             marker_body: str = '') -> tuple[Path, Path, Path, bytes, dict]:
    root = tmp_path / 'repo'
    root.mkdir()
    config_dir = root / 'configs' / 'slam_benchmark_profiles'
    config_dir.mkdir(parents=True)
    (root / 'README.md').write_text(
        ROOT.joinpath('README.md').read_text(encoding='utf-8').replace(
            f'{BEGIN}\n{END}', f'{BEGIN}\n{marker_body}{END}'),
        encoding='utf-8')
    (config_dir / SCHEMA.name).write_bytes(SCHEMA.read_bytes())
    config = json.loads(CONFIG.read_text(encoding='utf-8'))
    config['status'] = 'READY' if ready else 'NOT_READY'
    config['publication_spec'] = 'publication-spec.json' if ready else None
    config['sealed_root'] = 'sealed' if ready else None
    config['config_sha256'] = MODULE._sha_bytes(
        MODULE._canonical({key: value for key, value in config.items()
                           if key != 'config_sha256'}))
    config_path = config_dir / CONFIG.name
    _write_json(config_path, config)
    claim = marker_body.encode('utf-8')
    if ready:
        (root / 'publication-spec.json').write_text('{}\n', encoding='utf-8')
        sealed = root / 'sealed'
        sealed.mkdir()
        receipt = {
            'status': 'PASS',
            'claim_eligible': True,
            'receipt_sha256': 'a' * 64,
            'rendered_markdown_sha256': _sha(claim),
        }
        (sealed / 'claim.md').write_bytes(claim)
        _write_json(sealed / 'publication_receipt.json', receipt)
    return root, config_path, root / 'README.md', claim, config


def _fake_publisher(claim: bytes):
    def publish(_spec_path: Path, output_root: Path) -> dict[str, object]:
        output_root.mkdir(parents=True)
        receipt = {
            'status': 'PASS',
            'claim_eligible': True,
            'receipt_sha256': 'a' * 64,
            'rendered_markdown_sha256': _sha(claim),
        }
        (output_root / 'claim.md').write_bytes(claim)
        _write_json(output_root / 'publication_receipt.json', receipt)
        return receipt
    return publish


def test_current_repository_guard_is_pass_not_ready() -> None:
    """The checked-in NOT_READY marker passes without claim eligibility."""
    result = MODULE.verify(ROOT / 'README.md', CONFIG, repo_root=ROOT)
    assert result == {
        'status': 'PASS', 'claim_eligible': False,
        'publication_status': 'NOT_READY',
    }


@pytest.mark.parametrize('text', [
    f'{BEGIN}\n{BEGIN}\n{END}\n',
    f'{BEGIN}\n',
    f'{END}\n{BEGIN}\n{END}\n',
])
def test_duplicate_missing_and_nested_markers_fail(text: str) -> None:
    """Duplicate, missing, and reordered marker pairs fail closed."""
    with pytest.raises(MODULE.ReadmeClaimGuardError):
        MODULE._marker_body(text, {'markers': {'begin': BEGIN, 'end': END}})


def test_not_ready_manual_marker_content_fails(tmp_path: Path) -> None:
    """Manual content cannot enter a NOT_READY marker."""
    root, config, readme, _, _ = _fixture(tmp_path, marker_body='manual claim\n')
    with pytest.raises(MODULE.ReadmeClaimGuardError):
        MODULE.verify(readme, config, repo_root=root)


def test_config_tamper_fails_self_hash(tmp_path: Path) -> None:
    """Changing a guard field without resealing its hash is rejected."""
    root, config, _, _, value = _fixture(tmp_path)
    value['legacy_allowlist_policy'] = 'tampered'
    _write_json(config, value)
    with pytest.raises(MODULE.ReadmeClaimGuardError, match='self-hash'):
        MODULE.verify(root / 'README.md', config, repo_root=root)


def test_resealed_new_legacy_allowlist_entry_fails(tmp_path: Path) -> None:
    """A new legacy exception needs a versioned guard migration."""
    root, config, _, _, value = _fixture(tmp_path)
    value['legacy_allowlist'].append({
        'legacy_id': 'synthetic-extra',
        'heading': '## Synthetic fixture',
        'sha256': '0' * 64,
    })
    value.pop('config_sha256')
    value['config_sha256'] = MODULE._sha_bytes(MODULE._canonical(value))
    _write_json(config, value)
    with pytest.raises(MODULE.ReadmeClaimGuardError, match='versioned guard'):
        MODULE.verify(root / 'README.md', config, repo_root=root)


def test_unallowlisted_comprehensive_text_fails(tmp_path: Path) -> None:
    """Comprehensive superiority language outside the marker is rejected."""
    root, config, readme, _, _ = _fixture(tmp_path)
    readme.write_text(readme.read_text(encoding='utf-8').replace(
        f'{BEGIN}\n{END}',
        f'This is SOTA across all datasets.\n\n{BEGIN}\n{END}'),
        encoding='utf-8')
    with pytest.raises(MODULE.ReadmeClaimGuardError, match='comprehensive'):
        MODULE.verify(readme, config, repo_root=root)


def test_legacy_allowlist_survives_unrelated_paragraph_insertion(tmp_path: Path) -> None:
    """Stable paragraph hashes tolerate unrelated README paragraphs above them."""
    root, config, readme, _, _ = _fixture(tmp_path)
    readme.write_text(readme.read_text(encoding='utf-8').replace(
        'On the same HILTI 2022',
        'This is neutral context.\n\nOn the same HILTI 2022', 1), encoding='utf-8')
    assert MODULE.verify(readme, config, repo_root=root)['status'] == 'PASS'


def test_modified_legacy_paragraph_fails_hash_binding(tmp_path: Path) -> None:
    """Changing an allowlisted scoped comparison paragraph is rejected."""
    root, config, readme, _, _ = _fixture(tmp_path)
    readme.write_text(readme.read_text(encoding='utf-8').replace(
        '34.7% lower median APE', '34.8% lower median APE', 1), encoding='utf-8')
    with pytest.raises(MODULE.ReadmeClaimGuardError, match='exactly one paragraph'):
        MODULE.verify(readme, config, repo_root=root)


def test_new_scoped_superiority_sentence_outside_marker_fails(tmp_path: Path) -> None:
    """New percentage/rival superiority wording must be generated."""
    root, config, readme, _, _ = _fixture(tmp_path)
    readme.write_text(readme.read_text(encoding='utf-8').replace(
        f'{BEGIN}\n{END}',
        f'Our system achieved 20% lower APE than the rival.\n\n{BEGIN}\n{END}'),
        encoding='utf-8')
    with pytest.raises(MODULE.ReadmeClaimGuardError, match='comprehensive'):
        MODULE.verify(readme, config, repo_root=root)


def test_ready_reopens_publisher_and_matches_marker(tmp_path: Path,
                                                    monkeypatch: pytest.MonkeyPatch) -> None:
    """READY mode reopens and byte-matches the publisher output."""
    claim = b'Generated claim\n\nThis is SOTA across all datasets.\n'
    root, config, readme, _, _ = _fixture(
        tmp_path, ready=True, marker_body=claim.decode('utf-8'))
    readme.write_bytes(readme.read_bytes().replace(b'\n', b'\r\n'))
    monkeypatch.setattr(MODULE, 'publish', _fake_publisher(claim))
    result = MODULE.verify(readme, config, repo_root=root)
    assert result['status'] == 'PASS'
    assert result['claim_eligible'] is True
    assert result['rendered_markdown_sha256'] == _sha(claim)


def test_ready_empty_marker_fails_closed(tmp_path: Path) -> None:
    """READY cannot be represented by an empty generated section."""
    root, config, readme, _, _ = _fixture(tmp_path, ready=True)
    with pytest.raises(MODULE.ReadmeClaimGuardError, match='must contain'):
        MODULE.verify(readme, config, repo_root=root)


def test_ready_marker_receipt_mismatch_fails(tmp_path: Path,
                                             monkeypatch: pytest.MonkeyPatch) -> None:
    """A hand-edited READY marker fails after receipt identity checks."""
    claim = b'Generated claim\n'
    root, config, readme, _, _ = _fixture(
        tmp_path, ready=True, marker_body='Tampered claim\n')
    (root / 'sealed' / 'claim.md').write_bytes(claim)
    _write_json(root / 'sealed' / 'publication_receipt.json', {
        'status': 'PASS',
        'claim_eligible': True,
        'receipt_sha256': 'a' * 64,
        'rendered_markdown_sha256': _sha(claim),
    })
    monkeypatch.setattr(MODULE, 'publish', _fake_publisher(claim))
    with pytest.raises(MODULE.ReadmeClaimGuardError, match='README marker'):
        MODULE.verify(readme, config, repo_root=root)


def test_ready_generated_receipt_drift_fails(tmp_path: Path,
                                             monkeypatch: pytest.MonkeyPatch) -> None:
    """A generated receipt that drifts from the sealed receipt is rejected."""
    claim = b'Generated claim\n'
    root, config, readme, _, _ = _fixture(
        tmp_path, ready=True, marker_body=claim.decode('utf-8'))

    def drift(_spec_path: Path, output_root: Path) -> dict[str, object]:
        output_root.mkdir(parents=True)
        receipt = {
            'status': 'PASS',
            'claim_eligible': True,
            'receipt_sha256': 'b' * 64,
            'rendered_markdown_sha256': _sha(claim),
        }
        (output_root / 'claim.md').write_bytes(claim)
        _write_json(output_root / 'publication_receipt.json', receipt)
        return receipt

    monkeypatch.setattr(MODULE, 'publish', drift)
    with pytest.raises(MODULE.ReadmeClaimGuardError, match='receipt'):
        MODULE.verify(readme, config, repo_root=root)


def test_ready_publisher_failure_is_fail_closed(
        tmp_path: Path, monkeypatch: pytest.MonkeyPatch) -> None:
    """A publisher failure cannot produce a README claim."""
    claim = b'Generated claim\n'
    root, config, readme, _, _ = _fixture(
        tmp_path, ready=True, marker_body=claim.decode('utf-8'))

    def fail(_spec_path: Path, _output_root: Path) -> dict[str, object]:
        raise MODULE.PublicationError('synthetic publisher failure')

    monkeypatch.setattr(MODULE, 'publish', fail)
    with pytest.raises(MODULE.ReadmeClaimGuardError, match='publisher failed'):
        MODULE.verify(readme, config, repo_root=root)


def test_traversal_and_symlink_inputs_fail_closed(tmp_path: Path) -> None:
    """Traversal and symlinked guard inputs are not trusted."""
    root, config, _, _, _ = _fixture(tmp_path)
    with pytest.raises(MODULE.ReadmeClaimGuardError):
        MODULE._relative('../outside', 'path')
    (root / 'README-link').symlink_to(root / 'README.md')
    with pytest.raises(MODULE.ReadmeClaimGuardError):
        MODULE._resolve(root, 'README-link', 'readme')
    external = tmp_path / 'external.json'
    external.write_text('{}\n', encoding='utf-8')
    external_config = tmp_path / 'external-config.json'
    external_config.symlink_to(config)
    with pytest.raises(MODULE.ReadmeClaimGuardError):
        MODULE.verify(root / 'README.md', external_config, repo_root=root)

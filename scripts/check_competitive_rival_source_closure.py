#!/usr/bin/env python3
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

"""Audit the immutable upstream source closure of every pinned rival.

This checker is deliberately offline.  It does not clone, download, build, or
run a competitor.  A preregistration records the official commit-addressed
archive and its independently downloaded SHA-256, while this tool verifies the
recorded shape, local recipe/config/patch bytes, and all profile/receipt
bindings.  A missing remote proof is NOT_READY, never an inferred pass.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import json
import os
from pathlib import Path
import re
import stat
import sys
from typing import Any, Iterable, Mapping
from urllib.parse import urlparse

import yaml


try:
    from lidarslam_benchmark_tools import package_root
except ModuleNotFoundError:  # direct ``python scripts/<tool>.py`` execution
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]


ROOT = package_root()
DEFAULT_PROFILE = ROOT / "configs/slam_benchmark_profiles/competitive_slam_v1.yaml"
DEFAULT_RECEIPT = ROOT / (
    "configs/slam_benchmark_profiles/competitive_execution_selection_2026-08.yaml"
)
SCHEMA_VERSION = 1
TREE_HASH_KIND = "relative_path_content_sha256_v1"
SELECTION_HASH_KIND = "canonical_rival_source_closure_selection_sha256_v1"
CLOSURE_IDENTITY_HASH_KIND = "canonical_rival_source_closure_identity_sha256_v1"
SHA256_RE = re.compile(r"^[0-9a-fA-F]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-fA-F]{40}$")
CLOSURE_ID_RE = re.compile(r"^[a-z0-9][a-z0-9._-]{2,127}$")
IMAGE_DIGEST_RE = re.compile(r"^sha256:[0-9a-fA-F]{64}$")
GITHUB_REPOSITORY_RE = re.compile(
    r"^https://github\.com/(?P<owner>[A-Za-z0-9_.-]+)/(?P<repo>[A-Za-z0-9_.-]+?)(?:\.git)?/?$"
)
ARCHIVE_RE = re.compile(
    r"^https://github\.com/(?P<owner>[A-Za-z0-9_.-]+)/(?P<repo>[A-Za-z0-9_.-]+?)/archive/(?P<revision>[0-9a-fA-F]{40})\.tar\.gz$"
)
COMMIT_URL_RE = re.compile(
    r"^https://github\.com/(?P<owner>[A-Za-z0-9_.-]+)/(?P<repo>[A-Za-z0-9_.-]+?)/commit/(?P<revision>[0-9a-fA-F]{40})$"
)
REQUIRED_RECIPE_FIELDS = ("dockerfile", "build_script", "runner", "configs")
RECEIPT_CLOSURE_IDENTITY_FIELDS = (
    "closure_id", "closure_revision", "closure_identity_hash_kind",
    "closure_identity_sha256", "selection_id", "selection_path",
    "selection_sha256",
)


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _iter_tree_entries(path: Path) -> Iterable[tuple[Path, os.stat_result]]:
    """Yield only regular, non-link files and reject hard-link ambiguity."""
    if path.is_symlink():
        raise ValueError(f"tree root is a symlink: {path}")
    if not path.is_dir():
        raise ValueError(f"tree root is not a directory: {path}")
    for directory, directory_names, file_names in os.walk(path, followlinks=False):
        directory_path = Path(directory)
        for name in sorted(directory_names + file_names):
            candidate = directory_path / name
            metadata = candidate.lstat()
            if stat.S_ISLNK(metadata.st_mode):
                raise ValueError(f"symlink in source tree: {candidate}")
            if not stat.S_ISREG(metadata.st_mode):
                raise ValueError(f"non-regular source-tree entry: {candidate}")
            if metadata.st_nlink != 1:
                raise ValueError(f"hard-linked source-tree entry: {candidate}")
            yield candidate, metadata


def sha256_tree(path: Path) -> str:
    """Hash sorted relative POSIX names plus a NUL and each file's bytes."""
    digest = hashlib.sha256()
    entries = sorted(
        _iter_tree_entries(path), key=lambda item: item[0].relative_to(path).as_posix()
    )
    for candidate, _ in entries:
        digest.update(candidate.relative_to(path).as_posix().encode("utf-8"))
        digest.update(b"\0")
        with candidate.open("rb") as stream:
            for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
                digest.update(block)
    return digest.hexdigest()


def canonical_json_sha256(value: Any) -> str:
    encoded = json.dumps(
        value, sort_keys=True, separators=(",", ":"), ensure_ascii=True
    ).encode("utf-8")
    return hashlib.sha256(encoded).hexdigest()


def canonical_rival_source_closure_identity(policy: Mapping[str, Any]) -> str:
    """Hash the current recipe/legal closure, excluding its selection pointer.

    The selection sidecar records this value and is itself bound by the
    profile.  Excluding the pointer avoids a profile/selection hash cycle while
    retaining every source, recipe, legal-policy, and supersession field in
    the closure identity.
    """
    payload = copy.deepcopy(dict(policy))
    payload.pop("active_selection", None)
    return canonical_json_sha256(payload)


def current_rival_source_closure_identity(
        profile: Mapping[str, Any], *, root: Path = ROOT) -> dict[str, Any]:
    """Return and validate the current closure ID/hash used by runners.

    This is intentionally narrower than the claim auditor: legal provenance
    may keep a campaign ineligible, but a runner must never emit an unversioned
    or superseded recipe identity.
    """
    contract = profile.get("competitive_slam_profile", profile)
    policy = (contract.get("evidence_gate_v2", {}).get("rival_source_closure")
              if isinstance(contract, Mapping) and
              isinstance(contract.get("evidence_gate_v2"), Mapping) else None)
    if not isinstance(policy, Mapping):
        raise ValueError("current rival source closure is missing")
    closure_id = policy.get("closure_id")
    revision = policy.get("closure_revision")
    if not isinstance(closure_id, str) or CLOSURE_ID_RE.fullmatch(closure_id) is None:
        raise ValueError("current rival source closure ID is missing or malformed")
    if not isinstance(revision, int) or isinstance(revision, bool) or revision < 1:
        raise ValueError("current rival source closure revision is missing or malformed")
    if policy.get("closure_identity_hash_kind") != CLOSURE_IDENTITY_HASH_KIND:
        raise ValueError("current rival source closure hash kind is invalid")
    active = _mapping(policy.get("active_selection"))
    if active is None or active.get("status") != "CURRENT":
        raise ValueError("current rival source closure selection is not CURRENT")
    selection_path = _relative_path(
        root, active.get("path"), "current rival source closure selection.path", [])
    selection_sha = active.get("sha256")
    if selection_path is None or not _sha(selection_sha):
        raise ValueError("current rival source closure selection path/SHA is malformed")
    if not selection_path.is_file() or selection_path.is_symlink():
        raise ValueError("current rival source closure selection is not a regular file")
    if sha256_file(selection_path).lower() != str(selection_sha).lower():
        raise ValueError("current rival source closure selection SHA differs from bytes")
    try:
        selection = yaml.safe_load(selection_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        raise ValueError("current rival source closure selection is unreadable") from error
    if not isinstance(selection, Mapping):
        raise ValueError("current rival source closure selection must be a mapping")
    if selection.get("selection_id") != active.get("selection_id"):
        raise ValueError("current rival source closure selection ID differs from profile")
    if selection.get("closure_id") != closure_id or selection.get("closure_revision") != revision:
        raise ValueError("current rival source closure selection revision differs from profile")
    expected_identity = canonical_rival_source_closure_identity(policy)
    if selection.get("closure_identity_hash_kind") != CLOSURE_IDENTITY_HASH_KIND or \
            selection.get("closure_identity_sha256") != expected_identity:
        raise ValueError("current rival source closure identity differs from selection")
    return {
        "closure_id": closure_id,
        "closure_revision": revision,
        "closure_identity_hash_kind": CLOSURE_IDENTITY_HASH_KIND,
        "closure_identity_sha256": expected_identity,
        "selection_id": active.get("selection_id"),
        "selection_path": str(active.get("path")),
        "selection_sha256": str(selection_sha).lower(),
    }


def validate_rival_source_closure_receipt_identity(
        receipt: Mapping[str, Any], expected: Mapping[str, Any], *,
        label: str = "execution receipt") -> list[str]:
    """Return errors when a receipt is not bound to the current closure.

    The historical execution-selection receipt is intentionally retained in
    the repository.  It predates the versioned r2 source closure and must not
    be treated as a usable identity merely because its older repository and
    image fields still parse.  Every producer/consumer can use this small
    validator without duplicating the field-level contract.
    """
    errors: list[str] = []
    common = receipt.get("common_identity") if isinstance(receipt, Mapping) else None
    observed = common.get("rival_source_closure") if isinstance(common, Mapping) else None
    if not isinstance(observed, Mapping):
        return [f"{label} common_identity.rival_source_closure is missing"]
    for field in RECEIPT_CLOSURE_IDENTITY_FIELDS:
        if field not in observed:
            errors.append(
                f"{label} rival_source_closure.{field} is missing")
            continue
        if observed.get(field) != expected.get(field):
            errors.append(
                f"{label} rival_source_closure.{field} does not match current closure")
    return errors


def _sha(value: Any) -> bool:
    return isinstance(value, str) and SHA256_RE.fullmatch(value) is not None


def _commit(value: Any) -> bool:
    return isinstance(value, str) and COMMIT_RE.fullmatch(value) is not None


def _mapping(value: Any) -> Mapping[str, Any] | None:
    return value if isinstance(value, Mapping) else None


def _relative_path(root: Path, value: Any, label: str,
                   errors: list[str]) -> Path | None:
    if not isinstance(value, str) or not value:
        errors.append(f"{label} must be a non-empty repository-relative path")
        return None
    candidate = Path(value)
    if candidate.is_absolute() or ".." in candidate.parts:
        errors.append(f"{label} must not be absolute or traverse the repository")
        return None
    resolved = (root / candidate).resolve()
    try:
        resolved.relative_to(root.resolve())
    except ValueError:
        errors.append(f"{label} resolves outside the repository")
        return None
    return resolved


def _hash_local_artifact(root: Path, value: Any, label: str,
                         errors: list[str], incomplete: list[str]) -> bool:
    item = _mapping(value)
    if item is None:
        incomplete.append(f"{label} mapping is missing")
        return False
    path = _relative_path(root, item.get("path"), f"{label}.path", errors)
    expected = item.get("sha256")
    if expected is None:
        incomplete.append(f"{label}.sha256 is missing")
        return False
    if not _sha(expected):
        errors.append(f"{label}.sha256 must be a 64-hex SHA-256")
        return False
    if path is None:
        return False
    try:
        metadata = path.lstat()
    except FileNotFoundError:
        errors.append(f"{label}.path does not exist: {item.get('path')}")
        return False
    if stat.S_ISLNK(metadata.st_mode):
        errors.append(f"{label}.path must not be a symlink")
        return False
    if stat.S_ISREG(metadata.st_mode):
        if metadata.st_nlink != 1:
            errors.append(f"{label}.path must not be hard-linked")
            return False
        actual = sha256_file(path)
    elif stat.S_ISDIR(metadata.st_mode):
        try:
            actual = sha256_tree(path)
        except ValueError as exc:
            errors.append(f"{label}: {exc}")
            return False
    else:
        errors.append(f"{label}.path must be a regular file or directory")
        return False
    if actual.lower() != str(expected).lower():
        errors.append(f"{label}.sha256 does not match current bytes")
        return False
    declared_kind = item.get("hash_kind")
    if stat.S_ISDIR(metadata.st_mode):
        if declared_kind != "canonical_tree_sha256":
            errors.append(f"{label}.hash_kind must be canonical_tree_sha256")
            return False
    elif declared_kind not in (None, "file_sha256"):
        errors.append(f"{label}.hash_kind must be file_sha256")
        return False
    return True


def _repo_parts(url: Any, label: str, errors: list[str]) -> tuple[str, str] | None:
    if not isinstance(url, str) or not url:
        errors.append(f"{label} must be an official HTTPS repository URL")
        return None
    match = GITHUB_REPOSITORY_RE.fullmatch(url)
    if match is None:
        errors.append(f"{label} must be an official primary GitHub repository URL")
        return None
    return match.group("owner").lower(), match.group("repo").lower()


def _check_upstream_source(source: Any, label: str,
                           errors: list[str], incomplete: list[str],
                           not_ready: list[str]) -> bool:
    item = _mapping(source)
    if item is None:
        incomplete.append(f"{label} mapping is missing")
        return False
    valid = True
    status = item.get("status")
    if status not in {"READY", "NOT_READY"}:
        incomplete.append(f"{label}.status must be READY or NOT_READY")
        valid = False
    if status == "NOT_READY":
        valid = False
        reasons = item.get("not_ready_reasons")
        if not isinstance(reasons, list) or not reasons or not all(
                isinstance(reason, str) and reason for reason in reasons):
            incomplete.append(f"{label}.not_ready_reasons is required")
            valid = False
        else:
            not_ready.extend(f"{label}: {reason}" for reason in reasons)

    repository_parts = _repo_parts(item.get("repository_url"),
                                   f"{label}.repository_url", errors)
    revision = item.get("revision")
    if not _commit(revision):
        errors.append(f"{label}.revision must be an exact 40-hex commit")
        valid = False
    if item.get("revision_kind") != "immutable_commit":
        errors.append(
            f"{label}.revision_kind must be immutable_commit; branches/tags/latest are not pins"
        )
        valid = False
    if repository_parts is not None:
        archive = ARCHIVE_RE.fullmatch(str(item.get("archive_url", "")))
        if archive is None:
            errors.append(f"{label}.archive_url must be commit-addressed GitHub tar.gz")
            valid = False
        else:
            archive_parts = (archive.group("owner").lower(), archive.group("repo").lower())
            if archive_parts != repository_parts:
                errors.append(f"{label}.archive_url repository differs from repository_url")
                valid = False
            if not _commit(revision) or archive.group("revision").lower() != str(revision).lower():
                errors.append(f"{label}.archive_url revision does not match source revision")
                valid = False
        citation = item.get("citation") if isinstance(item.get("citation"), Mapping) else {}
        commit_url = COMMIT_URL_RE.fullmatch(str(citation.get("commit_url", "")))
        if commit_url is None:
            errors.append(f"{label}.commit_url must be an official commit citation")
            valid = False
        elif (commit_url.group("owner").lower(), commit_url.group("repo").lower()) != repository_parts or \
                not _commit(revision) or commit_url.group("revision").lower() != str(revision).lower():
            errors.append(f"{label}.commit_url does not cite the exact repository commit")
            valid = False
    if not _sha(item.get("archive_sha256")):
        incomplete.append(f"{label}.archive_sha256 is missing or malformed")
        valid = False
    if not _sha(item.get("source_tree_sha256")):
        incomplete.append(f"{label}.source_tree_sha256 is missing or malformed")
        valid = False
    if item.get("source_tree_hash_kind") != TREE_HASH_KIND:
        errors.append(f"{label}.source_tree_hash_kind must be {TREE_HASH_KIND}")
        valid = False

    submodules = item.get("submodules")
    if not isinstance(submodules, list):
        incomplete.append(f"{label}.submodules must explicitly be a list")
        valid = False
    else:
        seen_submodules: set[str] = set()
        for index, submodule in enumerate(submodules):
            sublabel = f"{label}.submodules[{index}]"
            submap = _mapping(submodule)
            if submap is None:
                errors.append(f"{sublabel} must be a mapping")
                valid = False
                continue
            name = submap.get("name")
            if not isinstance(name, str) or not name or name in seen_submodules:
                errors.append(f"{sublabel}.name must be unique non-empty text")
                valid = False
            else:
                seen_submodules.add(name)
            # Recursive closure entries use the same immutable source proof.
            if not _check_upstream_source(submap, sublabel, errors, incomplete, not_ready):
                valid = False

    archive_artifacts = item.get("archive_artifacts", [])
    if not isinstance(archive_artifacts, list):
        incomplete.append(f"{label}.archive_artifacts must be a list")
        valid = False
        archive_artifacts = []
    archive_roles: set[str] = set()
    archive_paths: set[str] = set()
    for index, artifact in enumerate(archive_artifacts):
        artifact_label = f"{label}.archive_artifacts[{index}]"
        artifact_map = _mapping(artifact)
        if artifact_map is None:
            errors.append(f"{artifact_label} must be a mapping")
            valid = False
            continue
        role = artifact_map.get("role")
        path = artifact_map.get("path")
        if not isinstance(role, str) or not role or role in archive_roles:
            errors.append(f"{artifact_label}.role must be unique non-empty text")
            valid = False
        else:
            archive_roles.add(role)
        if (not isinstance(path, str) or not path or Path(path).is_absolute() or
                ".." in Path(path).parts):
            errors.append(f"{artifact_label}.path must be relative to the upstream archive")
            valid = False
        elif path in archive_paths:
            errors.append(f"{artifact_label}.path must be unique")
            valid = False
        else:
            archive_paths.add(path)
        if artifact_map.get("hash_kind") != "upstream_archive_file_sha256":
            errors.append(f"{artifact_label}.hash_kind must be upstream_archive_file_sha256")
            valid = False
        if not _sha(artifact_map.get("sha256")):
            incomplete.append(f"{artifact_label}.sha256 is missing or malformed")
            valid = False
        artifact_status = artifact_map.get("status")
        if artifact_status not in {"READY", "NOT_READY"}:
            incomplete.append(f"{artifact_label}.status is missing or malformed")
            valid = False
        if artifact_status == "NOT_READY":
            reasons = artifact_map.get("not_ready_reasons")
            if not isinstance(reasons, list) or not reasons:
                incomplete.append(f"{artifact_label}.not_ready_reasons is required")
                valid = False
            else:
                valid = False
                not_ready.extend(f"{artifact_label}: {reason}" for reason in reasons)

    license_data = _mapping(item.get("license"))
    if license_data is None:
        incomplete.append(f"{label}.license is missing")
        valid = False
    else:
        license_status = license_data.get("status")
        legal_statuses = {"READY", "NOT_READY", "NOT_READY_LEGAL_PROVENANCE"}
        if license_status not in legal_statuses:
            incomplete.append(f"{label}.license.status is missing or malformed")
            valid = False

        def check_license_files(files: Any, files_label: str) -> bool:
            files_valid = True
            if not isinstance(files, list):
                incomplete.append(f"{files_label} must be a list")
                return False
            for index, license_file in enumerate(files):
                file_label = f"{files_label}[{index}]"
                file_map = _mapping(license_file)
                if file_map is None:
                    errors.append(f"{file_label} must be a mapping")
                    files_valid = False
                    continue
                path = file_map.get("path")
                if (not isinstance(path, str) or not path or
                        Path(path).is_absolute() or ".." in Path(path).parts):
                    errors.append(f"{file_label}.path must be relative to the upstream tree")
                    files_valid = False
                if not isinstance(file_map.get("identity"), str) or not file_map.get("identity"):
                    incomplete.append(f"{file_label}.identity is missing")
                    files_valid = False
                if not _sha(file_map.get("sha256")):
                    incomplete.append(f"{file_label}.sha256 is missing or malformed")
                    files_valid = False
            return files_valid

        files = license_data.get("files")
        files_valid = check_license_files(files, f"{label}.license.files")
        if license_status == "READY" and (not isinstance(files, list) or not files):
            incomplete.append(f"{label}.license.files must not be empty when READY")
            valid = False
        valid = files_valid and valid

        components = license_data.get("components")
        if not isinstance(components, list) or not components:
            incomplete.append(f"{label}.license.components must be a non-empty list")
            valid = False
            components = []
        component_statuses: list[str] = []
        for index, component in enumerate(components):
            component_label = f"{label}.license.components[{index}]"
            component_map = _mapping(component)
            if component_map is None:
                errors.append(f"{component_label} must be a mapping")
                valid = False
                continue
            name = component_map.get("name")
            if not isinstance(name, str) or not name:
                incomplete.append(f"{component_label}.name is missing")
                valid = False
            component_status = component_map.get("status")
            component_statuses.append(str(component_status))
            if component_status not in {"READY", "NOT_READY_LEGAL_PROVENANCE"}:
                incomplete.append(f"{component_label}.status is missing or malformed")
                valid = False
            component_files = component_map.get("files")
            component_files_valid = check_license_files(
                component_files, f"{component_label}.files")
            if component_status == "READY" and (not isinstance(component_files, list) or not component_files):
                incomplete.append(f"{component_label}.files must not be empty when READY")
                valid = False
            valid = component_files_valid and valid
            declarations = component_map.get("declarations", [])
            if not isinstance(declarations, list):
                incomplete.append(f"{component_label}.declarations must be a list")
                valid = False
                declarations = []
            for declaration_index, declaration in enumerate(declarations):
                declaration_label = f"{component_label}.declarations[{declaration_index}]"
                declaration_map = _mapping(declaration)
                if declaration_map is None:
                    errors.append(f"{declaration_label} must be a mapping")
                    valid = False
                    continue
                declaration_path = declaration_map.get("path")
                if (not isinstance(declaration_path, str) or not declaration_path or
                        Path(declaration_path).is_absolute() or ".." in Path(declaration_path).parts):
                    errors.append(f"{declaration_label}.path must be relative to the upstream tree")
                    valid = False
                if not isinstance(declaration_map.get("identity"), str) or not declaration_map.get("identity"):
                    incomplete.append(f"{declaration_label}.identity is missing")
                    valid = False
                if not _sha(declaration_map.get("sha256")):
                    incomplete.append(f"{declaration_label}.sha256 is missing or malformed")
                    valid = False
            if component_status == "NOT_READY_LEGAL_PROVENANCE":
                valid = False
                reasons = component_map.get("not_ready_reasons")
                if not isinstance(reasons, list) or not reasons:
                    incomplete.append(f"{component_label}.not_ready_reasons is required")
                    valid = False
                else:
                    not_ready.extend(f"{component_label}: {reason}" for reason in reasons)
        if license_status == "READY" and any(
                status != "READY" for status in component_statuses):
            errors.append(f"{label}.license cannot be READY with a non-READY component")
            valid = False
        if license_status in {"NOT_READY", "NOT_READY_LEGAL_PROVENANCE"}:
            valid = False
            reasons = license_data.get("not_ready_reasons")
            if not isinstance(reasons, list) or not reasons:
                incomplete.append(f"{label}.license.not_ready_reasons is required")
                valid = False
            else:
                not_ready.extend(f"{label}.license: {reason}" for reason in reasons)

    citation = _mapping(item.get("citation"))
    if citation is None:
        incomplete.append(f"{label}.citation is missing")
        valid = False
    else:
        if citation.get("primary_upstream") is not True:
            errors.append(f"{label}.citation.primary_upstream must be true")
            valid = False
        if citation.get("compatibility_status") not in {"READY", "NOT_READY"}:
            incomplete.append(f"{label}.citation.compatibility_status is missing")
            valid = False
        elif citation.get("compatibility_status") == "NOT_READY":
            valid = False
        if citation.get("compatibility_note") in (None, ""):
            incomplete.append(f"{label}.citation.compatibility_note is missing")
            valid = False
    if status == "READY" and license_data is not None and license_data.get("status") != "READY":
        errors.append(f"{label} cannot be READY with a non-READY license proof")
        valid = False
    return valid


def _check_recipe(root: Path, recipe: Any, label: str,
                  errors: list[str], incomplete: list[str],
                  not_ready: list[str],
                  sources: Mapping[str, Mapping[str, Any]] | None = None) -> bool:
    item = _mapping(recipe)
    if item is None:
        incomplete.append(f"{label} mapping is missing")
        return False
    valid = True
    for key in REQUIRED_RECIPE_FIELDS:
        if key not in item:
            incomplete.append(f"{label}.{key} is missing")
            valid = False
    for key in ("dockerfile", "build_script", "runner"):
        if key in item:
            valid = _hash_local_artifact(root, item[key], f"{label}.{key}", errors, incomplete) and valid
    if "wrapper" in item:
        valid = _hash_local_artifact(root, item["wrapper"], f"{label}.wrapper", errors, incomplete) and valid
    configs = item.get("configs")
    if not isinstance(configs, list):
        incomplete.append(f"{label}.configs must be a list")
        valid = False
    else:
        for index, config in enumerate(configs):
            config_map = _mapping(config)
            status = config_map.get("status") if config_map else None
            if status == "NOT_READY":
                reasons = config_map.get("not_ready_reasons") if config_map else None
                if not isinstance(reasons, list) or not reasons:
                    incomplete.append(f"{label}.configs[{index}].not_ready_reasons is required")
                    valid = False
                else:
                    not_ready.extend(f"{label}.configs[{index}]: {reason}" for reason in reasons)
                if config_map is None or not isinstance(config_map.get("path"), str) or not config_map.get("path"):
                    incomplete.append(f"{label}.configs[{index}].path is required even when NOT_READY")
                    valid = False
                if config_map is None or not _sha(config_map.get("sha256")):
                    incomplete.append(f"{label}.configs[{index}].sha256 is required even when NOT_READY")
                    valid = False
                continue
            if config_map is not None and config_map.get(
                    "path_kind") == "pinned_upstream_archive_relative_path":
                config_label = f"{label}.configs[{index}]"
                config_path = config_map.get("path")
                if (not isinstance(config_path, str) or not config_path or
                        Path(config_path).is_absolute() or
                        ".." in Path(config_path).parts):
                    errors.append(f"{config_label}.path must be relative to the pinned upstream archive")
                    valid = False
                if config_map.get("hash_kind") != "upstream_archive_file_sha256":
                    errors.append(f"{config_label}.hash_kind must be upstream_archive_file_sha256")
                    valid = False
                if not _sha(config_map.get("sha256")):
                    incomplete.append(f"{config_label}.sha256 is missing or malformed")
                    valid = False
                source_name = config_map.get("source")
                role = config_map.get("role")
                source_map = (sources.get(source_name)
                              if isinstance(sources, Mapping) and
                              isinstance(source_name, str) else None)
                archive_match = None
                if isinstance(source_map, Mapping):
                    archive_match = next(
                        (artifact for artifact in source_map.get("archive_artifacts", [])
                         if isinstance(artifact, Mapping) and
                         artifact.get("path") == config_path and
                         artifact.get("role") == role), None)
                if not isinstance(source_map, Mapping):
                    errors.append(f"{config_label}.source is not a declared upstream source")
                    valid = False
                elif not isinstance(archive_match, Mapping):
                    errors.append(f"{config_label} is not bound to a declared archive artifact")
                    valid = False
                elif (archive_match.get("sha256") != config_map.get("sha256") or
                      archive_match.get("status") != "READY"):
                    errors.append(f"{config_label} archive artifact hash/status does not match")
                    valid = False
                continue
            valid = _hash_local_artifact(root, config, f"{label}.configs[{index}]", errors, incomplete) and valid
    base = _mapping(item.get("base_image"))
    if base is None:
        incomplete.append(f"{label}.base_image is missing")
        valid = False
    else:
        if not isinstance(base.get("name"), str) or not base.get("name"):
            incomplete.append(f"{label}.base_image.name is missing")
            valid = False
        if not isinstance(base.get("digest"), str) or not IMAGE_DIGEST_RE.fullmatch(base.get("digest", "")):
            incomplete.append(f"{label}.base_image.digest is missing or malformed")
            valid = False
    options = item.get("build_options")
    if not isinstance(options, Mapping) or not options:
        incomplete.append(f"{label}.build_options must be a non-empty mapping")
        valid = False
    dockerfile = _mapping(item.get("dockerfile"))
    if dockerfile is not None and base is not None:
        path = _relative_path(root, dockerfile.get("path"), f"{label}.dockerfile.path", errors)
        if path is not None and path.is_file():
            first_from = next((line.strip() for line in path.read_text(encoding="utf-8").splitlines()
                               if line.strip().upper().startswith("FROM ")), None)
            expected = f"FROM {base.get('name')}@{base.get('digest')}"
            if first_from != expected:
                errors.append(f"{label}.dockerfile FROM does not match pinned base image")
                valid = False
    return valid


def _system_recipe(contract: Mapping[str, Any], system: str,
                   receipt: Mapping[str, Any] | None = None) -> Mapping[str, Any] | None:
    # Prefer the current profile/selection binding.  Retained receipts may
    # intentionally contain the superseded r1 recipe bytes.
    systems = contract.get("systems")
    if not isinstance(systems, Mapping) or system not in systems:
        systems = (receipt or {}).get("systems") if isinstance(receipt, Mapping) else None
    if not isinstance(systems, Mapping):
        return None
    record = systems.get(system)
    if not isinstance(record, Mapping):
        return None
    container = record.get("container")
    return container.get("recipe") if isinstance(container, Mapping) and isinstance(container.get("recipe"), Mapping) else None


def _check_profile_bindings(contract: Mapping[str, Any], system: str,
                            rival: Mapping[str, Any], recipe: Mapping[str, Any] | None,
                            errors: list[str], incomplete: list[str],
                            receipt: Mapping[str, Any] | None = None) -> bool:
    valid = True
    profile_rival = contract.get("rivals", {}).get(system) if isinstance(contract.get("rivals"), Mapping) else None
    primary = rival.get("sources", [None])[0] if isinstance(rival.get("sources"), list) and rival.get("sources") else None
    if not isinstance(profile_rival, Mapping) or not isinstance(primary, Mapping):
        incomplete.append(f"profile binding for {system} is missing")
        return False
    if primary.get("repository_url") != profile_rival.get("repository"):
        errors.append(f"{system} primary repository does not match profile.rivals")
        valid = False
    if profile_rival.get("revision_kind") != "immutable_commit":
        errors.append(
            f"profile.rivals.{system}.revision_kind must be immutable_commit"
        )
        valid = False
    if str(primary.get("revision", "")).lower() != str(profile_rival.get("revision", "")).lower():
        errors.append(f"{system} primary revision does not match profile.rivals")
        valid = False
    if recipe is None:
        incomplete.append(f"profile.systems.{system}.container.recipe is missing")
        return False
    declared_recipe = _system_recipe(contract, system, None)
    if not isinstance(declared_recipe, Mapping):
        # r2 binds recipes in the current source-closure selection sidecar.
        # _check_active_selection has already verified that binding.
        declared_recipe = recipe
    if not isinstance(declared_recipe, Mapping):
        incomplete.append(f"profile.systems.{system}.container.recipe is missing")
        valid = False
    else:
        for key in ("dockerfile", "build_script"):
            expected = _mapping(recipe.get(key))
            if declared_recipe is recipe:
                declared_path = expected.get("path") if expected is not None else None
                declared_sha = expected.get("sha256") if expected is not None else None
            else:
                declared_path = (declared_recipe.get("path")
                                 if key == "dockerfile" else
                                 declared_recipe.get("build_entrypoint_path"))
                declared_sha = (declared_recipe.get("sha256")
                                if key == "dockerfile" else
                                declared_recipe.get("build_entrypoint_sha256"))
            if expected is None:
                continue
            if declared_path != expected.get("path") or str(declared_sha).lower() != str(expected.get("sha256")).lower():
                errors.append(f"{system} profile {key} path/SHA does not match source closure")
                valid = False
        expected_runner = _mapping(recipe.get("runner"))
        system_records = contract.get("systems")
        declared_system = (system_records.get(system)
                           if isinstance(system_records, Mapping) else None)
        declared_runner = (_mapping(declared_system.get("runner"))
                           if isinstance(declared_system, Mapping) else None)
        if declared_runner is None and declared_recipe is recipe:
            declared_runner = expected_runner
        if expected_runner is not None and (not isinstance(declared_runner, Mapping) or
                                             declared_runner.get("path") != expected_runner.get("path") or
                                             str(declared_runner.get("sha256")).lower() != str(expected_runner.get("sha256")).lower()):
            errors.append(f"{system} profile runner path/SHA does not match source closure")
            valid = False
    return valid


def _artifact_projection(value: Any) -> dict[str, Any] | None:
    item = _mapping(value)
    if item is None:
        return None
    projection = {
        key: item.get(key)
        for key in ("path", "hash_kind", "sha256", "path_kind", "source", "role")
        if key in item
    }
    return projection


def _recipe_projection(recipe: Mapping[str, Any]) -> dict[str, Any]:
    projection: dict[str, Any] = {}
    for key in ("dockerfile", "build_script", "runner", "wrapper"):
        artifact = _artifact_projection(recipe.get(key))
        if artifact is not None:
            projection[key] = artifact
    projection["configs"] = [
        _artifact_projection(config)
        for config in recipe.get("configs", [])
        if _artifact_projection(config) is not None
    ]
    projection["patches"] = [
        _artifact_projection(patch)
        for patch in recipe.get("patches", [])
        if _artifact_projection(patch) is not None
    ]
    return projection


def _check_legal_policy(policy: Mapping[str, Any], rivals: Mapping[str, Any],
                        errors: list[str], incomplete: list[str]) -> bool:
    legal = _mapping(policy.get("legal_policy"))
    if legal is None:
        incomplete.append("rival_source_closure.legal_policy is missing")
        return False
    valid = True
    expected = {
        "source_fetch_mode": "SOURCE_FETCH_ONLY_NO_REDISTRIBUTION",
        "redistribution_mode": "PROHIBITED_UNPROVEN_LICENSE",
        "image_publication_status": "BLOCKED_PENDING_LICENSE_PROVENANCE",
        "claim_eligibility": "BLOCKED",
    }
    for key, value in expected.items():
        if legal.get(key) != value:
            errors.append(f"rival_source_closure.legal_policy.{key} is not {value}")
            valid = False
    listed = legal.get("nonredistributable_sources")
    if not isinstance(listed, list) or not listed or not all(
            isinstance(item, str) and item for item in listed):
        incomplete.append(
            "rival_source_closure.legal_policy.nonredistributable_sources is missing")
        return False
    nonready_sources: set[str] = set()
    for rival in rivals.values():
        if not isinstance(rival, Mapping):
            continue
        for source in rival.get("sources", []):
            license_data = (source.get("license")
                            if isinstance(source, Mapping) else None)
            if isinstance(source, Mapping) and (
                    not isinstance(license_data, Mapping) or
                    license_data.get("status") != "READY"):
                name = source.get("name")
                if isinstance(name, str):
                    nonready_sources.add(name)
    missing = sorted(nonready_sources - set(listed))
    if missing:
        errors.append(
            "legal policy must prohibit redistribution for every non-ready "
            f"source: {missing}")
        valid = False
    if not isinstance(legal.get("warning"), str) or not legal.get("warning"):
        incomplete.append("rival_source_closure.legal_policy.warning is missing")
        valid = False
    return valid


def _check_active_selection(root: Path, contract: Mapping[str, Any],
                            policy: Mapping[str, Any], rivals: Mapping[str, Any],
                            errors: list[str], incomplete: list[str],
                            not_ready: list[str]) -> bool:
    active = _mapping(policy.get("active_selection"))
    if active is None:
        incomplete.append("rival_source_closure.active_selection is missing")
        return False
    valid = True
    selection_id = active.get("selection_id")
    if not isinstance(selection_id, str) or CLOSURE_ID_RE.fullmatch(selection_id) is None:
        errors.append("rival_source_closure.active_selection.selection_id is malformed")
        valid = False
    if active.get("status") != "CURRENT":
        errors.append("rival_source_closure.active_selection.status must be CURRENT")
        valid = False
    selection_path = _relative_path(
        root, active.get("path"),
        "rival_source_closure.active_selection.path", errors)
    selection_sha = active.get("sha256")
    if not _sha(selection_sha):
        incomplete.append("rival_source_closure.active_selection.sha256 is missing")
        valid = False
    if selection_path is None:
        valid = False
    elif not selection_path.is_file():
        errors.append("rival_source_closure.active_selection.path does not exist")
        valid = False
    elif selection_path.is_symlink() or selection_path.stat().st_nlink != 1:
        errors.append("rival_source_closure.active_selection.path must be a unique regular file")
        valid = False
    elif _sha(selection_sha) and sha256_file(selection_path).lower() != str(selection_sha).lower():
        errors.append("rival_source_closure.active_selection.sha256 does not match bytes")
        valid = False
    if selection_path is None or not selection_path.is_file():
        return valid
    try:
        selection = yaml.safe_load(selection_path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as error:
        errors.append(f"rival_source_closure.active_selection is unreadable: {error}")
        return False
    if not isinstance(selection, Mapping):
        errors.append("rival_source_closure.active_selection must contain a mapping")
        return False
    if selection.get("selection_id") != selection_id:
        errors.append("active selection ID does not match profile")
        valid = False
    if selection.get("status") != "CURRENT":
        errors.append("active selection status must be CURRENT")
        valid = False
    if selection.get("closure_id") != policy.get("closure_id") or \
            selection.get("closure_revision") != policy.get("closure_revision"):
        errors.append("active selection closure revision does not match profile")
        valid = False
    expected_identity = canonical_rival_source_closure_identity(policy)
    if selection.get("closure_identity_hash_kind") != CLOSURE_IDENTITY_HASH_KIND or \
            selection.get("closure_identity_sha256") != expected_identity:
        errors.append("active selection closure identity hash does not match profile")
        valid = False
    bindings = selection.get("current_recipe_bindings")
    if not isinstance(bindings, Mapping):
        incomplete.append("active selection current_recipe_bindings is missing")
        valid = False
    else:
        for system in sorted(str(name) for name in rivals):
            expected_recipe = rivals.get(system, {}).get("recipe", {})
            observed = bindings.get(system)
            if not isinstance(expected_recipe, Mapping) or not isinstance(observed, Mapping):
                errors.append(f"active selection recipe binding is missing for {system}")
                valid = False
                continue
            expected_projection = _recipe_projection(expected_recipe)
            if "patches" not in expected_recipe:
                expected_projection["patches"] = [
                    _artifact_projection(patch)
                    for patch in rivals.get(system, {}).get("local_patches", [])
                    if _artifact_projection(patch) is not None
                ]
            if expected_projection != _recipe_projection(observed):
                errors.append(f"active selection recipe binding drifts for {system}")
                valid = False
    supersedes = active.get("supersedes")
    if not isinstance(supersedes, list) or not supersedes:
        incomplete.append("rival_source_closure.active_selection.supersedes is missing")
        valid = False
    else:
        for index, item in enumerate(supersedes):
            record = _mapping(item)
            label = f"rival_source_closure.active_selection.supersedes[{index}]"
            if record is None:
                errors.append(f"{label} must be a mapping")
                valid = False
                continue
            if record.get("status") != "SUPERSEDED_RECIPE_REVISION":
                errors.append(f"{label}.status must be SUPERSEDED_RECIPE_REVISION")
                valid = False
            if record.get("claim_eligible") is not False:
                errors.append(f"{label}.claim_eligible must be false")
                valid = False
            if _relative_path(root, record.get("path"), f"{label}.path", errors) is None:
                valid = False
            if not _sha(record.get("sha256")):
                incomplete.append(f"{label}.sha256 is missing or malformed")
                valid = False
            if not isinstance(record.get("reason"), str) or not record.get("reason"):
                incomplete.append(f"{label}.reason is missing")
                valid = False
    return valid


def verify_rival_source_closure(profile: Mapping[str, Any], *, root: Path = ROOT,
                                receipt: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Return a fail-closed source-closure audit without mutating inputs."""
    errors: list[str] = []
    incomplete: list[str] = []
    not_ready: list[str] = []
    contract = profile.get("competitive_slam_profile", profile)
    if not isinstance(contract, Mapping):
        return {
            "schema_version": SCHEMA_VERSION, "status": "INVALID", "pass": False,
            "errors": ["competitive profile must be a mapping"],
            "incomplete": [], "not_ready": [], "checks": {},
        }
    policy = contract.get("evidence_gate_v2", {}).get("rival_source_closure") if isinstance(contract.get("evidence_gate_v2"), Mapping) else None
    if not isinstance(policy, Mapping):
        return {
            "schema_version": SCHEMA_VERSION, "status": "INCOMPLETE", "pass": False,
            "errors": [], "incomplete": ["profile.evidence_gate_v2.rival_source_closure is missing"],
            "not_ready": [], "checks": {},
        }
    if policy.get("schema_version") != SCHEMA_VERSION:
        errors.append("rival_source_closure.schema_version must be 1")
    if policy.get("required") is not True:
        errors.append("rival_source_closure.required must be true")
    closure_id = policy.get("closure_id")
    if not isinstance(closure_id, str) or CLOSURE_ID_RE.fullmatch(closure_id) is None:
        incomplete.append("rival_source_closure.closure_id is missing or malformed")
    if (not isinstance(policy.get("closure_revision"), int) or
            isinstance(policy.get("closure_revision"), bool) or
            policy.get("closure_revision") < 1):
        incomplete.append("rival_source_closure.closure_revision is missing or malformed")
    if policy.get("closure_identity_hash_kind") != CLOSURE_IDENTITY_HASH_KIND:
        errors.append(
            "rival_source_closure.closure_identity_hash_kind must be "
            f"{CLOSURE_IDENTITY_HASH_KIND}")
    declared_status = policy.get("status")
    if declared_status not in {"READY", "NOT_READY", "INVALID"}:
        incomplete.append("rival_source_closure.status is missing or malformed")
    elif declared_status == "NOT_READY":
        declared_reasons = policy.get("status_reason")
        if not isinstance(declared_reasons, list) or not declared_reasons:
            incomplete.append("rival_source_closure.status_reason is required for NOT_READY")
        else:
            not_ready.extend(
                "declared policy: " + str(reason) for reason in declared_reasons
            )
    required = policy.get("required_systems")
    if not isinstance(required, list) or not required:
        incomplete.append("rival_source_closure.required_systems is missing")
        required = []
    rivals = policy.get("rivals")
    if not isinstance(rivals, Mapping):
        incomplete.append("rival_source_closure.rivals is missing")
        rivals = {}
    legal_valid = _check_legal_policy(policy, rivals, errors, incomplete)
    active_selection_valid = _check_active_selection(
        root, contract, policy, rivals, errors, incomplete, not_ready)
    expected_rivals = sorted(str(system) for system in required if str(system) != "ours")
    observed_rivals = sorted(str(system) for system in rivals)
    if observed_rivals != expected_rivals:
        errors.append(f"source closure rivals must exactly cover {expected_rivals}; observed {observed_rivals}")
    tracks = contract.get("tracks")
    track_rivals = sorted({track.get("rival") for track in tracks.values() if isinstance(track, Mapping) and isinstance(track.get("rival"), str)}
                          if isinstance(tracks, Mapping) else set())
    missing_tracks = sorted(set(expected_rivals) - set(track_rivals))
    if missing_tracks:
        errors.append(f"required rivals omitted from every declared track: {missing_tracks}")
    checks: dict[str, Any] = {
        "legal_policy": {"pass": legal_valid},
        "active_selection": {"pass": active_selection_valid},
        "required_rivals": {"pass": observed_rivals == expected_rivals,
                             "required": expected_rivals, "observed": observed_rivals},
        "track_coverage": {"pass": not missing_tracks,
                            "missing_rivals": missing_tracks, "track_rivals": track_rivals},
    }
    source_results: dict[str, Any] = {}
    for system in expected_rivals:
        rival = rivals.get(system)
        if not isinstance(rival, Mapping):
            incomplete.append(f"rival_source_closure.rivals.{system} is missing")
            source_results[system] = {"pass": False, "status": "INCOMPLETE"}
            continue
        sources = rival.get("sources")
        if not isinstance(sources, list) or not sources:
            incomplete.append(f"rival_source_closure.rivals.{system}.sources is missing")
            source_results[system] = {"pass": False, "status": "INCOMPLETE"}
            continue
        source_ids: set[str] = set()
        rival_status = rival.get("status")
        rival_valid = rival_status == "READY"
        if rival_status == "NOT_READY":
            reasons = rival.get("not_ready_reasons")
            if not isinstance(reasons, list) or not reasons:
                incomplete.append(f"rival_source_closure.rivals.{system}.not_ready_reasons is required")
                rival_valid = False
            else:
                not_ready.extend(f"rival_source_closure.rivals.{system}: {reason}" for reason in reasons)
        elif rival_status != "READY":
            incomplete.append(f"rival_source_closure.rivals.{system}.status must be READY or NOT_READY")
            rival_valid = False
        for index, source in enumerate(sources):
            label = f"rival_source_closure.rivals.{system}.sources[{index}]"
            if isinstance(source, Mapping):
                source_id = source.get("name")
                if not isinstance(source_id, str) or not source_id or source_id in source_ids:
                    errors.append(f"{label}.name must be unique non-empty text")
                    rival_valid = False
                else:
                    source_ids.add(source_id)
            if not _check_upstream_source(source, label, errors, incomplete, not_ready):
                rival_valid = False
        recipe = rival.get("recipe")
        source_by_name = {
            str(source.get("name")): source
            for source in sources if isinstance(source, Mapping) and
            isinstance(source.get("name"), str)
        }
        recipe_valid = _check_recipe(root, recipe,
                                     f"rival_source_closure.rivals.{system}.recipe",
                                     errors, incomplete, not_ready,
                                     source_by_name)
        binding_valid = _check_profile_bindings(
            contract, system, rival,
            recipe if isinstance(recipe, Mapping) else None,
            errors, incomplete, receipt)
        rival_valid = rival_valid and recipe_valid and binding_valid and not missing_tracks
        if isinstance(rival.get("local_patches"), list):
            orders = []
            for index, patch in enumerate(rival.get("local_patches", [])):
                patch_map = _mapping(patch)
                if patch_map is None:
                    errors.append(f"rival_source_closure.rivals.{system}.local_patches[{index}] must be a mapping")
                    rival_valid = False
                    continue
                order = patch_map.get("order")
                orders.append(order)
                if order != index + 1:
                    errors.append(f"{system}.local_patches order must be contiguous and deterministic")
                    rival_valid = False
                if not _hash_local_artifact(root, patch_map, f"{system}.local_patches[{index}]", errors, incomplete):
                    rival_valid = False
            if orders != list(range(1, len(orders) + 1)):
                rival_valid = False
        else:
            incomplete.append(f"rival_source_closure.rivals.{system}.local_patches is missing")
            rival_valid = False
        source_results[system] = {
            "pass": rival_valid,
            "status": "PASS" if rival_valid else "FAIL_CLOSED",
            "source_count": len(sources),
            "source_names": sorted(source_ids),
        }
    checks["rivals"] = source_results

    if receipt is not None:
        receipt_common = receipt.get("common_identity") if isinstance(receipt, Mapping) else None
        receipt_closure = (receipt_common.get("rival_source_closure")
                           if isinstance(receipt_common, Mapping) else None)
        closure_hash = canonical_json_sha256(policy)
        try:
            expected_receipt_identity = current_rival_source_closure_identity(
                profile, root=root)
        except (OSError, ValueError, TypeError, UnicodeError, yaml.YAMLError):
            # Keep the detailed closure diagnostics below useful for a
            # malformed synthetic profile, while still requiring the core
            # version/hash fields whenever a receipt is supplied.
            active = _mapping(policy.get("active_selection"))
            expected_receipt_identity = {
                "closure_id": policy.get("closure_id"),
                "closure_revision": policy.get("closure_revision"),
                "closure_identity_hash_kind": policy.get(
                    "closure_identity_hash_kind"),
                "closure_identity_sha256": canonical_rival_source_closure_identity(
                    policy),
                "selection_id": active.get("selection_id") if active else None,
                "selection_path": active.get("path") if active else None,
                "selection_sha256": active.get("sha256") if active else None,
            }
        if not isinstance(receipt_closure, Mapping):
            errors.append("execution receipt common_identity.rival_source_closure is missing")
            receipt_is_current = False
        else:
            identity_errors = validate_rival_source_closure_receipt_identity(
                receipt, expected_receipt_identity)
            if identity_errors:
                errors.extend(identity_errors)
                errors.append(
                    "execution receipt is SUPERSEDED_RECIPE_REVISION or has a "
                    "missing/mismatched closure identity")
                receipt_is_current = False
            else:
                receipt_is_current = True
            schema_path = _relative_path(
                root, receipt_closure.get("schema_path"),
                "execution receipt rival_source_closure.schema_path", errors)
            schema_sha = receipt_closure.get("schema_sha256")
            if schema_path is None or not _sha(schema_sha):
                incomplete.append("execution receipt rival_source_closure schema path/SHA is missing")
            elif (receipt_is_current and
                  (not schema_path.is_file() or
                   sha256_file(schema_path).lower() != schema_sha.lower())):
                errors.append("execution receipt rival_source_closure schema SHA does not match")
            if (receipt_is_current and
                    receipt_closure.get("profile_section_hash_kind") !=
                    "canonical_rival_source_closure_sha256_v1"):
                errors.append("execution receipt rival_source_closure hash kind is invalid")
            if (receipt_is_current and
                    receipt_closure.get("profile_section_sha256") != closure_hash):
                errors.append("execution receipt rival_source_closure profile section SHA does not match profile")
            if receipt_is_current and receipt_closure.get("status") != policy.get("status"):
                errors.append("execution receipt rival_source_closure status does not match profile")
        receipt_systems = receipt.get("systems") if isinstance(receipt, Mapping) else None
        receipt_results: dict[str, Any] = {}
        if not isinstance(receipt_systems, Mapping):
            incomplete.append("execution receipt systems mapping is missing")
        else:
            for system in expected_rivals:
                expected_rival = rivals.get(system)
                receipt_item = receipt_systems.get(system)
                receipt_ok = isinstance(receipt_item, Mapping)
                if not receipt_ok:
                    incomplete.append(f"execution receipt systems.{system} is missing")
                elif isinstance(expected_rival, Mapping):
                    primary = (expected_rival.get("sources") or [None])[0]
                    receipt_repo = receipt_item.get("repository")
                    if isinstance(primary, Mapping) and isinstance(receipt_repo, Mapping):
                        if receipt_repo.get("url") != primary.get("repository_url") or str(receipt_repo.get("revision", "")).lower() != str(primary.get("revision", "")).lower():
                            errors.append(f"execution receipt {system} repository/revision drifts from source closure")
                            receipt_ok = False
                receipt_results[system] = receipt_ok
        checks["execution_receipt_bindings"] = {
            "pass": all(receipt_results.values()) if receipt_results else False,
            "per_system": receipt_results,
            "profile_section_sha256": closure_hash,
        }

    status = "INVALID" if errors else ("NOT_READY" if not_ready else ("INCOMPLETE" if incomplete else "PASS"))
    return {
        "schema_version": SCHEMA_VERSION,
        "status": status,
        "pass": status == "PASS",
        "errors": errors,
        "incomplete": incomplete,
        "not_ready": not_ready,
        "checks": checks,
        "tree_hash_kind": TREE_HASH_KIND,
    }


def main() -> int:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--profile", type=Path, default=DEFAULT_PROFILE)
    parser.add_argument("--receipt", type=Path, default=DEFAULT_RECEIPT)
    parser.add_argument("--output", type=Path, required=True)
    args = parser.parse_args()
    profile = yaml.safe_load(args.profile.read_text(encoding="utf-8"))
    receipt = yaml.safe_load(args.receipt.read_text(encoding="utf-8")) if args.receipt.is_file() else None
    result = verify_rival_source_closure(profile, root=ROOT, receipt=receipt)
    args.output.parent.mkdir(parents=True, exist_ok=True)
    args.output.write_text(json.dumps(result, indent=2, sort_keys=True) + "\n", encoding="utf-8")
    print(json.dumps(result, indent=2, sort_keys=True))
    return 0 if result["pass"] else 1


if __name__ == "__main__":
    try:
        sys.exit(main())
    except (OSError, UnicodeError, ValueError, TypeError, yaml.YAMLError,
            json.JSONDecodeError) as exc:
        print(f"error: {exc}", file=sys.stderr)
        sys.exit(2)

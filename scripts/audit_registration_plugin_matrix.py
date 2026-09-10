#!/usr/bin/env python3
"""Read-only Humble/Jazzy registration-plugin matrix audit.

This audit is intentionally not a build or container runner.  It verifies the
current source identity, records the locally available ROS/toolchain prefixes,
checks optional dependency visibility, and classifies historical registration
receipts.  Historical documentation claims are never promoted to current
evidence: an artifact without an immutable sidecar, or without the current
transaction/provenance/ODR source manifest, is reported as superseded.

The optional ``--inspect-images`` mode performs only ``docker image inspect``
with a pinned digest.  It never builds, pulls, starts, or removes a container.
No network, bag, GT, scorer, or map operation is part of this script.
"""

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

from __future__ import print_function

import argparse
import hashlib
import importlib.metadata
import importlib.util
import json
import os
import re
import shutil
import stat
import subprocess
import sys
import uuid
import xml.etree.ElementTree as ET
from pathlib import Path


SCHEMA_VERSION = 1
PROFILE_SCHEMA = "registration-plugin-matrix-v1"
RELEASE_MATRIX_SCHEMA = "registration-plugin-release-matrix-v1"
CAMPAIGN_SET_SCHEMA = "registration-plugin-campaign-set-v1"
CAMPAIGN_SET_VERSION = 1
CAMPAIGN_ID_RE = re.compile(r"^[a-z0-9][a-z0-9._-]{7,127}$")
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
COMMIT_RE = re.compile(r"^[0-9a-fA-F]{40}$")
ROS_IMAGE_RE = re.compile(
    r"(?<![A-Za-z0-9_./-])(?:docker\.io/library/)?ros:[A-Za-z0-9_.-]+"
    r"(?:@sha256:[0-9a-f]{64})?"
)
OFFICIAL_ROS_REPOSITORY = "docker.io/library/ros"
HUMBLE_INDEX_DIGEST = "sha256:ebae805c9d985e443b26e13a47339098dc0a42eee4626055bfd4ebc6dcdb4988"
HUMBLE_AMD64_CHILD_DIGEST = "sha256:32bf718e63618482ffb1fe232cf0f834635c57162e2506fb0bc0b092ef776c1e"
OCI_INDEX_MEDIA_TYPE = "application/vnd.oci.image.index.v1+json"
OCI_IMAGE_MEDIA_TYPE = "application/vnd.oci.image.manifest.v1+json"
OFFICIAL_DEPENDENCY_REPOSITORIES = {
    "fast_gicp": {
        "official_url": "https://github.com/SMRT-AIST/fast_gicp.git",
        "archive_prefix": "https://codeload.github.com/SMRT-AIST/fast_gicp/tar.gz/",
    },
    "small_gicp": {
        "official_url": "https://github.com/koide3/small_gicp.git",
        "archive_prefix": "https://codeload.github.com/koide3/small_gicp/tar.gz/",
    },
}
APT_SOURCE_URL_POLICY = {
    "schema": "registration-plugin-apt-source-url-policy-v1",
    "schema_version": 1,
    "https": "allowed",
    "http_prefixes": [
        "http://archive.ubuntu.com/ubuntu",
        "http://packages.ros.org/ros2/ubuntu",
        "http://security.ubuntu.com/ubuntu",
    ],
    "http_default_port": 80,
    "redirect_policy": "same-host-only-and-requested-url-recorded",
}
CONSUMER_PROFILE_BINDING_SCHEMA = "registration-plugin-rosdep-consumer-profile-binding-v1"
DISCOVERY_PROFILE_SHA256 = (
    "6f4faf87ff7a81dabe75141d0335367a7f0cd3ae39ee74d878aaf68da4888c86"
)
RELEASE_DECLARATION_POLICY = {
    "schema": "registration-plugin-release-declaration-size-policy-v1",
    "schema_version": 1,
    "max_declared_bytes": 16 * 1024 * 1024 * 1024,
    "decimal_grammar": "0|[1-9][0-9]*",
    "zero_allowed": True,
}
PACKAGES_FORMAT_POLICY = {
    "schema": "registration-plugin-packages-format-policy-v1",
    "schema_version": 1,
    "selection_order": ["xz", "gz", "plain"],
    "allowed_formats": ["xz", "gz", "plain"],
    "compressed_max_bytes": 64 * 1024 * 1024,
    "decoded_max_bytes": 256 * 1024 * 1024,
    "max_expansion_ratio": 256,
    "trailing_data": "reject",
    "concatenated_streams": "reject",
    "terminal_empty_stream_policy": {
        "schema": "registration-plugin-xz-terminal-empty-stream-policy-v1",
        "schema_version": 1,
        "enabled": True,
        "max_streams": 2,
        "compressed_bytes": 32,
        "decoded_bytes": 0,
        "padding": "forbidden",
        "binding": "index-offset-bytes-sha256-decoded-bytes",
    },
    "signed_empty_packages_policy": {
        "schema": "registration-plugin-signed-empty-packages-policy-v1",
        "schema_version": 1,
        "enabled": True,
        "signed_empty": True,
        "decoded_sha256": (
            "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
        ),
        "decoded_bytes": 0,
        "allowed_formats": ["xz"],
        "xz_stream_count": 1,
        "xz_compressed_bytes": 32,
        "padding": "forbidden",
        "trailing_data": "reject",
        "binding": "signed-release-pair-compressed-bytes-and-exact-empty-decode",
    },
}
GPGV_STATUS_POLICY = {
    "schema": "registration-plugin-gpgv-status-policy-v1",
    "schema_version": 1,
    "gnupg_version": "2.2.27",
    "required_proof_tags": ["NEWSIG", "GOODSIG", "VALIDSIG"],
    "optional_informational_tags": [
        "KEY_CONSIDERED", "VERIFICATION_COMPLIANCE_MODE"
    ],
    "informational_pair": "independently-optional",
    "status_order": [
        "NEWSIG", "KEY_CONSIDERED", "SIG_ID", "GOODSIG", "VALIDSIG",
        "VERIFICATION_COMPLIANCE_MODE"
    ],
    "key_considered_fields": 4,
    "key_considered_fingerprint": "validsig-primary-or-signer",
    "key_considered_flags": "canonical-decimal-0..2147483647",
    "verification_compliance_mode_fields": 3,
    "verification_compliance_mode": "canonical-decimal-0..2147483647",
    "informational_only": True,
    "proof": "GOODSIG_VALIDSIG_only",
    "keyring_input": "dearmored_path_only",
}
GPG_INVENTORY_POLICY = {
    "schema": "registration-plugin-gpg-key-inventory-policy-v1",
    "schema_version": 1,
    "executable": "/usr/bin/gpg",
    "versions": ["gpg (GnuPG) 2.2.27", "gpg (GnuPG) 2.4.4"],
    "version_argv": ["/usr/bin/gpg", "--version"],
    "argv_template": [
        "/usr/bin/gpg", "--batch", "--no-options", "--no-default-keyring",
        "--no-auto-key-retrieve", "--no-auto-key-import", "--no-auto-key-locate",
        "--no-autostart", "--homedir", "<fresh-homedir>", "--with-colons",
        "--with-fingerprint", "--with-subkey-fingerprint", "--import-options",
        "show-only", "--import", "<dearmored-keyring>",
    ],
    "homedir": {
        "parent": "/workspace/evidence-parent/gpg-inventory-work",
        "parent_relative": "gpg-inventory-work",
        "prefix": "glim-clean-room-r3-gpg-inventory-", "token_hex_length": 24,
        "mode": 448, "post_absent": True, "parent_empty_before_after": True,
        "cleanup": "owned-tree-only-and-absence-required",
    },
    "environment": "fixed-locale-no-home-or-agent",
    "shell": False, "network": "none",
    "version_timeout_seconds": 30, "inventory_timeout_seconds": 120,
    "metadata_residue_schema": "registration-plugin-gpg-inventory-metadata-v1",
    "metadata_residue_policy": "exact-pubring.kbx-trustdb.gpg-supported-show-only",
    "metadata_residue_names": ["pubring.kbx", "trustdb.gpg"],
    "metadata_residue_mode": 384,
    "metadata_residue_max_bytes": 16777216,
    "metadata_residue_retention": "descriptor-only-cleaned",
    "failure_status": "REVIEW_REQUIRED",
}
CAPTURE_CONTRACT_SCHEMA = "registration-plugin-dependency-capture-integration-v1"
CAPTURE_CONTRACT_BINDING_PATHS = {
    "capture_tool": "scripts/capture_registration_plugin_dependency_closure.py",
    "capture_schema": "configs/slam_benchmark_profiles/registration_plugin_dependency_capture_executor_v1.schema.json",
    "closure_tool": "scripts/registration_plugin_dependency_closure.py",
    "closure_schema": "configs/slam_benchmark_profiles/registration_plugin_dependency_closure_v1.schema.json",
    "directory_tool": "scripts/registration_plugin_evidence_directory.py",
    "directory_schema": "configs/slam_benchmark_profiles/registration_plugin_capture_directory_contract_v1.schema.json",
    "source_validator": "docker/benchmark_adapters/glim_clean_room/phase3d/r3/apt_allowlist_composer.py",
    "source_schema": "configs/slam_benchmark_profiles/registration_plugin_apt_source_snapshot_v2.schema.json",
    "rosdep_prepare_tool": "scripts/prepare_registration_plugin_rosdep.py",
    "rosdep_prepare_schema": "configs/slam_benchmark_profiles/registration_plugin_rosdep_prepare_v1.schema.json",
}
CAPTURE_REQUIRED_ARTIFACTS = [
    "capture_input", "base_status", "apt_source_capture", "dependency_capture",
    "rosdep_prepare",
]
CAMPAIGN_AGGREGATION_SCHEMA = "registration-plugin-release-campaign-aggregation-v1"
CAMPAIGN_AGGREGATION_BINDING_PATHS = {
    "tool": "scripts/run_registration_plugin_release_campaign.py",
    "campaign_schema": "configs/slam_benchmark_profiles/registration_plugin_release_campaign_v1.schema.json",
    "partial_receipt_schema": "configs/slam_benchmark_profiles/registration_plugin_release_child_partial_v1.schema.json",
}
DEPENDENCY_CAPTURE_CAMPAIGN_AGGREGATION_SCHEMA = (
    "registration-plugin-dependency-capture-campaign-aggregation-v1"
)
DEPENDENCY_CAPTURE_CAMPAIGN_AGGREGATION_BINDING_PATHS = {
    "tool": "scripts/capture_registration_plugin_dependency_closure.py",
    "campaign_schema": (
        "configs/slam_benchmark_profiles/"
        "registration_plugin_dependency_capture_campaign_v1.schema.json"
    ),
    "capture_executor_schema": (
        "configs/slam_benchmark_profiles/"
        "registration_plugin_dependency_capture_executor_v1.schema.json"
    ),
    "partial_receipt_schema": (
        "configs/slam_benchmark_profiles/"
        "registration_plugin_dependency_capture_child_partial_v1.schema.json"
    ),
}
REFERENCE_POLICY = {
    "schema": "registration-plugin-capture-reference-policy-v1",
    "schema_version": 1,
    "identity_domains": {
        "release": "registration-plugin-release-artifact-identity-v1",
        "keyring": "registration-plugin-gpg-keyring-identity-v1",
        "signature": "registration-plugin-release-signature-identity-v1",
        "package": "registration-plugin-packages-artifact-identity-v1",
    },
    "identity_token_hex_length": 24,
    "artifact_naming": "basename-plus-full-identity-sha256-prefix",
    "prefix_collision": "reject-before-write",
    "cache_semantics": "first-attempt-retained-deep-copy",
    "release_deduplication": "immutable-release-identity",
    "keyring_deduplication": "source-scoped-keyring-identity",
    "signature_deduplication": "release-keyring-signer-policy-identity",
    "reference_projection": "exact-map-and-counts",
    "partial_projection": "raw-release-before-later-validation",
    "promotion": "non-promoting-until-signed-receipt",
}
RESTORE_OWNER_POLICY = {
    "schema": "registration-plugin-apt-partial-owner-restore-v1",
    "schema_version": 1,
    "phase_order": [
        "restore_build_partial_owner", "restore_runtime_partial_owner",
    ],
    "reference_mode": 0o700,
    "reference_mount": "ordered-host-source-container-target-bindings-v1",
    "mount_access": "read-only-bind",
    "command": "docker-exec-chown-reference",
    "mode_policy": "reference-mode-verified",
    "target_children": "empty-before-and-after",
    "identity": "host-dev-inode-owner-mode-nlink-pre-post",
    "cleanup": "no-post-hoc-chown-or-chmod",
    "before_policy": "presealed-lstat-only-on-exact-transient-owner",
    "transient_owner": {
        "schema": "registration-plugin-apt-transient-owner-policy-v2",
        "schema_version": 2,
        "identities": [
            {"distro": "humble", "name": "_apt", "uid": 100, "gid": 0},
            {"distro": "jazzy", "name": "_apt", "uid": 42, "gid": 0},
        ],
        "identity_evidence": "profile-distro-image-digest-and-base-status/passwd",
        "binding": "distro-profile-image-digest",
    },
}
DEFAULT_PROFILE = (
    Path(__file__).resolve().parents[1]
    / "configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json"
)

ARTIFACT_MANIFEST_SCHEMA = "registration-plugin-artifact-manifest-v1"
ARTIFACT_MANIFEST_VERSION = 1
ARTIFACT_ROOT_SCHEMA = "registration-plugin-evidence-root-v1"
ARTIFACT_ROOT_VERSION = 1
FUNCTIONAL_RESOURCE_POLICY_SCHEMA = "registration-plugin-functional-resource-policy-v1"
FUNCTIONAL_RESOURCE_POLICY_VERSION = 1
FUNCTIONAL_RESOURCE_MODE = "functional_non_authoritative_v1"
FUNCTIONAL_TIMING_AUTHORITY = "NON_AUTHORITATIVE_CONTAMINATED"
PERFORMANCE_RESOURCE_POLICY_SCHEMA = "registration-plugin-performance-quiescence-policy-v1"
PERFORMANCE_RESOURCE_POLICY_VERSION = 1
ARTIFACT_REQUIRED_ROLES = frozenset({
    "root_marker_receipt", "root_marker_receipt_sidecar", "command_log", "installed_tree_file",
    "test_result", "dso_receipt", "dso_receipt_sidecar", "provenance_receipt",
    "provenance_receipt_sidecar", "resource_failure_receipt",
    "resource_failure_receipt_sidecar", "dependency_environment_receipt",
    "dependency_environment_receipt_sidecar",
})
EVIDENCE_STORAGE_SCHEMA = "registration-plugin-evidence-storage-v1"
EVIDENCE_STORAGE_UUID_RE = re.compile(r"^[0-9a-f]{8}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{4}-[0-9a-f]{12}$")


class AuditError(RuntimeError):
    """A fail-closed audit error."""


def _artifact_fail(message):
    raise AuditError("ARTIFACT_INTEGRITY: {}".format(message))


def normalize_artifact_relative_path(value, label="artifact path"):
    """Return a portable relative path or fail before any filesystem access."""
    if not isinstance(value, str) or not value or "\\" in value or "\x00" in value:
        _artifact_fail("{} is not a portable relative path".format(label))
    if value.startswith("/") or value.startswith("./") or value.endswith("/"):
        _artifact_fail("{} is absolute or unnormalized: {}".format(label, value))
    parts = value.split("/")
    if any(part in ("", ".", "..") for part in parts):
        _artifact_fail("{} contains traversal or empty components: {}".format(label, value))
    normalized = "/".join(parts)
    if normalized != value:
        _artifact_fail("{} is not normalized: {}".format(label, value))
    return normalized


def _artifact_root(path):
    path = Path(path).absolute()
    try:
        if path.is_symlink() or not path.is_dir() or path.resolve() != path:
            _artifact_fail("evidence root is not a canonical regular directory: {}".format(path))
    except OSError as exc:
        _artifact_fail("evidence root cannot be inspected: {}".format(exc))
    return path


def _artifact_file(root, relative, label="artifact"):
    root = _artifact_root(root)
    relative = normalize_artifact_relative_path(relative, label)
    candidate = root
    parts = relative.split("/")
    for index, part in enumerate(parts):
        candidate = candidate / part
        try:
            mode = os.lstat(str(candidate)).st_mode
        except OSError as exc:
            _artifact_fail("{} is missing: {}".format(label, exc))
        if stat.S_ISLNK(mode):
            _artifact_fail("{} contains a symlink: {}".format(label, relative))
        if index != len(parts) - 1 and not stat.S_ISDIR(mode):
            _artifact_fail("{} has a non-directory path component: {}".format(label, relative))
    try:
        mode = os.lstat(str(candidate)).st_mode
    except OSError as exc:
        _artifact_fail("{} cannot be inspected: {}".format(label, exc))
    if not stat.S_ISREG(mode):
        _artifact_fail("{} is not a regular file: {}".format(label, relative))
    if stat.S_IMODE(mode) == 0 or getattr(os.stat(str(candidate)), "st_nlink", 1) != 1:
        _artifact_fail("{} has ambiguous hard-link or invalid mode metadata: {}".format(
            label, relative))
    return candidate


def _artifact_directory(root, relative, label="artifact directory"):
    root = _artifact_root(root)
    if relative == ".":
        return root
    relative = normalize_artifact_relative_path(relative, label)
    candidate = root
    for part in relative.split("/"):
        candidate = candidate / part
        try:
            mode = os.lstat(str(candidate)).st_mode
        except OSError as exc:
            _artifact_fail("{} is missing: {}".format(label, exc))
        if stat.S_ISLNK(mode) or not stat.S_ISDIR(mode):
            _artifact_fail("{} is not a non-symlink directory: {}".format(label, relative))
    return candidate


def _artifact_file_entry(root, relative, role):
    path = _artifact_file(root, relative, role)
    return {
        "role": role,
        "path": normalize_artifact_relative_path(relative, role),
        "size_bytes": path.stat().st_size,
        "sha256": sha256_file(path),
    }


def _artifact_file_entries(root, relative_root, role, require_nonempty=True):
    directory = _artifact_directory(root, relative_root, role + " root")
    entries = []
    for current, directories, files in os.walk(str(directory), topdown=True, followlinks=False):
        current_path = Path(current)
        for directory_name in list(directories):
            candidate = current_path / directory_name
            try:
                if candidate.is_symlink():
                    _artifact_fail("{} root contains a symlink directory: {}".format(role, candidate))
            except OSError as exc:
                _artifact_fail("{} root cannot inspect directory: {}".format(role, exc))
        for file_name in files:
            candidate = current_path / file_name
            relative = candidate.relative_to(root).as_posix()
            entries.append(_artifact_file_entry(root, relative, role))
    if require_nonempty and not entries:
        _artifact_fail("required {} root is empty: {}".format(role, relative_root))
    return entries


def artifact_root_identity(*, logical_name, distro, dependency_leg,
                           profile_sha256, source_manifest_sha256):
    payload = {
        "schema": ARTIFACT_ROOT_SCHEMA,
        "schema_version": ARTIFACT_ROOT_VERSION,
        "logical_name": logical_name,
        "distro": distro,
        "dependency_leg": dependency_leg,
        "profile_sha256": profile_sha256,
        "source_manifest_sha256": source_manifest_sha256,
    }
    payload["identity_sha256"] = hashlib.sha256(
        _canonical(payload).encode("utf-8")).hexdigest()
    return payload


def build_artifact_manifest(root, root_identity, required_roots, explicit_files,
                            role_overrides=None):
    """Build a deterministic manifest for the final per-leg evidence tree."""
    root = _artifact_root(root)
    if not isinstance(root_identity, dict) or root_identity.get("schema") != ARTIFACT_ROOT_SCHEMA:
        _artifact_fail("root identity schema is invalid")
    entries = []
    normalized_roots = []
    for item in required_roots:
        if not isinstance(item, dict) or not isinstance(item.get("path"), str):
            _artifact_fail("required artifact root is malformed")
        relative = item["path"]
        if relative != ".":
            relative = normalize_artifact_relative_path(relative, "required artifact root")
        role = item.get("role")
        if not isinstance(role, str) or not role:
            _artifact_fail("required artifact root role is missing")
        normalized_roots.append({"path": relative, "role": role,
                                 "required": item.get("required", True) is not False})
        entries.extend(_artifact_file_entries(
            root, relative, role, require_nonempty=item.get("required", True) is not False))
    for item in explicit_files:
        if not isinstance(item, dict) or not isinstance(item.get("path"), str):
            _artifact_fail("explicit artifact is malformed")
        role = item.get("role")
        if not isinstance(role, str) or not role:
            _artifact_fail("explicit artifact role is missing")
        entry = _artifact_file_entry(root, item["path"], role)
        if item.get("for_path") is not None:
            entry["for_path"] = normalize_artifact_relative_path(item["for_path"],
                                                                   "artifact sidecar target")
        entries.append(entry)
    overrides = role_overrides or {}
    for entry in entries:
        if entry["path"] in overrides:
            entry["role"] = overrides[entry["path"]]
    paths = [entry["path"] for entry in entries]
    if len(paths) != len(set(paths)):
        _artifact_fail("artifact manifest contains duplicate normalized paths")
    entries.sort(key=lambda value: (value["path"], value["role"]))
    return {
        "schema": ARTIFACT_MANIFEST_SCHEMA,
        "schema_version": ARTIFACT_MANIFEST_VERSION,
        "evidence_root": root_identity,
        "required_roots": normalized_roots,
        "entries": entries,
        "entry_count": len(entries),
        "hash_kind": "relative_path_size_content_sha256_v1",
    }


def _read_artifact_sidecar(root, receipt_entry, entries_by_path):
    receipt_path = receipt_entry["path"]
    sidecar_relative = receipt_path + ".sha256"
    sidecar = entries_by_path.get(sidecar_relative)
    if sidecar is None or sidecar.get("role") != receipt_entry["role"] + "_sidecar":
        _artifact_fail("receipt sidecar is missing from the artifact manifest: {}".format(receipt_path))
    sidecar_path = _artifact_file(root, sidecar_relative, "receipt sidecar")
    try:
        tokens = sidecar_path.read_text(encoding="ascii").strip().split()
    except (OSError, UnicodeError) as exc:
        _artifact_fail("receipt sidecar is not ASCII: {}".format(exc))
    if len(tokens) != 2 or tokens[1] != Path(receipt_path).name or tokens[0] != receipt_entry["sha256"]:
        _artifact_fail("receipt sidecar content mismatch: {}".format(receipt_path))
    observed = _artifact_file_entry(root, sidecar_relative, sidecar["role"])
    if observed["size_bytes"] != sidecar["size_bytes"] or observed["sha256"] != sidecar["sha256"]:
        _artifact_fail("receipt sidecar bytes changed: {}".format(sidecar_relative))


def validate_artifact_manifest(root, manifest, expected_root_identity,
                               required_roles=None):
    """Reopen and verify every declared final artifact under ``root``."""
    root = _artifact_root(root)
    if not isinstance(manifest, dict) or manifest.get("schema") != ARTIFACT_MANIFEST_SCHEMA or \
            manifest.get("schema_version") != ARTIFACT_MANIFEST_VERSION:
        _artifact_fail("artifact manifest schema/version mismatch")
    if manifest.get("evidence_root") != expected_root_identity:
        _artifact_fail("evidence root identity mismatch")
    entries = manifest.get("entries")
    if not isinstance(entries, list) or not entries:
        _artifact_fail("artifact manifest entries are missing")
    if manifest.get("entry_count") != len(entries):
        _artifact_fail("artifact manifest entry count mismatch")
    entries_by_path = {}
    for entry in entries:
        if not isinstance(entry, dict):
            _artifact_fail("artifact manifest entry is malformed")
        path = normalize_artifact_relative_path(entry.get("path"), "artifact manifest path")
        if path in entries_by_path:
            _artifact_fail("artifact manifest contains duplicate path: {}".format(path))
        role = entry.get("role")
        if not isinstance(role, str) or not role:
            _artifact_fail("artifact manifest role is missing: {}".format(path))
        if not isinstance(entry.get("size_bytes"), int) or entry["size_bytes"] < 0 or \
                not SHA256_RE.fullmatch(str(entry.get("sha256", ""))):
            _artifact_fail("artifact manifest size/hash is invalid: {}".format(path))
        observed = _artifact_file_entry(root, path, role)
        if observed["size_bytes"] != entry["size_bytes"] or observed["sha256"] != entry["sha256"]:
            _artifact_fail("artifact bytes changed: {}".format(path))
        entries_by_path[path] = entry
    for root_spec in manifest.get("required_roots", []):
        if not isinstance(root_spec, dict):
            _artifact_fail("required artifact root is malformed")
        relative = root_spec.get("path")
        if relative != ".":
            relative = normalize_artifact_relative_path(relative, "required artifact root")
        role = root_spec.get("role")
        expected = {path for path, entry in entries_by_path.items()
                    if path == relative or path.startswith(relative + "/")}
        actual = set()
        directory = _artifact_directory(root, relative, "required artifact root")
        for current, directories, files in os.walk(str(directory), topdown=True, followlinks=False):
            current_path = Path(current)
            for directory_name in list(directories):
                if (current_path / directory_name).is_symlink():
                    _artifact_fail("required artifact root contains a symlink directory")
            for file_name in files:
                candidate = current_path / file_name
                path = candidate.relative_to(root).as_posix()
                _artifact_file(root, path, "required artifact file")
                actual.add(path)
        if root_spec.get("required", True) is not False and not actual:
            _artifact_fail("required artifact root is empty: {}".format(relative))
        if actual != expected:
            missing = sorted(actual - expected)
            undeclared = sorted(expected - actual)
            _artifact_fail("artifact root declaration mismatch {} missing={} undeclared={}".format(
                relative, missing, undeclared))
    required_roles = set(required_roles or ())
    missing_roles = sorted(role for role in required_roles
                           if not any(entry.get("role") == role for entry in entries))
    if missing_roles:
        _artifact_fail("required artifact roles are missing: {}".format(missing_roles))
    for entry in entries:
        if entry["role"].endswith("_receipt"):
            _read_artifact_sidecar(root, entry, entries_by_path)
    return {
        "status": "PASS",
        "schema": manifest["schema"],
        "schema_version": manifest["schema_version"],
        "entry_count": len(entries),
        "root_identity": expected_root_identity,
    }


def validate_tree_manifest(root, recorded, label="tree manifest"):
    """Recompute a runner tree manifest from the downloaded evidence root."""
    if not isinstance(recorded, dict) or recorded.get("status") != "PASS":
        _artifact_fail("{} is not a PASS manifest".format(label))
    relative = recorded.get("root_relative")
    if not isinstance(relative, str):
        _artifact_fail("{} root_relative is missing".format(label))
    directory = _artifact_directory(root, relative, label + " root")
    entries = []
    for current, directories, files in os.walk(str(directory), topdown=True, followlinks=False):
        current_path = Path(current)
        for directory_name in list(directories):
            candidate = current_path / directory_name
            try:
                mode = os.lstat(str(candidate)).st_mode
            except OSError as exc:
                _artifact_fail("{} directory cannot be inspected: {}".format(label, exc))
            if stat.S_ISLNK(mode):
                _artifact_fail("{} contains a symlink directory".format(label))
            entries.append({
                "kind": "directory",
                "mode": stat.S_IMODE(mode),
                "path": candidate.relative_to(directory).as_posix(),
            })
        for file_name in files:
            candidate = current_path / file_name
            relative_file = candidate.relative_to(directory).as_posix()
            _artifact_file(root, directory.relative_to(root).as_posix() + "/" + relative_file,
                           label + " file")
            mode = os.lstat(str(candidate)).st_mode
            entries.append({
                "kind": "file",
                "mode": stat.S_IMODE(mode),
                "path": relative_file,
                "size_bytes": candidate.stat().st_size,
                "sha256": sha256_file(candidate),
            })
    entries.sort(key=lambda value: value["path"])
    observed = {
        "status": "PASS",
        "root_relative": relative,
        "hash_kind": "relative_lstat_tree_manifest_v1",
        "symlink_count": 0,
        "entry_count": len(entries),
        "entries": entries,
        "tree_sha256": hashlib.sha256(_canonical(entries).encode("utf-8")).hexdigest(),
    }
    for key in ("root_relative", "hash_kind", "symlink_count", "entry_count", "tree_sha256", "entries"):
        if recorded.get(key) != observed.get(key):
            _artifact_fail("{} changed at {}".format(label, key))
    return observed


def _canonical(value):
    return json.dumps(value, sort_keys=True, separators=(",", ":"), ensure_ascii=True)


def sha256_file(path):
    digest = hashlib.sha256()
    with open(str(path), "rb") as stream:
        for chunk in iter(lambda: stream.read(1024 * 1024), b""):
            digest.update(chunk)
    return digest.hexdigest()


def _absolute(path_value, label):
    path = Path(path_value).expanduser()
    if not path.is_absolute():
        raise AuditError("{} must be absolute: {}".format(label, path_value))
    return path


def _regular_file(path_value, label):
    path = _absolute(path_value, label)
    try:
        if path.is_symlink():
            raise AuditError("{} must not be a symlink: {}".format(label, path))
        if not path.is_file():
            raise AuditError("{} is not a regular file: {}".format(label, path))
    except OSError as exc:
        raise AuditError("{} cannot be inspected: {}".format(label, exc))
    return path


def _relative_source(repo_root, relative, label):
    if not isinstance(relative, str) or not relative or os.path.isabs(relative):
        raise AuditError("{} must be a relative path: {}".format(label, relative))
    path = (repo_root / relative).resolve(strict=False)
    root = repo_root.resolve()
    try:
        path.relative_to(root)
    except ValueError:
        raise AuditError("{} escapes repository: {}".format(label, relative))
    return path


def _load_profile(path):
    path = _regular_file(path, "matrix profile")
    try:
        profile = json.loads(path.read_text(encoding="utf-8"))
    except (OSError, ValueError) as exc:
        raise AuditError("matrix profile is not valid JSON: {}".format(exc))
    if not isinstance(profile, dict) or profile.get("schema") != PROFILE_SCHEMA:
        raise AuditError("unsupported matrix profile schema")
    if profile.get("schema_version") != SCHEMA_VERSION:
        raise AuditError("unsupported matrix profile version")
    return profile, path


def _canonical_ros_image_tag(tag, label="image tag"):
    if not isinstance(tag, str) or not tag:
        raise AuditError("{} is missing".format(label))
    if tag.startswith("ros:"):
        tag = OFFICIAL_ROS_REPOSITORY + tag[len("ros"):]
    if not tag.startswith(OFFICIAL_ROS_REPOSITORY + ":"):
        raise AuditError("{} must use the official Docker Hub ROS repository".format(label))
    if "@" in tag or tag.endswith(":"):
        raise AuditError("{} must contain only an official image tag".format(label))
    return tag


def _validate_image_contract(image, distro, require_humble_manifest=False):
    if not isinstance(image, dict):
        raise AuditError("{} image is missing".format(distro))
    tag = _canonical_ros_image_tag(image.get("tag"), "{} image tag".format(distro))
    digest = image.get("digest")
    if not isinstance(digest, str) or not re.fullmatch(r"sha256:[0-9a-f]{64}", digest):
        raise AuditError("{} image digest is not pinned".format(distro))
    reference = image.get("reference")
    expected_reference = tag + "@" + digest
    if reference != expected_reference:
        raise AuditError("{} image reference is not bound to tag and digest".format(distro))
    manifest = image.get("manifest")
    if require_humble_manifest:
        if not isinstance(manifest, dict):
            raise AuditError("{} image manifest identity is missing".format(distro))
        expected = {
            "index_digest": HUMBLE_INDEX_DIGEST,
            "index_media_type": OCI_INDEX_MEDIA_TYPE,
            "linux_amd64_digest": HUMBLE_AMD64_CHILD_DIGEST,
            "linux_amd64_media_type": OCI_IMAGE_MEDIA_TYPE,
            "linux_amd64_os": "linux",
            "linux_amd64_architecture": "amd64",
        }
        for key, value in expected.items():
            if manifest.get(key) != value:
                raise AuditError(
                    "{} image manifest {} is not bound to verified identity".format(
                        distro, key
                    )
                )
    return {
        "tag": tag,
        "digest": digest,
        "reference": expected_reference,
        "manifest": manifest,
    }


def _validate_workflow_image_contract(repo_root, profile):
    workflow = _relative_source(repo_root, ".github/workflows/main.yml", "CI workflow")
    if workflow.is_symlink() or not workflow.is_file():
        raise AuditError("CI workflow is missing or symlinked")
    text = workflow.read_text(encoding="utf-8")
    expected = {}
    for row in profile.get("distros", []):
        name = row.get("name")
        image = _validate_image_contract(
            row.get("image"), name, require_humble_manifest=name == "humble"
        )
        expected[name] = image["reference"]
    literals = ROS_IMAGE_RE.findall(text)
    if not literals:
        raise AuditError("CI workflow has no ROS image references")
    normalized = []
    for literal in literals:
        if "@sha256:" not in literal:
            raise AuditError("CI workflow contains an unpinned ROS image: {}".format(literal))
        normalized_tag = _canonical_ros_image_tag(literal.split("@", 1)[0], "CI image")
        normalized_ref = normalized_tag + "@" + literal.split("@", 1)[1]
        if normalized_ref not in expected.values():
            raise AuditError("CI workflow image is absent from the profile: {}".format(literal))
        normalized.append(normalized_ref)
    missing = sorted(set(expected.values()) - set(normalized))
    if missing:
        raise AuditError("CI workflow is missing profile image references: {}".format(", ".join(missing)))
    return {
        "status": "PASS",
        "path": ".github/workflows/main.yml",
        "sha256": sha256_file(workflow),
        "references": sorted(set(normalized)),
    }


def _validate_consumer_profile_binding(value):
    """Validate the historical discovery allowlist in the active profile."""
    if not isinstance(value, dict) or set(value) != {
            "schema", "schema_version", "consumer_profile",
            "discovery_profile_sha256", "discovery_allowlist"}:
        raise AuditError("consumer profile binding fields are incomplete or extra")
    if (value["schema"] != CONSUMER_PROFILE_BINDING_SCHEMA or
            value["schema_version"] != 1 or
            value["discovery_profile_sha256"] != DISCOVERY_PROFILE_SHA256):
        raise AuditError("consumer profile binding schema/identity is invalid")
    consumer = value["consumer_profile"]
    if not isinstance(consumer, dict) or set(consumer) != {
            "path", "sha256_source"}:
        raise AuditError("consumer profile binding consumer identity is invalid")
    expected_profile = Path(__file__).resolve().parents[1] / (
        "configs/slam_benchmark_profiles/"
        "registration_plugin_matrix_current_2026-08.json"
    )
    if (consumer["path"] != str(expected_profile) or
            consumer["sha256_source"] != "active_profile_bytes_sha256"):
        raise AuditError("consumer profile binding path/source is invalid")
    prepare_path = Path(__file__).resolve().parent / (
        "prepare_registration_plugin_rosdep.py"
    )
    spec = importlib.util.spec_from_file_location(
        "registration_plugin_rosdep_prepare_policy_audit", prepare_path
    )
    if spec is None or spec.loader is None:
        raise AuditError("consumer profile binding helper cannot be loaded")
    prepare = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(prepare)
        expected_allowlist = prepare._discovery_allowlist()
    except Exception as exc:  # pragma: no cover - defensive profile boundary
        raise AuditError(
            "consumer profile discovery allowlist cannot be loaded: {}".format(exc)
        ) from exc
    if value["discovery_allowlist"] != expected_allowlist:
        raise AuditError("consumer profile discovery allowlist is not exact")


def _validate_campaign_aggregation_contract(value):
    """Validate the additive campaign partial-row contract and source pins."""
    required = {
        "schema", "schema_version", "tool", "campaign_schema",
        "partial_receipt_schema", "row_state_policy", "partial_rows_policy",
        "normal_rows_exclude_partial", "status", "benchmark_eligible",
        "promotion",
    }
    if not isinstance(value, dict) or set(value) != required or \
            value.get("schema") != CAMPAIGN_AGGREGATION_SCHEMA or \
            value.get("schema_version") != 1 or \
            value.get("row_state_policy") != "fixed-four-state-vector" or \
            value.get("partial_rows_policy") != "sealed-started-child-only" or \
            value.get("normal_rows_exclude_partial") is not True or \
            value.get("status") != "REVIEW_REQUIRED" or \
            value.get("benchmark_eligible") is not False or \
            value.get("promotion") != "FORBIDDEN_UNTIL_SIGNED_RECEIPT":
        raise AuditError("release_matrix campaign aggregation contract is invalid")
    repo_root = Path(__file__).resolve().parents[1]
    for key, relative in CAMPAIGN_AGGREGATION_BINDING_PATHS.items():
        binding = value.get(key)
        if not isinstance(binding, dict) or set(binding) != {"path", "sha256"} or \
                binding.get("path") != relative or \
                not SHA256_RE.fullmatch(str(binding.get("sha256", ""))):
            raise AuditError("campaign aggregation binding is invalid: {}".format(key))
        path = _relative_source(repo_root, relative, "campaign aggregation source")
        if sha256_file(path) != binding["sha256"]:
            raise AuditError("campaign aggregation source hash drift: {}".format(relative))


def _validate_dependency_capture_campaign_aggregation_contract(value):
    """Validate the capture command's separate partial-row contract."""
    required = {
        "schema", "schema_version", "tool", "campaign_schema",
        "capture_executor_schema", "partial_receipt_schema",
        "prepare_mount_policy", "row_state_policy", "rows_policy",
        "partial_rows_policy", "normal_rows_exclude_partial",
        "partial_row_identity", "status", "benchmark_eligible", "promotion",
    }
    if (not isinstance(value, dict) or set(value) != required or
            value.get("schema") !=
            DEPENDENCY_CAPTURE_CAMPAIGN_AGGREGATION_SCHEMA or
            value.get("schema_version") != 1 or
            value.get("row_state_policy") != "fixed-four-state-vector" or
            value.get("rows_policy") != "completed-pass-only" or
            value.get("partial_rows_policy") !=
            "sealed-started-child-only" or
            value.get("normal_rows_exclude_partial") is not True or
            value.get("partial_row_identity") !=
            "campaign-row-campaign-set-profile-child-receipt" or
            value.get("status") != "REVIEW_REQUIRED" or
            value.get("benchmark_eligible") is not False or
            value.get("promotion") != "FORBIDDEN_UNTIL_SIGNED_RECEIPT"):
        raise AuditError(
            "release_matrix dependency capture aggregation contract is invalid")
    repo_root = Path(__file__).resolve().parents[1]
    for key, relative in DEPENDENCY_CAPTURE_CAMPAIGN_AGGREGATION_BINDING_PATHS.items():
        binding = value.get(key)
        if (not isinstance(binding, dict) or set(binding) != {"path", "sha256"} or
                binding.get("path") != relative or
                not SHA256_RE.fullmatch(str(binding.get("sha256", "")))):
            raise AuditError(
                "dependency capture aggregation binding is invalid: {}".format(key))
        path = _relative_source(
            repo_root, relative, "dependency capture aggregation source")
        if sha256_file(path) != binding["sha256"]:
            raise AuditError(
                "dependency capture aggregation source hash drift: {}".format(relative))
    closure_path = repo_root / "scripts/registration_plugin_dependency_closure.py"
    closure_spec = importlib.util.spec_from_file_location(
        "registration_plugin_dependency_closure_aggregation_audit", closure_path)
    if closure_spec is None or closure_spec.loader is None:
        raise AuditError("dependency capture aggregation policy cannot be loaded")
    closure_module = importlib.util.module_from_spec(closure_spec)
    try:
        closure_spec.loader.exec_module(closure_module)
        expected_mount_policy = closure_module.ROSDEP_PREPARE_MOUNT_POLICY
    except Exception as error:  # pragma: no cover - defensive profile boundary
        raise AuditError(
            "dependency capture aggregation policy cannot be loaded: {}".format(error)
        ) from error
    if value.get("prepare_mount_policy") != expected_mount_policy:
        raise AuditError("dependency capture aggregation mount policy is not exact")


def _validate_release_matrix_profile(profile):
    """Validate the release-facing distro/optional-dependency matrix.

    The ordinary audit remains a read-only host inspection.  This validator is
    deliberately stricter than that legacy report because the release job is
    allowed to promote only a receipt produced by a real isolated build/load
    leg.  In particular, an optional-dependency ``present`` leg with no
    immutable upstream pin is an explicit block, never a skipped test.
    """
    release = profile.get("release_matrix")
    if not isinstance(release, dict):
        raise AuditError("profile is missing release_matrix")
    if release.get("schema") != RELEASE_MATRIX_SCHEMA:
        raise AuditError("unsupported release_matrix schema")
    if release.get("schema_version") != SCHEMA_VERSION:
        raise AuditError("unsupported release_matrix version")
    _validate_campaign_set_contract(release.get("campaign_set"))
    _validate_campaign_aggregation_contract(release.get("campaign_aggregation"))
    _validate_dependency_capture_campaign_aggregation_contract(
        release.get("dependency_capture_campaign_aggregation"))
    policy = release.get("policy")
    if not isinstance(policy, dict):
        raise AuditError("release_matrix policy is missing")
    _validate_evidence_storage_contract(release.get("evidence_storage"))
    required_policy = {
        "repository_mount": "read_only",
        "install_mode": "isolated_non_symlink",
        "artifact_retention_days": 14,
        "pull": False,
        "formal_replay": False,
        "bag_gt_scorer_map": False,
        "dependency_fetch": "official_archive_only",
        "archive_verification_before_extraction": True,
        "network_after_dependency_fetch": "forbidden_by_command_policy",
    }
    for key, expected in required_policy.items():
        if policy.get(key) != expected:
            raise AuditError(
                "release_matrix policy {} must be {!r}, got {!r}".format(
                    key, expected, policy.get(key)))
    dependency_closure = release.get("dependency_closure")
    expected_closure_keys = {
        "schema", "schema_version", "status", "required", "benchmark_eligible",
        "provisioning_network", "runtime_network", "runtime_flags",
        "signature_receipt_required", "required_phase_order", "capture_status",
        "promotion_status", "apt_source_url_policy", "consumer_profile_binding",
        "capture_contract",
    }
    if (not isinstance(dependency_closure, dict) or
            set(dependency_closure) != expected_closure_keys or
            dependency_closure.get("schema") !=
            "registration-plugin-dependency-closure-v1" or
            dependency_closure.get("schema_version") != 1 or
            dependency_closure.get("status") != "NOT_READY" or
            dependency_closure.get("required") is not True or
            dependency_closure.get("benchmark_eligible") is not False or
            dependency_closure.get("provisioning_network") !=
            "exact_apt_rosdep_and_pinned_archives_only" or
            dependency_closure.get("runtime_network") != "none" or
            dependency_closure.get("runtime_flags") !=
            ["--pull=never", "--network=none"] or
            dependency_closure.get("signature_receipt_required") is not True or
            dependency_closure.get("required_phase_order") != [
                "image_inspect", "preexisting_container_check", "container_start",
                "apt_update", "base_status_snapshot", "apt_source_snapshot",
                "resolver_build", "download_build", "restore_build_partial_owner",
                "resolver_runtime", "download_runtime", "restore_runtime_partial_owner",
                "dependency_install", "network_disconnect",
                "rosdep_prepare", "rosdep_resolution", "dependency_capture",
                "archive_prefetch", "post_disconnect_inspect"] or
            dependency_closure.get("capture_status") !=
            "CAPTURED_REVIEW_REQUIRED" or
            dependency_closure.get("promotion_status") !=
            "FORBIDDEN_UNTIL_SIGNED_RECEIPT" or
            dependency_closure.get("apt_source_url_policy") != APT_SOURCE_URL_POLICY):
        raise AuditError("release_matrix dependency closure contract is invalid")
    _validate_consumer_profile_binding(
        dependency_closure["consumer_profile_binding"]
    )
    _validate_capture_contract(repo_root=None, value=dependency_closure["capture_contract"])
    distro_names = {str(row.get("name")) for row in release.get("distros", [])
                    if isinstance(row, dict)}
    if distro_names != {"humble", "jazzy"}:
        raise AuditError("release_matrix must define exactly Humble and Jazzy")
    for row in release.get("distros", []):
        if not isinstance(row, dict):
            raise AuditError("release_matrix distro row is not an object")
        _validate_image_contract(
            row.get("image"), row.get("name"), require_humble_manifest=row.get("name") == "humble"
        )
        for leg in ("absent", "present"):
            leg_row = row.get("legs", {}).get(leg)
            if not isinstance(leg_row, dict):
                raise AuditError("{} {} leg is missing".format(row["name"], leg))
            if leg == "absent":
                if leg_row.get("status") != "REQUIRED":
                    raise AuditError("{} absent leg must be REQUIRED".format(row["name"]))
            else:
                status = leg_row.get("status")
                if status not in {"READY_PINNED", "BLOCKED_MISSING_PIN"}:
                    raise AuditError("{} present leg has invalid status {}".format(
                        row["name"], status))
                dependencies = leg_row.get("dependencies")
                if not isinstance(dependencies, list) or not dependencies:
                    raise AuditError("{} present leg has no dependency rows".format(row["name"]))
                for dependency in dependencies:
                    if not isinstance(dependency, dict):
                        raise AuditError("present dependency row is not an object")
                    required_dependency_keys = [
                        "name", "official_url", "commit", "source_sha256",
                        "build_argv", "install_argv", "expected_classes",
                    ]
                    if status == "READY_PINNED":
                        required_dependency_keys.extend([
                            "archive_url", "archive_sha256", "archive_top_level",
                            "source_tree_sha256", "license", "license_path",
                            "license_sha256", "compatibility",
                        ])
                    for key in required_dependency_keys:
                        if key not in dependency:
                            raise AuditError("present dependency missing {}".format(key))
                    if (not isinstance(dependency.get("build_argv"), list) or
                            not dependency.get("build_argv") or
                            not isinstance(dependency.get("install_argv"), list) or
                            not dependency.get("install_argv") or
                            not isinstance(dependency.get("expected_classes"), list) or
                            not dependency.get("expected_classes")):
                        raise AuditError("present dependency build/class contract is incomplete")
                    name = dependency.get("name")
                    repository = OFFICIAL_DEPENDENCY_REPOSITORIES.get(name)
                    url = str(dependency.get("official_url", ""))
                    if repository is None or url != repository["official_url"]:
                        raise AuditError("present dependency {} has a non-official repository URL".format(name))
                    if (not url.startswith("https://") or
                            any(token in url.lower() for token in ("latest", "main", "master"))):
                        raise AuditError("present dependency {} has an invalid official URL".format(
                            name))
                    commit = dependency.get("commit")
                    source_sha = dependency.get("source_sha256")
                    if status == "READY_PINNED":
                        if (not isinstance(commit, str) or not COMMIT_RE.fullmatch(commit) or
                                not SHA256_RE.fullmatch(str(source_sha)) or
                                source_sha != dependency.get("source_tree_sha256") or
                                not SHA256_RE.fullmatch(str(dependency.get("archive_sha256"))) or
                                not SHA256_RE.fullmatch(str(dependency.get("license_sha256")))):
                            raise AuditError(
                                "present dependency {} is not immutably pinned".format(
                                    name))
                        expected_archive = repository["archive_prefix"] + commit
                        if dependency.get("archive_url") != expected_archive:
                            raise AuditError("present dependency {} archive URL/commit mismatch".format(name))
                        expected_top_level = "{}-{}".format(name, commit)
                        if dependency.get("archive_top_level") != expected_top_level:
                            raise AuditError("present dependency {} archive top-level mismatch".format(name))
                        license_identity = {"fast_gicp": "BSD-3-Clause", "small_gicp": "MIT"}[name]
                        if dependency.get("license") != license_identity:
                            raise AuditError("present dependency {} license identity mismatch".format(name))
                        license_path = dependency.get("license_path")
                        if (not isinstance(license_path, str) or not license_path or
                                Path(license_path).is_absolute() or ".." in Path(license_path).parts):
                            raise AuditError("present dependency {} license path is unsafe".format(name))
                        compatibility = dependency.get("compatibility")
                        if (not isinstance(compatibility, dict) or
                                compatibility.get("status") != "STATIC_REVIEWED_RUNTIME_UNVERIFIED" or
                                set(compatibility.get("ros_distros", [])) != {"humble", "jazzy"} or
                                not compatibility.get("rationale")):
                            raise AuditError("present dependency {} compatibility rationale is incomplete".format(name))
                    elif any(dependency.get(key) is not None for key in (
                            "commit", "source_sha256", "archive_url", "archive_sha256",
                            "archive_top_level", "source_tree_sha256", "license",
                            "license_path", "license_sha256", "compatibility")):
                        raise AuditError(
                            "BLOCKED_MISSING_PIN {} must not carry a partial pin".format(
                                name))
    commands = release.get("commands")
    if not isinstance(commands, dict):
        raise AuditError("release_matrix commands are missing")
    for key in ("build", "test", "consumer", "template", "odr_gate"):
        value = commands.get(key)
        if not isinstance(value, list) or not value or not all(isinstance(v, str) for v in value):
            raise AuditError("release_matrix command {} is invalid".format(key))
    required_markers = release.get("required_evidence_markers")
    if not isinstance(required_markers, list) or not required_markers:
        raise AuditError("release_matrix required evidence markers are missing")
    artifact_contract = release.get("artifact_integrity")
    if not isinstance(artifact_contract, dict):
        raise AuditError("release_matrix artifact_integrity contract is missing")
    if artifact_contract.get("schema") != ARTIFACT_MANIFEST_SCHEMA or \
            artifact_contract.get("schema_version") != ARTIFACT_MANIFEST_VERSION:
        raise AuditError("release_matrix artifact manifest schema/version is invalid")
    if artifact_contract.get("root_schema") != ARTIFACT_ROOT_SCHEMA or \
            artifact_contract.get("root_version") != ARTIFACT_ROOT_VERSION:
        raise AuditError("release_matrix artifact root schema/version is invalid")
    if artifact_contract.get("receipt_name") != "registration_plugin_release.receipt.json" or \
            artifact_contract.get("manifest_name") != "artifact_manifest.json":
        raise AuditError("release_matrix artifact filenames are invalid")
    required_roles = artifact_contract.get("required_roles")
    if (not isinstance(required_roles, list) or
            set(required_roles) != set(ARTIFACT_REQUIRED_ROLES)):
        raise AuditError("release_matrix artifact required roles are incomplete")
    required_roots = artifact_contract.get("required_root_paths")
    present_roots = artifact_contract.get("present_root_paths")
    if (required_roots != ["work/logs", "work/test-results", "work/install",
                           "work/consumer", "work/template", "work/evidence"] or
            present_roots != ["work/archives"]):
        raise AuditError("release_matrix artifact root paths are invalid")
    for key in ("relative_only", "reject_symlinks", "reject_hardlinks"):
        if artifact_contract.get(key) is not True:
            raise AuditError("release_matrix artifact {} must be true".format(key))
    if artifact_contract.get("sidecar_format") != "sha256 two-token filename binding":
        raise AuditError("release_matrix artifact sidecar format is invalid")
    resource_policy = release.get("resource_policy")
    if not isinstance(resource_policy, dict):
        raise AuditError("release_matrix resource_policy contract is missing")
    functional = resource_policy.get("functional")
    performance = resource_policy.get("performance")
    if (resource_policy.get("schema") != FUNCTIONAL_RESOURCE_POLICY_SCHEMA or
            resource_policy.get("schema_version") != FUNCTIONAL_RESOURCE_POLICY_VERSION or
            not isinstance(functional, dict) or
            functional.get("mode") != FUNCTIONAL_RESOURCE_MODE or
            functional.get("allow_unrelated_cpu_load") is not True or
            functional.get("max_external_forbidden_processes") != 0 or
            functional.get("docker_idle_required") is not True or
            functional.get("max_build_workers") != 1 or
            functional.get("nice") != 19 or
            functional.get("ionice_class") != 3 or
            functional.get("timing_authority") != FUNCTIONAL_TIMING_AUTHORITY or
            functional.get("performance_gate_eligible") is not False or
            not isinstance(functional.get("min_mem_available_bytes"), int) or
            functional.get("min_mem_available_bytes") <= 0 or
            not isinstance(functional.get("min_disk_free_bytes"), int) or
            functional.get("min_disk_free_bytes") <= 0):
        raise AuditError("release_matrix functional resource policy is invalid")
    if (not isinstance(performance, dict) or
            performance.get("schema") != PERFORMANCE_RESOURCE_POLICY_SCHEMA or
            performance.get("schema_version") != PERFORMANCE_RESOURCE_POLICY_VERSION or
            performance.get("max_cpu_busy_percent") != 5.0 or
            performance.get("max_load1_per_cpu") != 0.5 or
            performance.get("timing_authority") != "AUTHORITATIVE_ONLY_AFTER_PASS"):
        raise AuditError("release_matrix performance quiescence policy is invalid")
    for row in profile.get("distros", []):
        _validate_image_contract(
            row.get("image"), row.get("name"), require_humble_manifest=row.get("name") == "humble"
        )
    return release


def _validate_capture_contract(repo_root, value):
    """Validate the fixed v2 capture/source/directory contract projection.

    ``repo_root`` is optional because callers that only validate the profile
    shape cannot safely assume a checkout path.  The full audit supplies the
    canonical repository root and reopens every bound source file before
    reporting a pass.
    """
    required = {
        "schema", "schema_version", "status", "benchmark_eligible",
        "contract_bindings", "required_artifacts", "artifact_policy",
        "directory_policy", "managed_directory_paths",
        "release_declaration_policy", "packages_format_policy",
        "gpgv_status_policy", "gpg_inventory_policy", "reference_policy",
        "restore_owner_policy", "source_snapshot", "promotion", "rosdep_prepare",
    }
    if not isinstance(value, dict) or set(value) != required:
        raise AuditError("capture contract fields are incomplete or extra")
    if value["schema"] != CAPTURE_CONTRACT_SCHEMA or value["schema_version"] != 1 or \
            value["status"] != "REVIEW_REQUIRED" or \
            value["benchmark_eligible"] is not False or \
            value["required_artifacts"] != CAPTURE_REQUIRED_ARTIFACTS or \
            value["artifact_policy"] != "exact_existing_sidecar_sealed_allowlist_v1" or \
            value["directory_policy"] != "host_precreated_0700_owner_bound_nofollow_v1" or \
            not isinstance(value["managed_directory_paths"], list) or \
            value["release_declaration_policy"] != RELEASE_DECLARATION_POLICY or \
            value["packages_format_policy"] != PACKAGES_FORMAT_POLICY or \
            value["gpgv_status_policy"] != GPGV_STATUS_POLICY or \
            value["gpg_inventory_policy"] != GPG_INVENTORY_POLICY or \
            value["reference_policy"] != REFERENCE_POLICY or \
            value["restore_owner_policy"] != RESTORE_OWNER_POLICY or \
            not isinstance(value.get("rosdep_prepare"), dict) or \
            value["promotion"] != "FORBIDDEN_UNTIL_SIGNED_RECEIPT":
        raise AuditError("capture contract status/policy is invalid")
    bindings = value["contract_bindings"]
    if not isinstance(bindings, dict) or set(bindings) != set(CAPTURE_CONTRACT_BINDING_PATHS):
        raise AuditError("capture contract bindings are incomplete or extra")
    for key, relative in CAPTURE_CONTRACT_BINDING_PATHS.items():
        item = bindings[key]
        if not isinstance(item, dict) or set(item) != {"path", "sha256"} or \
                item["path"] != relative or not SHA256_RE.fullmatch(str(item["sha256"])):
            raise AuditError("capture contract binding is invalid: {}".format(key))
        if repo_root is not None:
            path = _relative_source(repo_root, relative, "capture contract source")
            if path.is_symlink() or not path.is_file() or sha256_file(path) != item["sha256"]:
                raise AuditError("capture contract source hash drift: {}".format(relative))
    snapshot = value["source_snapshot"]
    expected_snapshot = {
        "schema": "glim_clean_room_r3_apt_source_snapshot_v2",
        "schema_path": CAPTURE_CONTRACT_BINDING_PATHS["source_schema"][
            "path"] if isinstance(CAPTURE_CONTRACT_BINDING_PATHS["source_schema"], dict)
        else CAPTURE_CONTRACT_BINDING_PATHS["source_schema"],
        "schema_sha256": bindings["source_schema"]["sha256"],
        "validator_path": CAPTURE_CONTRACT_BINDING_PATHS["source_validator"],
        "validator_sha256": bindings["source_validator"]["sha256"],
    }
    # Keep the expected projection explicit; the path map above is intentionally
    # plain strings so profile JSON cannot inject an alternate source location.
    if snapshot != expected_snapshot:
        raise AuditError("capture contract source snapshot binding is invalid")
    closure_path = Path(__file__).resolve().parents[1] / \
        CAPTURE_CONTRACT_BINDING_PATHS["closure_tool"]
    closure_spec = importlib.util.spec_from_file_location(
        "registration_plugin_dependency_closure_audit", closure_path)
    if closure_spec is None or closure_spec.loader is None:
        raise AuditError("dependency closure producer cannot be loaded")
    closure_module = importlib.util.module_from_spec(closure_spec)
    try:
        closure_spec.loader.exec_module(closure_module)
        expected_prepare = closure_module.rosdep_prepare_policy_projection()
    except Exception as error:
        raise AuditError(
            "rosdep prepare policy cannot be loaded from closure: {}".format(error)) from error
    if value["rosdep_prepare"] != expected_prepare:
        raise AuditError("capture contract rosdep prepare policy is not exact")
    expected_directories = list(closure_module.CAPTURE_DIRECTORY_RELATIVE_PATHS)
    if value["managed_directory_paths"] != expected_directories:
        raise AuditError("capture contract managed directory paths are not exact")
    return {"status": "PASS", "bindings": bindings}


def _validate_campaign_set_contract(campaign):
    """Validate the precommitted identity shared by all four release rows."""
    if not isinstance(campaign, dict) or set(campaign) != {
            "schema", "schema_version", "campaign_id", "rows",
            "row_set_sha256", "identity_sha256"}:
        raise AuditError("release_matrix campaign_set contract is missing or non-canonical")
    if (campaign.get("schema") != CAMPAIGN_SET_SCHEMA or
            campaign.get("schema_version") != CAMPAIGN_SET_VERSION or
            not isinstance(campaign.get("campaign_id"), str) or
            not CAMPAIGN_ID_RE.fullmatch(campaign["campaign_id"])):
        raise AuditError("release_matrix campaign_set identity is invalid")
    rows = campaign.get("rows")
    expected_rows = [
        {"distro": "humble", "dependency_leg": "absent"},
        {"distro": "humble", "dependency_leg": "present"},
        {"distro": "jazzy", "dependency_leg": "absent"},
        {"distro": "jazzy", "dependency_leg": "present"},
    ]
    if rows != expected_rows:
        raise AuditError("release_matrix campaign_set rows are not the exact four-leg set")
    row_set_sha256 = hashlib.sha256(_canonical(rows).encode("utf-8")).hexdigest()
    if campaign.get("row_set_sha256") != row_set_sha256:
        raise AuditError("release_matrix campaign_set row identity is stale")
    identity_payload = {
        "schema": campaign["schema"],
        "schema_version": campaign["schema_version"],
        "campaign_id": campaign["campaign_id"],
        "rows": rows,
        "row_set_sha256": row_set_sha256,
    }
    identity_sha256 = hashlib.sha256(_canonical(identity_payload).encode("utf-8")).hexdigest()
    if campaign.get("identity_sha256") != identity_sha256:
        raise AuditError("release_matrix campaign_set identity hash is stale")
    return campaign


def _validate_evidence_storage_contract(storage):
    """Validate the host filesystem identity required for release evidence.

    The launcher must never treat an unmounted removable-drive mountpoint as a
    writable evidence root.  Keep the expected mountpoint and block-device
    UUID in the profile so the host preflight can compare the live mount
    identity before reserving any directory.
    """
    if not isinstance(storage, dict):
        raise AuditError("release_matrix evidence_storage contract is missing")
    if storage.get("schema") != EVIDENCE_STORAGE_SCHEMA:
        raise AuditError("release_matrix evidence_storage schema is invalid")
    mountpoint = storage.get("mountpoint")
    if (not isinstance(mountpoint, str) or not mountpoint.startswith("/") or
            any(part in ("", ".", "..") for part in Path(mountpoint).parts) or
            any(character in mountpoint for character in ("\x00", "\n", "\r"))):
        raise AuditError("release_matrix evidence_storage mountpoint is unsafe")
    source_uuid = storage.get("source_uuid")
    if not isinstance(source_uuid, str) or not EVIDENCE_STORAGE_UUID_RE.fullmatch(source_uuid.lower()):
        raise AuditError("release_matrix evidence_storage source UUID is invalid")
    filesystem = storage.get("filesystem")
    if (not isinstance(filesystem, str) or not filesystem or
            not re.fullmatch(r"[A-Za-z0-9._+-]+", filesystem)):
        raise AuditError("release_matrix evidence_storage filesystem is invalid")
    label = storage.get("label")
    if (not isinstance(label, str) or not label or
            not re.fullmatch(r"[A-Za-z0-9._+-]+", label)):
        raise AuditError("release_matrix evidence_storage label is invalid")
    return {
        "schema": EVIDENCE_STORAGE_SCHEMA,
        "mountpoint": mountpoint,
        "source_uuid": source_uuid.lower(),
        "filesystem": filesystem,
        "label": label,
    }


def _release_matrix_static_report(profile):
    """Return a non-executing view used by the legacy audit receipt."""
    release = _validate_release_matrix_profile(profile)
    rows = []
    blocked = []
    for distro in release["distros"]:
        present = distro["legs"]["present"]
        if present.get("status") == "BLOCKED_MISSING_PIN":
            blocked.append(distro["name"])
        rows.append({
            "distro": distro["name"],
            "absent_leg": "REQUIRED",
            "present_leg": present.get("status"),
        })
    return {
        "schema": RELEASE_MATRIX_SCHEMA,
        "status": "BLOCKED_MISSING_PIN" if blocked else "READY_PINNED",
        "blocked_present_distros": blocked,
        "rows": rows,
        "source_manifest_required": True,
        "execution": "not_executed_by_host_audit",
    }


def _validated_repo_root(repo_root):
    """Return one canonical repository path suitable for scoped Git trust.

    Git's ``safe.directory`` value is passed as a command-local ``-c``
    setting.  Reject control characters before constructing that setting so
    a mounted or injected path can never become Git configuration syntax.
    """
    raw = os.fspath(repo_root)
    if any(character in raw for character in ("\x00", "\n", "\r")):
        raise AuditError("repository path contains unsafe control characters")
    try:
        canonical = Path(raw).resolve(strict=True)
    except (OSError, RuntimeError) as exc:
        raise AuditError("repository root cannot be canonicalized: {}".format(exc))
    if not canonical.is_dir():
        raise AuditError("repository root is not a directory: {}".format(canonical))
    return canonical


def _git_argv(repo_root, *args):
    """Build a Git argv with trust scoped to exactly one canonical root."""
    canonical = _validated_repo_root(repo_root)
    return ["git", "-c", "safe.directory={}".format(str(canonical)),
            "-C", str(canonical)] + list(args)


def _git_snapshot(repo_root):
    canonical = _validated_repo_root(repo_root)

    def run_git(*args):
        completed = subprocess.run(
            _git_argv(canonical, *args),
            stdout=subprocess.PIPE,
            stderr=subprocess.PIPE,
            universal_newlines=True,
            check=False,
        )
        if completed.returncode != 0:
            raise AuditError(
                "git {} failed: {}".format(
                    args[0], (completed.stderr or completed.stdout).strip()
                )
            )
        return completed.stdout

    status = run_git("status", "--porcelain=v1", "--untracked-files=all")
    diff = run_git("diff", "--binary", "HEAD", "--")
    untracked = []
    for line in run_git("ls-files", "--others", "--exclude-standard").splitlines():
        if not line:
            continue
        path = _relative_source(canonical, line, "untracked source")
        if path.is_symlink():
            untracked.append({"path": line, "kind": "symlink"})
        elif path.is_file():
            untracked.append({"path": line, "kind": "file", "sha256": sha256_file(path)})
        elif path.is_dir():
            untracked.append({"path": line, "kind": "directory"})
        else:
            raise AuditError("unsupported untracked entry: {}".format(path))
    dirty_components = {
        "status": status,
        "diff_sha256": hashlib.sha256(diff.encode("utf-8")).hexdigest(),
        "untracked": untracked,
    }
    return {
        "revision": run_git("rev-parse", "HEAD").strip(),
        "head_tree": run_git("rev-parse", "HEAD^{tree}").strip(),
        "dirty": bool(status),
        "dirty_tree_sha256": hashlib.sha256(
            _canonical(dirty_components).encode("utf-8")
        ).hexdigest(),
        "dirty_components": dirty_components,
    }


def _v2_source_snapshot_audit(repo_root, profile):
    """Reopen an optional v2 APT source snapshot with its strict validator.

    The current matrix profile has no captured v2 snapshot yet.  That state is
    reported explicitly rather than treating a missing capture as ready.  A
    future profile may opt in with a repository-relative snapshot root and
    manifest; both the composer and its source descriptors then become part of
    this read-only audit.
    """
    source = profile.get("source", {})
    config = source.get("v2_snapshot") if isinstance(source, dict) else None
    if config is None:
        return {
            "status": "NOT_CONFIGURED",
            "checked": False,
            "promotion": "NOT_AUTHORIZED",
        }
    required = {"root", "manifest", "repositories"}
    if not isinstance(config, dict) or set(config) != required:
        raise AuditError("source v2 snapshot configuration is incomplete or extra")
    root_value = config["root"]
    manifest_value = config["manifest"]
    if (not isinstance(root_value, str) or not os.path.isabs(root_value) or
            Path(root_value).as_posix() != root_value or
            not isinstance(manifest_value, str) or not manifest_value or
            os.path.isabs(manifest_value) or "\\" in manifest_value or
            any(part in ("", ".", "..") for part in manifest_value.split("/")) or
            Path(manifest_value).as_posix() != manifest_value):
        raise AuditError("source v2 snapshot path is not canonical")
    root = Path(root_value)
    manifest_path = root / manifest_value
    try:
        root_info = root.lstat()
        if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
            raise AuditError("source v2 snapshot root is not a directory")
    except OSError as exc:
        raise AuditError("source v2 snapshot root cannot be inspected: {}".format(exc))
    composer_path = repo_root / (
        "docker/benchmark_adapters/glim_clean_room/phase3d/r3/apt_allowlist_composer.py")
    try:
        if composer_path.is_symlink() or not composer_path.is_file():
            raise AuditError("v2 source composer is missing or is a symlink")
        spec = importlib.util.spec_from_file_location(
            "registration_plugin_r3_v2_source_composer", composer_path)
        if spec is None or spec.loader is None:
            raise AuditError("v2 source composer cannot be loaded")
        module = importlib.util.module_from_spec(spec)
        spec.loader.exec_module(module)
        repositories = config["repositories"]
        if not isinstance(repositories, list) or not repositories:
            raise AuditError("source v2 snapshot repositories are empty")
        _, manifest_sha256 = module._validate_source_snapshot(
            root, manifest_path, repositories)
        return {
            "status": "PASS_REOPENED_REVIEW_REQUIRED",
            "checked": True,
            "promotion": "NOT_AUTHORIZED",
            "root": str(root),
            "manifest": manifest_value,
            "manifest_sha256": manifest_sha256,
        }
    except AuditError:
        raise
    except (OSError, ValueError, TypeError) as exc:
        return {
            "status": "FAIL_CLOSED",
            "checked": True,
            "promotion": "NOT_AUTHORIZED",
            "root": str(root),
            "manifest": manifest_value,
            "reason": str(exc),
        }
    except Exception as exc:  # pragma: no cover - defensive validator boundary
        return {
            "status": "FAIL_CLOSED",
            "checked": True,
            "promotion": "NOT_AUTHORIZED",
            "root": str(root),
            "manifest": manifest_value,
            "reason": str(exc),
        }


def _source_manifest(repo_root, profile):
    entries = []
    mismatches = []
    files = profile.get("source", {}).get("files", [])
    if not files:
        raise AuditError("matrix profile has no current source files")
    for item in files:
        if not isinstance(item, dict):
            raise AuditError("source file entry is not an object")
        relative = item.get("path")
        expected = item.get("sha256")
        if not SHA256_RE.fullmatch(str(expected or "")):
            raise AuditError("source entry has invalid SHA-256: {}".format(relative))
        path = _relative_source(repo_root, relative, "source file")
        entry = {"path": relative, "expected_sha256": expected}
        try:
            if path.is_symlink():
                entry.update({"status": "REJECTED_SYMLINK", "actual_sha256": None})
                mismatches.append(relative)
            elif not path.is_file():
                entry.update({"status": "MISSING", "actual_sha256": None})
                mismatches.append(relative)
            else:
                actual = sha256_file(path)
                entry.update({
                    "status": "PASS" if actual == expected else "HASH_MISMATCH",
                    "actual_sha256": actual,
                    "size_bytes": path.stat().st_size,
                })
                if actual != expected:
                    mismatches.append(relative)
        except OSError as exc:
            entry.update({"status": "INSPECTION_ERROR", "error": str(exc)})
            mismatches.append(relative)
        entries.append(entry)
    projection = [{"path": e["path"], "sha256": e.get("actual_sha256")} for e in entries]
    v2_snapshot = _v2_source_snapshot_audit(repo_root, profile)
    if v2_snapshot["status"] == "FAIL_CLOSED":
        mismatches.append(v2_snapshot["manifest"])
    return {
        "status": "PASS" if not mismatches else "FAIL_CLOSED",
        "hash_kind": "relative_path_content_sha256_v1",
        "files": entries,
        "manifest_sha256": hashlib.sha256(_canonical(projection).encode("utf-8")).hexdigest(),
        "mismatches": mismatches,
        "v2_snapshot": v2_snapshot,
    }


def _package_version(prefix, package_name):
    package_xml = prefix / "share" / package_name / "package.xml"
    if package_xml.is_symlink() or not package_xml.is_file():
        return {"status": "MISSING", "version": None, "path": str(package_xml)}
    try:
        root = ET.parse(str(package_xml)).getroot()
        version = root.findtext("version")
    except (OSError, ET.ParseError) as exc:
        return {"status": "INVALID", "version": None, "path": str(package_xml), "error": str(exc)}
    return {
        "status": "PASS" if version else "INVALID",
        "version": version.strip() if version else None,
        "path": str(package_xml),
        "sha256": sha256_file(package_xml),
    }


def _tool_version(tool):
    executable = shutil.which(tool)
    if executable is None:
        return {"status": "MISSING", "path": None, "version": None}
    # colcon intentionally has no ``--version`` or ``version`` verb.  Query
    # the installed Python distribution instead of recording its normal help
    # error as a toolchain failure.
    if tool == "colcon":
        versions = []
        for distribution in ("colcon-core", "colcon-common-extensions"):
            try:
                versions.append(
                    "{}=={}".format(
                        distribution, importlib.metadata.version(distribution)
                    )
                )
            except importlib.metadata.PackageNotFoundError:
                continue
        return {
            "status": "PASS" if versions else "ERROR",
            "path": executable,
            "version": ", ".join(versions) if versions else None,
            "version_probe": "python_distribution_metadata",
        }
    completed = subprocess.run(
        [executable, "--version"],
        stdout=subprocess.PIPE,
        stderr=subprocess.STDOUT,
        universal_newlines=True,
        check=False,
    )
    first = (completed.stdout or "").strip().splitlines()
    return {
        "status": "PASS" if completed.returncode == 0 else "ERROR",
        "path": executable,
        "version": first[0] if first else None,
        "returncode": completed.returncode,
    }


def _optional_dependency(prefix, dependency):
    names = dependency.get("names", [dependency.get("name")])
    markers = []
    for name in names:
        if not name:
            continue
        for candidate in (
            prefix / "share" / name,
            prefix / "include" / name,
            Path("/usr/include") / name,
            Path("/usr/local/include") / name,
        ):
            if candidate.exists() and not candidate.is_symlink():
                markers.append(str(candidate))
    return {
        "name": dependency.get("name"),
        "status": "PRESENT" if markers else "ABSENT",
        "markers": sorted(set(markers)),
        "required": bool(dependency.get("required", False)),
        "source_policy": dependency.get("source_policy", "optional").strip(),
    }


def _install_probe(repo_root, row):
    configured = Path(row.get("install_prefix", "install"))
    prefix = configured if configured.is_absolute() else repo_root / configured
    result = {
        "prefix": str(prefix),
        "status": "NOT_PRESENT",
        "artifacts": [],
        "tree_hash_kind": "configured_probe_manifest_v1",
        "tree_sha256": None,
    }
    if not prefix.exists() or prefix.is_symlink() or not prefix.is_dir():
        return result
    result["status"] = "PASS"
    projection = []
    for relative in row.get("install_probes", []):
        path = prefix / relative
        if path.is_symlink():
            target = os.readlink(str(path))
            result["artifacts"].append({
                "path": str(path),
                "status": "SYMLINK_OVERLAY",
                "target": target,
            })
            projection.append({
                "path": relative,
                "status": "SYMLINK_OVERLAY",
                "target": target,
            })
        elif path.is_file():
            digest = sha256_file(path)
            size = path.stat().st_size
            result["artifacts"].append({
                "path": str(path),
                "status": "REGULAR",
                "sha256": digest,
                "size_bytes": size,
            })
            projection.append({
                "path": relative,
                "status": "REGULAR",
                "sha256": digest,
                "size_bytes": size,
            })
        else:
            result["artifacts"].append({"path": str(path), "status": "MISSING"})
            projection.append({"path": relative, "status": "MISSING"})
    result["tree_sha256"] = hashlib.sha256(
        _canonical(projection).encode("utf-8")
    ).hexdigest()
    if any(item["status"] == "SYMLINK_OVERLAY" for item in result["artifacts"]):
        result["status"] = "SYMLINK_OVERLAY_NOT_INDEPENDENT"
    return result


def _image_probe(row, inspect_images):
    image = row.get("image", {})
    digest = image.get("digest")
    if not isinstance(digest, str) or not re.fullmatch(r"sha256:[0-9a-f]{64}", digest):
        return {"status": "INVALID_PROFILE_DIGEST", "digest": digest}
    result = {
        "tag": image.get("tag"),
        "digest": digest,
        "status": "NOT_PROBED",
        "network": "not_used",
        "build": False,
        "pull": False,
    }
    if not inspect_images:
        result["reason"] = "read_only_image_probe_not_requested"
        return result
    docker = shutil.which("docker")
    if docker is None:
        result.update({"status": "NO_GO", "reason": "docker_not_installed"})
        return result
    completed = subprocess.run(
        [docker, "image", "inspect", digest],
        stdout=subprocess.PIPE,
        stderr=subprocess.PIPE,
        universal_newlines=True,
        check=False,
    )
    result["returncode"] = completed.returncode
    if completed.returncode != 0:
        result.update({"status": "NO_GO", "reason": "pinned_image_not_present"})
        return result
    try:
        inspected = json.loads(completed.stdout)
    except ValueError:
        result.update({"status": "NO_GO", "reason": "docker_inspect_not_json"})
        return result
    if not isinstance(inspected, list) or len(inspected) != 1:
        result.update({"status": "NO_GO", "reason": "docker_inspect_ambiguous"})
        return result
    actual_id = inspected[0].get("Id")
    result["actual_id"] = actual_id
    result["repo_digests"] = inspected[0].get("RepoDigests", [])
    result["os"] = inspected[0].get("Os")
    result["architecture"] = inspected[0].get("Architecture")
    identity_ok = (
        actual_id == digest
        and any(str(value).endswith("@" + digest) for value in result["repo_digests"])
        and result["os"] == "linux"
        and result["architecture"] == "amd64"
    )
    result["status"] = "PASS" if identity_ok else "NO_GO"
    if not identity_ok:
        result["reason"] = "image_identity_mismatch"
    return result


def _consumer_static_contract(repo_root, profile):
    relative = profile.get("consumer_contract", {}).get("script")
    path = _relative_source(repo_root, relative, "consumer contract script")
    if path.is_symlink() or not path.is_file():
        return {"status": "NO_GO", "path": str(path)}
    text = path.read_text(encoding="utf-8")
    required = profile["consumer_contract"].get("required_markers", [])
    missing = [marker for marker in required if marker not in text]
    return {
        "status": "PASS" if not missing else "FAIL_CLOSED",
        "path": relative,
        "sha256": sha256_file(path),
        "declared_cxx_standard": profile["consumer_contract"].get("cxx_standard"),
        "required_markers_missing": missing,
        "runtime_evidence": "not_executed_by_audit",
    }


def _historical_audit(repo_root, profile):
    reports = []
    for item in profile.get("historical", []):
        source_document = item.get("source_document")
        source_report = None
        if source_document:
            source_path = _relative_source(repo_root, source_document, "historical source document")
            if source_path.is_file() and not source_path.is_symlink():
                source_report = {"path": source_document, "sha256": sha256_file(source_path)}
            else:
                source_report = {"path": source_document, "status": "MISSING"}
        paths = []
        for value in item.get("receipt_paths", []):
            path = _absolute(value, "historical receipt path")
            row = {"path": str(path)}
            if path.is_symlink():
                row["status"] = "REJECTED_SYMLINK"
            elif path.is_file():
                actual = sha256_file(path)
                expected = item.get("expected_sha256", {}).get(str(path))
                mode = stat.S_IMODE(path.stat().st_mode)
                row.update({
                    "status": "HASH_PASS" if expected in (None, actual) else "HASH_MISMATCH",
                    "sha256": actual,
                    "mode": mode,
                    "immutability": "PASS" if mode == 0o444 else "MUTABLE_MODE",
                })
                if mode != 0o444:
                    row["status"] = "MUTABLE_MODE"
                sidecar = Path(str(path) + ".sha256")
                if not sidecar.is_file() or sidecar.is_symlink():
                    row["sidecar_status"] = "MISSING_OR_INVALID"
                else:
                    sidecar_text = sidecar.read_text(encoding="utf-8").strip().split()
                    sidecar_ok = bool(sidecar_text) and sidecar_text[0] == actual
                    sidecar_mode = stat.S_IMODE(sidecar.stat().st_mode)
                    row["sidecar_mode"] = sidecar_mode
                    if sidecar_mode != 0o444:
                        row["sidecar_status"] = "MUTABLE_MODE"
                    else:
                        row["sidecar_status"] = "PASS" if sidecar_ok else "HASH_MISMATCH"
            else:
                row["status"] = "MISSING_OR_NOT_REGULAR"
            paths.append(row)
        if not paths:
            status = "SUPERSEDED_UNBOUND_DOCUMENT_CLAIM"
            reason = "documentation claim has no immutable receipt path"
        elif all(
            row.get("status") == "HASH_PASS"
            and row.get("immutability") == "PASS"
            and row.get("sidecar_status") == "PASS"
            and row.get("sidecar_mode") == 0o444
            for row in paths
        ):
            status = "SUPERSEDED_CURRENT_SOURCE_NOT_BOUND"
            reason = "historical receipt does not bind the current source manifest"
        else:
            status = "SUPERSEDED_ARTIFACT_OR_SIDECAR_INVALID"
            reason = "historical artifact is missing, mutable, or lacks a valid sidecar"
        reports.append({
            "id": item.get("id"),
            "claim": item.get("claim"),
            "status": status,
            "reason": reason,
            "source_document": source_report,
            "receipts": paths,
        })
    return reports


def _distro_audit(repo_root, row, source_report, inspect_images):
    name = row.get("name")
    prefix = _absolute(row.get("prefix", "/opt/ros/{}".format(name)), "ROS prefix")
    setup = prefix / "setup.bash"
    result = {
        "name": name,
        "prefix": str(prefix),
        "setup": str(setup),
        "source_manifest_sha256": source_report["manifest_sha256"],
        "source_status": source_report["status"],
        "image": _image_probe(row, inspect_images),
        "install": _install_probe(repo_root, row),
        "toolchain": {},
        "packages": {},
        "optional_dependencies": [],
        "execution": {
            "network": "none_not_started",
            "container": "not_started",
            "repository_mount": {
                "required_for_container_gate": "read_only",
                "status": "NOT_APPLICABLE_HOST_ONLY",
            },
            "build": False,
            "formal": False,
        },
    }
    if prefix.is_symlink() or not prefix.is_dir() or setup.is_symlink() or not setup.is_file():
        result["optional_dependencies"] = [
            {
                "name": dependency.get("name"),
                "status": "NOT_PROBED",
                "markers": [],
                "required": bool(dependency.get("required", False)),
                "source_policy": dependency.get("source_policy", "optional").strip(),
                "reason": "ROS_SETUP_MISSING_OR_SYMLINK",
            }
            for dependency in row.get("optional_dependencies", [])
        ]
        result.update({"status": "NO_GO", "reason": "ROS_SETUP_MISSING_OR_SYMLINK"})
        return result
    for package in ("rclcpp", "pluginlib", "class_loader", "pcl_conversions"):
        result["packages"][package] = _package_version(prefix, package)
    for tool in ("g++", "cmake", "python3", "colcon"):
        result["toolchain"][tool] = _tool_version(tool)
    for dependency in row.get("optional_dependencies", []):
        result["optional_dependencies"].append(_optional_dependency(prefix, dependency))
    pluginlib_ok = result["packages"]["pluginlib"]["status"] == "PASS"
    rclcpp_ok = result["packages"]["rclcpp"]["status"] == "PASS"
    if source_report["status"] != "PASS":
        result.update({"status": "FAIL_CLOSED", "reason": "CURRENT_SOURCE_HASH_MISMATCH"})
    elif not (pluginlib_ok and rclcpp_ok):
        result.update({"status": "NO_GO", "reason": "required_ros_package_missing"})
    else:
        result.update({
            "status": "PASS_HOST_TOOLCHAIN_ONLY",
            "reason": "prefix is present; no container/build was executed",
        })
    return result


def audit(repo_root, profile, distros=None, inspect_images=False):
    repo_root = _absolute(repo_root, "repository root")
    if repo_root.is_symlink() or not repo_root.is_dir():
        raise AuditError("repository root must be a non-symlink directory")
    release = _validate_release_matrix_profile(profile)
    capture_contract = _validate_capture_contract(
        repo_root, release["dependency_closure"]["capture_contract"])
    source_report = _source_manifest(repo_root, profile)
    if source_report["status"] == "PASS":
        workflow_report = _validate_workflow_image_contract(repo_root, profile)
    else:
        workflow_report = {
            "status": "NOT_VALIDATED_SOURCE_MISMATCH",
            "reason": "current source manifest must pass before cross-file image validation",
        }
    git_report = _git_snapshot(repo_root)
    consumer = _consumer_static_contract(repo_root, profile)
    selected = profile.get("distros", [])
    if distros:
        wanted = set(distros)
        selected = [row for row in selected if row.get("name") in wanted]
        if len(selected) != len(wanted):
            unknown = sorted(wanted - {row.get("name") for row in selected})
            raise AuditError("profile has no requested distro: {}".format(", ".join(unknown)))
    distro_reports = [
        _distro_audit(repo_root, row, source_report, inspect_images) for row in selected
    ]
    release_matrix = _release_matrix_static_report(profile)
    release_matrix["capture_contract"] = capture_contract
    history = _historical_audit(repo_root, profile)
    no_go = [row["name"] for row in distro_reports if row["status"] == "NO_GO"]
    if source_report["status"] != "PASS" or consumer["status"] != "PASS":
        status = "FAIL_CLOSED"
    elif no_go:
        status = "NO_GO:" + ",".join(no_go)
    else:
        status = "PASS_HOST_TOOLCHAIN_ONLY"
    return {
        "schema": PROFILE_SCHEMA,
        "schema_version": SCHEMA_VERSION,
        "profile_id": profile.get("profile_id"),
        "status": status,
        "current_source": source_report,
        "git": git_report,
        "consumer_contract": consumer,
        "workflow_images": workflow_report,
        "release_matrix": release_matrix,
        "capture_contract": capture_contract,
        "distros": distro_reports,
        "historical_evidence": history,
        "historical_passes_promoted": False,
        "superseded_historical_count": len(history),
        "safety": {
            "network_used": False,
            "docker_build": False,
            "docker_pull": False,
            "docker_run": False,
            "bag_opened": False,
            "ground_truth_opened": False,
            "scorer_opened": False,
            "map_written": False,
            "formal_replay": False,
        },
    }


def _exclusive_write(path, payload):
    path = _absolute(path, "receipt output")
    path.parent.mkdir(parents=True, exist_ok=True)
    if path.is_symlink() or path.exists():
        raise AuditError("receipt output would overwrite or follow a symlink: {}".format(path))
    temporary = path.parent / ("." + path.name + "." + uuid.uuid4().hex + ".part")
    data = payload
    descriptor = None
    try:
        descriptor = os.open(str(temporary), os.O_WRONLY | os.O_CREAT | os.O_EXCL, 0o600)
        with os.fdopen(descriptor, "wb") as stream:
            descriptor = None
            stream.write(data)
            stream.flush()
            os.fsync(stream.fileno())
        os.link(str(temporary), str(path))
        os.chmod(str(path), 0o444)
    except FileExistsError:
        raise AuditError("receipt output would overwrite an existing file: {}".format(path))
    finally:
        if descriptor is not None:
            os.close(descriptor)
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass
    return path


def seal_receipt(path, report):
    data = (_canonical(report) + "\n").encode("utf-8")
    receipt = _exclusive_write(path, data)
    digest = hashlib.sha256(data).hexdigest()
    sidecar = Path(str(receipt) + ".sha256")
    _exclusive_write(sidecar, ("{}  {}\n".format(digest, receipt.name)).encode("ascii"))
    return str(receipt), str(sidecar), digest


def main(argv=None):
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument("--repo-root", default=str(Path(__file__).resolve().parents[1]))
    parser.add_argument("--profile", default=str(DEFAULT_PROFILE))
    parser.add_argument("--distro", action="append", dest="distros")
    parser.add_argument("--output", help="fresh immutable JSON receipt path")
    parser.add_argument(
        "--inspect-images",
        action="store_true",
        help="read-only docker image inspect using profile-pinned digests",
    )
    args = parser.parse_args(argv)
    try:
        profile_arg = Path(args.profile)
        repo_arg = Path(args.repo_root)
        profile, profile_path = _load_profile(
            profile_arg if profile_arg.is_absolute() else Path.cwd() / profile_arg
        )
        report = audit(
            repo_arg if repo_arg.is_absolute() else Path.cwd() / repo_arg,
            profile,
            args.distros,
            args.inspect_images,
        )
        report["profile"] = {
            "path": str(profile_path),
            "sha256": sha256_file(profile_path),
        }
        if args.output:
            receipt, sidecar, digest = seal_receipt(args.output, report)
            report["sealed_receipt"] = {
                "path": receipt,
                "sidecar": sidecar,
                "sha256": digest,
            }
        print(json.dumps(report, sort_keys=True, indent=2))
        return 0 if report["status"] == "PASS_HOST_TOOLCHAIN_ONLY" else 2
    except (AuditError, OSError) as exc:
        print("registration plugin matrix audit: FAIL_CLOSED: {}".format(exc), file=sys.stderr)
        return 2


if __name__ == "__main__":
    sys.exit(main())

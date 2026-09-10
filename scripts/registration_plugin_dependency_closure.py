#!/usr/bin/env python3
"""Compose and validate a sealed dependency closure for release-plugin legs.

The registration-plugin release runner is intentionally split into two
phases.  A provisioning worker may capture APT/rosdep bytes while a pinned
container is connected; this module only consumes that host-owned capture
packet.  It never runs a resolver, downloads a package, invokes Docker, or
changes a profile.  Every input is reopened and bound to its bytes before a
closure is written.  The resulting document is always
``REVIEW_REQUIRED``: a separate custodian review and signature receipt must
promote it to the runtime ``PASS`` state.

The apt/deb part deliberately reuses the existing GLIM clean-room composer.
That implementation parses ``apt-get --print-uris`` as a locator, then
requires downloaded bytes to match the SHA-256 in a Release-bound Packages
index.  An apt URI MD5 is never used as an authenticity root here.
"""

from __future__ import annotations

import argparse
import copy
import hashlib
import importlib.util
import json
import os
import re
import stat
import subprocess
import sys
from pathlib import Path
from typing import Any, Callable, Mapping
from urllib.parse import urlsplit


ROOT = Path(__file__).resolve().parents[1]
APT_COMPOSER_PATH = ROOT / (
    "docker/benchmark_adapters/glim_clean_room/phase3d/r3/"
    "apt_allowlist_composer.py"
)
CAPTURE_EXECUTOR_SCHEMA_PATH = ROOT / (
    "configs/slam_benchmark_profiles/"
    "registration_plugin_dependency_capture_executor_v1.schema.json"
)
CAPTURE_EXECUTOR_SCHEMA_RELATIVE = (
    "configs/slam_benchmark_profiles/"
    "registration_plugin_dependency_capture_executor_v1.schema.json"
)
CAPTURE_EXECUTOR_TOOL_RELATIVE = (
    "scripts/capture_registration_plugin_dependency_closure.py"
)
CAPTURE_DIRECTORY_TOOL_RELATIVE = "scripts/registration_plugin_evidence_directory.py"
CAPTURE_DIRECTORY_SCHEMA_RELATIVE = (
    "configs/slam_benchmark_profiles/"
    "registration_plugin_capture_directory_contract_v1.schema.json"
)
SOURCE_MANIFEST_TOOL_RELATIVE = (
    "docker/benchmark_adapters/glim_clean_room/phase3d/r3/apt_allowlist_composer.py"
)
SOURCE_MANIFEST_SCHEMA_RELATIVE = (
    "configs/slam_benchmark_profiles/"
    "registration_plugin_apt_source_snapshot_v2.schema.json"
)
CAPTURE_EXECUTOR_TOOL_PATH = ROOT / CAPTURE_EXECUTOR_TOOL_RELATIVE
CAPTURE_DIRECTORY_TOOL_PATH = ROOT / CAPTURE_DIRECTORY_TOOL_RELATIVE
CAPTURE_DIRECTORY_SCHEMA_PATH = ROOT / CAPTURE_DIRECTORY_SCHEMA_RELATIVE
SOURCE_MANIFEST_TOOL_PATH = ROOT / SOURCE_MANIFEST_TOOL_RELATIVE
SOURCE_MANIFEST_SCHEMA_PATH = ROOT / SOURCE_MANIFEST_SCHEMA_RELATIVE
ROSDEP_PREPARE_TOOL_RELATIVE = "scripts/prepare_registration_plugin_rosdep.py"
ROSDEP_PREPARE_SCHEMA_RELATIVE = (
    "configs/slam_benchmark_profiles/registration_plugin_rosdep_prepare_v1.schema.json"
)
ROSDEP_PREPARE_TOOL_PATH = ROOT / ROSDEP_PREPARE_TOOL_RELATIVE
ROSDEP_PREPARE_SCHEMA_PATH = ROOT / ROSDEP_PREPARE_SCHEMA_RELATIVE
ROSDEP_DISCOVERY_HELPER_RELATIVE = (
    "scripts/capture_registration_plugin_rosdep_discovery.py"
)
ROSDEP_DISCOVERY_HELPER_PATH = ROOT / ROSDEP_DISCOVERY_HELPER_RELATIVE
ROSDEP_PREPARE_INPUT_RELATIVE = "rosdep-prepare/input"
ROSDEP_PREPARE_CONTAINER_ROOT = "/opt/registration-plugin/prepare"
ROSDEP_PREPARE_CONTAINER_TOOL = (
    ROSDEP_PREPARE_CONTAINER_ROOT + "/prepare_registration_plugin_rosdep.py"
)
ROSDEP_PREPARE_CONTAINER_SCHEMA = (
    ROSDEP_PREPARE_CONTAINER_ROOT + "/registration_plugin_rosdep_prepare_v1.schema.json"
)
ROSDEP_PREPARE_CONTAINER_DISCOVERY = (
    ROSDEP_PREPARE_CONTAINER_ROOT + "/capture_registration_plugin_rosdep_discovery.py"
)
ROSDEP_PREPARE_CONTAINER_DISCOVERY_ROOT = "/opt/registration-plugin/discovery"
ROSDEP_PREPARE_CONTAINER_PROFILE = (
    ROSDEP_PREPARE_CONTAINER_ROOT + "/consumer-profile.json"
)
ROSDEP_PREPARE_INPUT_SCHEMA = "registration-plugin-rosdep-prepare-input-v1"

# The capture executor records concrete host paths and pre/post identities in
# its plan.  This static projection fixes the only acceptable order, target,
# bind type, and access mode for those concrete entries in profile/closure
# contracts without embedding a per-run output-root path here.
ROSDEP_PREPARE_MOUNT_POLICY = {
    "schema": "registration-plugin-rosdep-prepare-mount-policy-v1",
    "schema_version": 1,
    "identity": "ordered-host-source-container-target-bindings-v1",
    "entries": [
        {"role": "input_bundle", "source": "plan-output-relative:rosdep-prepare/input",
         "target": "/opt/registration-plugin/prepare", "read_only": True,
         "type": "bind", "noexec": False},
        {"role": "discovery_root", "source": "discovery.root",
         "target": ROSDEP_PREPARE_CONTAINER_DISCOVERY_ROOT, "read_only": True,
         "type": "bind", "noexec": False},
        {"role": "discovery_files", "source": "discovery.root-relative:artifacts/rosdistro/files",
         "target": "discovery.expected_local_source_root", "read_only": True,
         "type": "bind", "noexec": False},
        {"role": "rosdistro_work", "source": "plan-output-relative:rosdep-prepare/work/rosdistro",
         "target": "/opt/registration-plugin/rosdistro", "read_only": False,
         "type": "bind", "noexec": False},
        {"role": "rosdep_cache_work", "source": "plan-output-relative:rosdep-prepare/work/rosdep-cache",
         "target": "discovery.expected_rosdep_cache", "read_only": False,
         "type": "bind", "noexec": False},
        {"role": "ros_home_work", "source": "plan-output-relative:rosdep-prepare/work/ros-home",
         "target": "discovery.expected_ros_home", "read_only": False,
         "type": "bind", "noexec": False},
        {"role": "source_output", "source": "plan-output-relative:rosdep-prepare/artifacts/rosdep/sources.list.d",
         "target": "discovery.expected_source_list_dir", "read_only": False,
         "type": "bind", "noexec": False},
        {"role": "cache_output", "source": "plan-output-relative:rosdep-prepare/artifacts/rosdep/cache",
         "target": "/opt/registration-plugin/rosdep-output-cache", "read_only": False,
         "type": "bind", "noexec": False},
    ],
}

SCHEMA = "registration-plugin-dependency-closure-v1"
CAPTURE_SCHEMA = "registration-plugin-dependency-capture-input-v1"
SCHEMA_VERSION = 1
REVIEW_STATUS = "REVIEW_REQUIRED"
RUNTIME_STATUS = "PASS"
DISTROS = frozenset(("humble", "jazzy"))
LEGS = frozenset(("absent", "present"))
ROLES = ("build", "runtime")
PLATFORM = {"os": "linux", "architecture": "amd64"}
MAX_JSON_BYTES = 16 * 1024 * 1024
MAX_LOG_BYTES = 8 * 1024 * 1024
MAX_TEXT_BYTES = 64 * 1024 * 1024
MAX_DECODED_PACKAGES_BYTES = 256 * 1024 * 1024
MAX_DEB_BYTES = 512 * 1024 * 1024
# A signed Debian Release file may describe large Contents indexes even when
# the capture only materializes a much smaller Packages index.  This bound is
# for the decimal size declared inside signed metadata only; it is never an
# allocation or download limit.  The latter remain bounded by MAX_TEXT_BYTES
# and MAX_DEB_BYTES below.
MAX_RELEASE_DECLARED_BYTES = 16 * 1024 * 1024 * 1024
RELEASE_DECLARATION_POLICY = {
    "schema": "registration-plugin-release-declaration-size-policy-v1",
    "schema_version": 1,
    "max_declared_bytes": MAX_RELEASE_DECLARED_BYTES,
    "decimal_grammar": "0|[1-9][0-9]*",
    "zero_allowed": True,
}
EMPTY_PACKAGES_SHA256 = (
    "e3b0c44298fc1c149afbf4c8996fb92427ae41e4649b934ca495991b7852b855"
)
SIGNED_EMPTY_PACKAGES_POLICY = {
    "schema": "registration-plugin-signed-empty-packages-policy-v1",
    "schema_version": 1,
    "enabled": True,
    "signed_empty": True,
    "decoded_sha256": EMPTY_PACKAGES_SHA256,
    "decoded_bytes": 0,
    "allowed_formats": ["xz"],
    "xz_stream_count": 1,
    "xz_compressed_bytes": 32,
    "padding": "forbidden",
    "trailing_data": "reject",
    "binding": "signed-release-pair-compressed-bytes-and-exact-empty-decode",
}
PACKAGES_FORMAT_POLICY = {
    "schema": "registration-plugin-packages-format-policy-v1",
    "schema_version": 1,
    "selection_order": ["xz", "gz", "plain"],
    "allowed_formats": ["xz", "gz", "plain"],
    "compressed_max_bytes": MAX_TEXT_BYTES,
    "decoded_max_bytes": MAX_DECODED_PACKAGES_BYTES,
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
    "signed_empty_packages_policy": SIGNED_EMPTY_PACKAGES_POLICY,
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
    "shell": False,
    "network": "none",
    "version_timeout_seconds": 30,
    "inventory_timeout_seconds": 120,
    "metadata_residue_schema": "registration-plugin-gpg-inventory-metadata-v1",
    "metadata_residue_policy": "exact-pubring.kbx-trustdb.gpg-supported-show-only",
    "metadata_residue_names": ["pubring.kbx", "trustdb.gpg"],
    "metadata_residue_mode": 384,
    "metadata_residue_max_bytes": 16777216,
    "metadata_residue_retention": "descriptor-only-cleaned",
    "failure_status": "REVIEW_REQUIRED",
}
MAX_TREE_FILES = 20000
MAX_PHASES = 32
MAX_URL_LENGTH = 2048
SHA256_RE = re.compile(r"^[0-9a-f]{64}$")
IMAGE_DIGEST_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
SAFE_RELATIVE_RE = re.compile(r"^[^/\\.][^\\]*$")
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
VERIFICATION_CHAIN = {
    "release_signature": "gpgv_evidence_required_before_closure",
    "release_to_packages": "signed_release_sha256_size_exact",
    "packages_to_deb": "packages_sha256_size_and_dpkg_metadata_exact",
}
CLOSURE_FILENAME = "registration-plugin-dependency-closure.json"
CLOSURE_FILE_ALLOWLIST = frozenset((
    CLOSURE_FILENAME,
    CLOSURE_FILENAME + ".sha256",
    "apt/apt-allowlist-generation-proposal.json",
    "apt/apt-allowlist-generation-proposal.json.sha256",
    "apt/package-allowlist.json",
    "apt/package-allowlist.json.sha256",
    "apt/apt-acquisition-ledger.json",
    "apt/apt-acquisition-ledger.json.sha256",
))
CLOSURE_DIRECTORY_ALLOWLIST = frozenset(("apt",))
APT_FILE_ALLOWLIST = frozenset((
    "apt-allowlist-generation-proposal.json",
    "apt-allowlist-generation-proposal.json.sha256",
    "package-allowlist.json",
    "package-allowlist.json.sha256",
    "apt-acquisition-ledger.json",
    "apt-acquisition-ledger.json.sha256",
))
CAPTURE_CONTRACT_SCHEMA = "registration-plugin-dependency-capture-integration-v1"
CAPTURE_CONTRACT_SCHEMA_VERSION = 1
CAPTURE_REQUIRED_ARTIFACTS = (
    "capture_input", "base_status", "apt_source_capture", "dependency_capture",
    "rosdep_prepare",
)
CAPTURE_ARTIFACT_POLICY = "exact_existing_sidecar_sealed_allowlist_v1"
CAPTURE_DIRECTORY_POLICY = "host_precreated_0700_owner_bound_nofollow_v1"
# This is the static expected set for the outer capture directory contract.
# In particular, the nested Docker bind target below must already exist on
# the host; Docker must not create it while mounting the read-only discovery
# files subtree.
CAPTURE_DIRECTORY_RELATIVE_PATHS = (
    "logs",
    "gpg-inventory-work",
    "restore-owner-references",
    "restore-owner-references/build-partial",
    "restore-owner-references/runtime-partial",
    "apt-source", "apt-source/etc", "apt-source/etc/apt",
    "apt-source/etc/apt/sources.list.d", "apt-source/indices",
    "apt-keyrings", "apt-index", "raw-indexes", "signatures",
    "captured-debs", "captured-debs/build", "captured-debs/runtime",
    "rosdep", "rosdep/cache", "licenses", "licenses/build", "licenses/runtime",
    "debs-build", "debs-build/partial",
    "debs-runtime", "debs-runtime/partial",
    "rosdep-prepare", "rosdep-prepare/input",
    "rosdep-prepare/commands", "rosdep-prepare/artifacts",
    "rosdep-prepare/artifacts/rosdep",
    "rosdep-prepare/artifacts/rosdep/sources.list.d",
    "rosdep-prepare/artifacts/rosdep/cache",
    "rosdep-prepare/work", "rosdep-prepare/work/rosdistro",
    "rosdep-prepare/work/rosdistro/files",
    "rosdep-prepare/work/rosdistro/sources.list.d",
    "rosdep-prepare/work/rosdep-cache", "rosdep-prepare/work/ros-home",
    "rosdep-prepare/work/ros-home/rosdep",
    "rosdep-prepare/work/ros-home/rosdep/meta.cache",
    "rosdep-prepare/work/ros-home/rosdep/sources.cache",
)
APT_TRANSIENT_OWNER_POLICY = {
    "schema": "registration-plugin-apt-transient-owner-policy-v2",
    "schema_version": 2,
    "identities": [
        {"distro": "humble", "name": "_apt", "uid": 100, "gid": 0},
        {"distro": "jazzy", "name": "_apt", "uid": 42, "gid": 0},
    ],
    "identity_evidence": "profile-distro-image-digest-and-base-status/passwd",
    "binding": "distro-profile-image-digest",
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
    "transient_owner": dict(APT_TRANSIENT_OWNER_POLICY),
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
FORBIDDEN_COMMAND_TOKENS = frozenset((
    "bash", "sh", "zsh", "-c", "-lc", "curl", "wget", "nc", "socat",
    "ssh", "scp", "git", "pip", "pip3",
))
EXPECTED_PHASES = (
    "image_inspect",
    "preexisting_container_check",
    "container_start",
    "apt_update",
    "base_status_snapshot",
    "apt_source_snapshot",
    "resolver_build",
    "download_build",
    "restore_build_partial_owner",
    "resolver_runtime",
    "download_runtime",
    "restore_runtime_partial_owner",
    "dependency_install",
    "network_disconnect",
    "rosdep_prepare",
    "rosdep_resolution",
    "dependency_capture",
    "archive_prefetch",
    "post_disconnect_inspect",
)
LEGACY_EXPECTED_PHASES = (
    "image_inspect",
    "preexisting_container_check",
    "container_start",
    "apt_update",
    "base_status_snapshot",
    "apt_source_snapshot",
    "resolver_build",
    "download_build",
    "resolver_runtime",
    "download_runtime",
    "dependency_install",
    "rosdep_resolution",
    "dependency_capture",
    "archive_prefetch",
    "network_disconnect",
    "post_disconnect_inspect",
)
# Captures produced by the 17-phase contract remain readable as historical
# packets.  New packets use EXPECTED_PHASES and therefore include the two
# explicit APT partial-owner restore phases.
PRE_RESTORE_EXPECTED_PHASES = (
    "image_inspect", "preexisting_container_check", "container_start", "apt_update",
    "base_status_snapshot", "apt_source_snapshot", "resolver_build", "download_build",
    "resolver_runtime", "download_runtime", "dependency_install",
    "network_disconnect", "rosdep_prepare", "rosdep_resolution",
    "dependency_capture", "archive_prefetch", "post_disconnect_inspect",
)
PHASE_NETWORK = {
    "image_inspect": False,
    "preexisting_container_check": False,
    "container_start": True,
    "apt_update": True,
    "base_status_snapshot": False,
    "apt_source_snapshot": False,
    "resolver_build": True,
    "download_build": True,
    "restore_build_partial_owner": False,
    "resolver_runtime": True,
    "download_runtime": True,
    "restore_runtime_partial_owner": False,
    "dependency_install": True,
    "network_disconnect": False,
    "rosdep_prepare": False,
    "rosdep_resolution": False,
    "dependency_capture": False,
    "archive_prefetch": None,
    "post_disconnect_inspect": False,
}


class ClosureError(ValueError):
    """Raised whenever a dependency closure cannot be trusted."""

    def __init__(self, kind: str, message: str):
        super().__init__(message)
        self.kind = kind


def _load_apt_composer():
    spec = importlib.util.spec_from_file_location(
        "registration_plugin_apt_allowlist_composer", APT_COMPOSER_PATH
    )
    if spec is None or spec.loader is None:
        raise ClosureError("APT_COMPOSER_MISSING", str(APT_COMPOSER_PATH))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


APT = _load_apt_composer()
EXCLUDE_DOWNLOAD_HOUSEKEEPING = APT._exclude_download_housekeeping
APT_SOURCE_SCHEMA_V2 = APT.SOURCE_SCHEMA_V2


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def canonical_hash(value: Any, excluded: str = "canonical_sha256") -> str:
    if not isinstance(value, Mapping):
        raise ClosureError("CANONICAL_VALUE_INVALID", "canonical value must be an object")
    projection = dict(value)
    projection.pop(excluded, None)
    return hashlib.sha256(canonical_bytes(projection)).hexdigest()


def _immutable_copy(value: Any) -> Any:
    """Copy JSON-shaped evidence before adding it to a closure."""
    return copy.deepcopy(value)


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _file_sha256(path: Path, label: str, maximum: int = MAX_JSON_BYTES) -> str:
    """Hash a bounded regular schema file used as a contract binding."""
    # Checked-in source contracts are repository files, not sealed evidence
    # artifacts.  Their current-source SHA is bound by the profile; requiring
    # mode 0444 here would make ordinary source checkout validation fail.
    data, _ = _read_regular(path, label, maximum, require_readonly=False)
    return sha256_bytes(data)


def capture_contract_bindings() -> dict[str, dict[str, str]]:
    """Return the fixed source/schema identities used by every capture path.

    These are source-contract bindings, not evidence of a successful capture.
    Keeping the projection here gives the composer, outer receipt and closure
    validator one exact set of names and prevents a stale helper/schema from
    being hidden behind a self-rehashed receipt.
    """
    paths = {
        "capture_tool": (CAPTURE_EXECUTOR_TOOL_RELATIVE, CAPTURE_EXECUTOR_TOOL_PATH),
        "capture_schema": (CAPTURE_EXECUTOR_SCHEMA_RELATIVE, CAPTURE_EXECUTOR_SCHEMA_PATH),
        "closure_tool": ("scripts/registration_plugin_dependency_closure.py", Path(__file__)),
        "closure_schema": (
            "configs/slam_benchmark_profiles/registration_plugin_dependency_closure_v1.schema.json",
            ROOT / "configs/slam_benchmark_profiles/registration_plugin_dependency_closure_v1.schema.json",
        ),
        "directory_tool": (CAPTURE_DIRECTORY_TOOL_RELATIVE, CAPTURE_DIRECTORY_TOOL_PATH),
        "directory_schema": (CAPTURE_DIRECTORY_SCHEMA_RELATIVE, CAPTURE_DIRECTORY_SCHEMA_PATH),
        "source_validator": (SOURCE_MANIFEST_TOOL_RELATIVE, SOURCE_MANIFEST_TOOL_PATH),
        "source_schema": (SOURCE_MANIFEST_SCHEMA_RELATIVE, SOURCE_MANIFEST_SCHEMA_PATH),
        "rosdep_prepare_tool": (ROSDEP_PREPARE_TOOL_RELATIVE, ROSDEP_PREPARE_TOOL_PATH),
        "rosdep_prepare_schema": (ROSDEP_PREPARE_SCHEMA_RELATIVE, ROSDEP_PREPARE_SCHEMA_PATH),
    }
    return {
        key: {"path": relative, "sha256": _file_sha256(path, key)}
        for key, (relative, path) in paths.items()
    }


def _load_rosdep_prepare_module() -> Any:
    """Load the pinned prepare producer for one shared discovery projection."""
    spec = importlib.util.spec_from_file_location(
        "registration_plugin_rosdep_prepare_for_closure", ROSDEP_PREPARE_TOOL_PATH
    )
    if spec is None or spec.loader is None:
        raise ClosureError("ROSDEP_PREPARE_TOOL_MISSING", str(ROSDEP_PREPARE_TOOL_PATH))
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
    except Exception as error:
        raise ClosureError("ROSDEP_PREPARE_TOOL_INVALID", str(error)) from error
    return module


def rosdep_prepare_contract(*, required: bool = True,
                            profile_path: Path | None = None,
                            profile_sha256: str | None = None) -> dict[str, Any]:
    """Return the exact rosdep-prepare binding shared by plan and profile.

    Discovery-11 is an immutable, already-reviewed input.  The dynamic
    capture profile binding is kept in an executor plan, while the static
    capture contract uses :func:`rosdep_prepare_policy_projection` below and
    therefore avoids a self-referential profile hash.
    """
    prepare = _load_rosdep_prepare_module()
    if profile_path is None:
        profile_path = ROOT / (
            "configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json"
        )
    if profile_sha256 is None:
        profile_sha256 = _file_sha256(profile_path, "capture profile")
    if not isinstance(profile_sha256, str) or not re.fullmatch(
            r"[0-9a-f]{64}", profile_sha256):
        raise ClosureError("ROSDEP_PREPARE_CONTRACT_INVALID", "profile SHA")

    packages = [
        {key: item[key] for key in (
            "name", "version", "architecture", "filename", "uri",
            "size_bytes", "sha256", "artifact_path")}
        for item in prepare.EXPECTED_PACKAGES
    ]
    selected_files = [
        {key: item[key] for key in (
            "relative_path", "bytes", "sha256", "artifact_path")}
        for item in prepare.EXPECTED_SELECTED_FILES
    ]
    discovery = {
        "discovery_id": prepare.DISCOVERY_ID,
        "root": str(prepare.DISCOVERY_ROOT),
        "receipt": {
            "path": str(prepare.DISCOVERY_ROOT / prepare.DISCOVERY_RECEIPT_NAME),
            "sha256": prepare.EXPECTED_RECEIPT_SHA256,
            "canonical_sha256": prepare.EXPECTED_DISCOVERY_CANONICAL_SHA256,
        },
        "profile": {
            "path": str(prepare.PROFILE_PATH),
            "sha256": prepare.EXPECTED_PROFILE_SHA256,
        },
        "image": {
            "digest": prepare.EXPECTED_IMAGE_DIGEST,
            "platform": prepare.EXPECTED_IMAGE_PLATFORM,
            "repo_digests": ["ros@" + prepare.EXPECTED_IMAGE_DIGEST],
        },
        "rosdistro": {
            "repository": "https://github.com/ros/rosdistro.git",
            "commit": prepare.EXPECTED_COMMIT,
            "archive_url": (
                "https://codeload.github.com/ros/rosdistro/tar.gz/"
                + prepare.EXPECTED_COMMIT
            ),
            "archive_bytes": prepare.EXPECTED_ARCHIVE_BYTES,
            "archive_sha256": prepare.EXPECTED_ARCHIVE_SHA256,
        },
        "packages": packages,
        "selected_files": selected_files,
        "source_records": list(prepare.EXPECTED_SOURCE_RECORDS),
        "receipt_bytes_sha256": prepare.EXPECTED_RECEIPT_SHA256,
    }
    input_bundle = {
        "schema": ROSDEP_PREPARE_INPUT_SCHEMA,
        "schema_version": 1,
        "source_relative_root": ROSDEP_PREPARE_INPUT_RELATIVE,
        "container_root": ROSDEP_PREPARE_CONTAINER_ROOT,
        "mount": {
            "source_relative": ROSDEP_PREPARE_INPUT_RELATIVE,
            "target": ROSDEP_PREPARE_CONTAINER_ROOT,
            "read_only": True,
            "noexec": False,
            "repository_bind": "forbidden",
        },
        "copy_policy": "atomic-regular-0444-single-link-host-owned",
        "identity_policy": "source-and-pre-post-copy-descriptor-exact",
        "files": [
            {
                "role": "tool",
                "source_path": ROSDEP_PREPARE_TOOL_RELATIVE,
                "container_path": ROSDEP_PREPARE_CONTAINER_TOOL,
                "sha256": _file_sha256(ROSDEP_PREPARE_TOOL_PATH, "prepare tool"),
            },
            {
                "role": "schema",
                "source_path": ROSDEP_PREPARE_SCHEMA_RELATIVE,
                "container_path": ROSDEP_PREPARE_CONTAINER_SCHEMA,
                "sha256": _file_sha256(ROSDEP_PREPARE_SCHEMA_PATH, "prepare schema"),
            },
            {
                "role": "discovery_validator",
                "source_path": ROSDEP_DISCOVERY_HELPER_RELATIVE,
                "container_path": ROSDEP_PREPARE_CONTAINER_DISCOVERY,
                "sha256": _file_sha256(
                    ROSDEP_DISCOVERY_HELPER_PATH, "discovery helper"),
            },
            {
                "role": "consumer_profile",
                "source_path": "<active-profile>",
                "container_path": ROSDEP_PREPARE_CONTAINER_PROFILE,
                "sha256_source": "plan_profile_sha256",
            },
        ],
    }
    contract = {
        "schema": "registration-plugin-rosdep-prepare-contract-v1",
        "schema_version": 1,
        "required": bool(required),
        "phase_order": [
            "dependency_install", "network_disconnect", "rosdep_prepare",
            "rosdep_resolution",
        ],
        "tool": {
            "path": ROSDEP_PREPARE_TOOL_RELATIVE,
            "sha256": _file_sha256(ROSDEP_PREPARE_TOOL_PATH, "prepare tool"),
        },
        "schema_binding": {
            "path": ROSDEP_PREPARE_SCHEMA_RELATIVE,
            "sha256": _file_sha256(ROSDEP_PREPARE_SCHEMA_PATH, "prepare schema"),
        },
        "input_bundle": input_bundle,
        "mount_policy": dict(ROSDEP_PREPARE_MOUNT_POLICY),
        "capture_profile": {
            "path": str(profile_path), "sha256": profile_sha256,
        },
        "discovery": discovery,
        "execution": {
            "image_digest": prepare.EXPECTED_IMAGE_DIGEST,
            "platform": prepare.EXPECTED_IMAGE_PLATFORM,
            "pull": False, "build": False, "network": "none",
            "network_disconnect_required": True,
            "fallback": "forbidden", "runner": "existing_prepare_tool",
        },
        "output": {
            "receipt_relative": (
                "rosdep-prepare/" + prepare.PREPARE_SCHEMA.replace(
                    "registration-plugin-", "registration_plugin_"
                ).replace("-v1", ".receipt.json")
            ),
            "source_and_cache": "receipt-bound-sealed-descriptors",
            "deb_inputs": "discovery-11-exact-four",
        },
        "promotion": {
            "status": "REVIEW_REQUIRED",
            "benchmark_eligible": False,
            "claim_eligible": False,
            "active_profile_switch": False,
            "promotion": "FORBIDDEN_UNTIL_SIGNED_REVIEW",
        },
    }
    # The producer's filename is intentionally fixed independently of the
    # schema name; keeping it explicit avoids accepting a future rename.
    contract["output"]["receipt_relative"] = (
        "rosdep-prepare/registration_plugin_rosdep_prepare.receipt.json"
    )
    contract["identity_sha256"] = canonical_hash(contract, "identity_sha256")
    return contract


def rosdep_prepare_policy_projection() -> dict[str, Any]:
    """Return the static profile projection without dynamic profile identity."""
    contract = rosdep_prepare_contract(required=True)
    return {
        key: _immutable_copy(contract[key])
        for key in ("schema", "schema_version", "phase_order", "tool",
                    "schema_binding", "input_bundle", "mount_policy", "discovery",
                    "execution", "output", "promotion")
    }


def capture_contract() -> dict[str, Any]:
    """Return the one non-promoting capture contract used by profile/candidates.

    This is deliberately a projection of source identities, not a claim that a
    capture has happened.  The same function is used by the closure receipt and
    readback validators so profile and candidate documents cannot silently
    describe a different v2 source or host-owned directory contract.
    """
    bindings = capture_contract_bindings()
    return {
        "schema": CAPTURE_CONTRACT_SCHEMA,
        "schema_version": CAPTURE_CONTRACT_SCHEMA_VERSION,
        "status": REVIEW_STATUS,
        "benchmark_eligible": False,
        "contract_bindings": bindings,
        "required_artifacts": list(CAPTURE_REQUIRED_ARTIFACTS),
        "artifact_policy": CAPTURE_ARTIFACT_POLICY,
        "directory_policy": CAPTURE_DIRECTORY_POLICY,
        "managed_directory_paths": list(CAPTURE_DIRECTORY_RELATIVE_PATHS),
        "release_declaration_policy": dict(RELEASE_DECLARATION_POLICY),
        "packages_format_policy": dict(PACKAGES_FORMAT_POLICY),
        "gpgv_status_policy": dict(GPGV_STATUS_POLICY),
        "gpg_inventory_policy": dict(GPG_INVENTORY_POLICY),
        "reference_policy": dict(REFERENCE_POLICY),
        "restore_owner_policy": dict(RESTORE_OWNER_POLICY),
        "source_snapshot": {
            "schema": "glim_clean_room_r3_apt_source_snapshot_v2",
            "schema_path": SOURCE_MANIFEST_SCHEMA_RELATIVE,
            "schema_sha256": bindings["source_schema"]["sha256"],
            "validator_path": SOURCE_MANIFEST_TOOL_RELATIVE,
            "validator_sha256": bindings["source_validator"]["sha256"],
        },
        "rosdep_prepare": rosdep_prepare_policy_projection(),
        "promotion": "FORBIDDEN_UNTIL_SIGNED_RECEIPT",
    }


REFERENCE_MAP_FIELDS = ("release", "keyring", "signature", "packages")
REFERENCE_COUNT_FIELDS = (
    "repository_references", "release_evidence", "package_evidence",
    "signature_evidence", "unique_release_references",
    "unique_keyring_references", "unique_signature_references",
    "unique_package_references",
)


def _empty_reference_projection(signature_count: int = 0) -> tuple[dict[str, Any], dict[str, int]]:
    """Return the explicit empty reference projection used before APT evidence."""
    if type(signature_count) is not int or signature_count < 0:
        raise ClosureError("REFERENCE_PROJECTION_INVALID", "signature count")
    return (
        {field: {} for field in REFERENCE_MAP_FIELDS},
        {
            "repository_references": signature_count,
            "release_evidence": 0,
            "package_evidence": 0,
            "signature_evidence": 0,
            "unique_release_references": 0,
            "unique_keyring_references": 0,
            "unique_signature_references": 0,
            "unique_package_references": 0,
        },
    )


def _validate_reference_projection(reference_map: Any, reference_counts: Any,
                                   label: str = "reference projection") -> tuple[dict[str, Any], dict[str, int]]:
    """Validate the map/count shape without trusting a self-rehashed receipt."""
    if (not isinstance(reference_map, Mapping) or
            set(reference_map) != set(REFERENCE_MAP_FIELDS) or
            not isinstance(reference_counts, Mapping) or
            set(reference_counts) != set(REFERENCE_COUNT_FIELDS)):
        raise ClosureError("REFERENCE_PROJECTION_INVALID", label)
    normalized_map: dict[str, dict[str, str]] = {}
    for field in REFERENCE_MAP_FIELDS:
        entries = reference_map[field]
        if not isinstance(entries, Mapping):
            raise ClosureError("REFERENCE_PROJECTION_INVALID", label)
        normalized: dict[str, str] = {}
        for index, identity in entries.items():
            if (not isinstance(index, str) or
                    not re.fullmatch(r"(?:0|[1-9][0-9]*)", index)):
                raise ClosureError("REFERENCE_PROJECTION_INVALID", label)
            if not isinstance(identity, str) or not SHA256_RE.fullmatch(identity):
                raise ClosureError("REFERENCE_PROJECTION_INVALID", label)
            normalized[index] = identity
        normalized_map[field] = normalized
    normalized_counts: dict[str, int] = {}
    for field in REFERENCE_COUNT_FIELDS:
        value = reference_counts[field]
        if type(value) is not int or value < 0 or value > 4096:
            raise ClosureError("REFERENCE_PROJECTION_INVALID", label)
        normalized_counts[field] = value
    if normalized_counts["unique_release_references"] != len(
            set(normalized_map["release"].values())) or \
            normalized_counts["unique_keyring_references"] != len(
                set(normalized_map["keyring"].values())) or \
            normalized_counts["unique_signature_references"] != len(
                set(normalized_map["signature"].values())) or \
            normalized_counts["unique_package_references"] != len(
                set(normalized_map["packages"].values())):
        raise ClosureError("REFERENCE_PROJECTION_INVALID", label)
    return normalized_map, normalized_counts


def _capture_reference_projection(capture_root: Path) -> tuple[dict[str, Any], dict[str, int]]:
    """Read the inner APT projection when present, otherwise use an empty one.

    The inner receipt is intentionally not treated as a promotion proof here;
    the capture executor performs the authoritative byte/signature validation.
    This closure-level projection only prevents a shared Release/keyring result
    from disappearing or being duplicated when the closure is composed.
    """
    inner_path = capture_root / "in-container-apt-source.json"
    if not inner_path.exists() and not inner_path.is_symlink():
        return _empty_reference_projection()
    inner, _ = _read_json(inner_path, "in-container APT source", sidecar=True)
    if inner.get("canonical_sha256") != canonical_hash(inner):
        raise ClosureError("REFERENCE_PROJECTION_INVALID", "inner APT source identity")
    has_map = "reference_map" in inner
    has_counts = "reference_counts" in inner
    if has_map != has_counts:
        raise ClosureError("REFERENCE_PROJECTION_INVALID", "inner APT source map/count")
    if not has_map:
        return _empty_reference_projection()
    return _validate_reference_projection(
        inner["reference_map"], inner["reference_counts"],
        "inner APT source reference projection")


def _require_sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or not SHA256_RE.fullmatch(value):
        raise ClosureError("SHA_INVALID", label)
    return value


def _require_digest(value: Any, label: str) -> str:
    if not isinstance(value, str) or not IMAGE_DIGEST_RE.fullmatch(value):
        raise ClosureError("IMAGE_DIGEST_INVALID", label)
    return value


def _require_safe_text(value: Any, label: str, maximum: int = 512) -> str:
    if (not isinstance(value, str) or not value or len(value) > maximum or
            any(ord(character) < 0x20 or ord(character) == 0x7f
                for character in value)):
        raise ClosureError("TEXT_INVALID", label)
    return value


def _safe_relative(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value or "\\" in value or "\x00" in value:
        raise ClosureError("PATH_INVALID", label)
    path = Path(value)
    if path.is_absolute() or any(part in ("", ".", "..") for part in path.parts):
        raise ClosureError("PATH_INVALID", label)
    if path.as_posix() != value:
        raise ClosureError("PATH_INVALID", label)
    return value


def _absolute(value: Any, label: str) -> Path:
    if not isinstance(value, str) or not value or "\x00" in value or "\n" in value:
        raise ClosureError("PATH_INVALID", label)
    path = Path(value)
    if not path.is_absolute() or any(part in ("", ".", "..") for part in path.parts):
        raise ClosureError("PATH_INVALID", label)
    return path


def _check_parent_components(path: Path, label: str) -> None:
    if not path.is_absolute():
        raise ClosureError("PATH_INVALID", label)
    current = Path("/")
    for component in path.parts[1:-1]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise ClosureError("PATH_MISSING", "{}: {}".format(label, current)) from error
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise ClosureError("PATH_PARENT_INVALID", "{}: {}".format(label, current))


def _read_regular(path: Path, label: str, maximum: int,
                  *, require_readonly: bool = True,
                  allow_empty: bool = False) -> tuple[bytes, os.stat_result]:
    path = _absolute(str(path), label)
    _check_parent_components(path, label)
    try:
        parent_before = path.parent.lstat()
        before = path.lstat()
    except OSError as error:
        raise ClosureError("FILE_MISSING", "{}: {}".format(label, path)) from error
    if stat.S_ISLNK(parent_before.st_mode) or not stat.S_ISDIR(parent_before.st_mode):
        raise ClosureError("FILE_PARENT_INVALID", label)
    if (stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or
            before.st_nlink != 1 or before.st_size < 0 or
            (before.st_size == 0 and not allow_empty) or
            before.st_size > maximum):
        raise ClosureError("FILE_INVALID", label)
    if require_readonly and stat.S_IMODE(before.st_mode) != 0o444:
        raise ClosureError("FILE_MUTABLE", label)
    flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
    try:
        fd = os.open(str(path), flags)
    except OSError as error:
        raise ClosureError("FILE_OPEN_FAILED", label) from error
    try:
        fd_before = os.fstat(fd)
        if (fd_before.st_dev != before.st_dev or fd_before.st_ino != before.st_ino or
                fd_before.st_nlink != 1 or fd_before.st_size != before.st_size):
            raise ClosureError("FILE_IDENTITY_CHANGED", label)
        chunks = []
        total = 0
        while True:
            block = os.read(fd, min(1024 * 1024, maximum - total + 1))
            if not block:
                break
            chunks.append(block)
            total += len(block)
            if total > maximum:
                raise ClosureError("FILE_OVERSIZE", label)
        fd_after = os.fstat(fd)
    except OSError as error:
        raise ClosureError("FILE_READ_FAILED", label) from error
    finally:
        os.close(fd)
    data = b"".join(chunks)
    if (len(data) != before.st_size or fd_after.st_dev != before.st_dev or
            fd_after.st_ino != before.st_ino or fd_after.st_size != before.st_size):
        raise ClosureError("FILE_CHANGED", label)
    try:
        after = path.lstat()
        parent_after = path.parent.lstat()
    except OSError as error:
        raise ClosureError("FILE_CHANGED", label) from error
    if (after.st_dev != before.st_dev or after.st_ino != before.st_ino or
            after.st_size != before.st_size or after.st_nlink != 1):
        raise ClosureError("FILE_CHANGED", label)
    if (parent_after.st_dev != parent_before.st_dev or
            parent_after.st_ino != parent_before.st_ino or
            stat.S_ISLNK(parent_after.st_mode) or
            not stat.S_ISDIR(parent_after.st_mode)):
        raise ClosureError("FILE_PARENT_CHANGED", label)
    return data, before


def _sidecar(path: Path, digest: str) -> dict[str, Any]:
    sidecar = Path(str(path) + ".sha256")
    payload, info = _read_regular(sidecar, "sidecar", 512)
    try:
        tokens = payload.decode("ascii").strip().split()
    except UnicodeError as error:
        raise ClosureError("SIDECAR_INVALID", str(sidecar)) from error
    if tokens != [digest, path.name]:
        raise ClosureError("SIDECAR_MISMATCH", str(path))
    return {
        "path": str(sidecar), "bytes": info.st_size,
        "sha256": sha256_bytes(payload), "mode": stat.S_IMODE(info.st_mode),
        "uid": info.st_uid, "gid": info.st_gid, "nlink": info.st_nlink,
        "device": info.st_dev, "inode": info.st_ino,
    }


def _file_descriptor(path: Path, label: str, maximum: int = MAX_TEXT_BYTES,
                     *, sidecar: bool = False,
                     allow_empty: bool = False,
                     require_readonly: bool = True) -> dict[str, Any]:
    data, info = _read_regular(
        path, label, maximum, allow_empty=allow_empty,
        require_readonly=require_readonly)
    result = {
        "path": str(path), "bytes": len(data), "sha256": sha256_bytes(data),
        "mode": stat.S_IMODE(info.st_mode), "uid": info.st_uid,
        "gid": info.st_gid, "nlink": info.st_nlink, "device": info.st_dev,
        "inode": info.st_ino,
    }
    if sidecar:
        result["sidecar"] = _sidecar(path, result["sha256"])
    return result


def _verify_descriptor(value: Any, label: str, maximum: int = MAX_TEXT_BYTES,
                       *, require_sidecar: bool = False) -> dict[str, Any]:
    if not isinstance(value, Mapping):
        raise ClosureError("DESCRIPTOR_INVALID", label)
    required = {"path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
                "device", "inode"}
    if require_sidecar:
        required.add("sidecar")
    if set(value) != required:
        raise ClosureError("DESCRIPTOR_FIELDS_INVALID", label)
    path = _absolute(value["path"], label)
    expected = dict(value)
    actual = _file_descriptor(path, label, maximum, sidecar=require_sidecar)
    if actual != expected:
        raise ClosureError("DESCRIPTOR_DRIFT", label)
    return actual


def _tree_descriptor(path: Path, label: str, maximum: int = MAX_TEXT_BYTES,
                     *, allow_empty_files: bool = False) -> dict[str, Any]:
    path = _absolute(str(path), label)
    _check_parent_components(path / "placeholder", label)
    try:
        root_info = path.lstat()
    except OSError as error:
        raise ClosureError("TREE_MISSING", label) from error
    if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
        raise ClosureError("TREE_INVALID", label)
    files = []
    directories = []
    pending = [path]
    while pending:
        current = pending.pop()
        try:
            entries = sorted(os.scandir(str(current)), key=lambda item: item.name)
        except OSError as error:
            raise ClosureError("TREE_READ_FAILED", label) from error
        for entry in entries:
            child = Path(entry.path)
            info = entry.stat(follow_symlinks=False)
            relative = child.relative_to(path).as_posix()
            _safe_relative(relative, "{} entry".format(label))
            if stat.S_ISLNK(info.st_mode):
                raise ClosureError("TREE_SYMLINK", relative)
            if stat.S_ISDIR(info.st_mode):
                directories.append(relative)
                pending.append(child)
                continue
            if (not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or
                    stat.S_IMODE(info.st_mode) != 0o444):
                raise ClosureError("TREE_FILE_INVALID", relative)
            if len(files) >= MAX_TREE_FILES:
                raise ClosureError("TREE_TOO_LARGE", label)
            descriptor = _file_descriptor(
                child, "{} entry".format(label), maximum,
                allow_empty=allow_empty_files)
            files.append({"path": relative, "bytes": descriptor["bytes"],
                          "sha256": descriptor["sha256"], "mode": descriptor["mode"],
                          "uid": descriptor["uid"], "gid": descriptor["gid"],
                          "nlink": descriptor["nlink"], "device": descriptor["device"],
                          "inode": descriptor["inode"]})
    files.sort(key=lambda item: item["path"])
    directories.sort()
    if not files:
        raise ClosureError("TREE_EMPTY", label)
    projection = {"root": str(path), "directories": directories, "files": files}
    return {
        "root": str(path), "directories": directories, "files": files,
        "file_count": len(files), "tree_sha256": sha256_bytes(canonical_bytes(projection)),
    }


def _deb_tree_descriptor(path: Path, label: str) -> dict[str, Any]:
    """Describe only safe downloaded debs after exact APT housekeeping checks."""
    path = _absolute(str(path), label)
    _check_parent_components(path / "placeholder", label)
    try:
        root_info = path.lstat()
        children = list(path.iterdir())
    except OSError as error:
        raise ClosureError("TREE_MISSING", label) from error
    if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
        raise ClosureError("TREE_INVALID", label)
    try:
        evidence = EXCLUDE_DOWNLOAD_HOUSEKEEPING(children, root_info, label)
    except Exception as error:
        raise ClosureError("TREE_HOUSEKEEPING_INVALID", str(error)) from error
    files = []
    for child in sorted(evidence, key=lambda item: item.name):
        try:
            info = child.lstat()
        except OSError as error:
            raise ClosureError("TREE_FILE_INVALID", child.name) from error
        if (stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or
                info.st_nlink != 1 or stat.S_IMODE(info.st_mode) not in
                {0o444, 0o644} or not child.name.endswith(".deb")):
            raise ClosureError("TREE_FILE_INVALID", child.name)
        if len(files) >= MAX_TREE_FILES:
            raise ClosureError("TREE_TOO_LARGE", label)
        descriptor = _file_descriptor(
            child, "{} entry".format(label), MAX_DEB_BYTES,
            require_readonly=False)
        files.append({"path": child.name, "bytes": descriptor["bytes"],
                      "sha256": descriptor["sha256"], "mode": descriptor["mode"],
                      "uid": descriptor["uid"], "gid": descriptor["gid"],
                      "nlink": descriptor["nlink"], "device": descriptor["device"],
                      "inode": descriptor["inode"]})
    if not files:
        raise ClosureError("TREE_EMPTY", label)
    projection = {"root": str(path), "directories": [], "files": files}
    return {
        **projection, "file_count": len(files),
        "tree_sha256": sha256_bytes(canonical_bytes(projection)),
    }


def _verify_tree(value: Any, label: str, *, allow_empty_files: bool = False,
                 maximum: int = MAX_TEXT_BYTES) -> dict[str, Any]:
    if not isinstance(value, Mapping) or set(value) != {
            "root", "directories", "files", "file_count", "tree_sha256"}:
        raise ClosureError("TREE_DESCRIPTOR_INVALID", label)
    root = _absolute(value["root"], label)
    actual = _tree_descriptor(
        root, label, maximum=maximum, allow_empty_files=allow_empty_files)
    if actual != dict(value):
        raise ClosureError("TREE_DESCRIPTOR_DRIFT", label)
    return actual


def _verify_deb_tree(value: Any, label: str) -> dict[str, Any]:
    if not isinstance(value, Mapping) or set(value) != {
            "root", "directories", "files", "file_count", "tree_sha256"}:
        raise ClosureError("TREE_DESCRIPTOR_INVALID", label)
    actual = _deb_tree_descriptor(_absolute(value["root"], label), label)
    if actual != dict(value):
        raise ClosureError("TREE_DESCRIPTOR_DRIFT", label)
    return actual


def _read_json(path: Path, label: str, *, sidecar: bool = False) -> tuple[dict[str, Any], dict[str, Any]]:
    data, info = _read_regular(path, label, MAX_JSON_BYTES)
    digest = sha256_bytes(data)
    if sidecar:
        _sidecar(path, digest)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise ClosureError("JSON_INVALID", label) from error
    if not isinstance(value, dict):
        raise ClosureError("JSON_OBJECT_REQUIRED", label)
    return value, {
        "path": str(path), "bytes": info.st_size, "sha256": digest,
        "mode": stat.S_IMODE(info.st_mode), "uid": info.st_uid,
        "gid": info.st_gid, "nlink": info.st_nlink, "device": info.st_dev,
        "inode": info.st_ino,
    }


def _https(value: Any, label: str) -> str:
    if (not isinstance(value, str) or len(value) > MAX_URL_LENGTH or
            not value.startswith("https://") or any(char in value for char in "\x00\r\n") or
            "?" in value or "#" in value or "@" in value.split("//", 1)[-1].split("/", 1)[0]):
        raise ClosureError("URL_INVALID", label)
    return value


def _validate_apt_source_url_policy(value: Any, label: str = "APT URL policy") -> dict[str, Any]:
    if value != APT_SOURCE_URL_POLICY:
        raise ClosureError("APT_URL_POLICY_INVALID", label)
    return dict(APT_SOURCE_URL_POLICY)


def _http_url_matches_prefix(value: str, prefix: str) -> bool:
    try:
        actual = urlsplit(value)
        expected = urlsplit(prefix)
        port = actual.port
    except ValueError:
        return False
    if actual.scheme != "http" or expected.scheme != "http" or port not in (None, 80):
        return False
    if actual.hostname != expected.hostname or actual.query or actual.fragment:
        return False
    base_path = expected.path.rstrip("/")
    return actual.path == base_path or actual.path.startswith(base_path + "/")


def _apt_or_https(value: Any, label: str) -> str:
    if not isinstance(value, str) or len(value) > MAX_URL_LENGTH or \
            any(char in value for char in "\x00\r\n"):
        raise ClosureError("URL_INVALID", label)
    try:
        parts = urlsplit(value)
        netloc, query, fragment = parts.netloc, parts.query, parts.fragment
        username, password = parts.username, parts.password
    except ValueError as error:
        raise ClosureError("URL_INVALID", label) from error
    if not netloc or query or fragment or username or password:
        raise ClosureError("URL_INVALID", label)
    if parts.scheme == "https":
        return value
    if parts.scheme != "http" or not any(
            _http_url_matches_prefix(value, prefix)
            for prefix in APT_SOURCE_URL_POLICY["http_prefixes"]):
        raise ClosureError("URL_INVALID", label)
    return value


def _validate_capture(value: Any) -> dict[str, Any]:
    required = {
        "schema", "schema_version", "status", "distro", "dependency_leg",
        "image_digest", "platform", "requirements_path", "base_status_path",
        "source_root", "source_manifest_path", "resolvers", "phase_records",
        "optional_prefetch", "gpgv_status_policy", "canonical_sha256",
    }
    allowed = required | {"apt_source_url_policy", "release_declaration_policy",
                          "packages_format_policy", "rosdep_prepare_contract",
                          "rosdep_prepare"}
    if (not isinstance(value, Mapping) or not required.issubset(set(value)) or
            set(value) - allowed):
        raise ClosureError("CAPTURE_SCHEMA_INVALID", "capture input fields")
    if (value["schema"] != CAPTURE_SCHEMA or value["schema_version"] != 1 or
            value["status"] != "CAPTURED_REVIEW_REQUIRED" or
            value["distro"] not in DISTROS or value["dependency_leg"] not in LEGS or
            value["platform"] != PLATFORM or
            value["canonical_sha256"] != canonical_hash(value)):
        raise ClosureError("CAPTURE_IDENTITY_INVALID", "capture identity/status")
    if "apt_source_url_policy" in value:
        _validate_apt_source_url_policy(value["apt_source_url_policy"],
                                        "capture input URL policy")
    if "release_declaration_policy" in value and \
            value["release_declaration_policy"] != RELEASE_DECLARATION_POLICY:
        raise ClosureError("RELEASE_DECLARATION_POLICY_INVALID", "capture input policy")
    if value.get("packages_format_policy", PACKAGES_FORMAT_POLICY) != PACKAGES_FORMAT_POLICY:
        raise ClosureError("PACKAGES_FORMAT_POLICY_INVALID", "capture input policy")
    if value["gpgv_status_policy"] != GPGV_STATUS_POLICY:
        raise ClosureError("GPGV_STATUS_POLICY_INVALID", "capture input policy")
    prepare_contract_present = "rosdep_prepare_contract" in value
    prepare_projection_present = "rosdep_prepare" in value
    if prepare_contract_present != prepare_projection_present:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "capture input pair")
    if prepare_contract_present:
        _validate_rosdep_prepare_binding(
            value["rosdep_prepare_contract"], value["rosdep_prepare"])
    for key in ("requirements_path", "base_status_path", "source_manifest_path"):
        _safe_relative(value[key], key)
    _safe_relative(value["source_root"], "source_root")
    _require_digest(value["image_digest"], "capture image digest")
    resolvers = value["resolvers"]
    if not isinstance(resolvers, Mapping) or set(resolvers) != set(ROLES):
        raise ClosureError("CAPTURE_RESOLVER_INVALID", "resolver roles")
    for role in ROLES:
        item = resolvers[role]
        required_resolver = {"resolver_path", "raw_log_path", "deb_root_path",
                             "command", "exit_status"}
        if not isinstance(item, Mapping) or set(item) != required_resolver:
            raise ClosureError("CAPTURE_RESOLVER_INVALID", role)
        for key in ("resolver_path", "raw_log_path", "deb_root_path"):
            _safe_relative(item[key], "{} {}".format(role, key))
        _validate_command(item["command"], "{} resolver".format(role))
        if type(item["exit_status"]) is not int or item["exit_status"] != 0:
            raise ClosureError("CAPTURE_RESOLVER_FAILED", role)
    # A production capture explicitly carries the prepare contract and uses
    # the current 19-phase order.  Keep both historical 16-phase packets and
    # the pre-owner-restore 17-phase synthetic/test shape readable without
    # allowing either to masquerade as a current packet.
    phase_count = len(value.get("phase_records", ()))
    if "rosdep_prepare_contract" in value or phase_count == len(EXPECTED_PHASES):
        phase_order = EXPECTED_PHASES
    elif phase_count == len(PRE_RESTORE_EXPECTED_PHASES):
        phase_order = PRE_RESTORE_EXPECTED_PHASES
    else:
        phase_order = LEGACY_EXPECTED_PHASES
    _validate_phase_records(value["phase_records"], value["dependency_leg"], phase_order)
    references = {
        "requirements": value["requirements_path"],
        "base_status": value["base_status_path"],
        "source_manifest": value["source_manifest_path"],
        "source_root": value["source_root"],
    }
    for role in ROLES:
        resolver = value["resolvers"][role]
        for key in ("resolver_path", "raw_log_path", "deb_root_path"):
            references["{} {}".format(role, key)] = resolver[key]
    for index, item in enumerate(value["phase_records"]):
        references["phase {} stdout".format(index)] = item["stdout_path"]
        references["phase {} stderr".format(index)] = item["stderr_path"]
    seen = {}
    for label, relative in references.items():
        if relative in seen:
            raise ClosureError("CAPTURE_PATH_ALIAS", "{} aliases {}".format(label, seen[relative]))
        seen[relative] = label
    optional = value["optional_prefetch"]
    if not isinstance(optional, Mapping):
        raise ClosureError("CAPTURE_PREFETCH_INVALID", "optional_prefetch")
    if value["dependency_leg"] == "absent":
        if dict(optional) != {"status": "NOT_APPLICABLE"}:
            raise ClosureError("CAPTURE_PREFETCH_INVALID", "absent leg prefetch")
    else:
        if set(optional) != {"status", "root_path", "dependencies"} or \
                optional["status"] != "SEALED_PREFETCH":
            raise ClosureError("CAPTURE_PREFETCH_INVALID", "present leg prefetch")
        _safe_relative(optional["root_path"], "prefetch root")
        if not isinstance(optional["dependencies"], list) or not optional["dependencies"]:
            raise ClosureError("CAPTURE_PREFETCH_INVALID", "prefetch dependencies")
    return dict(value)


def _validate_command(value: Any, label: str) -> list[str]:
    if (not isinstance(value, list) or not value or
            any(not isinstance(token, str) or not token or len(token) > 512 or
                any(ord(character) < 0x20 or ord(character) == 0x7f for character in token)
                for token in value)):
        raise ClosureError("COMMAND_INVALID", label)
    if any(token in FORBIDDEN_COMMAND_TOKENS for token in value):
        raise ClosureError("NETWORK_FALLBACK_COMMAND", label)
    return list(value)


def _validate_phase_records(value: Any, dependency_leg: str,
                            phase_order: tuple[str, ...] | None = None
                            ) -> list[dict[str, Any]]:
    phase_order = phase_order or EXPECTED_PHASES
    if not isinstance(value, list) or len(value) != len(phase_order):
        raise ClosureError("PHASE_ORDER_INVALID", "phase count")
    if len(value) > MAX_PHASES:
        raise ClosureError("PHASE_ORDER_INVALID", "phase count")
    return [_validate_phase_record(item, index, dependency_leg, phase_order)
            for index, item in enumerate(value)]


def _validate_phase_record(item: Any, index: int, dependency_leg: str,
                           phase_order: tuple[str, ...] | None = None
                           ) -> dict[str, Any]:
    phase_order = phase_order or EXPECTED_PHASES
    required = {"phase", "argv", "argv_sha256", "returncode", "network_used",
                "timeout_seconds", "attempts", "stdout_path", "stderr_path"}
    if not isinstance(item, Mapping) or set(item) != required:
        raise ClosureError("PHASE_RECORD_INVALID", str(index))
    if index >= len(phase_order):
        raise ClosureError("PHASE_ORDER_INVALID", str(index))
    expected_phase = phase_order[index]
    if item["phase"] != expected_phase:
        raise ClosureError("PHASE_ORDER_INVALID", expected_phase)
    argv = _validate_command(item["argv"], expected_phase)
    if item["argv_sha256"] != sha256_bytes(canonical_bytes(argv)):
        raise ClosureError("PHASE_COMMAND_HASH_INVALID", expected_phase)
    if type(item["returncode"]) is not int or item["returncode"] != 0:
        raise ClosureError("PHASE_FAILED", expected_phase)
    expected_network = PHASE_NETWORK[expected_phase]
    if expected_network is None:
        expected_network = dependency_leg == "present"
    if type(item["network_used"]) is not bool or item["network_used"] != expected_network:
        raise ClosureError("PHASE_NETWORK_INVALID", expected_phase)
    if (type(item["timeout_seconds"]) is not int or
            not 1 <= item["timeout_seconds"] <= 600 or
            type(item["attempts"]) is not int or item["attempts"] != 1):
        raise ClosureError("PHASE_LIMIT_INVALID", expected_phase)
    for key in ("stdout_path", "stderr_path"):
        _safe_relative(item[key], "{} {}".format(expected_phase, key))
    return dict(item)


def _default_dpkg_reader(path: Path) -> Mapping[str, Any]:
    """Read only package control fields through a fixed, bounded argv."""
    argv = ["dpkg-deb", "--field", str(path), "Package", "Version",
            "Architecture", "Essential", "Pre-Depends", "Multi-Arch"]
    try:
        completed = subprocess.run(
            argv, stdout=subprocess.PIPE, stderr=subprocess.PIPE, check=False,
            timeout=15, stdin=subprocess.DEVNULL,
        )
    except (OSError, subprocess.SubprocessError) as error:
        raise ClosureError("DPKG_DEB_FAILED", str(path)) from error
    if completed.returncode != 0 or len(completed.stdout) > 64 * 1024:
        raise ClosureError("DPKG_DEB_FAILED", str(path))
    fields = {}
    for line in completed.stdout.decode("utf-8").splitlines():
        if ": " not in line:
            continue
        key, value = line.split(": ", 1)
        if key in fields:
            raise ClosureError("DPKG_DEB_DUPLICATE_FIELD", str(path))
        fields[key] = value
    required = {"Package", "Version", "Architecture"}
    if not required.issubset(fields):
        raise ClosureError("DPKG_DEB_FIELDS_MISSING", str(path))
    return {
        "name": fields["Package"], "version": fields["Version"],
        "architecture": fields["Architecture"],
        "essential": fields.get("Essential", "no").lower() == "yes",
        "pre_depends": [item.strip() for item in fields.get("Pre-Depends", "").split(",") if item.strip()],
        "multiarch": fields.get("Multi-Arch", "same"),
    }


def _phase_log_descriptors(capture_root: Path, records: list[dict[str, Any]]) -> list[dict[str, Any]]:
    result = []
    for item in records:
        entry = dict(item)
        entry["stdout"] = _file_descriptor(
            capture_root / item["stdout_path"], "{} stdout".format(item["phase"]),
            MAX_LOG_BYTES)
        entry["stderr"] = _file_descriptor(
            capture_root / item["stderr_path"], "{} stderr".format(item["phase"]),
            MAX_LOG_BYTES)
        del entry["stdout_path"], entry["stderr_path"]
        entry["stdout_sha256"] = entry["stdout"]["sha256"]
        entry["stderr_sha256"] = entry["stderr"]["sha256"]
        result.append(entry)
    return result


def _capture_file(capture_root: Path, relative: str, label: str,
                  maximum: int = MAX_TEXT_BYTES) -> dict[str, Any]:
    return _file_descriptor(capture_root / _safe_relative(relative, label), label, maximum)


def _apt_output_descriptors(apt_root: Path) -> dict[str, Any]:
    expected = {
        "proposal": "apt-allowlist-generation-proposal.json",
        "allowlist": "package-allowlist.json",
        "ledger": "apt-acquisition-ledger.json",
    }
    result = {}
    for key, filename in expected.items():
        result[key] = _file_descriptor(apt_root / filename, "apt {}".format(key),
                                       MAX_JSON_BYTES, sidecar=True)
    proposal, _ = _read_json(apt_root / expected["proposal"], "apt proposal")
    validated = APT.validate_proposal(apt_root)
    if proposal.get("status") != "PROPOSED_REVIEW_REQUIRED" or \
            validated.get("status") != "PASS":
        raise ClosureError("APT_PROPOSAL_INVALID", "proposal status")
    result["closure_identity_sha256"] = proposal.get("closure_identity_sha256")
    if not SHA256_RE.fullmatch(str(result["closure_identity_sha256"] or "")):
        raise ClosureError("APT_PROPOSAL_INVALID", "closure identity")
    return result


def _requested_urls(apt_root: Path) -> tuple[list[str], list[str]]:
    proposal, _ = _read_json(apt_root / "apt-allowlist-generation-proposal.json",
                             "apt proposal")
    ledger = proposal.get("apt_acquisition_ledger")
    if not isinstance(ledger, Mapping):
        raise ClosureError("APT_LEDGER_INVALID", "acquisition ledger")
    requested = set()
    final = set()
    for repository in ledger.get("repositories", []):
        for key in ("url", "release_url", "packages_url"):
            value = _apt_or_https(repository.get(key), "apt repository URL")
            requested.add(value)
            final.add(value)
    for entry in ledger.get("entries", []):
        value = _apt_or_https(entry.get("url"), "apt package URL")
        requested.add(value)
        final.add(value)
    source_v2 = ledger.get("source_manifest_schema") == APT_SOURCE_SCHEMA_V2
    for entry in ledger.get("apt_source_urls", []):
        if source_v2:
            if not isinstance(entry, Mapping) or set(entry) != {"path", "urls"} or \
                    not isinstance(entry["urls"], list):
                raise ClosureError("APT_LEDGER_INVALID", "v2 apt source URLs")
            values = entry["urls"]
        else:
            if not isinstance(entry, Mapping) or set(entry) != {"path", "url"}:
                raise ClosureError("APT_LEDGER_INVALID", "v1 apt source URL")
            values = [entry["url"]]
        for source_url in values:
            value = _apt_or_https(source_url, "apt source URL")
            requested.add(value)
            final.add(value)
    for entry in ledger.get("rosdep_source_urls", []):
        value = _https(entry.get("url"), "rosdep source URL")
        requested.add(value)
        final.add(value)
    return sorted(requested), sorted(final)


def _validate_closure_url_token(value: Any) -> str:
    if (isinstance(value, str) and ":" in value and
            not value.startswith(("http://", "https://"))):
        # Present-leg archive audit entries use name:sha256.
        if not SHA256_RE.fullmatch(value.rsplit(":", 1)[-1]):
            raise ClosureError("CLOSURE_URLS_INVALID", str(value))
        return value
    return _apt_or_https(value, "closure URL")


def _validate_prepare_input_binding_shape(binding: Any) -> None:
    """Validate the dynamic host-copy/input-mount projection shape."""
    required = {
        "schema", "schema_version", "relative_root", "container_root",
        "mount", "files", "identity_sha256",
    }
    if not isinstance(binding, Mapping) or set(binding) != required or \
            binding.get("schema") != ROSDEP_PREPARE_INPUT_SCHEMA or \
            binding.get("schema_version") != 1 or \
            binding.get("relative_root") != ROSDEP_PREPARE_INPUT_RELATIVE or \
            binding.get("container_root") != ROSDEP_PREPARE_CONTAINER_ROOT or \
            binding.get("identity_sha256") != canonical_hash(
                binding, "identity_sha256"):
        raise ClosureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", "shape")
    mount = binding.get("mount")
    if not isinstance(mount, Mapping) or set(mount) != {
            "source", "target", "read_only", "noexec", "repository_bind"} or \
            not isinstance(mount.get("source"), str) or \
            not mount["source"].startswith("/") or \
            mount.get("target") != ROSDEP_PREPARE_CONTAINER_ROOT or \
            mount.get("read_only") is not True or mount.get("noexec") is not False or \
            mount.get("repository_bind") != "forbidden":
        raise ClosureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", "mount")
    files = binding.get("files")
    if not isinstance(files, list) or len(files) != 4:
        raise ClosureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", "files")
    expected = [
        ("tool", ROSDEP_PREPARE_TOOL_RELATIVE, ROSDEP_PREPARE_CONTAINER_TOOL),
        ("schema", ROSDEP_PREPARE_SCHEMA_RELATIVE, ROSDEP_PREPARE_CONTAINER_SCHEMA),
        ("discovery_validator", ROSDEP_DISCOVERY_HELPER_RELATIVE,
         ROSDEP_PREPARE_CONTAINER_DISCOVERY),
        ("consumer_profile", "<active-profile>", ROSDEP_PREPARE_CONTAINER_PROFILE),
    ]
    descriptor_fields = {
        "path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
        "device", "inode", "sidecar",
    }
    source_fields = {
        "path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
        "device", "inode",
    }
    seen_relative: set[str] = set()
    for item, (role, source_path, container_path) in zip(files, expected):
        if not isinstance(item, Mapping) or set(item) != {
                "role", "source_path", "container_path", "relative_path",
                "source", "before", "after"} or \
                item.get("role") != role or item.get("source_path") != source_path or \
                item.get("container_path") != container_path:
            raise ClosureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", role)
        relative = item.get("relative_path")
        if (not isinstance(relative, str) or not relative or
                relative.startswith("/") or ".." in relative.split("/") or
                relative in seen_relative):
            raise ClosureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", role)
        seen_relative.add(relative)
        source = item.get("source")
        if not isinstance(source, Mapping) or set(source) != source_fields or \
                not isinstance(source.get("path"), str) or \
                not source["path"].startswith("/") or \
                not SHA256_RE.fullmatch(str(source.get("sha256"))) or \
                type(source.get("bytes")) is not int or source["bytes"] <= 0 or \
                type(source.get("nlink")) is not int or source["nlink"] != 1:
            raise ClosureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", role)
        for descriptor_key in ("before", "after"):
            descriptor = item.get(descriptor_key)
            if not isinstance(descriptor, Mapping) or \
                    set(descriptor) != descriptor_fields or \
                    descriptor.get("path") != mount["source"] + "/" + relative or \
                    descriptor.get("mode") != 0o444 or descriptor.get("nlink") != 1 or \
                    type(descriptor.get("bytes")) is not int or descriptor["bytes"] <= 0 or \
                    not SHA256_RE.fullmatch(str(descriptor.get("sha256"))):
                raise ClosureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", role)
            if descriptor.get("sha256") != source.get("sha256") or \
                    descriptor.get("bytes") != source.get("bytes"):
                raise ClosureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", role)
        if item["before"] != item["after"]:
            raise ClosureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", role)


def _validate_rosdep_prepare_input_bundle(bundle: Any) -> None:
    """Validate the fixed container-visible prepare bundle contract."""
    required = {
        "schema", "schema_version", "source_relative_root", "container_root",
        "mount", "copy_policy", "identity_policy", "files",
    }
    if not isinstance(bundle, Mapping) or set(bundle) != required or \
            bundle.get("schema") != ROSDEP_PREPARE_INPUT_SCHEMA or \
            bundle.get("schema_version") != 1 or \
            bundle.get("source_relative_root") != ROSDEP_PREPARE_INPUT_RELATIVE or \
            bundle.get("container_root") != ROSDEP_PREPARE_CONTAINER_ROOT or \
            bundle.get("copy_policy") != "atomic-regular-0444-single-link-host-owned" or \
            bundle.get("identity_policy") != "source-and-pre-post-copy-descriptor-exact":
        raise ClosureError("ROSDEP_PREPARE_INPUT_BUNDLE_INVALID", "contract")
    mount = bundle.get("mount")
    if not isinstance(mount, Mapping) or set(mount) != {
            "source_relative", "target", "read_only", "noexec",
            "repository_bind"} or mount != {
                "source_relative": ROSDEP_PREPARE_INPUT_RELATIVE,
                "target": ROSDEP_PREPARE_CONTAINER_ROOT,
                "read_only": True,
                "noexec": False,
                "repository_bind": "forbidden",
            }:
        raise ClosureError("ROSDEP_PREPARE_INPUT_BUNDLE_INVALID", "mount")
    files = bundle.get("files")
    if not isinstance(files, list) or len(files) != 4:
        raise ClosureError("ROSDEP_PREPARE_INPUT_BUNDLE_INVALID", "files")
    expected = [
        ("tool", ROSDEP_PREPARE_TOOL_RELATIVE, ROSDEP_PREPARE_CONTAINER_TOOL),
        ("schema", ROSDEP_PREPARE_SCHEMA_RELATIVE, ROSDEP_PREPARE_CONTAINER_SCHEMA),
        ("discovery_validator", ROSDEP_DISCOVERY_HELPER_RELATIVE,
         ROSDEP_PREPARE_CONTAINER_DISCOVERY),
        ("consumer_profile", "<active-profile>", ROSDEP_PREPARE_CONTAINER_PROFILE),
    ]
    for item, (role, source_path, container_path) in zip(files, expected):
        if not isinstance(item, Mapping) or item.get("role") != role or \
                item.get("source_path") != source_path or \
                item.get("container_path") != container_path:
            raise ClosureError("ROSDEP_PREPARE_INPUT_BUNDLE_INVALID", role)
        if role == "consumer_profile":
            if set(item) != {"role", "source_path", "container_path", "sha256_source"} or \
                    item.get("sha256_source") != "plan_profile_sha256":
                raise ClosureError("ROSDEP_PREPARE_INPUT_BUNDLE_INVALID", role)
        elif set(item) != {"role", "source_path", "container_path", "sha256"} or \
                not SHA256_RE.fullmatch(str(item.get("sha256"))):
            raise ClosureError("ROSDEP_PREPARE_INPUT_BUNDLE_INVALID", role)


def _validate_prepare_root_descriptor(value: Any, label: str) -> None:
    """Validate one logical or container-visible prepare root identity."""
    fields = {"path", "type", "mode", "uid", "gid", "nlink", "device", "inode"}
    if not isinstance(value, Mapping) or set(value) != fields:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label)
    _absolute(value.get("path"), label + " path")
    if value.get("type") != "directory" or value.get("mode") != 0o700:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label)
    for key in ("uid", "gid", "nlink", "device", "inode"):
        if type(value.get(key)) is not int or value[key] < 0:
            raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label)
    if value["nlink"] < 2 or value["inode"] < 1:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label)


def _validate_prepare_root_mount(value: Any, logical: Mapping[str, Any],
                                 transport: Mapping[str, Any]) -> None:
    """Bind the logical root and transport root through one identity tuple."""
    if not isinstance(value, Mapping) or set(value) != {
            "source", "target", "read_only", "type", "noexec", "identity"}:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "root mount")
    if (value.get("source") != logical.get("path") or
            value.get("target") != transport.get("path") or
            value.get("read_only") is not False or value.get("type") != "bind" or
            value.get("noexec") is not False):
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "root mount")
    identity = value.get("identity")
    expected = {
        key: transport[key]
        for key in ("device", "inode", "uid", "gid", "mode", "nlink")
    }
    if identity != expected:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "root mount identity")


def _validate_prepare_transport_descriptor(value: Any, label: str) -> None:
    """Validate the metadata descriptor visible through the discovery bind."""
    fields = {
        "path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
        "device", "inode", "sidecar",
    }
    if not isinstance(value, Mapping) or set(value) != fields:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label)
    _absolute(value.get("path"), label + " path")
    if (type(value.get("bytes")) is not int or value["bytes"] < 1 or
            value["bytes"] > 536870912 or
            not SHA256_RE.fullmatch(str(value.get("sha256"))) or
            value.get("mode") != 0o444 or value.get("nlink") != 1):
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label)
    for key in ("uid", "gid", "device", "inode"):
        if type(value.get(key)) is not int or value[key] < 0:
            raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label)
    if value["inode"] < 1:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label)
    sidecar = value.get("sidecar")
    if not isinstance(sidecar, Mapping) or set(sidecar) != fields - {"sidecar"}:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label + " sidecar")
    if (sidecar.get("path") != value["path"] + ".sha256" or
            type(sidecar.get("bytes")) is not int or sidecar["bytes"] < 1 or
            sidecar["bytes"] > 512 or
            not SHA256_RE.fullmatch(str(sidecar.get("sha256"))) or
            sidecar.get("mode") != 0o444 or sidecar.get("nlink") != 1):
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label + " sidecar")
    for key in ("uid", "gid", "device", "inode"):
        if type(sidecar.get(key)) is not int or sidecar[key] < 0:
            raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label + " sidecar")
    if sidecar["inode"] < 1:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", label + " sidecar")


def _validate_prepare_transport_artifact(value: Any, index: int,
                                         transport_root: str) -> None:
    """Validate one fixed discovery artifact's logical and transport paths."""
    if not isinstance(value, Mapping) or set(value) != {
            "role", "index", "relative_path", "transport_path",
            "descriptor", "transport_descriptor"}:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "transport artifact")
    expected_role = "archive" if index == 0 else (
        "package" if index < 5 else "selected_file")
    expected_index = 0 if index == 0 else (index - 1 if index < 5 else index - 5)
    if value.get("role") != expected_role or value.get("index") != expected_index:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "transport artifact")
    relative = _safe_relative(value.get("relative_path"), "transport relative path")
    expected_transport = transport_root + "/" + relative
    if value.get("transport_path") != expected_transport:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "transport path")
    descriptor = value.get("descriptor")
    if (not isinstance(descriptor, Mapping) or
            set(descriptor) != {
                "path", "sidecar", "bytes", "sha256", "mode", "nlink"} or
            descriptor.get("path") != relative or
            descriptor.get("sidecar") != relative + ".sha256" or
            type(descriptor.get("bytes")) is not int or descriptor["bytes"] < 1 or
            descriptor.get("mode") != 0o444 or descriptor.get("nlink") != 1 or
            not SHA256_RE.fullmatch(str(descriptor.get("sha256")))):
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "transport descriptor")
    transport_descriptor = value.get("transport_descriptor")
    _validate_prepare_transport_descriptor(
        transport_descriptor, "transport descriptor")
    if (transport_descriptor.get("path") != expected_transport or
            transport_descriptor.get("sidecar", {}).get("path") !=
            expected_transport + ".sha256" or
            any(transport_descriptor.get(key) != descriptor.get(key)
                for key in ("bytes", "sha256", "mode", "nlink"))):
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "transport descriptor binding")


def _validate_prepare_discovery_transport(value: Any,
                                          discovery: Mapping[str, Any]) -> None:
    """Validate the complete fixed discovery-root transport projection."""
    fields = {
        "schema", "schema_version", "logical_root", "transport_root",
        "mount", "artifact_count", "artifacts", "identity_sha256",
    }
    if not isinstance(value, Mapping) or set(value) != fields:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "discovery transport")
    logical_root = discovery.get("root")
    transport_root = value.get("transport_root")
    if (value.get("schema") != "registration-plugin-discovery-transport-binding-v1" or
            value.get("schema_version") != 1 or
            value.get("logical_root") != logical_root or
            transport_root != ROSDEP_PREPARE_CONTAINER_DISCOVERY_ROOT):
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "discovery transport identity")
    _absolute(logical_root, "discovery logical root")
    _absolute(transport_root, "discovery transport root")
    mount = value.get("mount")
    if not isinstance(mount, Mapping) or set(mount) != {
            "source", "target", "read_only", "type", "noexec"} or mount != {
                "source": logical_root, "target": transport_root,
                "read_only": True, "type": "bind", "noexec": False,
            }:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "discovery transport mount")
    if value.get("artifact_count") != 13 or not isinstance(value.get("artifacts"), list) or \
            len(value["artifacts"]) != 13:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "discovery transport artifacts")
    for index, artifact in enumerate(value["artifacts"]):
        _validate_prepare_transport_artifact(artifact, index, transport_root)
    if len({item["transport_path"] for item in value["artifacts"]}) != 13:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "transport artifact aliases")
    if value.get("identity_sha256") != canonical_hash(value, "identity_sha256"):
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "discovery transport hash")


def _validate_rosdep_prepare_binding(contract: Any, projection: Any) -> None:
    """Validate the additive offline rosdep-prepare capture projection."""
    required = {
        "schema", "schema_version", "required", "phase_order", "tool",
        "schema_binding", "input_bundle", "mount_policy", "capture_profile", "discovery", "execution",
        "output", "promotion", "identity_sha256",
    }
    if (not isinstance(contract, Mapping) or set(contract) != required or
            contract.get("schema") != "registration-plugin-rosdep-prepare-contract-v1" or
            contract.get("schema_version") != 1 or
            type(contract.get("required")) is not bool or
            contract.get("phase_order") != [
                "dependency_install", "network_disconnect", "rosdep_prepare",
                "rosdep_resolution",
            ] or contract.get("identity_sha256") != canonical_hash(
                contract, "identity_sha256")):
        raise ClosureError("ROSDEP_PREPARE_CONTRACT_INVALID", "contract shape")
    for key in ("tool", "schema_binding"):
        item = contract.get(key)
        if (not isinstance(item, Mapping) or set(item) != {"path", "sha256"} or
                not isinstance(item["path"], str) or
                not SHA256_RE.fullmatch(str(item["sha256"]))):
            raise ClosureError("ROSDEP_PREPARE_CONTRACT_INVALID", key)
    _validate_rosdep_prepare_input_bundle(contract.get("input_bundle"))
    if contract.get("mount_policy") != ROSDEP_PREPARE_MOUNT_POLICY:
        raise ClosureError("ROSDEP_PREPARE_MOUNT_POLICY_INVALID", "mount policy")
    expected_contract = rosdep_prepare_contract(
        required=contract["required"],
        profile_path=Path(contract["capture_profile"]["path"]),
        profile_sha256=contract["capture_profile"]["sha256"],
    )
    if contract.get("input_bundle") != expected_contract.get("input_bundle"):
        raise ClosureError("ROSDEP_PREPARE_INPUT_BUNDLE_INVALID", "source identity")
    profile = contract.get("capture_profile")
    if (not isinstance(profile, Mapping) or set(profile) != {"path", "sha256"} or
            not isinstance(profile.get("path"), str) or
            not SHA256_RE.fullmatch(str(profile.get("sha256")))):
        raise ClosureError("ROSDEP_PREPARE_CONTRACT_INVALID", "profile")
    execution = contract.get("execution")
    if (not isinstance(execution, Mapping) or
            execution.get("network") != "none" or
            execution.get("pull") is not False or execution.get("build") is not False or
            execution.get("network_disconnect_required") is not True or
            execution.get("fallback") != "forbidden"):
        raise ClosureError("ROSDEP_PREPARE_CONTRACT_INVALID", "execution")
    output = contract.get("output")
    if (not isinstance(output, Mapping) or
            output.get("receipt_relative") != (
                "rosdep-prepare/registration_plugin_rosdep_prepare.receipt.json") or
            output.get("source_and_cache") != "receipt-bound-sealed-descriptors" or
            output.get("deb_inputs") != "discovery-11-exact-four"):
        raise ClosureError("ROSDEP_PREPARE_CONTRACT_INVALID", "output")
    promotion = contract.get("promotion")
    if promotion != {
            "status": "REVIEW_REQUIRED", "benchmark_eligible": False,
            "claim_eligible": False, "active_profile_switch": False,
            "promotion": "FORBIDDEN_UNTIL_SIGNED_REVIEW"}:
        raise ClosureError("ROSDEP_PREPARE_CONTRACT_INVALID", "promotion")
    projection_fields = {
        "required", "status", "receipt", "canonical_sha256", "discovery",
        "root", "transport_root", "root_mount", "discovery_transport",
        "source_list", "cache_tree", "deb_artifacts", "input_binding",
    }
    if not isinstance(projection, Mapping) or set(projection) != projection_fields:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "fields")
    if projection.get("required") is not contract["required"]:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "required")
    if not contract["required"]:
        if projection != {
                "required": False, "status": "NOT_CONFIGURED", "receipt": None,
                "canonical_sha256": None, "discovery": None, "source_list": None,
                "cache_tree": None, "deb_artifacts": [], "input_binding": None,
                "root": None, "transport_root": None, "root_mount": None,
                "discovery_transport": None}:
            raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "unconfigured")
        return
    if projection["status"] not in {
            "NOT_RUN", "REVIEW_REQUIRED", "PARTIAL_FAILURE_REVIEW_REQUIRED"}:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "status")
    if projection["status"] == "NOT_RUN":
        if any(projection.get(key) is not None for key in (
                "receipt", "discovery", "root", "transport_root", "root_mount",
                "discovery_transport", "source_list", "cache_tree")) or \
                projection.get("deb_artifacts") != []:
            raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "not-run")
        if projection.get("input_binding") is not None:
            _validate_prepare_input_binding_shape(projection["input_binding"])
        return
    receipt = projection.get("receipt")
    if (not isinstance(receipt, Mapping) or
            not isinstance(projection.get("discovery"), Mapping) or
            not isinstance(projection.get("root"), Mapping) or
            not isinstance(projection.get("transport_root"), Mapping) or
            not isinstance(projection.get("root_mount"), Mapping) or
            not isinstance(projection.get("discovery_transport"), Mapping) or
            not isinstance(projection.get("deb_artifacts"), list) or
            projection.get("canonical_sha256") is None):
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "receipt")
    _validate_prepare_root_descriptor(projection["root"], "logical root")
    _validate_prepare_root_descriptor(projection["transport_root"], "transport root")
    if any(projection["root"].get(key) != projection["transport_root"].get(key)
           for key in ("type", "mode", "uid", "gid", "nlink", "device", "inode")):
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "root identity")
    _validate_prepare_root_mount(
        projection["root_mount"], projection["root"], projection["transport_root"])
    _validate_prepare_discovery_transport(
        projection["discovery_transport"], projection["discovery"])
    if projection.get("input_binding") is not None:
        _validate_prepare_input_binding_shape(projection["input_binding"])
    if projection["status"] == "REVIEW_REQUIRED" and receipt is None:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "success receipt")


def _capture_executor_reference(capture: Mapping[str, Any],
                                capture_descriptor: Mapping[str, Any],
                                reference_map: Mapping[str, Any] | None = None,
                                reference_counts: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Bind a composed closure to the capture-executor contract.

    The outer executor receipt is sealed after the closure is composed, so a
    direct receipt hash here would create a circular identity.  This
    projection instead binds the immutable capture-input bytes, the exact
    executor schema bytes, and the fixed non-promoting status.  The outer
    receipt independently binds the resulting closure and is reopened by the
    capture executor after its own seal.
    """
    schema_sha256 = _file_sha256(
        CAPTURE_EXECUTOR_SCHEMA_PATH, "capture executor schema", MAX_JSON_BYTES)
    if not isinstance(capture_descriptor, Mapping) or set(capture_descriptor) != {
            "path", "bytes", "sha256", "mode", "uid", "gid", "nlink", "device", "inode",
            "sidecar"}:
        raise ClosureError("CAPTURE_EXECUTOR_BINDING_INVALID", "capture input descriptor")
    if capture.get("status") != "CAPTURED_REVIEW_REQUIRED":
        raise ClosureError("CAPTURE_EXECUTOR_BINDING_INVALID", "capture status")
    if reference_map is None or reference_counts is None:
        reference_map, reference_counts = _empty_reference_projection()
    reference_map, reference_counts = _validate_reference_projection(
        reference_map, reference_counts, "capture executor reference projection")
    return {
        "schema": "registration-plugin-dependency-capture-executor-v1",
        "schema_version": 1,
        "schema_path": CAPTURE_EXECUTOR_SCHEMA_RELATIVE,
        "schema_sha256": schema_sha256,
        "contract_bindings": capture_contract_bindings(),
        "status": "REVIEW_REQUIRED",
        "benchmark_eligible": False,
        "capture_input": dict(capture_descriptor),
        "capture_input_sha256": capture_descriptor["sha256"],
        "reference_policy": dict(REFERENCE_POLICY),
        "reference_map": reference_map,
        "reference_counts": reference_counts,
        "distro": capture["distro"],
        "dependency_leg": capture["dependency_leg"],
        "image_digest": capture["image_digest"],
        "phase_count": len(capture["phase_records"]),
        "signature_status": "REQUIRED_NOT_PROVEN",
        "promotion": "FORBIDDEN_UNTIL_SIGNED_RECEIPT",
    }


def _write_new(path: Path, payload: bytes) -> dict[str, Any]:
    if path.exists() or path.is_symlink():
        raise ClosureError("OUTPUT_COLLISION", str(path))
    if not path.is_absolute() or not path.parent.is_dir() or path.parent.is_symlink():
        raise ClosureError("OUTPUT_PARENT_INVALID", str(path))
    _check_parent_components(path, "output")
    try:
        parent_before = path.parent.lstat()
    except OSError as error:
        raise ClosureError("OUTPUT_PARENT_INVALID", str(path)) from error
    if stat.S_ISLNK(parent_before.st_mode) or not stat.S_ISDIR(parent_before.st_mode):
        raise ClosureError("OUTPUT_PARENT_INVALID", str(path.parent))
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    fd = None
    created = None
    try:
        fd = os.open(str(path), flags, 0o600)
        created = os.fstat(fd)
        offset = 0
        while offset < len(payload):
            written = os.write(fd, payload[offset:])
            if written <= 0:
                raise ClosureError("OUTPUT_NO_PROGRESS", str(path))
            offset += written
        os.fsync(fd)
        os.fchmod(fd, 0o444)
        os.fsync(fd)
    except Exception as error:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
        if created is not None:
            try:
                info = path.lstat()
                if info.st_dev == created.st_dev and info.st_ino == created.st_ino:
                    path.unlink()
            except OSError:
                pass
        if isinstance(error, ClosureError):
            raise
        raise ClosureError("OUTPUT_WRITE_FAILED", str(path)) from error
    else:
        os.close(fd)
    try:
        parent_after = path.parent.lstat()
        if (parent_after.st_dev != parent_before.st_dev or
                parent_after.st_ino != parent_before.st_ino or
                stat.S_ISLNK(parent_after.st_mode) or
                not stat.S_ISDIR(parent_after.st_mode)):
            raise ClosureError("OUTPUT_PARENT_CHANGED", str(path.parent))
        directory_fd = os.open(
            str(path.parent), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) |
            getattr(os, "O_NOFOLLOW", 0))
        directory_info = os.fstat(directory_fd)
        if (directory_info.st_dev != parent_before.st_dev or
                directory_info.st_ino != parent_before.st_ino):
            os.close(directory_fd)
            raise ClosureError("OUTPUT_PARENT_CHANGED", str(path.parent))
        os.fsync(directory_fd)
        os.close(directory_fd)
    except OSError as error:
        raise ClosureError("OUTPUT_DIRSYNC_FAILED", str(path.parent)) from error
    return _file_descriptor(path, "sealed output", max(len(payload), 1))


def _seal_pair(root: Path, filename: str, value: Mapping[str, Any]) -> tuple[dict[str, Any], dict[str, Any]]:
    payload = canonical_bytes(value) + b"\n"
    path = root / filename
    descriptor = _write_new(path, payload)
    sidecar_payload = ("{}  {}\n".format(descriptor["sha256"], path.name)).encode("ascii")
    try:
        sidecar = _write_new(Path(str(path) + ".sha256"), sidecar_payload)
    except Exception:
        try:
            if path.lstat().st_ino == descriptor["inode"]:
                path.unlink()
        except OSError:
            pass
        raise
    return descriptor, sidecar


def _harden_tree(root: Path) -> None:
    root = _absolute(str(root), "output root")
    _check_parent_components(root / "placeholder", "output root")
    try:
        root_info = root.lstat()
    except OSError as error:
        raise ClosureError("OUTPUT_ROOT_INVALID", str(root)) from error
    if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
        raise ClosureError("OUTPUT_ROOT_INVALID", str(root))
    pending = [root]
    while pending:
        current = pending.pop()
        try:
            entries = sorted(os.scandir(str(current)), key=lambda item: item.name)
        except OSError as error:
            raise ClosureError("OUTPUT_ROOT_READ_FAILED", str(current)) from error
        for entry in entries:
            path = Path(entry.path)
            info = entry.stat(follow_symlinks=False)
            if stat.S_ISLNK(info.st_mode):
                raise ClosureError("OUTPUT_SYMLINK", str(path))
            if stat.S_ISDIR(info.st_mode):
                pending.append(path)
                continue
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise ClosureError("OUTPUT_FILE_INVALID", str(path))
            flags = os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
            fd = None
            try:
                fd = os.open(str(path), flags)
                before = os.fstat(fd)
                if before.st_dev != info.st_dev or before.st_ino != info.st_ino:
                    raise ClosureError("OUTPUT_FILE_CHANGED", str(path))
                os.fchmod(fd, 0o444)
                os.fsync(fd)
                after = os.fstat(fd)
            except OSError as error:
                raise ClosureError("OUTPUT_HARDEN_FAILED", str(path)) from error
            finally:
                if fd is not None:
                    try:
                        os.close(fd)
                    except OSError:
                        pass
            if (after.st_dev != info.st_dev or after.st_ino != info.st_ino or
                    after.st_nlink != 1 or stat.S_IMODE(after.st_mode) != 0o444):
                raise ClosureError("OUTPUT_FILE_CHANGED", str(path))
        try:
            directory_fd = os.open(
                str(current), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) |
                getattr(os, "O_NOFOLLOW", 0))
            os.fsync(directory_fd)
            os.close(directory_fd)
        except OSError as error:
            raise ClosureError("OUTPUT_DIRSYNC_FAILED", str(current)) from error


def _assert_exact_tree_shape(root: Path, expected_files: set[str],
                             expected_directories: set[str], label: str) -> None:
    """Reject output files not covered by an immutable directory contract."""
    root = _absolute(str(root), label)
    try:
        root_info = root.lstat()
    except OSError as error:
        raise ClosureError("OUTPUT_ROOT_INVALID", str(root)) from error
    if (stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode) or
            stat.S_IMODE(root_info.st_mode) != 0o700):
        raise ClosureError("OUTPUT_ROOT_INVALID", label)
    files = set()
    directories = set()
    pending = [root]
    while pending:
        current = pending.pop()
        try:
            entries = sorted(os.scandir(str(current)), key=lambda item: item.name)
        except OSError as error:
            raise ClosureError("OUTPUT_ROOT_READ_FAILED", label) from error
        for entry in entries:
            child = Path(entry.path)
            info = entry.stat(follow_symlinks=False)
            relative = child.relative_to(root).as_posix()
            _safe_relative(relative, "{} entry".format(label))
            if stat.S_ISLNK(info.st_mode):
                raise ClosureError("OUTPUT_SYMLINK", relative)
            if stat.S_ISDIR(info.st_mode):
                if (relative in expected_directories and
                        stat.S_IMODE(info.st_mode) != 0o700):
                    raise ClosureError("OUTPUT_DIRECTORY_MODE_INVALID", relative)
                directories.add(relative)
                pending.append(child)
                continue
            if not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
                raise ClosureError("OUTPUT_FILE_INVALID", relative)
            files.add(relative)
    if files != expected_files or directories != expected_directories:
        raise ClosureError("OUTPUT_ALLOWLIST_MISMATCH", json.dumps({
            "expected_files": sorted(expected_files),
            "observed_files": sorted(files),
            "expected_directories": sorted(expected_directories),
            "observed_directories": sorted(directories),
        }, sort_keys=True))
    for relative in sorted(files):
        maximum = MAX_JSON_BYTES if relative.endswith(".json") else 512
        _file_descriptor(root / relative, "{} {}".format(label, relative), maximum)


def _assert_exact_output_shape(root: Path, label: str = "closure output") -> None:
    """Reject any closure output file not covered by its allowlist."""
    _assert_exact_tree_shape(root, set(CLOSURE_FILE_ALLOWLIST),
                             set(CLOSURE_DIRECTORY_ALLOWLIST), label)


def _load_capture(capture_root: Path) -> tuple[dict[str, Any], dict[str, Any]]:
    capture_root = _absolute(str(capture_root), "capture root")
    try:
        info = capture_root.lstat()
    except OSError as error:
        raise ClosureError("CAPTURE_ROOT_MISSING", str(capture_root)) from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise ClosureError("CAPTURE_ROOT_INVALID", str(capture_root))
    value, descriptor = _read_json(capture_root / "capture-input.json",
                                   "capture input", sidecar=True)
    _validate_capture(value)
    descriptor["sidecar"] = _sidecar(capture_root / "capture-input.json", descriptor["sha256"])
    return value, descriptor


def compose_closure(*, capture_root: Path, output_root: Path,
                    dpkg_reader: Callable[[Path], Mapping[str, Any]] = _default_dpkg_reader) -> dict[str, Any]:
    """Compose a fresh review-required closure from a sealed capture packet.

    ``capture_root`` and all of its referenced files must already be immutable
    regular files.  This function never creates or overwrites a capture input.
    """
    capture_root = _absolute(str(capture_root), "capture root")
    output_root = _absolute(str(output_root), "closure output root")
    if output_root.exists() or output_root.is_symlink() or not output_root.parent.is_dir():
        raise ClosureError("OUTPUT_ROOT_NOT_FRESH", str(output_root))
    _check_parent_components(output_root, "closure output root")
    capture, capture_descriptor = _load_capture(capture_root)
    capture_reference_map, capture_reference_counts = _capture_reference_projection(
        capture_root)
    apt_source_url_policy = _validate_apt_source_url_policy(
        capture.get("apt_source_url_policy", APT_SOURCE_URL_POLICY),
        "capture URL policy")
    distro = capture["distro"]
    dependency_leg = capture["dependency_leg"]
    requirements = capture_root / capture["requirements_path"]
    base_status = capture_root / capture["base_status_path"]
    source_root = capture_root / capture["source_root"]
    source_manifest = capture_root / capture["source_manifest_path"]
    for path, label in ((requirements, "requirements"), (base_status, "base status"),
                        (source_manifest, "source manifest")):
        _file_descriptor(path, label, MAX_JSON_BYTES)
    _tree_descriptor(
        source_root, "apt source snapshot",
        maximum=MAX_DECODED_PACKAGES_BYTES, allow_empty_files=True)
    resolver_paths = {}
    resolver_logs = {}
    deb_roots = {}
    for role in ROLES:
        item = capture["resolvers"][role]
        resolver_paths[role] = capture_root / item["resolver_path"]
        resolver_logs[role] = capture_root / item["raw_log_path"]
        deb_roots[role] = capture_root / item["deb_root_path"]
        _file_descriptor(resolver_paths[role], "{} resolver".format(role), MAX_JSON_BYTES)
        _file_descriptor(resolver_logs[role], "{} resolver log".format(role), MAX_LOG_BYTES)
        _deb_tree_descriptor(deb_roots[role], "{} downloaded debs".format(role))
    output_root.mkdir(mode=0o700)
    apt_root = output_root / "apt"
    try:
        apt_result = APT.compose_proposal(
            requirements_path=requirements, base_status_path=base_status,
            build_resolver_path=resolver_paths["build"], runtime_resolver_path=resolver_paths["runtime"],
            build_resolver_log_path=resolver_logs["build"], runtime_resolver_log_path=resolver_logs["runtime"],
            source_root=source_root, source_manifest_path=source_manifest,
            build_deb_root=deb_roots["build"], runtime_deb_root=deb_roots["runtime"],
            output_root=apt_root, dpkg_reader=dpkg_reader)
        del apt_result
        _harden_tree(apt_root)
        _assert_exact_tree_shape(
            apt_root, set(APT_FILE_ALLOWLIST), set(), "apt output")
        apt_descriptors = _apt_output_descriptors(apt_root)
        phase_records = _phase_log_descriptors(capture_root, capture["phase_records"])
        input_bindings = {
            "capture_input": capture_descriptor,
            "requirements": _file_descriptor(requirements, "requirements", MAX_JSON_BYTES),
            "base_status": _file_descriptor(base_status, "base status", MAX_JSON_BYTES),
            "source_manifest": _file_descriptor(source_manifest, "source manifest", MAX_JSON_BYTES),
            "source_root": _tree_descriptor(
                source_root, "apt source snapshot",
                maximum=MAX_DECODED_PACKAGES_BYTES, allow_empty_files=True),
            "resolvers": {}, "resolver_logs": {}, "deb_roots": {},
        }
        for role in ROLES:
            input_bindings["resolvers"][role] = _file_descriptor(
                resolver_paths[role], "{} resolver".format(role), MAX_JSON_BYTES)
            input_bindings["resolver_logs"][role] = _file_descriptor(
                resolver_logs[role], "{} resolver log".format(role), MAX_LOG_BYTES)
            input_bindings["deb_roots"][role] = _deb_tree_descriptor(
                deb_roots[role], "{} downloaded debs".format(role))
        requested_urls, final_urls = _requested_urls(apt_root)
        optional = {"status": "NOT_APPLICABLE"}
        if dependency_leg == "present":
            prefetch = capture["optional_prefetch"]
            prefetch_root = capture_root / prefetch["root_path"]
            dependencies = list(prefetch["dependencies"])
            try:
                import registration_plugin_dependency_prefetch as prefetch_module
                prefetch_result = prefetch_module.validate_prefetch_root(
                    prefetch_root, dependencies, distro, capture["image_digest"],
                    authority="HOST_PROMOTION")
            except Exception as error:
                raise ClosureError("PREFETCH_INVALID", str(error)) from error
            optional = {
                "status": "SEALED_PREFETCH", "root": str(prefetch_root),
                "receipt_sha256": prefetch_result["receipt_sha256"],
                "manifest_sha256": prefetch_result["manifest_sha256"],
                "records": prefetch_result["records"],
            }
            for record in prefetch_result["records"]:
                # The prefetch validator has already enforced exact pinned URLs;
                # this projection is only the closure's URL audit trail.
                requested_urls.append(record["name"] + ":" + record["archive_sha256"])
            requested_urls = sorted(set(requested_urls))
        signature = {
            "status": "REQUIRED_NOT_PROVEN", "runtime": "NOT_RUN",
            "required": True, "promotion": "FORBIDDEN_UNTIL_SIGNED_RECEIPT",
        }
        closure = {
            "schema": SCHEMA, "schema_version": SCHEMA_VERSION,
            "status": REVIEW_STATUS, "benchmark_eligible": False,
            "distro": distro, "dependency_leg": dependency_leg,
            "base_image": {"digest": capture["image_digest"], "platform": PLATFORM},
            "capture_input": capture_descriptor,
            "capture_executor": _capture_executor_reference(
                capture, capture_descriptor, capture_reference_map,
                capture_reference_counts),
            "capture_contract": capture_contract(),
            "input_bindings": input_bindings,
            "apt": apt_descriptors,
            "apt_source_url_policy": apt_source_url_policy,
            "release_declaration_policy": dict(RELEASE_DECLARATION_POLICY),
            "packages_format_policy": dict(PACKAGES_FORMAT_POLICY),
            "gpgv_status_policy": dict(GPGV_STATUS_POLICY),
            "verification_chain": dict(VERIFICATION_CHAIN),
            "phase_records": phase_records,
            "optional_prefetch": optional,
            "requested_urls": sorted(set(requested_urls)),
            "final_urls": sorted(set(final_urls)),
            "signature_verification": signature,
            "runtime_policy": {
                "docker_pull": False, "network": "none",
                "required_flags": ["--pull=never", "--network=none"],
                "build_test_network_connected": False,
                "build_test_network_used": False,
            },
            "provisioning_policy": {
                "network_used": True, "archive_fetch_network_used": dependency_leg == "present",
                "max_retries": 0, "max_command_timeout_seconds": 600,
                "redirect_policy": "same-host-only-and-requested-url-recorded",
            },
        }
        if "rosdep_prepare_contract" in capture:
            closure["rosdep_prepare_contract"] = _immutable_copy(
                capture["rosdep_prepare_contract"])
            closure["rosdep_prepare"] = _immutable_copy(
                capture["rosdep_prepare"])
        closure["canonical_sha256"] = canonical_hash(closure)
        _seal_pair(output_root, CLOSURE_FILENAME, closure)
        _harden_tree(output_root)
        return validate_closure_root(output_root, distro=distro,
                                     image_digest=capture["image_digest"],
                                     dependency_leg=dependency_leg)
    except Exception:
        # Keep an owned partial root for diagnosis; no later call may reuse it.
        raise


def _validate_apt_binding(root: Path, value: Mapping[str, Any]) -> dict[str, Any]:
    apt_root = root / "apt"
    if not apt_root.is_dir() or apt_root.is_symlink():
        raise ClosureError("APT_OUTPUT_INVALID", str(apt_root))
    descriptors = value.get("apt")
    if not isinstance(descriptors, Mapping) or set(descriptors) != {
            "proposal", "allowlist", "ledger", "closure_identity_sha256"}:
        raise ClosureError("APT_OUTPUT_INVALID", "descriptor fields")
    actual = _apt_output_descriptors(apt_root)
    if dict(descriptors) != actual:
        raise ClosureError("APT_OUTPUT_DRIFT", "apt output descriptors")
    return actual


def validate_closure_root(root: Path, *, distro: str, image_digest: str,
                          dependency_leg: str) -> dict[str, Any]:
    """Reopen a review-required closure and all of its captured bytes."""
    root = _absolute(str(root), "closure root")
    if not root.is_dir() or root.is_symlink():
        raise ClosureError("CLOSURE_ROOT_INVALID", str(root))
    _assert_exact_output_shape(root)
    value, _ = _read_json(root / CLOSURE_FILENAME,
                           "dependency closure")
    closure_path = root / CLOSURE_FILENAME
    _sidecar(closure_path, sha256_bytes(_read_regular(
        closure_path, "dependency closure", MAX_JSON_BYTES)[0]))
    required = {
        "schema", "schema_version", "status", "benchmark_eligible", "distro",
        "dependency_leg", "base_image", "capture_input", "input_bindings", "apt",
        "apt_source_url_policy", "release_declaration_policy", "packages_format_policy",
        "gpgv_status_policy",
        "verification_chain", "phase_records",
        "optional_prefetch", "requested_urls", "final_urls",
        "signature_verification", "runtime_policy", "provisioning_policy",
        "canonical_sha256", "capture_contract",
    }
    allowed = required | {
        "capture_executor", "rosdep_prepare_contract", "rosdep_prepare"
    }
    if not set(value).issubset(allowed) or not required.issubset(value) or \
            value.get("schema") != SCHEMA or \
            value.get("schema_version") != SCHEMA_VERSION or \
            value.get("status") != REVIEW_STATUS or value.get("benchmark_eligible") is not False or \
            value.get("canonical_sha256") != canonical_hash(value):
        raise ClosureError("CLOSURE_IDENTITY_INVALID", "closure fields/status")
    if value["distro"] != distro or value["dependency_leg"] != dependency_leg:
        raise ClosureError("CLOSURE_LEG_MISMATCH", "distro/dependency leg")
    if value["apt_source_url_policy"] != _validate_apt_source_url_policy(
            value["apt_source_url_policy"], "closure URL policy"):
        raise ClosureError("APT_URL_POLICY_INVALID", "closure URL policy")
    if value["release_declaration_policy"] != RELEASE_DECLARATION_POLICY:
        raise ClosureError("RELEASE_DECLARATION_POLICY_INVALID", "closure declarations")
    if value["packages_format_policy"] != PACKAGES_FORMAT_POLICY:
        raise ClosureError("PACKAGES_FORMAT_POLICY_INVALID", "closure packages policy")
    if value["gpgv_status_policy"] != GPGV_STATUS_POLICY:
        raise ClosureError("GPGV_STATUS_POLICY_INVALID", "closure gpgv policy")
    if value["verification_chain"] != VERIFICATION_CHAIN:
        raise ClosureError("CLOSURE_VERIFICATION_CHAIN_INVALID", "verification chain")
    base_image = value["base_image"]
    if (not isinstance(base_image, Mapping) or set(base_image) != {"digest", "platform"} or
            base_image["digest"] != image_digest or base_image["platform"] != PLATFORM):
        raise ClosureError("CLOSURE_IMAGE_MISMATCH", "base image")
    _require_digest(image_digest, "expected image digest")
    input_bindings = value["input_bindings"]
    if not isinstance(input_bindings, Mapping) or set(input_bindings) != {
            "capture_input", "requirements", "base_status", "source_manifest",
            "source_root", "resolvers", "resolver_logs", "deb_roots"}:
        raise ClosureError("CLOSURE_INPUTS_INVALID", "input bindings")
    _verify_descriptor(value["capture_input"], "capture input", MAX_JSON_BYTES,
                       require_sidecar=True)
    capture_path = Path(value["capture_input"]["path"])
    capture_root = capture_path.parent
    capture, _ = _load_capture(capture_root)
    if capture["distro"] != distro or capture["dependency_leg"] != dependency_leg or \
            capture["image_digest"] != image_digest:
        raise ClosureError("CLOSURE_CAPTURE_MISMATCH", "capture identity")
    if "capture_executor" in value:
        reference_map, reference_counts = _capture_reference_projection(capture_root)
        expected_executor = _capture_executor_reference(
            capture, value["capture_input"], reference_map, reference_counts)
        if value["capture_executor"] != expected_executor:
            raise ClosureError("CAPTURE_EXECUTOR_BINDING_INVALID", "executor reference")
    if value.get("capture_contract") != capture_contract():
        raise ClosureError("CAPTURE_CONTRACT_BINDING_INVALID", "capture contract")
    prepare_contract_present = "rosdep_prepare_contract" in value
    prepare_projection_present = "rosdep_prepare" in value
    if prepare_contract_present != prepare_projection_present:
        raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "closure input pair")
    if prepare_contract_present:
        _validate_rosdep_prepare_binding(
            value["rosdep_prepare_contract"], value["rosdep_prepare"])
        if (capture.get("rosdep_prepare_contract") !=
                value["rosdep_prepare_contract"] or
                capture.get("rosdep_prepare") != value["rosdep_prepare"]):
            raise ClosureError("ROSDEP_PREPARE_PROJECTION_INVALID", "closure drift")
    expected_rel = {
        "requirements": capture["requirements_path"],
        "base_status": capture["base_status_path"],
        "source_manifest": capture["source_manifest_path"],
    }
    for key, relative in expected_rel.items():
        descriptor = input_bindings[key]
        if descriptor.get("path") != str(capture_root / relative):
            raise ClosureError("CLOSURE_INPUTS_INVALID", key)
        _verify_descriptor(descriptor, key, MAX_JSON_BYTES)
    _verify_tree(input_bindings["source_root"], "apt source snapshot",
                 allow_empty_files=True, maximum=MAX_DECODED_PACKAGES_BYTES)
    if input_bindings["source_root"]["root"] != str(capture_root / capture["source_root"]):
        raise ClosureError("CLOSURE_INPUTS_INVALID", "source root")
    for role in ROLES:
        for collection, key, maximum in (
                ("resolvers", "resolver_path", MAX_JSON_BYTES),
                ("resolver_logs", "raw_log_path", MAX_LOG_BYTES)):
            descriptor = input_bindings[collection][role]
            expected_path = capture_root / capture["resolvers"][role][key]
            if descriptor.get("path") != str(expected_path):
                raise ClosureError("CLOSURE_INPUTS_INVALID", "{} {}".format(collection, role))
            _verify_descriptor(descriptor, "{} {}".format(collection, role), maximum)
        tree = input_bindings["deb_roots"][role]
        if tree.get("root") != str(capture_root / capture["resolvers"][role]["deb_root_path"]):
            raise ClosureError("CLOSURE_INPUTS_INVALID", "deb root")
        _verify_deb_tree(tree, "{} downloaded debs".format(role))
    _validate_apt_binding(root, value)
    _validate_phase_records_for_reopen(
        value["phase_records"], capture_root, dependency_leg,
        capture["phase_records"])
    optional = value["optional_prefetch"]
    if dependency_leg == "absent":
        if optional != {"status": "NOT_APPLICABLE"}:
            raise ClosureError("CLOSURE_PREFETCH_INVALID", "absent prefetch")
    else:
        _validate_prefetch_binding(optional, capture, capture_root)
    expected_urls, expected_finals = _requested_urls(root / "apt")
    if dependency_leg == "present":
        for record in optional["records"]:
            expected_urls.append(record["name"] + ":" + record["archive_sha256"])
        expected_urls = sorted(set(expected_urls))
    urls = value["requested_urls"]
    finals = value["final_urls"]
    if not isinstance(urls, list) or urls != sorted(set(urls)) or \
            not isinstance(finals, list) or finals != sorted(set(finals)):
        raise ClosureError("CLOSURE_URLS_INVALID", "URL projection")
    if urls != sorted(set(expected_urls)) or finals != sorted(set(expected_finals)):
        raise ClosureError("CLOSURE_URL_BINDING_INVALID", "URL projection drift")
    for url in urls + finals:
        _validate_closure_url_token(url)
    signature = value["signature_verification"]
    if not isinstance(signature, Mapping) or set(signature) != {
            "status", "runtime", "required", "promotion"} or \
            signature != {
                "status": "REQUIRED_NOT_PROVEN", "runtime": "NOT_RUN",
                "required": True, "promotion": "FORBIDDEN_UNTIL_SIGNED_RECEIPT"}:
        raise ClosureError("CLOSURE_SIGNATURE_INVALID", "signature requirement")
    runtime = value["runtime_policy"]
    if runtime != {
            "docker_pull": False, "network": "none",
            "required_flags": ["--pull=never", "--network=none"],
            "build_test_network_connected": False,
            "build_test_network_used": False}:
        raise ClosureError("CLOSURE_RUNTIME_POLICY_INVALID", "runtime policy")
    provisioning = value["provisioning_policy"]
    if (not isinstance(provisioning, Mapping) or set(provisioning) != {
            "network_used", "archive_fetch_network_used", "max_retries",
            "max_command_timeout_seconds", "redirect_policy"} or
            provisioning["network_used"] is not True or
            provisioning["archive_fetch_network_used"] is not (dependency_leg == "present") or
            provisioning["max_retries"] != 0 or
            provisioning["max_command_timeout_seconds"] != 600 or
            provisioning["redirect_policy"] != "same-host-only-and-requested-url-recorded"):
        raise ClosureError("CLOSURE_PROVISIONING_POLICY_INVALID", "provisioning policy")
    return {
        "status": REVIEW_STATUS, "benchmark_eligible": False,
        "distro": distro, "dependency_leg": dependency_leg,
        "closure_sha256": sha256_bytes(_read_regular(
            closure_path, "dependency closure", MAX_JSON_BYTES)[0]),
        "apt_closure_identity_sha256": value["apt"]["closure_identity_sha256"],
    }


def _validate_phase_records_for_reopen(value: Any, capture_root: Path,
                                       dependency_leg: str,
                                       capture_records: list[dict[str, Any]]) -> None:
    phase_count = len(capture_records)
    if any(item.get("phase") == "rosdep_prepare" for item in capture_records
           if isinstance(item, Mapping)):
        phase_order = EXPECTED_PHASES if phase_count == len(EXPECTED_PHASES) else (
            PRE_RESTORE_EXPECTED_PHASES if phase_count == len(PRE_RESTORE_EXPECTED_PHASES)
            else EXPECTED_PHASES)
    elif phase_count == len(PRE_RESTORE_EXPECTED_PHASES):
        phase_order = PRE_RESTORE_EXPECTED_PHASES
    else:
        phase_order = LEGACY_EXPECTED_PHASES
    if not isinstance(value, list) or len(value) != len(phase_order):
        raise ClosureError("PHASE_ORDER_INVALID", "sealed phase records")
    for index, item in enumerate(value):
        if not isinstance(item, Mapping) or set(item) != {
                "phase", "argv", "argv_sha256", "returncode", "network_used",
                "timeout_seconds", "attempts", "stdout", "stderr",
                "stdout_sha256", "stderr_sha256"}:
            raise ClosureError("PHASE_RECORD_INVALID", str(index))
        if item["phase"] != phase_order[index]:
            raise ClosureError("PHASE_ORDER_INVALID", item.get("phase"))
        captured = capture_records[index]
        for key in ("phase", "argv", "argv_sha256", "returncode",
                    "network_used", "timeout_seconds", "attempts"):
            if item[key] != captured[key]:
                raise ClosureError("PHASE_BINDING_INVALID", item["phase"])
        if item["stdout"]["path"] != str(capture_root / captured["stdout_path"]):
            raise ClosureError("PHASE_LOG_BINDING_INVALID", item["phase"])
        if item["stderr"]["path"] != str(capture_root / captured["stderr_path"]):
            raise ClosureError("PHASE_LOG_BINDING_INVALID", item["phase"])
        _validate_command(item["argv"], item["phase"])
        if item["stdout_sha256"] != item["stdout"].get("sha256") or \
                item["stderr_sha256"] != item["stderr"].get("sha256"):
            raise ClosureError("PHASE_LOG_BINDING_INVALID", item["phase"])
        _verify_descriptor(item["stdout"], item["phase"] + " stdout", MAX_LOG_BYTES)
        _verify_descriptor(item["stderr"], item["phase"] + " stderr", MAX_LOG_BYTES)
        _validate_phase_record({
            "phase": item["phase"], "argv": item["argv"],
            "argv_sha256": item["argv_sha256"], "returncode": item["returncode"],
            "network_used": item["network_used"],
            "timeout_seconds": item["timeout_seconds"], "attempts": item["attempts"],
            "stdout_path": "x", "stderr_path": "y",
        }, index, dependency_leg, phase_order)


def _validate_prefetch_binding(value: Any, capture: Mapping[str, Any], capture_root: Path) -> None:
    if not isinstance(value, Mapping) or set(value) != {
            "status", "root", "receipt_sha256", "manifest_sha256", "records"} or \
            value["status"] != "SEALED_PREFETCH":
        raise ClosureError("CLOSURE_PREFETCH_INVALID", "prefetch descriptor")
    _require_sha(value["receipt_sha256"], "prefetch receipt SHA")
    _require_sha(value["manifest_sha256"], "prefetch manifest SHA")
    prefetch = capture["optional_prefetch"]
    expected_root = capture_root / prefetch["root_path"]
    if value["root"] != str(expected_root):
        raise ClosureError("CLOSURE_PREFETCH_INVALID", "prefetch root")
    try:
        import registration_plugin_dependency_prefetch as prefetch_module
        result = prefetch_module.validate_prefetch_root(
            expected_root, prefetch["dependencies"], capture["distro"],
            capture["image_digest"], authority="HOST_PROMOTION")
    except Exception as error:
        raise ClosureError("CLOSURE_PREFETCH_INVALID", str(error)) from error
    if (result["receipt_sha256"] != value["receipt_sha256"] or
            result["manifest_sha256"] != value["manifest_sha256"] or
            result["records"] != value["records"]):
        raise ClosureError("CLOSURE_PREFETCH_INVALID", "prefetch identity drift")


def validate_runtime_closure(root: Path, *, distro: str, image_digest: str,
                             dependency_leg: str,
                             capture_receipt_root: Path | None = None) -> dict[str, Any]:
    """Validate the promotion-level closure required before a leg starts.

    Current composer output deliberately cannot satisfy this function because
    it has no independent Release signature receipt.  A reviewed external
    receipt may produce a new immutable ``PASS`` closure; this function never
    treats a status edit or self-rehash as that review.  When the optional
    capture receipt root is supplied, it is reopened first so a malformed or
    mutable capture cannot be mistaken for a runtime closure.  Its
    ``REVIEW_REQUIRED`` result is still deliberately non-promoting.
    """
    validate_closure_root(root, distro=distro, image_digest=image_digest,
                          dependency_leg=dependency_leg)
    if capture_receipt_root is not None:
        validate_capture_executor_receipt(capture_receipt_root)
    raise ClosureError("CLOSURE_REVIEW_REQUIRED", "dependency closure has no signed runtime promotion")


def validate_capture_executor_receipt(root: Path) -> dict[str, Any]:
    """Reopen one capture-executor receipt at the closure integration boundary.

    This adapter intentionally returns only the capture module's non-promoting
    result.  It is kept separate from ``validate_runtime_closure`` so callers
    cannot accidentally use a provisioning receipt as runtime authorization.
    """
    capture_path = ROOT / "scripts" / "capture_registration_plugin_dependency_closure.py"
    spec = importlib.util.spec_from_file_location(
        "registration_plugin_dependency_capture_for_closure", capture_path)
    if spec is None or spec.loader is None:
        raise ClosureError("CAPTURE_EXECUTOR_MISSING", str(capture_path))
    module = importlib.util.module_from_spec(spec)
    try:
        spec.loader.exec_module(module)
        result = module.validate_receipt(Path(root))
    except ClosureError:
        raise
    except Exception as error:
        raise ClosureError("CAPTURE_EXECUTOR_INVALID", str(error)) from error
    if result.get("benchmark_eligible") is not False or result.get("status") not in {
            "REVIEW_REQUIRED", "PARTIAL_FAILURE_REVIEW_REQUIRED"}:
        raise ClosureError("CAPTURE_EXECUTOR_PROMOTION_INVALID", "non-promoting status")
    return result


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    compose = sub.add_parser("compose")
    compose.add_argument("--capture-root", type=Path, required=True)
    compose.add_argument("--output-root", type=Path, required=True)
    validate = sub.add_parser("validate")
    validate.add_argument("--root", type=Path, required=True)
    validate.add_argument("--distro", choices=sorted(DISTROS), required=True)
    validate.add_argument("--dependency-leg", choices=sorted(LEGS), required=True)
    validate.add_argument("--image-digest", required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "compose":
            result = compose_closure(capture_root=args.capture_root, output_root=args.output_root)
        else:
            result = validate_closure_root(
                args.root, distro=args.distro, image_digest=args.image_digest,
                dependency_leg=args.dependency_leg)
        print(json.dumps(result, sort_keys=True))
        return 0
    except Exception as error:
        print(json.dumps({"status": "FAIL_CLOSED",
                          "kind": getattr(error, "kind", "CLOSURE_FAILURE"),
                          "message": str(error)}, sort_keys=True), file=sys.stderr)
        return 1


if __name__ == "__main__":
    raise SystemExit(main())

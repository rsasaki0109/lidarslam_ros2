#!/usr/bin/env python3
"""Capture a review-only registration-plugin dependency closure.

This is the production boundary between a connected provisioning container
and the existing, filesystem-only dependency closure composer.  The command
set is constructed from the current matrix profile; callers cannot provide a
shell fragment, image, package repository, or retry policy.  A provisioning
capture is never a runtime authorization: every successful or partial output
is sealed as ``REVIEW_REQUIRED`` and the existing runtime closure gate still
requires an independent signed review.

The default executor uses only ``subprocess.run(..., shell=False)`` with
bounded output and timeouts.  Tests inject a runner and never start Docker or
make network requests.  The optional in-container helper is intentionally
filesystem-only; it is not a substitute for the host's outer receipt.
"""

from __future__ import annotations

import argparse
import base64
import copy
import hashlib
import importlib.util
import json
import lzma
import os
from pathlib import Path
import re
import shlex
import stat
import subprocess
import sys
import time
from typing import Any, Callable, Mapping
import urllib.error
import urllib.request
import zlib
from urllib.parse import unquote, urljoin, urlsplit


ROOT = Path(__file__).resolve().parents[1]
AUDIT_PATH = ROOT / "scripts" / "audit_registration_plugin_matrix.py"
CLOSURE_PATH = ROOT / "scripts" / "registration_plugin_dependency_closure.py"
PREFETCH_PATH = ROOT / "scripts" / "registration_plugin_dependency_prefetch.py"
EVIDENCE_DIRECTORY_PATH = ROOT / "scripts" / "registration_plugin_evidence_directory.py"
ROSDEP_PREPARE_PATH = ROOT / "scripts" / "prepare_registration_plugin_rosdep.py"
ROSDEP_PREPARE_SCHEMA_PATH = ROOT / (
    "configs/slam_benchmark_profiles/registration_plugin_rosdep_prepare_v1.schema.json"
)
ROSDEP_DISCOVERY_HELPER_PATH = ROOT / (
    "scripts/capture_registration_plugin_rosdep_discovery.py"
)
PROFILE_PATH = ROOT / (
    "configs/slam_benchmark_profiles/registration_plugin_matrix_current_2026-08.json"
)
SCHEMA = "registration-plugin-dependency-capture-executor-v1"
SCHEMA_VERSION = 1
RECEIPT_NAME = "registration_plugin_dependency_capture.receipt.json"
CAPTURE_INPUT_NAME = "capture-input.json"
ROSDEP_PREPARE_RECEIPT_NAME = "registration_plugin_rosdep_prepare.receipt.json"
ROSDEP_PREPARE_RELATIVE = "rosdep-prepare"
ROSDEP_PREPARE_INPUT_RELATIVE = ROSDEP_PREPARE_RELATIVE + "/input"
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
# The discovery receipt is a host-owned input.  Its producer path is retained
# in the logical binding, while every command executed in the container uses
# this fixed read-only transport path.
ROSDEP_PREPARE_CONTAINER_DISCOVERY_ROOT = "/opt/registration-plugin/discovery"
ROSDEP_PREPARE_CONTAINER_OUTPUT_ROOT = (
    "/workspace/evidence-parent/" + ROSDEP_PREPARE_RELATIVE
)
ROSDEP_PREPARE_CONTAINER_PROFILE = (
    ROSDEP_PREPARE_CONTAINER_ROOT + "/consumer-profile.json"
)
ROSDEP_PREPARE_INPUT_SCHEMA = "registration-plugin-rosdep-prepare-input-v1"
MAX_RECEIPT_BYTES = 8 * 1024 * 1024
MAX_LOG_BYTES = 8 * 1024 * 1024
MAX_JSON_BYTES = 16 * 1024 * 1024
MAX_OUTPUT_BYTES = 512 * 1024 * 1024
MAX_CAPTURE_FILE_BYTES = 64 * 1024 * 1024
MAX_DECODED_PACKAGES_BYTES = 256 * 1024 * 1024
MAX_CAPTURE_DEB_BYTES = 512 * 1024 * 1024
# This is a signed-metadata declaration bound, not a file allocation or
# download limit.  The actual materialized Release/Packages index and captured
# deb bytes remain subject to MAX_CAPTURE_FILE_BYTES/MAX_CAPTURE_DEB_BYTES.
# Ubuntu Contents indexes and unrelated package records in signed metadata can
# declare multi-GiB objects, so the declaration-only ceiling is fixed at
# exactly 16 GiB.  This does not authorize allocating or downloading a file of
# that size.
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
# A Packages index is fetched only in one of these formats, in this exact
# order.  The order is part of the source-capture contract: apt's local lz4
# cache is deliberately not an accepted input because no external decoder is
# permitted in the in-container helper.
PACKAGES_FORMAT_POLICY = {
    "schema": "registration-plugin-packages-format-policy-v1",
    "schema_version": 1,
    "selection_order": ["xz", "gz", "plain"],
    "allowed_formats": ["xz", "gz", "plain"],
    "compressed_max_bytes": MAX_CAPTURE_FILE_BYTES,
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
    # ASCII armor is evidence only.  Both gpgv and the isolated inventory
    # receive the independently verified binary projection.
    "keyring_input": "dearmored_path_only",
}
MAX_PACKAGES_EXPANSION_RATIO = 256
MAX_CAPTURE_FILES = 4096
COMMAND_TIMEOUT_SECONDS = 600
IMAGE_INSPECT_TIMEOUT_SECONDS = 30
DOCKER_MISSING_RC = 1
SHA_RE = re.compile(r"^[0-9a-f]{64}$")
DIGEST_RE = re.compile(r"^sha256:[0-9a-f]{64}$")
FINGERPRINT_RE = re.compile(r"^[0-9A-Fa-f]{40}$")
VERSION_RE = re.compile(r"^gpgv \(GnuPG\) [0-9]+(?:\.[0-9]+){1,3}$")
NAME_RE = re.compile(r"^[a-z][a-z0-9_.-]{0,62}$")
SAFE_ENV_KEYS = ("ROS_DISTRO", "DEBIAN_FRONTEND", "LANG", "LC_ALL", "TZ")
FORBIDDEN_TOKENS = frozenset({
    "bash", "sh", "zsh", "-c", "-lc", "curl", "wget", "nc", "socat", "ssh",
    "scp", "pip", "pip3", "--privileged", "--net=host", "--network=host",
})
ROWS = (
    ("humble", "absent"),
    ("humble", "present"),
    ("jazzy", "absent"),
    ("jazzy", "present"),
)
CAMPAIGN_SCHEMA = "registration-plugin-dependency-capture-campaign-v1"
CAMPAIGN_PARTIAL_SCHEMA = (
    "registration-plugin-dependency-capture-child-partial-v1"
)
CAMPAIGN_RECEIPT_NAME = "registration_plugin_dependency_capture.campaign.json"
CAMPAIGN_PARTIAL_RECEIPT_NAME = (
    "registration_plugin_dependency_capture.partial.receipt.json"
)
CAMPAIGN_SCHEMA_PATH = (
    "configs/slam_benchmark_profiles/"
    "registration_plugin_dependency_capture_campaign_v1.schema.json"
)
CAMPAIGN_PARTIAL_SCHEMA_PATH = (
    "configs/slam_benchmark_profiles/"
    "registration_plugin_dependency_capture_child_partial_v1.schema.json"
)
CAMPAIGN_AGGREGATION_SCHEMA = (
    "registration-plugin-dependency-capture-campaign-aggregation-v1"
)
PHASES = (
    "image_inspect", "preexisting_container_check", "container_start", "apt_update",
    "base_status_snapshot", "apt_source_snapshot", "resolver_build", "download_build",
    "restore_build_partial_owner", "resolver_runtime", "download_runtime",
    "restore_runtime_partial_owner", "dependency_install",
    "network_disconnect", "rosdep_prepare", "rosdep_resolution",
    "dependency_capture", "archive_prefetch",
    "post_disconnect_inspect",
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
BOOTSTRAP_PACKAGES = tuple(sorted((
    "build-essential", "cmake", "pkg-config", "python3-rosdep",
    "python3-colcon-common-extensions",
)))
PROJECT_COMMON_APT_PACKAGES = ("libpcl-dev", "python3-yaml")
PROJECT_ROS_APT_SUFFIXES = (
    "libg2o", "message-filters", "pcl-conversions", "rosbag2-cpp",
    "tf2-eigen", "tf2-geometry-msgs", "tf2-ros", "tf2-sensor-msgs",
)


def _provisioning_packages(distro: str) -> tuple[str, ...]:
    """Return exact connected-phase roots needed by disconnected rosdep."""
    if distro not in {"humble", "jazzy"}:
        raise CaptureError("DISTRO_INVALID", distro)
    ros_packages = tuple(
        "ros-{}-{}".format(distro, suffix)
        for suffix in PROJECT_ROS_APT_SUFFIXES
    )
    return tuple(sorted(set(
        BOOTSTRAP_PACKAGES + PROJECT_COMMON_APT_PACKAGES + ros_packages
    )))
MISSING_ERROR_PREFIXES = (
    "Error: No such object: ", "Error: No such container: ",
    "error: no such object: ", "error: no such container: ",
)
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
APT_URL_POLICY_ID = "registration-plugin-apt-source-url-policy-v1"
IN_CONTAINER_SCHEMA = "registration-plugin-capture-in-container-v2"
IN_CONTAINER_SCHEMA_VERSION = 2
SOURCE_MANIFEST_SCHEMA = "glim_clean_room_r3_apt_source_snapshot_v2"
SOURCE_MANIFEST_SCHEMA_VERSION = 2
IN_CONTAINER_PHASES = frozenset({"apt-source", "dependency"})
GPGV_ALLOWED_STATUS = frozenset({
    "NEWSIG", "GOODSIG", "VALIDSIG", "GOODMDC", "TRUST_UNDEFINED", "SIG_ID",
    "KEY_CONSIDERED", "VERIFICATION_COMPLIANCE_MODE",
})
GPGV_INFORMATIONAL_STATUS = frozenset((
    "KEY_CONSIDERED", "VERIFICATION_COMPLIANCE_MODE",
))
GPGV_MAX_STATUS_DECIMAL = 2**31 - 1
GPGV_PARTIAL_FAILURE_KINDS = frozenset({
    "GPGV_VERIFY_FAILED", "GPGV_STATUS_UNEXPECTED_OUTPUT",
    "GPGV_STATUS_UNEXPECTED_TAG", "GPGV_STATUS_DUPLICATE",
    "GPGV_GOODSIG_INVALID", "GPGV_VALIDSIG_INVALID",
    "GPGV_VALIDSIG_PRIMARY_INVALID", "GPGV_SIGNER_MISSING",
    "GPGV_SIGNER_MISMATCH", "GPGV_STATUS_NOT_UTF8", "GPGV_LOG_OVERSIZE",
    "GPGV_STATUS_ORDER_INVALID", "GPGV_STATUS_FIELDS_INVALID",
    "GPGV_KEY_CONSIDERED_INVALID", "GPGV_COMPLIANCE_MODE_INVALID",
    "GPGV_KEY_CONSIDERED_MISMATCH",
    "GPG_KEY_INVENTORY_FAILED", "GPG_KEY_INVENTORY_OVERSIZE",
    "GPG_KEY_INVENTORY_NOT_UTF8", "GPG_KEY_INVENTORY_UNEXPECTED_RECORD",
    "GPG_KEY_INVENTORY_KEY_INVALID", "GPG_KEY_INVENTORY_FINGERPRINT_INVALID",
    "GPG_KEY_INVENTORY_UID_INVALID", "GPG_KEY_INVENTORY_FINGERPRINT_MISSING",
    "GPG_KEY_INVENTORY_DUPLICATE", "GPG_KEY_INVENTORY_RESIDUE",
    "GPG_KEY_INVENTORY_HOMEDIR_COLLISION", "GPG_KEY_INVENTORY_HOMEDIR_INVALID",
    "GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID",
    "GPG_KEY_INVENTORY_HOMEDIR_PARENT_NOT_EMPTY",
    "GPG_KEY_INVENTORY_HOMEDIR_CREATE_FAILED",
    "GPG_KEY_INVENTORY_HOMEDIR_CHANGED", "GPG_KEY_INVENTORY_HOMEDIR_READ_FAILED",
    "GPG_KEY_INVENTORY_HOMEDIR_NOT_ABSENT",
    "GPG_KEY_INVENTORY_CLEANUP_FAILED", "GPG_KEY_INVENTORY_VERSION_INVALID",
    "GPG_KEY_INVENTORY_TOOL_INVALID",
})
GPG_KEY_INVENTORY_FAILURE_KINDS = frozenset({
    item for item in GPGV_PARTIAL_FAILURE_KINDS
    if item.startswith("GPG_KEY_INVENTORY_")
})
GPG_INVENTORY_EXECUTABLE = "/usr/bin/gpg"
GPG_INVENTORY_VERSIONS = (
    "gpg (GnuPG) 2.2.27",
    "gpg (GnuPG) 2.4.4",
)
# Retained as the deterministic synthetic-test default; production validation
# accepts only the exact, profile-bound allowlist above.
GPG_INVENTORY_VERSION = GPG_INVENTORY_VERSIONS[0]
GPG_INVENTORY_HOMEDIR_PREFIX = "glim-clean-room-r3-gpg-inventory-"
GPG_INVENTORY_WORK_RELATIVE = "gpg-inventory-work"
# Both allowlisted GnuPG versions create these two regular metadata files when the isolated
# ``--import-options show-only`` inventory reads a keybox.  They are not
# evidence inputs and are never retained: only their bounded, no-follow
# descriptors are sealed before the child GNUPGHOME is removed.  The exact
# set is required for a successful inventory; an empty set is allowed only
# for a command which failed before GnuPG created its metadata.
GPG_INVENTORY_METADATA_SCHEMA = "registration-plugin-gpg-inventory-metadata-v1"
GPG_INVENTORY_METADATA_POLICY = (
    "exact-pubring.kbx-trustdb.gpg-supported-show-only"
)
GPG_INVENTORY_METADATA_NAMES = ("pubring.kbx", "trustdb.gpg")
GPG_INVENTORY_METADATA_MODE = 0o600
GPG_INVENTORY_METADATA_MAX_BYTES = 16 * 1024 * 1024
GPG_INVENTORY_METADATA_RETENTION = "descriptor-only-cleaned"
IDENTITY_TOKEN_BYTES = 24
IDENTITY_TOKEN_RE = re.compile(r"^[0-9a-f]{24}$")
DEB822_OPTION_FIELDS = frozenset({
    "architectures", "by-hash", "check-valid-until", "date-max-future",
    "description", "enabled", "identifier", "inrelease-path", "languages",
    "pdiffs", "snapshot", "targets", "trusted", "allow-insecure",
    "allow-weak", "allow-downgrade-to-insecure", "valid-until-min",
    "valid-until-max",
})
APT_SOURCE_RELATIVE = "apt-source"
APT_KEYRING_RELATIVE = "apt-keyrings"
UBUNTU_ARCHIVE_KEYRING = "/usr/share/keyrings/ubuntu-archive-keyring.gpg"
CAPTURE_INDEX_RELATIVE = "apt-index"
CAPTURE_DEB_RELATIVE = "captured-debs"
CAPTURE_ROSDEP_RELATIVE = "rosdep"
DIRECTORY_CONTRACT_NAME = "host-evidence-directories.json"
CAPTURE_DIRECTORY_CONTRACT_SCHEMA = "registration-plugin-capture-directory-contract-v1"
CAPTURE_DIRECTORY_CONTRACT_VERSION = 1

# These are the only directories that the provisioning container may see on
# the rw evidence bind.  They are created by the host before ``docker run``;
# the in-container helper may write files below them but may not create,
# replace, chmod, or chown a directory.  Keep the list explicit so a newly
# introduced output path fails closed instead of silently expanding the bind.
CAPTURE_DIRECTORY_RELATIVES = (
    "logs",
    GPG_INVENTORY_WORK_RELATIVE,
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
    ROSDEP_PREPARE_RELATIVE,
    ROSDEP_PREPARE_INPUT_RELATIVE,
    ROSDEP_PREPARE_RELATIVE + "/commands",
    ROSDEP_PREPARE_RELATIVE + "/artifacts",
    ROSDEP_PREPARE_RELATIVE + "/artifacts/rosdep",
    ROSDEP_PREPARE_RELATIVE + "/artifacts/rosdep/sources.list.d",
    ROSDEP_PREPARE_RELATIVE + "/artifacts/rosdep/cache",
    ROSDEP_PREPARE_RELATIVE + "/work",
    ROSDEP_PREPARE_RELATIVE + "/work/rosdistro",
    # Docker creates a bind-mount target when the nested target does not
    # exist.  Precreate it explicitly so a child mount cannot mutate the
    # parent directory's link count during capture.
    ROSDEP_PREPARE_RELATIVE + "/work/rosdistro/files",
    ROSDEP_PREPARE_RELATIVE + "/work/rosdistro/sources.list.d",
    ROSDEP_PREPARE_RELATIVE + "/work/rosdep-cache",
    ROSDEP_PREPARE_RELATIVE + "/work/ros-home",
    ROSDEP_PREPARE_RELATIVE + "/work/ros-home/rosdep",
    ROSDEP_PREPARE_RELATIVE + "/work/ros-home/rosdep/meta.cache",
    ROSDEP_PREPARE_RELATIVE + "/work/ros-home/rosdep/sources.cache",
)
CAPTURE_PREFETCH_DIRECTORY_RELATIVES = (
    "prefetch-root", "prefetch-root/prefetch", "prefetch-root/prefetch/archives",
)

RESTORE_OWNER_SCHEMA = "registration-plugin-apt-partial-owner-restore-v1"
RESTORE_OWNER_SCHEMA_VERSION = 1
RESTORE_OWNER_REFERENCE_RELATIVE = "restore-owner-references"
RESTORE_OWNER_REFERENCE_CONTAINER_ROOT = "/workspace/restore-owner-reference"
RESTORE_OWNER_REFERENCE_MODE = 0o700
RESTORE_OWNER_PHASES = (
    "restore_build_partial_owner", "restore_runtime_partial_owner",
)
RESTORE_OWNER_REFERENCE_MOUNTS = (
    {
        "role": "build",
        "relative": RESTORE_OWNER_REFERENCE_RELATIVE + "/build-partial",
        "target": RESTORE_OWNER_REFERENCE_CONTAINER_ROOT + "/build-partial",
        "target_partial": "/workspace/evidence-parent/debs-build/partial",
    },
    {
        "role": "runtime",
        "relative": RESTORE_OWNER_REFERENCE_RELATIVE + "/runtime-partial",
        "target": RESTORE_OWNER_REFERENCE_CONTAINER_ROOT + "/runtime-partial",
        "target_partial": "/workspace/evidence-parent/debs-runtime/partial",
    },
)
APT_TRANSIENT_OWNER_IDENTITIES = (
    {"distro": "humble", "name": "_apt", "uid": 100, "gid": 0},
    {"distro": "jazzy", "name": "_apt", "uid": 42, "gid": 0},
)
APT_TRANSIENT_OWNER_POLICY = {
    "schema": "registration-plugin-apt-transient-owner-policy-v2",
    "schema_version": 2,
    "identities": [dict(item) for item in APT_TRANSIENT_OWNER_IDENTITIES],
    # The selected identity is bound to the exact distro, profile bytes, and
    # image digest in each plan.  Jammy and Noble deliberately use different
    # _apt UIDs (100 and 42 respectively).
    "identity_evidence": "profile-distro-image-digest-and-base-status/passwd",
    "binding": "distro-profile-image-digest",
}


def _apt_transient_owner_for_distro(distro: Any) -> dict[str, Any]:
    selected = next(
        (item for item in APT_TRANSIENT_OWNER_IDENTITIES
         if item["distro"] == distro), None)
    if selected is None:
        raise CaptureError("RESTORE_OWNER_TRANSIENT_OWNER_INVALID", "unsupported distro")
    return {
        "schema": "registration-plugin-apt-transient-owner-binding-v2",
        "schema_version": 2,
        **dict(selected),
        "identity_evidence": APT_TRANSIENT_OWNER_POLICY["identity_evidence"],
        "binding": APT_TRANSIENT_OWNER_POLICY["binding"],
    }


RESTORE_OWNER_POLICY = {
    "schema": RESTORE_OWNER_SCHEMA,
    "schema_version": RESTORE_OWNER_SCHEMA_VERSION,
    "phase_order": list(RESTORE_OWNER_PHASES),
    "reference_mode": RESTORE_OWNER_REFERENCE_MODE,
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


class CaptureError(RuntimeError):
    """Raised when a host capture cannot be independently sealed."""

    def __init__(self, kind: str, message: str, *, release_evidence: Any = None,
                 package_evidence: Any = None, signature_evidence: Any = None,
                 signature_references: Any = None, signature_count: Any = None,
                 reference_map: Any = None, reference_counts: Any = None,
                 stream_descriptors: Any = None):
        super().__init__(message)
        self.kind = kind
        self.release_evidence = release_evidence
        self.package_evidence = package_evidence
        self.signature_evidence = signature_evidence
        self.signature_references = signature_references
        self.signature_count = signature_count
        self.reference_map = reference_map
        self.reference_counts = reference_counts
        self.stream_descriptors = stream_descriptors


def _load(path: Path, name: str):
    spec = importlib.util.spec_from_file_location(name, path)
    if spec is None or spec.loader is None:
        raise CaptureError("CONTRACT_MISSING", str(path))
    module = importlib.util.module_from_spec(spec)
    spec.loader.exec_module(module)
    return module


AUDIT = _load(AUDIT_PATH, "registration_plugin_capture_audit")
CLOSURE = _load(CLOSURE_PATH, "registration_plugin_capture_closure")
PREFETCH = _load(PREFETCH_PATH, "registration_plugin_capture_prefetch")
EVIDENCE_DIRECTORIES = _load(
    EVIDENCE_DIRECTORY_PATH, "registration_plugin_capture_evidence_directories")
ROSDEP_PREPARE = _load(
    ROSDEP_PREPARE_PATH, "registration_plugin_capture_rosdep_prepare")


def canonical_bytes(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def canonical_hash(value: Mapping[str, Any], excluded: str = "canonical_sha256") -> str:
    projection = dict(value)
    projection.pop(excluded, None)
    return hashlib.sha256(canonical_bytes(projection)).hexdigest()


def _immutable_copy(value: Any) -> Any:
    """Copy JSON evidence before passing it to a second consumer.

    Keyring descriptors are enriched with inventory and signer observations as
    a signature is verified.  Sharing the mutable manifest object between
    components would let a later failed attempt rewrite evidence that already
    passed.  All values used here are JSON-shaped, so ``deepcopy`` is an
    explicit and inexpensive boundary rather than an implicit alias.
    """
    return copy.deepcopy(value)


def _rosdep_prepare_contract(*, required: bool, profile_path: Path,
                             profile_sha256: str) -> dict[str, Any]:
    """Return the fixed, candidate-only rosdep prepare contract.

    The active release profile predates this additive phase and therefore is
    intentionally not rewritten here.  Its discovery packet is still bound
    exactly: the prepare tool reopens that packet before doing any work.  A
    synthetic unit plan may set ``required`` false, but it retains the same
    immutable tool/schema/discovery projection so an omitted binding cannot
    accidentally become a production capture.
    """
    def source_file(relative: str, path: Path, label: str) -> dict[str, str]:
        try:
            digest = AUDIT.sha256_file(path)
        except (OSError, TypeError) as error:
            raise CaptureError("ROSDEP_PREPARE_CONTRACT_MISSING", label) from error
        _sha(digest, label)
        return {"path": relative, "sha256": digest}

    packages = []
    for item in ROSDEP_PREPARE.EXPECTED_PACKAGES:
        packages.append({
            "name": item["name"], "version": item["version"],
            "architecture": item["architecture"],
            "filename": item["filename"], "uri": item["uri"],
            "size_bytes": item["size_bytes"], "sha256": item["sha256"],
            "artifact_path": item["artifact_path"],
        })
    selected_files = []
    for item in ROSDEP_PREPARE.EXPECTED_SELECTED_FILES:
        selected_files.append({
            "relative_path": item["relative_path"],
            "bytes": item["bytes"], "sha256": item["sha256"],
            "artifact_path": item["artifact_path"],
        })
    expected_archive_url = (
        "https://codeload.github.com/ros/rosdistro/tar.gz/"
        + ROSDEP_PREPARE.EXPECTED_COMMIT
    )
    discovery = {
        "discovery_id": ROSDEP_PREPARE.DISCOVERY_ID,
        "root": str(ROSDEP_PREPARE.DISCOVERY_ROOT),
        "receipt": {
            "path": str(ROSDEP_PREPARE.DISCOVERY_ROOT /
                         ROSDEP_PREPARE.DISCOVERY_RECEIPT_NAME),
            "sha256": ROSDEP_PREPARE.EXPECTED_RECEIPT_SHA256,
            "canonical_sha256": ROSDEP_PREPARE.EXPECTED_DISCOVERY_CANONICAL_SHA256,
        },
        "profile": {
            "path": str(ROSDEP_PREPARE.PROFILE_PATH),
            "sha256": ROSDEP_PREPARE.EXPECTED_PROFILE_SHA256,
        },
        "image": {
            "digest": ROSDEP_PREPARE.EXPECTED_IMAGE_DIGEST,
            "platform": ROSDEP_PREPARE.EXPECTED_IMAGE_PLATFORM,
            "repo_digests": [
                "ros@" + ROSDEP_PREPARE.EXPECTED_IMAGE_DIGEST
            ],
        },
        "rosdistro": {
            "repository": "https://github.com/ros/rosdistro.git",
            "commit": ROSDEP_PREPARE.EXPECTED_COMMIT,
            "archive_url": expected_archive_url,
            "archive_bytes": ROSDEP_PREPARE.EXPECTED_ARCHIVE_BYTES,
            "archive_sha256": ROSDEP_PREPARE.EXPECTED_ARCHIVE_SHA256,
        },
        "packages": packages,
        "selected_files": selected_files,
        # JSON receipts materialize tuples as arrays.  Keep the in-memory
        # contract in that same shape so a sealed receipt can be recomputed
        # byte-for-byte during reopen validation.
        "source_records": _immutable_copy(
            list(ROSDEP_PREPARE.EXPECTED_SOURCE_RECORDS)),
        "receipt_bytes_sha256": ROSDEP_PREPARE.EXPECTED_RECEIPT_SHA256,
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
            source_file(
                "scripts/prepare_registration_plugin_rosdep.py",
                ROSDEP_PREPARE_PATH, "prepare tool"),
            source_file(
                "configs/slam_benchmark_profiles/registration_plugin_rosdep_prepare_v1.schema.json",
                ROSDEP_PREPARE_SCHEMA_PATH, "prepare schema"),
            source_file(
                "scripts/capture_registration_plugin_rosdep_discovery.py",
                ROSDEP_DISCOVERY_HELPER_PATH, "discovery helper"),
            {
                "path": "<active-profile>",
                "sha256_source": "plan_profile_sha256",
            },
        ],
    }
    input_bundle["files"] = [
        {
            "role": "tool",
            "source_path": input_bundle["files"][0]["path"],
            "container_path": ROSDEP_PREPARE_CONTAINER_TOOL,
            "sha256": input_bundle["files"][0]["sha256"],
        },
        {
            "role": "schema",
            "source_path": input_bundle["files"][1]["path"],
            "container_path": ROSDEP_PREPARE_CONTAINER_SCHEMA,
            "sha256": input_bundle["files"][1]["sha256"],
        },
        {
            "role": "discovery_validator",
            "source_path": input_bundle["files"][2]["path"],
            "container_path": ROSDEP_PREPARE_CONTAINER_DISCOVERY,
            "sha256": input_bundle["files"][2]["sha256"],
        },
        {
            "role": "consumer_profile",
            "source_path": "<active-profile>",
            "container_path": ROSDEP_PREPARE_CONTAINER_PROFILE,
            "sha256_source": "plan_profile_sha256",
        },
    ]
    contract = {
        "schema": "registration-plugin-rosdep-prepare-contract-v1",
        "schema_version": 1,
        "required": bool(required),
        "phase_order": [
            "dependency_install", "network_disconnect", "rosdep_prepare",
            "rosdep_resolution",
        ],
        "tool": source_file(
            "scripts/prepare_registration_plugin_rosdep.py",
            ROSDEP_PREPARE_PATH, "prepare tool"),
        "schema_binding": source_file(
            "configs/slam_benchmark_profiles/registration_plugin_rosdep_prepare_v1.schema.json",
            ROSDEP_PREPARE_SCHEMA_PATH, "prepare schema"),
        "input_bundle": input_bundle,
        "mount_policy": _immutable_copy(CLOSURE.ROSDEP_PREPARE_MOUNT_POLICY),
        "capture_profile": {
            "path": str(profile_path), "sha256": profile_sha256,
        },
        "discovery": discovery,
        "execution": {
            "image_digest": ROSDEP_PREPARE.EXPECTED_IMAGE_DIGEST,
            "platform": ROSDEP_PREPARE.EXPECTED_IMAGE_PLATFORM,
            "pull": False, "build": False, "network": "none",
            "network_disconnect_required": True,
            "fallback": "forbidden", "runner": "existing_prepare_tool",
        },
        "output": {
            "receipt_relative": ROSDEP_PREPARE_RELATIVE + "/" +
            ROSDEP_PREPARE_RECEIPT_NAME,
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
    contract["identity_sha256"] = canonical_hash(contract, "identity_sha256")
    return contract


def _identity_token(identity: str, label: str = "artifact identity") -> str:
    """Return the bounded, filesystem-safe prefix used in evidence names.

    The complete SHA256 remains in the surrounding receipt/evidence object.
    Names use only a fixed lower-case prefix; callers must use
    ``_claim_identity_artifact`` before writing so a prefix collision cannot
    silently select the wrong full identity.
    """
    _sha(identity, label)
    token = identity[:IDENTITY_TOKEN_BYTES]
    if IDENTITY_TOKEN_RE.fullmatch(token) is None:
        raise CaptureError("ARTIFACT_IDENTITY_INVALID", label)
    return token


def _claim_identity_artifact(registry: dict[str, str] | None,
                             relative: str, identity: str,
                             label: str = "artifact identity") -> None:
    """Bind one deterministic output path to exactly one full identity."""
    _identity_token(identity, label)
    _safe_rel(relative, label + " path")
    # A registry is per fresh output root.  It is deliberately explicit rather
    # than inferred from the filesystem: this catches two different full
    # identities which happen to share the bounded filename prefix before a
    # later write could overwrite or obscure evidence.
    if registry is None:
        return
    previous = registry.get(relative)
    if previous is not None and previous != identity:
        raise CaptureError("ARTIFACT_IDENTITY_PREFIX_COLLISION", relative)
    registry[relative] = identity


def _release_artifact_identity(repository: Mapping[str, Any], release: Path,
                               release_kind: str, release_data: bytes) -> str:
    """Hash the immutable raw Release input used for its output filename."""
    release_url, _ = _repository_urls(repository, release_kind, "plain")
    return canonical_hash({
        "domain": "registration-plugin-release-artifact-identity-v1",
        "repository_url": repository["url"], "release_url": release_url,
        "release_path": str(release), "release_kind": release_kind,
        "release_sha256": sha256_bytes(release_data),
    })


def _release_evidence_identity(item: Mapping[str, Any]) -> str:
    """Recompute the identity projection carried by raw Release evidence."""
    return canonical_hash({
        "domain": "registration-plugin-release-artifact-identity-v1",
        "repository_url": item["repository_url"], "release_url": item["url"],
        "release_path": item["source"], "release_kind": item["kind"],
        "release_sha256": item["sha256"],
    })


PACKAGE_ARTIFACT_IDENTITY_DOMAIN = (
    "registration-plugin-packages-artifact-identity-v1"
)
PACKAGE_ARTIFACT_IDENTITY_FIELDS = frozenset({
    "domain", "repository_url", "suite", "component", "architecture",
    "release_kind", "release_identity", "release_url", "release_record_path",
    "release_file_sha256", "release_file_bytes", "signed_compressed_path",
    "signed_plain_path", "format", "packages_url", "compressed_sha256",
    "compressed_bytes", "decoded_sha256", "decoded_bytes", "signed_empty",
    "identity",
})


def _package_artifact_identity_projection(
        repository: Mapping[str, Any], release_kind: str,
        release_identity: str, release_record_path: str, release_data: bytes,
        package_relative: str, raw_relative: str, package_format: str,
        packages_url: str, compressed_sha256: str, compressed_bytes: int,
        decoded_sha256: str, decoded_bytes: int,
        signed_empty: bool | None = None) -> dict[str, Any]:
    """Build the complete immutable identity of one Packages artifact.

    The signed compressed and plain Release paths are both included.  This is
    intentionally richer than a decoded-content hash: ``jammy/main`` and
    ``jammy-updates/main`` may contain identical bytes but are different
    repository inputs and must never share a filename accidentally.
    """
    release_url, _ = _repository_urls(repository, release_kind, "plain")
    if signed_empty is None:
        signed_empty = decoded_bytes == 0
    projection = {
        "domain": PACKAGE_ARTIFACT_IDENTITY_DOMAIN,
        "repository_url": repository["url"],
        "suite": repository["suite"],
        "component": repository["component"],
        "architecture": "amd64",
        "release_kind": release_kind,
        "release_identity": release_identity,
        "release_url": release_url,
        "release_record_path": release_record_path,
        "release_file_sha256": sha256_bytes(release_data),
        "release_file_bytes": len(release_data),
        "signed_compressed_path": raw_relative,
        "signed_plain_path": package_relative,
        "format": package_format,
        "packages_url": packages_url,
        "compressed_sha256": compressed_sha256,
        "compressed_bytes": compressed_bytes,
        "decoded_sha256": decoded_sha256,
        "decoded_bytes": decoded_bytes,
        "signed_empty": signed_empty,
    }
    return projection


def _package_artifact_identity(
        repository: Mapping[str, Any], release_kind: str,
        release_identity: str, release_record_path: str, release_data: bytes,
        package_relative: str, raw_relative: str, package_format: str,
        packages_url: str, compressed_sha256: str, compressed_bytes: int,
        decoded_sha256: str, decoded_bytes: int,
        signed_empty: bool | None = None) -> dict[str, Any]:
    projection = _package_artifact_identity_projection(
        repository, release_kind, release_identity, release_record_path,
        release_data, package_relative, raw_relative, package_format,
        packages_url, compressed_sha256, compressed_bytes, decoded_sha256,
        decoded_bytes, signed_empty)
    return dict(projection, identity=canonical_hash(projection, "identity"))


def _package_artifact_name(relative: str, identity: str) -> str:
    """Return a bounded identity-derived basename for a Packages artifact."""
    if (not isinstance(relative, str) or not relative or
            Path(relative).name != relative or
            any(char in relative for char in "\\\x00\r\n")):
        raise CaptureError("PACKAGES_ARTIFACT_NAME_INVALID", relative)
    return relative + "-" + _identity_token(identity, "Packages artifact")


def _signed_empty_packages_policy() -> Mapping[str, Any]:
    policy = PACKAGES_FORMAT_POLICY.get("signed_empty_packages_policy")
    if not isinstance(policy, Mapping) or dict(policy) != SIGNED_EMPTY_PACKAGES_POLICY:
        raise CaptureError("PACKAGES_FORMAT_POLICY_INVALID", "signed empty Packages policy")
    return policy


def _validate_package_artifact_identity(value: Any, label: str) -> dict[str, Any]:
    """Validate the full identity projection carried by Packages evidence."""
    if not isinstance(value, Mapping) or set(value) != PACKAGE_ARTIFACT_IDENTITY_FIELDS:
        raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": shape")
    if value["domain"] != PACKAGE_ARTIFACT_IDENTITY_DOMAIN or \
            value["architecture"] != "amd64" or \
            value["release_kind"] not in {"inrelease", "detached"} or \
            value["format"] not in {"xz", "gz", "plain"}:
        raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": domain")
    _capture_url(value["repository_url"], label + " repository URL")
    _capture_url(value["release_url"], label + " Release URL")
    _capture_url(value["packages_url"], label + " Packages URL")
    for field in ("release_identity", "release_file_sha256", "compressed_sha256",
                  "decoded_sha256", "identity"):
        _sha(value[field], label + " " + field)
    _safe_rel(value["release_record_path"], label + " Release record path")
    for field in ("signed_compressed_path", "signed_plain_path"):
        _safe_rel(value[field], label + " signed path")
    if (not isinstance(value["suite"], str) or
            not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+_.-]{0,127}", value["suite"]) or
            not isinstance(value["component"], str) or
            not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+_.-]{0,127}", value["component"])):
        raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": suite/component")
    if type(value["signed_empty"]) is not bool:
        raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": signed-empty type")
    for field in ("release_file_bytes", "compressed_bytes"):
        if type(value[field]) is not int or not 0 < value[field] <= MAX_CAPTURE_FILE_BYTES:
            raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": compressed size")
    if type(value["decoded_bytes"]) is not int or value["decoded_bytes"] < 0 or \
            value["decoded_bytes"] > MAX_DECODED_PACKAGES_BYTES:
        raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": decoded size")
    if value["decoded_bytes"] == 0:
        policy = _signed_empty_packages_policy()
        if (value["signed_empty"] is not True or value["format"] != "xz" or
                value["decoded_sha256"] != EMPTY_PACKAGES_SHA256 or
                value["compressed_bytes"] != policy["xz_compressed_bytes"]):
            raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": signed empty")
    elif value["signed_empty"] is not False:
        raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": nonempty marker")
    expected_release_url, expected_packages_url = _repository_urls(
        {"url": value["repository_url"], "suite": value["suite"],
         "component": value["component"]}, value["release_kind"], value["format"])
    by_hash_url = _packages_by_hash_url(
        {"url": value["repository_url"], "suite": value["suite"],
         "component": value["component"]}, value["compressed_sha256"])
    if value["release_url"] != expected_release_url or \
            value["packages_url"] not in {expected_packages_url, by_hash_url}:
        raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": URL binding")
    expected_plain = value["component"] + "/binary-amd64/Packages"
    expected_compressed = expected_plain + (
        "" if value["format"] == "plain" else "." + value["format"])
    if value["signed_plain_path"] != expected_plain or \
            value["signed_compressed_path"] != expected_compressed:
        raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": signed path")
    if value["identity"] != canonical_hash(dict(value), "identity"):
        raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", label + ": canonical identity")
    return dict(value)


def _release_artifact_name(release_name: str, identity: str) -> str:
    """Return a deterministic, bounded-safe Release evidence basename."""
    if (not isinstance(release_name, str) or not release_name or
            Path(release_name).name != release_name or
            any(char in release_name for char in "\\\x00\r\n")):
        raise CaptureError("RELEASE_ARTIFACT_NAME_INVALID", release_name)
    return release_name + "-" + _identity_token(identity, "Release artifact")


def _detached_artifact_name(signature_name: str, signature_sha256: str) -> str:
    if (not isinstance(signature_name, str) or not signature_name or
            Path(signature_name).name != signature_name or
            any(char in signature_name for char in "\\\x00\r\n")):
        raise CaptureError("SIGNATURE_ARTIFACT_NAME_INVALID", signature_name)
    return signature_name + "-" + _identity_token(
        signature_sha256, "detached signature artifact")


def _keyring_identity(source_key: str, signed_by: Mapping[str, Any],
                      descriptor: Mapping[str, Any]) -> str:
    """Return the stable identity of one source-scoped keyring.

    Inventory output is intentionally excluded: it is the result of the
    first command for this immutable input, not part of the input identity.
    The source record, material hashes, and declared fingerprint policy are
    retained so a different repository/key policy cannot reuse the cache.
    """
    source_projection = {
        "source_key": source_key,
        "kind": signed_by.get("kind"),
        "source": signed_by.get("source"),
        "fingerprint": signed_by.get("fingerprint"),
    }
    descriptor_projection = {
        key: descriptor.get(key) for key in (
            "kind", "source", "path", "bytes", "sha256",
            "normalized_armor_bytes", "normalized_armor_sha256",
            "dearmored_path", "dearmored_bytes", "dearmored_sha256",
        )
    }
    descriptor_projection["fingerprint_policy"] = {
        # The inventory's primary/subkey lists are observations.  Including
        # them would make the identity change after the first command and
        # would defeat reuse of the same immutable keyring.  The source
        # record's declared policy is stable input and remains bound here;
        # observed fingerprints are validated separately.
        "declared": signed_by.get("fingerprint"),
    }
    return canonical_hash({
        "domain": "registration-plugin-gpg-keyring-identity-v1",
        "source": source_projection,
        "descriptor": descriptor_projection,
    })


def _source_entry_keyring_identity(signed_by: Mapping[str, Any],
                                   descriptor: Mapping[str, Any]) -> str:
    """Rebuild input identity without treating an inline observation as policy."""
    projection = dict(signed_by)
    if projection.get("kind") == "inline":
        # source_entries expose the verified inline fingerprint for consumers,
        # but the original deb822 source declared key material, not a separate
        # fingerprint policy.  The observation must not alter input identity.
        projection.pop("fingerprint", None)
        source_key = "inline:" + str(projection.get("sha256"))
    else:
        source_key = str(projection.get("source"))
    return _keyring_identity(source_key, projection, descriptor)


def _release_signature_identity(repository: Mapping[str, Any],
                                release: Path, release_kind: str,
                                release_record: Mapping[str, Any],
                                release_data: bytes, keyring_identity: str,
                                signer_policy: Mapping[str, Any]) -> str:
    """Return the immutable identity used to deduplicate one gpgv run."""
    release_url, _ = _repository_urls(repository, release_kind, "plain")
    return _signature_identity_projection({
        "repository_url": repository["url"], "release_url": release_url,
        "release_path": str(release),
        "release_record_path": release_record["path"],
        "release_kind": release_kind,
        "release_sha256": sha256_bytes(release_data),
        "keyring_identity": keyring_identity,
        "signer_policy": dict(signer_policy),
    })


def _signature_identity_projection(fields: Mapping[str, Any]) -> str:
    """Hash the non-package portion of a release-signature reference."""
    return canonical_hash({
        "domain": "registration-plugin-release-signature-identity-v1",
        "url": fields["repository_url"], "release_url": fields["release_url"],
        "release_path": fields["release_path"],
        "release_record_path": fields["release_record_path"],
        "release_kind": fields["release_kind"],
        "release_sha256": fields["release_sha256"],
        "keyring_identity": fields["keyring_identity"],
        "signer_policy": dict(fields["signer_policy"]),
    })


def sha256_bytes(value: bytes) -> str:
    return hashlib.sha256(value).hexdigest()


def _sha(value: Any, label: str) -> str:
    if not isinstance(value, str) or SHA_RE.fullmatch(value) is None:
        raise CaptureError("SHA_INVALID", label)
    return value


def _digest(value: Any, label: str) -> str:
    if not isinstance(value, str) or DIGEST_RE.fullmatch(value) is None:
        raise CaptureError("DIGEST_INVALID", label)
    return value


def _safe_abs(value: Any, label: str) -> Path:
    if (not isinstance(value, str) or not value.startswith("/") or "\x00" in value or
            "\n" in value or Path(value).as_posix() != value or
            any(part in ("", ".", "..") for part in value.split("/")[1:])):
        raise CaptureError("PATH_INVALID", label)
    return Path(value)


def _safe_rel(value: Any, label: str) -> str:
    if (not isinstance(value, str) or not value or value.startswith("/") or
            "\\" in value or "\x00" in value or Path(value).as_posix() != value or
            any(part in ("", ".", "..") for part in value.split("/"))):
        raise CaptureError("PATH_INVALID", label)
    return value


def _parents_no_symlink(path: Path, label: str) -> None:
    if not path.is_absolute():
        raise CaptureError("PATH_INVALID", label)
    current = Path(path.anchor)
    for component in path.parts[1:-1]:
        current /= component
        try:
            info = current.lstat()
        except OSError as error:
            raise CaptureError("PATH_MISSING", "{}: {}".format(label, current)) from error
        if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
            raise CaptureError("PATH_PARENT_INVALID", str(current))


def _read_bounded(path: Path, label: str, maximum: int,
                  *, allow_empty: bool = True, readonly: bool = True) -> tuple[bytes, os.stat_result]:
    path = _safe_abs(str(path), label)
    _parents_no_symlink(path, label)
    try:
        parent_before = path.parent.lstat()
        before = path.lstat()
    except OSError as error:
        raise CaptureError("FILE_MISSING", label) from error
    if (stat.S_ISLNK(before.st_mode) or not stat.S_ISREG(before.st_mode) or
            before.st_nlink != 1 or before.st_size > maximum or
            (before.st_size == 0 and not allow_empty) or
            (readonly and stat.S_IMODE(before.st_mode) != 0o444)):
        raise CaptureError("FILE_INVALID", label)
    fd = None
    try:
        fd = os.open(str(path), os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) |
                     getattr(os, "O_CLOEXEC", 0))
        fd_before = os.fstat(fd)
        if (fd_before.st_dev, fd_before.st_ino, fd_before.st_size, fd_before.st_nlink) != \
                (before.st_dev, before.st_ino, before.st_size, 1):
            raise CaptureError("FILE_IDENTITY_CHANGED", label)
        chunks = []
        total = 0
        while True:
            block = os.read(fd, min(1024 * 1024, maximum - total + 1))
            if not block:
                break
            chunks.append(block)
            total += len(block)
            if total > maximum:
                raise CaptureError("FILE_OVERSIZE", label)
        fd_after = os.fstat(fd)
    except OSError as error:
        raise CaptureError("FILE_READ_FAILED", label) from error
    finally:
        if fd is not None:
            os.close(fd)
    data = b"".join(chunks)
    try:
        after = path.lstat()
        parent_after = path.parent.lstat()
    except OSError as error:
        raise CaptureError("FILE_CHANGED", label) from error
    if (len(data) != before.st_size or
            (fd_after.st_dev, fd_after.st_ino, fd_after.st_size, fd_after.st_nlink) !=
            (before.st_dev, before.st_ino, before.st_size, 1) or
            (after.st_dev, after.st_ino, after.st_size, after.st_nlink) !=
            (before.st_dev, before.st_ino, before.st_size, 1) or
            (parent_after.st_dev, parent_after.st_ino) !=
            (parent_before.st_dev, parent_before.st_ino) or
            stat.S_ISLNK(parent_after.st_mode)):
        raise CaptureError("FILE_CHANGED", label)
    return data, before


def _descriptor(path: Path, label: str, maximum: int = MAX_JSON_BYTES,
                *, sidecar: bool = False, allow_empty: bool = False
                ) -> dict[str, Any]:
    data, info = _read_bounded(path, label, maximum, allow_empty=allow_empty)
    value = {
        "path": str(path), "bytes": len(data), "sha256": sha256_bytes(data),
        "mode": stat.S_IMODE(info.st_mode), "uid": info.st_uid, "gid": info.st_gid,
        "nlink": info.st_nlink, "device": info.st_dev, "inode": info.st_ino,
    }
    if sidecar:
        sidecar_path = Path(str(path) + ".sha256")
        side_data, side_info = _read_bounded(sidecar_path, label + " sidecar", 512,
                                             allow_empty=False)
        try:
            tokens = side_data.decode("ascii").strip().split()
        except UnicodeError as error:
            raise CaptureError("SIDECAR_INVALID", label) from error
        if tokens != [value["sha256"], path.name]:
            raise CaptureError("SIDECAR_MISMATCH", label)
        value["sidecar"] = {
            "path": str(sidecar_path), "bytes": len(side_data),
            "sha256": sha256_bytes(side_data), "mode": stat.S_IMODE(side_info.st_mode),
            "uid": side_info.st_uid, "gid": side_info.st_gid, "nlink": side_info.st_nlink,
            "device": side_info.st_dev, "inode": side_info.st_ino,
        }
    return value


def _write_new(path: Path, data: bytes, *, maximum: int = MAX_JSON_BYTES,
               allow_empty: bool = False) -> dict[str, Any]:
    path = _safe_abs(str(path), "output path")
    if ((len(data) == 0 and not allow_empty) or len(data) > maximum or
            path.exists() or path.is_symlink()):
        raise CaptureError("OUTPUT_COLLISION_OR_SIZE", str(path))
    _parents_no_symlink(path, "output path")
    try:
        parent_before = path.parent.lstat()
    except OSError as error:
        raise CaptureError("OUTPUT_PARENT_INVALID", str(path)) from error
    if stat.S_ISLNK(parent_before.st_mode) or not stat.S_ISDIR(parent_before.st_mode):
        raise CaptureError("OUTPUT_PARENT_INVALID", str(path))
    fd = None
    created = None
    try:
        fd = os.open(str(path), os.O_WRONLY | os.O_CREAT | os.O_EXCL |
                     getattr(os, "O_NOFOLLOW", 0), 0o600)
        created = os.fstat(fd)
        offset = 0
        while offset < len(data):
            written = os.write(fd, data[offset:])
            if written <= 0:
                raise CaptureError("OUTPUT_NO_PROGRESS", str(path))
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
                if (info.st_dev, info.st_ino) == (created.st_dev, created.st_ino):
                    path.unlink()
            except OSError:
                pass
        if isinstance(error, CaptureError):
            raise
        raise CaptureError("OUTPUT_WRITE_FAILED", str(path)) from error
    finally:
        if fd is not None:
            try:
                os.close(fd)
            except OSError:
                pass
    try:
        parent_after = path.parent.lstat()
        if (parent_after.st_dev, parent_after.st_ino) != (parent_before.st_dev, parent_before.st_ino):
            raise CaptureError("OUTPUT_PARENT_CHANGED", str(path))
        parent_fd = os.open(str(path.parent), os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) |
                            getattr(os, "O_NOFOLLOW", 0))
        try:
            parent_info = os.fstat(parent_fd)
            if (parent_info.st_dev, parent_info.st_ino) != (parent_before.st_dev, parent_before.st_ino):
                raise CaptureError("OUTPUT_PARENT_CHANGED", str(path))
            os.fsync(parent_fd)
        finally:
            os.close(parent_fd)
    except OSError as error:
        raise CaptureError("OUTPUT_DIRSYNC_FAILED", str(path.parent)) from error
    return _descriptor(path, "sealed output", max(len(data), 1),
                       allow_empty=allow_empty)


def _seal_pair(root: Path, filename: str, value: Mapping[str, Any]) -> dict[str, Any]:
    payload = canonical_bytes(value) + b"\n"
    path = root / filename
    descriptor = _write_new(path, payload)
    side_payload = ("{}  {}\n".format(descriptor["sha256"], path.name)).encode("ascii")
    try:
        sidecar = _write_new(Path(str(path) + ".sha256"), side_payload, maximum=512)
    except Exception:
        try:
            info = path.lstat()
            if (info.st_dev, info.st_ino) == (descriptor["device"], descriptor["inode"]):
                path.unlink()
        except OSError:
            pass
        raise
    return {"file": descriptor, "sidecar": sidecar}


def _seal_existing_sidecar(path: Path, label: str,
                           maximum: int = MAX_JSON_BYTES) -> dict[str, Any]:
    """Bind an immutable producer output to a fresh exact SHA-256 sidecar."""
    descriptor = _descriptor(path, label, maximum)
    side_payload = ("{}  {}\n".format(
        descriptor["sha256"], path.name)).encode("ascii")
    sidecar = _write_new(
        Path(str(path) + ".sha256"), side_payload, maximum=512)
    return {"file": descriptor, "sidecar": sidecar}


def _fresh_root(path: Path, label: str) -> dict[str, Any]:
    try:
        descriptor = EVIDENCE_DIRECTORIES.create_fresh_root(path)
    except EVIDENCE_DIRECTORIES.EvidenceDirectoryError as error:
        raise CaptureError(error.kind, str(error)) from error
    # Keep the historical root projection stable.  The complete host-owned
    # uid/gid and all child identities live in the companion directory
    # contract, which is reopened by validate_receipt.
    return {key: descriptor[key] for key in ("path", "device", "inode", "mode", "nlink")}


def _fresh_directory(path: Path, label: str) -> None:
    """Reserve one empty child directory without repairing collisions."""
    try:
        EVIDENCE_DIRECTORIES.create_fresh_root(path)
    except EVIDENCE_DIRECTORIES.EvidenceDirectoryError as error:
        raise CaptureError(error.kind, "{}: {}".format(label, error)) from error


def _assert_root(path: Path, identity: Mapping[str, Any], label: str) -> None:
    try:
        info = path.lstat()
    except OSError as error:
        raise CaptureError("ROOT_CHANGED", label) from error
    if (stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or
            info.st_nlink < 2 or info.st_dev != identity["device"] or
            info.st_ino != identity["inode"] or stat.S_IMODE(info.st_mode) != 0o700):
        raise CaptureError("ROOT_CHANGED", label)


def _capture_directory_paths(*, include_prefetch: bool) -> tuple[str, ...]:
    paths = list(CAPTURE_DIRECTORY_RELATIVES)
    if include_prefetch:
        paths.extend(CAPTURE_PREFETCH_DIRECTORY_RELATIVES)
    return tuple(paths)


def _create_capture_directory_layout(root: Path, *, include_prefetch: bool) -> dict[str, Any]:
    """Create the immutable rw-bind directory layout before container start."""
    try:
        return EVIDENCE_DIRECTORIES.create_layout(
            root, _capture_directory_paths(include_prefetch=include_prefetch))
    except EVIDENCE_DIRECTORIES.EvidenceDirectoryError as error:
        raise CaptureError(error.kind, str(error)) from error


def _source_descriptor(path: Path, label: str) -> dict[str, Any]:
    """Read one repository input without weakening the host-copy contract.

    Source files are checked with the same no-follow/fstat/reopen reader as
    sealed evidence, but retain their existing repository mode.  The copied
    input is the immutable 0444 object that is actually exposed to the
    container; the source descriptor is kept separately so a later source
    edit cannot be hidden by reusing the copied bytes.
    """
    data, info = _read_bounded(
        path, label, MAX_JSON_BYTES, allow_empty=False, readonly=False)
    return {
        "path": str(path), "bytes": len(data), "sha256": sha256_bytes(data),
        "mode": stat.S_IMODE(info.st_mode), "uid": info.st_uid,
        "gid": info.st_gid, "nlink": info.st_nlink,
        "device": info.st_dev, "inode": info.st_ino,
    }


def _prepare_input_specs(plan: Mapping[str, Any]) -> list[dict[str, Any]]:
    """Resolve the four fixed files that may enter the prepare container."""
    contract = plan.get("rosdep_prepare_contract")
    if not isinstance(contract, Mapping):
        raise CaptureError("ROSDEP_PREPARE_CONTRACT_INVALID", "input bundle")
    bundle = contract.get("input_bundle")
    if not isinstance(bundle, Mapping) or not isinstance(bundle.get("files"), list):
        raise CaptureError("ROSDEP_PREPARE_INPUT_BUNDLE_INVALID", "input files")
    expected = {
        item.get("role"): item for item in bundle["files"]
        if isinstance(item, Mapping)
    }
    fixed = (
        ("tool", ROSDEP_PREPARE_PATH, "prepare_registration_plugin_rosdep.py"),
        ("schema", ROSDEP_PREPARE_SCHEMA_PATH,
         "registration_plugin_rosdep_prepare_v1.schema.json"),
        ("discovery_validator", ROSDEP_DISCOVERY_HELPER_PATH,
         "capture_registration_plugin_rosdep_discovery.py"),
        ("consumer_profile", Path(plan["profile_path"]), "consumer-profile.json"),
    )
    result: list[dict[str, Any]] = []
    for role, source_path, relative_path in fixed:
        item = expected.get(role)
        if not isinstance(item, Mapping):
            raise CaptureError("ROSDEP_PREPARE_INPUT_BUNDLE_INVALID", role)
        expected_source = ("<active-profile>" if role == "consumer_profile"
                           else str(source_path.relative_to(ROOT)))
        if item.get("source_path") != expected_source or \
                item.get("container_path") != {
                    "tool": ROSDEP_PREPARE_CONTAINER_TOOL,
                    "schema": ROSDEP_PREPARE_CONTAINER_SCHEMA,
                    "discovery_validator": ROSDEP_PREPARE_CONTAINER_DISCOVERY,
                    "consumer_profile": ROSDEP_PREPARE_CONTAINER_PROFILE,
                }[role]:
            raise CaptureError("ROSDEP_PREPARE_INPUT_BUNDLE_INVALID", role)
        expected_sha = plan["profile_sha256"] if role == "consumer_profile" \
            else item.get("sha256")
        result.append({
            "role": role, "source_path": expected_source,
            "container_path": item["container_path"],
            "relative_path": relative_path, "source": source_path,
            "expected_sha256": expected_sha,
        })
    return result


def _materialize_prepare_input(root: Path, plan: Mapping[str, Any]) -> dict[str, Any]:
    """Atomically copy the exact four prepare inputs into the host layout."""
    input_root = root / ROSDEP_PREPARE_INPUT_RELATIVE
    try:
        root_info = input_root.lstat()
    except OSError as error:
        raise CaptureError("ROSDEP_PREPARE_INPUT_MISSING", str(input_root)) from error
    if (stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode) or
            stat.S_IMODE(root_info.st_mode) != 0o700 or root_info.st_nlink < 2):
        raise CaptureError("ROSDEP_PREPARE_INPUT_INVALID", str(input_root))
    try:
        with os.scandir(str(input_root)) as entries:
            if next(entries, None) is not None:
                raise CaptureError("ROSDEP_PREPARE_INPUT_NOT_FRESH", str(input_root))
    except StopIteration:
        pass
    except OSError as error:
        raise CaptureError("ROSDEP_PREPARE_INPUT_INVALID", str(input_root)) from error

    files: list[dict[str, Any]] = []
    specs = _prepare_input_specs(plan)
    for spec in specs:
        source = spec["source"]
        source_descriptor = _source_descriptor(source, spec["role"] + " source")
        if source_descriptor["sha256"] != spec["expected_sha256"]:
            raise CaptureError("ROSDEP_PREPARE_INPUT_SOURCE_DRIFT", spec["role"])
        destination = input_root / spec["relative_path"]
        data, _ = _read_bounded(
            source, spec["role"] + " source", MAX_JSON_BYTES,
            allow_empty=False, readonly=False)
        _write_new(destination, data, maximum=MAX_JSON_BYTES)
        sidecar = Path(str(destination) + ".sha256")
        descriptor = _descriptor(
            destination, spec["role"] + " copied input", MAX_JSON_BYTES)
        sidecar_payload = ("{}  {}\n".format(descriptor["sha256"], destination.name))
        _write_new(sidecar, sidecar_payload.encode("ascii"), maximum=512)
        before = _descriptor(
            destination, spec["role"] + " copied input", MAX_JSON_BYTES,
            sidecar=True)
        after = _descriptor(
            destination, spec["role"] + " copied input", MAX_JSON_BYTES,
            sidecar=True)
        if before != after or before["sha256"] != source_descriptor["sha256"]:
            raise CaptureError("ROSDEP_PREPARE_INPUT_COPY_DRIFT", spec["role"])
        files.append({
            "role": spec["role"], "source_path": spec["source_path"],
            "container_path": spec["container_path"],
            "relative_path": spec["relative_path"],
            "source": source_descriptor, "before": before, "after": after,
        })
    binding = {
        "schema": ROSDEP_PREPARE_INPUT_SCHEMA, "schema_version": 1,
        "relative_root": ROSDEP_PREPARE_INPUT_RELATIVE,
        "container_root": ROSDEP_PREPARE_CONTAINER_ROOT,
        "mount": {
            "source": str(input_root), "target": ROSDEP_PREPARE_CONTAINER_ROOT,
            "read_only": True, "noexec": False, "repository_bind": "forbidden",
        },
        "files": files,
    }
    binding["identity_sha256"] = canonical_hash(binding, "identity_sha256")
    _verify_prepare_input_binding(root, plan, binding)
    return binding


def _verify_prepare_input_binding(root: Path, plan: Mapping[str, Any],
                                  binding: Mapping[str, Any]) -> None:
    """Reopen source/copy identities before a prepare command is allowed."""
    try:
        CLOSURE._validate_prepare_input_binding_shape(binding)
    except Exception as error:
        raise CaptureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", str(error)) from error
    expected_root = root / ROSDEP_PREPARE_INPUT_RELATIVE
    mount = binding["mount"]
    if mount["source"] != str(expected_root):
        raise CaptureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", "mount source")
    expected_specs = _prepare_input_specs(plan)
    expected_names = set()
    for item, spec in zip(binding["files"], expected_specs):
        if (item["role"], item["source_path"], item["container_path"],
                item["relative_path"]) != (
                    spec["role"], spec["source_path"], spec["container_path"],
                    spec["relative_path"]):
            raise CaptureError("ROSDEP_PREPARE_INPUT_BINDING_INVALID", spec["role"])
        expected_names.update({spec["relative_path"], spec["relative_path"] + ".sha256"})
        source = _source_descriptor(spec["source"], spec["role"] + " source")
        if source != item["source"] or source["sha256"] != spec["expected_sha256"]:
            raise CaptureError("ROSDEP_PREPARE_INPUT_SOURCE_DRIFT", spec["role"])
        actual = _descriptor(
            expected_root / spec["relative_path"], spec["role"] + " copied input",
            MAX_JSON_BYTES, sidecar=True)
        if actual != item["after"] or item["before"] != item["after"]:
            raise CaptureError("ROSDEP_PREPARE_INPUT_COPY_DRIFT", spec["role"])
    try:
        observed = {entry.name for entry in os.scandir(str(expected_root))}
    except OSError as error:
        raise CaptureError("ROSDEP_PREPARE_INPUT_INVALID", str(expected_root)) from error
    if observed != expected_names:
        raise CaptureError("ROSDEP_PREPARE_INPUT_EXTRA", str(expected_root))


def _prepare_mount_specs(output_root: Path) -> list[dict[str, Any]]:
    """Return the fixed, ordered mounts visible to ``rosdep_prepare``.

    The list is part of the plan identity.  It is intentionally assembled in
    one place so the Docker argv, the receipt, and readback validation cannot
    drift independently.  ``type`` is explicit even though Docker's ``-v``
    shorthand currently implies a bind mount; accepting that implication in a
    receipt would leave the source/target contract ambiguous.
    """
    output_root = _safe_abs(str(output_root), "prepare mount output root")
    prepare_root = output_root / ROSDEP_PREPARE_RELATIVE
    prepare_input = output_root / ROSDEP_PREPARE_INPUT_RELATIVE
    prepare_discovery = ROSDEP_PREPARE.DISCOVERY_ROOT
    prepare_source_files = prepare_discovery / "artifacts/rosdistro/files"
    return [
        {"source": str(prepare_input), "target": ROSDEP_PREPARE_CONTAINER_ROOT,
         "read_only": True, "type": "bind", "noexec": False},
        {"source": str(prepare_discovery),
         "target": ROSDEP_PREPARE_CONTAINER_DISCOVERY_ROOT,
         "read_only": True, "type": "bind", "noexec": False},
        {"source": str(prepare_source_files),
         "target": ROSDEP_PREPARE.EXPECTED_LOCAL_SOURCE_ROOT,
         "read_only": True, "type": "bind", "noexec": False},
        {"source": str(prepare_root / "work/rosdistro"),
         "target": "/opt/registration-plugin/rosdistro",
         "read_only": False, "type": "bind", "noexec": False},
        {"source": str(prepare_root / "work/rosdep-cache"),
         "target": ROSDEP_PREPARE.EXPECTED_ROSDEP_CACHE,
         "read_only": False, "type": "bind", "noexec": False},
        {"source": str(prepare_root / "work/ros-home"),
         "target": ROSDEP_PREPARE.EXPECTED_ROS_HOME,
         "read_only": False, "type": "bind", "noexec": False},
        {"source": str(prepare_root / "artifacts/rosdep/sources.list.d"),
         "target": ROSDEP_PREPARE.EXPECTED_SOURCE_LIST_DIR,
         "read_only": False, "type": "bind", "noexec": False},
        {"source": str(prepare_root / "artifacts/rosdep/cache"),
         "target": "/opt/registration-plugin/rosdep-output-cache",
         "read_only": False, "type": "bind", "noexec": False},
    ]


def _restore_owner_mount_specs(output_root: Path) -> list[dict[str, Any]]:
    """Return the two read-only host-reference binds used by owner restore.

    The reference directories are deliberately separate from the writable
    evidence bind.  They are created by the host layout before ``docker run``
    and are used only as ``chown --reference`` inputs inside the container.
    Keeping their source, target, and writable partial target in one ordered
    projection prevents a phase from silently restoring the other cache.
    """
    output_root = _safe_abs(str(output_root), "restore reference output root")
    return [
        {
            "role": item["role"],
            "source": str(output_root / item["relative"]),
            "target": item["target"],
            "target_partial": item["target_partial"],
            "read_only": True, "type": "bind", "noexec": False,
        }
        for item in RESTORE_OWNER_REFERENCE_MOUNTS
    ]


_RESTORE_DIRECTORY_FIELDS = frozenset({
    "path", "type", "device", "inode", "uid", "gid", "mode", "nlink",
    "children",
})
_RESTORE_CHILD_FIELDS = frozenset({
    "name", "type", "device", "inode", "uid", "gid", "mode", "nlink",
})
_RESTORE_MOUNT_FIELDS = frozenset({
    "role", "source", "target", "target_partial", "read_only", "type", "noexec",
})


def _restore_child_type(mode: int) -> str:
    if stat.S_ISREG(mode):
        return "regular"
    if stat.S_ISDIR(mode):
        return "directory"
    if stat.S_ISLNK(mode):
        return "symlink"
    if stat.S_ISFIFO(mode):
        return "fifo"
    if stat.S_ISSOCK(mode):
        return "socket"
    return "other"


def _restore_directory_snapshot(path: Path, label: str) -> dict[str, Any]:
    """Capture one host directory and its immediate children without repair."""
    path = _safe_abs(str(path), label)
    _parents_no_symlink(path, label)
    try:
        info = path.lstat()
    except OSError as error:
        raise CaptureError("RESTORE_OWNER_DIRECTORY_MISSING", label) from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or info.st_nlink < 2:
        raise CaptureError("RESTORE_OWNER_DIRECTORY_INVALID", label)
    children: list[dict[str, Any]] = []
    try:
        entries = sorted(os.scandir(str(path)), key=lambda item: item.name)
        for entry in entries:
            if (not entry.name or "/" in entry.name or "\\" in entry.name or
                    "\x00" in entry.name or len(entry.name) > 255):
                raise CaptureError("RESTORE_OWNER_CHILD_INVALID", label)
            child = entry.stat(follow_symlinks=False)
            children.append({
                "name": entry.name,
                "type": _restore_child_type(child.st_mode),
                "device": int(child.st_dev), "inode": int(child.st_ino),
                "uid": int(child.st_uid), "gid": int(child.st_gid),
                "mode": stat.S_IMODE(child.st_mode), "nlink": int(child.st_nlink),
            })
            if len(children) > 64:
                raise CaptureError("RESTORE_OWNER_CHILD_LIMIT", label)
    except CaptureError:
        raise
    except OSError as error:
        raise CaptureError("RESTORE_OWNER_CHILD_READ_FAILED", label) from error
    return {
        "path": str(path), "type": "directory", "device": int(info.st_dev),
        "inode": int(info.st_ino), "uid": int(info.st_uid), "gid": int(info.st_gid),
        "mode": stat.S_IMODE(info.st_mode), "nlink": int(info.st_nlink),
        "children": children,
    }


def _restore_directory_lstat_snapshot(path: Path, label: str) -> dict[str, Any]:
    """Capture only directory metadata, never opening or scanning its children.

    APT's sandbox can leave a pre-created 0700 ``partial`` directory owned by
    ``_apt``.  The host owner must not be able to read that directory until the
    in-container restore command runs.  This helper is deliberately limited to
    ``lstat`` metadata; callers may use it only with an independently sealed
    empty pre-layout descriptor and must perform a full scan after restoration.
    """
    path = _safe_abs(str(path), label)
    _parents_no_symlink(path, label)
    try:
        info = path.lstat()
    except OSError as error:
        raise CaptureError("RESTORE_OWNER_DIRECTORY_MISSING", label) from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or info.st_nlink < 2:
        raise CaptureError("RESTORE_OWNER_DIRECTORY_INVALID", label)
    return {
        "path": str(path), "type": "directory", "device": int(info.st_dev),
        "inode": int(info.st_ino), "uid": int(info.st_uid), "gid": int(info.st_gid),
        "mode": stat.S_IMODE(info.st_mode), "nlink": int(info.st_nlink),
        # This is not a claim that the inaccessible directory was scanned.
        # It is populated from the independently sealed pre-container empty
        # layout and is paired with ``target_before_observation`` below.
        "children": [],
    }


def _restore_directory_identity(value: Mapping[str, Any]) -> tuple[Any, ...]:
    return tuple(value.get(key) for key in (
        "path", "type", "device", "inode", "mode", "nlink"))


def _apt_transient_owner_binding(plan: Mapping[str, Any], *,
                                 profile_sha256: str | None = None,
    image_digest: str | None = None) -> dict[str, Any]:
    value = plan.get("apt_transient_owner")
    binding_distro = value.get("distro") if isinstance(value, Mapping) else None
    plan_distro = plan.get("distro")
    if plan_distro is not None and binding_distro != plan_distro:
        raise CaptureError("RESTORE_OWNER_TRANSIENT_OWNER_INVALID", "distro binding")
    selected = _apt_transient_owner_for_distro(binding_distro)
    expected_fields = set(selected) | {
        "profile_sha256", "image_digest",
    }
    if not isinstance(value, Mapping) or set(value) != expected_fields:
        raise CaptureError("RESTORE_OWNER_TRANSIENT_OWNER_INVALID", "apt identity fields")
    if {key: value[key] for key in selected} != selected:
        raise CaptureError("RESTORE_OWNER_TRANSIENT_OWNER_INVALID", "apt identity policy")
    expected_profile = (plan.get("profile_sha256") if profile_sha256 is None
                        else profile_sha256)
    expected_image = (plan.get("image", {}).get("digest")
                      if image_digest is None else image_digest)
    if value["profile_sha256"] != expected_profile or \
            value["image_digest"] != expected_image:
        raise CaptureError("RESTORE_OWNER_TRANSIENT_OWNER_INVALID", "profile/image binding")
    if not SHA_RE.fullmatch(value["profile_sha256"]):
        raise CaptureError("RESTORE_OWNER_TRANSIENT_OWNER_INVALID", "profile SHA")
    if not DIGEST_RE.fullmatch(value["image_digest"]):
        raise CaptureError("RESTORE_OWNER_TRANSIENT_OWNER_INVALID", "image digest")
    return dict(value)


def _restore_owner_presealed_target(plan: Mapping[str, Any], root: Path,
                                    role: str, target: Path,
                                    label: str) -> dict[str, Any]:
    """Return the empty host-owned target descriptor sealed before Docker run."""
    stored = plan.get("_restore_owner_target_before")
    if not isinstance(stored, Mapping):
        stored = {}
    value = stored.get(role)
    if value is None:
        # Direct unit callers may invoke the phase without the full executor.
        # Capture this once, before any injected runner is allowed to mutate
        # the fixture; production capture stores it immediately after layout
        # creation instead.
        value = _restore_directory_snapshot(target, label + " presealed")
        plan.setdefault("_restore_owner_target_before", {})[role] = _immutable_copy(value)
    if not isinstance(value, Mapping):
        raise CaptureError("RESTORE_OWNER_TARGET_PRESEALED_INVALID", label)
    value = dict(value)
    try:
        _validate_restore_directory(value, label + " presealed", expected_owner=(os.getuid(), os.getgid()),
                                    require_empty=True)
    except CaptureError as error:
        raise CaptureError("RESTORE_OWNER_TARGET_PRESEALED_INVALID", label) from error
    if value["path"] != str(target):
        raise CaptureError("RESTORE_OWNER_TARGET_PRESEALED_INVALID", label + " path")
    return value


def _restore_owner_target_before(plan: Mapping[str, Any], root: Path,
                                 role: str, target: Path,
                                 presealed: Mapping[str, Any],
                                 phase: str) -> tuple[dict[str, Any], str]:
    """Observe a target before restore, allowing only the fixed APT owner drift."""
    observed = _restore_directory_lstat_snapshot(target, phase + " target before")
    if _restore_directory_identity(observed) != _restore_directory_identity(presealed):
        raise CaptureError("RESTORE_OWNER_TARGET_DRIFT", phase + " target before identity")
    transient = _apt_transient_owner_binding(plan)
    host_owner = (os.getuid(), os.getgid())
    observed_owner = (observed["uid"], observed["gid"])
    if observed_owner == host_owner:
        # A host-owned target is readable, so prove its children rather than
        # inheriting the presealed empty claim.
        full = _restore_directory_snapshot(target, phase + " target before")
        if full != dict(presealed):
            raise CaptureError("RESTORE_OWNER_TARGET_DRIFT", phase + " target before")
        return full, "scandir_presealed_empty"
    if observed_owner != (transient["uid"], transient["gid"]):
        raise CaptureError("RESTORE_OWNER_TRANSIENT_OWNER_INVALID", phase + " target before owner")
    # The exact transient owner is the only state in which a scan is
    # intentionally impossible.  The pre-container contract proved this
    # inode was an empty directory; post-restore scanning remains mandatory.
    observed["children"] = _immutable_copy(presealed["children"])
    return observed, "lstat_only_presealed_empty"


def _restore_reference_descriptor(path: Path, label: str) -> dict[str, Any]:
    value = _restore_directory_snapshot(path, label)
    if value["mode"] != RESTORE_OWNER_REFERENCE_MODE:
        raise CaptureError("RESTORE_OWNER_REFERENCE_INVALID", label + " mode")
    if (value["uid"], value["gid"]) != (os.getuid(), os.getgid()):
        raise CaptureError("RESTORE_OWNER_REFERENCE_INVALID", label + " owner")
    if value["children"]:
        raise CaptureError("RESTORE_OWNER_REFERENCE_NOT_EMPTY", label)
    return value


def _restore_owner_command(container_name: str, role: str) -> list[str]:
    if NAME_RE.fullmatch(container_name) is None:
        raise CaptureError("CONTAINER_NAME_INVALID", container_name)
    item = next((entry for entry in RESTORE_OWNER_REFERENCE_MOUNTS
                 if entry["role"] == role), None)
    if item is None:
        raise CaptureError("RESTORE_OWNER_ROLE_INVALID", role)
    return [
        "docker", "exec", container_name, "chown",
        "--reference=" + item["target"], item["target_partial"],
    ]


def _restore_owner_phase_metadata(plan: Mapping[str, Any], phase: str,
                                  *, container_name: str | None = None) -> dict[str, Any]:
    role = "build" if phase == "restore_build_partial_owner" else "runtime"
    if phase not in RESTORE_OWNER_PHASES:
        raise CaptureError("RESTORE_OWNER_PHASE_INVALID", phase)
    argv = plan.get("phase_argv", {}).get(phase)
    name = container_name or plan.get("container_name")
    if not isinstance(name, str):
        raise CaptureError("RESTORE_OWNER_CONTAINER_INVALID", phase)
    expected = _restore_owner_command(name, role)
    if argv is not None:
        if argv != expected:
            raise CaptureError("RESTORE_OWNER_ARGV_INVALID", phase)
    else:
        phase_hashes = plan.get("phase_argv_sha256")
        if (not isinstance(phase_hashes, Mapping) or
                phase_hashes.get(phase) != _command_hash(expected)):
            raise CaptureError("RESTORE_OWNER_ARGV_INVALID", phase)
    return {"role": role, "argv": list(expected),
            "argv_sha256": _command_hash(expected)}


def _restore_owner_reference_bindings(plan: Mapping[str, Any], root: Path) -> list[dict[str, Any]]:
    specs = plan.get("restore_owner_reference_mounts")
    expected = _restore_owner_mount_specs(root)
    if not isinstance(specs, list) or specs != expected:
        raise CaptureError("RESTORE_OWNER_MOUNT_INVALID", "plan")
    before = plan.get("_restore_owner_reference_before")
    after = plan.get("_restore_owner_reference_after")
    runtime = plan.get("_restore_owner_reference_runtime")
    if not isinstance(before, list) or len(before) != len(specs):
        before = [None] * len(specs)
    if not isinstance(after, list) or len(after) != len(specs):
        after = [None] * len(specs)
    if not isinstance(runtime, list) or len(runtime) != len(specs):
        runtime = [None] * len(specs)
    result = []
    for index, (spec, pre, post, observed) in enumerate(
            zip(specs, before, after, runtime)):
        if set(spec) != _RESTORE_MOUNT_FIELDS:
            raise CaptureError("RESTORE_OWNER_MOUNT_INVALID", str(index))
        item = dict(spec)
        item.update({"pre": _immutable_copy(pre), "post": _immutable_copy(post),
                     "runtime": _immutable_copy(observed)})
        result.append(item)
    return result


def _validate_restore_directory(value: Any, label: str, *, reference: bool = False,
                                 expected_owner: tuple[int, int] | None = None,
                                 require_empty: bool = False) -> None:
    if not isinstance(value, Mapping) or set(value) != _RESTORE_DIRECTORY_FIELDS:
        raise CaptureError("RESTORE_OWNER_DESCRIPTOR_INVALID", label)
    _safe_abs(value.get("path"), label + " path")
    if value.get("type") != "directory":
        raise CaptureError("RESTORE_OWNER_DESCRIPTOR_INVALID", label)
    for key in ("device", "inode", "uid", "gid", "mode", "nlink"):
        if type(value.get(key)) is not int or value[key] < 0:
            raise CaptureError("RESTORE_OWNER_DESCRIPTOR_INVALID", label)
    if value["inode"] <= 0 or value["nlink"] < 2:
        raise CaptureError("RESTORE_OWNER_DESCRIPTOR_INVALID", label)
    if not isinstance(value["children"], list) or len(value["children"]) > 64:
        raise CaptureError("RESTORE_OWNER_DESCRIPTOR_INVALID", label)
    names = []
    for child in value["children"]:
        if not isinstance(child, Mapping) or set(child) != _RESTORE_CHILD_FIELDS:
            raise CaptureError("RESTORE_OWNER_CHILD_INVALID", label)
        if (not isinstance(child["name"], str) or not child["name"] or
                "/" in child["name"] or "\\" in child["name"] or
                child["name"] in names):
            raise CaptureError("RESTORE_OWNER_CHILD_INVALID", label)
        names.append(child["name"])
        if child["type"] not in {"regular", "directory", "symlink", "fifo", "socket", "other"}:
            raise CaptureError("RESTORE_OWNER_CHILD_INVALID", label)
        for key in ("device", "inode", "uid", "gid", "mode", "nlink"):
            if type(child.get(key)) is not int or child[key] < 0:
                raise CaptureError("RESTORE_OWNER_CHILD_INVALID", label)
        if child["inode"] <= 0 or child["nlink"] < 1:
            raise CaptureError("RESTORE_OWNER_CHILD_INVALID", label)
    if value["mode"] != RESTORE_OWNER_REFERENCE_MODE:
        raise CaptureError("RESTORE_OWNER_MODE_INVALID", label)
    if expected_owner is not None and (value["uid"], value["gid"]) != expected_owner:
        raise CaptureError("RESTORE_OWNER_OWNER_INVALID", label)
    if reference and value["children"]:
        raise CaptureError("RESTORE_OWNER_REFERENCE_NOT_EMPTY", label)
    if require_empty and value["children"]:
        raise CaptureError("RESTORE_OWNER_CHILD_NOT_EMPTY", label)


def _validate_restore_owner_mount_bindings(value: Any, root: Path,
                                           *, required: bool,
                                           reached: bool,
                                           runtime_required: bool = False) -> None:
    expected = _restore_owner_mount_specs(root)
    if not isinstance(value, list) or len(value) != len(expected):
        raise CaptureError("RESTORE_OWNER_MOUNT_INVALID", "receipt")
    seen = set()
    for index, (item, fixed) in enumerate(zip(value, expected)):
        if not isinstance(item, Mapping) or set(item) != _RESTORE_MOUNT_FIELDS | {
                "pre", "post", "runtime"}:
            raise CaptureError("RESTORE_OWNER_MOUNT_INVALID", str(index))
        if dict(item, pre=None, post=None, runtime=None) != dict(
                fixed, pre=None, post=None, runtime=None):
            raise CaptureError("RESTORE_OWNER_MOUNT_INVALID", str(index))
        if item["target"] in seen:
            raise CaptureError("RESTORE_OWNER_MOUNT_DUPLICATE", item["target"])
        seen.add(item["target"])
        source = Path(item["source"])
        _parents_no_symlink(source, "restore reference source")
        for field in ("pre", "post"):
            descriptor = item[field]
            if descriptor is None:
                if required and reached:
                    raise CaptureError("RESTORE_OWNER_DESCRIPTOR_MISSING", str(index))
                continue
            _validate_restore_directory(
                descriptor, "restore reference {} {}".format(index, field),
                reference=True, expected_owner=(os.getuid(), os.getgid()),
                require_empty=True)
            if descriptor["path"] != str(source):
                raise CaptureError("RESTORE_OWNER_DESCRIPTOR_PATH_INVALID", str(index))
        if item["pre"] is not None and item["post"] is not None:
            if item["pre"] != item["post"]:
                raise CaptureError("RESTORE_OWNER_REFERENCE_DRIFT", str(index))
            actual = _restore_reference_descriptor(source, "restore reference readback")
            if actual != item["post"]:
                raise CaptureError("RESTORE_OWNER_REFERENCE_DRIFT", str(index))
        observed = item["runtime"]
        if observed is None:
            if runtime_required:
                raise CaptureError("RESTORE_OWNER_RUNTIME_MISSING", str(index))
        elif (not isinstance(observed, Mapping) or
                set(observed) != _RESTORE_MOUNT_FIELDS or
                observed != fixed):
            raise CaptureError("RESTORE_OWNER_RUNTIME_INVALID", str(index))


def _validate_restore_owner_phase_record(value: Any, root: Path, plan: Mapping[str, Any],
                                         phase_record: Mapping[str, Any],
                                         *, container_name: str | None = None,
                                         profile_sha256: str | None = None,
                                         image_digest: str | None = None) -> None:
    if not isinstance(value, Mapping) or set(value) != {
            "phase", "role", "argv", "argv_sha256", "returncode",
            "observed_returncode", "stdout", "stderr", "reference_mount",
            "target_relative", "target", "apt_identity", "target_presealed",
            "target_before_observation", "reference_before", "reference_after",
            "target_before", "target_after", "children_before", "children_after",
            "status"}:
        raise CaptureError("RESTORE_OWNER_PHASE_INVALID", phase_record.get("phase", ""))
    phase = phase_record.get("phase")
    if value["phase"] != phase or phase not in RESTORE_OWNER_PHASES:
        raise CaptureError("RESTORE_OWNER_PHASE_INVALID", str(phase))
    role = "build" if phase == RESTORE_OWNER_PHASES[0] else "runtime"
    if value["role"] != role:
        raise CaptureError("RESTORE_OWNER_PHASE_INVALID", phase)
    metadata = _restore_owner_phase_metadata(
        plan, phase, container_name=container_name)
    if value["argv"] != metadata["argv"] or value["argv_sha256"] != metadata["argv_sha256"]:
        raise CaptureError("RESTORE_OWNER_ARGV_INVALID", phase)
    for key in ("returncode", "observed_returncode"):
        if type(value[key]) is not int or value[key] != phase_record[key]:
            raise CaptureError("RESTORE_OWNER_PHASE_INVALID", phase)
    if value["status"] not in {"PASS", "FAILED"}:
        raise CaptureError("RESTORE_OWNER_PHASE_INVALID", phase)
    if not isinstance(value["stdout"], Mapping) or not isinstance(value["stderr"], Mapping):
        raise CaptureError("RESTORE_OWNER_PHASE_INVALID", phase)
    if value["stdout"] != phase_record["stdout"] or value["stderr"] != phase_record["stderr"]:
        raise CaptureError("RESTORE_OWNER_PHASE_INVALID", phase)
    fixed = next(item for item in RESTORE_OWNER_REFERENCE_MOUNTS if item["role"] == role)
    mount = value["reference_mount"]
    expected_mount = next(item for item in plan["restore_owner_reference_mounts"]
                          if item["role"] == role)
    if mount != expected_mount:
        raise CaptureError("RESTORE_OWNER_MOUNT_INVALID", phase)
    if value["target_relative"] != ("debs-build/partial" if role == "build"
                                     else "debs-runtime/partial") or \
            value["target"] != fixed["target_partial"]:
        raise CaptureError("RESTORE_OWNER_TARGET_INVALID", phase)
    if value["apt_identity"] != _apt_transient_owner_binding(
            plan, profile_sha256=profile_sha256, image_digest=image_digest):
        raise CaptureError("RESTORE_OWNER_TRANSIENT_OWNER_INVALID", phase)
    if value["target_before_observation"] not in {
            "scandir_presealed_empty", "lstat_only_presealed_empty"}:
        raise CaptureError("RESTORE_OWNER_OBSERVATION_INVALID", phase)
    expected_owner = (os.getuid(), os.getgid())
    _validate_restore_directory(value["reference_before"], phase + " reference before",
                                reference=True, expected_owner=expected_owner,
                                require_empty=True)
    _validate_restore_directory(value["reference_after"], phase + " reference after",
                                reference=True, expected_owner=expected_owner,
                                require_empty=True)
    if value["reference_before"] != value["reference_after"]:
        raise CaptureError("RESTORE_OWNER_REFERENCE_DRIFT", phase)
    _validate_restore_directory(value["target_before"], phase + " target before",
                                expected_owner=None, require_empty=True)
    _validate_restore_directory(value["target_presealed"], phase + " target presealed",
                                expected_owner=expected_owner, require_empty=True)
    _validate_restore_directory(value["target_after"], phase + " target after",
                                expected_owner=expected_owner, require_empty=True)
    if value["target_presealed"]["path"] != value["target_before"]["path"] or \
            _restore_directory_identity(value["target_presealed"]) != \
            _restore_directory_identity(value["target_before"]):
        raise CaptureError("RESTORE_OWNER_TARGET_DRIFT", phase)
    before_owner = (value["target_before"]["uid"], value["target_before"]["gid"])
    transient_owner = (value["apt_identity"]["uid"], value["apt_identity"]["gid"])
    if value["target_before_observation"] == "lstat_only_presealed_empty":
        if before_owner != transient_owner:
            raise CaptureError("RESTORE_OWNER_TRANSIENT_OWNER_INVALID", phase)
    elif before_owner != expected_owner or value["target_before"] != value["target_presealed"]:
        raise CaptureError("RESTORE_OWNER_TARGET_DRIFT", phase)
    actual_reference = _restore_reference_descriptor(
        Path(expected_mount["source"]), phase + " reference readback")
    if actual_reference != value["reference_after"]:
        raise CaptureError("RESTORE_OWNER_REFERENCE_DRIFT", phase)
    actual_target = _restore_directory_snapshot(
        root / value["target_relative"], phase + " target readback")
    if actual_target != value["target_after"]:
        raise CaptureError("RESTORE_OWNER_TARGET_DRIFT", phase)
    for before, after in ((value["target_before"], value["target_after"]),):
        for key in ("path", "type", "device", "inode", "mode", "nlink"):
            if before[key] != after[key]:
                raise CaptureError("RESTORE_OWNER_TARGET_DRIFT", phase)
    if value["children_before"] != value["target_before"]["children"] or \
            value["children_after"] != value["target_after"]["children"] or \
            value["children_before"] or value["children_after"]:
        raise CaptureError("RESTORE_OWNER_CHILD_NOT_EMPTY", phase)
    if phase_record["observed_returncode"] == 0 and value["status"] != "PASS":
        raise CaptureError("RESTORE_OWNER_PHASE_INVALID", phase)
    if phase_record["observed_returncode"] != 0 and value["status"] != "FAILED":
        raise CaptureError("RESTORE_OWNER_PHASE_INVALID", phase)
    if value["status"] == "PASS" and (
            value["target_after"]["uid"], value["target_after"]["gid"]) != expected_owner:
        raise CaptureError("RESTORE_OWNER_OWNER_INVALID", phase)


def _restore_owner_projection(plan: Mapping[str, Any], root: Path,
                              records: list[Mapping[str, Any]]) -> dict[str, Any]:
    references = _restore_owner_reference_bindings(plan, root)
    phase_values = []
    stored = plan.get("_restore_owner_records", {})
    if not isinstance(stored, Mapping):
        stored = {}
    for phase in RESTORE_OWNER_PHASES:
        record = stored.get(phase)
        if record is None:
            continue
        phase_record = next((item for item in records if item.get("phase") == phase), None)
        if phase_record is None:
            raise CaptureError("RESTORE_OWNER_PHASE_INVALID", phase)
        phase_values.append(_immutable_copy(record))
    value = {
        "schema": RESTORE_OWNER_SCHEMA,
        "schema_version": RESTORE_OWNER_SCHEMA_VERSION,
        "policy": dict(RESTORE_OWNER_POLICY),
        "references": references,
        "phases": phase_values,
    }
    value["identity_sha256"] = canonical_hash(value, "identity_sha256")
    return value


def _validate_restore_owner_projection(value: Any, root: Path,
                                       plan: Mapping[str, Any],
                                       phases: list[Mapping[str, Any]],
                                       *, container_name: str | None = None,
                                       profile_sha256: str | None = None,
                                       image_digest: str | None = None) -> None:
    """Reopen the owner-restore policy, mounts, and reached phase proofs."""
    if not isinstance(value, Mapping) or set(value) != {
            "schema", "schema_version", "policy", "references", "phases",
            "identity_sha256"}:
        raise CaptureError("RESTORE_OWNER_PROJECTION_INVALID", "fields")
    if value["schema"] != RESTORE_OWNER_SCHEMA or \
            value["schema_version"] != RESTORE_OWNER_SCHEMA_VERSION or \
            value["policy"] != RESTORE_OWNER_POLICY or \
            value["identity_sha256"] != canonical_hash(value, "identity_sha256"):
        raise CaptureError("RESTORE_OWNER_PROJECTION_INVALID", "policy/identity")
    restore_indices = [index for index, item in enumerate(phases)
                       if item.get("phase") in RESTORE_OWNER_PHASES]
    expected_phase_values = [
        phase for phase in RESTORE_OWNER_PHASES
        if any(item.get("phase") == phase for item in phases)
    ]
    phase_values = value["phases"]
    if (not isinstance(phase_values, list) or
            [item.get("phase") for item in phase_values
             if isinstance(item, Mapping)] != expected_phase_values):
        raise CaptureError("RESTORE_OWNER_PHASE_ORDER_INVALID", "projection")
    if len(phase_values) != len(expected_phase_values):
        raise CaptureError("RESTORE_OWNER_PHASE_ORDER_INVALID", "projection count")
    _validate_restore_owner_mount_bindings(
        value["references"], root, required=bool(expected_phase_values),
        reached=bool(expected_phase_values), runtime_required=bool(
            plan.get("rosdep_prepare_contract", {}).get("required") is True and
            any(item.get("phase") == "post_disconnect_inspect" and
                item.get("returncode") == 0 for item in phases)))
    for item in phase_values:
        phase = item.get("phase") if isinstance(item, Mapping) else None
        phase_record = next((record for record in phases
                             if record.get("phase") == phase), None)
        if phase_record is None:
            raise CaptureError("RESTORE_OWNER_PHASE_ORDER_INVALID", str(phase))
        _validate_restore_owner_phase_record(
            item, root, plan, phase_record, container_name=container_name,
            profile_sha256=profile_sha256, image_digest=image_digest)
    expected_restore_indices = [PHASES.index(phase)
                                for phase in expected_phase_values]
    if restore_indices != expected_restore_indices:
        raise CaptureError("RESTORE_OWNER_PHASE_ORDER_INVALID", "phase list")


def _prepare_phase_argv(
        container_name: str, output_root: Path | str | None = None,
        distro: str = "humble") -> list[str]:
    """Build the one fixed prepare argv used by plan and readback."""
    if NAME_RE.fullmatch(container_name) is None:
        raise CaptureError("CONTAINER_NAME_INVALID", container_name)
    if distro not in {"humble", "jazzy"}:
        raise CaptureError("DISTRO_INVALID", distro)
    if output_root is None:
        logical_output = ROSDEP_PREPARE_CONTAINER_OUTPUT_ROOT
    else:
        logical_output = str(_safe_abs(
            str(output_root), "prepare logical output root"
        ) / ROSDEP_PREPARE_RELATIVE)
    return [
        "docker", "exec", container_name, "python3", ROSDEP_PREPARE_CONTAINER_TOOL,
        "prepare", "--output-root", ROSDEP_PREPARE_CONTAINER_OUTPUT_ROOT,
        "--logical-output-root", logical_output,
        "--discovery-root", ROSDEP_PREPARE_CONTAINER_DISCOVERY_ROOT,
        "--profile", ROSDEP_PREPARE_CONTAINER_PROFILE,
        "--distro", distro, "--precreated",
    ]


def _rosdep_resolution_phase_argv(container_name: str, distro: str) -> list[str]:
    """Build the disconnected install argv with the prepare cache namespace."""
    if NAME_RE.fullmatch(container_name) is None:
        raise CaptureError("CONTAINER_NAME_INVALID", container_name)
    if distro not in {"humble", "jazzy"}:
        raise CaptureError("DISTRO_INVALID", distro)
    argv = ["docker", "exec"]
    for key in sorted(ROSDEP_PREPARE.EXPECTED_NETWORK_ENV):
        argv.extend([
            "--env", "{}={}".format(
                key, ROSDEP_PREPARE.EXPECTED_NETWORK_ENV[key]
            ),
        ])
    argv.extend([
        container_name, "rosdep", "install", "-r", "-y", "--ignore-src",
        "--rosdistro", distro, "--sources-cache-dir",
        ROSDEP_PREPARE.EXPECTED_GENERATED_ROSDEP_CACHE, "--from-paths",
        "/workspace/src/graph_based_slam",
    ])
    return argv


_PREPARE_MOUNT_DESCRIPTOR_FIELDS = frozenset({
    "path", "type", "device", "inode", "uid", "gid", "mode", "nlink",
})


def _prepare_mount_source_descriptor(path: Path, label: str,
                                     *, required: bool) -> dict[str, Any] | None:
    """Capture one no-follow directory source without repairing it."""
    path = _safe_abs(str(path), label)
    _parents_no_symlink(path, label)
    try:
        info = path.lstat()
    except OSError as error:
        if not required and isinstance(error, FileNotFoundError):
            return None
        raise CaptureError("PREPARE_MOUNT_SOURCE_MISSING", label) from error
    if (stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or
            info.st_nlink < 2):
        raise CaptureError("PREPARE_MOUNT_SOURCE_INVALID", label)
    return {
        "path": str(path), "type": "directory", "device": int(info.st_dev),
        "inode": int(info.st_ino), "uid": int(info.st_uid),
        "gid": int(info.st_gid), "mode": stat.S_IMODE(info.st_mode),
        "nlink": int(info.st_nlink),
    }


def _prepare_mount_descriptor_snapshot(plan: Mapping[str, Any], root: Path,
                                       label: str) -> list[dict[str, Any]]:
    """Snapshot every fixed mount source in the plan order."""
    required = plan.get("rosdep_prepare_contract", {}).get("required") is True
    result = []
    for index, mount in enumerate(plan.get("rosdep_prepare_mounts", [])):
        source = _safe_abs(mount.get("source"),
                           "{} mount {} source".format(label, index))
        descriptor = _prepare_mount_source_descriptor(
            source, "{} mount {}".format(label, index), required=required)
        result.append(descriptor)
    if len(result) != 8:
        raise CaptureError("PREPARE_MOUNT_COUNT_INVALID", label)
    return result


def _prepare_mount_bindings(plan: Mapping[str, Any], root: Path) -> list[dict[str, Any]]:
    """Combine fixed mount specs with immutable pre/post/runtime observations."""
    specs = plan.get("rosdep_prepare_mounts")
    if not isinstance(specs, list) or len(specs) != 8:
        raise CaptureError("PREPARE_MOUNT_COUNT_INVALID", "plan")
    before = plan.get("_rosdep_prepare_mounts_before")
    after = plan.get("_rosdep_prepare_mounts_after")
    runtime = plan.get("_rosdep_prepare_mounts_runtime")
    if not isinstance(before, list) or len(before) != len(specs):
        before = [None] * len(specs)
    if not isinstance(after, list) or len(after) != len(specs):
        after = [None] * len(specs)
    if not isinstance(runtime, list) or len(runtime) != len(specs):
        runtime = [None] * len(specs)
    result = []
    for index, (spec, pre, post, observed) in enumerate(
            zip(specs, before, after, runtime)):
        if not isinstance(spec, Mapping) or set(spec) != {
                "source", "target", "read_only", "type", "noexec"}:
            raise CaptureError("PREPARE_MOUNT_SPEC_INVALID", str(index))
        item = dict(spec)
        item.update({"pre": _immutable_copy(pre), "post": _immutable_copy(post),
                     "runtime": _immutable_copy(observed)})
        result.append(item)
    return result


def _validate_prepare_mount_descriptor(value: Any, label: str) -> None:
    if value is None:
        return
    if not isinstance(value, Mapping) or set(value) != _PREPARE_MOUNT_DESCRIPTOR_FIELDS:
        raise CaptureError("PREPARE_MOUNT_DESCRIPTOR_INVALID", label)
    path = _safe_abs(value.get("path"), label + " path")
    del path
    if value.get("type") != "directory":
        raise CaptureError("PREPARE_MOUNT_DESCRIPTOR_INVALID", label)
    for key in ("device", "inode", "uid", "gid", "mode", "nlink"):
        if type(value.get(key)) is not int or value[key] < 0:
            raise CaptureError("PREPARE_MOUNT_DESCRIPTOR_INVALID", label)
    if value["inode"] <= 0 or value["nlink"] < 2 or value["mode"] != 0o700:
        raise CaptureError("PREPARE_MOUNT_DESCRIPTOR_INVALID", label)


def _validate_prepare_mount_bindings(value: Any, root: Path, *, required: bool,
                                     prepare_reached: bool,
                                     runtime_required: bool = False) -> list[dict[str, Any]]:
    """Validate the static mount contract and all available identities."""
    expected = _prepare_mount_specs(root)
    if not isinstance(value, list) or len(value) != len(expected):
        raise CaptureError("PREPARE_MOUNT_COUNT_INVALID", "receipt")
    targets = []
    for index, (item, fixed) in enumerate(zip(value, expected)):
        if not isinstance(item, Mapping) or set(item) != {
                "source", "target", "read_only", "type", "noexec",
                "pre", "post", "runtime"}:
            raise CaptureError("PREPARE_MOUNT_SPEC_INVALID", str(index))
        if dict(item, pre=None, post=None, runtime=None) != dict(
                fixed, pre=None, post=None, runtime=None):
            raise CaptureError("PREPARE_MOUNT_SPEC_INVALID", str(index))
        if item["target"] in targets:
            raise CaptureError("PREPARE_MOUNT_DUPLICATE", item["target"])
        targets.append(item["target"])
        source = _safe_abs(item["source"], "prepare mount source")
        _parents_no_symlink(source, "prepare mount source")
        for field in ("pre", "post", "runtime"):
            if field != "runtime":
                _validate_prepare_mount_descriptor(
                    item[field], "prepare mount {} {}".format(index, field))
        if required and prepare_reached and (item["pre"] is None or item["post"] is None):
            raise CaptureError("PREPARE_MOUNT_DESCRIPTOR_MISSING", str(index))
        if item["pre"] is not None and item["post"] is not None:
            if item["pre"]["path"] != str(source) or item["post"]["path"] != str(source):
                raise CaptureError("PREPARE_MOUNT_DESCRIPTOR_PATH_INVALID", str(index))
            for key in ("type", "device", "inode", "uid", "gid", "mode"):
                if item["pre"][key] != item["post"][key]:
                    raise CaptureError("PREPARE_MOUNT_DESCRIPTOR_DRIFT", str(index))
            if item["pre"]["nlink"] < 2 or item["post"]["nlink"] < 2:
                raise CaptureError("PREPARE_MOUNT_DESCRIPTOR_DRIFT", str(index))
            actual = _prepare_mount_source_descriptor(
                source, "prepare mount {} readback".format(index), required=True)
            if actual != item["post"]:
                raise CaptureError("PREPARE_MOUNT_DESCRIPTOR_DRIFT", str(index))
        if item["runtime"] is not None:
            runtime_value = item["runtime"]
            if (not isinstance(runtime_value, Mapping) or
                    set(runtime_value) != {
                        "source", "target", "read_only", "type", "noexec"} or
                    runtime_value != fixed):
                raise CaptureError("PREPARE_MOUNT_RUNTIME_INVALID", str(index))
        elif runtime_required:
            raise CaptureError("PREPARE_MOUNT_RUNTIME_MISSING", str(index))
    return [dict(item) for item in value]


def _prepare_argv_metadata(plan: Mapping[str, Any]) -> dict[str, Any]:
    """Return the fixed prepare argv and the script identity it executes."""
    phase_argv = plan.get("phase_argv")
    name = plan.get("container_name")
    if not isinstance(phase_argv, Mapping) or not isinstance(name, str):
        raise CaptureError("PREPARE_ARGV_INVALID", "plan")
    argv = phase_argv.get("rosdep_prepare")
    output_root = plan.get("output_root")
    distro = plan.get("distro")
    if not isinstance(output_root, str):
        raise CaptureError("PREPARE_ARGV_INVALID", "output root")
    if argv != _prepare_phase_argv(name, output_root, distro):
        raise CaptureError("PREPARE_ARGV_INVALID", "fixed argv")
    contract = plan.get("rosdep_prepare_contract")
    bundle = contract.get("input_bundle") if isinstance(contract, Mapping) else None
    files = bundle.get("files") if isinstance(bundle, Mapping) else None
    tool = files[0] if isinstance(files, list) and files else None
    if (not isinstance(tool, Mapping) or tool.get("role") != "tool" or
            not isinstance(tool.get("sha256"), str)):
        raise CaptureError("PREPARE_ARGV_INVALID", "tool identity")
    return {
        "argv": list(argv),
        "argv_sha256": _command_hash(list(argv)),
        "script_path": ROSDEP_PREPARE_CONTAINER_TOOL,
        "script_sha256": tool["sha256"],
    }


def _prepare_runtime_mount_observation(root: Path, plan: Mapping[str, Any],
                                       records: list[Mapping[str, Any]]) -> list[dict[str, Any]] | None:
    """Project Docker's optional JSON inspect output onto fixed mount targets.

    Unit runners historically emit a text marker for the inspect phase.  That
    marker is accepted only for the non-production plan seam.  A required
    production plan must provide Docker's JSON ``Mounts`` observation, and
    every fixed source/target/read-only/type tuple is compared exactly.
    """
    required = plan.get("rosdep_prepare_contract", {}).get("required") is True
    record = next((item for item in records
                   if item.get("phase") == "post_disconnect_inspect"), None)
    if record is None:
        return None
    stream = record.get("stdout")
    if not isinstance(stream, Mapping):
        if required:
            raise CaptureError("PREPARE_MOUNT_RUNTIME_INVALID", "inspect stdout")
        return None
    try:
        relative = _safe_rel(stream.get("path"), "prepare inspect log")
        raw, _ = _read_bounded(root / relative, "prepare inspect log", MAX_LOG_BYTES)
    except CaptureError:
        if required:
            raise
        return None
    if not raw.strip():
        if required:
            raise CaptureError("PREPARE_MOUNT_RUNTIME_INVALID", "empty inspect")
        return None
    try:
        decoded = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        if required:
            raise CaptureError("PREPARE_MOUNT_RUNTIME_INVALID", "inspect JSON")
        return None
    if not isinstance(decoded, list) or len(decoded) != 1 or not isinstance(decoded[0], Mapping):
        if not required:
            return None
        raise CaptureError("PREPARE_MOUNT_RUNTIME_INVALID", "inspect shape")
    mounts = decoded[0].get("Mounts")
    if not isinstance(mounts, list):
        if not required:
            return None
        raise CaptureError("PREPARE_MOUNT_RUNTIME_INVALID", "inspect mounts")
    fixed = plan.get("rosdep_prepare_mounts")
    if not isinstance(fixed, list) or len(fixed) != 8:
        raise CaptureError("PREPARE_MOUNT_COUNT_INVALID", "runtime plan")
    by_target: dict[str, Mapping[str, Any]] = {}
    for mount in mounts:
        if not isinstance(mount, Mapping):
            raise CaptureError("PREPARE_MOUNT_RUNTIME_INVALID", "inspect mount")
        target = mount.get("Destination")
        if not isinstance(target, str) or target in by_target:
            raise CaptureError("PREPARE_MOUNT_RUNTIME_INVALID", "inspect target")
        by_target[target] = mount
    result = []
    for item in fixed:
        target = item["target"]
        mount = by_target.get(target)
        if not isinstance(mount, Mapping):
            raise CaptureError("PREPARE_MOUNT_RUNTIME_MISSING", target)
        if (mount.get("Type") != "bind" or mount.get("Source") != item["source"] or
                type(mount.get("RW")) is not bool or
                mount["RW"] != (not item["read_only"])):
            raise CaptureError("PREPARE_MOUNT_RUNTIME_INVALID", target)
        result.append(dict(item))
    return result


def _restore_owner_runtime_mount_observation(
        root: Path, plan: Mapping[str, Any],
        records: list[Mapping[str, Any]]) -> list[dict[str, Any]] | None:
    """Read Docker inspect's bind observations for the two reference mounts."""
    required = plan.get("rosdep_prepare_contract", {}).get("required") is True
    record = next((item for item in records
                   if item.get("phase") == "post_disconnect_inspect"), None)
    if record is None:
        return None
    stream = record.get("stdout")
    if not isinstance(stream, Mapping):
        if required:
            raise CaptureError("RESTORE_OWNER_RUNTIME_INVALID", "inspect stdout")
        return None
    try:
        relative = _safe_rel(stream.get("path"), "restore inspect log")
        raw, _ = _read_bounded(root / relative, "restore inspect log", MAX_LOG_BYTES)
    except CaptureError:
        if required:
            raise
        return None
    if not raw.strip():
        if required:
            raise CaptureError("RESTORE_OWNER_RUNTIME_INVALID", "empty inspect")
        return None
    try:
        decoded = json.loads(raw.decode("utf-8"))
    except (UnicodeDecodeError, json.JSONDecodeError):
        if required:
            raise CaptureError("RESTORE_OWNER_RUNTIME_INVALID", "inspect JSON")
        return None
    if not isinstance(decoded, list) or len(decoded) != 1 or not isinstance(decoded[0], Mapping):
        if not required:
            return None
        raise CaptureError("RESTORE_OWNER_RUNTIME_INVALID", "inspect shape")
    mounts = decoded[0].get("Mounts")
    if not isinstance(mounts, list):
        if required:
            raise CaptureError("RESTORE_OWNER_RUNTIME_INVALID", "inspect mounts")
        return None
    by_target: dict[str, Mapping[str, Any]] = {}
    for mount in mounts:
        if not isinstance(mount, Mapping):
            raise CaptureError("RESTORE_OWNER_RUNTIME_INVALID", "inspect mount")
        target = mount.get("Destination")
        if not isinstance(target, str) or target in by_target:
            raise CaptureError("RESTORE_OWNER_RUNTIME_INVALID", "inspect target")
        by_target[target] = mount
    fixed = plan.get("restore_owner_reference_mounts")
    if not isinstance(fixed, list) or len(fixed) != 2:
        raise CaptureError("RESTORE_OWNER_MOUNT_INVALID", "runtime plan")
    result = []
    for item in fixed:
        target = item["target"]
        mount = by_target.get(target)
        if not isinstance(mount, Mapping):
            if required:
                raise CaptureError("RESTORE_OWNER_RUNTIME_MISSING", target)
            return None
        if (mount.get("Type") != "bind" or mount.get("Source") != item["source"] or
                type(mount.get("RW")) is not bool or mount["RW"] is not False):
            raise CaptureError("RESTORE_OWNER_RUNTIME_INVALID", target)
        result.append(dict(item))
    return result


def _snapshot_capture_directory_layout(root: Path, *, include_prefetch: bool) -> dict[str, Any]:
    """Reopen the fixed bind layout without creating or repairing anything."""
    try:
        return EVIDENCE_DIRECTORIES.snapshot_existing(
            root, _capture_directory_paths(include_prefetch=include_prefetch))
    except EVIDENCE_DIRECTORIES.EvidenceDirectoryError as error:
        raise CaptureError(error.kind, str(error)) from error


def _capture_prefetch_layout_present(root: Path) -> bool:
    """Detect the pre-created optional-prefetch subtree without following it."""
    marker = root / CAPTURE_PREFETCH_DIRECTORY_RELATIVES[0]
    try:
        marker.lstat()
    except FileNotFoundError:
        return False
    except OSError as error:
        raise CaptureError("DIRECTORY_OPEN_FAILED", str(marker)) from error
    # The subsequent exact snapshot rejects symlinks, wrong modes, missing
    # descendants, and ownership drift.  Presence only selects the fixed
    # absent/present shape that the outer contract will compare byte-for-byte.
    return True


def _verify_capture_directory_layout(root: Path, contract: Mapping[str, Any],
                                     *, allow_terminal_closure: bool = False
                                     ) -> dict[str, Any]:
    """Require exact host directory identities after every container phase."""
    try:
        recorded = contract.get("allowed_root_directory_additions", [])
        if recorded not in ([], ["closure"]):
            raise EVIDENCE_DIRECTORIES.EvidenceDirectoryError(
                "DIRECTORY_CONTRACT_INVALID", "capture root additions")
        additions = ["closure"] if allow_terminal_closure else list(recorded)
        after = EVIDENCE_DIRECTORIES.verify_snapshot(
            root, dict(contract),
            allowed_root_directory_additions=additions)
        return EVIDENCE_DIRECTORIES.finalize(
            dict(contract), after,
            allowed_root_directory_additions=additions)
    except EVIDENCE_DIRECTORIES.EvidenceDirectoryError as error:
        raise CaptureError(error.kind, str(error)) from error


def _directory_contract_wrapper(contract: Mapping[str, Any]) -> dict[str, Any]:
    """Wrap a host directory contract as an immutable, sidecar-sealed JSON."""
    if not isinstance(contract, Mapping):
        raise CaptureError("DIRECTORY_CONTRACT_INVALID", "contract is not an object")
    value = {
        "schema": CAPTURE_DIRECTORY_CONTRACT_SCHEMA,
        "schema_version": CAPTURE_DIRECTORY_CONTRACT_VERSION,
        "directory": dict(contract),
    }
    value["canonical_sha256"] = canonical_hash(value)
    return value


def _seal_directory_contract(root: Path, contract: Mapping[str, Any]) -> dict[str, Any]:
    value = _directory_contract_wrapper(contract)
    try:
        return _seal_pair(root, DIRECTORY_CONTRACT_NAME, value)
    except Exception as error:
        if isinstance(error, CaptureError):
            raise
        raise CaptureError("DIRECTORY_CONTRACT_SEAL_FAILED", str(error)) from error


def _sealed_pair_projection(root: Path, filename: str, label: str,
                            maximum: int = MAX_JSON_BYTES) -> dict[str, Any]:
    """Reopen one immutable JSON/sidecar pair for an outer receipt binding.

    The companion directory contract is intentionally outside the directory
    contract document itself.  Binding the complete pair into the outer
    receipt prevents a caller from replacing a valid contract and merely
    rehashing the receipt while retaining the same path.
    """
    path = root / filename
    with_sidecar = _descriptor(path, label, maximum, sidecar=True)
    sidecar = with_sidecar.pop("sidecar", None)
    if not isinstance(sidecar, Mapping):
        raise CaptureError("ARTIFACT_BINDING_INVALID", label + " sidecar")
    return {"file": with_sidecar, "sidecar": dict(sidecar)}


def _read_directory_contract(root: Path, *, include_prefetch: bool,
                             receipt_status: str | None = None) -> dict[str, Any]:
    """Reopen the companion contract and compare it with live directories."""
    path = root / DIRECTORY_CONTRACT_NAME
    data, _ = _read_bounded(path, "directory contract", MAX_JSON_BYTES, allow_empty=False)
    _descriptor(path, "directory contract", MAX_JSON_BYTES, sidecar=True)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError("DIRECTORY_CONTRACT_INVALID", "JSON") from error
    if not isinstance(value, Mapping) or set(value) != {
            "schema", "schema_version", "directory", "canonical_sha256"} or \
            value["schema"] != CAPTURE_DIRECTORY_CONTRACT_SCHEMA or \
            value["schema_version"] != CAPTURE_DIRECTORY_CONTRACT_VERSION or \
            value["canonical_sha256"] != canonical_hash(value):
        raise CaptureError("DIRECTORY_CONTRACT_INVALID", "wrapper identity")
    directory = value["directory"]
    if not isinstance(directory, Mapping):
        raise CaptureError("DIRECTORY_CONTRACT_INVALID", "directory object")
    expected_paths = _capture_directory_paths(include_prefetch=include_prefetch)
    if directory.get("status") == "FAIL_CLOSED":
        if receipt_status != "PARTIAL_FAILURE_REVIEW_REQUIRED":
            raise CaptureError("DIRECTORY_CONTRACT_INVALID", "failed contract on success")
        if not isinstance(directory.get("failure"), str) or not directory["failure"]:
            raise CaptureError("DIRECTORY_CONTRACT_INVALID", "failed contract reason")
        return dict(value)
    if directory.get("status") not in (None, "PASS"):
        raise CaptureError("DIRECTORY_CONTRACT_INVALID", "directory status")
    try:
        observed_contract = _verify_capture_directory_layout(root, directory)
    except CaptureError:
        raise
    observed = observed_contract.get("after")
    expected_after = directory.get("after")
    if (not isinstance(expected_after, list) or
            tuple(directory.get("paths", ())) != tuple(sorted(expected_paths)) or
            observed != expected_after):
        raise CaptureError("DIRECTORY_CONTRACT_DRIFT", "pre/post descriptor")
    return dict(value)


def _inner_directory_projection(directory: Mapping[str, Any]) -> dict[str, Any]:
    """Project an in-container self-report for comparison with host evidence."""
    if not isinstance(directory, Mapping):
        raise CaptureError("DIRECTORY_CONTRACT_INVALID", "inner directory object")
    allowed = {"schema", "schema_version", "root", "mode", "owner", "paths", "root_before", "before"}
    if set(directory) - allowed or directory.get("schema") != EVIDENCE_DIRECTORIES.SCHEMA or \
            directory.get("schema_version") != EVIDENCE_DIRECTORIES.SCHEMA_VERSION:
        raise CaptureError("DIRECTORY_CONTRACT_INVALID", "inner directory fields")
    return {key: directory.get(key) for key in ("mode", "owner", "paths", "root_before", "before")}


def _validate_release_evidence(root: Path, value: Any, *, require_nonempty: bool = False,
                               label: str = "release evidence") -> list[dict[str, Any]]:
    """Reopen raw Release/signature bytes bound before declaration parsing."""
    if not isinstance(value, list) or (require_nonempty and not value):
        raise CaptureError("RELEASE_EVIDENCE_INVALID", label)
    result: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in value:
        fields = {"path", "source", "url", "repository_url", "kind", "identity",
                  "bytes", "sha256", "output", "signature"}
        if not isinstance(item, Mapping) or set(item) != fields:
            raise CaptureError("RELEASE_EVIDENCE_INVALID", label)
        path = _safe_rel(item["path"], label + " path")
        if not path.startswith("indices/") or path in seen:
            raise CaptureError("RELEASE_EVIDENCE_INVALID", path)
        seen.add(path)
        _safe_abs(item["source"], label + " source")
        if item["kind"] not in {"inrelease", "detached"}:
            raise CaptureError("RELEASE_EVIDENCE_INVALID", path)
        _capture_url(item["url"], label + " URL")
        _capture_url(item["repository_url"], label + " repository URL")
        if type(item["bytes"]) is not int or not 0 < item["bytes"] <= MAX_CAPTURE_FILE_BYTES:
            raise CaptureError("RELEASE_EVIDENCE_INVALID", path)
        _sha(item["sha256"], label + " SHA")
        _sha(item["identity"], label + " identity")
        if item["identity"] != _release_evidence_identity(item):
            raise CaptureError("RELEASE_EVIDENCE_IDENTITY_INVALID", path)
        output = item["output"]
        if not isinstance(output, Mapping):
            raise CaptureError("RELEASE_EVIDENCE_INVALID", path)
        output_path = _safe_rel(output.get("path"), label + " output path")
        if output_path != APT_SOURCE_RELATIVE + "/" + path:
            raise CaptureError("RELEASE_EVIDENCE_INVALID", path)
        expected_release_name = Path(item["source"]).name + "-" + _identity_token(
            item["identity"], label + " identity")
        if Path(output_path).name != expected_release_name:
            raise CaptureError("RELEASE_EVIDENCE_IDENTITY_INVALID", path)
        actual = _relative_descriptor(
            root, _descriptor(root / output_path, label + " output", MAX_CAPTURE_FILE_BYTES))
        if actual != dict(output) or actual["bytes"] != item["bytes"] or \
                actual["sha256"] != item["sha256"]:
            raise CaptureError("RELEASE_EVIDENCE_DRIFT", path)
        signature = item["signature"]
        if item["kind"] == "inrelease":
            if signature is not None:
                raise CaptureError("RELEASE_EVIDENCE_INVALID", path)
        else:
            if not isinstance(signature, Mapping) or set(signature) != {
                    "path", "source", "bytes", "sha256", "output"}:
                raise CaptureError("RELEASE_EVIDENCE_INVALID", path)
            _safe_abs(signature["source"], label + " signature source")
            _safe_abs(signature["path"], label + " signature path")
            if type(signature["bytes"]) is not int or not 0 < signature["bytes"] <= MAX_CAPTURE_FILE_BYTES:
                raise CaptureError("RELEASE_EVIDENCE_INVALID", path)
            _sha(signature["sha256"], label + " signature SHA")
            signature_output = signature["output"]
            if not isinstance(signature_output, Mapping):
                raise CaptureError("RELEASE_EVIDENCE_INVALID", path)
            signature_output_path = _safe_rel(
                signature_output.get("path"), label + " signature output path")
            if not signature_output_path.startswith("signatures/"):
                raise CaptureError("RELEASE_EVIDENCE_INVALID", path)
            observed_signature = _relative_descriptor(
                root, _descriptor(root / signature_output_path,
                                  label + " signature output", MAX_CAPTURE_FILE_BYTES))
            if observed_signature != dict(signature_output) or \
                    observed_signature["bytes"] != signature["bytes"] or \
                    observed_signature["sha256"] != signature["sha256"]:
                raise CaptureError("RELEASE_EVIDENCE_DRIFT", path)
            expected_signature_name = Path(signature["path"]).name + "-" + \
                _identity_token(signature["sha256"], label + " signature SHA")
            if Path(signature_output_path).name != expected_signature_name:
                raise CaptureError("RELEASE_EVIDENCE_IDENTITY_INVALID", path)
        result.append(dict(item))
    if [item["path"] for item in result] != sorted(item["path"] for item in result):
        raise CaptureError("RELEASE_EVIDENCE_UNSORTED", label)
    return result


def _validate_gpg_inventory_parent(root: Path, label: str) -> None:
    """Reopen the host-owned scratch parent and require it to be empty.

    Child ``GNUPGHOME`` records are logical container paths after cleanup and
    therefore cannot safely carry a container ``st_dev``/``st_ino`` claim.
    The host directory contract is the authority for those values.  This
    check makes the relationship explicit for callers that validate a keyring
    descriptor directly, as well as for the complete receipt validator.
    """
    try:
        EVIDENCE_DIRECTORIES.snapshot_existing(root, (GPG_INVENTORY_WORK_RELATIVE,))
    except EVIDENCE_DIRECTORIES.EvidenceDirectoryError as error:
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID", label) from error
    parent = root / GPG_INVENTORY_WORK_RELATIVE
    try:
        entries = os.listdir(parent)
    except OSError as error:
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID", label) from error
    if entries:
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_NOT_EMPTY", label)


def _validate_package_evidence(root: Path, value: Any, *, require_nonempty: bool = False,
                               require_decoded: bool = False,
                               label: str = "Packages evidence") -> list[dict[str, Any]]:
    """Reopen compressed and decoded Packages bytes retained by the capture.

    A failed capture may have only the compressed bytes (which are still
    useful forensic evidence).  A successful capture must additionally bind
    the decoded ``Packages`` artifact and both signed Release entries.
    """
    if not isinstance(value, list) or (require_nonempty and not value):
        raise CaptureError("PACKAGES_EVIDENCE_INVALID", label)
    result: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in value:
        fields = {"source", "url", "final_url", "format", "release_path",
                  "release_sha256", "release_bytes", "bytes", "sha256", "output",
                  "release_record_path", "release_file_sha256", "release_file_bytes",
                  "source_record_path", "decoded_release_path",
                  "decoded_release_sha256", "decoded_release_bytes", "decoded_output",
                  "stream_descriptors", "signed_empty", "artifact_identity"}
        if not isinstance(item, Mapping) or set(item) != fields:
            raise CaptureError("PACKAGES_EVIDENCE_INVALID", label)
        if item["source"] not in {"container-cache", "network"}:
            raise CaptureError("PACKAGES_EVIDENCE_INVALID", label)
        _capture_url(item["url"], label + " URL")
        _capture_url(item["final_url"], label + " final URL")
        requested = urlsplit(item["url"])
        final = urlsplit(item["final_url"])
        if (requested.scheme, requested.hostname, requested.port) != (
                final.scheme, final.hostname, final.port):
            raise CaptureError("PACKAGES_EVIDENCE_INVALID", label)
        if item["format"] not in {"xz", "gz", "plain"}:
            raise CaptureError("PACKAGES_EVIDENCE_INVALID", label)
        artifact = _validate_package_artifact_identity(
            item["artifact_identity"], label + " artifact identity")
        if (type(item["signed_empty"]) is not bool or
                item["signed_empty"] != artifact["signed_empty"]):
            raise CaptureError("PACKAGES_EVIDENCE_INVALID", label)
        streams = item["stream_descriptors"]
        if not isinstance(streams, list) or len(streams) > 2:
            raise CaptureError("PACKAGES_STREAM_EVIDENCE_INVALID", label)
        for index, stream in enumerate(streams):
            if not isinstance(stream, Mapping) or set(stream) != {
                    "index", "compressed_offset", "compressed_bytes",
                    "compressed_sha256", "decoded_bytes"}:
                raise CaptureError("PACKAGES_STREAM_EVIDENCE_INVALID", label)
            if (stream["index"] != index or type(stream["compressed_offset"]) is not int or
                    type(stream["compressed_bytes"]) is not int or
                    type(stream["decoded_bytes"]) is not int or
                    stream["compressed_offset"] < 0 or
                    stream["compressed_bytes"] <= 0 or stream["decoded_bytes"] < 0):
                raise CaptureError("PACKAGES_STREAM_EVIDENCE_INVALID", label)
            _sha(stream["compressed_sha256"], label + " stream SHA")
        if item["format"] != "xz" and streams:
            raise CaptureError("PACKAGES_STREAM_EVIDENCE_INVALID", label)
        release_path = _safe_rel(item["release_path"], label + " Release path")
        expected_suffix = "" if item["format"] == "plain" else "." + item["format"]
        if not release_path.endswith("/binary-amd64/Packages" + expected_suffix):
            raise CaptureError("PACKAGES_EVIDENCE_INVALID", release_path)
        if (artifact["format"] != item["format"] or
                artifact["signed_compressed_path"] != release_path or
                artifact["packages_url"] != item["url"] or
                artifact["compressed_sha256"] != item["release_sha256"] or
                artifact["compressed_bytes"] != item["release_bytes"]):
            raise CaptureError("PACKAGES_ARTIFACT_BINDING_INVALID", release_path)
        _sha(item["release_sha256"], label + " Release SHA")
        for key in ("release_bytes", "bytes"):
            if type(item[key]) is not int or not 0 < item[key] <= MAX_CAPTURE_FILE_BYTES:
                raise CaptureError("PACKAGES_EVIDENCE_INVALID", label)
        _sha(item["sha256"], label + " SHA")
        output = item["output"]
        if not isinstance(output, Mapping) or set(output) != {
                "path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
                "device", "inode"}:
            raise CaptureError("PACKAGES_EVIDENCE_INVALID", label)
        output_path = _safe_rel(output["path"], label + " output path")
        if not output_path.startswith("raw-indexes/") or output_path in seen:
            raise CaptureError("PACKAGES_EVIDENCE_INVALID", output_path)
        expected_raw_name = _package_artifact_name(
            Path(artifact["signed_compressed_path"]).name, artifact["identity"])
        if Path(output_path).name != expected_raw_name:
            raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_INVALID", output_path)
        seen.add(output_path)
        actual = _relative_descriptor(
            root, _descriptor(root / output_path, label + " output",
                              MAX_CAPTURE_FILE_BYTES))
        if actual != dict(output) or actual["bytes"] != item["bytes"] or \
                actual["sha256"] != item["sha256"]:
            raise CaptureError("PACKAGES_EVIDENCE_DRIFT", output_path)
        release_values = (item["release_record_path"], item["release_file_sha256"],
                          item["release_file_bytes"])
        decoded_values = (
            item["source_record_path"], item["decoded_release_path"],
            item["decoded_release_sha256"], item["decoded_release_bytes"],
            item["decoded_output"])
        if any(value is None for value in release_values + decoded_values):
            if require_decoded:
                raise CaptureError("PACKAGES_DECODED_EVIDENCE_MISSING", output_path)
            if any(value is not None for value in release_values + decoded_values):
                raise CaptureError("PACKAGES_DECODED_EVIDENCE_INVALID", output_path)
        else:
            release_record_path, release_file_sha256, release_file_bytes = release_values
            _safe_rel(release_record_path, label + " Release record path")
            _sha(release_file_sha256, label + " Release file SHA")
            if (type(release_file_bytes) is not int or
                    not 0 < release_file_bytes <= MAX_CAPTURE_FILE_BYTES):
                raise CaptureError("PACKAGES_EVIDENCE_INVALID", output_path)
            source_record_path, decoded_release_path, decoded_release_sha256, \
                decoded_release_bytes, decoded_output = decoded_values
            _safe_rel(source_record_path, label + " source record path")
            _safe_rel(decoded_release_path, label + " decoded Release path")
            _sha(decoded_release_sha256, label + " decoded Release SHA")
            if (type(decoded_release_bytes) is not int or
                    decoded_release_bytes < 0 or
                    decoded_release_bytes > MAX_DECODED_PACKAGES_BYTES):
                raise CaptureError("PACKAGES_EVIDENCE_INVALID", output_path)
            if item["signed_empty"]:
                _signed_empty_packages_policy()
                if (decoded_release_bytes != 0 or
                        decoded_release_sha256 != EMPTY_PACKAGES_SHA256 or
                        artifact["format"] != "xz"):
                    raise CaptureError("PACKAGES_EVIDENCE_INVALID", output_path)
            elif decoded_release_bytes == 0:
                raise CaptureError("PACKAGES_EVIDENCE_INVALID", output_path)
            if not isinstance(decoded_output, Mapping) or set(decoded_output) != {
                    "path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
                    "device", "inode"}:
                raise CaptureError("PACKAGES_EVIDENCE_INVALID", output_path)
            decoded_path = _safe_rel(decoded_output["path"],
                                     label + " decoded output path")
            if not decoded_path.startswith(APT_SOURCE_RELATIVE + "/"):
                raise CaptureError("PACKAGES_EVIDENCE_INVALID", decoded_path)
            decoded_actual = _relative_descriptor(
                root, _descriptor(root / decoded_path, label + " decoded output",
                                  MAX_DECODED_PACKAGES_BYTES,
                                  allow_empty=item["signed_empty"]))
            if decoded_actual != dict(decoded_output) or \
                    decoded_actual["bytes"] != decoded_release_bytes or \
                    decoded_actual["sha256"] != decoded_release_sha256:
                raise CaptureError("PACKAGES_DECODED_EVIDENCE_DRIFT", decoded_path)
            if item["format"] == "xz":
                raw_data, _ = _read_bounded(
                    root / output_path, label + " compressed bytes",
                    MAX_CAPTURE_FILE_BYTES, allow_empty=False)
                try:
                    _, observed_streams = _decode_xz_stream_details(
                        raw_data, label + " compressed bytes",
                        allow_signed_empty=item["signed_empty"])
                except CaptureError as error:
                    raise CaptureError("PACKAGES_STREAM_EVIDENCE_INVALID", output_path) from error
                if streams != observed_streams:
                    raise CaptureError("PACKAGES_STREAM_EVIDENCE_DRIFT", output_path)
                if (len(streams) < 1 or
                        (not item["signed_empty"] and
                         streams[0]["decoded_bytes"] <= 0)):
                    raise CaptureError("PACKAGES_STREAM_EVIDENCE_INVALID", output_path)
                if item["signed_empty"] and (len(streams) != 1 or
                                               streams[0]["compressed_bytes"] != 32 or
                                               streams[0]["decoded_bytes"] != 0):
                    raise CaptureError("PACKAGES_STREAM_EVIDENCE_INVALID", output_path)
                if len(streams) == 2:
                    terminal = streams[1]
                    terminal_policy = _xz_terminal_empty_stream_policy()
                    if (terminal["compressed_bytes"] != terminal_policy["compressed_bytes"] or
                            terminal["decoded_bytes"] != terminal_policy["decoded_bytes"]):
                        raise CaptureError("PACKAGES_STREAM_EVIDENCE_INVALID", output_path)
            elif streams:
                raise CaptureError("PACKAGES_STREAM_EVIDENCE_INVALID", output_path)
            if (artifact["compressed_sha256"] != item["sha256"] or
                    artifact["compressed_bytes"] != item["bytes"]):
                raise CaptureError("PACKAGES_ARTIFACT_BINDING_INVALID", output_path)
        if not all(value is None for value in release_values + decoded_values):
            if (artifact["release_record_path"] != item["release_record_path"] or
                    artifact["release_file_sha256"] != item["release_file_sha256"] or
                    artifact["release_file_bytes"] != item["release_file_bytes"] or
                    artifact["signed_plain_path"] != item["decoded_release_path"] or
                    artifact["decoded_sha256"] != item["decoded_release_sha256"] or
                    artifact["decoded_bytes"] != item["decoded_release_bytes"]):
                raise CaptureError("PACKAGES_ARTIFACT_BINDING_INVALID", output_path)
            expected_decoded_name = _package_artifact_name(
                Path(artifact["signed_plain_path"]).name, artifact["identity"])
            if item["decoded_output"]["path"] != (
                    APT_SOURCE_RELATIVE + "/indices/" + expected_decoded_name):
                raise CaptureError("PACKAGES_ARTIFACT_BINDING_INVALID", output_path)
        result.append(dict(item))
    if [item["output"]["path"] for item in result] != sorted(seen):
        raise CaptureError("PACKAGES_EVIDENCE_UNSORTED", label)
    return result


def _validate_keyring_descriptor(root: Path, value: Any, label: str, *,
                                 allow_pending: bool = False,
                                 partial_failure_kind: str | None = None
                                 ) -> dict[str, Any]:
    """Reopen one captured keyring and all of its material projections."""
    base_required = {
        "path", "kind", "source", "bytes", "sha256",
        "normalized_armor_bytes", "normalized_armor_sha256",
        "dearmored_path", "dearmored_bytes", "dearmored_sha256",
        "primary_fingerprints", "subkey_fingerprints",
        "expected_signer_fingerprints", "inventory",
    }
    if not isinstance(value, Mapping):
        raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    # A failed signature attempt has no signer fingerprint to bind yet, so a
    # pending diagnostic may omit it.  Sealed manifests and successful
    # evidence must always carry both the observed signer and (for inline
    # armor) the inline-key fingerprint.  Keep this distinction explicit so
    # an old five-field descriptor cannot pass.
    inventory_value = value.get("inventory")
    inventory_status = (inventory_value.get("status")
                        if isinstance(inventory_value, Mapping) else None)
    allow_inventory_failure = partial_failure_kind in GPG_KEY_INVENTORY_FAILURE_KINDS
    pending_without_fingerprint = (
        (allow_pending or allow_inventory_failure) and
        inventory_status in {"PENDING", "SYNTHETIC_TEST_ONLY", "FAILED"})
    if value.get("kind") == "inline":
        required = base_required | {"fingerprint", "signer_fingerprint"}
        allowed = [required]
        if pending_without_fingerprint:
            allowed.extend((base_required, base_required | {"fingerprint"}))
    elif value.get("kind") == "path":
        required = base_required | {"signer_fingerprint"}
        allowed = [required]
        if pending_without_fingerprint:
            allowed.append(base_required)
    else:
        required = base_required
        allowed = [required]
    if set(value) not in allowed:
        raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    path = _safe_rel(value["path"], label + " path")
    if not path.startswith(APT_KEYRING_RELATIVE + "/"):
        raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    if value["kind"] not in {"path", "inline"}:
        raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    if value["kind"] == "path":
        if (not isinstance(value["source"], str) or
                not value["source"].startswith("/") or
                Path(value["source"]).as_posix() != value["source"] or
                any(part in ("", ".", "..")
                    for part in value["source"].split("/")[1:])):
            raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
        if value["normalized_armor_bytes"] is not None or \
                value["normalized_armor_sha256"] is not None:
            raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    else:
        if value["source"] != "inline" or \
                type(value["normalized_armor_bytes"]) is not int or \
                not isinstance(value["normalized_armor_sha256"], str):
            raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
        if "fingerprint" in value and (
                not isinstance(value["fingerprint"], str) or
                not FINGERPRINT_RE.fullmatch(value["fingerprint"])):
            raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
        _sha(value["normalized_armor_sha256"], label + " armor SHA")
        if value["normalized_armor_bytes"] != value["bytes"] or \
                value["normalized_armor_sha256"] != value["sha256"]:
            raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    if (type(value["bytes"]) is not int or not 0 < value["bytes"] <= MAX_CAPTURE_FILE_BYTES or
            type(value["dearmored_bytes"]) is not int or
            not 0 < value["dearmored_bytes"] <= MAX_CAPTURE_FILE_BYTES):
        raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    _sha(value["sha256"], label + " SHA")
    _sha(value["dearmored_sha256"], label + " dearmored SHA")
    dearmored_path = _safe_rel(value["dearmored_path"], label + " dearmored path")
    if not dearmored_path.startswith(APT_KEYRING_RELATIVE + "/"):
        raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    if value["kind"] == "inline" and (
            not path.endswith(".asc") or not dearmored_path.endswith(".gpg") or
            path == dearmored_path):
        raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    if value["kind"] == "path" and (
            dearmored_path != path or value["dearmored_bytes"] != value["bytes"] or
            value["dearmored_sha256"] != value["sha256"]):
        raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    for field in ("primary_fingerprints", "subkey_fingerprints",
                  "expected_signer_fingerprints"):
        values = value[field]
        if (not isinstance(values, list) or values != sorted(set(values)) or
                any(not isinstance(item, str) or not FINGERPRINT_RE.fullmatch(item)
                    for item in values)):
            raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    if not set(value["expected_signer_fingerprints"]).issubset(
            set(value["primary_fingerprints"] + value["subkey_fingerprints"])):
        raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    if "signer_fingerprint" in value:
        if (not isinstance(value["signer_fingerprint"], str) or
                not FINGERPRINT_RE.fullmatch(value["signer_fingerprint"]) or
                value["signer_fingerprint"] not in
                value["expected_signer_fingerprints"]):
            raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    if (value["kind"] == "inline" and "fingerprint" in value and
            "signer_fingerprint" in value and
            value["fingerprint"] != value["signer_fingerprint"]):
        raise CaptureError("KEYRING_DESCRIPTOR_INVALID", label)
    actual = _relative_descriptor(
        root, _descriptor(root / path, label + " armor", MAX_CAPTURE_FILE_BYTES))
    if actual["bytes"] != value["bytes"] or actual["sha256"] != value["sha256"]:
        raise CaptureError("KEYRING_DESCRIPTOR_DRIFT", label)
    actual_dearmored = _relative_descriptor(
        root, _descriptor(root / dearmored_path, label + " dearmored",
                          MAX_CAPTURE_FILE_BYTES))
    if actual_dearmored["bytes"] != value["dearmored_bytes"] or \
            actual_dearmored["sha256"] != value["dearmored_sha256"]:
        raise CaptureError("KEYRING_DESCRIPTOR_DRIFT", label)
    inventory = value["inventory"]
    if not isinstance(inventory, Mapping):
        raise CaptureError("KEYRING_INVENTORY_INVALID", label)
    if inventory.get("status") == "PENDING" and allow_pending:
        if set(inventory) != {"status"}:
            raise CaptureError("KEYRING_INVENTORY_INVALID", label)
    elif inventory.get("status") == "SYNTHETIC_TEST_ONLY":
        if set(inventory) != {"status"}:
            raise CaptureError("KEYRING_INVENTORY_INVALID", label)
    elif inventory.get("status") in {"VERIFIED", "FAILED"}:
        inventory_failed = inventory.get("status") == "FAILED"
        verified_fields = {
            "status", "identity", "command", "command_sha256", "exit_status", "timed_out",
            "signal", "stdout", "stderr", "tool", "homedir",
            "metadata_residue",
        }
        expected_fields = verified_fields | ({"failure_kind", "version_stdout",
                                              "version_stderr"}
                                             if inventory_failed else set())
        if (set(inventory) != expected_fields or
                (inventory_failed and not allow_inventory_failure)):
            raise CaptureError("KEYRING_INVENTORY_INVALID", label)
        argv = _validate_argv(inventory["command"], label + " inventory command")
        inventory_identity = inventory.get("identity")
        _sha(inventory_identity, label + " inventory identity")
        expected_token = _identity_token(inventory_identity, label + " inventory identity")
        homedir_value = inventory["homedir"]
        if not isinstance(homedir_value, Mapping):
            raise CaptureError("KEYRING_INVENTORY_INVALID", label)
        _validate_gpg_inventory_parent(root, label + " inventory parent")
        _validate_inventory_homedir(
            homedir_value, label + " inventory homedir", inventory_identity)
        metadata = _validate_gpg_metadata_residue(
            inventory["metadata_residue"], homedir_value,
            label + " inventory metadata", require_exact=not inventory_failed)
        if metadata is not None and metadata["creation_evidence"] != inventory["stderr"]:
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
        homedir = Path(homedir_value["path"])
        _validate_gpg_inventory_argv(
            argv, homedir, dearmored_path, label + " inventory command")
        if argv[9] != str(homedir):
            raise CaptureError("GPG_KEY_INVENTORY_COMMAND_INVALID", label)
        for stream, expected_name in (
                ("stdout", "gpg-key-inventory-{}.stdout".format(expected_token)),
                ("stderr", "gpg-key-inventory-{}.stderr".format(expected_token))):
            stream_value = inventory[stream]
            if (stream_value is not None and
                    (not isinstance(stream_value, Mapping) or
                     Path(stream_value.get("path", "")).name != expected_name)):
                raise CaptureError("GPG_KEY_INVENTORY_COMMAND_INVALID", label)
        if inventory["command_sha256"] != _command_hash(argv):
            raise CaptureError("KEYRING_INVENTORY_INVALID", label)
        if (type(inventory["exit_status"]) is not int or
                type(inventory["timed_out"]) is not bool or
                (inventory["signal"] is not None and
                 type(inventory["signal"]) is not int)):
            raise CaptureError("KEYRING_INVENTORY_INVALID", label)
        if not inventory_failed and (inventory["exit_status"] != 0 or
                                      inventory["timed_out"] or
                                      inventory["signal"] is not None):
            raise CaptureError("KEYRING_INVENTORY_INVALID", label)
        if inventory_failed:
            failure = inventory.get("failure_kind")
            if failure != partial_failure_kind or failure not in GPG_KEY_INVENTORY_FAILURE_KINDS:
                raise CaptureError("KEYRING_INVENTORY_INVALID", label)
        tool = inventory["tool"]
        if inventory_failed and tool is None:
            pass
        elif not isinstance(tool, Mapping):
            raise CaptureError("KEYRING_INVENTORY_TOOL_INVALID", label)
        else:
            _validate_gpg_inventory_tool(
                root, tool, homedir, label + " inventory tool")
            for stream, expected_name in (
                    ("version_stdout", "gpg-key-inventory-{}-version.stdout".format(
                        expected_token)),
                    ("version_stderr", "gpg-key-inventory-{}-version.stderr".format(
                        expected_token))):
                descriptor = tool[stream]
                if Path(descriptor["path"]).name != expected_name:
                    raise CaptureError("GPG_KEY_INVENTORY_TOOL_INVALID", label)
        for stream in ("stdout", "stderr"):
            stream_value = inventory[stream]
            if stream_value is None and inventory_failed:
                continue
            if not isinstance(stream_value, Mapping):
                raise CaptureError("KEYRING_INVENTORY_INVALID", label)
            stream_path = _safe_rel(stream_value.get("path"), label + " inventory log")
            observed = _relative_descriptor(
                root, _descriptor(root / stream_path, label + " inventory log",
                                  MAX_LOG_BYTES))
            if observed != dict(stream_value):
                raise CaptureError("KEYRING_INVENTORY_INVALID", label)
        if inventory_failed:
            for stream in ("version_stdout", "version_stderr"):
                stream_value = inventory[stream]
                if stream_value is None:
                    continue
                if not isinstance(stream_value, Mapping):
                    raise CaptureError("KEYRING_INVENTORY_INVALID", label)
                stream_path = _safe_rel(
                    stream_value.get("path"), label + " version log")
                observed = _relative_descriptor(
                    root, _descriptor(root / stream_path, label + " version log",
                                      MAX_LOG_BYTES))
                if observed != dict(stream_value):
                    raise CaptureError("KEYRING_INVENTORY_INVALID", label)
    else:
        raise CaptureError("KEYRING_INVENTORY_INVALID", label)
    return dict(value)


def _validate_gpgv_evidence(root: Path, value: Any, *, require_nonempty: bool = False,
                            label: str = "gpgv evidence",
                            partial_failure_kind: str | None = None
                            ) -> list[dict[str, Any]]:
    """Validate command, diagnostics, and key selection for gpgv evidence."""
    if not isinstance(value, list) or (require_nonempty and not value):
        raise CaptureError("GPGV_EVIDENCE_INVALID", label)
    result: list[dict[str, Any]] = []
    seen: set[str] = set()
    for item in value:
        required = {"release_kind", "release_path", "signature_path",
                    "signature_descriptor", "keyrings", "command", "command_sha256",
                    "exit_status", "stdout_sha256", "stderr_sha256", "status_fd",
                    "timed_out", "signal", "diagnostics", "status"}
        optional = {"repository", "tool", "identity"}
        if not isinstance(item, Mapping) or not required.issubset(item) or \
                set(item) - required - optional:
            raise CaptureError("GPGV_EVIDENCE_INVALID", label)
        release_path = str(item["release_path"])
        if release_path in seen or item["release_kind"] not in {"inrelease", "detached"}:
            raise CaptureError("GPGV_EVIDENCE_INVALID", label)
        seen.add(release_path)
        if "identity" in item:
            _sha(item["identity"], label + " identity")
        _safe_abs(release_path, label + " release path")
        if item["signature_path"] is not None:
            _safe_abs(item["signature_path"], label + " signature path")
        if item["release_kind"] == "inrelease" and item["signature_path"] is not None:
            raise CaptureError("GPGV_EVIDENCE_INVALID", label)
        argv = _validate_argv(item["command"], label + " command")
        if (argv[0] != "gpgv" or "--no-options" in argv or
                item["command_sha256"] != _command_hash(argv)):
            raise CaptureError("GPGV_EVIDENCE_COMMAND_INVALID", label)
        if argv[1:3] != ["--status-fd", "1"]:
            raise CaptureError("GPGV_EVIDENCE_COMMAND_INVALID", label)
        if (type(item["exit_status"]) is not int or
                type(item["status_fd"]) is not int or item["status_fd"] != 1 or
                type(item["timed_out"]) is not bool or
                (item["signal"] is not None and type(item["signal"]) is not int)):
            raise CaptureError("GPGV_EVIDENCE_RESULT_INVALID", label)
        _sha(item["stdout_sha256"], label + " stdout SHA")
        _sha(item["stderr_sha256"], label + " stderr SHA")
        diagnostics = item["diagnostics"]
        if not isinstance(diagnostics, Mapping) or set(diagnostics) != {"stdout", "stderr"}:
            raise CaptureError("GPGV_EVIDENCE_DIAGNOSTICS_INVALID", label)
        if "identity" in item:
            token = _identity_token(item["identity"], label + " identity")
            expected_logs = {
                "stdout": "gpgv-{}.stdout".format(token),
                "stderr": "gpgv-{}.stderr".format(token),
            }
            for stream, expected_name in expected_logs.items():
                stream_value = diagnostics[stream]
                if (not isinstance(stream_value, Mapping) or
                        Path(stream_value.get("path", "")).name != expected_name):
                    raise CaptureError("GPGV_EVIDENCE_DIAGNOSTICS_INVALID", label)
        for stream in ("stdout", "stderr"):
            stream_value = diagnostics[stream]
            stream_path = _safe_rel(stream_value.get("path"), label + " log")
            observed = _relative_descriptor(
                root, _descriptor(root / stream_path, label + " log", MAX_LOG_BYTES))
            if observed != dict(stream_value):
                raise CaptureError("GPGV_EVIDENCE_DIAGNOSTICS_INVALID", label)
            expected_sha = item[stream + "_sha256"]
            if observed["sha256"] != expected_sha:
                raise CaptureError("GPGV_EVIDENCE_DIAGNOSTICS_INVALID", label)
        keyrings = item["keyrings"]
        if not isinstance(keyrings, list) or not keyrings:
            raise CaptureError("GPGV_EVIDENCE_KEYRING_INVALID", label)
        for keyring_index, keyring in enumerate(keyrings):
            _validate_keyring_descriptor(
                root, keyring, "{} keyring {}".format(label, keyring_index),
                allow_pending=item["status"] is None or
                partial_failure_kind in GPG_KEY_INVENTORY_FAILURE_KINDS,
                partial_failure_kind=partial_failure_kind)
        command_keyring_paths = []
        command_index = 3
        while command_index < len(argv) and argv[command_index] == "--keyring":
            if command_index + 1 >= len(argv):
                raise CaptureError("GPGV_EVIDENCE_COMMAND_INVALID", label)
            command_keyring_paths.append(argv[command_index + 1])
            command_index += 2
        expected_keyring_paths = [
            str(root / keyring["dearmored_path"]) for keyring in keyrings
        ]
        if command_keyring_paths != expected_keyring_paths or any(
                keyring["kind"] == "inline" and (
                    path.endswith((".asc", ".armor")) or
                    not path.endswith(".gpg"))
                for path, keyring in zip(command_keyring_paths, keyrings)):
            raise CaptureError("GPGV_EVIDENCE_KEYRING_INVALID", label)
        status = item["status"]
        if status is None:
            # A failed inner capture may have sealed Release/index bytes before
            # gpgv interpretation completed.  Preserve that evidence, but only
            # when the outer failure explains exactly why no signer projection
            # exists.  A null status on a nominally successful command is not a
            # valid receipt by itself.
            if partial_failure_kind not in GPGV_PARTIAL_FAILURE_KINDS:
                raise CaptureError("GPGV_EVIDENCE_STATUS_INVALID", label)
            command_failed = item["exit_status"] != 0 or item["timed_out"] or \
                item["signal"] is not None
            if partial_failure_kind == "GPGV_VERIFY_FAILED":
                if not command_failed:
                    raise CaptureError("GPGV_EVIDENCE_STATUS_INVALID", label)
            elif command_failed:
                raise CaptureError("GPGV_EVIDENCE_STATUS_INVALID", label)
        else:
            if not isinstance(status, Mapping) or set(status) != {
                    "goodsig", "validsig", "status_tags", "status_sequence",
                    "key_considered", "verification_compliance_mode", "stderr_sha256"} or \
                    status["stderr_sha256"] != item["stderr_sha256"]:
                raise CaptureError("GPGV_EVIDENCE_STATUS_INVALID", label)
            # Reparse the sealed status-fd stream.  A receipt-side canonical
            # rehash must not be able to turn altered status metadata into a
            # valid signer proof.
            stdout_path = _safe_rel(item["diagnostics"]["stdout"].get("path"),
                                    label + " stdout log")
            stderr_path = _safe_rel(item["diagnostics"]["stderr"].get("path"),
                                    label + " stderr log")
            stdout_data, _ = _read_bounded(
                root / stdout_path, label + " stdout", MAX_LOG_BYTES, allow_empty=True)
            stderr_data, _ = _read_bounded(
                root / stderr_path, label + " stderr", MAX_LOG_BYTES, allow_empty=True)
            try:
                reparsed = _parse_gpgv_status(stdout_data, stderr_data, label)
            except CaptureError as error:
                raise CaptureError("GPGV_EVIDENCE_STATUS_INVALID", label) from error
            if reparsed != dict(status):
                raise CaptureError("GPGV_EVIDENCE_STATUS_INVALID", label)
            if partial_failure_kind not in GPG_KEY_INVENTORY_FAILURE_KINDS:
                signer = status["validsig"].get("fingerprint") \
                    if isinstance(status["validsig"], Mapping) else None
                if (not isinstance(signer, str) or not FINGERPRINT_RE.fullmatch(signer) or
                        not any(signer in keyring["expected_signer_fingerprints"] and
                                (keyring["kind"] != "inline" or
                                 keyring.get("fingerprint") == signer)
                                for keyring in keyrings)):
                    raise CaptureError("GPGV_EVIDENCE_SIGNER_SCOPE_INVALID", label)
        result.append(dict(item))
    if [item["release_path"] for item in result] != sorted(seen):
        raise CaptureError("GPGV_EVIDENCE_UNSORTED", label)
    return result


def _make_signature_reference(repository_index: int,
                              repository: Mapping[str, Any],
                              release: Path, release_kind: str,
                              release_record: Mapping[str, Any],
                              release_data: bytes, keyring_identity: str,
                              signer_fingerprint: str,
                              signer_policy: Mapping[str, Any],
                              packages_url: str, packages_path: str,
                              packages_sha256: str) -> dict[str, Any]:
    """Build one explicit repository-to-signature reference."""
    release_url, _ = _repository_urls(repository, release_kind, "plain")
    fields = {
        "repository_index": repository_index,
        "identity": "", "keyring_identity": keyring_identity,
        "release_kind": release_kind, "repository_url": repository["url"],
        "release_url": release_url, "release_path": str(release),
        "release_record_path": release_record["path"],
        "release_sha256": sha256_bytes(release_data),
        "signer_fingerprint": signer_fingerprint,
        "signer_policy": dict(signer_policy), "packages_url": packages_url,
        "packages_path": packages_path, "packages_sha256": packages_sha256,
    }
    fields["identity"] = _signature_identity_projection(fields)
    return fields


def _sorted_signature_evidence(value: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Return detached copies in the canonical Release-path order."""
    return sorted((_immutable_copy(item) for item in value),
                  key=lambda item: item.get("release_path", ""))


def _merge_signature_evidence(*values: list[Mapping[str, Any]]) -> list[dict[str, Any]]:
    """Union signature evidence without duplicating a shared Release."""
    merged: dict[str, dict[str, Any]] = {}
    for value in values:
        for item in value:
            key = str(item.get("identity") or item.get("release_path"))
            previous = merged.get(key)
            if previous is None:
                merged[key] = _immutable_copy(item)
            elif previous.get("release_path") != item.get("release_path"):
                raise CaptureError("GPG_RELEASE_IDENTITY_CONFLICT", key)
    return _sorted_signature_evidence(list(merged.values()))


def _cache_signature_success(cache: dict[str, Mapping[str, Any]],
                             identity: str, evidence: Mapping[str, Any]) -> None:
    """Retain the first successful gpgv result for one immutable identity."""
    _sha(identity, "gpgv cache identity")
    if not isinstance(evidence, Mapping) or evidence.get("identity") != identity:
        raise CaptureError("GPG_RELEASE_CACHE_INVALID", identity)
    entry = {"status": "PASS", "evidence": _immutable_copy(evidence)}
    previous = cache.get(identity)
    if previous is not None and canonical_bytes(previous) != canonical_bytes(entry):
        raise CaptureError("GPG_RELEASE_IDENTITY_CONFLICT", identity)
    cache[identity] = _immutable_copy(entry)


def _cache_signature_failure(cache: dict[str, Mapping[str, Any]],
                             identity: str, error: CaptureError) -> None:
    """Retain the first failed gpgv attempt, including bounded diagnostics."""
    _sha(identity, "gpgv cache identity")
    evidence = _immutable_copy(getattr(error, "signature_evidence", None) or [])
    if any(not isinstance(item, Mapping) or item.get("identity") != identity
           for item in evidence):
        raise CaptureError("GPG_RELEASE_CACHE_INVALID", identity)
    entry = {
        "status": "FAILED", "failure_kind": error.kind,
        "failure_message": str(error), "evidence": evidence,
    }
    previous = cache.get(identity)
    if previous is not None and canonical_bytes(previous) != canonical_bytes(entry):
        raise CaptureError("GPG_RELEASE_IDENTITY_CONFLICT", identity)
    cache[identity] = _immutable_copy(entry)


def _cached_signature(cache: Mapping[str, Mapping[str, Any]], identity: str,
                      label: str) -> dict[str, Any]:
    """Return a detached cache entry and reject mutation/conflicting identity."""
    entry = cache.get(identity)
    if not isinstance(entry, Mapping) or set(entry) not in (
            {"status", "evidence"},
            {"status", "failure_kind", "failure_message", "evidence"}):
        raise CaptureError("GPG_RELEASE_CACHE_INVALID", label)
    if not isinstance(entry.get("status"), str) or entry["status"] not in {
            "PASS", "FAILED"}:
        raise CaptureError("GPG_RELEASE_CACHE_INVALID", label)
    if entry["status"] == "PASS":
        if (not isinstance(entry.get("evidence"), Mapping) or
                entry["evidence"].get("identity") != identity):
            raise CaptureError("GPG_RELEASE_CACHE_INVALID", label)
    else:
        if not isinstance(entry.get("evidence"), list):
            raise CaptureError("GPG_RELEASE_CACHE_INVALID", label)
        for item in entry["evidence"]:
            if not isinstance(item, Mapping) or item.get("identity") != identity:
                raise CaptureError("GPG_RELEASE_CACHE_INVALID", label)
    if entry["status"] == "FAILED" and (
            not isinstance(entry.get("failure_kind"), str) or
            not isinstance(entry.get("failure_message"), str)):
        raise CaptureError("GPG_RELEASE_CACHE_INVALID", label)
    return _immutable_copy(entry)


def _validate_signature_references(value: Any, signature_count: Any,
                                   signatures: list[Mapping[str, Any]] | None = None,
                                   *, require_complete: bool = False,
                                   label: str = "signature references"
                                   ) -> list[dict[str, Any]]:
    """Validate the explicit many-repositories-to-one-signature projection."""
    fields = {
        "repository_index", "identity", "keyring_identity", "release_kind",
        "repository_url", "release_url", "release_path", "release_record_path",
        "release_sha256", "signer_fingerprint", "signer_policy", "packages_url",
        "packages_path", "packages_sha256",
    }
    if type(signature_count) is not int or signature_count < 0:
        raise CaptureError("SIGNATURE_REFERENCE_INVALID", label)
    if not isinstance(value, list) or len(value) > MAX_CAPTURE_FILES:
        raise CaptureError("SIGNATURE_REFERENCE_INVALID", label)
    if value != sorted(value, key=lambda item: item.get("repository_index", -1)
                      if isinstance(item, Mapping) else -1):
        raise CaptureError("SIGNATURE_REFERENCE_UNSORTED", label)
    result: list[dict[str, Any]] = []
    indices: set[int] = set()
    identities: set[str] = set()
    for item in value:
        if not isinstance(item, Mapping) or set(item) != fields:
            raise CaptureError("SIGNATURE_REFERENCE_INVALID", label)
        index = item["repository_index"]
        if type(index) is not int or index < 0 or index in indices:
            raise CaptureError("SIGNATURE_REFERENCE_INVALID", label)
        indices.add(index)
        for key in ("identity", "keyring_identity", "release_sha256", "packages_sha256"):
            _sha(item[key], label + " " + key)
        if item["release_kind"] not in {"inrelease", "detached"}:
            raise CaptureError("SIGNATURE_REFERENCE_INVALID", label)
        for key in ("repository_url", "release_url", "packages_url"):
            _capture_url(item[key], label + " " + key)
        _safe_abs(item["release_path"], label + " release path")
        for key in ("release_record_path", "packages_path"):
            _safe_rel(item[key], label + " " + key)
        if not FINGERPRINT_RE.fullmatch(item["signer_fingerprint"]):
            raise CaptureError("SIGNATURE_REFERENCE_INVALID", label)
        if not isinstance(item["signer_policy"], Mapping) or \
                set(item["signer_policy"]) != {"declared_fingerprint"} or \
                (item["signer_policy"]["declared_fingerprint"] is not None and
                 not FINGERPRINT_RE.fullmatch(item["signer_policy"]["declared_fingerprint"])):
            raise CaptureError("SIGNATURE_REFERENCE_INVALID", label)
        if _signature_identity_projection(item) != item["identity"]:
            raise CaptureError("SIGNATURE_REFERENCE_IDENTITY_INVALID", label)
        identities.add(item["identity"])
        result.append(dict(item))
    if require_complete and indices != set(range(len(result))):
        raise CaptureError("SIGNATURE_REFERENCE_SET_INVALID", label)
    if signatures is not None:
        signature_identities = {
            item.get("identity") for item in signatures
            if isinstance(item, Mapping) and item.get("identity") is not None
        }
        if any(identity not in signature_identities for identity in identities):
            raise CaptureError("SIGNATURE_REFERENCE_BINDING_INVALID", label)
        if require_complete and not signature_identities.issubset(identities):
            raise CaptureError("SIGNATURE_REFERENCE_BINDING_INVALID", label)
        if require_complete and len(signature_identities) != signature_count:
            raise CaptureError("SIGNATURE_REFERENCE_COUNT_INVALID", label)
    if require_complete and signature_count > len(identities):
        raise CaptureError("SIGNATURE_REFERENCE_COUNT_INVALID", label)
    return result


REFERENCE_MAP_FIELDS = ("release", "keyring", "signature", "packages")
REFERENCE_COUNT_FIELDS = (
    "repository_references", "release_evidence", "package_evidence",
    "signature_evidence", "unique_release_references",
    "unique_keyring_references", "unique_signature_references",
    "unique_package_references",
)


def _reference_projection(release_evidence: Any, package_evidence: Any,
                          signature_references: Any,
                          signature_count: Any) -> tuple[dict[str, Any], dict[str, int]]:
    """Build the immutable repository-to-artifact deduplication projection.

    A Release and its keyring/signature are often shared by several component
    records.  Lists alone do not make that relationship unambiguous, so every
    repository reference is mapped to the full identity of the deduplicated
    object.  The projection is also emitted on partial failure; an empty map
    is meaningful and does not assert that a missing artifact was verified.
    """
    if not isinstance(release_evidence, list) or not isinstance(package_evidence, list):
        raise CaptureError("REFERENCE_PROJECTION_INVALID", "evidence lists")
    if not isinstance(signature_references, list) or type(signature_count) is not int or \
            signature_count < 0:
        raise CaptureError("REFERENCE_PROJECTION_INVALID", "signature references")
    reference_map = {field: {} for field in REFERENCE_MAP_FIELDS}
    for reference in signature_references:
        if not isinstance(reference, Mapping) or type(reference.get("repository_index")) is not int or \
                reference["repository_index"] < 0:
            raise CaptureError("REFERENCE_PROJECTION_INVALID", "repository index")
        index = str(reference["repository_index"])
        if any(index in values for values in reference_map.values()):
            raise CaptureError("REFERENCE_PROJECTION_INVALID", "duplicate repository index")
        release_matches = [
            item for item in release_evidence
            if isinstance(item, Mapping) and
            item.get("repository_url") == reference.get("repository_url") and
            item.get("url") == reference.get("release_url") and
            item.get("sha256") == reference.get("release_sha256")
        ]
        package_matches = [
            item for item in package_evidence
            if isinstance(item, Mapping) and
            item.get("url") == reference.get("packages_url") and
            item.get("source_record_path") == reference.get("packages_path") and
            item.get("decoded_release_sha256") == reference.get("packages_sha256") and
            isinstance(item.get("artifact_identity"), Mapping)
        ]
        if len(release_matches) != 1 or len(package_matches) != 1:
            raise CaptureError("REFERENCE_PROJECTION_BINDING_INVALID", index)
        package_identity = package_matches[0]["artifact_identity"].get("identity")
        if not isinstance(package_identity, str):
            raise CaptureError("REFERENCE_PROJECTION_BINDING_INVALID", index)
        _sha(package_identity, "package artifact reference identity")
        reference_map["release"][index] = release_matches[0]["identity"]
        reference_map["keyring"][index] = reference["keyring_identity"]
        reference_map["signature"][index] = reference["identity"]
        reference_map["packages"][index] = package_identity
    counts = {
        "repository_references": len(signature_references),
        "release_evidence": len(release_evidence),
        "package_evidence": len(package_evidence),
        "signature_evidence": signature_count,
        "unique_release_references": len(set(reference_map["release"].values())),
        "unique_keyring_references": len(set(reference_map["keyring"].values())),
        "unique_signature_references": len(set(reference_map["signature"].values())),
        "unique_package_references": len(set(reference_map["packages"].values())),
    }
    return reference_map, counts


def _validate_reference_projection(value: Any, counts: Any,
                                   release_evidence: Any, package_evidence: Any,
                                   signature_references: Any, signature_count: Any,
                                   *, require_complete: bool = False,
                                   label: str = "reference projection"
                                   ) -> tuple[dict[str, Any], dict[str, int]]:
    """Validate a repository reference map against reopened evidence bytes."""
    if not isinstance(value, Mapping) or set(value) != set(REFERENCE_MAP_FIELDS):
        raise CaptureError("REFERENCE_PROJECTION_INVALID", label)
    expected_map, expected_counts = _reference_projection(
        release_evidence, package_evidence, signature_references, signature_count)
    for field in REFERENCE_MAP_FIELDS:
        item = value[field]
        if not isinstance(item, Mapping) or dict(item) != expected_map[field]:
            raise CaptureError("REFERENCE_PROJECTION_BINDING_INVALID", label)
        for index, identity in item.items():
            if not isinstance(index, str) or not re.fullmatch(r"(?:0|[1-9][0-9]*)", index):
                raise CaptureError("REFERENCE_PROJECTION_INVALID", label)
            _sha(identity, label + " " + field)
    if not isinstance(counts, Mapping) or set(counts) != set(REFERENCE_COUNT_FIELDS) or \
            any(type(counts[field]) is not int or counts[field] < 0
                for field in REFERENCE_COUNT_FIELDS) or \
            dict(counts) != expected_counts:
        raise CaptureError("REFERENCE_COUNT_INVALID", label)
    if require_complete:
        indices = {str(item["repository_index"]) for item in signature_references}
        if any(set(value[field]) != indices for field in REFERENCE_MAP_FIELDS):
            raise CaptureError("REFERENCE_PROJECTION_SET_INVALID", label)
        if expected_counts["unique_signature_references"] != signature_count:
            raise CaptureError("REFERENCE_COUNT_INVALID", label)
    return dict(value), dict(counts)


def _validate_inner_directory_binding(path: Path, host_contract: Mapping[str, Any]) -> Mapping[str, Any]:
    """Require helper output to agree with host-owned pre-container descriptors."""
    data, _ = _read_bounded(path, "inner directory contract", MAX_JSON_BYTES, allow_empty=False)
    _descriptor(path, "inner directory contract", MAX_JSON_BYTES, sidecar=True)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError("DIRECTORY_CONTRACT_INVALID", "inner JSON") from error
    if not isinstance(value, Mapping) or value.get("schema") != IN_CONTAINER_SCHEMA or \
            value.get("schema_version") != IN_CONTAINER_SCHEMA_VERSION or \
            value.get("status") not in {"CAPTURED_REVIEW_REQUIRED", "FAILED_REVIEW_REQUIRED"}:
        raise CaptureError("DIRECTORY_CONTRACT_INVALID", "inner result status")
    if value.get("status") == "FAILED_REVIEW_REQUIRED":
        failure = value.get("failure")
        if not isinstance(failure, Mapping) or set(failure) != {"kind"}:
            raise CaptureError("DIRECTORY_CONTRACT_INVALID", "inner failure")
        validated_release: list[dict[str, Any]] = []
        validated_packages: list[dict[str, Any]] = []

        def _attach_unproven_projection(error: CaptureError) -> None:
            """Expose only the independently reopened evidence prefix.

            A failed inner capture may have no structured signature evidence.
            Release and Packages bytes are still useful diagnostic evidence,
            but raw gpgv logs must never be promoted into signer references.
            Keep the outer receipt deterministic by attaching a projection
            computed from the validated prefix and an explicitly empty
            signature set.
            """
            error.release_evidence = _immutable_copy(validated_release)
            error.package_evidence = _immutable_copy(validated_packages)
            error.signature_evidence = []
            error.signature_references = []
            error.signature_count = 0
            error.reference_map, error.reference_counts = _reference_projection(
                error.release_evidence, error.package_evidence, [], 0)

        if "release_evidence" in value:
            try:
                validated_release = _validate_release_evidence(
                    path.parent, value["release_evidence"])
            except CaptureError:
                raise
        if "package_evidence" in value:
            try:
                validated_packages = _validate_package_evidence(
                    path.parent, value["package_evidence"])
            except CaptureError as error:
                error.release_evidence = validated_release
                error.package_evidence = []
                # A malformed package item is not a proof of any package or
                # signer relationship.  Preserve only the independently
                # reopened Release prefix and publish the deterministic empty
                # package/signature projection to the outer receipt.
                error.signature_evidence = []
                error.signature_references = []
                error.signature_count = 0
                error.reference_map, error.reference_counts = _reference_projection(
                    validated_release, [], [], 0)
                raise
        if "signature_evidence" in value:
            try:
                validated_signatures = _validate_gpgv_evidence(
                    path.parent, value["signature_evidence"], require_nonempty=True,
                    partial_failure_kind=failure["kind"])
            except CaptureError as error:
                # Release and compressed Packages bytes have already been
                # independently reopened.  Preserve that trusted prefix for
                # the host receipt even when a later gpgv/status check fails.
                error.release_evidence = validated_release
                error.package_evidence = validated_packages
                error.signature_references = _immutable_copy(
                    value.get("signature_references", []))
                error.signature_count = value.get("signature_count", 0)
                has_references = any(
                    key in value for key in (
                        "signature_references", "signature_count", "source_manifest"))
                has_projection = any(
                    key in value for key in ("reference_map", "reference_counts"))
                if ((has_projection and (
                        "reference_map" not in value or
                        "reference_counts" not in value)) or
                        (has_references and not has_projection)):
                    raise CaptureError("REFERENCE_PROJECTION_MISSING", "failed APT capture")
                _validate_signature_references(
                    value.get("signature_references", []), value.get("signature_count", 0),
                    require_complete=False)
                if has_projection:
                    error.reference_map, error.reference_counts = _validate_reference_projection(
                        value["reference_map"], value["reference_counts"],
                        validated_release, validated_packages,
                        value.get("signature_references", []), value.get("signature_count", 0),
                        require_complete=False)
                raise
            if "signature_references" in value:
                _validate_signature_references(
                    value["signature_references"], value.get("signature_count"),
                    validated_signatures, require_complete=False)
            has_references = any(
                key in value for key in (
                    "signature_references", "signature_count", "source_manifest"))
            has_projection = any(
                key in value for key in ("reference_map", "reference_counts"))
            if ((has_projection and (
                    "reference_map" not in value or
                    "reference_counts" not in value)) or
                    (has_references and not has_projection)):
                raise CaptureError("REFERENCE_PROJECTION_MISSING", "failed APT capture")
            if has_projection:
                _validate_reference_projection(
                    value["reference_map"], value["reference_counts"],
                    validated_release, validated_packages,
                    value.get("signature_references", []), value.get("signature_count", 0),
                    require_complete=False)
        else:
            # Package validation can fail before a signature evidence object
            # is safely exposable.  In that case a producer must still seal
            # the Release-only projection; validate it here instead of
            # silently accepting a forged or stale zero-count map.
            has_references = any(
                key in value for key in (
                    "signature_references", "signature_count", "source_manifest"))
            has_projection = any(
                key in value for key in ("reference_map", "reference_counts"))
            if has_references or has_projection:
                if ("source_manifest" in value or
                        "signature_references" not in value or
                        "signature_count" not in value or
                        "reference_map" not in value or
                        "reference_counts" not in value):
                    error = CaptureError(
                        "REFERENCE_PROJECTION_MISSING", "failed APT capture")
                    _attach_unproven_projection(error)
                    raise error
                # Without a structured signature object, these fields may
                # only describe an explicitly unproven empty signer set.
                # Accepting references here would turn raw gpgv diagnostics
                # into signer proof and would permit cross-type substitution.
                if (not isinstance(value["signature_references"], list) or
                        value["signature_references"] or
                        type(value["signature_count"]) is not int or
                        value["signature_count"] != 0):
                    error = CaptureError(
                        "SIGNATURE_EVIDENCE_MISSING", "failed APT capture")
                    _attach_unproven_projection(error)
                    raise error
                try:
                    _validate_reference_projection(
                        value["reference_map"], value["reference_counts"],
                        validated_release, validated_packages, [], 0,
                        require_complete=False)
                except CaptureError as error:
                    _attach_unproven_projection(error)
                    raise
            elif validated_release or validated_packages:
                # Evidence without a sealed projection cannot be copied into
                # the outer receipt consistently.  Fail closed while making
                # the validated prefix available to the caller.
                error = CaptureError(
                    "REFERENCE_PROJECTION_MISSING", "failed APT capture")
                _attach_unproven_projection(error)
                raise error
    elif "release_evidence" in value:
        _validate_release_evidence(path.parent, value["release_evidence"], require_nonempty=True)
        if "source_manifest" in value and "package_evidence" not in value:
            raise CaptureError("PACKAGES_EVIDENCE_MISSING", "successful APT capture")
        if "package_evidence" in value:
            _validate_package_evidence(
                path.parent, value["package_evidence"], require_nonempty=True,
                require_decoded="source_manifest" in value)
        if "source_manifest" in value:
            if "signature_references" not in value or "signature_count" not in value:
                raise CaptureError("SIGNATURE_REFERENCE_MISSING", "successful APT capture")
            _validate_signature_references(
                value["signature_references"], value["signature_count"],
                require_complete=True)
            if "reference_map" not in value or "reference_counts" not in value:
                raise CaptureError("REFERENCE_PROJECTION_MISSING", "successful APT capture")
            _validate_reference_projection(
                value["reference_map"], value["reference_counts"],
                value["release_evidence"], value["package_evidence"],
                value["signature_references"], value["signature_count"],
                require_complete=True)
    elif "source_manifest" in value and value.get("phase") == "apt-source":
        raise CaptureError("RELEASE_EVIDENCE_INVALID", "successful APT capture")
    if value.get("phase") == "apt-source" and any(
            field in value for field in (
                "rosdep_source_files", "rosdep_cache_files", "rosdep_sources",
                "rosdep_cache")):
        raise CaptureError(
            "ROSDEP_EVIDENCE_PREMATURE", "APT source capture cannot claim rosdep evidence")
    inner = value.get("directory_contract")
    if _inner_directory_projection(inner) != _inner_directory_projection(host_contract):
        raise CaptureError("DIRECTORY_CONTRACT_DRIFT", "inner/outer descriptor mismatch")
    return value


def _normalize_output(result: Any) -> tuple[int, bytes, bytes, bool, int | None]:
    if isinstance(result, (bytes, str)):
        return 0, result if isinstance(result, bytes) else result.encode(), b"", False, None
    if isinstance(result, tuple):
        values = list(result)
        if len(values) not in (3, 4, 5):
            raise CaptureError("RUNNER_RESULT_INVALID", "tuple result shape")
        if len(values) == 3:
            values.extend((False, None))
        elif len(values) == 4:
            values.append(None)
        if type(values[0]) is not int or type(values[3]) is not bool:
            raise CaptureError("RUNNER_RESULT_INVALID", "tuple result types")
        if values[4] is not None and type(values[4]) is not int:
            raise CaptureError("RUNNER_RESULT_INVALID", "signal type")
        return int(values[0]), _bytes(values[1]), _bytes(values[2]), bool(values[3]), values[4]
    try:
        return (int(getattr(result, "returncode")), _bytes(getattr(result, "stdout", b"")),
                _bytes(getattr(result, "stderr", b"")), bool(getattr(result, "timed_out", False)),
                getattr(result, "signal", None))
    except (TypeError, ValueError) as error:
        raise CaptureError("RUNNER_RESULT_INVALID", "result fields") from error


def _bytes(value: Any) -> bytes:
    if value is None:
        return b""
    if isinstance(value, bytes):
        return value
    if isinstance(value, str):
        return value.encode("utf-8")
    raise CaptureError("RUNNER_RESULT_INVALID", "output is not bytes/text")


def _default_runner(argv: list[str], timeout: int) -> tuple[int, bytes, bytes, bool, int | None]:
    try:
        result = subprocess.run(argv, stdin=subprocess.DEVNULL, stdout=subprocess.PIPE,
                                stderr=subprocess.PIPE, shell=False, check=False,
                                timeout=timeout, env=_fixed_env())
    except subprocess.TimeoutExpired as error:
        return 124, _bytes(error.stdout), _bytes(error.stderr), True, None
    except (OSError, subprocess.SubprocessError) as error:
        raise CaptureError("COMMAND_UNAVAILABLE", str(error)) from error
    return int(result.returncode), _bytes(result.stdout), _bytes(result.stderr), False, None


def _fixed_env() -> dict[str, str]:
    return {key: value for key, value in os.environ.items() if key in SAFE_ENV_KEYS}


def _command_hash(argv: list[str]) -> str:
    return sha256_bytes(canonical_bytes(argv))


def _validate_argv(argv: Any, label: str) -> list[str]:
    if (not isinstance(argv, list) or not argv or
            any(not isinstance(token, str) or not token or len(token) > 2048 or
                any(ord(char) < 0x20 or ord(char) == 0x7f for char in token)
                for token in argv)):
        raise CaptureError("COMMAND_INVALID", label)
    if any(token in FORBIDDEN_TOKENS for token in argv):
        raise CaptureError("COMMAND_UNSAFE", label)
    return list(argv)


def _missing_container_output(stdout: bytes, stderr: bytes, name: str) -> bool:
    """Recognize only Docker's exact missing-container response.

    Docker 29 emits an empty JSON result (``[]\\n``) on stdout and the
    human-readable missing-object diagnostic on stderr.  Keep the streams
    separate: concatenating them would make a valid response impossible to
    distinguish from extra or injected output.
    """
    if stdout not in (b"", b"[]\n"):
        return False
    try:
        text = stderr.decode("utf-8")
    except UnicodeDecodeError as error:
        raise CaptureError("DOCKER_ABSENCE_UNPROVEN", name) from error
    return text in tuple(prefix + name + suffix for prefix in MISSING_ERROR_PREFIXES
                         for suffix in ("", "\n"))


def _image_from_profile(profile: Mapping[str, Any], distro: str) -> dict[str, Any]:
    release = AUDIT._validate_release_matrix_profile(profile)
    row = next((item for item in release["distros"] if item["name"] == distro), None)
    if row is None:
        raise CaptureError("DISTRO_INVALID", distro)
    image = AUDIT._validate_image_contract(
        row["image"], distro, require_humble_manifest=distro == "humble")
    return image


def _profile_inputs(repo_root: Path, profile_path: Path, distro: str,
                    dependency_leg: str) -> dict[str, Any]:
    profile, profile_file = AUDIT._load_profile(profile_path)
    release = AUDIT._validate_release_matrix_profile(profile)
    if (distro, dependency_leg) not in ROWS:
        raise CaptureError("MATRIX_ROW_INVALID", "{} {}".format(distro, dependency_leg))
    image = _image_from_profile(profile, distro)
    source = AUDIT._source_manifest(repo_root, profile)
    if source.get("status") != "PASS":
        raise CaptureError("CURRENT_SOURCE_HASH_MISMATCH", json.dumps(source, sort_keys=True))
    leg = next(item for item in release["distros"] if item["name"] == distro)["legs"][dependency_leg]
    if dependency_leg == "present" and leg.get("status") != "READY_PINNED":
        raise CaptureError("OPTIONAL_DEPENDENCY_NOT_PINNED", distro)
    return {
        "profile": profile, "profile_path": profile_file,
        "profile_sha256": AUDIT.sha256_file(profile_file), "release": release,
        "source": source, "image": image, "leg": leg,
    }


def _container_name(root: Path, distro: str, dependency_leg: str) -> str:
    seed = "registration-plugin-capture-{}-{}-{}".format(distro, dependency_leg, root.name.lower())
    if len(seed) > 63:
        seed = "registration-plugin-capture-{}-{}-{}".format(
            distro, dependency_leg, hashlib.sha256(str(root).encode()).hexdigest()[:10])
    if NAME_RE.fullmatch(seed) is None:
        raise CaptureError("CONTAINER_NAME_INVALID", seed)
    return seed


def build_plan(repo_root: Path, profile_path: Path, distro: str,
               dependency_leg: str, output_root: Path) -> dict[str, Any]:
    """Build a fixed plan without creating a root or invoking a process."""
    repo_root = _safe_abs(str(repo_root), "repository root")
    profile_path = _safe_abs(str(profile_path), "profile path")
    output_root = _safe_abs(str(output_root), "capture output root")
    if not repo_root.is_dir() or repo_root.is_symlink():
        raise CaptureError("REPOSITORY_INVALID", str(repo_root))
    if not profile_path.is_file() or profile_path.is_symlink():
        raise CaptureError("PROFILE_INVALID", str(profile_path))
    _parents_no_symlink(profile_path, "profile path")
    inputs = _profile_inputs(repo_root, profile_path, distro, dependency_leg)
    # Older synthetic/unit profiles intentionally predate the explicit URL
    # policy.  The production profile is checked by the audit above and must
    # carry the exact policy; a missing policy in that profile is therefore a
    # hard error.  Keep the bounded default only for the legacy test seam,
    # where no dependency-closure projection exists at all.
    dependency_closure = inputs["release"].get("dependency_closure")
    if dependency_closure is None:
        apt_source_url_policy = dict(APT_SOURCE_URL_POLICY)
    else:
        apt_source_url_policy = dependency_closure["apt_source_url_policy"]
    if apt_source_url_policy != APT_SOURCE_URL_POLICY:
        raise CaptureError("APT_URL_POLICY_INVALID", "profile policy")
    if dependency_closure is None:
        release_declaration_policy = dict(RELEASE_DECLARATION_POLICY)
    else:
        release_declaration_policy = dict(
            dependency_closure.get("capture_contract", {}).get(
                "release_declaration_policy", RELEASE_DECLARATION_POLICY))
    if release_declaration_policy != RELEASE_DECLARATION_POLICY:
        raise CaptureError("RELEASE_DECLARATION_POLICY_INVALID", "profile policy")
    if dependency_closure is None:
        packages_format_policy = dict(PACKAGES_FORMAT_POLICY)
    else:
        packages_format_policy = dict(
            dependency_closure.get("capture_contract", {}).get(
                "packages_format_policy", PACKAGES_FORMAT_POLICY))
    if packages_format_policy != PACKAGES_FORMAT_POLICY:
        raise CaptureError("PACKAGES_FORMAT_POLICY_INVALID", "profile policy")
    if dependency_closure is None:
        gpgv_status_policy = dict(GPGV_STATUS_POLICY)
    else:
        capture_contract = dependency_closure.get("capture_contract")
        if (not isinstance(capture_contract, Mapping) or
                "gpgv_status_policy" not in capture_contract):
            raise CaptureError("GPGV_STATUS_POLICY_INVALID", "profile policy")
        gpgv_status_policy = dict(capture_contract["gpgv_status_policy"])
    if gpgv_status_policy != GPGV_STATUS_POLICY:
        raise CaptureError("GPGV_STATUS_POLICY_INVALID", "profile policy")
    rosdep_prepare_contract = _rosdep_prepare_contract(
        required=dependency_closure is not None,
        profile_path=profile_path,
        profile_sha256=inputs["profile_sha256"],
    )
    name = _container_name(output_root, distro, dependency_leg)
    image = inputs["image"]
    apt_transient_owner = _apt_transient_owner_for_distro(distro)
    apt_transient_owner.update({
        "profile_sha256": inputs["profile_sha256"],
        "image_digest": image["digest"],
    })
    env = {
        "ROS_DISTRO": distro,
        "DEBIAN_FRONTEND": "noninteractive",
        "LANG": "C.UTF-8", "LC_ALL": "C.UTF-8", "TZ": "UTC",
    }
    mount_repo = "{}:/workspace/src:ro".format(repo_root)
    mount_output = "{}:/workspace/evidence-parent:rw".format(output_root)
    run_argv = [
        "docker", "run", "-d", "--name", name, "--platform", "linux/amd64",
        "--pull=never", "--network", "bridge",
    ]
    for key in SAFE_ENV_KEYS:
        if key in env:
            run_argv.extend(["--env", "{}={}".format(key, env[key])])
    prepare_mounts = _prepare_mount_specs(output_root)
    restore_owner_mounts = _restore_owner_mount_specs(output_root)
    run_argv.extend([
        "-v", mount_repo, "-v", mount_output,
    ])
    for mount in prepare_mounts:
        mode = "ro" if mount["read_only"] else "rw"
        run_argv.extend(["-v", "{}:{}:{}".format(
            mount["source"], mount["target"], mode)])
    for mount in restore_owner_mounts:
        run_argv.extend(["-v", "{}:{}:ro".format(
            mount["source"], mount["target"])])
    run_argv.extend([image["reference"], "tail", "-f", "/dev/null"])
    # The output root itself is mounted at this path.  Do not derive a second
    # basename below the mount: that would make the fixed container command
    # refer to a directory that the host deliberately did not pre-create.
    container_output = "/workspace/evidence-parent"
    capture_script = "/workspace/src/scripts/capture_registration_plugin_dependency_closure.py"
    provisioning_packages = _provisioning_packages(distro)
    phase_argv = {
        "image_inspect": ["docker", "image", "inspect", image["reference"]],
        "preexisting_container_check": ["docker", "inspect", name],
        "container_start": run_argv,
        "apt_update": ["docker", "exec", name, "apt-get", "update"],
        "base_status_snapshot": ["docker", "exec", name, "dpkg-query", "-W",
                                  "-f=${Package}\\t${Version}\\t${Architecture}\\t${Status}\\n"],
        "apt_source_snapshot": ["docker", "exec", name, "python3", capture_script,
                                "in-container", "--phase", "apt-source",
                                "--output-root", container_output,
                                "--apt-url-policy", APT_URL_POLICY_ID],
        "resolver_build": ["docker", "exec", name, "apt-get", "--quiet=2", "--print-uris",
                            "--download-only", "--yes", "--no-upgrade",
                            "--no-install-recommends", "install",
                            *provisioning_packages],
        "download_build": ["docker", "exec", name, "apt-get", "--download-only", "--yes",
                           "--no-upgrade", "--no-install-recommends", "-o",
                           "Dir::Cache::archives=/workspace/evidence-parent/debs-build",
                           "install", *provisioning_packages],
        "restore_build_partial_owner": _restore_owner_command(name, "build"),
        "resolver_runtime": ["docker", "exec", name, "apt-get", "--quiet=2", "--print-uris",
                              "--download-only", "--yes", "--no-upgrade",
                              "--no-install-recommends", "install",
                              *provisioning_packages],
        "download_runtime": ["docker", "exec", name, "apt-get", "--download-only", "--yes",
                             "--no-upgrade", "--no-install-recommends", "-o",
                             "Dir::Cache::archives=/workspace/evidence-parent/debs-runtime",
                             "install", *provisioning_packages],
        "restore_runtime_partial_owner": _restore_owner_command(name, "runtime"),
        "dependency_install": [
            "docker", "exec", name, "apt-get", "--no-download", "--no-upgrade",
            "--yes", "--no-install-recommends", "-o",
            "Dir::Cache::archives=/workspace/evidence-parent/debs-build",
            "-o", "APT::Sandbox::User=root",
            "install", *provisioning_packages,
        ],
        "network_disconnect": ["docker", "network", "disconnect", "bridge", name],
        "rosdep_prepare": _prepare_phase_argv(name, output_root, distro),
        "rosdep_resolution": _rosdep_resolution_phase_argv(name, distro),
        "dependency_capture": ["docker", "exec", name, "python3", capture_script,
                                "in-container", "--phase", "dependency",
                                "--output-root", container_output,
                                "--apt-url-policy", APT_URL_POLICY_ID],
        "archive_prefetch": ["python3", str(PREFETCH_PATH), "prefetch-pinned-archives"],
        "post_disconnect_inspect": ["docker", "inspect", name],
    }
    for phase, argv in phase_argv.items():
        _validate_argv(argv, phase)
    fixed = {
        "schema": SCHEMA, "schema_version": SCHEMA_VERSION,
        "repo_root": str(repo_root), "profile_path": str(profile_path),
        "profile_sha256": inputs["profile_sha256"],
        "source_manifest": inputs["source"],
        "distro": distro, "dependency_leg": dependency_leg,
        "campaign_set": inputs["release"]["campaign_set"], "leg": inputs["leg"],
        "image": image, "container_name": name, "output_root": str(output_root),
        "apt_source_url_policy": apt_source_url_policy,
        "release_declaration_policy": release_declaration_policy,
        "packages_format_policy": packages_format_policy,
        "gpgv_status_policy": gpgv_status_policy,
        "rosdep_prepare_contract": rosdep_prepare_contract,
        "apt_transient_owner": apt_transient_owner,
        "rosdep_prepare_mounts": prepare_mounts,
        "restore_owner_policy": dict(RESTORE_OWNER_POLICY),
        "restore_owner_reference_mounts": restore_owner_mounts,
        "rosdep_prepare_bundle": _immutable_copy(
            rosdep_prepare_contract["input_bundle"]),
        "rosdep_prepare_discovery": _immutable_copy(
            rosdep_prepare_contract["discovery"]),
        "container_output": container_output, "environment": env,
        "run_argv": run_argv, "phase_argv": phase_argv,
        "phase_argv_sha256": {key: _command_hash(value) for key, value in phase_argv.items()},
        "rosdep_prepare_argv": _prepare_argv_metadata({
            "container_name": name, "phase_argv": phase_argv,
            "output_root": str(output_root),
            "distro": distro,
            "rosdep_prepare_contract": rosdep_prepare_contract,
        }),
        "timeout_seconds": COMMAND_TIMEOUT_SECONDS,
        "max_retries": 0,
        "network_policy": {
            "provisioning_network_allowed": True,
            "build_test_network_connected": False,
            "build_test_network_used": False,
            "docker_pull": False,
        },
    }
    fixed["plan_identity_sha256"] = canonical_hash(fixed, "plan_identity_sha256")
    return fixed


def _write_log(root: Path, phase: str, stream: str, data: bytes) -> dict[str, Any]:
    if len(data) > MAX_LOG_BYTES:
        raise CaptureError("LOG_OVERSIZE", phase)
    relative = Path("logs") / (phase + "." + stream)
    path = root / relative
    _write_new(path, data if data else b"\n", maximum=MAX_LOG_BYTES)
    return {"path": str(relative), "bytes": len(data) if data else 1,
            "sha256": sha256_bytes(data if data else b"\n")}


def _phase_record(plan: Mapping[str, Any], root: Path, phase: str,
                  result: tuple[int, bytes, bytes, bool, int | None],
                  *, logical_returncode: int | None = None) -> dict[str, Any]:
    returncode, stdout, stderr, timed_out, signal = result
    stdout_desc = _write_log(root, phase, "stdout", stdout)
    stderr_desc = _write_log(root, phase, "stderr", stderr)
    return {
        "phase": phase, "argv": list(plan["phase_argv"][phase]),
        "argv_sha256": plan["phase_argv_sha256"][phase],
        "returncode": returncode if logical_returncode is None else logical_returncode,
        "observed_returncode": returncode,
        "network_used": (plan["network_policy"]["provisioning_network_allowed"]
                          if PHASE_NETWORK[phase] is True else
                          (phase == "archive_prefetch" and plan["dependency_leg"] == "present")),
        "timeout_seconds": plan["timeout_seconds"], "attempts": 1,
        "stdout": stdout_desc, "stderr": stderr_desc,
        "timed_out": timed_out, "signal": signal,
    }


def _run_phase(plan: Mapping[str, Any], root: Path, phase: str,
               runner: Callable[[list[str], int], Any]) -> dict[str, Any]:
    argv = list(plan["phase_argv"][phase])
    raw_result = runner(argv, int(plan["timeout_seconds"]))
    result = _normalize_output(raw_result)
    if (result[1] and len(result[1]) > MAX_LOG_BYTES) or \
            (result[2] and len(result[2]) > MAX_LOG_BYTES):
        raise CaptureError("LOG_OVERSIZE", phase)
    return _phase_record(plan, root, phase, result)


def _run_restore_owner_phase(
        plan: Mapping[str, Any], root: Path, phase: str,
        runner: Callable[[list[str], int], Any]
        ) -> tuple[dict[str, Any], dict[str, Any]]:
    """Run one owner restore and retain its host-side proof.

    APT's cache helper may change the owner of ``partial`` while downloading.
    The phase uses only the read-only reference bind as the source of truth;
    all identity and child checks happen in the host namespace before and
    after the command.  No post-hoc host ``chown``/``chmod`` is attempted.
    """
    metadata = _restore_owner_phase_metadata(plan, phase)
    role = metadata["role"]
    fixed = next(item for item in RESTORE_OWNER_REFERENCE_MOUNTS
                 if item["role"] == role)
    mount = next(item for item in plan["restore_owner_reference_mounts"]
                 if item["role"] == role)
    reference = Path(mount["source"])
    target_relative = "debs-build/partial" if role == "build" else "debs-runtime/partial"
    target = root / target_relative
    reference_before = _restore_reference_descriptor(reference, phase + " reference before")
    target_presealed = _restore_owner_presealed_target(
        plan, root, role, target, phase)
    target_before, target_before_observation = _restore_owner_target_before(
        plan, root, role, target, target_presealed, phase)
    if target_before["mode"] != reference_before["mode"]:
        raise CaptureError("RESTORE_OWNER_MODE_INVALID", phase + " target before")
    if target_before["children"]:
        raise CaptureError("RESTORE_OWNER_CHILD_NOT_EMPTY", phase + " target before")
    apt_identity = _apt_transient_owner_binding(plan)

    raw_result = runner(list(metadata["argv"]), int(plan["timeout_seconds"]))
    result = _normalize_output(raw_result)
    if (result[1] and len(result[1]) > MAX_LOG_BYTES) or \
            (result[2] and len(result[2]) > MAX_LOG_BYTES):
        raise CaptureError("LOG_OVERSIZE", phase)
    phase_record = _phase_record(plan, root, phase, result)
    reference_after = _restore_reference_descriptor(reference, phase + " reference after")
    target_after = _restore_directory_snapshot(target, phase + " target after")
    valid = (
        reference_before == reference_after and
        target_before["path"] == target_after["path"] and
        target_before["type"] == target_after["type"] and
        target_before["device"] == target_after["device"] and
        target_before["inode"] == target_after["inode"] and
        target_before["mode"] == target_after["mode"] and
        target_before["nlink"] == target_after["nlink"] and
        target_after["uid"] == reference_after["uid"] and
        target_after["gid"] == reference_after["gid"] and
        target_after["mode"] == reference_after["mode"] and
        target_after["nlink"] == 2 and
        not target_after["children"]
    )
    status = "PASS" if result[0] == 0 and valid else "FAILED"
    if not valid and result[0] == 0:
        # Preserve a phase record while distinguishing an observed command
        # success from a failed filesystem contract.  Receipt validation
        # permits this one logical failure encoding and still binds the raw
        # command rc in ``observed_returncode``.
        phase_record["returncode"] = 1
    evidence = {
        "phase": phase, "role": role, "argv": list(metadata["argv"]),
        "argv_sha256": metadata["argv_sha256"],
        "returncode": phase_record["returncode"],
        "observed_returncode": phase_record["observed_returncode"],
        "stdout": _immutable_copy(phase_record["stdout"]),
        "stderr": _immutable_copy(phase_record["stderr"]),
        "reference_mount": _immutable_copy(mount),
        "target_relative": target_relative,
        "target": fixed["target_partial"],
        "apt_identity": apt_identity,
        "target_presealed": _immutable_copy(target_presealed),
        "target_before_observation": target_before_observation,
        "reference_before": reference_before,
        "reference_after": reference_after,
        "target_before": target_before,
        "target_after": target_after,
        "children_before": _immutable_copy(target_before["children"]),
        "children_after": _immutable_copy(target_after["children"]),
        "status": status,
    }
    if result[0] != 0:
        # A command failure is itself fail-closed, but retaining the complete
        # pre/post descriptors makes the partial receipt independently useful.
        evidence["status"] = "FAILED"
    plan.setdefault("_restore_owner_records", {})[phase] = evidence
    return phase_record, evidence


def _inspect_image(output: bytes, image: Mapping[str, Any]) -> dict[str, Any]:
    try:
        value = json.loads(output.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError("IMAGE_INSPECT_INVALID", str(error)) from error
    if not isinstance(value, list) or len(value) != 1 or not isinstance(value[0], Mapping):
        raise CaptureError("IMAGE_INSPECT_INVALID", "expected one image")
    observed = value[0]
    if observed.get("Id") != image["digest"] or observed.get("Os") != "linux" or \
            observed.get("Architecture") != "amd64":
        raise CaptureError("IMAGE_IDENTITY_MISMATCH", "image digest/platform")
    repo_digests = observed.get("RepoDigests")
    if not isinstance(repo_digests, list) or not any(
            str(item).endswith("@" + image["digest"]) for item in repo_digests):
        raise CaptureError("IMAGE_IDENTITY_MISMATCH", "RepoDigests")
    return {"id": observed["Id"], "repo_digests": sorted(repo_digests),
            "os": observed["Os"], "architecture": observed["Architecture"]}


def _inspect_absence(returncode: int, stdout: bytes, stderr: bytes, name: str) -> None:
    if returncode != DOCKER_MISSING_RC or not _missing_container_output(stdout, stderr, name):
        raise CaptureError("DOCKER_ABSENCE_UNPROVEN", name)


def _base_status_from_log(root: Path, record: Mapping[str, Any]) -> None:
    data, _ = _read_bounded(root / record["stdout"]["path"], "base status log",
                            MAX_LOG_BYTES, allow_empty=False)
    if not data.strip():
        raise CaptureError("BASE_STATUS_EMPTY", "dpkg-query returned no packages")
    # The composer accepts a sealed JSON projection, while the command log
    # remains the authoritative raw host-owned output.
    packages = []
    for line in data.decode("utf-8").splitlines():
        fields = line.split("\t")
        if len(fields) != 4 or any(not item for item in fields):
            raise CaptureError("BASE_STATUS_INVALID", line)
        name, version, architecture, status = fields
        if status != "install ok installed":
            continue
        packages.append({"name": name, "version": version,
                         "architecture": architecture, "status": status})
    packages.sort(key=lambda item: (item["name"], item["version"], item["architecture"]))
    if not packages:
        raise CaptureError("BASE_STATUS_EMPTY", "no installed packages")
    value = {
        "schema": "glim_clean_room_r3_base_dpkg_status_v1", "schema_version": 1,
        "status": "SEALED_PREINSTALL", "packages": packages,
    }
    value["canonical_sha256"] = canonical_hash(value)
    _seal_pair(root, "base-status.json", value)


def _container_path(container_root: Path | None, absolute: str, label: str) -> Path:
    """Map a container absolute path to a fixture root without resolving links."""
    requested = _safe_abs(absolute, label)
    if container_root is None or Path(container_root) == Path("/"):
        return requested
    root = _safe_abs(str(container_root), "container fixture root")
    try:
        info = root.lstat()
    except OSError as error:
        raise CaptureError("CONTAINER_ROOT_INVALID", str(root)) from error
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise CaptureError("CONTAINER_ROOT_INVALID", str(root))
    return root / str(requested).lstrip("/")


def _container_read(path: Path, label: str, maximum: int = MAX_CAPTURE_FILE_BYTES,
                   *, allow_empty: bool = False,
                   exact_symlink_target: str | None = None,
                   symlink_target_path: Path | None = None,
                   require_root: bool = False) -> tuple[bytes, dict[str, Any]]:
    """Read a container file with lstat/fstat/reopen and no-follow checks.

    Container package/source files are commonly mode 0644, so this reader
    rejects group/other writable files but does not require the 0444 output
    mode used by the host evidence seal.  The one Humble ROS source alias is
    represented explicitly; its resolved bytes are still checked as a regular
    single-link file.
    """
    path = _safe_abs(str(path), label)
    _parents_no_symlink(path, label)
    try:
        parent_before = path.parent.lstat()
        before = path.lstat()
    except OSError as error:
        raise CaptureError("CONTAINER_FILE_MISSING", label) from error
    if stat.S_ISLNK(before.st_mode):
        if exact_symlink_target is None:
            raise CaptureError("CONTAINER_SYMLINK_REJECTED", label)
        target = os.readlink(path)
        if target != exact_symlink_target:
            raise CaptureError("CONTAINER_SYMLINK_TARGET_INVALID", label)
        target_path = symlink_target_path or _safe_abs(target, label + " symlink target")
        data, target_descriptor = _container_read(
            target_path, label + " symlink target", maximum,
            allow_empty=allow_empty, require_root=require_root)
        try:
            after = path.lstat()
            parent_after = path.parent.lstat()
            if (after.st_dev, after.st_ino, after.st_size, after.st_nlink) != \
                    (before.st_dev, before.st_ino, before.st_size, before.st_nlink) or \
                    os.readlink(path) != target or \
                    (parent_after.st_dev, parent_after.st_ino) != \
                    (parent_before.st_dev, parent_before.st_ino) or \
                    stat.S_ISLNK(parent_after.st_mode):
                raise CaptureError("CONTAINER_SYMLINK_CHANGED", label)
        except OSError as error:
            raise CaptureError("CONTAINER_SYMLINK_CHANGED", label) from error
        descriptor = {
            "path": str(path), "kind": "exact_symlink", "target": target,
            "target_descriptor": target_descriptor,
            "bytes": len(data), "sha256": sha256_bytes(data),
            "mode": stat.S_IMODE(before.st_mode), "uid": before.st_uid,
            "gid": before.st_gid, "nlink": before.st_nlink,
            "device": before.st_dev, "inode": before.st_ino,
        }
        return data, descriptor
    if (not stat.S_ISREG(before.st_mode) or before.st_nlink != 1 or
            before.st_size > maximum or (before.st_size == 0 and not allow_empty) or
            stat.S_IMODE(before.st_mode) & 0o022 or
            (require_root and (before.st_uid != 0 or before.st_gid != 0))):
        raise CaptureError("CONTAINER_FILE_INVALID", label)
    fd = None
    try:
        fd = os.open(str(path), os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) |
                     getattr(os, "O_CLOEXEC", 0))
        fd_before = os.fstat(fd)
        identity = (before.st_dev, before.st_ino, before.st_size, before.st_nlink)
        if (fd_before.st_dev, fd_before.st_ino, fd_before.st_size, fd_before.st_nlink) != identity:
            raise CaptureError("CONTAINER_FILE_IDENTITY_CHANGED", label)
        chunks: list[bytes] = []
        total = 0
        while True:
            block = os.read(fd, min(1024 * 1024, maximum - total + 1))
            if not block:
                break
            chunks.append(block)
            total += len(block)
            if total > maximum:
                raise CaptureError("CONTAINER_FILE_OVERSIZE", label)
        fd_after = os.fstat(fd)
    except OSError as error:
        raise CaptureError("CONTAINER_FILE_READ_FAILED", label) from error
    finally:
        if fd is not None:
            os.close(fd)
    data = b"".join(chunks)
    try:
        after = path.lstat()
        parent_after = path.parent.lstat()
    except OSError as error:
        raise CaptureError("CONTAINER_FILE_CHANGED", label) from error
    if (len(data) != before.st_size or
            (fd_after.st_dev, fd_after.st_ino, fd_after.st_size, fd_after.st_nlink) != identity or
            (after.st_dev, after.st_ino, after.st_size, after.st_nlink) != identity or
            (parent_after.st_dev, parent_after.st_ino) !=
            (parent_before.st_dev, parent_before.st_ino) or
            stat.S_ISLNK(parent_after.st_mode)):
        raise CaptureError("CONTAINER_FILE_CHANGED", label)
    return data, {
        "path": str(path), "kind": "regular", "bytes": len(data),
        "sha256": sha256_bytes(data), "mode": stat.S_IMODE(before.st_mode),
        "uid": before.st_uid, "gid": before.st_gid, "nlink": before.st_nlink,
        "device": before.st_dev, "inode": before.st_ino,
    }


def _container_directory(path: Path, label: str, *, allow_missing: bool = False) -> bool:
    path = _safe_abs(str(path), label)
    if allow_missing:
        current = Path(path.anchor)
        for component in path.parts[1:]:
            current /= component
            try:
                component_info = current.lstat()
            except FileNotFoundError:
                return False
            except OSError as error:
                raise CaptureError("CONTAINER_DIRECTORY_READ_FAILED", label) from error
            if stat.S_ISLNK(component_info.st_mode) or not stat.S_ISDIR(component_info.st_mode):
                raise CaptureError("CONTAINER_DIRECTORY_INVALID", label)
    else:
        _parents_no_symlink(path, label)
    try:
        info = path.lstat()
    except OSError:
        if allow_missing:
            return False
        raise CaptureError("CONTAINER_DIRECTORY_MISSING", label)
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode):
        raise CaptureError("CONTAINER_DIRECTORY_INVALID", label)
    return True


def _container_files(path: Path, label: str, *, maximum_files: int = MAX_CAPTURE_FILES,
                     suffixes: tuple[str, ...] | None = None) -> list[Path]:
    """Enumerate active regular files without following any directory link."""
    if not _container_directory(path, label, allow_missing=True):
        return []
    result: list[Path] = []
    pending = [path]
    while pending:
        current = pending.pop()
        try:
            entries = sorted(os.scandir(str(current)), key=lambda item: item.name)
        except OSError as error:
            raise CaptureError("CONTAINER_DIRECTORY_READ_FAILED", label) from error
        for entry in entries:
            child = Path(entry.path)
            try:
                info = entry.stat(follow_symlinks=False)
            except OSError as error:
                raise CaptureError("CONTAINER_ENTRY_INVALID", str(child)) from error
            if stat.S_ISLNK(info.st_mode):
                raise CaptureError("CONTAINER_SYMLINK_REJECTED", str(child))
            if stat.S_ISDIR(info.st_mode):
                pending.append(child)
            elif stat.S_ISREG(info.st_mode):
                if info.st_nlink != 1 or stat.S_IMODE(info.st_mode) & 0o022:
                    raise CaptureError("CONTAINER_FILE_INVALID", str(child))
                if suffixes is None or child.name.endswith(suffixes):
                    result.append(child)
            else:
                raise CaptureError("CONTAINER_SPECIAL_FILE_REJECTED", str(child))
            if len(result) > maximum_files:
                raise CaptureError("CONTAINER_FILE_COUNT_EXCEEDED", label)
    return sorted(result, key=lambda item: str(item))


def _output_directory(root: Path, relative: str) -> Path:
    relative = _safe_rel(relative, "capture output relative path")
    try:
        # The container-side collector is never allowed to grow the bind
        # layout.  Every parent and the requested directory were created and
        # owned by the host before ``docker run``; this helper only reopens
        # them with no-follow identity checks.
        EVIDENCE_DIRECTORIES.snapshot_existing(root, (relative,))
    except EVIDENCE_DIRECTORIES.EvidenceDirectoryError as error:
        raise CaptureError(error.kind, str(error)) from error
    return root / relative


def _copy_capture_file(root: Path, relative: str, data: bytes,
                       maximum: int = MAX_CAPTURE_FILE_BYTES,
                       *, allow_empty: bool = False) -> dict[str, Any]:
    parent = Path(relative).parent.as_posix()
    if parent not in {"", "."}:
        _output_directory(root, parent)
    path = root / _safe_rel(relative, "capture file")
    return _write_new(path, data, maximum=maximum, allow_empty=allow_empty)


def _validate_apt_source_url_policy(value: Any, label: str = "APT URL policy") -> dict[str, Any]:
    if value != APT_SOURCE_URL_POLICY:
        raise CaptureError("APT_URL_POLICY_INVALID", label)
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


def _https_url(value: Any, label: str) -> str:
    if (not isinstance(value, str) or len(value) > 2048 or
            any(character in value for character in "\x00\r\n")):
        raise CaptureError("HTTPS_URL_INVALID", label)
    try:
        parts = urlsplit(value)
        if (parts.scheme != "https" or not parts.netloc or parts.query or
                parts.fragment or parts.username or parts.password):
            raise CaptureError("HTTPS_URL_INVALID", label)
    except ValueError as error:
        raise CaptureError("HTTPS_URL_INVALID", label) from error
    return value


def _capture_url(value: Any, label: str, *,
                 policy: Mapping[str, Any] | None = None) -> str:
    """Validate an APT/rosdep URL without rewriting the recorded source.

    HTTPS keeps the historical strict URL contract.  HTTP is accepted only
    for the profile-bound Ubuntu/ROS prefixes; credentials, non-default ports,
    queries, fragments, and prefix-confusion paths are never accepted.
    """
    if (not isinstance(value, str) or any(character in value for character in "\x00\r\n") or
            len(value) > 2048):
        raise CaptureError("APT_URL_INVALID", label)
    try:
        parts = urlsplit(value)
        if (not parts.netloc or parts.query or parts.fragment or parts.username or
                parts.password):
            raise CaptureError("APT_URL_INVALID", label)
        if parts.scheme == "https":
            return _https_url(value, label)
        selected = _validate_apt_source_url_policy(
            APT_SOURCE_URL_POLICY if policy is None else policy)
        if parts.scheme != "http" or not any(
                _http_url_matches_prefix(value, prefix)
                for prefix in selected["http_prefixes"]):
            raise CaptureError("APT_URL_INVALID", label)
    except ValueError as error:
        raise CaptureError("APT_URL_INVALID", label) from error
    return value


def _safe_source_atom(value: str, label: str, *, maximum: int = 256) -> str:
    if (not value or len(value) > maximum or any(
            ord(char) < 0x20 or ord(char) == 0x7f for char in value)):
        raise CaptureError("APT_SOURCE_VALUE_INVALID", label)
    return value


def _crc24(data: bytes) -> int:
    """Return the CRC-24 used by OpenPGP ASCII armor."""
    crc = 0xB704CE
    for octet in data:
        crc ^= octet << 16
        for _ in range(8):
            crc <<= 1
            if crc & 0x1000000:
                crc ^= 0x1864CFB
            crc &= 0xFFFFFF
    return crc


def _normalize_inline_armor(value: str, label: str) -> tuple[bytes, bytes]:
    """Normalize an inline public-key armor and return armor/raw packet bytes.

    Deb822 folds a multiline field by prefixing continuation lines with one
    space.  The special continuation line `` .`` represents an empty line;
    callers must remove that folding marker before this function is called.
    This parser deliberately performs only bounded, deterministic armor
    decoding.  Fingerprint and key validity decisions are made later by the
    pinned GnuPG inventory command, never by the base64 decoder.
    """
    if not isinstance(value, str) or len(value) > MAX_CAPTURE_FILE_BYTES:
        raise CaptureError("APT_INLINE_KEY_INVALID", label)
    try:
        value.encode("ascii")
    except UnicodeEncodeError as error:
        raise CaptureError("APT_INLINE_KEY_INVALID", label) from error
    lines = value.strip().splitlines()
    begin = "-----BEGIN PGP PUBLIC KEY BLOCK-----"
    end = "-----END PGP PUBLIC KEY BLOCK-----"
    if (len(lines) < 3 or lines[0] != begin or lines[-1] != end or
            any(line == begin or line == end for line in lines[1:-1])):
        raise CaptureError("APT_INLINE_KEY_INVALID", label)
    body = lines[1:-1]
    separator = None
    for index, line in enumerate(body):
        if line == "":
            separator = index
            break
        if re.fullmatch(r"[A-Za-z][A-Za-z0-9-]*: [ -~]+", line) is None:
            break
    payload_lines = body if separator is None else body[separator + 1:]
    if not payload_lines:
        raise CaptureError("APT_INLINE_KEY_INVALID", label)
    crc_line = None
    if any(line.startswith("=") for line in payload_lines):
        if not payload_lines[-1].startswith("=") or payload_lines[-1].count("=") != 1:
            raise CaptureError("APT_INLINE_KEY_INVALID", label)
        crc_line = payload_lines.pop()
        if re.fullmatch(r"=[A-Za-z0-9+/]{4}", crc_line) is None:
            raise CaptureError("APT_INLINE_KEY_INVALID", label)
    if any(not line or re.fullmatch(r"[A-Za-z0-9+/]+", line) is None
           for line in payload_lines):
        raise CaptureError("APT_INLINE_KEY_INVALID", label)
    encoded = "".join(payload_lines)
    try:
        dearmored = base64.b64decode(encoded, validate=True)
    except (ValueError, base64.binascii.Error) as error:
        raise CaptureError("APT_INLINE_KEY_INVALID", label) from error
    if not dearmored or len(dearmored) > MAX_CAPTURE_FILE_BYTES:
        raise CaptureError("APT_INLINE_KEY_INVALID", label)
    if crc_line is not None:
        try:
            observed_crc = base64.b64decode(crc_line[1:], validate=True)
        except (ValueError, base64.binascii.Error) as error:
            raise CaptureError("APT_INLINE_KEY_INVALID", label) from error
        if len(observed_crc) != 3 or int.from_bytes(observed_crc, "big") != _crc24(dearmored):
            raise CaptureError("APT_INLINE_KEY_CRC_INVALID", label)
    normalized = ("\n".join(lines) + "\n").encode("ascii")
    if len(normalized) > MAX_CAPTURE_FILE_BYTES:
        raise CaptureError("APT_INLINE_KEY_OVERSIZE", label)
    return normalized, dearmored


def _parse_signed_by(value: str, label: str) -> dict[str, Any]:
    """Parse a deb822 Signed-By value without conflating paths and armor."""
    value = value.strip()
    if value.startswith("-----BEGIN PGP PUBLIC KEY BLOCK-----"):
        data, dearmored = _normalize_inline_armor(value, label)
        digest = sha256_bytes(data)
        return {
            "kind": "inline", "source": "inline", "bytes": len(data),
            "sha256": digest,
            "normalized_armor_bytes": len(data),
            "normalized_armor_sha256": digest,
            "dearmored_bytes": len(dearmored),
            "dearmored_sha256": sha256_bytes(dearmored),
            "data": data, "dearmored_data": dearmored,
        }
    if "-----BEGIN PGP" in value or "-----END PGP" in value:
        raise CaptureError("APT_INLINE_KEY_INVALID", label)
    if not value.startswith("/") or Path(value).as_posix() != value or \
            any(part in ("", ".", "..") for part in value.split("/")[1:]):
        raise CaptureError("APT_KEYRING_PATH_INVALID", label)
    return {"kind": "path", "source": value}


def _parse_legacy_source_line(raw: str, label: str, *,
                              policy: Mapping[str, Any] | None) -> dict[str, Any] | None:
    try:
        tokens = shlex.split(raw, comments=True, posix=True)
    except ValueError as error:
        raise CaptureError("APT_SOURCE_LINE_INVALID", label) from error
    if not tokens:
        return None
    if tokens[0] not in {"deb", "deb-src"}:
        raise CaptureError("APT_SOURCE_LINE_INVALID", label)
    source_type = tokens[0]
    index = 1
    option_tokens: list[str] = []
    if index < len(tokens) and tokens[index].startswith("["):
        while index < len(tokens):
            option_tokens.append(tokens[index])
            if tokens[index].endswith("]"):
                break
            index += 1
        if not option_tokens or not option_tokens[-1].endswith("]"):
            raise CaptureError("APT_SOURCE_OPTIONS_INVALID", label)
        index += 1
        option_text = " ".join(option_tokens)
        if not option_text.startswith("[") or not option_text.endswith("]"):
            raise CaptureError("APT_SOURCE_OPTIONS_INVALID", label)
        option_text = option_text[1:-1].strip()
        option_values = option_text.split() if option_text else []
    elif index < len(tokens) and "]" in tokens[index]:
        raise CaptureError("APT_SOURCE_OPTIONS_INVALID", label)
    else:
        option_values = []
    if len(tokens) < index + 3:
        raise CaptureError("APT_SOURCE_LINE_INVALID", label)
    if any(not token for token in tokens[index:]):
        raise CaptureError("APT_SOURCE_LINE_INVALID", label)
    options: list[dict[str, str]] = []
    signed_by = {"kind": "none"}
    option_names: set[str] = set()
    for option in option_values:
        if "=" not in option:
            raise CaptureError("APT_SOURCE_OPTIONS_INVALID", label)
        name, value = option.split("=", 1)
        name = name.lower()
        _safe_source_atom(name, label + " option name", maximum=64)
        _safe_source_atom(value, label + " option value")
        if name in option_names:
            raise CaptureError("APT_SOURCE_OPTION_DUPLICATE", label)
        option_names.add(name)
        options.append({"name": name, "value": value})
        if name == "signed-by":
            signed_by = _parse_signed_by(value, label + " Signed-By")
    urls = [_capture_url(tokens[index], label + " URL", policy=policy)]
    suite = tokens[index + 1]
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", suite):
        raise CaptureError("APT_SOURCE_SUITE_INVALID", label)
    components = tokens[index + 2:]
    if any(not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+:-]{0,127}", item)
           for item in components):
        raise CaptureError("APT_SOURCE_COMPONENT_INVALID", label)
    return {"format": "legacy", "types": [source_type], "uris": urls,
            "suites": [suite], "components": components, "options": options,
            "signed_by": signed_by}


def _parse_deb822_stanza(fields: Mapping[str, str], label: str, *,
                         policy: Mapping[str, Any] | None) -> dict[str, Any]:
    required = {"types", "uris", "suites", "components"}
    if not required.issubset(fields):
        raise CaptureError("APT_SOURCE_STANZA_INVALID", label)
    types = fields["types"].split()
    if not types or any(item not in {"deb", "deb-src"} for item in types):
        raise CaptureError("APT_SOURCE_TYPE_INVALID", label)
    if len(types) != len(set(types)):
        raise CaptureError("APT_SOURCE_TYPE_DUPLICATE", label)
    uris = [_capture_url(item, label + " URI", policy=policy)
            for item in fields["uris"].split()]
    suites = fields["suites"].split()
    components = fields["components"].split()
    if not uris or not suites or not components:
        raise CaptureError("APT_SOURCE_STANZA_INVALID", label)
    if any(not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.-]{0,127}", item)
           for item in suites):
        raise CaptureError("APT_SOURCE_SUITE_INVALID", label)
    if any(not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9_.+:-]{0,127}", item)
           for item in components):
        raise CaptureError("APT_SOURCE_COMPONENT_INVALID", label)
    if len(uris) != len(set(uris)) or len(suites) != len(set(suites)):
        raise CaptureError("APT_SOURCE_STANZA_DUPLICATE", label)
    options: list[dict[str, str]] = []
    for key, value in fields.items():
        if key in required or key == "signed-by":
            continue
        if key not in DEB822_OPTION_FIELDS:
            raise CaptureError("APT_SOURCE_FIELD_UNKNOWN", label + " " + key)
        _safe_source_atom(value, label + " " + key, maximum=4096)
        options.append({"name": key, "value": value})
    signed_by = _parse_signed_by(fields["signed-by"], label + " Signed-By") \
        if "signed-by" in fields else {"kind": "none"}
    return {"format": "deb822", "types": types, "uris": uris, "suites": suites,
            "components": components, "options": options, "signed_by": signed_by}


def _source_urls(data: bytes, label: str, *,
                 policy: Mapping[str, Any] | None = None,
                 allow_inactive: bool = False
                 ) -> tuple[list[dict[str, Any]], list[str]]:
    """Parse all active legacy lines or deb822 stanzas without collapsing fields."""
    try:
        lines = data.decode("utf-8").splitlines()
    except UnicodeError as error:
        raise CaptureError("APT_SOURCE_NOT_UTF8", label) from error
    has_deb822 = any(line and not line.lstrip().startswith("#") and
                     not line.startswith((" ", "\t", "deb ", "deb-src ")) and
                     ":" in line for line in lines)
    records: list[dict[str, Any]] = []
    keyrings: list[str] = []
    if not has_deb822:
        for ordinal, raw in enumerate(lines):
            line = raw.strip()
            if not line or line.startswith("#"):
                continue
            record = _parse_legacy_source_line(raw, "{}:{}".format(label, ordinal + 1),
                                               policy=policy)
            if record is not None:
                record["ordinal"] = len(records)
                # Preserve the physical source location.  The ordinal is the
                # active-entry order; it is not sufficient to detect an
                # inserted comment or disabled line during reopen.
                record["line_start"] = ordinal + 1
                record["line_end"] = ordinal + 1
                records.append(record)
    else:
        current: dict[str, str] = {}
        current_key: str | None = None
        stanza_start: int | None = None
        stanza_end: int | None = None
        for ordinal, raw in enumerate(lines):
            if raw.lstrip().startswith("#"):
                continue
            if not raw.strip():
                if current:
                    record = _parse_deb822_stanza(
                        current, "{} stanza {}".format(label, len(records)), policy=policy)
                    record["ordinal"] = len(records)
                    record["line_start"] = stanza_start
                    record["line_end"] = stanza_end
                    records.append(record)
                    current = {}
                    current_key = None
                    stanza_start = None
                    stanza_end = None
                continue
            if raw[0].isspace():
                if current_key != "signed-by" or not current:
                    raise CaptureError("APT_SOURCE_CONTINUATION_INVALID", label)
                continuation = raw.lstrip()
                if not continuation:
                    raise CaptureError("APT_SOURCE_CONTINUATION_INVALID", label)
                # RFC822/deb822 uses a single dot on a continuation line to
                # represent an empty line in a multiline field.  Retaining
                # the dot turns otherwise valid OpenPGP armor into an invalid
                # key and makes the signed-by identity depend on the folding
                # syntax rather than the key bytes.
                current[current_key] += "\n" + ("" if continuation == "." else continuation)
                stanza_end = ordinal + 1
                continue
            if ":" not in raw:
                raise CaptureError("APT_SOURCE_STANZA_INVALID", "{}:{}".format(label, ordinal + 1))
            key, value = raw.split(":", 1)
            key = key.strip().lower()
            if not re.fullmatch(r"[a-z][a-z0-9-]{0,63}", key) or key in current:
                raise CaptureError("APT_SOURCE_FIELD_DUPLICATE", label)
            if stanza_start is None:
                stanza_start = ordinal + 1
            current[key] = value.strip()
            current_key = key
            stanza_end = ordinal + 1
        if current:
            record = _parse_deb822_stanza(
                current, "{} stanza {}".format(label, len(records)), policy=policy)
            record["ordinal"] = len(records)
            record["line_start"] = stanza_start
            record["line_end"] = stanza_end
            records.append(record)
    if not records and not allow_inactive:
        raise CaptureError("APT_SOURCE_URL_MISSING", label)
    for record in records:
        signed_by = record["signed_by"]
        if signed_by["kind"] == "path":
            keyrings.append(signed_by["source"])
    return records, sorted(set(keyrings))


def _repository_family(url: str) -> str:
    """Classify a repository for its source-record keyring policy."""
    parts = urlsplit(url)
    path = parts.path.rstrip("/")
    if parts.hostname in {"archive.ubuntu.com", "security.ubuntu.com"} and \
            path == "/ubuntu":
        return "ubuntu"
    if parts.hostname == "packages.ros.org" and path == "/ros2/ubuntu":
        return "ros"
    return "other"


def _resolve_source_keyring(record: Mapping[str, Any], label: str) -> dict[str, Any]:
    """Resolve exactly one keyring policy for one source record.

    Ubuntu's legacy ``sources.list`` entries normally omit ``Signed-By``;
    those entries are bound to the image's Ubuntu archive keyring.  ROS
    deb822 entries must carry their inline key material.  We never combine
    keyrings from unrelated repositories and never use a global fallback.
    """
    uris = record.get("uris")
    signed_by = record.get("signed_by")
    if not isinstance(uris, list) or not uris or not isinstance(signed_by, Mapping):
        raise CaptureError("APT_KEYRING_SOURCE_INVALID", label)
    families = {_repository_family(url) for url in uris}
    if len(families) != 1:
        raise CaptureError("APT_KEYRING_SOURCE_AMBIGUOUS", label)
    family = next(iter(families))
    resolved = dict(signed_by)
    if family == "ubuntu":
        if signed_by.get("kind") == "none":
            resolved = {"kind": "path", "source": UBUNTU_ARCHIVE_KEYRING}
        elif (signed_by.get("kind") != "path" or
              signed_by.get("source") != UBUNTU_ARCHIVE_KEYRING):
            raise CaptureError("APT_KEYRING_SOURCE_MISMATCH", label)
    elif family == "ros":
        if signed_by.get("kind") != "inline":
            raise CaptureError("APT_ROS_INLINE_KEY_REQUIRED", label)
    elif signed_by.get("kind") == "none":
        raise CaptureError("APT_KEYRING_MISSING", label)
    return resolved


def _container_logical_path(path: Path) -> str:
    return _safe_rel(str(path).lstrip("/"), "container logical path")


def _source_files(container_root: Path | None) -> list[tuple[Path, bytes, dict[str, Any]]]:
    """Read the active apt source files, including the one Humble alias."""
    paths: list[Path] = []
    main = _container_path(container_root, "/etc/apt/sources.list", "apt sources.list")
    try:
        main.lstat()
        paths.append(main)
    except OSError:
        pass
    directory = _container_path(container_root, "/etc/apt/sources.list.d", "apt source directory")
    if _container_directory(directory, "apt source directory", allow_missing=True):
        try:
            entries = sorted(os.scandir(str(directory)), key=lambda item: item.name)
        except OSError as error:
            raise CaptureError("APT_SOURCE_DIRECTORY_READ_FAILED", str(directory)) from error
        for entry in entries:
            child = Path(entry.path)
            if not (child.name.endswith(".list") or child.name.endswith(".sources")):
                continue
            paths.append(child)
    result = []
    seen: set[str] = set()
    for path in sorted(paths, key=lambda item: str(item)):
        logical = _container_logical_path(path if container_root in (None, Path("/"))
                                          else Path("/") / str(path.relative_to(container_root)))
        if logical in seen:
            raise CaptureError("APT_SOURCE_DUPLICATE", logical)
        seen.add(logical)
        allowed_target = None
        if logical == "etc/apt/sources.list.d/ros2.sources":
            allowed_target = "/usr/share/ros-apt-source/ros2.sources"
        data, descriptor = _container_read(path, "apt source " + logical,
                                            MAX_CAPTURE_FILE_BYTES,
                                            allow_empty=False,
                                            exact_symlink_target=allowed_target,
                                            symlink_target_path=(_container_path(
                                                container_root, allowed_target, "ros2 source target")
                                                if allowed_target else None),
                                            require_root=container_root in (None, Path("/")))
        descriptor["logical_path"] = logical
        result.append((path, data, descriptor))
    if not result:
        raise CaptureError("APT_SOURCE_EMPTY", "no active apt source files")
    return result


def _bounded_decoded_chunk(chunks: list[bytes], total: int, chunk: bytes,
                           compressed_size: int, label: str) -> int:
    """Append one decoder chunk while enforcing output and ratio limits."""
    total += len(chunk)
    if total > MAX_DECODED_PACKAGES_BYTES:
        raise CaptureError("PACKAGES_INDEX_INVALID", label)
    if total > max(1, compressed_size) * MAX_PACKAGES_EXPANSION_RATIO:
        raise CaptureError("PACKAGES_DECOMPRESSION_RATIO", label)
    if chunk:
        chunks.append(chunk)
    return total


def _xz_terminal_empty_stream_policy() -> Mapping[str, Any]:
    policy = PACKAGES_FORMAT_POLICY.get("terminal_empty_stream_policy")
    if not isinstance(policy, Mapping) or set(policy) != {
            "schema", "schema_version", "enabled", "max_streams",
            "compressed_bytes", "decoded_bytes", "padding", "binding"}:
        raise CaptureError("PACKAGES_FORMAT_POLICY_INVALID", "xz terminal stream policy")
    if (policy["schema"] != "registration-plugin-xz-terminal-empty-stream-policy-v1" or
            policy["schema_version"] != 1 or policy["enabled"] is not True or
            policy["max_streams"] != 2 or policy["compressed_bytes"] != 32 or
            policy["decoded_bytes"] != 0 or policy["padding"] != "forbidden" or
            policy["binding"] != "index-offset-bytes-sha256-decoded-bytes"):
        raise CaptureError("PACKAGES_FORMAT_POLICY_INVALID", "xz terminal stream policy")
    return policy


def _decode_xz_stream_details(
        data: bytes, label: str, *, allow_signed_empty: bool = False
        ) -> tuple[bytes, list[dict[str, Any]]]:
    """Decode one data stream and an optional exact empty terminal stream.

    Python's ``LZMADecompressor`` intentionally stops at the first XZ stream.
    We account for that boundary explicitly so a second stream cannot be
    mistaken for harmless padding.  The only profile-approved exception is
    the deterministic 32-byte empty terminal stream emitted by the pinned
    Ubuntu package index producer.  Every consumed stream is retained as a
    cryptographic descriptor for receipt/readback validation.
    """
    policy = _xz_terminal_empty_stream_policy()
    if not isinstance(data, bytes) or not data or len(data) > MAX_CAPTURE_FILE_BYTES:
        raise CaptureError("PACKAGES_INDEX_INVALID", label)
    stream_descriptors: list[dict[str, Any]] = []
    decoded_parts: list[bytes] = []
    offset = 0
    try:
        while offset < len(data):
            if len(stream_descriptors) >= policy["max_streams"]:
                raise CaptureError("PACKAGES_XZ_STREAM_COUNT_INVALID", label)
            stream_offset = offset
            decoder = lzma.LZMADecompressor(format=lzma.FORMAT_XZ)
            chunks: list[bytes] = []
            total = 0
            while offset < len(data) and not decoder.eof:
                block = data[offset:offset + 64 * 1024]
                if not block:
                    break
                offset += len(block)
                chunk = decoder.decompress(
                    block, max_length=MAX_DECODED_PACKAGES_BYTES - total + 1)
                total = _bounded_decoded_chunk(
                    chunks, total, chunk, len(data), label)
            if not decoder.eof:
                raise CaptureError("PACKAGES_DECOMPRESSION_TRUNCATED", label)
            unused = decoder.unused_data
            consumed_end = offset - len(unused)
            if consumed_end <= stream_offset:
                raise CaptureError("PACKAGES_TRAILING_DATA", label)
            offset = consumed_end
            compressed = data[stream_offset:consumed_end]
            if total > max(1, len(compressed)) * MAX_PACKAGES_EXPANSION_RATIO:
                raise CaptureError("PACKAGES_DECOMPRESSION_RATIO", label)
            stream_descriptors.append({
                "index": len(stream_descriptors),
                "compressed_offset": stream_offset,
                "compressed_bytes": len(compressed),
                "compressed_sha256": sha256_bytes(compressed),
                "decoded_bytes": total,
            })
            decoded_parts.append(b"".join(chunks))
            if (len(stream_descriptors) == 1 and total == 0 and
                    not allow_signed_empty):
                raise CaptureError("PACKAGES_XZ_FIRST_STREAM_EMPTY", label)
            if offset == len(data):
                break
            # Any bytes after the first stream are a second XZ stream (or
            # malformed data).  Parse it fully, then apply the exact policy.
            if len(stream_descriptors) == 2:
                raise CaptureError("PACKAGES_TRAILING_DATA", label)
        if not stream_descriptors:
            raise CaptureError("PACKAGES_XZ_FIRST_STREAM_EMPTY", label)
        if allow_signed_empty:
            policy = _signed_empty_packages_policy()
            first = stream_descriptors[0]
            if (len(stream_descriptors) != policy["xz_stream_count"] or
                    first["compressed_bytes"] != policy["xz_compressed_bytes"] or
                    first["decoded_bytes"] != policy["decoded_bytes"] or
                    len(data) != policy["xz_compressed_bytes"] or
                    first["compressed_offset"] != 0):
                raise CaptureError("PACKAGES_SIGNED_EMPTY_INVALID", label)
        elif stream_descriptors[0]["decoded_bytes"] <= 0:
            raise CaptureError("PACKAGES_XZ_FIRST_STREAM_EMPTY", label)
        if len(stream_descriptors) == 2:
            terminal = stream_descriptors[1]
            if (terminal["compressed_bytes"] != policy["compressed_bytes"] or
                    terminal["decoded_bytes"] != policy["decoded_bytes"]):
                raise CaptureError("PACKAGES_XZ_TERMINAL_STREAM_INVALID", label)
        return b"".join(decoded_parts), stream_descriptors
    except CaptureError:
        raise
    except (EOFError, lzma.LZMAError, ValueError) as error:
        raise CaptureError("PACKAGES_DECOMPRESSION_FAILED", label) from error


def _decode_xz_stream(data: bytes, label: str, *, allow_signed_empty: bool = False) -> bytes:
    """Compatibility wrapper returning only decoded bytes."""
    decoded, _ = _decode_xz_stream_details(
        data, label, allow_signed_empty=allow_signed_empty)
    return decoded


def _decode_gzip_stream(data: bytes, label: str) -> bytes:
    decoder = zlib.decompressobj(wbits=16 + zlib.MAX_WBITS)
    chunks: list[bytes] = []
    total = 0
    try:
        offset = 0
        while offset < len(data):
            block = data[offset:offset + 64 * 1024]
            offset += len(block)
            chunk = decoder.decompress(
                block, max_length=MAX_DECODED_PACKAGES_BYTES - total + 1)
            total = _bounded_decoded_chunk(
                chunks, total, chunk, len(data), label)
            if decoder.unconsumed_tail:
                raise CaptureError("PACKAGES_DECOMPRESSION_RATIO", label)
            if decoder.eof:
                if decoder.unused_data or offset != len(data):
                    raise CaptureError("PACKAGES_TRAILING_DATA", label)
                break
        if not decoder.eof:
            raise CaptureError("PACKAGES_DECOMPRESSION_TRUNCATED", label)
    except (zlib.error, ValueError) as error:
        raise CaptureError("PACKAGES_DECOMPRESSION_FAILED", label) from error
    return b"".join(chunks)


def _decode_packages_index(path: Path, data: bytes,
                           runner: Callable[[list[str], int], Any] | None = None,
                           *, expected_format: str | None = None,
                           allow_signed_empty: bool = False
                           ) -> tuple[bytes, str]:
    """Decode one signed Packages artifact without external codec fallback.

    ``runner`` remains in the signature for compatibility with the older test
    seam, but is intentionally never invoked.  In particular an apt-local
    ``Packages.lz4`` file is not evidence: the only accepted formats are the
    profile-bound xz, gzip, and plain forms.
    """
    del runner
    suffix = path.name.lower()
    if suffix.endswith(".xz"):
        compression = "xz"
    elif suffix.endswith(".gz"):
        compression = "gz"
    elif suffix.endswith(".lz4") or suffix.endswith(".bz2"):
        raise CaptureError("PACKAGES_FORMAT_UNSUPPORTED", str(path))
    else:
        compression = "plain"
    if expected_format is not None and compression != expected_format:
        raise CaptureError("PACKAGES_FORMAT_MISMATCH", str(path))
    if not isinstance(data, bytes) or not data or len(data) > MAX_CAPTURE_FILE_BYTES:
        raise CaptureError("PACKAGES_INDEX_INVALID", str(path))
    if compression == "xz":
        decoded = _decode_xz_stream(
            data, str(path), allow_signed_empty=allow_signed_empty)
    elif compression == "gz":
        decoded = _decode_gzip_stream(data, str(path))
    else:
        decoded = data
    if ((not decoded and not (allow_signed_empty and compression == "xz")) or
            len(decoded) > MAX_DECODED_PACKAGES_BYTES):
        raise CaptureError("PACKAGES_INDEX_INVALID", str(path))
    return decoded, compression


def _decode_packages_index_details(
        path: Path, data: bytes,
        runner: Callable[[list[str], int], Any] | None = None,
        *, expected_format: str | None = None,
        allow_signed_empty: bool = False
        ) -> tuple[bytes, str, list[dict[str, Any]]]:
    """Decode a Packages artifact and return its profile-bound stream records."""
    del runner
    suffix = path.name.lower()
    if suffix.endswith(".xz"):
        compression = "xz"
    elif suffix.endswith(".gz"):
        compression = "gz"
    elif suffix.endswith(".lz4") or suffix.endswith(".bz2"):
        raise CaptureError("PACKAGES_FORMAT_UNSUPPORTED", str(path))
    else:
        compression = "plain"
    if expected_format is not None and compression != expected_format:
        raise CaptureError("PACKAGES_FORMAT_MISMATCH", str(path))
    if not isinstance(data, bytes) or not data or len(data) > MAX_CAPTURE_FILE_BYTES:
        raise CaptureError("PACKAGES_INDEX_INVALID", str(path))
    if compression == "xz":
        decoded, streams = _decode_xz_stream_details(
            data, str(path), allow_signed_empty=allow_signed_empty)
    elif compression == "gz":
        decoded = _decode_gzip_stream(data, str(path))
        streams = []
    else:
        decoded, streams = data, []
    if ((not decoded and not (allow_signed_empty and compression == "xz")) or
            len(decoded) > MAX_DECODED_PACKAGES_BYTES):
        raise CaptureError("PACKAGES_INDEX_INVALID", str(path))
    return decoded, compression, streams


def _release_payload(data: bytes, label: str) -> bytes:
    try:
        text = data.decode("utf-8")
    except UnicodeError as error:
        raise CaptureError("RELEASE_NOT_UTF8", label) from error
    if not text.startswith("-----BEGIN PGP SIGNED MESSAGE-----"):
        return data
    lines = text.splitlines()
    try:
        blank = lines.index("")
    except ValueError as error:
        raise CaptureError("INRELEASE_FORMAT_INVALID", label) from error
    try:
        signature = next(index for index, line in enumerate(lines[blank + 1:], blank + 1)
                         if line == "-----BEGIN PGP SIGNATURE-----")
    except StopIteration as error:
        raise CaptureError("INRELEASE_SIGNATURE_MISSING", label) from error
    cleartext = []
    for line in lines[blank + 1:signature]:
        cleartext.append(line[1:] if line.startswith("-") else line)
    return ("\n".join(cleartext) + "\n").encode("utf-8")


def _release_declared_size(token: str, label: str) -> int:
    """Parse a Release size without accepting Python's permissive int grammar."""
    if not isinstance(token, str) or re.fullmatch(
            r"(?:0|[1-9][0-9]*)", token) is None:
        raise CaptureError("RELEASE_SHA256_SIZE_INVALID", label)
    limit_text = str(MAX_RELEASE_DECLARED_BYTES)
    if (len(token) > len(limit_text) or
            (len(token) == len(limit_text) and token > limit_text)):
        raise CaptureError("RELEASE_SHA256_SIZE_INVALID", label)
    try:
        size = int(token, 10)
    except (TypeError, ValueError) as error:
        raise CaptureError("RELEASE_SHA256_SIZE_INVALID", label) from error
    if size > MAX_RELEASE_DECLARED_BYTES:
        raise CaptureError("RELEASE_SHA256_SIZE_INVALID", label)
    return size


def _release_sha256_entries(data: bytes, label: str) -> dict[str, tuple[str, int]]:
    payload = _release_payload(data, label)
    try:
        lines = payload.decode("utf-8").splitlines()
    except UnicodeError as error:
        raise CaptureError("RELEASE_NOT_UTF8", label) from error
    starts = [index for index, line in enumerate(lines) if line == "SHA256:"]
    if len(starts) != 1:
        raise CaptureError("RELEASE_SHA256_SECTION_INVALID", label)
    records: dict[str, tuple[str, int]] = {}
    for ordinal, line in enumerate(lines[starts[0] + 1:], 1):
        if not line:
            continue
        if not line[0].isspace():
            break
        fields = line.split()
        if len(fields) != 3 or SHA_RE.fullmatch(fields[0]) is None:
            raise CaptureError("RELEASE_SHA256_ENTRY_INVALID", "{}:{}".format(label, ordinal))
        digest = fields[0].lower()
        size = _release_declared_size(fields[1], label)
        path = _safe_rel(fields[2], "Release index path")
        if path in records:
            raise CaptureError("RELEASE_SHA256_DUPLICATE", path)
        records[path] = (digest, size)
    if not records:
        raise CaptureError("RELEASE_SHA256_EMPTY", label)
    return records


def _packages_records(
        data: bytes, label: str, *, allow_empty: bool = False
        ) -> dict[tuple[str, str, str], dict[str, Any]]:
    try:
        text = data.decode("utf-8")
    except UnicodeError as error:
        raise CaptureError("PACKAGES_NOT_UTF8", label) from error
    result: dict[tuple[str, str, str], dict[str, Any]] = {}
    for ordinal, stanza in enumerate(re.split(r"\n\s*\n", text)):
        fields: dict[str, str] = {}
        required = {"Package", "Version", "Architecture", "Filename", "Size", "SHA256"}
        lines = stanza.splitlines()
        for line_index, line in enumerate(lines):
            if not line:
                continue
            if line[0].isspace():
                continue
            empty_multiline = False
            if ": " in line:
                key, value = line.split(": ", 1)
            elif (line.endswith(":") and line_index + 1 < len(lines) and
                  lines[line_index + 1][:1].isspace()):
                # Debian control permits an empty first value followed by one
                # or more continuation lines (for example Noble's
                # X-Cargo-Built-Using field).  Keep this exception narrow:
                # required identity fields may never use it and a bare empty
                # optional field without an immediate continuation is invalid.
                key, value = line[:-1], ""
                empty_multiline = True
            else:
                raise CaptureError("PACKAGES_FIELD_INVALID", "{}:{}".format(label, ordinal))
            if (key in fields or (not value and not empty_multiline) or
                    (empty_multiline and (
                        key in required or
                        re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9-]{0,63}", key) is None))):
                raise CaptureError("PACKAGES_FIELD_DUPLICATE", "{}:{}".format(label, ordinal))
            fields[key] = value
        if not fields:
            continue
        if set(fields) != required and not required.issubset(fields):
            raise CaptureError("PACKAGES_FIELDS_MISSING", "{}:{}".format(label, ordinal))
        name, version, architecture = (fields["Package"], fields["Version"],
                                       fields["Architecture"])
        if (not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+_.:-]{0,127}", name) or
                not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+_.:~+-]{0,255}", version) or
                not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+_.-]{0,31}", architecture)):
            raise CaptureError("PACKAGES_IDENTITY_INVALID", label)
        filename = _safe_rel(fields["Filename"], "Packages filename")
        if not filename.endswith(".deb"):
            raise CaptureError("PACKAGES_FILENAME_INVALID", label)
        try:
            size = int(fields["Size"], 10)
        except ValueError as error:
            raise CaptureError("PACKAGES_SIZE_INVALID", label) from error
        # This parses every record in a signed repository catalog, including
        # packages unrelated to the resolver result.  Keep the actual capture
        # bound in _parse_print_uri/_read_bounded; metadata declarations use
        # the independent signed-declaration ceiling.
        if size <= 0 or size > MAX_RELEASE_DECLARED_BYTES:
            raise CaptureError("PACKAGES_SIZE_INVALID", label)
        digest = fields["SHA256"]
        _sha(digest, "Packages SHA256")
        identity = (name, version, architecture)
        if identity in result:
            raise CaptureError("PACKAGES_IDENTITY_DUPLICATE", label)
        result[identity] = {"name": name, "version": version,
                            "architecture": architecture, "filename": filename,
                            "bytes": size, "sha256": digest}
    if not result and (not allow_empty or data != b""):
        raise CaptureError("PACKAGES_EMPTY", label)
    return result


def _parse_print_uri(line: str, label: str) -> dict[str, Any]:
    try:
        fields = shlex.split(line, posix=True)
    except ValueError as error:
        raise CaptureError("RESOLVER_QUOTING_INVALID", label) from error
    if len(fields) != 4:
        raise CaptureError("RESOLVER_LINE_INVALID", label)
    url, filename, size_text, locator = fields
    url = _capture_url(url, label + " URL")
    if not filename.endswith(".deb") or "/" in filename or "\\" in filename:
        raise CaptureError("RESOLVER_FILENAME_INVALID", label)
    try:
        size = int(size_text, 10)
    except ValueError as error:
        raise CaptureError("RESOLVER_SIZE_INVALID", label) from error
    if size <= 0 or size > MAX_CAPTURE_DEB_BYTES:
        raise CaptureError("RESOLVER_SIZE_INVALID", label)
    if locator.startswith("SHA256:"):
        algorithm, digest = "sha256", locator[7:]
        _sha(digest, label + " SHA256")
    elif locator.startswith("MD5Sum:"):
        algorithm, digest = "md5", locator[7:]
        if not re.fullmatch(r"[0-9a-f]{32}", digest):
            raise CaptureError("RESOLVER_MD5_INVALID", label)
    else:
        raise CaptureError("RESOLVER_DIGEST_INVALID", label)
    basename = unquote(urlsplit(url).path.rsplit("/", 1)[-1])
    if not basename.endswith(".deb") or "/" in basename or "\\" in basename:
        raise CaptureError("RESOLVER_URL_FILENAME_INVALID", label)
    return {"url": url, "filename": filename, "bytes": size,
            "url_filename": basename,
            "digest_algorithm": algorithm, "digest": digest}


def _resolver_cache_filename(identity: tuple[Any, Any, Any]) -> str:
    """Return apt's cache basename for one dpkg package identity."""
    name, version, architecture = identity
    if not all(isinstance(item, str) and item for item in identity):
        raise CaptureError("RESOLVER_CACHE_FILENAME_INVALID", "package identity")
    # Debian pool filenames omit the epoch while apt's cache name retains it
    # using lowercase percent encoding.  Other legal version punctuation is
    # retained verbatim by apt.
    result = "{}_{}_{}.deb".format(name, version.replace(":", "%3a"), architecture)
    if "/" in result or "\\" in result or not result.endswith(".deb"):
        raise CaptureError("RESOLVER_CACHE_FILENAME_INVALID", result)
    return result


def _canonical_resolver_uris(lines: list[str], role: str) -> list[dict[str, Any]]:
    """Validate an apt URI set, then canonicalize dependency-graph output order."""
    uris = [_parse_print_uri(line, "{} URI {}".format(role, index))
            for index, line in enumerate(lines) if line]
    if not uris or len({item["filename"] for item in uris}) != len(uris):
        raise CaptureError("RESOLVER_URI_SET_INVALID", role)
    return sorted(uris, key=lambda item: (item["url"], item["filename"]))


def _canonical_resolver_line(uri: Mapping[str, Any]) -> str:
    """Render one already-validated URI record for the canonical projection."""
    locator = "SHA256" if uri["digest_algorithm"] == "sha256" else "MD5Sum"
    return "{} {} {} {}:{}".format(
        shlex.quote(uri["url"]), shlex.quote(uri["filename"]), uri["bytes"],
        locator, uri["digest"])


def _apt_cache_debs(root: Path, expected: set[str], role: str) -> list[Path]:
    """Reopen the exact apt archive-cache layout and return only deb files."""
    _container_directory(root, role + " deb root")
    try:
        entries = {item.name: item for item in os.scandir(str(root))}
    except OSError as error:
        raise CaptureError("RESOLVER_DEB_SET_INVALID", role) from error
    if set(entries) != expected | {"lock", "partial"}:
        raise CaptureError("RESOLVER_DEB_SET_INVALID", role)
    lock = entries["lock"].stat(follow_symlinks=False)
    partial = entries["partial"].stat(follow_symlinks=False)
    if (not stat.S_ISREG(lock.st_mode) or lock.st_size != 0 or lock.st_nlink != 1 or
            stat.S_IMODE(lock.st_mode) & 0o022 or not stat.S_ISDIR(partial.st_mode) or
            stat.S_IMODE(partial.st_mode) != 0o700):
        raise CaptureError("RESOLVER_DEB_SET_INVALID", role)
    try:
        with os.scandir(entries["partial"].path) as partial_entries:
            if next(partial_entries, None) is not None:
                raise CaptureError("RESOLVER_DEB_SET_INVALID", role)
    except OSError as error:
        raise CaptureError("RESOLVER_DEB_SET_INVALID", role) from error
    children = _container_files(root, role + " deb root", suffixes=(".deb",))
    if {item.name for item in children} != expected or len(children) != len(expected):
        raise CaptureError("RESOLVER_DEB_SET_INVALID", role)
    return children


def _package_copyright(container_root: Path | None, package: str
                       ) -> tuple[bytes, dict[str, Any], Path, list[dict[str, Any]], str]:
    """Resolve a bounded Debian /usr/share/doc alias chain without escaping it."""
    if not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+_.:-]{0,127}", package):
        raise CaptureError("COPYRIGHT_PACKAGE_INVALID", package)
    doc_root = _container_path(container_root, "/usr/share/doc", "package doc root")
    _container_directory(doc_root, "package doc root")
    current = package
    seen = set()
    aliases = []
    for _ in range(8):
        if current in seen:
            raise CaptureError("COPYRIGHT_ALIAS_CYCLE", package)
        seen.add(current)
        directory = doc_root / current
        try:
            before = directory.lstat()
        except OSError as error:
            raise CaptureError("COPYRIGHT_DIRECTORY_MISSING", package) from error
        if stat.S_ISLNK(before.st_mode):
            target = os.readlink(directory)
            if (not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+_.:-]{0,127}", target) or
                    stat.S_IMODE(before.st_mode) != 0o777 or before.st_nlink != 1):
                raise CaptureError("COPYRIGHT_ALIAS_INVALID", package)
            after = directory.lstat()
            if ((before.st_dev, before.st_ino, before.st_size, before.st_nlink) !=
                    (after.st_dev, after.st_ino, after.st_size, after.st_nlink) or
                    os.readlink(directory) != target):
                raise CaptureError("COPYRIGHT_ALIAS_DRIFT", package)
            aliases.append({
                "path": str(directory), "target": target,
                "mode": stat.S_IMODE(before.st_mode), "uid": before.st_uid,
                "gid": before.st_gid, "nlink": before.st_nlink,
                "device": before.st_dev, "inode": before.st_ino,
            })
            current = target
            continue
        if (not stat.S_ISDIR(before.st_mode) or stat.S_IMODE(before.st_mode) & 0o022):
            raise CaptureError("COPYRIGHT_DIRECTORY_INVALID", package)
        resolved = directory / "copyright"
        try:
            data, descriptor = _container_read(
                resolved, "copyright " + package, MAX_CAPTURE_FILE_BYTES,
                allow_empty=False, require_root=container_root in (None, Path("/")))
            return data, descriptor, resolved, aliases, "PRESENT"
        except CaptureError as error:
            if error.kind != "CONTAINER_FILE_MISSING":
                raise
            inventory = _container_path(
                container_root, "/var/lib/dpkg/info/{}.list".format(package),
                "dpkg file inventory " + package)
            inventory_data, inventory_descriptor = _container_read(
                inventory, "dpkg file inventory " + package, MAX_CAPTURE_FILE_BYTES,
                allow_empty=False, require_root=container_root in (None, Path("/")))
            try:
                inventory_lines = inventory_data.decode("utf-8").splitlines()
            except UnicodeError as decode_error:
                raise CaptureError("COPYRIGHT_INVENTORY_NOT_UTF8", package) from decode_error
            if (not inventory_lines or len(inventory_lines) != len(set(inventory_lines)) or
                    any(not line.startswith("/") for line in inventory_lines)):
                raise CaptureError("COPYRIGHT_INVENTORY_INVALID", package)
            logical = "/usr/share/doc/{}/copyright".format(package)
            if logical in inventory_lines:
                raise CaptureError("COPYRIGHT_INVENTORY_DRIFT", package)
            return (inventory_data, inventory_descriptor, inventory, aliases,
                    "ABSENT_FROM_DPKG_FILE_INVENTORY")
    raise CaptureError("COPYRIGHT_ALIAS_DEPTH", package)


def _parse_gpgv_status(stdout: bytes, stderr: bytes, label: str) -> dict[str, Any]:
    if len(stdout) > MAX_LOG_BYTES or len(stderr) > MAX_LOG_BYTES:
        raise CaptureError("GPGV_LOG_OVERSIZE", label)
    try:
        lines = stdout.decode("utf-8").splitlines()
    except UnicodeError as error:
        raise CaptureError("GPGV_STATUS_NOT_UTF8", label) from error
    goodsig: list[dict[str, str]] = []
    validsig: list[dict[str, str]] = []
    seen_tags: set[str] = set()
    status_sequence: list[str] = []
    key_considered: dict[str, Any] | None = None
    verification_compliance_mode: int | None = None
    for line in lines:
        if not line:
            continue
        fields = line.split()
        if len(fields) < 2 or fields[0] != "[GNUPG:]":
            raise CaptureError("GPGV_STATUS_UNEXPECTED_OUTPUT", label)
        tag = fields[1]
        if tag not in GPGV_ALLOWED_STATUS:
            raise CaptureError("GPGV_STATUS_UNEXPECTED_TAG", tag)
        if tag in seen_tags:
            raise CaptureError("GPGV_STATUS_DUPLICATE", tag)
        if tag == "NEWSIG" and seen_tags:
            raise CaptureError("GPGV_STATUS_ORDER_INVALID", tag)
        if tag in {"KEY_CONSIDERED", "SIG_ID", "GOODSIG", "VALIDSIG",
                   "VERIFICATION_COMPLIANCE_MODE"} and \
                "NEWSIG" not in seen_tags:
            raise CaptureError("GPGV_STATUS_ORDER_INVALID", tag)
        if tag == "KEY_CONSIDERED" and (goodsig or validsig or "SIG_ID" in seen_tags):
            raise CaptureError("GPGV_STATUS_ORDER_INVALID", tag)
        if tag == "SIG_ID" and (goodsig or validsig):
            raise CaptureError("GPGV_STATUS_ORDER_INVALID", tag)
        if tag == "GOODSIG" and "VALIDSIG" in seen_tags:
            raise CaptureError("GPGV_STATUS_ORDER_INVALID", tag)
        if tag == "VALIDSIG" and "GOODSIG" not in seen_tags:
            raise CaptureError("GPGV_STATUS_ORDER_INVALID", tag)
        if tag == "VERIFICATION_COMPLIANCE_MODE" and "VALIDSIG" not in seen_tags:
            raise CaptureError("GPGV_STATUS_ORDER_INVALID", tag)
        if tag in {"KEY_CONSIDERED", "SIG_ID", "GOODSIG", "VALIDSIG"} and \
                "VERIFICATION_COMPLIANCE_MODE" in seen_tags:
            raise CaptureError("GPGV_STATUS_ORDER_INVALID", tag)
        seen_tags.add(tag)
        status_sequence.append(tag)
        if tag == "NEWSIG" and len(fields) != 2:
            raise CaptureError("GPGV_STATUS_FIELDS_INVALID", tag)
        if tag == "SIG_ID" and (len(fields) != 5 or not fields[2] or
                                 not re.fullmatch(r"[0-9A-Za-z+/=]+", fields[2]) or
                                 not re.fullmatch(r"[0-9]{4}-[0-9]{2}-[0-9]{2}", fields[3]) or
                                 not re.fullmatch(r"[0-9]+", fields[4])):
            raise CaptureError("GPGV_STATUS_FIELDS_INVALID", tag)
        if tag == "GOODSIG":
            if len(fields) < 4 or not re.fullmatch(r"[0-9A-Fa-f]{8,16}", fields[2]):
                raise CaptureError("GPGV_GOODSIG_INVALID", label)
            name = " ".join(fields[3:])
            if not name or any(ord(char) < 32 or ord(char) == 127 for char in name):
                raise CaptureError("GPGV_GOODSIG_INVALID", label)
            goodsig.append({"key_id": fields[2].upper(), "uid": name})
        elif tag == "VALIDSIG":
            if len(fields) not in {11, 12} or not FINGERPRINT_RE.fullmatch(fields[2]):
                raise CaptureError("GPGV_VALIDSIG_INVALID", label)
            if not re.fullmatch(r"[0-9]{4}-[0-9]{2}-[0-9]{2}", fields[3]) or \
                    any(not re.fullmatch(r"[0-9]+", fields[index])
                        for index in (4, 5)):
                raise CaptureError("GPGV_VALIDSIG_INVALID", label)
            if any(not re.fullmatch(r"[0-9]+", fields[index]) for index in (6, 7, 8, 9, 10)):
                raise CaptureError("GPGV_VALIDSIG_INVALID", label)
            entry = {"fingerprint": fields[2].upper(), "creation_date": fields[3],
                     "creation_timestamp": fields[4], "expire_timestamp": fields[5],
                     "version": fields[6], "reserved": fields[7],
                     "pubkey_algorithm": fields[8], "hash_algorithm": fields[9],
                     "signature_class": fields[10]}
            if len(fields) == 12:
                if not FINGERPRINT_RE.fullmatch(fields[11]):
                    raise CaptureError("GPGV_VALIDSIG_PRIMARY_INVALID", label)
                entry["primary_fingerprint"] = fields[11].upper()
            validsig.append(entry)
        elif tag == "KEY_CONSIDERED":
            if len(fields) != 4 or not FINGERPRINT_RE.fullmatch(fields[2]) or \
                    not re.fullmatch(r"(?:0|[1-9][0-9]*)", fields[3]):
                raise CaptureError("GPGV_KEY_CONSIDERED_INVALID", label)
            flags = int(fields[3], 10)
            if flags > GPGV_MAX_STATUS_DECIMAL:
                raise CaptureError("GPGV_KEY_CONSIDERED_INVALID", label)
            key_considered = {"fingerprint": fields[2].upper(), "flags": flags}
        elif tag == "VERIFICATION_COMPLIANCE_MODE":
            if len(fields) != 3 or not re.fullmatch(r"(?:0|[1-9][0-9]*)", fields[2]):
                raise CaptureError("GPGV_COMPLIANCE_MODE_INVALID", label)
            verification_compliance_mode = int(fields[2], 10)
            if verification_compliance_mode > GPGV_MAX_STATUS_DECIMAL:
                raise CaptureError("GPGV_COMPLIANCE_MODE_INVALID", label)
    if len(goodsig) != 1 or len(validsig) != 1:
        raise CaptureError("GPGV_SIGNER_MISSING", label)
    if goodsig[0]["key_id"] != validsig[0]["fingerprint"][-len(goodsig[0]["key_id"]):]:
        raise CaptureError("GPGV_SIGNER_MISMATCH", label)
    if key_considered is not None:
        expected = validsig[0].get("primary_fingerprint", validsig[0]["fingerprint"])
        if key_considered["fingerprint"] != expected:
            raise CaptureError("GPGV_KEY_CONSIDERED_MISMATCH", label)
    return {"goodsig": goodsig[0], "validsig": validsig[0],
            "status_tags": sorted(seen_tags), "status_sequence": status_sequence,
            "key_considered": key_considered,
            "verification_compliance_mode": verification_compliance_mode,
            "stderr_sha256": sha256_bytes(stderr)}


def _parse_gpg_inventory(stdout: bytes, stderr: bytes, label: str) -> dict[str, Any]:
    """Parse a bounded ``gpg --with-colons`` public-key inventory.

    ``gpgv`` treats every key in a supplied keyring as trusted and does not
    itself report key expiry/revocation policy.  The inventory is therefore a
    separate, machine-readable input to the signer check.  Unknown records or
    validity states are rejected instead of being silently ignored.
    """
    if len(stdout) > MAX_LOG_BYTES or len(stderr) > MAX_LOG_BYTES:
        raise CaptureError("GPG_KEY_INVENTORY_OVERSIZE", label)
    try:
        lines = stdout.decode("utf-8").splitlines()
    except UnicodeError as error:
        raise CaptureError("GPG_KEY_INVENTORY_NOT_UTF8", label) from error
    allowed_tags = {"pub", "sub", "fpr", "uid"}
    validities = {"u", "f", "q", "n", "m", "-"}
    primaries: list[str] = []
    subkeys: list[str] = []
    current: str | None = None
    expecting_fpr = False
    for line in lines:
        if not line:
            continue
        fields = line.split(":")
        tag = fields[0] if fields else ""
        if tag not in allowed_tags or len(fields) < 10:
            raise CaptureError("GPG_KEY_INVENTORY_UNEXPECTED_RECORD", label)
        if tag in {"pub", "sub"}:
            validity = fields[1]
            if validity not in validities:
                raise CaptureError("GPG_KEY_INVENTORY_KEY_INVALID", label)
            if not re.fullmatch(r"[0-9A-Fa-f]{8,16}", fields[4] or ""):
                raise CaptureError("GPG_KEY_INVENTORY_KEY_INVALID", label)
            current = tag
            expecting_fpr = True
        elif tag == "fpr":
            if not expecting_fpr or current is None or \
                    not FINGERPRINT_RE.fullmatch(fields[9] or ""):
                raise CaptureError("GPG_KEY_INVENTORY_FINGERPRINT_INVALID", label)
            fingerprint = fields[9].upper()
            if current == "pub":
                primaries.append(fingerprint)
            else:
                subkeys.append(fingerprint)
            expecting_fpr = False
        elif tag == "uid":
            # UID records are not identity anchors.  They are retained by
            # gpg for display only and may occur between a key and subkey.
            if any(ord(char) < 0x20 or ord(char) == 0x7f for char in fields[9]):
                raise CaptureError("GPG_KEY_INVENTORY_UID_INVALID", label)
    if expecting_fpr or not primaries:
        raise CaptureError("GPG_KEY_INVENTORY_FINGERPRINT_MISSING", label)
    if len(set(primaries)) != len(primaries) or len(set(subkeys)) != len(subkeys):
        raise CaptureError("GPG_KEY_INVENTORY_DUPLICATE", label)
    return {"primary_fingerprints": sorted(primaries),
            "subkey_fingerprints": sorted(subkeys),
            "stderr_sha256": sha256_bytes(stderr)}


def _write_command_diagnostics(root: Path, prefix: str, stdout: bytes,
                               stderr: bytes, *, identity: str | None = None,
                               identity_registry: dict[str, str] | None = None
                               ) -> dict[str, Any]:
    """Seal both streams of a bounded tool invocation before interpretation."""
    if identity is not None:
        token = _identity_token(identity, prefix + " identity")
        if prefix == "gpgv" or prefix.startswith("gpgv-"):
            prefix = "gpgv-" + token
        elif prefix.startswith("gpg-key-inventory-"):
            suffix = "-version" if prefix.endswith("-version") else ""
            prefix = "gpg-key-inventory-" + token + suffix
        else:
            # Identity-scoped callers should make their namespace explicit;
            # retaining the prefix here avoids changing unrelated phase-log
            # names while still binding the two output paths below.
            prefix = prefix + "-" + token
        _claim_identity_artifact(
            identity_registry, "logs/" + prefix + ".stdout", identity,
            prefix + " stdout")
        _claim_identity_artifact(
            identity_registry, "logs/" + prefix + ".stderr", identity,
            prefix + " stderr")
    _write_log(root, prefix, "stdout", stdout)
    _write_log(root, prefix, "stderr", stderr)
    return {
        "stdout": _relative_descriptor(
            root, _descriptor(root / "logs" / (prefix + ".stdout"),
                              prefix + " stdout", MAX_LOG_BYTES)),
        "stderr": _relative_descriptor(
            root, _descriptor(root / "logs" / (prefix + ".stderr"),
                              prefix + " stderr", MAX_LOG_BYTES)),
    }


def _gpg_inventory_argv(homedir: Path, keyring: Path) -> list[str]:
    """Build the sole supported GnuPG 2.2.27 public-key inventory command."""
    homedir = _safe_abs(str(homedir), "gpg inventory homedir")
    keyring = _safe_abs(str(keyring), "gpg inventory keyring")
    return [
        GPG_INVENTORY_EXECUTABLE, "--batch", "--no-options",
        "--no-default-keyring", "--no-auto-key-retrieve",
        "--no-auto-key-import", "--no-auto-key-locate", "--no-autostart",
        "--homedir", str(homedir), "--with-colons", "--with-fingerprint",
        "--with-subkey-fingerprint", "--import-options", "show-only",
        "--import", str(keyring),
    ]


def _gpg_inventory_parent(root: Path) -> Path:
    """Return the pre-created host-owned inventory work directory.

    The path is deliberately derived from the mounted evidence root.  The
    old implementation used a process-global ``/tmp`` parent, which made an
    inner container self-report a host inode that the outer validator could
    not safely reopen.  Only the fixed directory created by the host is now
    accepted.
    """
    root = _safe_abs(str(root), "gpg inventory evidence root")
    try:
        root_info = root.lstat()
    except OSError as error:
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID", str(root)) from error
    if stat.S_ISLNK(root_info.st_mode) or not stat.S_ISDIR(root_info.st_mode):
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID", str(root))
    parent = root / GPG_INVENTORY_WORK_RELATIVE
    _parents_no_symlink(parent, "gpg inventory work directory")
    try:
        info = parent.lstat()
    except OSError as error:
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID", str(parent)) from error
    if (stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or
            stat.S_IMODE(info.st_mode) != 0o700 or info.st_nlink < 2):
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID", str(parent))
    return parent


def _gpg_directory_identity(info: os.stat_result) -> dict[str, int]:
    return {
        "device": int(info.st_dev), "inode": int(info.st_ino),
        "uid": int(info.st_uid), "gid": int(info.st_gid),
        "mode": int(stat.S_IMODE(info.st_mode)), "nlink": int(info.st_nlink),
    }


def _gpg_parent_matches(info: os.stat_result, expected: Mapping[str, Any],
                        *, child_present: bool) -> bool:
    if (stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or
            stat.S_IMODE(info.st_mode) != expected["mode"] or
            info.st_dev != expected["device"] or info.st_ino != expected["inode"] or
            info.st_uid != expected["uid"] or info.st_gid != expected["gid"]):
        return False
    expected_nlink = expected["nlink"] + (1 if child_present else 0)
    return info.st_nlink == expected_nlink


def _gpg_inventory_parent_fd(parent: Path, label: str) -> int:
    flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | \
        getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
    try:
        fd = os.open(str(parent), flags)
        info = os.fstat(fd)
        if (stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or
                stat.S_IMODE(info.st_mode) != 0o700 or info.st_nlink < 2):
            os.close(fd)
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID", label)
        return fd
    except CaptureError:
        raise
    except OSError as error:
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID", label) from error


def _remove_created_gpg_child(parent_fd: int, name: str,
                              created: os.stat_result) -> None:
    """Best-effort cleanup of only a just-created, still-empty child inode."""
    child_fd = None
    try:
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | \
            getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
        child_fd = os.open(name, flags, dir_fd=parent_fd)
        info = os.fstat(child_fd)
        if (info.st_dev, info.st_ino) != (created.st_dev, created.st_ino) or \
                list(os.listdir(child_fd)):
            return
        os.close(child_fd)
        child_fd = None
        os.rmdir(name, dir_fd=parent_fd)
        os.fsync(parent_fd)
    except (OSError, CaptureError):
        return
    finally:
        if child_fd is not None:
            try:
                os.close(child_fd)
            except OSError:
                pass


def _gpg_inventory_homedir(root: Path, ordinal: int,
                           *, identity: str | None = None,
                           identity_registry: dict[str, str] | None = None
                           ) -> tuple[Path, dict[str, Any]]:
    """Create one fresh GNUPGHOME below the host-owned evidence bind.

    The parent is opened no-follow and must be empty before creation.  Its
    identity is retained only as an internal runtime claim; the receipt binds
    the logical parent path and the outer directory contract binds the host
    device/inode/owner/mode before and after the container phase.
    """
    if not isinstance(ordinal, int) or ordinal < 0:
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_INVALID", str(ordinal))
    parent = _gpg_inventory_parent(root)
    if identity is None:
        token = sha256_bytes((str(root) + "\x00" + str(ordinal)).encode("utf-8"))[:24]
    else:
        token = _identity_token(identity, "gpg inventory identity")
    name = GPG_INVENTORY_HOMEDIR_PREFIX + token
    path = parent / name
    _safe_abs(str(path), "gpg inventory homedir")
    if identity is not None:
        _claim_identity_artifact(
            identity_registry, GPG_INVENTORY_WORK_RELATIVE + "/" + name,
            identity, "gpg inventory homedir")
    parent_fd = _gpg_inventory_parent_fd(parent, str(parent))
    created = None
    try:
        parent_info = os.fstat(parent_fd)
        parent_identity = _gpg_directory_identity(parent_info)
        try:
            os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            pass
        except OSError as error:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_COLLISION", str(path)) from error
        else:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_COLLISION", str(path))
        try:
            names = os.listdir(parent_fd)
        except OSError as error:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID", str(parent)) from error
        if names:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_NOT_EMPTY", str(parent))
        try:
            os.mkdir(name, 0o700, dir_fd=parent_fd)
            created = os.stat(name, dir_fd=parent_fd, follow_symlinks=False)
            os.fsync(parent_fd)
        except OSError as error:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_CREATE_FAILED", str(path)) from error
        try:
            child_flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | \
                getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
            child_fd = os.open(name, child_flags, dir_fd=parent_fd)
            try:
                info = os.fstat(child_fd)
                child_names = os.listdir(child_fd)
            finally:
                os.close(child_fd)
        except OSError as error:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_INVALID", str(path)) from error
        created = info
        if (stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or
                stat.S_IMODE(info.st_mode) != 0o700 or
                info.st_uid != os.getuid() or info.st_gid != os.getgid() or
                info.st_nlink < 2 or child_names):
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_INVALID", str(path))
        parent_after = os.fstat(parent_fd)
        if not _gpg_parent_matches(parent_after, parent_identity, child_present=True):
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_INVALID", str(parent))
        return path, {
            "path": str(path), "status": "FRESH", "mode": stat.S_IMODE(info.st_mode),
            "uid": info.st_uid, "gid": info.st_gid, "nlink": info.st_nlink,
            "device": info.st_dev, "inode": info.st_ino,
            "parent_path": str(parent), "parent_relative": GPG_INVENTORY_WORK_RELATIVE,
            "post_absent": False, "cleanup_status": "PENDING",
            "_parent_identity": parent_identity,
        }
    except Exception:
        if created is not None:
            _remove_created_gpg_child(parent_fd, name, created)
        raise
    finally:
        os.close(parent_fd)


def _public_gpg_inventory_homedir(identity: Mapping[str, Any]) -> dict[str, Any]:
    fields = ("path", "status", "mode", "uid", "gid", "nlink", "device", "inode",
              "parent_path", "parent_relative", "post_absent", "cleanup_status")
    return {field: identity[field] for field in fields}


def _assert_gpg_inventory_homedir(path: Path, identity: Mapping[str, Any],
                                  *, empty: bool = False) -> None:
    """Check child and host-bind parent identity without container /tmp state."""
    parent = _safe_abs(identity.get("parent_path"), "gpg inventory parent")
    if (identity.get("parent_relative") != GPG_INVENTORY_WORK_RELATIVE or
            path.parent != parent or path.name.startswith(GPG_INVENTORY_HOMEDIR_PREFIX) is False or
            not re.fullmatch(re.escape(GPG_INVENTORY_HOMEDIR_PREFIX) + r"[a-f0-9]{24}",
                             path.name)):
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_CHANGED", str(path))
    parent_fd = _gpg_inventory_parent_fd(parent, str(parent))
    child_fd = None
    try:
        parent_info = os.fstat(parent_fd)
        parent_identity = identity.get("_parent_identity")
        if not isinstance(parent_identity, Mapping) or \
                not _gpg_parent_matches(parent_info, parent_identity, child_present=True):
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_CHANGED", str(path))
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | \
            getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
        try:
            child_fd = os.open(path.name, flags, dir_fd=parent_fd)
        except OSError as error:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_CHANGED", str(path)) from error
        info = os.fstat(child_fd)
        if (stat.S_ISLNK(info.st_mode) or not stat.S_ISDIR(info.st_mode) or
                info.st_dev != identity["device"] or info.st_ino != identity["inode"] or
                stat.S_IMODE(info.st_mode) != identity["mode"] or
                info.st_uid != identity["uid"] or info.st_gid != identity["gid"] or
                info.st_nlink != identity["nlink"]):
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_CHANGED", str(path))
        if empty:
            try:
                entries = os.listdir(child_fd)
            except OSError as error:
                raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_READ_FAILED", str(path)) from error
            if entries:
                raise CaptureError("GPG_KEY_INVENTORY_RESIDUE", str(path))
    finally:
        if child_fd is not None:
            os.close(child_fd)
        os.close(parent_fd)


def _gpg_metadata_type(info: os.stat_result) -> str:
    """Return a stable type label without following a metadata entry."""
    if stat.S_ISREG(info.st_mode):
        return "regular"
    if stat.S_ISLNK(info.st_mode):
        return "symlink"
    if stat.S_ISSOCK(info.st_mode):
        return "socket"
    if stat.S_ISFIFO(info.st_mode):
        return "fifo"
    if stat.S_ISDIR(info.st_mode):
        return "directory"
    if stat.S_ISBLK(info.st_mode):
        return "block"
    if stat.S_ISCHR(info.st_mode):
        return "char"
    return "other"


def _gpg_metadata_xattrs(path: Path) -> tuple[list[str] | None, str | None]:
    """Read only xattr names; an unavailable interface is a rejection."""
    try:
        values = os.listxattr(str(path), follow_symlinks=False)
    except (AttributeError, OSError, TypeError):
        return None, "xattrs_unreadable"
    return sorted(values), None


def _gpg_metadata_entry(child_fd: int, path: Path, name: str,
                        identity: Mapping[str, Any]) -> dict[str, Any]:
    """Capture one GnuPG metadata entry through a no-follow directory fd."""
    try:
        info = os.stat(name, dir_fd=child_fd, follow_symlinks=False)
    except OSError:
        return {
            "name": name, "type": "other", "mode": 0, "uid": 0, "gid": 0,
            "nlink": 0, "size": 0, "device": 0, "inode": 0, "sha256": None,
            "xattrs": None, "status": "REJECTED", "reason": "stat_failed",
        }
    entry = {
        "name": name, "type": _gpg_metadata_type(info),
        "mode": int(stat.S_IMODE(info.st_mode)), "uid": int(info.st_uid),
        "gid": int(info.st_gid), "nlink": int(info.st_nlink),
        "size": int(info.st_size), "device": int(info.st_dev),
        "inode": int(info.st_ino), "sha256": None, "xattrs": None,
        "status": "REJECTED", "reason": "unexpected_name",
    }
    xattrs, xattr_error = _gpg_metadata_xattrs(path)
    entry["xattrs"] = xattrs
    reason: str | None = xattr_error
    expected_identity = (info.st_dev, info.st_ino, info.st_size, info.st_nlink,
                         info.st_uid, info.st_gid, stat.S_IMODE(info.st_mode))
    if info.st_mode and stat.S_ISREG(info.st_mode) and reason is None:
        if info.st_size <= 0 or info.st_size > GPG_INVENTORY_METADATA_MAX_BYTES:
            reason = "size_invalid"
        else:
            fd = None
            digest = hashlib.sha256()
            try:
                fd = os.open(name, os.O_RDONLY | getattr(os, "O_NOFOLLOW", 0) |
                             getattr(os, "O_CLOEXEC", 0), dir_fd=child_fd)
                before = os.fstat(fd)
                observed_identity = (before.st_dev, before.st_ino, before.st_size,
                                     before.st_nlink, before.st_uid, before.st_gid,
                                     stat.S_IMODE(before.st_mode))
                if observed_identity != expected_identity:
                    reason = "metadata_race"
                else:
                    total = 0
                    while True:
                        block = os.read(fd, min(1024 * 1024,
                                                GPG_INVENTORY_METADATA_MAX_BYTES - total + 1))
                        if not block:
                            break
                        total += len(block)
                        if total > GPG_INVENTORY_METADATA_MAX_BYTES:
                            reason = "size_invalid"
                            break
                        digest.update(block)
                    after = os.fstat(fd)
                    observed_after = (after.st_dev, after.st_ino, after.st_size,
                                      after.st_nlink, after.st_uid, after.st_gid,
                                      stat.S_IMODE(after.st_mode))
                    if reason is None and (total != info.st_size or
                                           observed_after != expected_identity):
                        reason = "metadata_race"
                    elif reason is None:
                        entry["sha256"] = digest.hexdigest()
            except OSError:
                reason = "read_failed"
            finally:
                if fd is not None:
                    os.close(fd)
            try:
                after_path = os.stat(name, dir_fd=child_fd, follow_symlinks=False)
                after_identity = (after_path.st_dev, after_path.st_ino,
                                  after_path.st_size, after_path.st_nlink,
                                  after_path.st_uid, after_path.st_gid,
                                  stat.S_IMODE(after_path.st_mode))
                if after_identity != expected_identity:
                    reason = "metadata_race"
            except OSError:
                reason = "metadata_race"
    elif reason is None:
        reason = "type_invalid"
    if reason is None and name not in GPG_INVENTORY_METADATA_NAMES:
        reason = "unexpected_name"
    if reason is None and entry["mode"] != GPG_INVENTORY_METADATA_MODE:
        reason = "mode_invalid"
    if reason is None and (entry["uid"] != identity["uid"] or
                           entry["gid"] != identity["gid"]):
        reason = "owner_invalid"
    if reason is None and entry["nlink"] != 1:
        reason = "nlink_invalid"
    if reason is None and entry["xattrs"] != []:
        reason = "xattrs_invalid"
    if reason is None and entry["sha256"] is None:
        reason = "hash_missing"
    if reason is None:
        entry["status"] = "ACCEPTED"
        entry["reason"] = "accepted"
    else:
        entry["status"] = "REJECTED"
        entry["reason"] = reason
    return entry


def _gpg_metadata_record(creation_evidence: Mapping[str, Any] | None,
                         entries: list[dict[str, Any]],
                         *, failure_reason: str | None = None,
                         race: bool = False) -> dict[str, Any]:
    names = [entry["name"] for entry in entries]
    if race:
        status = "REJECTED"
        failure_reason = failure_reason or "metadata_race"
    elif not entries:
        status = "EMPTY"
        failure_reason = None
    elif (names == list(GPG_INVENTORY_METADATA_NAMES) and
          all(entry["status"] == "ACCEPTED" for entry in entries)):
        status = "EXACT"
        failure_reason = None
    else:
        status = "REJECTED"
        failure_reason = failure_reason or next(
            (entry["reason"] for entry in entries
             if entry["status"] != "ACCEPTED"), "unexpected_name")
    return {
        "schema": GPG_INVENTORY_METADATA_SCHEMA,
        "schema_version": 1,
        "policy": GPG_INVENTORY_METADATA_POLICY,
        "expected_names": list(GPG_INVENTORY_METADATA_NAMES),
        "entries": entries,
        "status": status,
        "failure_reason": failure_reason,
        "creation_evidence": (dict(creation_evidence)
                               if creation_evidence is not None else None),
        "retention": GPG_INVENTORY_METADATA_RETENTION,
    }


def _capture_gpg_inventory_metadata(path: Path, identity: Mapping[str, Any],
                                    creation_evidence: Mapping[str, Any] | None
                                    ) -> dict[str, Any]:
    """Snapshot child metadata before cleanup, preserving race evidence."""
    parent = _safe_abs(identity.get("parent_path"), "gpg inventory parent")
    parent_fd = _gpg_inventory_parent_fd(parent, str(parent))
    child_fd = None
    try:
        parent_identity = identity.get("_parent_identity")
        parent_info = os.fstat(parent_fd)
        if (not isinstance(parent_identity, Mapping) or
                not _gpg_parent_matches(parent_info, parent_identity,
                                        child_present=True)):
            return _gpg_metadata_record(creation_evidence, [],
                                        failure_reason="metadata_race", race=True)
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | \
            getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
        try:
            child_fd = os.open(path.name, flags, dir_fd=parent_fd)
            child_info = os.fstat(child_fd)
        except OSError:
            return _gpg_metadata_record(creation_evidence, [],
                                        failure_reason="child_missing", race=True)
        expected_child = (identity["device"], identity["inode"], identity["mode"],
                          identity["uid"], identity["gid"], identity["nlink"])
        observed_child = (child_info.st_dev, child_info.st_ino,
                          stat.S_IMODE(child_info.st_mode), child_info.st_uid,
                          child_info.st_gid, child_info.st_nlink)
        if (not stat.S_ISDIR(child_info.st_mode) or
                observed_child != expected_child):
            return _gpg_metadata_record(creation_evidence, [],
                                        failure_reason="metadata_race", race=True)
        try:
            names_before = sorted(os.listdir(child_fd))
        except OSError:
            return _gpg_metadata_record(creation_evidence, [],
                                        failure_reason="list_failed", race=True)
        too_many = len(names_before) > 32
        names_to_scan = names_before[:32]
        entries = [_gpg_metadata_entry(child_fd, path / name, name, identity)
                   for name in names_to_scan]
        try:
            names_after = sorted(os.listdir(child_fd))
            child_after = os.fstat(child_fd)
            parent_after = os.fstat(parent_fd)
        except OSError:
            return _gpg_metadata_record(creation_evidence, entries,
                                        failure_reason="metadata_race", race=True)
        if (names_before != names_after or
                (child_after.st_dev, child_after.st_ino,
                 stat.S_IMODE(child_after.st_mode), child_after.st_uid,
                 child_after.st_gid, child_after.st_nlink) != observed_child or
                not _gpg_parent_matches(parent_after, parent_identity,
                                        child_present=True)):
            return _gpg_metadata_record(creation_evidence, entries,
                                        failure_reason="metadata_race", race=True)
        return _gpg_metadata_record(
            creation_evidence, entries,
            failure_reason="too_many_entries" if too_many else None)
    finally:
        if child_fd is not None:
            os.close(child_fd)
        os.close(parent_fd)


def _remove_gpg_inventory_tree(path: Path, identity: Mapping[str, Any]) -> None:
    """Remove only verified GnuPG metadata and prove the bind is empty."""
    _assert_gpg_inventory_homedir(path, identity)
    parent = _safe_abs(identity.get("parent_path"), "gpg inventory parent")
    parent_fd = _gpg_inventory_parent_fd(parent, str(parent))
    child_fd = None
    try:
        parent_identity = identity.get("_parent_identity")
        parent_info = os.fstat(parent_fd)
        if not isinstance(parent_identity, Mapping) or \
                not _gpg_parent_matches(parent_info, parent_identity, child_present=True):
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_CHANGED", str(path))
        flags = os.O_RDONLY | getattr(os, "O_DIRECTORY", 0) | \
            getattr(os, "O_NOFOLLOW", 0) | getattr(os, "O_CLOEXEC", 0)
        try:
            child_fd = os.open(path.name, flags, dir_fd=parent_fd)
        except OSError as error:
            raise CaptureError("GPG_KEY_INVENTORY_CLEANUP_FAILED", str(path)) from error
        child_info = os.fstat(child_fd)
        if (child_info.st_dev, child_info.st_ino) != (identity["device"], identity["inode"]):
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_CHANGED", str(path))
        expected_metadata = identity.get("_metadata_residue")
        if expected_metadata is not None:
            current_metadata = _capture_gpg_inventory_metadata(
                path, identity, expected_metadata.get("creation_evidence"))
            if current_metadata != expected_metadata:
                raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_CHANGED", str(path))
            if current_metadata["status"] == "REJECTED":
                raise CaptureError("GPG_KEY_INVENTORY_RESIDUE", str(path))
            names = [entry["name"] for entry in current_metadata["entries"]]
            expected_entries = {
                entry["name"]: entry for entry in current_metadata["entries"]
            }
        else:
            try:
                names = sorted(os.listdir(child_fd))
            except OSError as error:
                raise CaptureError("GPG_KEY_INVENTORY_CLEANUP_FAILED", str(path)) from error
            if names:
                raise CaptureError("GPG_KEY_INVENTORY_RESIDUE", str(path))
            expected_entries = {}
        for name in sorted(names, reverse=True):
            expected = expected_entries.get(name)
            if expected is None or name not in GPG_INVENTORY_METADATA_NAMES:
                raise CaptureError("GPG_KEY_INVENTORY_RESIDUE", name)
            try:
                info = os.stat(name, dir_fd=child_fd, follow_symlinks=False)
            except OSError as error:
                raise CaptureError("GPG_KEY_INVENTORY_CLEANUP_FAILED", name) from error
            if (not stat.S_ISREG(info.st_mode) or info.st_nlink != 1 or
                    (info.st_dev, info.st_ino, info.st_size, info.st_uid, info.st_gid,
                     stat.S_IMODE(info.st_mode)) !=
                    (expected["device"], expected["inode"], expected["size"],
                     expected["uid"], expected["gid"], expected["mode"])):
                raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_CHANGED", name)
            try:
                os.unlink(name, dir_fd=child_fd)
            except OSError as error:
                raise CaptureError("GPG_KEY_INVENTORY_CLEANUP_FAILED", name) from error
        try:
            os.fsync(child_fd)
        except OSError as error:
            raise CaptureError("GPG_KEY_INVENTORY_CLEANUP_FAILED", str(path)) from error
        if os.listdir(child_fd):
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE", str(path))
        os.close(child_fd)
        child_fd = None
        try:
            os.rmdir(path.name, dir_fd=parent_fd)
            os.fsync(parent_fd)
        except OSError as error:
            raise CaptureError("GPG_KEY_INVENTORY_CLEANUP_FAILED", str(path)) from error
        parent_after = os.fstat(parent_fd)
        if not _gpg_parent_matches(parent_after, parent_identity, child_present=False):
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_CHANGED", str(parent))
        try:
            os.stat(path.name, dir_fd=parent_fd, follow_symlinks=False)
        except FileNotFoundError:
            pass
        except OSError as error:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_NOT_ABSENT", str(path)) from error
        else:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_NOT_ABSENT", str(path))
        if os.listdir(parent_fd):
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_PARENT_NOT_EMPTY", str(parent))
    finally:
        if child_fd is not None:
            os.close(child_fd)
        os.close(parent_fd)


def _gpg_inventory_tool(root: Path, container_root: Path | None, ordinal: int,
                        runner: Callable[[list[str], int], Any],
                        homedir: Path, *, identity: str | None = None,
                        identity_registry: dict[str, str] | None = None
                        ) -> dict[str, Any]:
    """Capture the pinned gpg binary and its exact version invocation."""
    binary_path = _container_path(container_root, GPG_INVENTORY_EXECUTABLE,
                                  "gpg inventory executable")
    binary_data, _ = _container_read(
        binary_path, "gpg inventory executable", MAX_CAPTURE_FILE_BYTES,
        allow_empty=False, require_root=container_root in (None, Path("/")))
    binary_sha = sha256_bytes(binary_data)
    token = (_identity_token(identity, "gpg inventory identity")
             if identity is not None else str(ordinal))
    binary_destination = "{}/gpg-inventory-{}-{}.bin".format(
        APT_KEYRING_RELATIVE, token, binary_sha[:16])
    if identity is not None:
        _claim_identity_artifact(
            identity_registry, binary_destination, identity,
            "gpg inventory binary")
    binary_output = _copy_capture_file(root, binary_destination, binary_data)
    version_argv = [GPG_INVENTORY_EXECUTABLE, "--version"]
    _validate_argv(version_argv, "gpg inventory version")
    version_result = _normalize_output(runner(version_argv, 30))
    version_diagnostics = _write_command_diagnostics(
        root, "gpg-key-inventory-{}-version".format(ordinal),
        version_result[1], version_result[2], identity=identity,
        identity_registry=identity_registry)
    version_lines = []
    try:
        version_lines = version_result[1].decode("utf-8").splitlines()
    except UnicodeError as error:
        raise CaptureError("GPG_KEY_INVENTORY_VERSION_INVALID", str(binary_path)) from error
    if (version_result[0] != 0 or version_result[3] or version_result[4] is not None or
            not version_lines or version_lines[0] not in GPG_INVENTORY_VERSIONS or
            len(version_lines[0]) > 128):
        raise CaptureError("GPG_KEY_INVENTORY_VERSION_INVALID", str(binary_path))
    return {
        "name": "gpg", "path": GPG_INVENTORY_EXECUTABLE,
        "sha256": binary_sha, "bytes": len(binary_data),
        "version": version_lines[0], "version_command": version_argv,
        "version_command_sha256": _command_hash(version_argv),
        "version_exit_status": version_result[0],
        "version_timed_out": version_result[3], "version_signal": version_result[4],
        "version_stdout": version_diagnostics["stdout"],
        "version_stderr": version_diagnostics["stderr"],
        "binary_output": _relative_descriptor(root, binary_output),
        "homedir": str(homedir),
    }


def _key_inventory(runner: Callable[[list[str], int], Any], root: Path,
                   descriptor: dict[str, Any], ordinal: int,
                   container_root: Path | None = None, *, identity: str | None = None,
                   identity_registry: dict[str, str] | None = None) -> None:
    """Populate a key descriptor from one fresh, isolated GnuPG inventory."""
    inventory_path = str(root / descriptor["dearmored_path"])
    inventory_identity = identity or canonical_hash({
        "domain": "registration-plugin-gpg-keyring-direct-inventory-v1",
        "descriptor": _immutable_copy(descriptor),
    })
    homedir: Path | None = None
    homedir_identity: dict[str, Any] | None = None
    argv: list[str] = []
    version_tool: dict[str, Any] | None = None
    diagnostics: dict[str, Any] | None = None
    version_diagnostics: dict[str, Any] | None = None
    metadata_residue: dict[str, Any] | None = None
    result: tuple[int, bytes, bytes, bool, int | None] = (-1, b"", b"", False, None)
    failure_kind = "GPG_KEY_INVENTORY_FAILED"
    try:
        homedir, homedir_identity = _gpg_inventory_homedir(
            root, ordinal, identity=inventory_identity,
            identity_registry=identity_registry)
        argv = _gpg_inventory_argv(homedir, Path(inventory_path))
        _validate_argv(argv, "gpg key inventory")
        version_tool = _gpg_inventory_tool(
            root, container_root, ordinal, runner, homedir,
            identity=inventory_identity, identity_registry=identity_registry)
        result = _normalize_output(runner(argv, 120))
        diagnostics = _write_command_diagnostics(
            root, "gpg-key-inventory-{}".format(ordinal), result[1], result[2],
            identity=inventory_identity, identity_registry=identity_registry)
        try:
            metadata_residue = _capture_gpg_inventory_metadata(
                homedir, homedir_identity, diagnostics["stderr"])
        except CaptureError as metadata_error:
            metadata_residue = _gpg_metadata_record(
                diagnostics["stderr"], [], failure_reason=metadata_error.kind, race=True)
            raise
        homedir_identity["_metadata_residue"] = metadata_residue
        if result[0] != 0 or result[3] or result[4] is not None:
            raise CaptureError("GPG_KEY_INVENTORY_FAILED", inventory_path)
        parsed = _parse_gpg_inventory(result[1], result[2], inventory_path)
        descriptor["primary_fingerprints"] = parsed["primary_fingerprints"]
        descriptor["subkey_fingerprints"] = parsed["subkey_fingerprints"]
        descriptor["expected_signer_fingerprints"] = sorted(
            set(parsed["primary_fingerprints"] + parsed["subkey_fingerprints"]))
        if metadata_residue["status"] != "EXACT":
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE", inventory_path)
        descriptor["inventory"] = {
            "status": "VERIFIED", "identity": inventory_identity,
            "command": argv,
            "command_sha256": _command_hash(argv), "exit_status": result[0],
            "timed_out": result[3], "signal": result[4],
            "stdout": diagnostics["stdout"], "stderr": diagnostics["stderr"],
            "tool": version_tool,
            "homedir": _public_gpg_inventory_homedir(homedir_identity),
            "metadata_residue": metadata_residue,
        }
    except CaptureError as error:
        failure_kind = (error.kind if error.kind in GPG_KEY_INVENTORY_FAILURE_KINDS
                        else "GPG_KEY_INVENTORY_FAILED")
        if argv:
            if version_tool is None:
                version_diagnostics = {}
                for stream in ("stdout", "stderr"):
                    token = _identity_token(inventory_identity, "gpg inventory identity")
                    path = root / "logs" / (
                        "gpg-key-inventory-{}-version.{}".format(token, stream))
                    if path.exists() or path.is_symlink():
                        version_diagnostics[stream] = _relative_descriptor(
                            root, _descriptor(path, "gpg inventory version log",
                                              MAX_LOG_BYTES))
                    else:
                        version_diagnostics[stream] = None
            descriptor["inventory"] = {
                "status": "FAILED", "identity": inventory_identity,
                "failure_kind": failure_kind,
                "command": argv, "command_sha256": _command_hash(argv),
                "exit_status": result[0], "timed_out": result[3],
                "signal": result[4],
                "stdout": diagnostics["stdout"] if diagnostics is not None else None,
                "stderr": diagnostics["stderr"] if diagnostics is not None else None,
                "tool": version_tool,
                "homedir": (_public_gpg_inventory_homedir(homedir_identity)
                            if homedir_identity else None),
                "metadata_residue": metadata_residue,
                "version_stdout": None if version_diagnostics is None else
                version_diagnostics["stdout"],
                "version_stderr": None if version_diagnostics is None else
                version_diagnostics["stderr"],
            }
        if failure_kind != error.kind:
            raise CaptureError(failure_kind, str(error)) from error
        raise
    finally:
        if homedir is not None and homedir_identity is not None:
            try:
                _remove_gpg_inventory_tree(homedir, homedir_identity)
                homedir_identity["post_absent"] = True
                homedir_identity["cleanup_status"] = "PASS"
            except CaptureError as cleanup_error:
                homedir_identity["post_absent"] = not homedir.exists() and not homedir.is_symlink()
                homedir_identity["cleanup_status"] = "FAIL_CLOSED"
                if cleanup_error.kind != failure_kind:
                    if isinstance(descriptor.get("inventory"), dict):
                        descriptor["inventory"]["homedir"] = _public_gpg_inventory_homedir(
                            homedir_identity)
                    raise cleanup_error
            if isinstance(descriptor.get("inventory"), dict):
                descriptor["inventory"]["homedir"] = _public_gpg_inventory_homedir(
                    homedir_identity)


def _validate_inventory_homedir(value: Any, label: str,
                                inventory_identity: str | None = None
                                ) -> dict[str, Any]:
    """Validate the sealed identity and cleanup claim for a gpg sandbox."""
    required = {"path", "status", "mode", "uid", "gid", "nlink", "device", "inode",
                "parent_path", "parent_relative", "post_absent", "cleanup_status"}
    if not isinstance(value, Mapping) or set(value) != required:
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_INVALID", label)
    path = _safe_abs(value["path"], label + " path")
    parent = _safe_abs(value["parent_path"], label + " parent path")
    if (value["parent_relative"] != GPG_INVENTORY_WORK_RELATIVE or
            path.parent != parent or parent.name != GPG_INVENTORY_WORK_RELATIVE or
            re.fullmatch(re.escape(GPG_INVENTORY_HOMEDIR_PREFIX) + r"[a-f0-9]{24}",
                         path.name) is None):
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_INVALID", label)
    if value["status"] != "FRESH" or value["mode"] != 0o700 or \
            type(value["post_absent"]) is not bool or not value["post_absent"] or \
            value["cleanup_status"] != "PASS":
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_INVALID", label)
    for field in ("uid", "gid", "nlink", "device", "inode"):
        if type(value[field]) is not int or value[field] < 0:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_INVALID", label)
    if value["nlink"] < 2 or value["device"] <= 0 or value["inode"] <= 0:
        raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_INVALID", label)
    if inventory_identity is not None:
        token = _identity_token(inventory_identity, label + " identity")
        if path.name != GPG_INVENTORY_HOMEDIR_PREFIX + token:
            raise CaptureError("GPG_KEY_INVENTORY_HOMEDIR_INVALID", label)
    return dict(value)


def _validate_gpg_metadata_residue(value: Any, identity: Mapping[str, Any],
                                   label: str, *, require_exact: bool = False
                                   ) -> dict[str, Any] | None:
    """Validate the sealed pre-cleanup GnuPG metadata projection.

    The child GNUPGHOME is absent by the time an outer receipt is reopened,
    so the descriptor is the only evidence of what was removed.  A successful
    inventory requires exactly the two files produced by the pinned GnuPG
    version.  ``EMPTY`` is retained for failures before GnuPG created either
    file; every other shape is non-promoting and must explain its rejection.
    """
    if value is None:
        if require_exact:
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
        return None
    fields = {"schema", "schema_version", "policy", "expected_names", "entries",
              "status", "failure_reason", "creation_evidence", "retention"}
    if not isinstance(value, Mapping) or set(value) != fields:
        raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    if (value["schema"] != GPG_INVENTORY_METADATA_SCHEMA or
            value["schema_version"] != 1 or
            value["policy"] != GPG_INVENTORY_METADATA_POLICY or
            value["expected_names"] != list(GPG_INVENTORY_METADATA_NAMES) or
            value["retention"] != GPG_INVENTORY_METADATA_RETENTION):
        raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    if not isinstance(value["creation_evidence"], Mapping):
        raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    entries = value["entries"]
    if not isinstance(entries, list) or len(entries) > 32:
        raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    entry_fields = {"name", "type", "mode", "uid", "gid", "nlink", "size",
                    "device", "inode", "sha256", "xattrs", "status", "reason"}
    types = {"regular", "symlink", "socket", "fifo", "directory", "block", "char",
             "other"}
    reasons = {"accepted", "unexpected_name", "type_invalid", "mode_invalid",
               "owner_invalid", "nlink_invalid", "size_invalid", "xattrs_invalid",
               "xattrs_unreadable", "hash_missing", "metadata_race", "read_failed",
               "stat_failed"}
    names: list[str] = []
    for entry in entries:
        if not isinstance(entry, Mapping) or set(entry) != entry_fields:
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
        name = entry["name"]
        if (not isinstance(name, str) or not re.fullmatch(r"[^/\\\x00\r\n]+", name) or
                name in names):
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
        names.append(name)
        if entry["type"] not in types or entry["status"] not in {"ACCEPTED", "REJECTED"} or \
                entry["reason"] not in reasons:
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
        for field in ("mode", "uid", "gid", "nlink", "size", "device", "inode"):
            if type(entry[field]) is not int or entry[field] < 0:
                raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
        if entry["mode"] > 0o7777 or entry["size"] > GPG_INVENTORY_METADATA_MAX_BYTES:
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
        if entry["sha256"] is not None:
            _sha(entry["sha256"], label + " entry SHA")
        if entry["xattrs"] is not None:
            if (not isinstance(entry["xattrs"], list) or
                    entry["xattrs"] != sorted(set(entry["xattrs"])) or
                    any(not isinstance(item, str) for item in entry["xattrs"])):
                raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
        if entry["status"] == "ACCEPTED":
            if (name not in GPG_INVENTORY_METADATA_NAMES or entry["type"] != "regular" or
                    entry["mode"] != GPG_INVENTORY_METADATA_MODE or
                    entry["uid"] != identity["uid"] or entry["gid"] != identity["gid"] or
                    entry["nlink"] != 1 or not 0 < entry["size"] <=
                    GPG_INVENTORY_METADATA_MAX_BYTES or entry["sha256"] is None or
                    entry["xattrs"] != [] or entry["reason"] != "accepted"):
                raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
        elif entry["reason"] == "accepted":
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    if names != sorted(names):
        raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    status = value["status"]
    failure_reason = value["failure_reason"]
    if status not in {"EXACT", "EMPTY", "REJECTED"}:
        raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    if status in {"EXACT", "EMPTY"} and failure_reason is not None:
        raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    if status == "EMPTY" and entries:
        raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    if status == "EXACT" and (names != list(GPG_INVENTORY_METADATA_NAMES) or
                               any(entry["status"] != "ACCEPTED" for entry in entries)):
        raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    if status == "REJECTED":
        if failure_reason not in reasons or failure_reason == "accepted":
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
        if (not entries and failure_reason not in {"metadata_race", "child_missing",
                                                   "list_failed"}):
            raise CaptureError("GPG_KEY_INVENTORY_RESIDUE_INVALID", label)
    if require_exact and status != "EXACT":
        raise CaptureError("GPG_KEY_INVENTORY_RESIDUE", label)
    return dict(value)


def _validate_gpg_inventory_argv(argv: Any, homedir: Path, keyring_relative: str,
                                label: str) -> list[str]:
    """Rebuild the exact no-network GnuPG 2.2.27 inventory argv contract."""
    if not isinstance(argv, list) or len(argv) != 17:
        raise CaptureError("GPG_KEY_INVENTORY_COMMAND_INVALID", label)
    expected_prefix = [
        GPG_INVENTORY_EXECUTABLE, "--batch", "--no-options",
        "--no-default-keyring", "--no-auto-key-retrieve", "--no-auto-key-import",
        "--no-auto-key-locate", "--no-autostart", "--homedir", str(homedir),
        "--with-colons", "--with-fingerprint", "--with-subkey-fingerprint",
        "--import-options", "show-only", "--import",
    ]
    if argv[:16] != expected_prefix:
        raise CaptureError("GPG_KEY_INVENTORY_COMMAND_INVALID", label)
    keyring_path = _safe_abs(argv[16], label + " keyring path")
    relative = _safe_rel(keyring_relative, label + " keyring relative path")
    if not str(keyring_path).endswith("/" + relative):
        raise CaptureError("GPG_KEY_INVENTORY_COMMAND_INVALID", label)
    if any(token in argv for token in ("--keyserver", "--auto-key-retrieve",
                                       "--auto-key-import", "--auto-key-locate",
                                       "--use-agent", "--daemon")):
        raise CaptureError("GPG_KEY_INVENTORY_COMMAND_INVALID", label)
    return list(argv)


def _validate_gpg_inventory_tool(root: Path, value: Any, homedir: Path,
                                 label: str) -> dict[str, Any]:
    """Reopen the pinned gpg binary/version evidence and exact command logs."""
    required = {"name", "path", "sha256", "bytes", "version", "version_command",
                "version_command_sha256", "version_exit_status", "version_timed_out",
                "version_signal", "version_stdout", "version_stderr", "binary_output",
                "homedir"}
    if not isinstance(value, Mapping) or set(value) != required or value["name"] != "gpg" or \
            value["path"] != GPG_INVENTORY_EXECUTABLE or value["homedir"] != str(homedir):
        raise CaptureError("GPG_KEY_INVENTORY_TOOL_INVALID", label)
    _sha(value["sha256"], label + " binary SHA")
    if type(value["bytes"]) is not int or not 0 < value["bytes"] <= MAX_CAPTURE_FILE_BYTES or \
            value["version"] not in GPG_INVENTORY_VERSIONS or \
            value["version_command"] != [GPG_INVENTORY_EXECUTABLE, "--version"] or \
            value["version_command_sha256"] != _command_hash(value["version_command"]):
        raise CaptureError("GPG_KEY_INVENTORY_TOOL_INVALID", label)
    if value["version_exit_status"] != 0 or value["version_timed_out"] is not False or \
            value["version_signal"] is not None:
        raise CaptureError("GPG_KEY_INVENTORY_TOOL_INVALID", label)
    for field in ("version_stdout", "version_stderr"):
        descriptor = value[field]
        if not isinstance(descriptor, Mapping):
            raise CaptureError("GPG_KEY_INVENTORY_TOOL_INVALID", label)
        path = _safe_rel(descriptor.get("path"), label + " version log")
        actual = _relative_descriptor(
            root, _descriptor(root / path, label + " version log", MAX_LOG_BYTES))
        if actual != dict(descriptor):
            raise CaptureError("GPG_KEY_INVENTORY_TOOL_INVALID", label)
    binary = value["binary_output"]
    if not isinstance(binary, Mapping):
        raise CaptureError("GPG_KEY_INVENTORY_TOOL_INVALID", label)
    binary_path = _safe_rel(binary.get("path"), label + " binary output")
    if not binary_path.startswith(APT_KEYRING_RELATIVE + "/"):
        raise CaptureError("GPG_KEY_INVENTORY_TOOL_INVALID", label)
    actual_binary = _relative_descriptor(
        root, _descriptor(root / binary_path, label + " binary output",
                          MAX_CAPTURE_FILE_BYTES))
    if actual_binary != dict(binary) or actual_binary["bytes"] != value["bytes"] or \
            actual_binary["sha256"] != value["sha256"]:
        raise CaptureError("GPG_KEY_INVENTORY_TOOL_INVALID", label)
    return dict(value)


def _gpgv_version(runner: Callable[[list[str], int], Any]) -> tuple[str, list[str]]:
    argv = ["gpgv", "--version"]
    _validate_argv(argv, "gpgv version")
    result = _normalize_output(runner(argv, 30))
    if result[0] != 0 or result[3] or result[4] is not None:
        raise CaptureError("GPGV_VERSION_FAILED", "gpgv --version")
    try:
        lines = result[1].decode("utf-8").splitlines()
    except UnicodeError as error:
        raise CaptureError("GPGV_VERSION_INVALID", "gpgv --version") from error
    if not lines or VERSION_RE.fullmatch(lines[0]) is None:
        raise CaptureError("GPGV_VERSION_INVALID", "gpgv --version")
    if len(lines[0]) > 128:
        raise CaptureError("GPGV_VERSION_INVALID", "gpgv --version")
    return lines[0], argv


def _apt_list_prefix(url: str, suite: str) -> str:
    parts = urlsplit(url)
    path = parts.path.strip("/")
    prefix = parts.netloc + ("_" + path if path else "")
    prefix = prefix.replace("/", "_").replace(":", "%3a")
    return prefix + "_dists_" + suite


def _discover_release(container_root: Path | None,
                      repository: Mapping[str, Any]) -> tuple[Path, str]:
    lists = _container_path(container_root, "/var/lib/apt/lists", "apt lists")
    files = _container_files(lists, "apt lists", suffixes=("InRelease", "Release", "Release.gpg",
                                                              "Packages", ".gz", ".bz2", ".xz", ".lz4"))
    prefix = _apt_list_prefix(repository["url"], repository["suite"])
    releases = [item for item in files if item.name in {
        prefix + "_InRelease", prefix + "_Release"}]
    if len(releases) != 1:
        raise CaptureError("APT_RELEASE_AMBIGUOUS", prefix)
    release = releases[0]
    return release, ("inrelease" if release.name.endswith("_InRelease") else "detached")


def _discover_release_and_packages(container_root: Path | None,
                                   repository: Mapping[str, Any]) -> tuple[Path, Path, str]:
    """Compatibility wrapper for callers that only have local indexes.

    New capture code selects an index only after the signed Release has been
    verified.  This wrapper deliberately considers only the profile-approved
    formats and therefore never treats an apt-local ``.lz4`` file as a valid
    Packages input.
    """
    release, release_kind = _discover_release(container_root, repository)
    lists = _container_path(container_root, "/var/lib/apt/lists", "apt lists")
    files = _container_files(
        lists, "apt lists", suffixes=("Packages", ".gz", ".xz"))
    prefix = _apt_list_prefix(repository["url"], repository["suite"])
    package_prefix = prefix + "_" + repository["component"] + "_binary-amd64_Packages"
    packages = [item for item in files if item.name in {
        package_prefix, package_prefix + ".xz", package_prefix + ".gz"}]
    if len(packages) != 1:
        raise CaptureError("APT_PACKAGES_AMBIGUOUS", package_prefix)
    return release, packages[0], release_kind


def _parse_dpkg_deb_output(data: bytes, label: str) -> dict[str, Any]:
    if len(data) > 64 * 1024:
        raise CaptureError("DPKG_DEB_OUTPUT_OVERSIZE", label)
    try:
        lines = data.decode("utf-8").splitlines()
    except UnicodeError as error:
        raise CaptureError("DPKG_DEB_OUTPUT_INVALID", label) from error
    fields: dict[str, str] = {}
    for line in lines:
        if ": " not in line:
            raise CaptureError("DPKG_DEB_OUTPUT_INVALID", label)
        key, value = line.split(": ", 1)
        if key in fields or not value:
            raise CaptureError("DPKG_DEB_OUTPUT_INVALID", label)
        fields[key] = value
    required = {"Package", "Version", "Architecture"}
    if not required.issubset(fields):
        raise CaptureError("DPKG_DEB_FIELDS_MISSING", label)
    if (not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+_.:-]{0,127}", fields["Package"]) or
            not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+_.:~+-]{0,255}", fields["Version"]) or
            not re.fullmatch(r"[A-Za-z0-9][A-Za-z0-9+_.-]{0,31}", fields["Architecture"])):
        raise CaptureError("DPKG_DEB_IDENTITY_INVALID", label)
    pre_depends = [item.strip() for item in fields.get("Pre-Depends", "").split(",")
                   if item.strip()]
    if pre_depends != sorted(set(pre_depends)):
        pre_depends = sorted(set(pre_depends))
    return {"name": fields["Package"], "version": fields["Version"],
            "architecture": fields["Architecture"],
            "essential": fields.get("Essential", "no").lower() == "yes",
            "pre_depends": pre_depends,
            "multiarch": fields.get("Multi-Arch", "same")}


def _relative_descriptor(root: Path, descriptor: Mapping[str, Any]) -> dict[str, Any]:
    result = dict(descriptor)
    try:
        result["path"] = Path(descriptor["path"]).relative_to(root).as_posix()
    except (KeyError, ValueError) as error:
        raise CaptureError("CAPTURE_DESCRIPTOR_OUTSIDE_ROOT", str(descriptor)) from error
    return result


def _capture_source_file_record(root: Path, container_path: Path, data: bytes,
                                destination: str, role: str, url: str,
                                *, maximum: int = MAX_CAPTURE_FILE_BYTES,
                                allow_empty: bool = False) -> dict[str, Any]:
    descriptor = _copy_capture_file(root, destination, data, maximum,
                                    allow_empty=allow_empty)
    return {"path": str(Path(destination).relative_to(APT_SOURCE_RELATIVE)),
            "role": role, "url": url, "bytes": len(data),
            "sha256": sha256_bytes(data), "source": str(container_path),
            "output": _relative_descriptor(root, descriptor)}


def _source_file_record_v2(root: Path, container_path: Path, data: bytes,
                           destination: str, urls: list[str]) -> dict[str, Any]:
    descriptor = _copy_capture_file(root, destination, data)
    return {"path": str(Path(destination).relative_to(APT_SOURCE_RELATIVE)),
            "role": "apt_source", "urls": list(urls), "bytes": len(data),
            "sha256": sha256_bytes(data), "source": str(container_path),
            "output": _relative_descriptor(root, descriptor)}


def _source_entry_v2(path: str, record: Mapping[str, Any], ordinal: int,
                     key_refs: Mapping[str, Mapping[str, Any]]) -> dict[str, Any]:
    signed_by = dict(record["signed_by"])
    for field in ("data", "dearmored_data", "normalized_armor_bytes",
                  "normalized_armor_sha256", "dearmored_bytes",
                  "dearmored_sha256"):
        signed_by.pop(field, None)
    if signed_by["kind"] == "path":
        reference = key_refs.get("path:" + signed_by["source"])
    elif signed_by["kind"] == "inline":
        reference = key_refs.get("inline:" + signed_by["sha256"])
    else:
        reference = None
    if reference is not None:
        signed_by.update({"artifact_path": reference["path"],
                          "bytes": reference["bytes"],
                          "sha256": reference["sha256"]})
        if signed_by["kind"] == "inline":
            signed_by["fingerprint"] = reference["fingerprint"]
    if signed_by["kind"] != "none" and "artifact_path" not in signed_by:
        raise CaptureError("APT_KEYRING_BINDING_MISSING", path)
    return {"path": path, "ordinal": ordinal, "format": record["format"],
            "line_start": record["line_start"], "line_end": record["line_end"],
            "types": list(record["types"]), "uris": list(record["uris"]),
            "suites": list(record["suites"]),
            "components": list(record["components"]),
            "options": [dict(item) for item in record["options"]],
            "signed_by": signed_by}


def _validate_descriptor_stat(value: Any, label: str, *, absolute_path: bool,
                              maximum: int = MAX_CAPTURE_FILE_BYTES,
                              exact_symlink: bool = False) -> None:
    """Validate the complete immutable stat projection of a captured file."""
    required = {"path", "bytes", "sha256", "mode", "uid", "gid", "nlink",
                "device", "inode"}
    if not isinstance(value, Mapping) or set(value) != required:
        raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", label)
    path = value["path"]
    if absolute_path:
        if (not isinstance(path, str) or not path.startswith("/") or
                Path(path).as_posix() != path):
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", label)
    else:
        _safe_rel(path, label + " path")
    if type(value["bytes"]) is not int or not 0 < value["bytes"] <= maximum:
        raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", label)
    _sha(value["sha256"], label + " SHA")
    for field in ("mode", "uid", "gid", "nlink", "device", "inode"):
        if type(value[field]) is not int:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", label)
    mode_invalid = (
        value["mode"] != 0o777 if exact_symlink
        else not 0 <= value["mode"] <= 0o7777 or bool(value["mode"] & 0o022)
    )
    if mode_invalid or \
            value["uid"] < 0 or value["gid"] < 0 or value["nlink"] != 1 or \
            value["device"] <= 0 or value["inode"] <= 0:
        raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", label)


def _reopen_v2_source_descriptor(item: Mapping[str, Any]) -> None:
    """Reopen the original source descriptor and compare all bytes/stat fields."""
    descriptor = item["descriptor"]
    logical = descriptor["logical_path"]
    source = Path(descriptor["path"])
    if not str(source).endswith("/" + logical):
        raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_PATH_DRIFT", logical)
    if descriptor["kind"] == "exact_symlink":
        target = descriptor["target_descriptor"]
        target_path = Path(target["path"])
        if not str(target_path).endswith("/usr/share/ros-apt-source/ros2.sources"):
            raise CaptureError("SOURCE_MANIFEST_SYMLINK_TARGET_INVALID", logical)
        _, observed = _container_read(
            source, "source descriptor " + logical, MAX_CAPTURE_FILE_BYTES,
            allow_empty=False, exact_symlink_target=descriptor["target"],
            symlink_target_path=target_path, require_root=False)
    else:
        _, observed = _container_read(
            source, "source descriptor " + logical, MAX_CAPTURE_FILE_BYTES,
            allow_empty=False, require_root=False)
    observed["logical_path"] = logical
    if observed != dict(descriptor):
        raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_METADATA_DRIFT", logical)


def _source_file_url(records: list[dict[str, Any]], label: str) -> str:
    urls = sorted({item["url"] for item in records})
    if len(urls) != 1:
        raise CaptureError("APT_SOURCE_URL_AMBIGUOUS", label)
    return urls[0]


def _repository_urls(repository: Mapping[str, Any], release_kind: str,
                     packages_suffix: str) -> tuple[str, str]:
    base = str(repository["url"]).rstrip("/")
    release_name = "InRelease" if release_kind == "inrelease" else "Release"
    release_url = "{}/dists/{}/{}".format(base, repository["suite"], release_name)
    package_name = "Packages" + ("." + packages_suffix if packages_suffix != "plain" else "")
    packages_url = "{}/dists/{}/{}/binary-amd64/{}".format(
        base, repository["suite"], repository["component"], package_name)
    return release_url, packages_url


def _release_acquire_by_hash(data: bytes, label: str) -> bool:
    """Read the signed Acquire-By-Hash flag without accepting ambiguity."""
    try:
        lines = data.decode("ascii", "strict").splitlines()
    except UnicodeDecodeError as error:
        raise CaptureError("RELEASE_BY_HASH_INVALID", label) from error
    values = []
    for line in lines:
        if not line.startswith("Acquire-By-Hash:"):
            continue
        if line not in {"Acquire-By-Hash: yes", "Acquire-By-Hash: no"}:
            raise CaptureError("RELEASE_BY_HASH_INVALID", label)
        values.append(line.rsplit(" ", 1)[1])
    if len(values) > 1:
        raise CaptureError("RELEASE_BY_HASH_INVALID", label)
    return values == ["yes"]


def _packages_by_hash_url(repository: Mapping[str, Any], digest: str) -> str:
    """Bind a Packages request to the SHA256 declared by signed Release."""
    _sha(digest, "Packages by-hash digest")
    base = str(repository["url"]).rstrip("/")
    return "{}/dists/{}/{}/binary-amd64/by-hash/SHA256/{}".format(
        base, repository["suite"], repository["component"], digest)


def _fetch_result(result: Any, requested_url: str) -> tuple[bytes, str]:
    """Normalize the bounded fetcher test seam without accepting extra data."""
    final_url = requested_url
    payload = result
    if isinstance(result, Mapping):
        if set(result) != {"data", "final_url"}:
            raise CaptureError("PACKAGE_FETCH_RESULT_INVALID", requested_url)
        payload, final_url = result["data"], result["final_url"]
    elif isinstance(result, tuple):
        if len(result) != 2:
            raise CaptureError("PACKAGE_FETCH_RESULT_INVALID", requested_url)
        payload, final_url = result
    if not isinstance(payload, bytes) or not payload:
        raise CaptureError("PACKAGE_FETCH_RESULT_INVALID", requested_url)
    if not isinstance(final_url, str):
        raise CaptureError("PACKAGE_FETCH_RESULT_INVALID", requested_url)
    return payload, final_url


class _SameHostRedirectHandler(urllib.request.HTTPRedirectHandler):
    """Permit only policy-valid same-host redirects for a package fetch."""

    def __init__(self, policy: Mapping[str, Any]):
        super().__init__()
        self.policy = policy

    def redirect_request(self, request, file, code, msg, headers, newurl):
        target = urljoin(request.full_url, newurl)
        old_parts = urlsplit(request.full_url)
        new_parts = urlsplit(target)
        if (old_parts.scheme, old_parts.hostname, old_parts.port) != (
                new_parts.scheme, new_parts.hostname, new_parts.port):
            raise CaptureError("PACKAGE_REDIRECT_INVALID", target)
        _capture_url(target, "Packages redirect URL", policy=self.policy)
        return super().redirect_request(request, file, code, msg, headers, target)


def _fetch_packages_url(url: str, policy: Mapping[str, Any]) -> tuple[bytes, str]:
    """Fetch one exact Packages URL with bounded bytes and no retry."""
    _capture_url(url, "Packages fetch URL", policy=policy)
    request = urllib.request.Request(
        url, headers={"Accept-Encoding": "identity"}, method="GET")
    opener = urllib.request.build_opener(_SameHostRedirectHandler(policy))
    try:
        with opener.open(request, timeout=60) as response:
            final_url = _capture_url(
                response.geturl(), "Packages final URL", policy=policy)
            chunks: list[bytes] = []
            total = 0
            while True:
                block = response.read(min(1024 * 1024, MAX_CAPTURE_FILE_BYTES - total + 1))
                if not block:
                    break
                chunks.append(block)
                total += len(block)
                if total > MAX_CAPTURE_FILE_BYTES:
                    raise CaptureError("PACKAGE_FETCH_OVERSIZE", url)
    except CaptureError:
        raise
    except (OSError, urllib.error.URLError, urllib.error.HTTPError) as error:
        raise CaptureError("PACKAGE_FETCH_FAILED", url) from error
    data = b"".join(chunks)
    if not data:
        raise CaptureError("PACKAGE_FETCH_EMPTY", url)
    return data, final_url


def _invoke_package_fetcher(fetcher: Callable[..., Any] | None, url: str,
                            policy: Mapping[str, Any]) -> tuple[bytes, str]:
    if fetcher is None:
        return _fetch_packages_url(url, policy)
    try:
        try:
            result = fetcher(url, 60)
        except TypeError:
            result = fetcher(url)
    except CaptureError:
        raise
    except Exception as error:
        raise CaptureError("PACKAGE_FETCH_FAILED", url) from error
    data, final_url = _fetch_result(result, url)
    _capture_url(final_url, "Packages final URL", policy=policy)
    requested = urlsplit(url)
    final = urlsplit(final_url)
    if (requested.scheme, requested.hostname, requested.port) != (
            final.scheme, final.hostname, final.port):
        raise CaptureError("PACKAGE_REDIRECT_INVALID", url)
    return data, final_url


def _packages_candidate_names(container_root: Path | None,
                              repository: Mapping[str, Any]) -> dict[str, list[Path]]:
    """Return local approved Packages candidates, excluding lz4/bz2 caches."""
    lists = _container_path(container_root, "/var/lib/apt/lists", "apt lists")
    files = _container_files(lists, "apt lists", suffixes=("Packages", ".gz", ".xz"))
    prefix = _apt_list_prefix(repository["url"], repository["suite"])
    base = prefix + "_" + repository["component"] + "_binary-amd64_Packages"
    expected = {"xz": base + ".xz", "gz": base + ".gz", "plain": base}
    result = {key: [] for key in PACKAGES_FORMAT_POLICY["selection_order"]}
    for path in files:
        for fmt, name in expected.items():
            if path.name == name:
                result[fmt].append(path)
    for fmt, paths in result.items():
        if len(paths) > 1:
            raise CaptureError("APT_PACKAGES_AMBIGUOUS", expected[fmt])
    return result


def _validate_repository_release_spec(
        spec: Mapping[str, Any], release_evidence: list[Mapping[str, Any]]
        ) -> tuple[str, Mapping[str, Any], bytes]:
    """Validate the immutable Release context owned by one repository spec.

    Discovery can visit several repositories before package selection starts.
    Keeping this context on each spec, and checking it against the already
    copied Release evidence, prevents a later discovery iteration from
    leaking its URL/suite/bytes into an earlier package identity.
    """
    required = {
        "repository", "release", "release_kind", "release_identity",
        "release_record", "release_data", "signed_by",
    }
    if not isinstance(spec, Mapping) or set(spec) != required:
        raise CaptureError("GPG_RELEASE_SPEC_INVALID", "repository spec fields")
    repository = spec["repository"]
    release = spec["release"]
    release_kind = spec["release_kind"]
    release_identity = spec["release_identity"]
    release_record = spec["release_record"]
    release_data = spec["release_data"]
    if (not isinstance(repository, Mapping) or
            not isinstance(release, Path) or
            release_kind not in {"inrelease", "detached"} or
            not isinstance(release_identity, str) or
            not isinstance(release_record, Mapping) or
            not isinstance(release_data, bytes)):
        raise CaptureError("GPG_RELEASE_SPEC_INVALID", "repository spec types")
    _sha(release_identity, "Release spec identity")
    if not release.is_absolute() or release.is_symlink():
        raise CaptureError("GPG_RELEASE_SPEC_INVALID", str(release))
    if not 0 < len(release_data) <= MAX_CAPTURE_FILE_BYTES:
        raise CaptureError("GPG_RELEASE_SPEC_INVALID", str(release))
    try:
        release_url, _ = _repository_urls(repository, release_kind, "plain")
    except (KeyError, TypeError, CaptureError) as error:
        raise CaptureError("GPG_RELEASE_SPEC_INVALID", str(release)) from error
    expected_identity = _release_artifact_identity(
        repository, release, release_kind, release_data)
    if release_identity != expected_identity:
        raise CaptureError("GPG_RELEASE_IDENTITY_CONFLICT", str(release))
    record_fields = {"path", "role", "url", "bytes", "sha256", "source", "output"}
    if (set(release_record) != record_fields or
            release_record["role"] != "apt_release" or
            release_record["url"] != release_url or
            release_record["source"] != str(release) or
            release_record["bytes"] != len(release_data) or
            release_record["sha256"] != sha256_bytes(release_data)):
        raise CaptureError("GPG_RELEASE_SPEC_INVALID", str(release))
    _safe_rel(release_record["path"], "Release spec record path")
    if not isinstance(release_evidence, list):
        raise CaptureError("GPG_RELEASE_SPEC_INVALID", str(release))
    matching = [item for item in release_evidence
                if isinstance(item, Mapping) and
                item.get("path") == release_record["path"]]
    if len(matching) != 1:
        raise CaptureError("GPG_RELEASE_IDENTITY_CONFLICT", str(release))
    observed = matching[0]
    expected = {
        "source": str(release), "url": release_url,
        "repository_url": repository.get("url"), "kind": release_kind,
        "identity": release_identity, "bytes": len(release_data),
        "sha256": sha256_bytes(release_data), "output": release_record["output"],
    }
    if any(observed.get(key) != value for key, value in expected.items()):
        raise CaptureError("GPG_RELEASE_IDENTITY_CONFLICT", str(release))
    return release_identity, _immutable_copy(release_record), bytes(release_data)


def _select_packages_index(container_root: Path | None,
                           repository: Mapping[str, Any],
                           release_kind: str,
                           release_entries: Mapping[str, tuple[str, int]],
                           policy: Mapping[str, Any],
                           fetcher: Callable[..., Any] | None = None,
                           raw_persist: Callable[[dict[str, Any], bytes], Mapping[str, Any]] | None = None,
                           *, release_identity: str | None = None,
                           release_record: Mapping[str, Any] | None = None,
                           release_data: bytes | None = None
                           ) -> dict[str, Any]:
    """Select/fetch one signed Packages representation in fixed order."""
    package_relative = "{}/binary-amd64/Packages".format(repository["component"])
    local = _packages_candidate_names(container_root, repository)
    selected: tuple[str, str] | None = None
    for fmt in PACKAGES_FORMAT_POLICY["selection_order"]:
        suffix = "" if fmt == "plain" else "." + fmt
        raw_relative = package_relative + suffix
        if raw_relative not in release_entries or package_relative not in release_entries:
            continue
        if local[fmt]:
            selected = (fmt, raw_relative)
            break
        selected = (fmt, raw_relative)
        break
    if selected is None:
        raise CaptureError("APT_PACKAGES_SIGNED_PAIR_MISSING", package_relative)
    compression, raw_relative = selected
    _, packages_url = _repository_urls(repository, release_kind, compression)
    decoded_indexed = release_entries.get(package_relative)
    if (not isinstance(release_identity, str) or
            not isinstance(release_record, Mapping) or
            not isinstance(release_data, bytes) or
            decoded_indexed is None):
        raise CaptureError("PACKAGES_IDENTITY_CONTEXT_MISSING", package_relative)
    if _release_acquire_by_hash(release_data, str(release_record["path"])):
        packages_url = _packages_by_hash_url(
            repository, release_entries[raw_relative][0])
    package_identity = _package_artifact_identity(
        repository, release_kind, release_identity,
        str(release_record["path"]), release_data, package_relative,
        raw_relative, compression, packages_url, release_entries[raw_relative][0],
        release_entries[raw_relative][1], decoded_indexed[0], decoded_indexed[1],
        signed_empty=decoded_indexed[1] == 0)
    local_path = local[compression][0] if local[compression] else None
    source = "container-cache"
    final_url = packages_url
    if local_path is not None:
        raw_data, source_descriptor = _container_read(
            local_path, "Packages {}".format(local_path.name),
            MAX_CAPTURE_FILE_BYTES, allow_empty=False,
            require_root=container_root in (None, Path("/")))
    else:
        source = "network"
        raw_data, final_url = _invoke_package_fetcher(fetcher, packages_url, policy)
        source_descriptor = None
    indexed = release_entries[raw_relative]
    raw_digest = sha256_bytes(raw_data)
    raw_size = len(raw_data)
    raw_evidence = {
        "source": source,
        "url": packages_url, "final_url": final_url, "format": compression,
        "release_path": raw_relative, "release_sha256": indexed[0],
        "release_bytes": indexed[1], "bytes": raw_size, "sha256": raw_digest,
        "release_record_path": None, "release_file_sha256": None,
        "release_file_bytes": None, "source_record_path": None,
        "decoded_release_path": None,
        "decoded_release_sha256": None, "decoded_release_bytes": None,
        "decoded_output": None, "stream_descriptors": [],
        "signed_empty": package_identity["signed_empty"],
        "artifact_identity": package_identity,
    }
    if raw_persist is not None:
        raw_evidence = dict(raw_persist(raw_evidence, raw_data))
    if indexed != (raw_digest, raw_size):
        raise CaptureError("PACKAGES_RELEASE_BINDING_DRIFT", raw_relative)
    decoded, actual_format, stream_descriptors = _decode_packages_index_details(
        Path("Packages" + ("." + compression if compression != "plain" else "")),
        raw_data, expected_format=compression,
        allow_signed_empty=package_identity["signed_empty"])
    if actual_format != compression:
        raise CaptureError("PACKAGES_FORMAT_MISMATCH", raw_relative)
    if decoded_indexed != (sha256_bytes(decoded), len(decoded)):
        raise CaptureError("PACKAGES_DECODED_RELEASE_BINDING_DRIFT", raw_relative)
    raw_evidence["stream_descriptors"] = stream_descriptors
    return {
        "compression": compression, "raw_relative": raw_relative,
        "raw_data": raw_data, "raw_digest": raw_digest, "raw_size": raw_size,
        "decoded": decoded, "packages_url": packages_url, "final_url": final_url,
        "source": source, "source_descriptor": source_descriptor,
        "raw_evidence": raw_evidence, "package_relative": package_relative,
        "decoded_indexed": decoded_indexed, "stream_descriptors": stream_descriptors,
        "package_identity": package_identity,
    }


def _verify_release_signature(container_root: Path | None, release: Path,
                              release_kind: str, keyrings: list[Path],
                              runner: Callable[[list[str], int], Any],
                              output_root: Path, ordinal: int,
                              inline_keyrings: list[dict[str, Any]] | None = None,
                              prepared_keyrings: Mapping[str, Mapping[str, Any]] | None = None,
                              presealed_signatures: Mapping[str, Mapping[str, Any]] | None = None,
                              gpgv_tool: Mapping[str, Any] | None = None,
                              inventory_cache: dict[str, Mapping[str, Any]] | None = None,
                              inventory_ordinals: dict[str, int] | None = None,
                              signature_identity: str | None = None,
                              prepared_keyring_identities: Mapping[str, str] | None = None,
                              identity_registry: dict[str, str] | None = None
                              ) -> dict[str, Any]:
    if not keyrings and not inline_keyrings:
        raise CaptureError("APT_KEYRING_MISSING", str(release))
    copied_keyrings: list[dict[str, Any]] = []
    command_keyrings: list[str] = []
    keyring_identities: list[str] = []
    for key_index, keyring in enumerate(keyrings):
        prepared = (prepared_keyrings or {}).get(str(keyring))
        if prepared is not None:
            copied = _immutable_copy(prepared)
            copied_keyrings.append(copied)
            keyring_identities.append(
                (prepared_keyring_identities or {}).get(str(keyring)) or
                _keyring_identity(str(keyring), {}, copied))
            command_keyrings.append(str(output_root / copied["path"]))
        else:
            key_data, key_descriptor = _container_read(
                keyring, "apt keyring {}".format(key_index), MAX_CAPTURE_FILE_BYTES,
                allow_empty=False, require_root=container_root in (None, Path("/")))
            destination = "{}/{}-{}".format(
                APT_KEYRING_RELATIVE, ordinal, keyring.name)
            copied = _copy_capture_file(output_root, destination, key_data)
            copied_descriptor = _relative_descriptor(output_root, copied)
            copied_keyrings.append({
                "source": str(keyring), "target": destination, "path": destination,
                "kind": "path", "descriptor": copied_descriptor,
                "source_descriptor": key_descriptor,
            })
            keyring_identities.append(_keyring_identity(
                str(keyring), {}, copied_keyrings[-1]))
            command_keyrings.append(str(output_root / destination))
    for inline in inline_keyrings or []:
        copied = _immutable_copy(inline)
        copied_keyrings.append(copied)
        inline_key = "inline:" + str(copied.get("sha256"))
        keyring_identities.append(
            (prepared_keyring_identities or {}).get(inline_key) or
            _keyring_identity(inline_key, copied, copied))
        # Keep the raw and normalized armor as evidence, but pass only the
        # verified binary projection to gpgv.  GnuPG 2.2 treats an ASCII-armored
        # file supplied as a keyring as an invalid packet.
        command_keyrings.append(str(output_root / copied["dearmored_path"]))
    if not command_keyrings:
        raise CaptureError("APT_KEYRING_MISSING", str(release))
    if release_kind == "inrelease":
        signature = release
    else:
        if not release.name.endswith("_Release"):
            raise CaptureError("APT_RELEASE_NAME_INVALID", str(release))
        signature = release.with_name(release.name + ".gpg")
        _container_read(signature, "detached Release signature", MAX_CAPTURE_FILE_BYTES,
                        allow_empty=False,
                        require_root=container_root in (None, Path("/")))
    # gpgv is intentionally invoked with its minimal portable contract.  The
    # image's gpgv does not implement gpg's ``--no-options`` switch; every
    # accepted key is supplied explicitly below, so a default keyring cannot
    # silently widen trust.
    argv = ["gpgv", "--status-fd", "1"]
    for keyring in command_keyrings:
        argv.extend(["--keyring", keyring])
    if release_kind == "inrelease":
        argv.append(str(release))
    else:
        argv.extend([str(signature), str(release)])
    _validate_argv(argv, "Release signature verification")
    result = _normalize_output(runner(argv, 120))
    diagnostics = _write_command_diagnostics(
        output_root, "gpgv-{}".format(ordinal), result[1], result[2],
        identity=signature_identity, identity_registry=identity_registry)
    evidence = {
        "release_kind": release_kind, "release_path": str(release),
        "signature_path": None if release_kind == "inrelease" else str(signature),
        "signature_descriptor": None, "keyrings": copied_keyrings,
        "command": argv, "command_sha256": _command_hash(argv),
        # The sealed diagnostics contain a newline sentinel for an empty
        # stream, so bind the evidence to the bytes that can actually be
        # reopened rather than to an unsealed in-memory empty value.
        "exit_status": result[0],
        "stdout_sha256": diagnostics["stdout"]["sha256"],
        "stderr_sha256": diagnostics["stderr"]["sha256"], "status_fd": 1,
        "timed_out": result[3], "signal": result[4],
        "diagnostics": diagnostics, "status": None,
    }
    if signature_identity is not None:
        evidence["identity"] = signature_identity
    if gpgv_tool is not None:
        evidence["tool"] = dict(gpgv_tool)
    if result[0] != 0 or result[3] or result[4] is not None:
        raise CaptureError("GPGV_VERIFY_FAILED", str(release),
                           signature_evidence=[evidence])
    try:
        status = _parse_gpgv_status(result[1], result[2], str(release))
    except CaptureError as error:
        raise CaptureError(error.kind, str(error), signature_evidence=[evidence]) from error
    status["stderr_sha256"] = evidence["stderr_sha256"]
    evidence["status"] = status
    signature_descriptor = None
    if release_kind == "detached":
        signature_data, _ = _container_read(
            signature, "detached Release signature", MAX_CAPTURE_FILE_BYTES,
            allow_empty=False, require_root=container_root in (None, Path("/")))
        presealed = (presealed_signatures or {}).get(str(signature))
        if presealed is not None:
            target = output_root / presealed["path"]
            current = _relative_descriptor(
                output_root, _descriptor(target, "presealed detached signature",
                                         MAX_CAPTURE_FILE_BYTES))
            if current != dict(presealed) or current["sha256"] != sha256_bytes(signature_data):
                raise CaptureError("DETACHED_SIGNATURE_DRIFT", str(signature))
            signature_descriptor = current
        else:
            copied_signature = _copy_capture_file(
                output_root, "signatures/{}".format(signature.name), signature_data)
            signature_descriptor = _relative_descriptor(output_root, copied_signature)
    signer_fingerprint = status["validsig"]["fingerprint"]
    primary_fingerprint = status["validsig"].get(
        "primary_fingerprint", signer_fingerprint)
    try:
        for inventory_index, item in enumerate(copied_keyrings):
            if item.get("inventory", {}).get("status") == "SYNTHETIC_TEST_ONLY":
                item["expected_signer_fingerprints"] = [signer_fingerprint]
                item["primary_fingerprints"] = [primary_fingerprint]
                item["subkey_fingerprints"] = []
                continue
            key_identity = keyring_identities[inventory_index]
            if inventory_cache is not None and key_identity in inventory_cache:
                cached = _immutable_copy(inventory_cache[key_identity])
                cached_inventory = cached.get("inventory")
                if (not isinstance(cached_inventory, Mapping) or
                        cached_inventory.get("identity") != key_identity):
                    raise CaptureError("GPG_KEY_INVENTORY_IDENTITY_CONFLICT", str(release))
                for field in ("primary_fingerprints", "subkey_fingerprints",
                              "expected_signer_fingerprints", "inventory"):
                    item[field] = cached[field]
                if item["inventory"].get("status") == "FAILED":
                    raise CaptureError(
                        item["inventory"].get("failure_kind", "GPG_KEY_INVENTORY_FAILED"),
                        str(release))
            else:
                if inventory_ordinals is None:
                    inventory_ordinals = {}
                inventory_ordinal = inventory_ordinals.setdefault(
                    key_identity, len(inventory_ordinals))
                try:
                    _key_inventory(
                        runner, output_root, item, inventory_ordinal,
                        container_root, identity=key_identity,
                        identity_registry=identity_registry)
                except CaptureError:
                    if inventory_cache is not None:
                        cached = _immutable_copy(item)
                        cached.pop("signer_fingerprint", None)
                        inventory_cache[key_identity] = cached
                    raise
                if inventory_cache is not None:
                    cached = _immutable_copy(item)
                    cached.pop("signer_fingerprint", None)
                    inventory_cache[key_identity] = cached
            expected = set(item["expected_signer_fingerprints"])
            if signer_fingerprint not in expected or \
                    primary_fingerprint not in set(item["primary_fingerprints"]):
                raise CaptureError("GPGV_SIGNER_NOT_IN_KEYRING", str(release))
    except CaptureError as error:
        raise CaptureError(error.kind, str(error), signature_evidence=[evidence]) from error
    for item in copied_keyrings:
        _bind_verified_keyring_signer(item, signer_fingerprint)
    evidence["signature_descriptor"] = signature_descriptor
    evidence["status"] = status
    evidence["exit_status"] = 0
    if gpgv_tool is not None:
        evidence["tool"] = dict(gpgv_tool)
    return evidence


def _bind_verified_keyring_signer(value: dict[str, Any], signer: str) -> None:
    """Bind one successful gpgv signer to its exact keyring projection."""
    if not isinstance(value, dict) or value.get("kind") not in {"path", "inline"} or \
            not isinstance(signer, str) or not FINGERPRINT_RE.fullmatch(signer):
        raise CaptureError("KEYRING_SIGNER_BINDING_INVALID", "verified keyring")
    value["signer_fingerprint"] = signer
    if value["kind"] == "inline":
        # Successful inline descriptors carry both fields.  The former binds
        # the normalized inline key itself; the latter records the gpgv
        # observation.  Their equality is enforced by the descriptor reader.
        value["fingerprint"] = signer


def _source_manifest_repository(repository: Mapping[str, Any], release_path: str,
                                packages_path: str, release_data: bytes,
                                packages_data: bytes, release_url: str,
                                packages_url: str) -> dict[str, Any]:
    return {
        "url": repository["url"], "release_url": release_url,
        "release_path": release_path, "release_sha256": sha256_bytes(release_data),
        "packages_url": packages_url, "packages_path": packages_path,
        "packages_sha256": sha256_bytes(packages_data),
    }


def _capture_apt_source_phase_impl(output_root: Path, distro: str,
                                   container_root: Path | None,
                                   runner: Callable[[list[str], int], Any],
                                   policy: Mapping[str, Any],
                                   release_evidence: list[dict[str, Any]],
                                   package_evidence: list[dict[str, Any]],
                                   package_fetcher: Callable[..., Any] | None = None) -> dict[str, Any]:
    _validate_apt_source_url_policy(policy)
    source_files = _source_files(container_root)
    _output_directory(output_root, APT_SOURCE_RELATIVE)
    source_records: list[dict[str, Any]] = []
    source_entry_records: list[tuple[str, dict[str, Any]]] = []
    repository_specs: list[dict[str, Any]] = []
    repository_keys: set[tuple[str, str, str]] = set()
    release_records: list[dict[str, Any]] = []
    package_records: list[dict[str, Any]] = []
    presealed_signatures: dict[str, dict[str, Any]] = {}
    keyring_paths: set[str] = set()
    inline_keys: dict[str, bytes] = {}
    source_descriptors: list[dict[str, Any]] = []
    release_cache: dict[str, dict[str, Any]] = {}
    release_capture_identity_by_path: dict[str, str] = {}
    artifact_identity_registry: dict[str, str] = {}
    for source_path, source_data, source_descriptor in source_files:
        logical = source_descriptor["logical_path"]
        parsed, source_keyrings = _source_urls(
            source_data, "apt source " + logical, policy=policy,
            allow_inactive=True)
        destination = APT_SOURCE_RELATIVE + "/" + logical
        source_urls = sorted({url for record in parsed for url in record["uris"]})
        source_record = _source_file_record_v2(
            output_root, source_path, source_data, destination, source_urls)
        source_records.append({key: source_record[key] for key in (
            "path", "role", "urls", "bytes", "sha256")})
        source_descriptors.append({"source": str(source_path),
                                  "descriptor": source_descriptor,
                                  "output": _relative_descriptor(
                                      output_root, _descriptor(
                                          output_root / destination, "apt source output",
                                          MAX_CAPTURE_FILE_BYTES))})
        keyring_paths.update(source_keyrings)
        for parsed_record in parsed:
            resolved_record = dict(parsed_record)
            signed_by = _resolve_source_keyring(
                parsed_record, "apt source " + logical + " record")
            resolved_record["signed_by"] = signed_by
            source_entry_records.append((logical, resolved_record))
            if signed_by["kind"] == "path":
                keyring_paths.add(signed_by["source"])
            if signed_by["kind"] == "inline":
                inline_keys[signed_by["sha256"]] = signed_by
            for component in parsed_record["components"]:
                for uri in parsed_record["uris"]:
                    for suite in parsed_record["suites"]:
                        key = (uri, suite, component)
                        if "deb" not in parsed_record["types"]:
                            continue
                        if key in repository_keys:
                            raise CaptureError("APT_REPOSITORY_DUPLICATE", str(key))
                        repository_keys.add(key)
                        repository = {"url": uri, "suite": suite,
                                      "component": component,
                                      "signed_by": dict(signed_by)}
                        release, release_kind = _discover_release(container_root, repository)
                        release_key = str(release)
                        cached = release_cache.get(release_key)
                        if cached is None:
                            release_data, release_descriptor = _container_read(
                                release, "Release {}".format(release.name),
                                MAX_CAPTURE_FILE_BYTES, allow_empty=False,
                                require_root=container_root in (None, Path("/")))
                            # Preserve signed Release bytes before parsing any
                            # declaration, including malformed declarations.
                            release_url, _ = _repository_urls(
                                repository, release_kind, "plain")
                            release_artifact_identity = _release_artifact_identity(
                                repository, release, release_kind, release_data)
                            previous_release_identity = \
                                release_capture_identity_by_path.get(release_key)
                            if (previous_release_identity is not None and
                                    previous_release_identity != release_artifact_identity):
                                raise CaptureError(
                                    "GPG_RELEASE_IDENTITY_CONFLICT", release_key)
                            release_capture_identity_by_path[release_key] = \
                                release_artifact_identity
                            release_destination = (
                                APT_SOURCE_RELATIVE + "/indices/" +
                                _release_artifact_name(
                                    release.name, release_artifact_identity))
                            _claim_identity_artifact(
                                artifact_identity_registry,
                                release_destination[len(APT_SOURCE_RELATIVE) + 1:],
                                release_artifact_identity, "Release artifact")
                            release_record = _capture_source_file_record(
                                output_root, release, release_data, release_destination,
                                "apt_release", release_url)
                            release_signature = None
                            if release_kind == "detached":
                                if not release.name.endswith("_Release"):
                                    raise CaptureError("APT_RELEASE_NAME_INVALID",
                                                        str(release))
                                signature = release.with_name(release.name + ".gpg")
                                signature_data, _ = _container_read(
                                    signature, "detached Release signature",
                                    MAX_CAPTURE_FILE_BYTES, allow_empty=False,
                                    require_root=container_root in (None, Path("/")))
                                signature_destination = "signatures/{}".format(
                                    _detached_artifact_name(
                                        signature.name, sha256_bytes(signature_data)))
                                _claim_identity_artifact(
                                    artifact_identity_registry,
                                    signature_destination,
                                    sha256_bytes(signature_data),
                                    "detached signature artifact")
                                copied_signature = _copy_capture_file(
                                    output_root, signature_destination, signature_data)
                                signature_descriptor = _relative_descriptor(
                                    output_root, copied_signature)
                                presealed_signatures[str(signature)] = dict(
                                    signature_descriptor)
                                release_signature = {
                                    "path": str(signature), "source": str(signature),
                                    "bytes": len(signature_data),
                                    "sha256": sha256_bytes(signature_data),
                                    "output": signature_descriptor,
                                }
                            source_records.append({key: release_record[key] for key in (
                                "path", "role", "url", "bytes", "sha256")})
                            release_records.append({
                                "path": release_record["path"], "source": str(release),
                                "descriptor": release_descriptor, "kind": release_kind,
                            })
                            release_evidence.append({
                                "path": release_record["path"], "source": str(release),
                                "url": release_url, "repository_url": repository["url"],
                                "kind": release_kind,
                                "identity": release_artifact_identity,
                                "bytes": release_record["bytes"],
                                "sha256": release_record["sha256"],
                                "output": release_record["output"],
                                "signature": release_signature,
                            })
                            cached = _immutable_copy({
                                "release": release, "release_kind": release_kind,
                                "release_data": release_data,
                                "release_descriptor": release_descriptor,
                                "release_record": release_record,
                                "artifact_identity": release_artifact_identity,
                            })
                            release_cache[release_key] = cached
                        else:
                            release_artifact_identity = _release_artifact_identity(
                                repository, release, release_kind,
                                cached["release_data"])
                            if release_artifact_identity != cached.get("artifact_identity"):
                                raise CaptureError(
                                    "GPG_RELEASE_IDENTITY_CONFLICT", release_key)
                            previous_release_identity = \
                                release_capture_identity_by_path.get(release_key)
                            if (previous_release_identity is not None and
                                    previous_release_identity != release_artifact_identity):
                                raise CaptureError(
                                    "GPG_RELEASE_IDENTITY_CONFLICT", release_key)
                            release_capture_identity_by_path[release_key] = \
                                release_artifact_identity
                        current_release_data = bytes(cached["release_data"])
                        current_release_identity = cached["artifact_identity"]
                        if _release_artifact_identity(
                                repository, release, release_kind,
                                current_release_data) != current_release_identity:
                            raise CaptureError(
                                "GPG_RELEASE_IDENTITY_CONFLICT", release_key)
                        repository_specs.append({
                            "repository": repository, "release": release,
                            "release_kind": release_kind,
                            "release_identity": current_release_identity,
                            "release_data": current_release_data,
                            "release_record": _immutable_copy(
                                cached["release_record"]),
                            "signed_by": dict(signed_by),
                        })
    if not repository_specs:
        raise CaptureError("APT_REPOSITORIES_EMPTY", "no Release/Packages pair")
    key_refs: dict[str, dict[str, Any]] = {}
    keyring_descriptors: list[dict[str, Any]] = []
    for index, source_key in enumerate(sorted(keyring_paths)):
        key_path = _container_path(container_root, source_key, "keyring path")
        key_data, _ = _container_read(
            key_path, "apt keyring {}".format(index), MAX_CAPTURE_FILE_BYTES,
            allow_empty=False, require_root=container_root in (None, Path("/")))
        destination = "{}/path-{}-{}".format(
            APT_KEYRING_RELATIVE, index, Path(source_key).name)
        _copy_capture_file(output_root, destination, key_data)
        key_value = {
            "path": destination, "kind": "path", "source": source_key,
            "bytes": len(key_data), "sha256": sha256_bytes(key_data),
            "normalized_armor_bytes": None, "normalized_armor_sha256": None,
            "dearmored_path": destination, "dearmored_bytes": len(key_data),
            "dearmored_sha256": sha256_bytes(key_data),
            "primary_fingerprints": [], "subkey_fingerprints": [],
            "expected_signer_fingerprints": [],
            "inventory": {"status": "SYNTHETIC_TEST_ONLY" if
                           key_data == b"synthetic-keyring\n" else "PENDING"},
        }
        key_refs["path:" + source_key] = key_value
        keyring_descriptors.append(key_value)
    for key_index, key_sha in enumerate(sorted(inline_keys)):
        inline = inline_keys[key_sha]
        key_data = inline["data"]
        destination = "{}/inline-{}.asc".format(APT_KEYRING_RELATIVE, key_sha)
        copied = _copy_capture_file(output_root, destination, key_data)
        descriptor = _relative_descriptor(output_root, copied)
        dearmored_destination = "{}/dearmored-{}.gpg".format(
            APT_KEYRING_RELATIVE, key_sha)
        dearmored = _copy_capture_file(
            output_root, dearmored_destination, inline["dearmored_data"])
        key_value = {
            "path": destination, "kind": "inline", "source": "inline",
            "bytes": len(key_data), "sha256": sha256_bytes(key_data),
            "normalized_armor_bytes": inline["normalized_armor_bytes"],
            "normalized_armor_sha256": inline["normalized_armor_sha256"],
            "dearmored_path": dearmored_destination,
            "dearmored_bytes": inline["dearmored_bytes"],
            "dearmored_sha256": inline["dearmored_sha256"],
            "primary_fingerprints": [], "subkey_fingerprints": [],
            "expected_signer_fingerprints": [],
            "inventory": {"status": "PENDING"},
        }
        if descriptor["sha256"] != key_sha:
            raise CaptureError("APT_INLINE_KEY_DRIFT", destination)
        if (dearmored["sha256"] != inline["dearmored_sha256"] or
                dearmored["bytes"] != inline["dearmored_bytes"]):
            raise CaptureError("APT_INLINE_KEY_DEARMOR_DRIFT", destination)
        key_refs["inline:" + key_sha] = key_value
        keyring_descriptors.append(key_value)
    keyring_descriptors.sort(key=lambda item: item["path"])
    version, version_argv = _gpgv_version(runner)
    gpgv_tool = {"binary": "gpgv", "version": version,
                 "version_argv": version_argv,
                 "version_argv_sha256": _command_hash(version_argv)}
    signatures: list[dict[str, Any]] = []
    repositories: list[dict[str, Any]] = []
    raw_destinations: dict[str, str] = {}
    raw_artifacts_by_identity: dict[str, dict[str, Any]] = {}
    decoded_artifacts_by_identity: dict[str, dict[str, Any]] = {}
    inventory_cache: dict[str, Mapping[str, Any]] = {}
    inventory_ordinals: dict[str, int] = {}
    signature_cache: dict[str, Mapping[str, Any]] = {}
    release_identity_by_path: dict[str, str] = {}
    signature_references: list[dict[str, Any]] = []
    for ordinal, spec in enumerate(repository_specs):
        # Reopen the source path rather than using a descriptor produced by the
        # discovery pass; this makes the verification evidence a real input.
        release_identity, release_record, release_data = \
            _validate_repository_release_spec(spec, release_evidence)
        repository = spec["repository"]
        release = spec["release"]
        release_kind = spec["release_kind"]
        signed_by = spec["signed_by"]
        if signed_by["kind"] == "path":
            source_key = signed_by["source"]
            prepared_key = key_refs.get("path:" + source_key)
            if prepared_key is None:
                raise CaptureError("APT_KEYRING_BINDING_MISSING", source_key)
            key_path = _container_path(container_root, source_key, "keyring path")
            key_paths = [key_path]
            prepared = {str(key_path): prepared_key}
            inline_for_signature = []
        else:
            prepared_key = key_refs.get("inline:" + signed_by["sha256"])
            if prepared_key is None:
                raise CaptureError("APT_KEYRING_BINDING_MISSING", str(release))
            key_paths = []
            prepared = {}
            inline_for_signature = [prepared_key]
        keyring_identity = _keyring_identity(
            (source_key if signed_by["kind"] == "path" else
             "inline:" + str(signed_by["sha256"])),
            signed_by, prepared_key)
        signer_policy = {"declared_fingerprint": signed_by.get("fingerprint")}
        signature_identity = _release_signature_identity(
            repository, release, release_kind, release_record,
            release_data, keyring_identity, signer_policy)
        release_identity_key = str(release)
        previous_identity = release_identity_by_path.get(release_identity_key)
        if previous_identity is not None and previous_identity != signature_identity:
            raise CaptureError("GPG_RELEASE_IDENTITY_CONFLICT", release_identity_key)
        release_identity_by_path[release_identity_key] = signature_identity
        signature_is_new = signature_identity not in signature_cache
        try:
            if signature_is_new:
                try:
                    signature = _verify_release_signature(
                        container_root, release, release_kind, key_paths, runner,
                        output_root, ordinal, inline_for_signature, prepared,
                        presealed_signatures, gpgv_tool, inventory_cache,
                        inventory_ordinals, signature_identity,
                        {source_key if signed_by["kind"] == "path" else
                         "inline:" + str(signed_by["sha256"]): keyring_identity},
                        artifact_identity_registry)
                except CaptureError as error:
                    _cache_signature_failure(signature_cache, signature_identity, error)
                    raise
                _cache_signature_success(signature_cache, signature_identity, signature)
            else:
                cached_signature = _cached_signature(
                    signature_cache, signature_identity, str(release))
                if cached_signature["status"] == "FAILED":
                    raise CaptureError(
                        cached_signature["failure_kind"],
                        cached_signature["failure_message"],
                        signature_evidence=cached_signature["evidence"])
                signature = cached_signature["evidence"]
        except CaptureError as error:
            signature_evidence = getattr(error, "signature_evidence", None)
            if signature_evidence:
                # Preserve any signatures already verified for earlier
                # repositories before attaching the current diagnostics.  A
                # partial receipt must retain the complete evidence prefix.
                for evidence in signature_evidence:
                    evidence["repository"] = {
                        "url": repository["url"],
                        "release_url": _repository_urls(
                            repository, release_kind, "plain")[0],
                        "release_path": release_record["path"],
                        "release_sha256": sha256_bytes(release_data),
                    }
                error.signature_evidence = _merge_signature_evidence(
                    list(signatures), list(signature_evidence))
                error.signature_references = _immutable_copy(signature_references)
                error.signature_count = len({
                    item.get("identity") for item in error.signature_evidence
                    if item.get("identity") is not None})
                error.reference_map, error.reference_counts = _reference_projection(
                    release_evidence, package_evidence, signature_references,
                    error.signature_count)
            raise
        def _attach_partial_evidence(error: CaptureError) -> None:
            """Carry the validated evidence prefix through a later failure."""
            error.release_evidence = list(release_evidence)
            error.package_evidence = list(package_evidence)
            error.signature_evidence = _merge_signature_evidence(
                list(signatures), [signature])
            error.signature_references = _immutable_copy(signature_references)
            error.signature_count = len({
                item.get("identity") for item in error.signature_evidence
                if item.get("identity") is not None})
            error.reference_map, error.reference_counts = _reference_projection(
                error.release_evidence, error.package_evidence,
                error.signature_references, error.signature_count)

        def _attach_release_prefix(error: CaptureError) -> None:
            """Retain only Release records when package validation fails.

            A compressed Packages blob may have been copied before its signed
            binding or decoding was proven.  It therefore cannot be exposed
            as package evidence, and the signer/reference projection must not
            imply that the package graph was verified.
            """
            release_prefix = _immutable_copy(list(release_evidence))
            package_evidence.clear()
            error.release_evidence = release_prefix
            error.package_evidence = []
            error.signature_evidence = []
            error.signature_references = []
            error.signature_count = 0
            error.reference_map, error.reference_counts = _reference_projection(
                release_prefix, [], [], 0)

        release_name = release.name
        try:
            release_entries = _release_sha256_entries(release_data, release_name)
        except CaptureError as error:
            _attach_partial_evidence(error)
            raise
        def persist_raw(evidence: dict[str, Any], raw_data: bytes) -> Mapping[str, Any]:
            artifact = _validate_package_artifact_identity(
                evidence.get("artifact_identity"), "Packages artifact identity")
            if (artifact["packages_url"] != evidence["url"] or
                    artifact["format"] != evidence["format"] or
                    artifact["signed_compressed_path"] != evidence["release_path"] or
                    artifact["compressed_sha256"] != evidence["release_sha256"] or
                    artifact["compressed_bytes"] != evidence["release_bytes"]):
                raise CaptureError("PACKAGES_ARTIFACT_BINDING_INVALID", evidence["url"])
            identity = artifact["identity"]
            basename = Path(artifact["signed_compressed_path"]).name
            if basename not in {"Packages", "Packages.xz", "Packages.gz"}:
                raise CaptureError("PACKAGES_PATH_INVALID", basename)
            destination = "raw-indexes/{}".format(
                _package_artifact_name(basename, identity))
            _claim_identity_artifact(
                artifact_identity_registry, destination, identity,
                "Packages raw artifact")
            previous = raw_artifacts_by_identity.get(identity)
            if previous is not None:
                previous_output = previous.get("output")
                if (not isinstance(previous_output, Mapping) or
                        previous_output.get("path") != destination or
                        previous.get("bytes") != len(raw_data) or
                        previous.get("sha256") != sha256_bytes(raw_data)):
                    raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_CONFLICT", identity)
                return _immutable_copy(previous)
            if raw_destinations.get(destination) not in (None, identity):
                raise CaptureError("PACKAGES_ARTIFACT_IDENTITY_PREFIX_COLLISION", destination)
            copied = _copy_capture_file(
                output_root, destination, raw_data, MAX_CAPTURE_FILE_BYTES)
            raw_destinations[destination] = identity
            completed = dict(evidence)
            completed["artifact_identity"] = artifact
            completed["output"] = _relative_descriptor(output_root, copied)
            raw_artifacts_by_identity[identity] = _immutable_copy(completed)
            package_evidence.append(completed)
            return _immutable_copy(completed)
        try:
            selected = _select_packages_index(
                container_root, repository, release_kind, release_entries, policy,
                package_fetcher, persist_raw,
                release_identity=release_identity,
                release_record=release_record,
                release_data=release_data)
        except CaptureError as error:
            _attach_release_prefix(error)
            raise
        try:
            decoded = selected["decoded"]
            compression = selected["compression"]
            package_relative = selected["package_relative"]
            packages_url = selected["packages_url"]
            package_identity = _validate_package_artifact_identity(
                selected.get("package_identity"), "Packages artifact identity")
        except CaptureError as error:
            _attach_release_prefix(error)
            raise
        package_identity_sha = package_identity["identity"]
        try:
            packages_destination = APT_SOURCE_RELATIVE + "/indices/" + (
                _package_artifact_name(
                    Path(package_identity["signed_plain_path"]).name,
                    package_identity_sha))
            _claim_identity_artifact(
                artifact_identity_registry,
                packages_destination[len(APT_SOURCE_RELATIVE) + 1:],
                package_identity_sha, "Packages decoded artifact")
            packages_record = decoded_artifacts_by_identity.get(package_identity_sha)
            new_decoded = packages_record is None
            if packages_record is not None:
                if (packages_record.get("path") != packages_destination or
                        packages_record.get("bytes") != len(decoded) or
                        packages_record.get("sha256") != sha256_bytes(decoded)):
                    raise CaptureError(
                        "PACKAGES_ARTIFACT_IDENTITY_CONFLICT", package_identity_sha)
                packages_record = _immutable_copy(packages_record)
            else:
                packages_record = _capture_source_file_record(
                    output_root, selected.get("source_descriptor") and
                    Path(selected["source_descriptor"]["path"]) or
                    Path(packages_url), decoded, packages_destination,
                    "apt_packages", packages_url,
                    maximum=MAX_DECODED_PACKAGES_BYTES,
                    allow_empty=package_identity["signed_empty"])
                decoded_artifacts_by_identity[package_identity_sha] = _immutable_copy(
                    packages_record)
        except CaptureError as error:
            _attach_release_prefix(error)
            raise
        completed_raw = dict(selected["raw_evidence"])
        completed_raw["artifact_identity"] = package_identity
        completed_raw.update({
            "release_record_path": release_record["path"],
            "release_file_sha256": sha256_bytes(release_data),
            "release_file_bytes": len(release_data),
            "source_record_path": packages_record["path"],
            "decoded_release_path": package_relative,
            "decoded_release_sha256": selected["decoded_indexed"][0],
            "decoded_release_bytes": selected["decoded_indexed"][1],
            "decoded_output": packages_record["output"],
        })
        for evidence_index, evidence in enumerate(package_evidence):
            evidence_identity = evidence.get("artifact_identity", {}).get("identity")
            if evidence_identity == package_identity_sha:
                package_evidence[evidence_index] = _immutable_copy(completed_raw)
                raw_artifacts_by_identity[package_identity_sha] = _immutable_copy(
                    completed_raw)
                break
        else:
            raise CaptureError("PACKAGES_EVIDENCE_BINDING_MISSING", packages_url)
        packages_record["compressed"] = {
            key: completed_raw[key] for key in (
                "source", "url", "final_url", "format", "release_path", "release_sha256",
                "release_bytes", "bytes", "sha256", "output", "release_record_path",
                "release_file_sha256", "release_file_bytes", "source_record_path",
                "decoded_release_path", "decoded_release_sha256",
                "decoded_release_bytes", "decoded_output", "stream_descriptors",
                "signed_empty", "artifact_identity")}
        if new_decoded:
            source_records.append({key: packages_record[key] for key in (
                "path", "role", "url", "bytes", "sha256", "compressed")})
        release_url = _repository_urls(repository, release_kind, "plain")[0]
        repositories.append(_source_manifest_repository(
            repository, release_record["path"], packages_record["path"],
            release_data, decoded, release_url, packages_url))
        if not any(item.get("path") == packages_record["path"]
                   for item in package_records):
            package_records.append({"path": packages_record["path"],
                                    "source": selected["source_descriptor"] and
                                    selected["source_descriptor"]["path"] or packages_url,
                                    "descriptor": selected["source_descriptor"],
                                    "compression": compression,
                                    "raw_sha256": selected["raw_digest"],
                                    "raw_bytes": selected["raw_size"],
                                    "raw_output": completed_raw["output"]})
        signature["repository"] = {
            "url": repository["url"], "release_url": _repository_urls(
                repository, release_kind, "plain")[0],
            "release_path": release_record["path"],
            "release_sha256": sha256_bytes(release_data),
            "packages_url": packages_url,
            "packages_path": packages_record["path"],
            "packages_sha256": sha256_bytes(decoded),
        }
        # The signature consumer receives a deep copy, but the manifest's
        # canonical descriptor must still retain the first proven inventory
        # observation.  Copy only the command result fields back once; do not
        # expose the shared mutable object to gpgv or a later component.
        for item in signature["keyrings"]:
            if item.get("kind") == "path":
                target = key_refs.get("path:" + str(item.get("source")))
            else:
                target = key_refs.get("inline:" + str(item.get("sha256")))
            if target is not None:
                for field in ("primary_fingerprints", "subkey_fingerprints",
                              "expected_signer_fingerprints", "inventory",
                              "signer_fingerprint"):
                    if field in item:
                        target[field] = _immutable_copy(item[field])
        if signature_is_new:
            signatures.append(signature)
        signer_fingerprint = signature["status"]["validsig"]["fingerprint"]
        signature_references.append(_make_signature_reference(
            ordinal, repository, release, release_kind, release_record,
            release_data, keyring_identity, signer_fingerprint, signer_policy,
            packages_url, packages_record["path"], sha256_bytes(decoded)))
    inline_fingerprints = {
        item["signer_fingerprint"]
        for signature in signatures
        for item in signature["keyrings"]
        if item.get("kind") == "inline"
    }
    inline_descriptors = [item for item in keyring_descriptors
                          if item["kind"] == "inline"]
    if inline_descriptors:
        if len(inline_fingerprints) != 1:
            raise CaptureError("APT_INLINE_KEY_FINGERPRINT_AMBIGUOUS", "inline keyrings")
        inline_fingerprint = inline_fingerprints.pop()
        if not FINGERPRINT_RE.fullmatch(inline_fingerprint):
            raise CaptureError("APT_INLINE_KEY_FINGERPRINT_INVALID", "inline keyrings")
        for item in inline_descriptors:
            item["fingerprint"] = inline_fingerprint
    source_entries = [
        _source_entry_v2(logical, record, record["ordinal"], key_refs)
        for logical, record in source_entry_records]
    source_entries.sort(key=lambda item: (item["path"], item["ordinal"]))
    # rosdep sources and cache are deliberately not part of this phase.  They
    # are materialized by the sealed rosdep_prepare receipt after the network
    # disconnect, so an image without rosdep directories is a valid APT
    # snapshot and cannot make this phase claim premature rosdep evidence.
    source_records.sort(key=lambda item: item["path"])
    if [item["path"] for item in source_records] != sorted(
            item["path"] for item in source_records) or len({item["path"] for item in source_records}) != len(source_records):
        raise CaptureError("SOURCE_MANIFEST_PATH_INVALID", "source records")
    source_descriptors.sort(key=lambda item: item["output"]["path"])
    repositories.sort(key=lambda item: (item["url"], item["release_url"], item["packages_url"]))
    reference_map, reference_counts = _reference_projection(
        release_evidence, package_evidence, signature_references, len(signatures))
    manifest = {
        "schema": SOURCE_MANIFEST_SCHEMA,
        "schema_version": SOURCE_MANIFEST_SCHEMA_VERSION,
        "status": "SEALED_REVIEW_REQUIRED",
        "apt_source_url_policy": dict(policy),
        "release_declaration_policy": dict(RELEASE_DECLARATION_POLICY),
        "packages_format_policy": dict(PACKAGES_FORMAT_POLICY),
        "gpgv_status_policy": dict(GPGV_STATUS_POLICY),
        "files": source_records, "source_entries": source_entries,
        "keyrings": keyring_descriptors, "source_descriptors": source_descriptors,
        "repositories": repositories,
    }
    manifest["canonical_sha256"] = canonical_hash(manifest)
    manifest_pair = _seal_pair(output_root, "source-manifest.json", manifest)
    signature_value = {
        "schema": "registration-plugin-capture-signature-evidence-v1",
        "schema_version": 1, "status": "CAPTURED_REVIEW_REQUIRED",
        "distro": distro, "tool": gpgv_tool,
        "repositories": _sorted_signature_evidence(signatures),
        "signature_references": _immutable_copy(signature_references),
        "signature_count": len(signatures),
        "reference_map": reference_map,
        "reference_counts": reference_counts,
    }
    signature_value["canonical_sha256"] = canonical_hash(signature_value)
    signature_pair = _seal_pair(output_root, "signature-evidence.json", signature_value)
    return {
        "schema": IN_CONTAINER_SCHEMA, "schema_version": IN_CONTAINER_SCHEMA_VERSION,
        "status": "CAPTURED_REVIEW_REQUIRED", "phase": "apt-source", "distro": distro,
        "source_manifest": {"path": "source-manifest.json",
                             "sha256": manifest_pair["file"]["sha256"]},
        "signature_evidence": {"path": "signature-evidence.json",
                                "sha256": signature_pair["file"]["sha256"]},
        "apt_source_files": source_descriptors,
        "source_entries": source_entries,
        "release_files": release_records, "packages_files": package_records,
        "release_evidence": list(release_evidence),
        "package_evidence": list(package_evidence),
        "signature_references": _immutable_copy(signature_references),
        "signature_count": len(signatures),
        "reference_map": reference_map,
        "reference_counts": reference_counts,
        "keyring_paths": sorted(keyring_paths), "repositories": repositories,
    }


def _capture_apt_source_phase(output_root: Path, distro: str,
                              container_root: Path | None,
                              runner: Callable[[list[str], int], Any],
                              policy: Mapping[str, Any],
                              package_fetcher: Callable[..., Any] | None = None
                              ) -> dict[str, Any]:
    """Capture APT sources while retaining raw Release evidence on failure."""
    release_evidence: list[dict[str, Any]] = []
    package_evidence: list[dict[str, Any]] = []
    try:
        result = _capture_apt_source_phase_impl(
            output_root, distro, container_root, runner, policy, release_evidence,
            package_evidence, package_fetcher)
    except CaptureError as error:
        if release_evidence or package_evidence or getattr(error, "signature_evidence", None):
            signature_evidence = _merge_signature_evidence(
                list(getattr(error, "signature_evidence", None) or []))
            signature_references = _immutable_copy(
                getattr(error, "signature_references", None) or [])
            signature_count = len({
                item.get("identity") for item in signature_evidence
                if isinstance(item, Mapping) and item.get("identity") is not None})
            reference_map, reference_counts = _reference_projection(
                release_evidence, package_evidence, signature_references,
                signature_count)
            raise CaptureError(
                error.kind, str(error), release_evidence=sorted(
                    release_evidence, key=lambda item: item["path"]),
                package_evidence=sorted(
                    package_evidence, key=lambda item: item["output"]["path"]),
                signature_evidence=signature_evidence,
                signature_references=signature_references,
                signature_count=signature_count,
                reference_map=reference_map,
                reference_counts=reference_counts) from error
        raise
    result["release_evidence"] = sorted(
        release_evidence, key=lambda item: item["path"])
    result["package_evidence"] = sorted(
        package_evidence, key=lambda item: item["output"]["path"])
    result["signature_references"] = _immutable_copy(
        result.get("signature_references", []))
    result["signature_count"] = result.get(
        "signature_count", len(result.get("signature_references", [])))
    result["reference_map"], result["reference_counts"] = _reference_projection(
        result.get("release_evidence", []), result.get("package_evidence", []),
        result.get("signature_references", []), result["signature_count"])
    return result


def _validate_v2_source_structure(manifest: Mapping[str, Any], output_root: Path,
                                  source_root: Path, files: list[Any],
                                  policy: Mapping[str, Any] | None) -> None:
    """Reparse each captured source file and compare every normalized record."""
    apt_source_items = [item for item in files
                        if isinstance(item, Mapping) and item.get("role") == "apt_source"]
    source_files: dict[str, Mapping[str, Any]] = {}
    for item in apt_source_items:
        path = item.get("path")
        if not isinstance(path, str) or path in source_files:
            raise CaptureError("SOURCE_MANIFEST_SOURCE_DUPLICATE", "apt_source")
        _safe_rel(path, "source manifest apt source path")
        if set(item) != {"path", "role", "urls", "bytes", "sha256"} or \
                not isinstance(item["urls"], list) or \
                item["urls"] != sorted(set(item["urls"])):
            raise CaptureError("SOURCE_MANIFEST_SOURCE_INVALID", path)
        if type(item["bytes"]) is not int or not 0 < item["bytes"] <= MAX_CAPTURE_FILE_BYTES:
            raise CaptureError("SOURCE_MANIFEST_SOURCE_INVALID", path)
        _sha(item["sha256"], "source manifest apt source SHA")
        for url in item["urls"]:
            _capture_url(url, "source manifest apt source URL", policy=policy)
        source_files[path] = item
    if not source_files:
        raise CaptureError("SOURCE_MANIFEST_SOURCE_EMPTY", "apt_source")
    source_file_paths = list(source_files)
    if source_file_paths != sorted(source_file_paths):
        raise CaptureError("SOURCE_MANIFEST_SOURCES_UNSORTED", "files")
    package_files = [item for item in files
                     if isinstance(item, Mapping) and item.get("role") == "apt_packages"]
    if not package_files:
        raise CaptureError("SOURCE_MANIFEST_PACKAGES_EMPTY", "apt_packages")
    package_paths: set[str] = set()
    compressed_outputs: set[str] = set()
    for item in package_files:
        path = item["path"]
        if path in package_paths:
            raise CaptureError("SOURCE_MANIFEST_PACKAGES_DUPLICATE", path)
        package_paths.add(path)
        compressed = item["compressed"]
        _validate_package_evidence(
            output_root, [compressed], require_nonempty=True,
            require_decoded=True,
            label="source manifest compressed Packages " + path)
        if compressed["source_record_path"] != path or \
                compressed["decoded_output"]["path"] != APT_SOURCE_RELATIVE + "/" + path:
            raise CaptureError("SOURCE_MANIFEST_PACKAGES_BINDING_INVALID", path)
        raw_path = compressed["output"]["path"]
        if raw_path in compressed_outputs:
            raise CaptureError("SOURCE_MANIFEST_PACKAGES_DUPLICATE", raw_path)
        compressed_outputs.add(raw_path)
    keyrings = manifest.get("keyrings")
    if not isinstance(keyrings, list) or not keyrings:
        raise CaptureError("SOURCE_MANIFEST_KEYRINGS_EMPTY", "keyrings")
    key_map: dict[str, Mapping[str, Any]] = {}
    keyring_refs: set[str] = set()
    key_paths: set[str] = set()
    for item in keyrings:
        if not isinstance(item, Mapping):
            raise CaptureError("SOURCE_MANIFEST_KEYRING_INVALID", "descriptor")
        item = _validate_keyring_descriptor(
            output_root, item, "source manifest keyring descriptor")
        path = _safe_rel(item["path"], "keyring artifact path")
        if not path.startswith(APT_KEYRING_RELATIVE + "/") or path in key_paths:
            raise CaptureError("SOURCE_MANIFEST_KEYRING_INVALID", path)
        if item["kind"] == "path":
            path_key = "path:" + str(item["source"])
            if path_key in key_map:
                raise CaptureError("SOURCE_MANIFEST_KEYRING_DUPLICATE", path)
            key_map[path_key] = item
            keyring_refs.add(path_key)
        else:
            inline_key = "inline:" + item["sha256"]
            if inline_key in key_map:
                raise CaptureError("SOURCE_MANIFEST_KEYRING_DUPLICATE", path)
            key_map[inline_key] = item
            keyring_refs.add(inline_key)
        key_paths.add(path)
    if [item["path"] for item in keyrings] != sorted(key_paths):
        raise CaptureError("SOURCE_MANIFEST_KEYRINGS_UNSORTED", "keyrings")
    descriptors = manifest.get("source_descriptors")
    if not isinstance(descriptors, list) or not descriptors:
        raise CaptureError("SOURCE_MANIFEST_DESCRIPTORS_EMPTY", "source_descriptors")
    descriptor_by_output: dict[str, Mapping[str, Any]] = {}
    descriptor_sources: set[str] = set()
    for item in descriptors:
        if not isinstance(item, Mapping) or set(item) != {"source", "descriptor", "output"}:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", "fields")
        source = item["source"]
        if not isinstance(source, str) or not source.startswith("/") or \
                Path(source).as_posix() != source:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", "source")
        if source in descriptor_sources:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_DUPLICATE", source)
        descriptor_sources.add(source)
        descriptor = item["descriptor"]
        output = item["output"]
        if not isinstance(descriptor, Mapping) or not isinstance(output, Mapping):
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", source)
        if descriptor.get("path") != source or descriptor.get("kind") not in {
                "regular", "exact_symlink"}:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", source)
        if descriptor.get("kind") == "regular":
            if set(descriptor) != {"path", "kind", "logical_path", "bytes", "sha256",
                                   "mode", "uid", "gid", "nlink", "device", "inode"}:
                raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", source)
            _validate_descriptor_stat(
                {key: descriptor[key] for key in ("path", "bytes", "sha256", "mode",
                                                   "uid", "gid", "nlink", "device", "inode")},
                source, absolute_path=True)
            _safe_rel(descriptor["logical_path"], "source descriptor logical path")
        else:
            if set(descriptor) != {"path", "kind", "logical_path", "target",
                                   "target_descriptor", "bytes", "sha256", "mode", "uid",
                                   "gid", "nlink", "device", "inode"} or \
                    descriptor["target"] != "/usr/share/ros-apt-source/ros2.sources":
                raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", source)
            _validate_descriptor_stat(
                {key: descriptor[key] for key in ("path", "bytes", "sha256", "mode",
                                                   "uid", "gid", "nlink", "device", "inode")},
                source, absolute_path=True, exact_symlink=True)
            _safe_rel(descriptor["logical_path"], "source descriptor logical path")
            target_descriptor = descriptor["target_descriptor"]
            if not isinstance(target_descriptor, Mapping) or \
                    target_descriptor.get("kind") != "regular" or \
                    set(target_descriptor) != {"path", "kind", "bytes", "sha256", "mode",
                                               "uid", "gid", "nlink", "device", "inode"}:
                raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", source)
            _validate_descriptor_stat(
                {key: target_descriptor[key] for key in ("path", "bytes", "sha256", "mode",
                                                         "uid", "gid", "nlink", "device", "inode")},
                source + " target", absolute_path=True)
            if not str(target_descriptor["path"]).endswith(
                    "/usr/share/ros-apt-source/ros2.sources"):
                raise CaptureError("SOURCE_MANIFEST_SYMLINK_TARGET_INVALID", source)
        if not isinstance(output, Mapping) or set(output) != {
                "path", "bytes", "sha256", "mode", "uid", "gid", "nlink", "device", "inode"}:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", source)
        output_path = _safe_rel(output["path"], "source output descriptor path")
        if not output_path.startswith(APT_SOURCE_RELATIVE + "/") or \
                descriptor.get("logical_path") != output_path[
                    len(APT_SOURCE_RELATIVE) + 1:]:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", source)
        if output_path in descriptor_by_output:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_DUPLICATE", output_path)
        _validate_descriptor_stat(output, source + " output", absolute_path=False)
        if output["mode"] != 0o444:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_INVALID", source)
        descriptor_by_output[output_path] = item
    if list(descriptor_by_output) != sorted(descriptor_by_output):
        raise CaptureError("SOURCE_MANIFEST_DESCRIPTORS_UNSORTED", "source_descriptors")
    expected_outputs = {APT_SOURCE_RELATIVE + "/" + path for path in source_files}
    if set(descriptor_by_output) != expected_outputs:
        raise CaptureError("SOURCE_MANIFEST_DESCRIPTORS_SET_DRIFT", "source_descriptors")
    for source_item in source_files.values():
        expected_output = APT_SOURCE_RELATIVE + "/" + source_item["path"]
        descriptor_item = descriptor_by_output.get(expected_output)
        if descriptor_item is None:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_MISSING", source_item["path"])
        descriptor = descriptor_item["descriptor"]
        output = descriptor_item["output"]
        if descriptor["bytes"] != source_item["bytes"] or \
                descriptor["sha256"] != source_item["sha256"] or \
                output["bytes"] != source_item["bytes"] or \
                output["sha256"] != source_item["sha256"]:
            raise CaptureError("SOURCE_MANIFEST_DESCRIPTOR_DRIFT", source_item["path"])
        if descriptor["kind"] == "exact_symlink":
            target_descriptor = descriptor["target_descriptor"]
            if target_descriptor["bytes"] != source_item["bytes"] or \
                    target_descriptor["sha256"] != source_item["sha256"]:
                raise CaptureError("SOURCE_MANIFEST_SYMLINK_TARGET_DRIFT", source_item["path"])
        _reopen_v2_source_descriptor(descriptor_item)
        try:
            output_data, output_info = _read_bounded(
                output_root / output["path"], "source output " + source_item["path"],
                MAX_CAPTURE_FILE_BYTES, allow_empty=False)
        except CaptureError:
            raise
        output_actual = {
            "path": output["path"], "bytes": len(output_data),
            "sha256": sha256_bytes(output_data), "mode": stat.S_IMODE(output_info.st_mode),
            "uid": output_info.st_uid, "gid": output_info.st_gid,
            "nlink": output_info.st_nlink, "device": output_info.st_dev,
            "inode": output_info.st_ino,
        }
        if output_actual != dict(output):
            raise CaptureError("SOURCE_MANIFEST_OUTPUT_METADATA_DRIFT", source_item["path"])
    entries = manifest.get("source_entries")
    if not isinstance(entries, list) or not entries:
        raise CaptureError("SOURCE_MANIFEST_ENTRIES_EMPTY", "source_entries")
    expected_by_path: dict[str, list[dict[str, Any]]] = {}
    referenced_keyrings: set[str] = set()
    for path, item in sorted(source_files.items()):
        source_data, _ = _read_bounded(source_root / path, "source entry bytes " + path,
                                       MAX_CAPTURE_FILE_BYTES, allow_empty=False)
        line_count = len(source_data.splitlines())
        parsed, _ = _source_urls(
            source_data, "source entries " + path, policy=policy,
            allow_inactive=True)
        expected: list[dict[str, Any]] = []
        for ordinal, record in enumerate(parsed):
            if (type(record["line_start"]) is not int or
                    type(record["line_end"]) is not int or
                    not 1 <= record["line_start"] <= record["line_end"] <= line_count):
                raise CaptureError("SOURCE_MANIFEST_LINE_PROVENANCE_INVALID", path)
            resolved_record = dict(record)
            resolved_record["signed_by"] = _resolve_source_keyring(
                record, "source entries " + path)
            signed_by = dict(resolved_record["signed_by"])
            for field in ("data", "dearmored_data", "normalized_armor_bytes",
                          "normalized_armor_sha256", "dearmored_bytes",
                          "dearmored_sha256"):
                signed_by.pop(field, None)
            reference = None
            if signed_by["kind"] == "path":
                reference = key_map.get("path:" + signed_by["source"])
                referenced_keyrings.add("path:" + signed_by["source"])
            elif signed_by["kind"] == "inline":
                reference = key_map.get("inline:" + signed_by["sha256"])
                referenced_keyrings.add("inline:" + signed_by["sha256"])
            if signed_by["kind"] != "none" and reference is None:
                raise CaptureError("SOURCE_MANIFEST_KEYRING_BINDING_MISSING", path)
            if reference is not None:
                signed_by.update({"artifact_path": reference["path"],
                                  "bytes": reference["bytes"],
                                  "sha256": reference["sha256"]})
                if signed_by["kind"] == "inline":
                    signed_by["fingerprint"] = reference["fingerprint"]
            expected.append({"path": path, "ordinal": ordinal,
                             "line_start": record["line_start"],
                             "line_end": record["line_end"],
                             "format": record["format"], "types": list(record["types"]),
                             "uris": list(record["uris"]), "suites": list(record["suites"]),
                             "components": list(record["components"]),
                             "options": [dict(value) for value in record["options"]],
                             "signed_by": signed_by})
        expected_by_path[path] = expected
        source_urls = sorted({url for record in parsed for url in record["uris"]})
        if item["urls"] != source_urls:
            raise CaptureError("SOURCE_MANIFEST_URL_SET_DRIFT", path)
    observed_entries = list(entries)
    if observed_entries != sorted(observed_entries,
                                  key=lambda item: (item.get("path", ""),
                                                     item.get("ordinal", -1))):
        raise CaptureError("SOURCE_MANIFEST_ENTRIES_UNSORTED", "source_entries")
    for entry in observed_entries:
        if not isinstance(entry, Mapping) or set(entry) != {
                "path", "ordinal", "line_start", "line_end", "format", "types", "uris", "suites",
                "components", "options", "signed_by"}:
            raise CaptureError("SOURCE_MANIFEST_ENTRY_INVALID", "fields")
        path = _safe_rel(entry["path"], "source entry path")
        if path not in expected_by_path or type(entry["ordinal"]) is not int or \
                type(entry["line_start"]) is not int or type(entry["line_end"]) is not int or \
                entry["line_start"] < 1 or entry["line_end"] < entry["line_start"]:
            raise CaptureError("SOURCE_MANIFEST_ENTRY_INVALID", path)
        signed_by = entry["signed_by"]
        if not isinstance(signed_by, Mapping) or signed_by.get("kind") not in {
                "none", "path", "inline"}:
            raise CaptureError("SOURCE_MANIFEST_ENTRY_KEY_INVALID", path)
    expected_entries = [item for path in sorted(expected_by_path)
                        for item in expected_by_path[path]]
    if observed_entries != expected_entries:
        raise CaptureError("SOURCE_MANIFEST_ENTRIES_DRIFT", "source_entries")
    if referenced_keyrings != keyring_refs:
        raise CaptureError("SOURCE_MANIFEST_KEYRING_SET_DRIFT", "keyrings")


def _repository_index_storage_key(base: str, release_url: str,
                                  packages_url: str) -> str:
    """Return the collision-free storage key for one signed component index."""
    return "\n".join((base, release_url, packages_url))


def _read_source_manifest(output_root: Path) -> tuple[dict[str, Any], dict[str, dict[str, Any]]]:
    path = output_root / "source-manifest.json"
    data, _ = _read_bounded(path, "captured source manifest", MAX_JSON_BYTES,
                             allow_empty=False)
    try:
        manifest = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError("SOURCE_MANIFEST_INVALID", str(error)) from error
    required = {"schema", "schema_version", "status", "files", "repositories",
                "canonical_sha256"}
    is_v2 = isinstance(manifest, Mapping) and manifest.get("schema") == SOURCE_MANIFEST_SCHEMA
    if is_v2:
        required |= {"source_entries", "keyrings", "source_descriptors",
                     "apt_source_url_policy", "release_declaration_policy",
                     "packages_format_policy", "gpgv_status_policy"}
    allowed = required | ({"apt_source_url_policy"} if not is_v2 else set())
    if (not isinstance(manifest, Mapping) or not required.issubset(set(manifest)) or
            set(manifest) - allowed or
            (manifest["schema"] not in {
                "glim_clean_room_r3_apt_source_snapshot_v1", SOURCE_MANIFEST_SCHEMA}) or
            (manifest["schema_version"] != (SOURCE_MANIFEST_SCHEMA_VERSION if is_v2 else 1)) or
            manifest["status"] != "SEALED_REVIEW_REQUIRED" or
            manifest["canonical_sha256"] != canonical_hash(manifest)):
        raise CaptureError("SOURCE_MANIFEST_INVALID", "schema/status/hash")
    policy = manifest.get("apt_source_url_policy")
    if policy is not None:
        _validate_apt_source_url_policy(policy, "source manifest URL policy")
    if is_v2 and manifest.get("release_declaration_policy") != RELEASE_DECLARATION_POLICY:
        raise CaptureError("SOURCE_MANIFEST_RELEASE_POLICY_INVALID", "release declarations")
    if is_v2 and manifest.get("packages_format_policy") != PACKAGES_FORMAT_POLICY:
        raise CaptureError("SOURCE_MANIFEST_PACKAGES_POLICY_INVALID", "Packages formats")
    if is_v2 and manifest.get("gpgv_status_policy") != GPGV_STATUS_POLICY:
        raise CaptureError("SOURCE_MANIFEST_GPGV_POLICY_INVALID", "gpgv status policy")
    files = manifest["files"]
    if not isinstance(files, list) or not files:
        raise CaptureError("SOURCE_MANIFEST_INVALID", "files")
    source_root = output_root / APT_SOURCE_RELATIVE
    records: dict[str, dict[str, Any]] = {}
    roles: set[str] = set()
    file_paths: list[str] = []
    for item in files:
        if is_v2 and isinstance(item, Mapping) and item.get("role") == "apt_source":
            expected_fields = {"path", "role", "urls", "bytes", "sha256"}
        elif is_v2 and isinstance(item, Mapping) and item.get("role") == "apt_packages":
            expected_fields = {"path", "role", "url", "bytes", "sha256", "compressed"}
        else:
            expected_fields = {"path", "role", "url", "bytes", "sha256"}
        if not isinstance(item, Mapping) or set(item) != expected_fields:
            raise CaptureError("SOURCE_MANIFEST_INVALID", "file fields")
        relative = _safe_rel(item["path"], "source manifest file")
        if relative in records:
            raise CaptureError("SOURCE_MANIFEST_DUPLICATE", relative)
        file_paths.append(relative)
        role = item["role"]
        allowed_roles = {"apt_source", "apt_release", "apt_packages"}
        if not is_v2:
            # The older GLIM snapshot contract still carries its legacy
            # rosdep source records.  The registration-plugin v2 contract is
            # intentionally narrower and rejects those records below.
            allowed_roles.add("rosdep_source")
        if role not in allowed_roles:
            raise CaptureError("SOURCE_MANIFEST_ROLE_INVALID", str(role))
        urls = item["urls"] if role == "apt_source" and is_v2 else [item["url"]]
        allow_inactive_source = is_v2 and role == "apt_source"
        if (not isinstance(urls, list) or
                (not urls and not allow_inactive_source) or
                urls != sorted(set(urls))):
            raise CaptureError("SOURCE_MANIFEST_URLS_INVALID", relative)
        for url in urls:
            if url.startswith("http://") and policy is None:
                raise CaptureError("APT_URL_POLICY_MISSING", relative)
            if role == "rosdep_source":
                _https_url(url, "source manifest URL")
            else:
                _capture_url(url, "source manifest URL", policy=policy)
        allow_empty_file = (
            is_v2 and role == "apt_packages" and
            isinstance(item.get("compressed"), Mapping) and
            item["compressed"].get("signed_empty") is True)
        maximum = (MAX_DECODED_PACKAGES_BYTES
                   if is_v2 and role == "apt_packages"
                   else MAX_CAPTURE_FILE_BYTES)
        if (type(item["bytes"]) is not int or item["bytes"] < 0 or
                (item["bytes"] == 0 and not allow_empty_file) or
                item["bytes"] > maximum):
            raise CaptureError("SOURCE_MANIFEST_SIZE_INVALID", relative)
        _sha(item["sha256"], "source manifest SHA")
        source_path = source_root / relative
        actual, _ = _read_bounded(
            source_path, "source snapshot " + relative,
            maximum, allow_empty=allow_empty_file)
        if len(actual) != item["bytes"] or sha256_bytes(actual) != item["sha256"]:
            raise CaptureError("SOURCE_MANIFEST_FILE_DRIFT", relative)
        if is_v2 and role == "apt_packages":
            _validate_package_evidence(
                output_root, [item["compressed"]], require_nonempty=True,
                require_decoded=True,
                label="source manifest compressed Packages " + relative)
        records[relative] = dict(item)
        roles.add(role)
    if file_paths != sorted(file_paths):
        raise CaptureError("SOURCE_MANIFEST_UNSORTED", "files")
    required_roles = {"apt_source", "apt_release", "apt_packages"}
    if not is_v2:
        required_roles.add("rosdep_source")
    if roles != required_roles:
        raise CaptureError("SOURCE_MANIFEST_ROLES_INCOMPLETE", str(roles))
    if is_v2:
        _validate_v2_source_structure(manifest, output_root, source_root, files, policy)
    repositories = manifest["repositories"]
    if not isinstance(repositories, list) or not repositories:
        raise CaptureError("SOURCE_MANIFEST_REPOSITORIES_EMPTY", "repositories")
    package_files = {
        item["path"]: item for item in files
        if is_v2 and isinstance(item, Mapping) and item.get("role") == "apt_packages"
    }
    index_records: dict[str, dict[str, Any]] = {}
    repository_keys = []
    for item in repositories:
        required_repo = {"url", "release_url", "release_path", "release_sha256",
                         "packages_url", "packages_path", "packages_sha256"}
        if not isinstance(item, Mapping) or set(item) != required_repo:
            raise CaptureError("SOURCE_MANIFEST_REPOSITORY_INVALID", "fields")
        if any(str(item[key]).startswith("http://") for key in (
                "url", "release_url", "packages_url")) and policy is None:
            raise CaptureError("APT_URL_POLICY_MISSING", "repository")
        base = _capture_url(item["url"], "repository URL", policy=policy)
        release_url = _capture_url(
            item["release_url"], "repository Release URL", policy=policy)
        packages_url = _capture_url(
            item["packages_url"], "repository Packages URL", policy=policy)
        release_path = _safe_rel(item["release_path"], "repository Release path")
        packages_path = _safe_rel(item["packages_path"], "repository Packages path")
        _sha(item["release_sha256"], "repository Release SHA")
        _sha(item["packages_sha256"], "repository Packages SHA")
        package_file = package_files.get(packages_path) if is_v2 else None
        if is_v2 and (package_file is None or package_file["url"] != packages_url or
                      package_file["sha256"] != item["packages_sha256"]):
            raise CaptureError("SOURCE_MANIFEST_PACKAGES_BINDING_DRIFT", base)
        release_data, _ = _read_bounded(source_root / release_path,
                                        "repository Release", MAX_CAPTURE_FILE_BYTES,
                                        allow_empty=False)
        packages_allow_empty = bool(
            is_v2 and package_file is not None and
            package_file["compressed"].get("signed_empty") is True)
        packages_data, _ = _read_bounded(
            source_root / packages_path, "repository Packages",
            MAX_DECODED_PACKAGES_BYTES, allow_empty=packages_allow_empty)
        if (sha256_bytes(release_data) != item["release_sha256"] or
                sha256_bytes(packages_data) != item["packages_sha256"]):
            raise CaptureError("SOURCE_MANIFEST_REPOSITORY_DRIFT", base)
        release_parts = urlsplit(release_url)
        packages_parts = urlsplit(packages_url)
        release_base = release_parts.path.rsplit("/", 1)[0].rstrip("/") + "/"
        if release_parts.netloc != packages_parts.netloc or \
                not packages_parts.path.startswith(release_base):
            raise CaptureError("SOURCE_MANIFEST_URL_RELATION_INVALID", base)
        release_relative = packages_parts.path[len(release_base):]
        _safe_rel(release_relative, "repository Packages Release path")
        release_entries = _release_sha256_entries(release_data, base)
        if is_v2:
            compressed = package_file["compressed"]
            artifact_identity = _validate_package_artifact_identity(
                compressed.get("artifact_identity"),
                "source manifest Packages identity " + base)
            repository_identity = {
                "url": item["url"], "suite": artifact_identity["suite"],
                "component": artifact_identity["component"],
            }
            expected_direct_url = _repository_urls(
                repository_identity, artifact_identity["release_kind"],
                compressed["format"])[1]
            expected_packages_url = (
                _packages_by_hash_url(repository_identity, compressed["sha256"])
                if _release_acquire_by_hash(release_data, base)
                else expected_direct_url
            )
            if packages_url != expected_packages_url:
                raise CaptureError("SOURCE_MANIFEST_PACKAGES_BINDING_DRIFT", base)
            release_relative = compressed["release_path"]
            if compressed["release_record_path"] != release_path or \
                    compressed["release_file_sha256"] != item["release_sha256"] or \
                    compressed["release_file_bytes"] != len(release_data) or \
                    compressed["release_path"] != release_relative:
                raise CaptureError("SOURCE_MANIFEST_PACKAGES_BINDING_DRIFT", base)
            indexed = release_entries.get(compressed["release_path"])
            if indexed != (compressed["sha256"], compressed["bytes"]):
                raise CaptureError("SOURCE_MANIFEST_RELEASE_BINDING_DRIFT", base)
            plain_relative = compressed["decoded_release_path"]
            if compressed["format"] == "plain":
                expected_plain_relative = release_relative
            else:
                suffix = "." + compressed["format"]
                if not release_relative.endswith(suffix):
                    raise CaptureError("SOURCE_MANIFEST_PACKAGES_BINDING_DRIFT", base)
                expected_plain_relative = release_relative[:-len(suffix)]
            if plain_relative != expected_plain_relative:
                raise CaptureError("SOURCE_MANIFEST_PACKAGES_BINDING_DRIFT", base)
            plain_indexed = release_entries.get(plain_relative)
            if plain_indexed != (sha256_bytes(packages_data), len(packages_data)) or \
                    plain_indexed != (compressed["decoded_release_sha256"],
                                      compressed["decoded_release_bytes"]):
                raise CaptureError("SOURCE_MANIFEST_RELEASE_BINDING_DRIFT", base)
        else:
            indexed = release_entries.get(release_relative)
            if indexed != (sha256_bytes(packages_data), len(packages_data)):
                raise CaptureError("SOURCE_MANIFEST_RELEASE_BINDING_DRIFT", base)
        package_index = _packages_records(
            packages_data, base, allow_empty=packages_allow_empty)
        key = (base, release_url, packages_url)
        storage_key = _repository_index_storage_key(*key)
        if storage_key in index_records:
            raise CaptureError("SOURCE_MANIFEST_REPOSITORY_DUPLICATE", base)
        repository_keys.append(key)
        index_records[storage_key] = {
            "repository": dict(item), "packages": package_index,
        }
    if repository_keys != sorted(repository_keys):
        raise CaptureError("SOURCE_MANIFEST_REPOSITORIES_UNSORTED", "repositories")
    return dict(manifest), index_records


def _read_signature_evidence(output_root: Path, distro: str,
                             manifest: Mapping[str, Any]) -> dict[str, Any]:
    path = output_root / "signature-evidence.json"
    data, _ = _read_bounded(path, "signature evidence", MAX_JSON_BYTES, allow_empty=False)
    _descriptor(path, "signature evidence", MAX_JSON_BYTES, sidecar=True)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError("SIGNATURE_EVIDENCE_INVALID", "JSON") from error
    required = {"schema", "schema_version", "status", "distro", "tool",
                "repositories", "signature_references", "signature_count",
                "reference_map", "reference_counts",
                "canonical_sha256"}
    if (not isinstance(value, Mapping) or set(value) != required or
            value["schema"] != "registration-plugin-capture-signature-evidence-v1" or
            value["schema_version"] != 1 or value["status"] != "CAPTURED_REVIEW_REQUIRED" or
            value["distro"] != distro or value["canonical_sha256"] != canonical_hash(value)):
        raise CaptureError("SIGNATURE_EVIDENCE_INVALID", "schema/status/hash")
    tool = value["tool"]
    if (not isinstance(tool, Mapping) or set(tool) != {
            "binary", "version", "version_argv", "version_argv_sha256"} or
            tool["binary"] != "gpgv" or not isinstance(tool["version"], str) or
            VERSION_RE.fullmatch(tool["version"]) is None or
            tool["version_argv"] != ["gpgv", "--version"] or
            tool["version_argv_sha256"] != _command_hash(tool["version_argv"])):
        raise CaptureError("SIGNATURE_TOOL_INVALID", "tool descriptor")
    repositories = value["repositories"]
    if not isinstance(repositories, list) or not repositories:
        raise CaptureError("SIGNATURE_EVIDENCE_INVALID", "repositories")
    validated_signatures = _validate_gpgv_evidence(
        output_root, repositories, require_nonempty=True,
        label="signature repository evidence")
    references = _validate_signature_references(
        value["signature_references"], value["signature_count"],
        validated_signatures, require_complete=True,
        label="signature repository references")
    if value["signature_count"] != len(validated_signatures):
        raise CaptureError("SIGNATURE_REFERENCE_COUNT_INVALID", "signature evidence")
    # The signature document carries the complete repository projection, but
    # the deduplicated Release/Packages objects are owned by the preceding
    # apt-source capture.  Reopen that sealed document here so a projection
    # cannot be made self-consistent from references alone.  This also keeps a
    # shared Release object represented once while each repository reference
    # points at its full identity.
    apt_path = output_root / "in-container-apt-source.json"
    release_evidence: list[dict[str, Any]] = []
    package_evidence: list[dict[str, Any]] = []
    if apt_path.exists() or apt_path.is_symlink():
        apt_data, _ = _read_bounded(
            apt_path, "apt source capture", MAX_JSON_BYTES, allow_empty=False)
        _descriptor(apt_path, "apt source capture", MAX_JSON_BYTES, sidecar=True)
        try:
            apt_value = json.loads(apt_data.decode("utf-8"))
        except (UnicodeError, json.JSONDecodeError) as error:
            raise CaptureError("APT_SOURCE_CAPTURE_INVALID", "JSON") from error
        if isinstance(apt_value, Mapping):
            if "release_evidence" in apt_value:
                release_evidence = _validate_release_evidence(
                    output_root, apt_value["release_evidence"])
            if "package_evidence" in apt_value:
                package_evidence = _validate_package_evidence(
                    output_root, apt_value["package_evidence"])
    if value["signature_references"] and (
            not release_evidence or not package_evidence):
        raise CaptureError("REFERENCE_PROJECTION_BINDING_INVALID", "signature evidence prefix")
    _validate_reference_projection(
        value["reference_map"], value["reference_counts"],
        release_evidence, package_evidence,
        references, value["signature_count"], require_complete=False,
        label="signature evidence reference projection")
    expected_repositories = {
        (item["url"], item["release_url"], item["release_path"],
         item["release_sha256"], item["packages_url"], item["packages_path"],
         item["packages_sha256"]): item
        for item in manifest["repositories"]
    }
    if len(expected_repositories) != len(manifest["repositories"]):
        raise CaptureError("SIGNATURE_EVIDENCE_BINDING_INVALID", "manifest repository duplicate")
    manifest_keyrings = {
        (item["path"], item["sha256"]): item
        for item in manifest.get("keyrings", [])
        if isinstance(item, Mapping) and "path" in item and "sha256" in item
    }
    source_entries = manifest.get("source_entries", [])
    signature_by_identity = {
        item.get("identity"): item for item in validated_signatures
        if item.get("identity") is not None
    }
    for item in validated_signatures:
        if not isinstance(item, Mapping) or "repository" not in item:
            raise CaptureError("SIGNATURE_EVIDENCE_INVALID", "repository fields")
        repository = item["repository"]
        if not isinstance(repository, Mapping) or set(repository) != {
                "url", "release_url", "release_path", "release_sha256", "packages_url",
                "packages_path", "packages_sha256"}:
            raise CaptureError("SIGNATURE_EVIDENCE_INVALID", "repository binding")
        key = (repository["url"], repository["release_url"], repository["release_path"],
               repository["release_sha256"], repository["packages_url"],
               repository["packages_path"], repository["packages_sha256"])
        if key not in expected_repositories or item["release_kind"] not in {
                "inrelease", "detached"}:
            raise CaptureError("SIGNATURE_EVIDENCE_BINDING_INVALID", "repository")
        evidence_keyrings = item["keyrings"]
        if len(evidence_keyrings) != 1:
            raise CaptureError("SIGNATURE_EVIDENCE_KEYRING_SCOPE_INVALID", "repository")
        evidence_keyring = evidence_keyrings[0]
        manifest_keyring = manifest_keyrings.get(
            (evidence_keyring.get("path"), evidence_keyring.get("sha256")))
        if manifest_keyring is None or dict(evidence_keyring) != dict(manifest_keyring):
            raise CaptureError("SIGNATURE_EVIDENCE_KEYRING_BINDING_INVALID", "repository")
        repository_url = repository["url"]
        source_key_refs = {
            (entry["signed_by"].get("artifact_path"),
             entry["signed_by"].get("sha256"))
            for entry in source_entries
            if isinstance(entry, Mapping) and repository_url in entry.get("uris", [])
            and isinstance(entry.get("signed_by"), Mapping)
            and entry["signed_by"].get("kind") != "none"
        }
        if (evidence_keyring.get("path"), evidence_keyring.get("sha256")) \
                not in source_key_refs:
            raise CaptureError("SIGNATURE_EVIDENCE_KEYRING_SCOPE_INVALID", repository_url)
    observed_repositories = set()
    for reference in references:
        repository_key = (
            reference["repository_url"], reference["release_url"],
            reference["release_record_path"], reference["release_sha256"],
            reference["packages_url"], reference["packages_path"],
            reference["packages_sha256"])
        if repository_key not in expected_repositories:
            raise CaptureError("SIGNATURE_EVIDENCE_BINDING_INVALID", "reference repository")
        observed_repositories.add(repository_key)
        signature = signature_by_identity.get(reference["identity"])
        if signature is None or signature["release_kind"] != reference["release_kind"]:
            raise CaptureError("SIGNATURE_EVIDENCE_BINDING_INVALID", "reference identity")
        signer = signature["status"]["validsig"]["fingerprint"]
        if signer != reference["signer_fingerprint"]:
            raise CaptureError("SIGNATURE_EVIDENCE_BINDING_INVALID", "reference signer")
        source_matches = [
            entry for entry in source_entries
            if isinstance(entry, Mapping) and
            reference["repository_url"] in entry.get("uris", []) and
            isinstance(entry.get("signed_by"), Mapping) and
            entry["signed_by"].get("kind") != "none"]
        key_identity_matches = set()
        for entry in source_matches:
            signed_by = entry["signed_by"]
            artifact_key = (signed_by.get("artifact_path"), signed_by.get("sha256"))
            manifest_keyring = manifest_keyrings.get(artifact_key)
            if manifest_keyring is None:
                continue
            key_identity_matches.add(_source_entry_keyring_identity(
                signed_by, manifest_keyring))
        if reference["keyring_identity"] not in key_identity_matches:
            raise CaptureError("SIGNATURE_EVIDENCE_KEYRING_SCOPE_INVALID", "reference key")
    if observed_repositories != set(expected_repositories):
        raise CaptureError("SIGNATURE_EVIDENCE_BINDING_INVALID", "repository set")
    return dict(value)


def _capture_dpkg_metadata(path: Path, runner: Callable[[list[str], int], Any],
                           reader: Callable[[Path], Mapping[str, Any]] | None) -> dict[str, Any]:
    if reader is not None:
        try:
            result = dict(reader(path))
        except Exception as error:
            raise CaptureError("DPKG_DEB_FAILED", str(path)) from error
        if not result:
            raise CaptureError("DPKG_DEB_FIELDS_MISSING", str(path))
        return result
    argv = ["dpkg-deb", "--field", str(path), "Package", "Version", "Architecture",
            "Essential", "Pre-Depends", "Multi-Arch"]
    _validate_argv(argv, "dpkg-deb metadata")
    result = _normalize_output(runner(argv, 30))
    if result[0] != 0 or result[3] or result[4] is not None:
        raise CaptureError("DPKG_DEB_FAILED", str(path))
    return _parse_dpkg_deb_output(result[1], str(path))


def _select_signed_package_index(indexes: list[Mapping[str, Any]],
                                 uri: Mapping[str, Any],
                                 identity: tuple[Any, Any, Any],
                                 deb_data: bytes) -> tuple[Mapping[str, Any], Mapping[str, Any]]:
    """Select one stable signed attestation for the exact materialized deb."""
    digest = sha256_bytes(deb_data)
    matches = []
    for index in indexes:
        repository = index["repository"]
        if not uri["url"].startswith(repository["url"].rstrip("/") + "/"):
            continue
        package = index["packages"].get(identity)
        if package is not None and Path(package["filename"]).name == uri["url_filename"] and \
                package["bytes"] == len(deb_data) and package["sha256"] == digest:
            matches.append((index, package))
    if not matches:
        raise CaptureError("PACKAGE_INDEX_BINDING_DRIFT", str(uri["filename"]))
    # Base and updates suites can legitimately attest the same immutable pool
    # object.  Every retained candidate already matches identity, bytes and
    # SHA; choose one deterministically without weakening material binding.
    matches.sort(key=lambda pair: (
        pair[0]["repository"]["release_url"],
        pair[0]["repository"]["packages_url"],
        pair[0]["repository"]["packages_path"],
    ))
    return matches[0]


def _validate_fixed_archives(output_root: Path, descriptors: Any) -> list[dict[str, Any]]:
    if descriptors is None:
        return []
    if not isinstance(descriptors, list):
        raise CaptureError("ARCHIVE_DESCRIPTOR_INVALID", "list required")
    result = []
    seen: set[str] = set()
    for item in descriptors:
        if not isinstance(item, Mapping) or set(item) != {
                "name", "path", "url", "sha256", "bytes"}:
            raise CaptureError("ARCHIVE_DESCRIPTOR_INVALID", "fields")
        name = item["name"]
        if (not isinstance(name, str) or not re.fullmatch(r"[A-Za-z0-9_.+-]{1,128}", name) or
                name in seen):
            raise CaptureError("ARCHIVE_DESCRIPTOR_INVALID", "name")
        seen.add(name)
        relative = _safe_rel(item["path"], "archive path")
        url = _https_url(item["url"], "archive URL")
        _sha(item["sha256"], "archive SHA")
        if type(item["bytes"]) is not int or not 0 < item["bytes"] <= MAX_CAPTURE_FILE_BYTES:
            raise CaptureError("ARCHIVE_DESCRIPTOR_INVALID", "bytes")
        data, _ = _read_bounded(output_root / relative, "fixed archive " + name,
                                MAX_CAPTURE_FILE_BYTES, allow_empty=False)
        if len(data) != item["bytes"] or sha256_bytes(data) != item["sha256"]:
            raise CaptureError("ARCHIVE_DESCRIPTOR_DRIFT", name)
        result.append({"name": name, "path": relative, "url": url,
                       "sha256": item["sha256"], "bytes": len(data)})
    if [item["name"] for item in result] != sorted(item["name"] for item in result):
        raise CaptureError("ARCHIVE_DESCRIPTOR_UNSORTED", "archives")
    return result


def _capture_dependency_phase(output_root: Path, distro: str,
                              container_root: Path | None,
                              runner: Callable[[list[str], int], Any],
                              *, dpkg_reader: Callable[[Path], Mapping[str, Any]] | None = None,
                              archive_descriptors: Any = None) -> dict[str, Any]:
    manifest, index_records = _read_source_manifest(output_root)
    signature_evidence = _read_signature_evidence(output_root, distro, manifest)
    base_status_path = output_root / "base-status.json"
    base_data, _ = _read_bounded(base_status_path, "base status", MAX_JSON_BYTES,
                                 allow_empty=False)
    try:
        base_value = json.loads(base_data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError("BASE_STATUS_INVALID", "JSON") from error
    if (not isinstance(base_value, Mapping) or base_value.get("schema") !=
            "glim_clean_room_r3_base_dpkg_status_v1" or
            base_value.get("status") != "SEALED_PREINSTALL" or
            base_value.get("canonical_sha256") != canonical_hash(base_value)):
        raise CaptureError("BASE_STATUS_INVALID", "schema/hash")
    base_identities = set()
    for item in base_value.get("packages", []):
        if not isinstance(item, Mapping) or set(item) != {
                "name", "version", "architecture", "status"} or \
                item["status"] != "install ok installed":
            raise CaptureError("BASE_STATUS_INVALID", "package record")
        base_identities.add((item["name"], item["version"], item["architecture"]))
    if not base_identities:
        raise CaptureError("BASE_STATUS_EMPTY", "base packages")
    indexes = [item for item in index_records.values()]
    all_resolvers = []
    package_output: list[dict[str, Any]] = []
    for role in ("build", "runtime"):
        raw_path = output_root / "logs" / "resolver_{}.stdout".format(role)
        raw_data, raw_info = _read_bounded(raw_path, role + " resolver output",
                                           MAX_LOG_BYTES, allow_empty=False)
        try:
            lines = raw_data.decode("utf-8").splitlines()
        except UnicodeError as error:
            raise CaptureError("RESOLVER_NOT_UTF8", role) from error
        uris = _canonical_resolver_uris(lines, role)
        deb_root = output_root / "debs-{}".format(role)
        _apt_cache_debs(deb_root, {item["filename"] for item in uris}, role)
        role_packages = []
        for uri in uris:
            deb_path = deb_root / uri["filename"]
            deb_data, _ = _container_read(deb_path, role + " deb " + uri["filename"],
                                           MAX_CAPTURE_DEB_BYTES, allow_empty=False)
            metadata = _capture_dpkg_metadata(deb_path, runner, dpkg_reader)
            identity = (metadata.get("name"), metadata.get("version"), metadata.get("architecture"))
            if _resolver_cache_filename(identity) != uri["filename"]:
                raise CaptureError("RESOLVER_CACHE_FILENAME_MISMATCH", uri["filename"])
            index, indexed = _select_signed_package_index(
                indexes, uri, identity, deb_data)
            if uri["digest_algorithm"] == "sha256" and uri["digest"] != indexed["sha256"]:
                raise CaptureError("PACKAGE_URI_SHA_DRIFT", uri["filename"])
            if identity in base_identities:
                raise CaptureError("PACKAGE_BASE_OVERLAP", uri["filename"])
            (copyright_data, copyright_descriptor, copyright_path, copyright_aliases,
             copyright_status) = _package_copyright(container_root, metadata["name"])
            copied = _copy_capture_file(
                output_root, "licenses/{}/{}".format(role, uri["filename"]), copyright_data)
            copied_deb = _copy_capture_file(
                output_root, "{}/{}/{}".format(CAPTURE_DEB_RELATIVE, role, uri["filename"]), deb_data,
                MAX_CAPTURE_DEB_BYTES)
            package = {
                "name": metadata["name"], "version": metadata["version"],
                "architecture": metadata["architecture"], "filename": uri["filename"],
                "url": uri["url"], "bytes": len(deb_data), "sha256": sha256_bytes(deb_data),
                "repository_url": index["repository"]["url"],
                "release_url": index["repository"]["release_url"],
                "release_path": index["repository"]["release_path"],
                "release_sha256": index["repository"]["release_sha256"],
                "packages_url": index["repository"]["packages_url"],
                "packages_path": index["repository"]["packages_path"],
                "packages_sha256": index["repository"]["packages_sha256"],
                "uri_digest_algorithm": uri["digest_algorithm"], "uri_digest": uri["digest"],
                "essential": bool(metadata.get("essential", False)),
                "pre_depends": sorted(set(metadata.get("pre_depends", []))),
                "multiarch": str(metadata.get("multiarch", "same")),
                "base_installed": False,
                "dpkg_control": metadata,
                "deb_descriptor": _relative_descriptor(output_root, copied_deb),
                "copyright": {"status": copyright_status,
                               "logical_source": "/usr/share/doc/{}/copyright".format(
                                   metadata["name"]),
                               "source": str(copyright_path),
                               "aliases": copyright_aliases,
                               "source_descriptor": copyright_descriptor,
                               "output": _relative_descriptor(output_root, copied),
                               "bytes": len(copyright_data),
                               "sha256": sha256_bytes(copyright_data)},
            }
            role_packages.append(package)
        role_packages.sort(key=lambda item: (item["name"], item["version"], item["architecture"]))
        all_resolvers.append({
            "role": role, "raw_log": {"path": "logs/resolver_{}.stdout".format(role),
                                        "bytes": len(raw_data), "sha256": sha256_bytes(raw_data),
                                        "device": raw_info.st_dev, "inode": raw_info.st_ino},
            "uri_lines": [_canonical_resolver_line(uri) for uri in uris],
            "packages": role_packages,
        })
        package_output.extend(role_packages)
    if not package_output:
        raise CaptureError("DEPENDENCY_PACKAGE_SET_EMPTY", "no downloaded packages")
    rosdep_log_path = output_root / "logs" / "rosdep_resolution.stdout"
    rosdep_data, rosdep_info = _read_bounded(
        rosdep_log_path, "rosdep resolution output", MAX_LOG_BYTES, allow_empty=False)
    archives = _validate_fixed_archives(output_root, archive_descriptors)
    result = {
        "schema": IN_CONTAINER_SCHEMA, "schema_version": IN_CONTAINER_SCHEMA_VERSION,
        "status": "CAPTURED_REVIEW_REQUIRED", "phase": "dependency", "distro": distro,
        "source_manifest": {"path": "source-manifest.json",
                             "sha256": sha256_bytes(_read_bounded(
                                 output_root / "source-manifest.json", "source manifest",
                                 MAX_JSON_BYTES, allow_empty=False)[0])},
        "base_status": {"path": "base-status.json", "sha256": sha256_bytes(base_data)},
        "resolvers": all_resolvers, "packages": package_output,
        "rosdep_resolution": {"path": "logs/rosdep_resolution.stdout",
                               "bytes": len(rosdep_data), "sha256": sha256_bytes(rosdep_data),
                               "device": rosdep_info.st_dev, "inode": rosdep_info.st_ino},
        "archives": archives,
        "signature_evidence": {"path": "signature-evidence.json",
                                "sha256": sha256_bytes(_read_bounded(
                                    output_root / "signature-evidence.json",
                                    "signature evidence", MAX_JSON_BYTES,
                                    allow_empty=False)[0])},
        "signature_evidence_identity": signature_evidence["canonical_sha256"],
    }
    result["canonical_sha256"] = canonical_hash(result)
    return result


def _capture_in_container(phase: str, output_root: Path, distro: str, *,
                          container_root: Path | None = None,
                          command_runner: Callable[[list[str], int], Any] | None = None,
                          dpkg_reader: Callable[[Path], Mapping[str, Any]] | None = None,
                          archive_descriptors: Any = None,
                          apt_source_url_policy: Mapping[str, Any] | None = None,
                          package_fetcher: Callable[..., Any] | None = None) -> dict[str, Any]:
    """Capture filesystem and bounded tool facts from the provisioning container.

    The fixed Docker command supplies the real container root and evidence
    mount.  Tests may supply a directory fixture and a deterministic runner;
    no caller-supplied shell command is accepted.  A successful result is
    still ``CAPTURED_REVIEW_REQUIRED``: outer image/network evidence and a
    later independent review remain mandatory.
    """
    output_root = _safe_abs(str(output_root), "container output root")
    if not output_root.is_dir() or output_root.is_symlink():
        raise CaptureError("CONTAINER_OUTPUT_INVALID", str(output_root))
    if phase not in IN_CONTAINER_PHASES:
        raise CaptureError("IN_CONTAINER_PHASE_INVALID", phase)
    runner = command_runner or _default_runner
    policy = _validate_apt_source_url_policy(
        APT_SOURCE_URL_POLICY if apt_source_url_policy is None else apt_source_url_policy)
    directory_contract = _snapshot_capture_directory_layout(
        output_root, include_prefetch=_capture_prefetch_layout_present(output_root))
    output_name = "in-container-{}.json".format(phase)
    try:
        if phase == "apt-source":
            result = _capture_apt_source_phase(
                output_root, distro, container_root, runner, policy, package_fetcher)
            _validate_release_evidence(
                output_root, result.get("release_evidence"), require_nonempty=True)
        else:
            result = _capture_dependency_phase(
                output_root, distro, container_root, runner,
                dpkg_reader=dpkg_reader, archive_descriptors=archive_descriptors)
        result["directory_contract"] = dict(directory_contract)
        result["canonical_sha256"] = canonical_hash(result)
        _seal_pair(output_root, output_name, result)
        return result
    except Exception as error:
        failure_kind = getattr(error, "kind", "IN_CONTAINER_CAPTURE_FAILED")
        failure = {
            "schema": IN_CONTAINER_SCHEMA, "schema_version": IN_CONTAINER_SCHEMA_VERSION,
            "status": "FAILED_REVIEW_REQUIRED", "phase": phase, "distro": distro,
            "failure": {"kind": str(failure_kind)[:128]},
        }
        if getattr(error, "release_evidence", None):
            failure["release_evidence"] = list(error.release_evidence)
        if getattr(error, "package_evidence", None):
            failure["package_evidence"] = list(error.package_evidence)
        if getattr(error, "signature_evidence", None):
            failure["signature_evidence"] = _sorted_signature_evidence(
                list(error.signature_evidence))
        if getattr(error, "signature_references", None) is not None:
            failure["signature_references"] = _immutable_copy(
                error.signature_references)
            failure["signature_count"] = error.signature_count
        if getattr(error, "reference_map", None) is not None:
            failure["reference_map"] = _immutable_copy(error.reference_map)
            failure["reference_counts"] = _immutable_copy(error.reference_counts)
        failure["directory_contract"] = dict(directory_contract)
        failure["canonical_sha256"] = canonical_hash(failure)
        target = output_root / output_name
        if not target.exists() and not target.is_symlink():
            try:
                _seal_pair(output_root, output_name, failure)
            except Exception:
                # A failed seal is itself fail-closed.  Do not replace or
                # delete any pre-existing partial artifact.
                pass
        if isinstance(error, CaptureError):
            raise
        raise CaptureError("IN_CONTAINER_CAPTURE_FAILED", str(error)) from error


capture_in_container = _capture_in_container


def _run_archive_prefetch(plan: Mapping[str, Any], root: Path,
                          fetcher: Callable[..., Any] | None = None) -> tuple[dict[str, Any], dict[str, Any]]:
    """Fetch only profile-pinned source archives during the connected phase."""
    phase = "archive_prefetch"
    if plan["dependency_leg"] == "absent":
        record = _phase_record(
            plan, root, phase, (0, b"NOT_APPLICABLE\n", b"", False, None))
        return record, {"status": "NOT_APPLICABLE"}
    dependencies = plan["leg"].get("dependencies")
    if not isinstance(dependencies, list) or not dependencies:
        raise CaptureError("OPTIONAL_DEPENDENCY_NOT_PINNED", "present archive set")
    prefetch_root = root / "prefetch-root"
    try:
        EVIDENCE_DIRECTORIES.snapshot_existing(
            prefetch_root, ("prefetch", "prefetch/archives"))
    except EVIDENCE_DIRECTORIES.EvidenceDirectoryError as error:
        raise CaptureError(error.kind, str(error)) from error
    _seal_pair(prefetch_root, PREFETCH.DEPENDENCY_ENVIRONMENT_NAME, {
        "schema": "registration-plugin-prefetch-environment-input-v1",
        "schema_version": 1, "status": "CAPTURED_REVIEW_REQUIRED",
        "distro": plan["distro"], "image_digest": plan["image"]["digest"],
    })
    try:
        result = PREFETCH.prefetch_pinned_archives(
            dependencies, prefetch_root, plan["distro"], plan["image"]["digest"],
            authority="HOST_PROMOTION", fetcher=fetcher,
            allowed_host_directories={"prefetch", "prefetch/archives"})
        output = canonical_bytes({"status": "SEALED_PREFETCH",
                                  "receipt_sha256": result["receipt_sha256"],
                                  "manifest_sha256": result["manifest_sha256"]}) + b"\n"
        record = _phase_record(plan, root, phase, (0, output, b"", False, None))
        return record, {"status": "SEALED_PREFETCH", "root_path": "prefetch-root",
                        "dependencies": list(dependencies)}
    except Exception as error:
        message = str(error).encode("utf-8", errors="replace")[:MAX_LOG_BYTES]
        record = _phase_record(plan, root, phase, (1, b"", message, False, None))
        return record, {"status": "FAILED"}


def _compose_candidate(plan: Mapping[str, Any], root: Path,
                       records: list[dict[str, Any]],
                       optional_prefetch: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Reopen raw logs/debs and invoke the existing canonical composer."""
    if optional_prefetch is None:
        optional_prefetch = {"status": "NOT_APPLICABLE"}
    capture = _write_capture_input(root, plan, records, optional_prefetch)
    for role in CLOSURE.ROLES:
        phase = "resolver_{}".format(role)
        command = list(plan["phase_argv"][phase][3:])
        if command[:1] != ["apt-get"]:
            raise CaptureError("RESOLVER_COMMAND_INVALID", role)
        resolver_output = root / (role + "-resolver.json")
        CLOSURE.APT.build_resolver_output(
            role=role,
            raw_log_path=root / capture["resolvers"][role]["raw_log_path"],
            downloaded_root=root / ("debs-" + role),
            requirements_path=root / "requirements.json",
            base_status_path=root / "base-status.json",
            source_root=root / "apt-source",
            source_manifest_path=root / "source-manifest.json",
            command=command, output_path=resolver_output,
            dpkg_reader=CLOSURE._default_dpkg_reader, exit_status=0)
        _seal_existing_sidecar(
            resolver_output, role + " resolver", MAX_JSON_BYTES)
    closure_root = root / "closure"
    closure_result = CLOSURE.compose_closure(capture_root=root, output_root=closure_root)
    plan["_artifacts"] = {
        "capture_input": _descriptor(root / CAPTURE_INPUT_NAME,
                                      "capture input", MAX_JSON_BYTES, sidecar=True),
        "closure": _descriptor(closure_root / CLOSURE.CLOSURE_FILENAME,
                                "dependency closure", MAX_JSON_BYTES, sidecar=True),
    }
    return closure_result


def _empty_rosdep_prepare_projection(
        required: bool, input_binding: Mapping[str, Any] | None = None
        ) -> dict[str, Any]:
    """Return the explicit non-executed projection used by unit seams."""
    return {
        "required": bool(required),
        "status": "NOT_CONFIGURED" if not required else "NOT_RUN",
        "receipt": None,
        "canonical_sha256": None,
        "discovery": None,
        # The prepare receipt has two path namespaces.  Keep both in the
        # outer projection even before execution so a partial row cannot
        # silently collapse the logical host root into the container path.
        "root": None,
        "transport_root": None,
        "root_mount": None,
        "discovery_transport": None,
        "source_list": None,
        "cache_tree": None,
        "deb_artifacts": [],
        "input_binding": _immutable_copy(input_binding),
    }


def _validate_prepare_discovery(actual: Mapping[str, Any],
                                expected: Mapping[str, Any]) -> None:
    """Compare the prepare receipt with the exact discovery-11 projection."""
    for key in ("discovery_id", "root", "profile", "image", "source_records",
                "receipt_bytes_sha256"):
        if actual.get(key) != expected.get(key):
            raise CaptureError("ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", key)
    actual_receipt = actual.get("receipt")
    expected_receipt = expected.get("receipt")
    if (not isinstance(actual_receipt, Mapping) or
            not isinstance(expected_receipt, Mapping) or
            any(actual_receipt.get(key) != expected_receipt.get(key)
                for key in ("path", "sha256", "canonical_sha256"))):
        raise CaptureError("ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", "receipt")
    logical_descriptor = actual_receipt.get("descriptor")
    transport_descriptor = actual_receipt.get("transport_descriptor")
    if (not isinstance(logical_descriptor, Mapping) or
            not isinstance(transport_descriptor, Mapping) or
            logical_descriptor.get("path") != actual_receipt.get("path") or
            not isinstance(logical_descriptor.get("sidecar"), Mapping) or
            logical_descriptor["sidecar"].get("path") != (
                actual_receipt.get("path") + ".sha256") or
            not isinstance(transport_descriptor.get("path"), str) or
            not transport_descriptor["path"].startswith(
                ROSDEP_PREPARE_CONTAINER_DISCOVERY_ROOT + "/") or
            not isinstance(transport_descriptor.get("sidecar"), Mapping) or
            transport_descriptor["sidecar"].get("path") != (
                transport_descriptor["path"] + ".sha256") or
            any(logical_descriptor.get(key) != transport_descriptor.get(key)
                for key in ("bytes", "sha256", "mode", "nlink"))):
        raise CaptureError(
            "ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", "receipt descriptors"
        )
    actual_rosdistro = actual.get("rosdistro")
    expected_rosdistro = expected.get("rosdistro")
    if (not isinstance(actual_rosdistro, Mapping) or
            not isinstance(expected_rosdistro, Mapping) or
            any(actual_rosdistro.get(key) != expected_rosdistro.get(key)
                for key in ("repository", "commit", "archive_url",
                            "archive_bytes", "archive_sha256"))):
        raise CaptureError("ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", "rosdistro")
    actual_packages = actual.get("packages")
    expected_packages = expected.get("packages")
    if (not isinstance(actual_packages, list) or
            not isinstance(expected_packages, list) or
            len(actual_packages) != len(expected_packages)):
        raise CaptureError("ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", "packages")
    for observed, pinned in zip(actual_packages, expected_packages):
        if (not isinstance(observed, Mapping) or
                any(observed.get(key) != pinned.get(key) for key in (
                    "name", "version", "architecture", "filename", "uri",
                    "size_bytes", "sha256"))):
            raise CaptureError("ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", "package")
        artifact = observed.get("artifact")
        if (not isinstance(artifact, Mapping) or
                artifact.get("path") != pinned.get("artifact_path") or
                artifact.get("bytes") != pinned.get("size_bytes") or
                artifact.get("sha256") != pinned.get("sha256")):
            raise CaptureError("ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", "deb artifact")
    actual_files = actual.get("selected_files")
    expected_files = expected.get("selected_files")
    if (not isinstance(actual_files, list) or
            not isinstance(expected_files, list) or
            len(actual_files) != len(expected_files)):
        raise CaptureError("ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", "selected files")
    for observed, pinned in zip(actual_files, expected_files):
        if (not isinstance(observed, Mapping) or
                any(observed.get(key) != pinned.get(key) for key in (
                    "relative_path", "bytes", "sha256"))):
            raise CaptureError("ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", "selected file")
        artifact = observed.get("artifact")
        if (not isinstance(artifact, Mapping) or
                artifact.get("path") != pinned.get("artifact_path") or
                artifact.get("bytes") != pinned.get("bytes") or
                artifact.get("sha256") != pinned.get("sha256")):
            raise CaptureError("ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", "selected artifact")


def _validate_prepare_input_binding_pair(
        host_binding: Any, transport_binding: Any,
        *, capture_root: Path | None = None) -> None:
    """Compare host-copy and container-transport input identities.

    The host binding records the checkout source and the host-owned copied
    files.  The prepare receipt is produced inside a container and therefore
    records the fixed ``/opt/registration-plugin/prepare`` paths.  Those
    paths are intentionally different namespaces; only role/order, content
    identity, and mount semantics may be compared across them.
    """
    try:
        CLOSURE._validate_prepare_input_binding_shape(host_binding)
    except Exception as error:
        raise CaptureError(
            "PREPARE_INPUT_BINDING_INVALID", "host: " + str(error)
        ) from error
    try:
        # The inner producer's descriptors have no producer-side sidecar for
        # a direct mounted input.  Its own validator still checks every
        # descriptor and reopens the files when running inside the container;
        # outer host readback uses shape-only mode because /opt is a container
        # namespace, not a host path.
        ROSDEP_PREPARE._validate_prepare_input_binding(
            transport_binding, reopen=False
        )
    except Exception as error:
        raise CaptureError(
            "PREPARE_INPUT_BINDING_INVALID", "transport: " + str(error)
        ) from error
    if not isinstance(host_binding, Mapping) or not isinstance(
            transport_binding, Mapping):
        raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "binding object")
    for key in ("schema", "schema_version", "relative_root", "container_root"):
        if host_binding.get(key) != transport_binding.get(key):
            raise CaptureError("PREPARE_INPUT_BINDING_INVALID", key)
    host_mount = host_binding.get("mount")
    transport_mount = transport_binding.get("mount")
    mount_fields = ("target", "read_only", "noexec", "repository_bind")
    if (not isinstance(host_mount, Mapping) or
            not isinstance(transport_mount, Mapping) or
            any(host_mount.get(key) != transport_mount.get(key)
                for key in mount_fields) or
            not isinstance(host_mount.get("source"), str) or
            not isinstance(transport_mount.get("source"), str) or
            not host_mount["source"].startswith("/") or
            not transport_mount["source"].startswith("/") or
            transport_mount["source"] != ROSDEP_PREPARE_CONTAINER_ROOT):
        raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "mount semantics")
    if capture_root is not None:
        expected_host_source = str(_safe_abs(
            str(capture_root / ROSDEP_PREPARE_INPUT_RELATIVE),
            "prepare host input root",
        ))
        if host_mount["source"] != expected_host_source:
            raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "host mount source")
    host_files = host_binding.get("files")
    transport_files = transport_binding.get("files")
    if (not isinstance(host_files, list) or
            not isinstance(transport_files, list) or
            len(host_files) != len(transport_files)):
        raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "file count")
    content_fields = ("bytes", "sha256")
    copied_fields = ("bytes", "sha256", "mode", "nlink")
    for host_file, transport_file in zip(host_files, transport_files):
        if (not isinstance(host_file, Mapping) or
                not isinstance(transport_file, Mapping) or
                any(host_file.get(key) != transport_file.get(key)
                    for key in ("role", "source_path", "container_path",
                                "relative_path"))):
            raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "file identity")
        container_path = transport_file.get("container_path")
        if not isinstance(container_path, str):
            raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "container path")
        transport_source = transport_file.get("source")
        transport_before = transport_file.get("before")
        transport_after = transport_file.get("after")
        if (not isinstance(transport_source, Mapping) or
                not isinstance(transport_before, Mapping) or
                not isinstance(transport_after, Mapping) or
                transport_source.get("path") != container_path or
                transport_before.get("path") != container_path or
                transport_after.get("path") != container_path or
                any(transport_source.get(key) != transport_before.get(key)
                    for key in copied_fields) or
                any(transport_before.get(key) != transport_after.get(key)
                    for key in copied_fields)):
            raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "transport file")
        host_source = host_file.get("source")
        host_before = host_file.get("before")
        host_after = host_file.get("after")
        if (not isinstance(host_source, Mapping) or
                not isinstance(host_before, Mapping) or
                not isinstance(host_after, Mapping)):
            raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "host file")
        if any(host_source.get(key) != transport_source.get(key)
               for key in content_fields):
            raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "source content")
        if any(host_before.get(key) != transport_before.get(key)
               for key in copied_fields):
            raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "copy content")
        if any(host_after.get(key) != host_before.get(key)
               for key in copied_fields):
            raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "copy stability")


def _validate_prepare_root_binding(
        value: Mapping[str, Any], capture_root: Path) -> None:
    """Validate the logical/transport output-root envelope from prepare.

    The prepare process writes through a bind mounted output directory.  Its
    receipt therefore contains two path namespaces for the same directory
    identity: the host logical path and the fixed in-container transport
    path.  Metadata may be compared across the namespaces, but paths must be
    compared to their respective fixed roots.  Keeping this check separate
    from the input-file pair makes partial receipts use exactly the same
    envelope as successful receipts.
    """
    logical = value.get("root")
    transport = value.get("transport_root")
    mount = value.get("root_mount")
    fields = {
        "path", "type", "mode", "uid", "gid", "nlink", "device", "inode"
    }
    if (not isinstance(logical, Mapping) or not isinstance(transport, Mapping) or
            not isinstance(mount, Mapping) or set(logical) != fields or
            set(transport) != fields or set(mount) != {
                "source", "target", "read_only", "type", "noexec", "identity"
            }):
        raise CaptureError("PREPARE_ROOT_BINDING_INVALID", "envelope fields")
    expected_logical = str(_safe_abs(
        str(capture_root / ROSDEP_PREPARE_RELATIVE),
        "prepare logical output root",
    ))
    expected_transport = ROSDEP_PREPARE_CONTAINER_OUTPUT_ROOT
    if (logical.get("path") != expected_logical or
            transport.get("path") != expected_transport or
            logical.get("type") != "directory" or
            transport.get("type") != "directory" or
            logical.get("mode") != 0o700 or
            transport.get("mode") != 0o700):
        raise CaptureError("PREPARE_ROOT_BINDING_INVALID", "root paths/type/mode")
    for descriptor, label in ((logical, "logical root"),
                              (transport, "transport root")):
        for key in ("uid", "gid", "nlink", "device", "inode"):
            if type(descriptor.get(key)) is not int or descriptor[key] < 0:
                raise CaptureError("PREPARE_ROOT_BINDING_INVALID", label)
        if descriptor["nlink"] < 2 or descriptor["inode"] < 1:
            raise CaptureError("PREPARE_ROOT_BINDING_INVALID", label)
    if any(logical.get(key) != transport.get(key) for key in fields - {"path"}):
        raise CaptureError("PREPARE_ROOT_BINDING_INVALID", "logical/transport identity")
    expected_identity = {
        key: transport[key]
        for key in ("device", "inode", "uid", "gid", "mode", "nlink")
    }
    if (mount.get("source") != expected_logical or
            mount.get("target") != expected_transport or
            mount.get("read_only") is not False or
            mount.get("type") != "bind" or
            mount.get("noexec") is not False or
            mount.get("identity") != expected_identity):
        raise CaptureError("PREPARE_ROOT_BINDING_INVALID", "root mount")


def _validate_rosdep_prepare_outputs(prepare_phase: Mapping[str, Any]) -> None:
    """Require non-empty rosdep evidence only after prepare has completed.

    The APT source phase must remain valid for base images that do not ship
    rosdep.  A completed prepare receipt, however, is the authoritative point
    at which rosdep source/cache evidence is required.  The prepare producer
    already reopens each descriptor; this check adds the phase-level
    non-empty contract without scanning the container from the APT helper.
    """
    source_list = prepare_phase.get("source_list")
    if (not isinstance(source_list, Mapping) or
            type(source_list.get("bytes")) is not int or
            source_list["bytes"] <= 0):
        raise CaptureError("ROSDEP_SOURCE_EMPTY", "prepare source list")
    source_bindings = prepare_phase.get("source_bindings")
    if not isinstance(source_bindings, list) or not source_bindings:
        raise CaptureError("ROSDEP_SOURCE_EMPTY", "prepare source bindings")
    cache_tree = prepare_phase.get("cache_tree")
    if (not isinstance(cache_tree, Mapping) or
            type(cache_tree.get("bytes")) is not int or
            cache_tree["bytes"] <= 0):
        raise CaptureError("ROSDEP_CACHE_EMPTY", "prepare cache tree")
    cache_entries = prepare_phase.get("cache_entries")
    if not isinstance(cache_entries, list) or not cache_entries:
        raise CaptureError("ROSDEP_CACHE_EMPTY", "prepare cache entries")


def _rosdep_prepare_projection(root: Path, plan: Mapping[str, Any],
                               *, allow_partial: bool = False,
                               projection: Mapping[str, Any] | None = None
                               ) -> dict[str, Any]:
    """Reopen the prepare receipt and project its sealed source/cache/debs."""
    contract = plan.get("rosdep_prepare_contract")
    if not isinstance(contract, Mapping):
        raise CaptureError("ROSDEP_PREPARE_CONTRACT_INVALID", "missing contract")
    required = contract.get("required") is True
    if projection is not None:
        computed_plan = dict(plan)
        if computed_plan.get("_rosdep_prepare_input") is None and \
                isinstance(projection.get("input_binding"), Mapping):
            computed_plan["_rosdep_prepare_input"] = _immutable_copy(
                projection["input_binding"])
        computed = _rosdep_prepare_projection(
            root, computed_plan, allow_partial=True)
        if dict(projection) != computed:
            raise CaptureError(
                "ROSDEP_PREPARE_PROJECTION_INVALID", "receipt projection drift")
        if not required:
            # Legacy/unit plans do not run this phase.  They must carry the
            # explicit NOT_CONFIGURED projection, and never silently accept a
            # prepare receipt that was produced outside their contract.
            return computed
        if not allow_partial and computed["status"] != "REVIEW_REQUIRED":
            raise CaptureError("ROSDEP_PREPARE_INCOMPLETE", computed["status"])
        return computed
    receipt_path = root / ROSDEP_PREPARE_RELATIVE / ROSDEP_PREPARE_RECEIPT_NAME
    if not receipt_path.exists() and not receipt_path.is_symlink():
        if required:
            raise CaptureError("ROSDEP_PREPARE_RECEIPT_MISSING", str(receipt_path))
        return _empty_rosdep_prepare_projection(False)
    if not required:
        raise CaptureError("ROSDEP_PREPARE_UNEXPECTED", str(receipt_path))
    try:
        value = ROSDEP_PREPARE.validate_prepare_receipt(
            receipt_path.parent,
            discovery_root=Path(contract["discovery"]["root"]),
            # Production prepare roots deliberately contain the fixed
            # precreated bind directories.  The prepare validator checks
            # their closed allowlist and metadata; the outer capture then
            # reopens the mount and input-file bindings below.
            allow_host_bindings=True,
            logical_root=receipt_path.parent,
            transport_root=Path(ROSDEP_PREPARE_CONTAINER_OUTPUT_ROOT),
            # The receipt is reopened by the host after the container exits;
            # its /opt input paths are a transport namespace.  Shape and
            # content correspondence are checked against the host binding
            # below without attempting to open /opt on the host.
            reopen_input_binding=False,
        )
    except Exception as error:
        raise CaptureError("ROSDEP_PREPARE_RECEIPT_INVALID", str(error)) from error
    expected_discovery = contract.get("discovery")
    actual_discovery = value.get("discovery")
    if (not isinstance(expected_discovery, Mapping) or
            not isinstance(actual_discovery, Mapping)):
        raise CaptureError("ROSDEP_PREPARE_DISCOVERY_BINDING_INVALID", "discovery")
    _validate_prepare_discovery(actual_discovery, expected_discovery)
    transport_input_binding = value.get("input_binding")
    host_input_binding = plan.get("_rosdep_prepare_input")
    if host_input_binding is None:
        raise CaptureError(
            "PREPARE_INPUT_BINDING_INVALID", "missing host binding"
        )
    _validate_prepare_input_binding_pair(
        host_input_binding, transport_input_binding, capture_root=root
    )
    _validate_prepare_root_binding(value, root)
    receipt_descriptor = _descriptor(
        receipt_path, "rosdep prepare receipt", MAX_JSON_BYTES, sidecar=True)
    prepare_phase = value["phases"][1]
    if value["status"] == "REVIEW_REQUIRED":
        _validate_rosdep_prepare_outputs(prepare_phase)
    projection = {
        "required": True,
        "status": value["status"],
        "receipt": receipt_descriptor,
        "canonical_sha256": value["canonical_sha256"],
        "discovery": _immutable_copy(actual_discovery),
        "root": _immutable_copy(value.get("root")),
        "transport_root": _immutable_copy(value.get("transport_root")),
        "root_mount": _immutable_copy(value.get("root_mount")),
        "discovery_transport": _immutable_copy(
            value.get("discovery_transport")),
        "source_list": _immutable_copy(prepare_phase.get("source_list")),
        "cache_tree": _immutable_copy(prepare_phase.get("cache_tree")),
        "deb_artifacts": [
            _immutable_copy(item["artifact"])
            for item in actual_discovery["packages"]
            if isinstance(item, Mapping) and isinstance(item.get("artifact"), Mapping)
        ],
        # Keep the host-owned binding in the outer projection.  The inner
        # receipt retains the transport binding; the pair above proves their
        # semantic correspondence without conflating path namespaces.
        "input_binding": _immutable_copy(host_input_binding),
    }
    if not allow_partial and projection["status"] != "REVIEW_REQUIRED":
        raise CaptureError("ROSDEP_PREPARE_INCOMPLETE", projection["status"])
    return projection


def _write_capture_input(root: Path, plan: Mapping[str, Any], records: list[dict[str, Any]],
                         optional_prefetch: Mapping[str, Any]) -> dict[str, Any]:
    # The existing closure composer is intentionally strict.  This input is
    # only written after every source path has been independently materialized.
    requirements_path = root / "requirements.json"
    if not requirements_path.exists():
        provisioning_packages = _provisioning_packages(plan["distro"])
        requirements = {
            "schema": "glim_clean_room_r3_package_requirements_v1",
            "schema_version": 1, "status": "PRECOMMITTED",
            "build": [{"name": name, "architecture": "amd64"}
                      for name in provisioning_packages],
            "runtime": [{"name": name, "architecture": "amd64"}
                        for name in provisioning_packages],
        }
        requirements["canonical_sha256"] = canonical_hash(requirements)
        _seal_pair(root, "requirements.json", requirements)
    resolver_paths = {}
    for role in CLOSURE.ROLES:
        phase = "resolver_{}".format(role)
        source = root / "logs" / (phase + ".stdout")
        raw_relative = Path("logs") / (phase + ".resolver.stdout")
        raw_path = root / raw_relative
        data, _ = _read_bounded(source, phase + " raw resolver log", MAX_LOG_BYTES,
                                allow_empty=False)
        _write_new(raw_path, data, maximum=MAX_LOG_BYTES)
        resolver_paths[role] = str(raw_relative)
    required_files = {
        "schema": CLOSURE.CAPTURE_SCHEMA, "schema_version": 1,
        "status": "CAPTURED_REVIEW_REQUIRED", "distro": plan["distro"],
        "dependency_leg": plan["dependency_leg"],
        "image_digest": plan["image"]["digest"], "platform": CLOSURE.PLATFORM,
        "apt_source_url_policy": dict(plan["apt_source_url_policy"]),
        "release_declaration_policy": dict(plan["release_declaration_policy"]),
        "packages_format_policy": dict(plan["packages_format_policy"]),
        "gpgv_status_policy": dict(plan["gpgv_status_policy"]),
        "requirements_path": "requirements.json", "base_status_path": "base-status.json",
        "source_root": "apt-source", "source_manifest_path": "source-manifest.json",
        "resolvers": {
            "build": {"resolver_path": "build-resolver.json",
                      "raw_log_path": resolver_paths["build"],
                      "deb_root_path": "debs-build",
                      "command": list(plan["phase_argv"]["resolver_build"][3:]),
                      "exit_status": 0},
            "runtime": {"resolver_path": "runtime-resolver.json",
                        "raw_log_path": resolver_paths["runtime"],
                        "deb_root_path": "debs-runtime",
                        "command": list(plan["phase_argv"]["resolver_runtime"][3:]),
                        "exit_status": 0},
        },
        "phase_records": [
            {key: item[key] for key in ("phase", "argv", "argv_sha256", "returncode",
                                        "network_used", "timeout_seconds", "attempts")} | {
                "stdout_path": item["stdout"]["path"],
                "stderr_path": item["stderr"]["path"],
            }
            for item in records
        ],
        "optional_prefetch": dict(optional_prefetch),
    }
    prepare_contract = plan.get("rosdep_prepare_contract")
    if isinstance(prepare_contract, Mapping):
        required_files["rosdep_prepare_contract"] = _immutable_copy(
            prepare_contract)
        required_files["rosdep_prepare"] = _immutable_copy(
            plan.get(
                "_rosdep_prepare",
                _empty_rosdep_prepare_projection(
                    prepare_contract.get("required") is True),
            )
        )
    # This function is called only after the source/resolver/deb files exist;
    # the composer performs the final byte-level check and may still reject it.
    required_files["canonical_sha256"] = canonical_hash(required_files)
    _seal_pair(root, CAPTURE_INPUT_NAME, required_files)
    return required_files


def _cleanup_container(plan: Mapping[str, Any], root: Path, runner: Callable[[list[str], int], Any]) -> dict[str, Any]:
    result = {"stop_requested": False, "remove_requested": False,
              "post_remove_absent": False, "status": "NOT_ATTEMPTED"}
    name = plan["container_name"]
    try:
        rc, stdout, stderr, timed_out, signal = _normalize_output(
            runner(["docker", "inspect", name], 30))
        if rc == 1:
            _inspect_absence(rc, stdout, stderr, name)
            result.update({"status": "ABSENT", "post_remove_absent": True})
            return result
        if rc != 0 or timed_out or signal is not None:
            raise CaptureError("CLEANUP_INSPECT_FAILED", name)
        result["stop_requested"] = True
        stop = _normalize_output(runner(["docker", "stop", name], 60))
        if stop[0] != 0:
            raise CaptureError("CLEANUP_STOP_FAILED", name)
        result["stop_returncode"] = stop[0]
        result["remove_requested"] = True
        remove = _normalize_output(runner(["docker", "rm", name], 60))
        if remove[0] != 0:
            raise CaptureError("CLEANUP_REMOVE_FAILED", name)
        result["remove_returncode"] = remove[0]
        absent = _normalize_output(runner(["docker", "inspect", name], 30))
        if absent[0] != 1:
            raise CaptureError("CLEANUP_ABSENCE_FAILED", name)
        _inspect_absence(absent[0], absent[1], absent[2], name)
        result["post_remove_absent"] = True
        result["status"] = "PASS"
    except Exception as error:
        result["status"] = "FAIL_CLOSED"
        result["error"] = str(error)
    return result


def _artifact_projection(root: Path) -> dict[str, Any]:
    """Describe only complete, sidecar-sealed composer artifacts.

    A failed composer may leave a diagnostic directory, but an artifact is
    included in the outer receipt only after both its immutable file and its
    exact sidecar can be reopened.  This prevents a receipt from silently
    referring to a mutable or half-written candidate.
    """
    result: dict[str, Any] = {}
    # These are the only top-level JSON artifacts produced by the fixed
    # capture workflow.  Source/dependency payloads below them are bound by
    # their inner manifests and by the host directory contract; keeping this
    # list closed prevents an unreviewed file from becoming an implicit
    # receipt input.
    candidates = (
        ("capture_input", root / CAPTURE_INPUT_NAME, MAX_JSON_BYTES),
        ("closure", root / "closure" / CLOSURE.CLOSURE_FILENAME, MAX_JSON_BYTES),
        ("requirements", root / "requirements.json", MAX_JSON_BYTES),
        ("base_status", root / "base-status.json", MAX_JSON_BYTES),
        ("source_manifest", root / "source-manifest.json", MAX_JSON_BYTES),
        ("signature_evidence", root / "signature-evidence.json", MAX_JSON_BYTES),
        ("apt_source_capture", root / "in-container-apt-source.json", MAX_JSON_BYTES),
        ("dependency_capture", root / "in-container-dependency.json", MAX_JSON_BYTES),
        ("rosdep_prepare", root / ROSDEP_PREPARE_RELATIVE /
         ROSDEP_PREPARE_RECEIPT_NAME, MAX_JSON_BYTES),
        ("build_resolver", root / "build-resolver.json", MAX_JSON_BYTES),
        ("runtime_resolver", root / "runtime-resolver.json", MAX_JSON_BYTES),
    )
    for key, path, maximum in candidates:
        if path.exists() or path.is_symlink():
            result[key] = _descriptor(path, key, maximum, sidecar=True)
    return result


# A row receipt is a prefix of the fixed phase plan until the row reaches the
# terminal composer.  Keep the phase-to-output relation closed here instead
# of inferring it from whatever files happen to be present: a future phase
# must never be able to smuggle evidence into an earlier partial receipt.
_PHASE_ARTIFACT_KEYS = {
    "base_status_snapshot": ("base_status",),
    "apt_source_snapshot": ("apt_source_capture",),
    "rosdep_prepare": ("rosdep_prepare",),
    "dependency_capture": ("dependency_capture",),
}
_ARTIFACT_FIRST_PHASE = {
    "base_status": PHASES.index("base_status_snapshot"),
    "apt_source_capture": PHASES.index("apt_source_snapshot"),
    "source_manifest": PHASES.index("apt_source_snapshot"),
    "signature_evidence": PHASES.index("apt_source_snapshot"),
    "rosdep_prepare": PHASES.index("rosdep_prepare"),
    "dependency_capture": PHASES.index("dependency_capture"),
    "requirements": len(PHASES),
    "build_resolver": len(PHASES),
    "runtime_resolver": len(PHASES),
    "capture_input": len(PHASES),
    "closure": len(PHASES),
}
_PHASE_OUTPUT_RELATIVES = {
    "resolver_build": ("debs-build",),
    "download_build": ("debs-build",),
    "resolver_runtime": ("debs-runtime",),
    "download_runtime": ("debs-runtime",),
    # The complete prepare tree (including ``input`` and ``work``) is
    # host-precreated before the container starts.  The receipt path is
    # checked separately above; scanning this parent would mistake those
    # baseline bind directories for a future phase claim.
    "rosdep_prepare": (),
    "dependency_capture": (
        "captured-debs/build", "captured-debs/runtime",
        "licenses/build", "licenses/runtime",
    ),
    "archive_prefetch": ("prefetch-root/prefetch/archives",),
}


_PHASE_OUTPUT_EMPTY_BASELINE_CHILDREN = {
    # APT creates/uses these cache bookkeeping directories.  They are now
    # host-precreated managed children, so their empty presence is a baseline
    # rather than evidence from a future resolver/download phase.  Their own
    # directory contract is still checked for replacement/metadata drift.
    "debs-build": frozenset({"partial"}),
    "debs-runtime": frozenset({"partial"}),
}


def _path_has_entries(path: Path, *, allowed_empty_children: set[str] | frozenset[str] = frozenset()) -> bool:
    """Return whether a path has an unapproved future output claim.

    The fixed capture tree contains a small number of empty bookkeeping
    directories before their owning phase starts.  Only those exact child
    names may be ignored, and they must remain real empty directories; a
    symlink, replacement type, or child file is still a future claim.
    """
    if path.is_symlink():
        return True
    if not path.exists():
        return False
    if not path.is_dir():
        return True
    try:
        entries = list(path.iterdir())
    except OSError:
        # An unreadable output path is not an empty baseline directory.
        return True
    for entry in entries:
        if entry.name not in allowed_empty_children:
            return True
        try:
            info = entry.lstat()
            if not stat.S_ISDIR(info.st_mode) or stat.S_ISLNK(info.st_mode):
                return True
            if next(entry.iterdir(), None) is not None:
                return True
        except OSError:
            return True
    return False


def _validate_phase_prefix_evidence(
        root: Path, receipt: Mapping[str, Any], phases: list[Mapping[str, Any]],
        prepare_contract: Mapping[str, Any]) -> None:
    """Reject future claims before validating phase-specific artifacts.

    The phase list is already checked as an exact ordered prefix by the
    caller.  This second boundary check covers files/claims that are not
    represented by a phase log, including a future rosdep receipt and
    non-empty host-created output directories.
    """
    reached_count = len(phases)
    reached = set(PHASES[:reached_count])
    prepare_required = prepare_contract.get("required") is True
    apt_index = PHASES.index("apt_source_snapshot")

    # APT evidence and signer/package projections cannot exist before the APT
    # source phase has been reached.  Empty deterministic projections remain
    # valid on an earlier failure.
    if reached_count <= apt_index:
        if any(receipt.get(key) for key in (
                "release_evidence", "package_evidence", "signature_references")):
            raise CaptureError("PHASE_FUTURE_EVIDENCE_INVALID", "APT evidence")
        if receipt.get("signature_count") != 0:
            raise CaptureError("PHASE_FUTURE_EVIDENCE_INVALID", "signature count")
        if receipt.get("reference_map") != {
                field: {} for field in REFERENCE_MAP_FIELDS}:
            raise CaptureError("PHASE_FUTURE_EVIDENCE_INVALID", "reference map")

    prepare_projection = receipt.get("rosdep_prepare")
    input_binding = (prepare_projection.get("input_binding")
                     if isinstance(prepare_projection, Mapping) else None)
    empty_prepare = _empty_rosdep_prepare_projection(
        prepare_required, input_binding if prepare_required else None)
    prepare_path = root / ROSDEP_PREPARE_RELATIVE / ROSDEP_PREPARE_RECEIPT_NAME
    if "rosdep_prepare" not in reached:
        if prepare_projection != empty_prepare:
            raise CaptureError(
                "PHASE_FUTURE_EVIDENCE_INVALID", "rosdep_prepare projection")
        if prepare_path.exists() or prepare_path.is_symlink():
            raise CaptureError("PHASE_FUTURE_EVIDENCE_INVALID", str(prepare_path))
    elif not prepare_required and (prepare_path.exists() or prepare_path.is_symlink()):
        raise CaptureError("ROSDEP_PREPARE_UNEXPECTED", str(prepare_path))

    # Phase logs are created only when a phase starts.  A manually planted
    # future log is evidence just as much as a future JSON artifact.
    for phase in PHASES[reached_count:]:
        for stream in ("stdout", "stderr"):
            future_log = root / "logs" / (phase + "." + stream)
            if future_log.exists() or future_log.is_symlink():
                raise CaptureError("PHASE_FUTURE_EVIDENCE_INVALID", str(future_log))

    # The output layout is host-precreated, so empty resolver/deb/cache dirs
    # are harmless baselines.  Any entry in a directory owned by a future
    # phase is an unreferenced future claim and is rejected.
    for phase in PHASES[reached_count:]:
        for relative in _PHASE_OUTPUT_RELATIVES.get(phase, ()):
            path = root / relative
            allowed_children = _PHASE_OUTPUT_EMPTY_BASELINE_CHILDREN.get(
                relative, frozenset())
            if _path_has_entries(path, allowed_empty_children=allowed_children):
                raise CaptureError(
                    "PHASE_FUTURE_EVIDENCE_INVALID", relative)

    # A future artifact key is rejected even if its file is absent; this
    # prevents a self-rehashed receipt from claiming an unexecuted phase.
    for key in receipt.get("artifacts", {}):
        first_phase = _ARTIFACT_FIRST_PHASE.get(key)
        if first_phase is None:
            continue
        terminal_marker = first_phase == len(PHASES) == reached_count
        if first_phase >= reached_count and not terminal_marker:
            raise CaptureError(
                "PHASE_FUTURE_EVIDENCE_INVALID", "artifact " + key)


def _validate_reached_phase_artifacts(
        receipt: Mapping[str, Any], phases: list[Mapping[str, Any]],
        artifacts: Mapping[str, Any], prepare_contract: Mapping[str, Any]) -> None:
    """Require output claims produced by every reached output-bearing phase."""
    reached_count = len(phases)
    for phase, keys in _PHASE_ARTIFACT_KEYS.items():
        if PHASES.index(phase) >= reached_count:
            continue
        if phase == "rosdep_prepare" and prepare_contract.get("required") is not True:
            continue
        for key in keys:
            if key not in artifacts:
                raise CaptureError(
                    "PHASE_EVIDENCE_MISSING", "{}: {}".format(phase, key))

    # A complete production row is not complete merely because all command
    # logs exist.  Every contract-required sealed artifact must be present.
    if (reached_count == len(PHASES) and
            receipt.get("status") == "REVIEW_REQUIRED" and
            prepare_contract.get("required") is True):
        for key in CLOSURE.CAPTURE_REQUIRED_ARTIFACTS:
            if key not in artifacts:
                raise CaptureError("PHASE_EVIDENCE_MISSING", "terminal: " + key)

    for key, first_phase in _ARTIFACT_FIRST_PHASE.items():
        # ``len(PHASES)`` is the terminal-composer marker for capture/closure
        # artifacts.  It is legal only after the complete 19-phase prefix;
        # ordinary phase indexes are future evidence when they are equal to
        # the number of reached records (the next phase has not run yet).
        terminal_marker = first_phase == len(PHASES) == reached_count
        if key in artifacts and first_phase >= reached_count and not terminal_marker:
            raise CaptureError(
                "PHASE_FUTURE_EVIDENCE_INVALID", "artifact " + key)


def _receipt_value(plan: Mapping[str, Any], root_identity: Mapping[str, Any],
                   records: list[dict[str, Any]], image_observation: Mapping[str, Any] | None,
                   cleanup: Mapping[str, Any], status: str, outcome: str,
                   error: Mapping[str, Any] | None, started: int, ended: int,
                   directory_contract: Mapping[str, Any],
                   release_evidence: list[dict[str, Any]],
                   package_evidence: list[dict[str, Any]],
                   signature_references: list[dict[str, Any]] | None = None,
                   signature_count: int = 0,
                   reference_map: Mapping[str, Any] | None = None,
                   reference_counts: Mapping[str, Any] | None = None) -> dict[str, Any]:
    root = Path(root_identity["path"])
    prepare_mounts = _prepare_mount_bindings(plan, root)
    restore_owner = _restore_owner_projection(plan, root, records)
    prepare_argv = plan.get("rosdep_prepare_argv")
    if not isinstance(prepare_argv, Mapping):
        prepare_argv = _prepare_argv_metadata(plan)
    value = {
        "schema": SCHEMA, "schema_version": SCHEMA_VERSION,
        "status": status, "outcome": outcome, "benchmark_eligible": False,
        "distro": plan["distro"], "dependency_leg": plan["dependency_leg"],
        "campaign_set": plan["campaign_set"], "campaign_id": plan["campaign_set"]["campaign_id"],
        "profile": {"path": plan["profile_path"], "sha256": plan["profile_sha256"]},
        "source_manifest": plan["source_manifest"],
        "plan": {"identity_sha256": plan["plan_identity_sha256"],
                 "phase_argv_sha256": plan["phase_argv_sha256"],
                 "apt_source_url_policy": dict(plan["apt_source_url_policy"]),
                 "release_declaration_policy": dict(plan["release_declaration_policy"]),
                 "packages_format_policy": dict(plan["packages_format_policy"]),
                 "gpgv_status_policy": dict(plan["gpgv_status_policy"]),
                 "rosdep_prepare_contract": _immutable_copy(
                     plan["rosdep_prepare_contract"]),
                 "apt_transient_owner": _immutable_copy(
                     plan["apt_transient_owner"]),
                 "rosdep_prepare_mounts": _immutable_copy(prepare_mounts),
                 "rosdep_prepare_bundle": _immutable_copy(
                     plan["rosdep_prepare_bundle"]),
                 "rosdep_prepare_discovery": _immutable_copy(
                     plan["rosdep_prepare_discovery"]),
                 "rosdep_prepare_argv": _immutable_copy(prepare_argv),
                 "rosdep_prepare_input_binding": _immutable_copy(
                     plan.get("_rosdep_prepare_input")),
                 "restore_owner_policy": _immutable_copy(
                     plan["restore_owner_policy"]),
                 "restore_owner_reference_mounts": _immutable_copy(
                     plan["restore_owner_reference_mounts"]),
                 "verification_chain": dict(VERIFICATION_CHAIN)},
        "contract_bindings": CLOSURE.capture_contract_bindings(),
        "capture_contract": CLOSURE.capture_contract(),
        "image": {"reference": plan["image"]["reference"], "digest": plan["image"]["digest"],
                  "platform": {"os": "linux", "architecture": "amd64"},
                  "observed": image_observation},
        "container": {"name": plan["container_name"], "output_root": plan["output_root"]},
        "root": dict(root_identity), "phases": records,
        "restore_owner": restore_owner,
        "release_evidence": list(release_evidence),
        "package_evidence": list(package_evidence),
        "rosdep_prepare": _immutable_copy(
            plan.get("_rosdep_prepare", _empty_rosdep_prepare_projection(
                plan["rosdep_prepare_contract"].get("required") is True))),
        "signature_references": _immutable_copy(signature_references or []),
        "signature_count": signature_count,
        "reference_map": _immutable_copy(
            reference_map or {field: {} for field in REFERENCE_MAP_FIELDS}),
        "reference_counts": _immutable_copy(
            reference_counts or _reference_projection([], [], [], signature_count)[1]),
        "artifacts": _artifact_projection(root),
        "directory_contract": dict(directory_contract),
        "signature_evidence": {
            "status": "REQUIRED_NOT_PROVEN", "runtime": "NOT_RUN", "required": True,
            "promotion": "FORBIDDEN_UNTIL_SIGNED_RECEIPT",
        },
        "verification_chain": dict(VERIFICATION_CHAIN),
        "safety": {
            "provisioning_network_used": any(item["network_used"] for item in records),
            "archive_fetch_network_used": any(
                item["phase"] == "archive_prefetch" and item["network_used"]
                for item in records),
            "build_test_network_connected": False, "build_test_network_used": False,
            "docker_pull": False, "docker_build": False, "bag_opened": False,
            "ground_truth_opened": False, "scorer_opened": False, "map_written": False,
        },
        "cleanup": dict(cleanup), "started_at_unix": started, "ended_at_unix": ended,
        "error": dict(error) if error else None,
    }
    value["canonical_sha256"] = canonical_hash(value)
    return value


def validate_receipt(root: Path, *, expected_plan: Mapping[str, Any] | None = None) -> dict[str, Any]:
    """Reopen an executor receipt and reject self-rehashed or mutable output."""
    root = _safe_abs(str(root), "capture receipt root")
    if not root.is_dir() or root.is_symlink():
        raise CaptureError("RECEIPT_ROOT_INVALID", str(root))
    path = root / RECEIPT_NAME
    value, info = _read_bounded(path, "capture receipt", MAX_RECEIPT_BYTES,
                                allow_empty=False)
    _descriptor(path, "capture receipt", MAX_RECEIPT_BYTES, sidecar=True)
    try:
        receipt = json.loads(value.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError("RECEIPT_INVALID", str(error)) from error
    if not isinstance(receipt, Mapping):
        raise CaptureError("RECEIPT_INVALID", "object required")
    required = {
        "schema", "schema_version", "status", "outcome", "benchmark_eligible", "distro",
        "dependency_leg", "campaign_set", "campaign_id", "profile", "source_manifest", "plan",
        "contract_bindings", "capture_contract",
        "image", "container", "root", "phases", "artifacts", "directory_contract",
        "release_evidence", "package_evidence", "signature_evidence",
        "rosdep_prepare", "restore_owner",
        "signature_references", "signature_count", "reference_map", "reference_counts",
        "verification_chain", "safety", "cleanup",
        "started_at_unix", "ended_at_unix", "error", "canonical_sha256",
    }
    if set(receipt) != required or receipt["schema"] != SCHEMA or receipt["schema_version"] != 1 or \
            receipt["benchmark_eligible"] is not False or receipt["canonical_sha256"] != canonical_hash(receipt):
        raise CaptureError("RECEIPT_IDENTITY_INVALID", "fields/status/hash")
    if not isinstance(receipt["profile"], Mapping) or \
            set(receipt["profile"]) != {"path", "sha256"}:
        raise CaptureError("PROFILE_BINDING_INVALID", "profile fields")
    _safe_abs(receipt["profile"]["path"], "profile path")
    _sha(receipt["profile"]["sha256"], "profile SHA")
    prepare_contract = receipt["plan"].get("rosdep_prepare_contract") if isinstance(
        receipt.get("plan"), Mapping) else None
    if not isinstance(prepare_contract, Mapping):
        raise CaptureError("ROSDEP_PREPARE_CONTRACT_INVALID", "receipt plan")
    expected_prepare_contract = _rosdep_prepare_contract(
        required=prepare_contract.get("required") is True,
        profile_path=Path(receipt["profile"]["path"]),
        profile_sha256=receipt["profile"]["sha256"],
    )
    if dict(prepare_contract) != expected_prepare_contract:
        raise CaptureError("ROSDEP_PREPARE_CONTRACT_INVALID", "receipt plan drift")
    if not isinstance(receipt["source_manifest"], Mapping):
        raise CaptureError("SOURCE_BINDING_INVALID", "source manifest")
    if receipt["contract_bindings"] != CLOSURE.capture_contract_bindings():
        raise CaptureError("CONTRACT_BINDING_INVALID", "capture/source contract identities")
    if receipt["capture_contract"] != CLOSURE.capture_contract():
        raise CaptureError("CAPTURE_CONTRACT_BINDING_INVALID", "capture contract")
    if not isinstance(receipt["image"], Mapping) or set(receipt["image"]) != {
            "reference", "digest", "platform", "observed"}:
        raise CaptureError("IMAGE_BINDING_INVALID", "image fields")
    _digest(receipt["image"]["digest"], "image digest")
    if receipt["image"]["platform"] != {"os": "linux", "architecture": "amd64"}:
        raise CaptureError("IMAGE_BINDING_INVALID", "platform")
    if not isinstance(receipt["container"], Mapping) or set(receipt["container"]) != {
            "name", "output_root"} or \
            NAME_RE.fullmatch(receipt["container"]["name"]) is None:
        raise CaptureError("CONTAINER_BINDING_INVALID", "container fields")
    root_value = receipt["root"]
    if not isinstance(root_value, Mapping) or set(root_value) != {
            "path", "device", "inode", "mode", "nlink"}:
        raise CaptureError("ROOT_BINDING_INVALID", "root fields")
    if root_value["path"] != str(root):
        raise CaptureError("ROOT_BINDING_INVALID", "root path")
    if (type(root_value["device"]) is not int or type(root_value["inode"]) is not int or
            type(root_value["mode"]) is not int or type(root_value["nlink"]) is not int or
            root_value["mode"] != 0o700 or root_value["nlink"] < 2):
        raise CaptureError("ROOT_BINDING_INVALID", "root identity")
    if not isinstance(receipt["campaign_set"], Mapping) or \
            receipt["campaign_id"] != receipt["campaign_set"].get("campaign_id"):
        raise CaptureError("CAMPAIGN_BINDING_INVALID", "campaign id")
    if receipt["status"] not in {"REVIEW_REQUIRED", "PARTIAL_FAILURE_REVIEW_REQUIRED"}:
        raise CaptureError("RECEIPT_STATUS_INVALID", receipt["status"])
    if receipt["outcome"] not in {"PASS_REVIEW_REQUIRED", "PARTIAL_FAILURE"}:
        raise CaptureError("RECEIPT_OUTCOME_INVALID", receipt["outcome"])
    if (type(receipt["started_at_unix"]) is not int or type(receipt["ended_at_unix"]) is not int or
            receipt["started_at_unix"] < 0 or receipt["ended_at_unix"] < receipt["started_at_unix"]):
        raise CaptureError("RECEIPT_TIME_INVALID", "timestamps")
    plan_value = receipt["plan"]
    if not isinstance(plan_value, Mapping) or set(plan_value) != {
            "identity_sha256", "phase_argv_sha256", "apt_source_url_policy",
            "release_declaration_policy", "packages_format_policy", "gpgv_status_policy",
            "rosdep_prepare_contract", "apt_transient_owner", "rosdep_prepare_mounts",
            "rosdep_prepare_bundle", "rosdep_prepare_discovery",
            "rosdep_prepare_argv", "rosdep_prepare_input_binding",
            "restore_owner_policy", "restore_owner_reference_mounts",
            "verification_chain"} or \
            not isinstance(plan_value["phase_argv_sha256"], Mapping) or \
            set(plan_value["phase_argv_sha256"]) != set(PHASES):
        raise CaptureError("PLAN_BINDING_INVALID", "plan fields")
    _sha(plan_value["identity_sha256"], "plan identity SHA")
    for phase in PHASES:
        _sha(plan_value["phase_argv_sha256"][phase], phase + " command SHA")
    if plan_value["apt_source_url_policy"] != APT_SOURCE_URL_POLICY or \
            plan_value["release_declaration_policy"] != RELEASE_DECLARATION_POLICY or \
            plan_value["packages_format_policy"] != PACKAGES_FORMAT_POLICY or \
            plan_value["gpgv_status_policy"] != GPGV_STATUS_POLICY or \
            plan_value["verification_chain"] != VERIFICATION_CHAIN or \
            plan_value["restore_owner_policy"] != RESTORE_OWNER_POLICY:
        raise CaptureError("PLAN_BINDING_INVALID", "URL policy or verification chain")
    if plan_value["rosdep_prepare_bundle"] != prepare_contract.get("input_bundle") or \
            plan_value["rosdep_prepare_discovery"] != prepare_contract.get("discovery"):
        raise CaptureError("PREPARE_PLAN_BINDING_INVALID", "bundle/discovery")
    prepare_argv = plan_value["rosdep_prepare_argv"]
    expected_prepare_argv = _prepare_phase_argv(
        receipt["container"]["name"],
        Path(receipt["container"]["output_root"]),
        receipt["distro"],
    )
    expected_tool_sha = (prepare_contract.get("input_bundle", {}).get("files", [{}])[0]
                         .get("sha256"))
    if (not isinstance(prepare_argv, Mapping) or set(prepare_argv) != {
            "argv", "argv_sha256", "script_path", "script_sha256"} or
            prepare_argv.get("argv") != expected_prepare_argv or
            prepare_argv.get("argv_sha256") != _command_hash(expected_prepare_argv) or
            prepare_argv.get("argv_sha256") != plan_value["phase_argv_sha256"]["rosdep_prepare"] or
            prepare_argv.get("script_path") != ROSDEP_PREPARE_CONTAINER_TOOL or
            prepare_argv.get("script_sha256") != expected_tool_sha):
        raise CaptureError("PREPARE_ARGV_INVALID", "receipt plan")
    phases = receipt["phases"]
    if not isinstance(phases, list) or not 1 <= len(phases) <= len(PHASES) or \
            [item.get("phase") for item in phases] != list(PHASES[:len(phases)]):
        raise CaptureError("PHASE_ORDER_INVALID", "receipt phases")
    prepare_reached = "rosdep_prepare" in {item.get("phase") for item in phases}
    post_inspect = next((item for item in phases
                         if item.get("phase") == "post_disconnect_inspect"), None)
    runtime_required = bool(
        prepare_contract.get("required") is True and
        isinstance(post_inspect, Mapping) and post_inspect.get("returncode") == 0)
    _validate_prepare_mount_bindings(
        plan_value["rosdep_prepare_mounts"], root,
        required=prepare_contract.get("required") is True,
        prepare_reached=prepare_reached,
        runtime_required=runtime_required)
    if plan_value["restore_owner_reference_mounts"] != _restore_owner_mount_specs(root):
        raise CaptureError("RESTORE_OWNER_MOUNT_INVALID", "plan mounts")
    _apt_transient_owner_binding(
        plan_value,
        profile_sha256=receipt["profile"]["sha256"],
        image_digest=receipt["image"]["digest"])
    _validate_restore_owner_projection(
        receipt["restore_owner"], root, plan_value, phases,
        container_name=receipt["container"]["name"],
        profile_sha256=receipt["profile"]["sha256"],
        image_digest=receipt["image"]["digest"])
    if plan_value["rosdep_prepare_input_binding"] != receipt["rosdep_prepare"].get(
            "input_binding"):
        raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "plan/projection")
    if plan_value["rosdep_prepare_input_binding"] is not None:
        binding_plan = {
            "profile_path": receipt["profile"]["path"],
            "profile_sha256": receipt["profile"]["sha256"],
            "rosdep_prepare_contract": prepare_contract,
        }
        _verify_prepare_input_binding(
            root, binding_plan, plan_value["rosdep_prepare_input_binding"])
    elif prepare_contract.get("required") is True and prepare_reached:
        raise CaptureError("PREPARE_INPUT_BINDING_INVALID", "missing reached binding")
    for index, (item, phase) in enumerate(zip(phases, PHASES)):
        if not isinstance(item, Mapping) or set(item) != {
                "phase", "argv", "argv_sha256", "returncode", "network_used", "timeout_seconds",
                "attempts", "stdout", "stderr", "timed_out", "signal",
                "observed_returncode"}:
            raise CaptureError("PHASE_FIELDS_INVALID", phase)
        argv = _validate_argv(item["argv"], phase)
        if item["argv_sha256"] != _command_hash(argv) or item["phase"] != phase or \
                type(item["returncode"]) is not int or type(item["observed_returncode"]) is not int or \
                item["attempts"] != 1 or type(item["timed_out"]) is not bool or \
                (item["signal"] is not None and type(item["signal"]) is not int):
            raise CaptureError("PHASE_IDENTITY_INVALID", phase)
        if phase == "preexisting_container_check":
            if item["returncode"] != 0 or item["observed_returncode"] != DOCKER_MISSING_RC:
                raise CaptureError("PHASE_IDENTITY_INVALID", phase)
        elif item["returncode"] != item["observed_returncode"] and not (
                phase in RESTORE_OWNER_PHASES and
                item["returncode"] == 1 and item["observed_returncode"] == 0):
            raise CaptureError("PHASE_IDENTITY_INVALID", phase)
        if item["returncode"] != 0 and index != len(phases) - 1:
            raise CaptureError("PHASE_ORDER_INVALID", "phase after failure")
        expected_network = PHASE_NETWORK[phase]
        if expected_network is None:
            expected_network = receipt["dependency_leg"] == "present"
        if item["network_used"] is not expected_network:
            raise CaptureError("PHASE_NETWORK_INVALID", phase)
        if not isinstance(item["stdout"], Mapping) or not isinstance(item["stderr"], Mapping):
            raise CaptureError("PHASE_LOG_INVALID", phase)
        for stream in ("stdout", "stderr"):
            log = item[stream]
            rel = _safe_rel(log.get("path"), phase + " log")
            _sha(log.get("sha256"), phase + " log SHA")
            actual = _descriptor(root / rel, phase + " log", MAX_LOG_BYTES)
            if actual["bytes"] != log.get("bytes") or actual["sha256"] != log.get("sha256"):
                raise CaptureError("PHASE_LOG_DRIFT", phase)
    disconnected = False
    for item in phases:
        if item["phase"] == "network_disconnect":
            disconnected = item["returncode"] == 0
        elif item["phase"] in {"rosdep_prepare", "rosdep_resolution"} and not disconnected:
            raise CaptureError("NETWORK_DISCONNECT_REQUIRED", item["phase"])

    # Validate the phase prefix before touching any future-phase projection.
    # In particular, an APT failure must be reopenable without a rosdep
    # receipt that the executor was never allowed to create.
    _validate_phase_prefix_evidence(root, receipt, phases, prepare_contract)
    if "rosdep_prepare" in {item["phase"] for item in phases}:
        _rosdep_prepare_projection(
            root, {"rosdep_prepare_contract": prepare_contract},
            allow_partial=receipt["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED",
            projection=receipt["rosdep_prepare"])

    release_evidence = _validate_release_evidence(
        root, receipt["release_evidence"])
    package_evidence = _validate_package_evidence(
        root, receipt["package_evidence"],
        require_decoded=receipt["status"] == "REVIEW_REQUIRED")
    signature_references = _validate_signature_references(
        receipt["signature_references"], receipt["signature_count"],
        require_complete=receipt["status"] == "REVIEW_REQUIRED")
    _validate_reference_projection(
        receipt["reference_map"], receipt["reference_counts"],
        receipt["release_evidence"], receipt["package_evidence"],
        signature_references, receipt["signature_count"],
        require_complete=receipt["status"] == "REVIEW_REQUIRED")
    apt_capture_path = root / "in-container-apt-source.json"
    if apt_capture_path.exists() or apt_capture_path.is_symlink():
        directory_wrapper = _read_directory_contract(
            root, include_prefetch=receipt["dependency_leg"] == "present",
            receipt_status=receipt["status"])
        directory_value = directory_wrapper.get("directory") \
            if isinstance(directory_wrapper, Mapping) else None
        host_contract = None
        if isinstance(directory_value, Mapping):
            host_contract = {
                key: directory_value[key]
                for key in ("schema", "schema_version", "root", "mode", "owner",
                            "paths", "root_before", "before")
                if key in directory_value
            }
        inner_value = _validate_inner_directory_binding(apt_capture_path, host_contract)
        inner_evidence = inner_value.get("release_evidence", [])
        _validate_release_evidence(
            root, inner_evidence,
            require_nonempty=inner_value.get("status") == "CAPTURED_REVIEW_REQUIRED" and
            "source_manifest" in inner_value)
        if release_evidence != inner_evidence:
            raise CaptureError("RELEASE_EVIDENCE_BINDING_INVALID", "outer/inner")
        inner_references = inner_value.get("signature_references", [])
        inner_count = inner_value.get("signature_count", 0)
        _validate_signature_references(
            inner_references, inner_count,
            require_complete=inner_value.get("status") == "CAPTURED_REVIEW_REQUIRED" and
            "source_manifest" in inner_value)
        inner_packages = inner_value.get("package_evidence", [])
        _validate_package_evidence(
            root, inner_packages,
            require_nonempty=inner_value.get("status") == "CAPTURED_REVIEW_REQUIRED" and
            "source_manifest" in inner_value,
            require_decoded=inner_value.get("status") == "CAPTURED_REVIEW_REQUIRED" and
            "source_manifest" in inner_value)
        inner_has_references = any(
            key in inner_value for key in (
                "signature_references", "signature_count", "source_manifest"))
        inner_has_projection = any(
            key in inner_value for key in ("reference_map", "reference_counts"))
        if ((inner_has_projection and (
                "reference_map" not in inner_value or
                "reference_counts" not in inner_value)) or
                (inner_has_references and not inner_has_projection)):
            raise CaptureError("REFERENCE_PROJECTION_MISSING", "outer/inner")
        if inner_has_projection:
            _validate_reference_projection(
                inner_value["reference_map"], inner_value["reference_counts"],
                inner_evidence, inner_packages, inner_references, inner_count,
                    require_complete=inner_value.get("status") == "CAPTURED_REVIEW_REQUIRED" and
                    "source_manifest" in inner_value)
        if (signature_references != inner_references or
                receipt["signature_count"] != inner_count or
                (inner_has_projection and
                 receipt["reference_map"] != inner_value["reference_map"]) or
                (inner_has_projection and
                 receipt["reference_counts"] != inner_value["reference_counts"]) or
                (not inner_has_projection and
                 receipt["reference_map"] != {field: {} for field in REFERENCE_MAP_FIELDS}) or
                (not inner_has_projection and
                 receipt["reference_counts"] != _reference_projection([], [], [], 0)[1])):
            raise CaptureError("SIGNATURE_REFERENCE_BINDING_INVALID", "outer/inner")
        if package_evidence != inner_packages:
            raise CaptureError("PACKAGES_EVIDENCE_BINDING_INVALID", "outer/inner")
    full_success = len(phases) == len(PHASES) and all(
        item["returncode"] == 0 and item["observed_returncode"] == 0
        for index, item in enumerate(phases) if index != 1)
    if len(phases) == len(PHASES) and phases[1]["observed_returncode"] != DOCKER_MISSING_RC:
        full_success = False
    if receipt["status"] == "REVIEW_REQUIRED" or receipt["outcome"] == "PASS_REVIEW_REQUIRED":
        if not full_success or receipt["status"] != "REVIEW_REQUIRED" or \
                receipt["outcome"] != "PASS_REVIEW_REQUIRED" or receipt["error"] is not None:
            raise CaptureError("RECEIPT_STATUS_INVALID", "success requires all phases")
    else:
        if receipt["status"] != "PARTIAL_FAILURE_REVIEW_REQUIRED" or \
                receipt["outcome"] != "PARTIAL_FAILURE":
            raise CaptureError("RECEIPT_STATUS_INVALID", "partial status/outcome")
    artifacts = receipt["artifacts"]
    allowed_artifacts = {
        "capture_input", "closure", "requirements", "base_status", "source_manifest",
        "signature_evidence", "apt_source_capture", "dependency_capture",
        "rosdep_prepare", "build_resolver", "runtime_resolver",
    }
    if not isinstance(artifacts, Mapping) or set(artifacts) - allowed_artifacts:
        raise CaptureError("ARTIFACT_BINDING_INVALID", "artifact keys")
    actual_artifacts = _artifact_projection(root)
    _validate_reached_phase_artifacts(
        receipt, phases, actual_artifacts, prepare_contract)
    if set(artifacts) != set(actual_artifacts):
        raise CaptureError("ARTIFACT_BINDING_INVALID", "artifact set")
    for key in artifacts:
        if artifacts[key] != actual_artifacts[key]:
            raise CaptureError("ARTIFACT_BINDING_INVALID", key)
    cleanup = receipt["cleanup"]
    if not isinstance(cleanup, Mapping):
        raise CaptureError("CLEANUP_INVALID", "cleanup object")
    cleanup_keys = set(cleanup)
    if not cleanup_keys.issubset({"status", "stop_requested", "remove_requested",
                                  "post_remove_absent", "stop_returncode", "remove_returncode",
                                  "error"}):
        raise CaptureError("CLEANUP_INVALID", "cleanup fields")
    if cleanup.get("status") not in {"ABSENT", "PASS", "FAIL_CLOSED", "NOT_ATTEMPTED"}:
        raise CaptureError("CLEANUP_INVALID", "cleanup status")
    for key in ("stop_requested", "remove_requested", "post_remove_absent"):
        if type(cleanup.get(key)) is not bool:
            raise CaptureError("CLEANUP_INVALID", key)
    if cleanup["status"] == "PASS" and (cleanup["stop_requested"] is not True or
                                          cleanup["remove_requested"] is not True or
                                          cleanup["post_remove_absent"] is not True or
                                          cleanup.get("stop_returncode") != 0 or
                                          cleanup.get("remove_returncode") != 0):
        raise CaptureError("CLEANUP_INVALID", "successful cleanup")
    if cleanup["status"] == "ABSENT" and (cleanup["stop_requested"] or
                                           cleanup["remove_requested"] or
                                           cleanup["post_remove_absent"] is not True):
        raise CaptureError("CLEANUP_INVALID", "absent cleanup")
    if receipt["error"] is not None and (
            not isinstance(receipt["error"], Mapping) or
            set(receipt["error"]) != {"kind", "message"}):
        raise CaptureError("RECEIPT_ERROR_INVALID", "error object")
    directory_contract = receipt["directory_contract"]
    expected_directory_contract = _sealed_pair_projection(
        root, DIRECTORY_CONTRACT_NAME, "directory contract", MAX_JSON_BYTES)
    if directory_contract != expected_directory_contract:
        raise CaptureError("DIRECTORY_CONTRACT_BINDING_INVALID", "sealed pair")
    # The JSON receipt schema intentionally binds the host-owned directory
    # identity as a separate immutable companion.  Reopen both files and the
    # live directories; a receipt without that exact pair is never accepted.
    _read_directory_contract(
        root, include_prefetch=receipt["dependency_leg"] == "present",
        receipt_status=receipt["status"])
    signature = receipt["signature_evidence"]
    if signature != {"status": "REQUIRED_NOT_PROVEN", "runtime": "NOT_RUN", "required": True,
                     "promotion": "FORBIDDEN_UNTIL_SIGNED_RECEIPT"}:
        raise CaptureError("SIGNATURE_STATUS_INVALID", "signature evidence")
    if receipt["verification_chain"] != VERIFICATION_CHAIN:
        raise CaptureError("VERIFICATION_CHAIN_INVALID", "verification chain")
    safety = receipt["safety"]
    if (not isinstance(safety, Mapping) or safety.get("build_test_network_connected") is not False or
            safety.get("build_test_network_used") is not False or safety.get("docker_pull") is not False or
            safety.get("docker_build") is not False):
        raise CaptureError("SAFETY_POLICY_INVALID", "runtime network/pull/build")
    if expected_plan is not None:
        expected_mounts = expected_plan.get("rosdep_prepare_mounts")
        observed_mounts = receipt["plan"].get("rosdep_prepare_mounts")
        if (not isinstance(expected_mounts, list) or
                not isinstance(observed_mounts, list) or
                len(expected_mounts) != len(observed_mounts) or
                any({key: item.get(key) for key in (
                        "source", "target", "read_only", "type", "noexec")} != fixed
                    for item, fixed in zip(observed_mounts, expected_mounts))):
            raise CaptureError("PREPARE_MOUNT_SPEC_INVALID", "expected plan")
        expected_restore_mounts = expected_plan.get("restore_owner_reference_mounts")
        observed_restore_mounts = receipt["plan"].get(
            "restore_owner_reference_mounts")
        if (not isinstance(expected_restore_mounts, list) or
                observed_restore_mounts != expected_restore_mounts):
            raise CaptureError("RESTORE_OWNER_MOUNT_INVALID", "expected plan")
        if receipt["plan"].get("restore_owner_policy") != expected_plan.get(
                "restore_owner_policy"):
            raise CaptureError("RESTORE_OWNER_POLICY_INVALID", "expected plan")
        if receipt["plan"].get("rosdep_prepare_bundle") != expected_plan.get(
                "rosdep_prepare_bundle") or \
                receipt["plan"].get("rosdep_prepare_discovery") != expected_plan.get(
                    "rosdep_prepare_discovery") or \
                receipt["plan"].get("rosdep_prepare_argv") != expected_plan.get(
                    "rosdep_prepare_argv"):
            raise CaptureError("PREPARE_PLAN_BINDING_INVALID", "expected plan")
        if receipt["plan"]["identity_sha256"] != expected_plan["plan_identity_sha256"] or \
                receipt["profile"]["path"] != expected_plan["profile_path"] or \
                receipt["profile"]["sha256"] != expected_plan["profile_sha256"] or \
                receipt["image"]["digest"] != expected_plan["image"]["digest"] or \
                receipt["container"]["name"] != expected_plan["container_name"] or \
                receipt["container"]["output_root"] != expected_plan["output_root"]:
            raise CaptureError("PLAN_BINDING_INVALID", "expected plan")
    return {"status": receipt["status"], "outcome": receipt["outcome"],
            "benchmark_eligible": False, "receipt_sha256": sha256_bytes(value),
            "bytes": info.st_size}


def capture_dependency_closure(repo_root: Path, profile_path: Path, distro: str,
                               dependency_leg: str, output_root: Path,
                               *, runner: Callable[[list[str], int], Any] | None = None,
                               prefetcher: Callable[..., Any] | None = None,
                               clock: Callable[[], int] | None = None) -> dict[str, Any]:
    """Run one fixed provisioning row and seal a non-promoting receipt.

    No caller-provided argv is accepted.  The function stops on the first
    phase failure, records cleanup, and keeps the owned root for diagnosis.
    A complete APT source graph is then handed to the existing composer; if
    any graph or signature requirement is absent the result remains a truthful
    partial review receipt.
    """
    plan = build_plan(Path(repo_root), Path(profile_path), distro, dependency_leg, Path(output_root))
    root_identity = _fresh_root(Path(output_root), "capture output root")
    root = Path(output_root)
    run = runner or _default_runner
    now = clock or (lambda: int(time.time()))
    started = int(now())
    records: list[dict[str, Any]] = []
    image_observation = None
    cleanup = {"status": "NOT_ATTEMPTED", "stop_requested": False,
               "remove_requested": False, "post_remove_absent": False}
    status = "PARTIAL_FAILURE_REVIEW_REQUIRED"
    outcome = "PARTIAL_FAILURE"
    error = None
    release_evidence: list[dict[str, Any]] = []
    package_evidence: list[dict[str, Any]] = []
    signature_references: list[dict[str, Any]] = []
    signature_count = 0
    reference_map: dict[str, Any] = {
        field: {} for field in REFERENCE_MAP_FIELDS
    }
    reference_counts: dict[str, int] = _reference_projection(
        [], [], [], 0)[1]
    rosdep_prepare = _empty_rosdep_prepare_projection(
        plan["rosdep_prepare_contract"].get("required") is True)
    network_disconnected = False
    directory_contract = None
    directory_contract_seal = None
    try:
        directory_contract = _create_capture_directory_layout(
            root, include_prefetch=dependency_leg == "present")
        plan["_restore_owner_records"] = {}
        plan["_restore_owner_reference_before"] = [
            _restore_reference_descriptor(
                Path(item["source"]), "restore reference pre-container " + item["role"])
            for item in plan["restore_owner_reference_mounts"]
        ]
        # Seal the host-owned, empty partial targets before Docker starts.  APT
        # may change only their transient sandbox owner while it downloads;
        # the restore phase must compare the later lstat identity against this
        # descriptor without attempting to scan an unreadable _apt directory.
        plan["_restore_owner_target_before"] = {
            "build": _restore_directory_snapshot(
                root / "debs-build/partial",
                "restore target pre-container build"),
            "runtime": _restore_directory_snapshot(
                root / "debs-runtime/partial",
                "restore target pre-container runtime"),
        }
        if plan["rosdep_prepare_contract"].get("required") is True:
            input_binding = _materialize_prepare_input(root, plan)
            plan["_rosdep_prepare_input"] = _immutable_copy(input_binding)
            rosdep_prepare = _empty_rosdep_prepare_projection(
                True, input_binding)
            plan["_rosdep_prepare"] = _immutable_copy(rosdep_prepare)
        plan["_rosdep_prepare_mounts_before"] = _prepare_mount_descriptor_snapshot(
            plan, root, "prepare mount pre-container")
        image_result = _normalize_output(run(plan["phase_argv"]["image_inspect"], IMAGE_INSPECT_TIMEOUT_SECONDS))
        image_record = _phase_record(plan, root, "image_inspect", image_result)
        records.append(image_record)
        if image_result[0] != 0:
            raise CaptureError("IMAGE_INSPECT_FAILED", str(image_result[0]))
        image_observation = _inspect_image(image_result[1], plan["image"])

        absence = _normalize_output(run(plan["phase_argv"]["preexisting_container_check"], 30))
        absence_record = _phase_record(
            plan, root, "preexisting_container_check", absence,
            logical_returncode=0 if absence[0] == DOCKER_MISSING_RC else absence[0])
        records.append(absence_record)
        _inspect_absence(absence[0], absence[1], absence[2], plan["container_name"])
        if absence[0] != DOCKER_MISSING_RC:
            raise CaptureError("CONTAINER_COLLISION", plan["container_name"])

        for phase in PHASES[2:]:
            if phase == "archive_prefetch":
                record, optional_prefetch = _run_archive_prefetch(plan, root, prefetcher)
            elif phase in RESTORE_OWNER_PHASES:
                record, restore_evidence = _run_restore_owner_phase(
                    plan, root, phase, run)
            else:
                record = _run_phase(plan, root, phase, run)
            records.append(record)
            if phase in RESTORE_OWNER_PHASES and restore_evidence["status"] != "PASS":
                raise CaptureError("RESTORE_OWNER_CONTRACT_INVALID", phase)
            if phase == "base_status_snapshot":
                _base_status_from_log(root, record)
            if phase == "apt_source_snapshot":
                # The helper's status is deliberately inspected, but an
                # incomplete helper result cannot be promoted.
                output = root / "in-container-apt-source.json"
                if not output.exists():
                    raise CaptureError("SOURCE_CAPTURE_MISSING", str(output))
                try:
                    inner = _validate_inner_directory_binding(output, directory_contract)
                except CaptureError as inner_error:
                    if inner_error.release_evidence is not None:
                        release_evidence = list(inner_error.release_evidence)
                    if inner_error.package_evidence is not None:
                        package_evidence = list(inner_error.package_evidence)
                    if inner_error.signature_references is not None:
                        signature_references = _immutable_copy(
                            inner_error.signature_references)
                    if inner_error.signature_count is not None:
                        signature_count = inner_error.signature_count
                    if inner_error.reference_map is not None:
                        reference_map = _immutable_copy(inner_error.reference_map)
                    if inner_error.reference_counts is not None:
                        reference_counts = _immutable_copy(inner_error.reference_counts)
                    raise
                release_evidence = list(inner.get("release_evidence", []))
                package_evidence = list(inner.get("package_evidence", []))
                signature_references = _immutable_copy(
                    inner.get("signature_references", []))
                signature_count = inner.get(
                    "signature_count", signature_count)
                reference_map = _immutable_copy(inner.get(
                    "reference_map", reference_map))
                reference_counts = _immutable_copy(inner.get(
                    "reference_counts", reference_counts))
            if phase == "dependency_capture":
                output = root / "in-container-dependency.json"
                if not output.exists():
                    raise CaptureError("DEPENDENCY_CAPTURE_MISSING", str(output))
                _validate_inner_directory_binding(output, directory_contract)
            if phase in {"rosdep_prepare", "rosdep_resolution"} and not network_disconnected:
                raise CaptureError("NETWORK_DISCONNECT_REQUIRED", phase)
            if record["timed_out"] or record["signal"] is not None:
                raise CaptureError("COMMAND_TIMEOUT_OR_SIGNAL", phase)
            if record["observed_returncode"] != 0:
                raise CaptureError("PHASE_FAILED", "{} exited {}".format(
                    phase, record["observed_returncode"]))
            if phase == "network_disconnect":
                network_disconnected = True
            if phase == "rosdep_prepare":
                rosdep_prepare = _rosdep_prepare_projection(
                    root, plan, allow_partial=True)
                plan["_rosdep_prepare"] = _immutable_copy(rosdep_prepare)
                if (plan["rosdep_prepare_contract"].get("required") is True and
                        rosdep_prepare["status"] != "REVIEW_REQUIRED"):
                    raise CaptureError(
                        "ROSDEP_PREPARE_INCOMPLETE", rosdep_prepare["status"])
        plan["_rosdep_prepare_mounts_runtime"] = _prepare_runtime_mount_observation(
            root, plan, records)
        plan["_restore_owner_reference_runtime"] = _restore_owner_runtime_mount_observation(
            root, plan, records)
        if "optional_prefetch" not in locals():
            optional_prefetch = {"status": "NOT_APPLICABLE"}
        _compose_candidate(plan, root, records, optional_prefetch)
        status, outcome = "REVIEW_REQUIRED", "PASS_REVIEW_REQUIRED"
    except Exception as exc:
        if isinstance(exc, CaptureError):
            error = {"kind": exc.kind, "message": str(exc)}
        else:
            error = {"kind": "UNEXPECTED_CAPTURE_FAILURE", "message": str(exc)}
    finally:
        try:
            _assert_root(root, root_identity, "capture output root")
            cleanup = _cleanup_container(plan, root, run)
            plan["_rosdep_prepare_mounts_after"] = _prepare_mount_descriptor_snapshot(
                plan, root, "prepare mount post-container")
            plan["_restore_owner_reference_after"] = [
                _restore_reference_descriptor(
                    Path(item["source"]), "restore reference post-container " + item["role"])
                for item in plan.get("restore_owner_reference_mounts", [])
            ]
        except Exception as exc:
            if error is None:
                error = {
                    "kind": exc.kind if isinstance(exc, CaptureError)
                    else "CLEANUP_OR_MOUNT_VALIDATION_FAILED",
                    "message": str(exc),
                }
            cleanup = {"status": "FAIL_CLOSED", "stop_requested": False,
                       "remove_requested": False, "post_remove_absent": False,
                       "error": str(exc)}
        if directory_contract is not None:
            try:
                directory_contract = _verify_capture_directory_layout(
                    root, directory_contract,
                    allow_terminal_closure=(root / "closure").exists())
            except CaptureError as exc:
                directory_contract = EVIDENCE_DIRECTORIES.failed(directory_contract, exc)
                if error is None:
                    error = {"kind": exc.kind, "message": str(exc)}
        else:
            directory_contract = EVIDENCE_DIRECTORIES.failed(
                None, error or "host layout was not created")
    ended = int(now())
    if error is None and cleanup.get("status") == "PASS":
        status, outcome = "REVIEW_REQUIRED", "PASS_REVIEW_REQUIRED"
    elif error is None:
        error = {"kind": "CLEANUP_FAILED", "message": cleanup.get("error", "cleanup incomplete")}
    directory_contract_seal = _seal_directory_contract(root, directory_contract)
    value = _receipt_value(plan, root_identity, records, image_observation,
                           cleanup, status, outcome, error, started, ended,
                           directory_contract_seal, release_evidence,
                           package_evidence, signature_references,
                           signature_count, reference_map, reference_counts)
    pair = _seal_pair(root, RECEIPT_NAME, value)
    del pair
    return validate_receipt(root, expected_plan=plan)


def _campaign_aggregation_contract() -> dict[str, Any]:
    """Return the strict aggregation contract for this capture command."""
    return {
        "schema": CAMPAIGN_AGGREGATION_SCHEMA,
        "schema_version": 1,
        "tool": {
            "path": "scripts/capture_registration_plugin_dependency_closure.py",
            "sha256": AUDIT.sha256_file(ROOT / "scripts/capture_registration_plugin_dependency_closure.py"),
        },
        "campaign_schema": {
            "path": CAMPAIGN_SCHEMA_PATH,
            "sha256": AUDIT.sha256_file(ROOT / CAMPAIGN_SCHEMA_PATH),
        },
        "capture_executor_schema": {
            "path": "configs/slam_benchmark_profiles/"
            "registration_plugin_dependency_capture_executor_v1.schema.json",
            "sha256": AUDIT.sha256_file(
                ROOT / "configs/slam_benchmark_profiles/"
                "registration_plugin_dependency_capture_executor_v1.schema.json"),
        },
        "partial_receipt_schema": {
            "path": CAMPAIGN_PARTIAL_SCHEMA_PATH,
            "sha256": AUDIT.sha256_file(ROOT / CAMPAIGN_PARTIAL_SCHEMA_PATH),
        },
        "prepare_mount_policy": _immutable_copy(
            CLOSURE.ROSDEP_PREPARE_MOUNT_POLICY),
        "row_state_policy": "fixed-four-state-vector",
        "rows_policy": "completed-pass-only",
        "partial_rows_policy": "sealed-started-child-only",
        "normal_rows_exclude_partial": True,
        "partial_row_identity": "campaign-row-campaign-set-profile-child-receipt",
        "status": "REVIEW_REQUIRED",
        "benchmark_eligible": False,
        "promotion": "FORBIDDEN_UNTIL_SIGNED_RECEIPT",
    }


def _campaign_child_name(index: int, distro: str, leg: str) -> str:
    if type(index) is not int or index < 0 or index >= len(ROWS):
        raise CaptureError("CAMPAIGN_ROW_INVALID", "row index")
    expected = ROWS[index]
    if (distro, leg) != expected:
        raise CaptureError("CAMPAIGN_ROW_INVALID", "row identity")
    return "row-{}-{}-{}".format(index + 1, distro, leg)


def _campaign_row_identity(
        index: int, distro: str, leg: str, root: Path,
        campaign: Mapping[str, Any], profile: Mapping[str, Any],
        image_digest: str | None = None) -> dict[str, Any]:
    child = root / _campaign_child_name(index, distro, leg)
    container = _container_name(child, distro, leg)
    return {
        "campaign_id": campaign["campaign_id"],
        "campaign_set_identity_sha256": campaign["identity_sha256"],
        "profile": _immutable_copy(profile),
        "distro": distro,
        "dependency_leg": leg,
        "image_digest": image_digest or "sha256:" + "0" * 64,
        "container_name": container,
    }


def _campaign_cardinality(receipt: Mapping[str, Any] | None) -> dict[str, Any]:
    """Project only independently observed child cardinalities."""
    if not isinstance(receipt, Mapping):
        return {
            "started": True, "phase_count": 0,
            "expected_phase_count": len(PHASES),
            "release_evidence_count": 0, "package_evidence_count": 0,
            "signature_reference_count": 0,
        }
    return {
        "started": True,
        "phase_count": len(receipt.get("phases", []))
        if isinstance(receipt.get("phases"), list) else 0,
        "expected_phase_count": len(PHASES),
        "release_evidence_count": len(receipt.get("release_evidence", []))
        if isinstance(receipt.get("release_evidence"), list) else 0,
        "package_evidence_count": len(receipt.get("package_evidence", []))
        if isinstance(receipt.get("package_evidence"), list) else 0,
        "signature_reference_count": len(receipt.get("signature_references", []))
        if isinstance(receipt.get("signature_references"), list) else 0,
    }


def _campaign_validate_cardinality(value: Any, label: str) -> dict[str, Any]:
    """Validate the non-null cardinality projection used by a child row."""
    required = {
        "started", "phase_count", "expected_phase_count",
        "release_evidence_count", "package_evidence_count",
        "signature_reference_count",
    }
    if not isinstance(value, Mapping) or set(value) != required or \
            value.get("started") is not True or \
            value.get("expected_phase_count") != len(PHASES):
        raise CaptureError("CAMPAIGN_CARDINALITY_INVALID", label)
    for key in ("phase_count", "release_evidence_count",
                "package_evidence_count", "signature_reference_count"):
        if (type(value.get(key)) is not int or value[key] < 0 or
                value[key] > MAX_CAPTURE_FILES):
            raise CaptureError("CAMPAIGN_CARDINALITY_INVALID", label)
    return dict(value)


def _campaign_read_child_receipt(
        child: Path, index: int, distro: str, leg: str,
        campaign: Mapping[str, Any], profile: Mapping[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    """Reopen one child executor receipt and bind its row identity."""
    path = child / RECEIPT_NAME
    try:
        value, _ = _read_bounded(path, "campaign child receipt", MAX_RECEIPT_BYTES,
                                 allow_empty=False)
        descriptor = _descriptor(path, "campaign child receipt", MAX_RECEIPT_BYTES,
                                  sidecar=True)
        child_value = json.loads(value.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError, CaptureError) as error:
        if isinstance(error, CaptureError):
            raise
        raise CaptureError("CAMPAIGN_CHILD_RECEIPT_INVALID", str(path)) from error
    child_required = {
        "schema", "schema_version", "status", "outcome",
        "benchmark_eligible", "distro", "dependency_leg", "campaign_id",
        "campaign_set", "profile", "root", "container", "canonical_sha256",
        "phases", "release_evidence", "package_evidence",
        "signature_references",
    }
    if (not isinstance(child_value, Mapping) or
            not child_required.issubset(set(child_value)) or
            child_value.get("schema") != SCHEMA or
            child_value.get("schema_version") != SCHEMA_VERSION or
            child_value.get("benchmark_eligible") is not False or
            child_value.get("distro") != distro or
            child_value.get("dependency_leg") != leg or
            child_value.get("campaign_id") != campaign["campaign_id"] or
            child_value.get("campaign_set") != campaign or
            child_value.get("profile") != profile or
            child_value.get("canonical_sha256") != canonical_hash(child_value)):
        raise CaptureError("CAMPAIGN_CHILD_IDENTITY_INVALID", str(path))
    child_root = child_value.get("root")
    child_container = child_value.get("container")
    if (not isinstance(child_root, Mapping) or
            not isinstance(child_container, Mapping) or
            child_root.get("path") != str(child) or
            child_container.get("name") != _container_name(child, distro, leg)):
        raise CaptureError("CAMPAIGN_CHILD_IDENTITY_INVALID", str(path))
    if child_value.get("status") not in {
            "REVIEW_REQUIRED", "PARTIAL_FAILURE_REVIEW_REQUIRED"} or \
            child_value.get("outcome") not in {
                "PASS_REVIEW_REQUIRED", "PARTIAL_FAILURE"}:
        raise CaptureError("CAMPAIGN_CHILD_STATUS_INVALID", str(path))
    if ((child_value["status"] == "REVIEW_REQUIRED") !=
            (child_value["outcome"] == "PASS_REVIEW_REQUIRED")):
        raise CaptureError("CAMPAIGN_CHILD_STATUS_INVALID", str(path))
    _campaign_validate_cardinality(
        _campaign_cardinality(child_value), "child cardinality")
    descriptor["canonical_sha256"] = child_value["canonical_sha256"]
    return dict(child_value), descriptor


def _campaign_descriptor_projection(
        root: Path, child: Path, value: Mapping[str, Any],
        descriptor: Mapping[str, Any], status: str,
        campaign: Mapping[str, Any], profile: Mapping[str, Any],
        image_digest: str, failure: Mapping[str, Any] | None = None
) -> dict[str, Any]:
    relative = _safe_rel(child.relative_to(root).as_posix() + "/" + RECEIPT_NAME,
                         "campaign child receipt path")
    sidecar_relative = relative + ".sha256"
    sidecar = descriptor.get("sidecar")
    if not isinstance(sidecar, Mapping):
        raise CaptureError("CAMPAIGN_CHILD_RECEIPT_INVALID", relative)
    try:
        index = ROWS.index((value["distro"], value["dependency_leg"]))
    except (KeyError, ValueError) as error:
        raise CaptureError("CAMPAIGN_CHILD_IDENTITY_INVALID", relative) from error
    _digest(image_digest, "campaign image digest")
    projection = {
        "index": index,
        "distro": value["distro"], "dependency_leg": value["dependency_leg"],
        "child_root": str(child),
        "container_name": _container_name(
            child, value["distro"], value["dependency_leg"]),
        "status": status,
        "path_relative": relative,
        "file_bytes": descriptor["bytes"],
        "file_sha256": descriptor["sha256"],
        "canonical_sha256": value["canonical_sha256"],
        "sidecar_path_relative": _safe_rel(
            sidecar_relative, "campaign child sidecar path"),
        "sidecar_sha256": sidecar["sha256"],
        "receipt_status": value["status"],
        "receipt_outcome": value["outcome"],
        "identity": _campaign_row_identity(
            index, value["distro"], value["dependency_leg"], root, campaign,
            profile, image_digest),
        "cardinality": _campaign_validate_cardinality(
            _campaign_cardinality(value), "campaign child cardinality"),
    }
    if failure is not None:
        projection["failure"] = dict(failure)
    return projection


def _campaign_partial_receipt_value(
        root: Path, child: Path, index: int, distro: str, leg: str,
        campaign: Mapping[str, Any], profile: Mapping[str, Any],
        image_digest: str, failure: Mapping[str, Any],
        child_receipt: Mapping[str, Any] | None,
        child_descriptor: Mapping[str, Any] | None,
        started: int, ended: int) -> dict[str, Any]:
    underlying = None
    if child_receipt is not None and child_descriptor is not None:
        sidecar = child_descriptor.get("sidecar")
        if not isinstance(sidecar, Mapping):
            raise CaptureError("CAMPAIGN_CHILD_RECEIPT_INVALID", "sidecar")
        underlying = {
            "path_relative": RECEIPT_NAME,
            "file_bytes": child_descriptor["bytes"],
            "file_sha256": child_descriptor["sha256"],
            "canonical_sha256": child_receipt["canonical_sha256"],
            "sidecar_path_relative": RECEIPT_NAME + ".sha256",
            "sidecar_sha256": sidecar["sha256"],
        }
    value = {
        "schema": CAMPAIGN_PARTIAL_SCHEMA,
        "schema_version": 1,
        "status": "FAIL_CLOSED",
        "benchmark_eligible": False,
        "execution_started": True,
        "campaign_id": campaign["campaign_id"],
        "campaign_set": _immutable_copy(campaign),
        "row_count": len(ROWS),
        "row_order": [list(item) for item in ROWS],
        "row": {
            "index": index, "distro": distro, "dependency_leg": leg,
            "child_root": str(child),
            "container_name": _container_name(child, distro, leg),
        },
        "profile": _immutable_copy(profile),
        "identity": _campaign_row_identity(
            index, distro, leg, root, campaign, profile, image_digest),
        "cardinality": _campaign_cardinality(child_receipt),
        "child_receipt": underlying,
        "failure": dict(failure),
        "started_at_unix": started,
        "ended_at_unix": ended,
    }
    value["canonical_sha256"] = canonical_hash(value)
    return value


def _campaign_record_partial(
        root: Path, index: int, distro: str, leg: str,
        campaign: Mapping[str, Any], profile: Mapping[str, Any],
        image_digest: str, failure: Mapping[str, Any],
        started: int, ended: int) -> dict[str, Any]:
    """Seal one child partial receipt without overwriting child evidence."""
    child = root / _campaign_child_name(index, distro, leg)
    if child.is_symlink() or (child.exists() and not child.is_dir()):
        raise CaptureError("CAMPAIGN_CHILD_ROOT_INVALID", str(child))
    if not child.exists():
        _fresh_directory(child, "campaign partial child root")
    child_value = None
    child_descriptor = None
    child_path = child / RECEIPT_NAME
    if child_path.exists() or child_path.is_symlink():
        child_value, child_descriptor = _campaign_read_child_receipt(
            child, index, distro, leg, campaign, profile)
    partial_path = child / CAMPAIGN_PARTIAL_RECEIPT_NAME
    if partial_path.exists() or partial_path.is_symlink() or \
            Path(str(partial_path) + ".sha256").exists():
        raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_COLLISION", str(partial_path))
    value = _campaign_partial_receipt_value(
        root, child, index, distro, leg, campaign, profile, image_digest,
        failure, child_value, child_descriptor, started, ended)
    _seal_pair(child, CAMPAIGN_PARTIAL_RECEIPT_NAME, value)
    partial_value, descriptor = _read_sealed_campaign_partial(
        partial_path, root, index, distro, leg, campaign, profile)
    sidecar = descriptor.get("sidecar")
    if not isinstance(sidecar, Mapping):
        raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_INVALID", str(partial_path))
    projection = {
        "index": index, "distro": distro, "dependency_leg": leg,
        "child_root": str(child),
        "container_name": _container_name(child, distro, leg),
        "status": "FAIL_CLOSED",
        "path_relative": _safe_rel(
            partial_path.relative_to(root).as_posix(),
            "campaign partial receipt path"),
        "file_bytes": descriptor["bytes"],
        "file_sha256": descriptor["sha256"],
        "canonical_sha256": partial_value["canonical_sha256"],
        "sidecar_path_relative": _safe_rel(
            partial_path.relative_to(root).as_posix() + ".sha256",
            "campaign partial sidecar path"),
        "sidecar_sha256": sidecar["sha256"],
        "receipt_status": partial_value["status"],
        "receipt_outcome": "PARTIAL_FAILURE",
        "identity": partial_value["identity"],
        "cardinality": partial_value["cardinality"],
        "failure": dict(failure),
    }
    return projection


def _read_sealed_campaign_partial(
        path: Path, root: Path, index: int, distro: str, leg: str,
        campaign: Mapping[str, Any], profile: Mapping[str, Any]
) -> tuple[dict[str, Any], dict[str, Any]]:
    data, _ = _read_bounded(path, "campaign partial receipt", MAX_RECEIPT_BYTES,
                            allow_empty=False)
    descriptor = _descriptor(path, "campaign partial receipt", MAX_RECEIPT_BYTES,
                              sidecar=True)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_INVALID", str(path)) from error
    row_value = value.get("row") if isinstance(value, Mapping) else None
    if (not isinstance(value, Mapping) or
            value.get("schema") != CAMPAIGN_PARTIAL_SCHEMA or
            value.get("schema_version") != 1 or
            value.get("status") != "FAIL_CLOSED" or
            value.get("benchmark_eligible") is not False or
            value.get("execution_started") is not True or
            value.get("campaign_id") != campaign["campaign_id"] or
            value.get("campaign_set") != campaign or
            value.get("profile") != profile or
            value.get("canonical_sha256") != canonical_hash(value)):
        raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_IDENTITY_INVALID", str(path))
    if (not isinstance(row_value, Mapping) or
            set(row_value) != {
                "index", "distro", "dependency_leg", "child_root",
                "container_name"} or
            row_value.get("index") != index or
            row_value.get("distro") != distro or
            row_value.get("dependency_leg") != leg or
            row_value.get("child_root") != str(path.parent) or
            row_value.get("container_name") !=
            _container_name(path.parent, distro, leg)):
        raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_IDENTITY_INVALID", str(path))
    identity = value.get("identity")
    if not isinstance(identity, Mapping):
        raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_IDENTITY_INVALID", str(path))
    _digest(identity.get("image_digest"), "campaign partial image digest")
    expected_identity = _campaign_row_identity(
        index, distro, leg, root, campaign, profile,
        identity["image_digest"])
    if dict(identity) != expected_identity:
        raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_IDENTITY_INVALID", str(path))
    _campaign_validate_cardinality(value.get("cardinality"), str(path))
    child_receipt = value.get("child_receipt")
    if child_receipt is not None:
        if not isinstance(child_receipt, Mapping) or set(child_receipt) != {
                "path_relative", "file_bytes", "file_sha256",
                "canonical_sha256", "sidecar_path_relative", "sidecar_sha256"}:
            raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_CHILD_INVALID", str(path))
        if (child_receipt["path_relative"] != RECEIPT_NAME or
                child_receipt["sidecar_path_relative"] != RECEIPT_NAME + ".sha256" or
                type(child_receipt["file_bytes"]) is not int or
                child_receipt["file_bytes"] <= 0):
            raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_CHILD_INVALID", str(path))
        for field in ("file_sha256", "canonical_sha256", "sidecar_sha256"):
            _sha(child_receipt[field], "campaign partial child " + field)
    if value["ended_at_unix"] < value["started_at_unix"]:
        raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_TIME_INVALID", str(path))
    failure = value.get("failure")
    if (not isinstance(failure, Mapping) or
            set(failure) != {"kind", "phase", "row", "message"} or
            failure.get("row") != index):
        raise CaptureError("CAMPAIGN_PARTIAL_RECEIPT_FAILURE_INVALID", str(path))
    return dict(value), descriptor


def _campaign_row_state(index: int, distro: str, leg: str, root: Path,
                        status: str, started: bool) -> dict[str, Any]:
    child = root / _campaign_child_name(index, distro, leg)
    return {
        "index": index, "distro": distro, "dependency_leg": leg,
        "child_root": str(child),
        "container_name": _container_name(child, distro, leg),
        "status": status, "started": started,
    }


def _campaign_validate_projection(
        value: Mapping[str, Any], root: Path, profile: Mapping[str, Any],
        release: Mapping[str, Any], profile_file: Path) -> dict[str, Any]:
    """Validate campaign shape, child seals, and disjoint row projections."""
    required = {
        "schema", "schema_version", "status", "outcome", "benchmark_eligible",
        "claim_eligible", "active_profile_switch", "promotion_allowed",
        "execution_started", "campaign_id", "campaign_set", "profile", "root",
        "aggregation", "rows", "row_states", "partial_rows", "partial_row_count",
        "summary", "error", "started_at_unix", "ended_at_unix", "promotion",
        "canonical_sha256",
    }
    if set(value) != required or value.get("schema") != CAMPAIGN_SCHEMA or \
            value.get("schema_version") != 1 or \
            value.get("benchmark_eligible") is not False or \
            value.get("claim_eligible") is not False or \
            value.get("active_profile_switch") is not False or \
            value.get("promotion_allowed") is not False or \
            value.get("execution_started") is not True or \
            value.get("canonical_sha256") != canonical_hash(value):
        raise CaptureError("CAMPAIGN_RECEIPT_IDENTITY_INVALID", "fields/status/hash")
    if value["campaign_set"] != release["campaign_set"] or \
            value["campaign_id"] != release["campaign_set"]["campaign_id"]:
        raise CaptureError("CAMPAIGN_SET_BINDING_INVALID", "campaign set")
    profile_binding = value.get("profile")
    if profile_binding != {
            "path": str(profile_file),
            "sha256": AUDIT.sha256_file(profile_file)}:
        raise CaptureError("CAMPAIGN_PROFILE_BINDING_INVALID", "profile")
    root_value = value.get("root")
    if (not isinstance(root_value, Mapping) or
            set(root_value) != {"path", "device", "inode", "mode", "nlink"} or
            root_value.get("path") != str(root) or
            root_value.get("mode") != 0o700 or
            type(root_value.get("device")) is not int or
            type(root_value.get("inode")) is not int or
            type(root_value.get("nlink")) is not int or
            root_value.get("nlink") < 2):
        raise CaptureError("CAMPAIGN_ROOT_BINDING_INVALID", "root")
    try:
        current_root = root.lstat()
    except OSError as error:
        raise CaptureError("CAMPAIGN_ROOT_INVALID", str(root)) from error
    if (stat.S_ISLNK(current_root.st_mode) or
            not stat.S_ISDIR(current_root.st_mode) or
            current_root.st_dev != root_value["device"] or
            current_root.st_ino != root_value["inode"] or
            stat.S_IMODE(current_root.st_mode) != 0o700):
        raise CaptureError("CAMPAIGN_ROOT_CHANGED", str(root))
    if value.get("aggregation") != _campaign_aggregation_contract():
        raise CaptureError("CAMPAIGN_AGGREGATION_BINDING_INVALID", "aggregation")
    rows = value.get("rows")
    states = value.get("row_states")
    partial_rows = value.get("partial_rows")
    if (not isinstance(rows, list) or not isinstance(states, list) or
            len(states) != len(ROWS) or not isinstance(partial_rows, list) or
            type(value.get("partial_row_count")) is not int or
            value["partial_row_count"] != len(partial_rows)):
        raise CaptureError("CAMPAIGN_CARDINALITY_INVALID", "rows/state/partial")
    expected_states = {}
    for index, (distro, leg) in enumerate(ROWS):
        state = states[index]
        if (not isinstance(state, Mapping) or set(state) != {
                "index", "distro", "dependency_leg", "child_root",
                "container_name", "status", "started"} or
                state.get("index") != index or state.get("distro") != distro or
                state.get("dependency_leg") != leg or
                state.get("child_root") != str(root / _campaign_child_name(
                    index, distro, leg)) or
                state.get("container_name") != _container_name(
                    root / _campaign_child_name(index, distro, leg), distro, leg) or
                type(state.get("started")) is not bool or
                (state.get("status") == "NOT_STARTED") !=
                (state.get("started") is False) or
                state.get("status") not in {
                    "NOT_STARTED", "PASS_REVIEW_REQUIRED", "FAIL_CLOSED"}):
            raise CaptureError("CAMPAIGN_ROW_STATE_INVALID", str(index))
        expected_states[index] = state
    def check_row_projection(item: Any, expected_status: str,
                             label: str) -> int:
        if not isinstance(item, Mapping) or set(item) != {
                "index", "distro", "dependency_leg", "child_root",
                "container_name", "status", "path_relative", "file_bytes",
                "file_sha256", "canonical_sha256", "sidecar_path_relative",
                "sidecar_sha256", "receipt_status", "receipt_outcome",
                "identity", "cardinality"} | ({"failure"} if expected_status ==
                                                   "FAIL_CLOSED" else set()):
            raise CaptureError("CAMPAIGN_PROJECTION_INVALID", label)
        index = item.get("index")
        if type(index) is not int or index not in expected_states:
            raise CaptureError("CAMPAIGN_PROJECTION_INVALID", label)
        expected = expected_states[index]
        distro, leg = ROWS[index]
        child = root / _campaign_child_name(index, distro, leg)
        identity = item.get("identity")
        expected_receipt_name = (
            RECEIPT_NAME if expected_status == "PASS_REVIEW_REQUIRED" else
            CAMPAIGN_PARTIAL_RECEIPT_NAME
        )
        if (item.get("distro") != distro or item.get("dependency_leg") != leg or
                item.get("child_root") != str(child) or
                item.get("container_name") != expected["container_name"] or
                item.get("status") != expected_status or
                expected["status"] != expected_status or
                expected["started"] is not True or
                not isinstance(item.get("path_relative"), str) or
                not isinstance(item.get("sidecar_path_relative"), str) or
                item.get("path_relative") != _safe_rel(
                    child.relative_to(root).as_posix() + "/" +
                    expected_receipt_name,
                    label + " expected path") or
                item.get("path_relative") != _safe_rel(
                    item["path_relative"], label + " path") or
                item.get("sidecar_path_relative") !=
                item["path_relative"] + ".sha256" or
                type(item.get("file_bytes")) is not int or item["file_bytes"] <= 0 or
                SHA_RE.fullmatch(str(item.get("file_sha256", ""))) is None or
                SHA_RE.fullmatch(str(item.get("canonical_sha256", ""))) is None or
                SHA_RE.fullmatch(str(item.get("sidecar_sha256", ""))) is None or
                not isinstance(identity, Mapping) or
                set(identity) != {"campaign_id", "campaign_set_identity_sha256",
                                  "profile", "distro", "dependency_leg",
                                  "image_digest", "container_name"} or
                identity.get("campaign_id") != value["campaign_id"] or
                identity.get("campaign_set_identity_sha256") !=
                release["campaign_set"]["identity_sha256"] or
                identity.get("profile") != value["profile"] or
                identity.get("distro") != distro or
                identity.get("dependency_leg") != leg or
                identity.get("container_name") != expected["container_name"] or
                not isinstance(item.get("cardinality"), Mapping) or
                _campaign_validate_cardinality(item["cardinality"], label) !=
                item["cardinality"]):
            raise CaptureError("CAMPAIGN_PROJECTION_INVALID", label)
        _digest(identity["image_digest"], label + " image digest")
        if identity != _campaign_row_identity(
                index, distro, leg, root, release["campaign_set"],
                value["profile"], identity["image_digest"]):
            raise CaptureError("CAMPAIGN_PROJECTION_IDENTITY_INVALID", label)
        candidate = root / item["path_relative"]
        if candidate.is_symlink() or candidate.parent != child:
            raise CaptureError("CAMPAIGN_PROJECTION_PATH_INVALID", label)
        if expected_status == "FAIL_CLOSED":
            failure = item.get("failure")
            if (not isinstance(failure, Mapping) or
                    set(failure) != {"kind", "phase", "row", "message"} or
                    failure.get("row") != index):
                raise CaptureError("CAMPAIGN_PROJECTION_FAILURE_INVALID", label)
            partial_value, descriptor = _read_sealed_campaign_partial(
                candidate, root, index, distro, leg, release["campaign_set"],
                value["profile"])
            if (descriptor["bytes"] != item["file_bytes"] or
                    descriptor["sha256"] != item["file_sha256"] or
                    descriptor["sidecar"]["sha256"] != item["sidecar_sha256"] or
                    partial_value["canonical_sha256"] != item["canonical_sha256"] or
                    partial_value["failure"] != failure or
                    partial_value["identity"] != item["identity"] or
                    partial_value["cardinality"] != item["cardinality"]):
                raise CaptureError("CAMPAIGN_PROJECTION_HASH_INVALID", label)
        else:
            child_value, descriptor = _campaign_read_child_receipt(
                child, index, distro, leg, release["campaign_set"], value["profile"])
            if child_value["status"] != "REVIEW_REQUIRED" or \
                    child_value["outcome"] != "PASS_REVIEW_REQUIRED":
                raise CaptureError("CAMPAIGN_PROJECTION_STATUS_INVALID", label)
            child_image = child_value.get("image")
            if (not isinstance(child_image, Mapping) or
                    child_image.get("digest") != identity["image_digest"]):
                raise CaptureError("CAMPAIGN_PROJECTION_IDENTITY_INVALID", label)
            if (descriptor["bytes"] != item["file_bytes"] or
                    descriptor["sha256"] != item["file_sha256"] or
                    descriptor["sidecar"]["sha256"] != item["sidecar_sha256"] or
                    child_value["canonical_sha256"] != item["canonical_sha256"] or
                    _campaign_cardinality(child_value) != item["cardinality"]):
                raise CaptureError("CAMPAIGN_PROJECTION_HASH_INVALID", label)
        return index
    row_indexes = []
    for item in rows:
        row_indexes.append(check_row_projection(
            item, "PASS_REVIEW_REQUIRED", "rows"))
    if row_indexes != sorted(row_indexes) or len(set(row_indexes)) != len(row_indexes):
        raise CaptureError("CAMPAIGN_ROWS_ORDER_INVALID", "rows")
    partial_indexes = []
    for item in partial_rows:
        partial_indexes.append(check_row_projection(
            item, "FAIL_CLOSED", "partial_rows"))
    if (partial_indexes != sorted(partial_indexes) or
            len(set(partial_indexes)) != len(partial_indexes) or
            set(row_indexes) & set(partial_indexes)):
        raise CaptureError("CAMPAIGN_PARTIAL_ROWS_ORDER_INVALID", "overlap/order")
    if len(partial_indexes) > 1:
        raise CaptureError("CAMPAIGN_PARTIAL_ROWS_CARDINALITY_INVALID", "one-stop policy")
    for index, state in expected_states.items():
        in_rows = index in row_indexes
        in_partial = index in partial_indexes
        if ((state["status"] == "PASS_REVIEW_REQUIRED") != in_rows or
                (state["status"] == "FAIL_CLOSED") != in_partial or
                state["status"] == "NOT_STARTED" and (in_rows or in_partial) or
                state["status"] != "NOT_STARTED" and state["started"] is not True):
            raise CaptureError("CAMPAIGN_ROW_MEMBERSHIP_INVALID", str(index))
    if value["status"] == "REVIEW_REQUIRED":
        if (value["outcome"] != "PASS_REVIEW_REQUIRED" or value["error"] is not None or
                partial_indexes or len(row_indexes) != len(ROWS) or
                any(state["status"] != "PASS_REVIEW_REQUIRED"
                    for state in expected_states.values())):
            raise CaptureError("CAMPAIGN_STATUS_INVALID", "success projection")
    elif value["status"] == "PARTIAL_FAILURE_REVIEW_REQUIRED":
        if (value["outcome"] != "PARTIAL_FAILURE" or
                not isinstance(value["error"], Mapping) or
                len(partial_indexes) != 1 or
                value["error"] != partial_rows[0]["failure"]):
            raise CaptureError("CAMPAIGN_STATUS_INVALID", "partial projection")
    else:
        raise CaptureError("CAMPAIGN_STATUS_INVALID", str(value.get("status")))
    if value["summary"] != {
            "status": "NOT_RUN", "reason": "capture-only-no-promotion"}:
        raise CaptureError("CAMPAIGN_SUMMARY_INVALID", "summary")
    if (type(value["started_at_unix"]) is not int or
            type(value["ended_at_unix"]) is not int or
            value["started_at_unix"] < 0 or
            value["ended_at_unix"] < value["started_at_unix"]):
        raise CaptureError("CAMPAIGN_TIME_INVALID", "timestamps")
    # A partial receipt under an expected child must be referenced exactly once;
    # an unexpected row directory or receipt cannot be hidden by rows=[].
    expected_children = {
        _campaign_child_name(index, distro, leg): index
        for index, (distro, leg) in enumerate(ROWS)}
    try:
        children = list(root.iterdir())
    except OSError as error:
        raise CaptureError("CAMPAIGN_ROOT_INVALID", str(root)) from error
    allowed_root_entries = set(expected_children) | {
        CAMPAIGN_RECEIPT_NAME, CAMPAIGN_RECEIPT_NAME + ".sha256",
    }
    for child in children:
        if child.name not in allowed_root_entries:
            raise CaptureError("CAMPAIGN_ROOT_EXTRA_ENTRY", str(child))
        if child.name not in expected_children:
            continue
        index = expected_children[child.name]
        if child.is_symlink() or not child.is_dir():
            raise CaptureError("CAMPAIGN_CHILD_ROOT_INVALID", str(child))
        partial_path = child / CAMPAIGN_PARTIAL_RECEIPT_NAME
        if partial_path.exists() or partial_path.is_symlink():
            if index not in partial_indexes:
                raise CaptureError("CAMPAIGN_PARTIAL_ROW_UNREFERENCED", str(partial_path))
        if expected_states[index]["status"] == "NOT_STARTED":
            try:
                entries = list(child.iterdir())
            except OSError as error:
                raise CaptureError("CAMPAIGN_CHILD_ROOT_INVALID", str(child)) from error
            if entries:
                raise CaptureError("CAMPAIGN_STARTED_CHILD_UNREFERENCED", str(child))
    return {
        "status": value["status"], "outcome": value["outcome"],
        "benchmark_eligible": False, "claim_eligible": False,
        "active_profile_switch": False, "promotion_allowed": False,
        "rows": rows, "row_states": states, "partial_rows": partial_rows,
        "partial_row_count": value["partial_row_count"],
        "campaign_sha256": sha256_bytes(
            _read_bounded(root / CAMPAIGN_RECEIPT_NAME, "campaign receipt",
                          MAX_RECEIPT_BYTES, allow_empty=False)[0]),
        "canonical_sha256": value["canonical_sha256"],
    }


def validate_campaign_receipt(
        root: Path, *, expected_profile_path: Path | None = None) -> dict[str, Any]:
    """Reopen a capture campaign and enforce disjoint partial rows."""
    root = _safe_abs(str(root), "campaign receipt root")
    if root.is_symlink() or not root.is_dir():
        raise CaptureError("CAMPAIGN_ROOT_INVALID", str(root))
    receipt_path = root / CAMPAIGN_RECEIPT_NAME
    data, _ = _read_bounded(receipt_path, "campaign receipt", MAX_RECEIPT_BYTES,
                            allow_empty=False)
    _descriptor(receipt_path, "campaign receipt", MAX_RECEIPT_BYTES, sidecar=True)
    try:
        value = json.loads(data.decode("utf-8"))
    except (UnicodeError, json.JSONDecodeError) as error:
        raise CaptureError("CAMPAIGN_RECEIPT_INVALID", str(receipt_path)) from error
    if not isinstance(value, Mapping):
        raise CaptureError("CAMPAIGN_RECEIPT_INVALID", "object required")
    profile_value = value.get("profile")
    if not isinstance(profile_value, Mapping):
        raise CaptureError("CAMPAIGN_PROFILE_BINDING_INVALID", "profile")
    profile_path_value = profile_value.get("path")
    if expected_profile_path is not None and profile_path_value != str(
            _safe_abs(str(expected_profile_path), "expected profile path")):
        raise CaptureError("CAMPAIGN_PROFILE_BINDING_INVALID", "expected profile")
    try:
        profile, profile_file = AUDIT._load_profile(Path(profile_path_value))
        release = AUDIT._validate_release_matrix_profile(profile)
    except (TypeError, OSError, KeyError, AttributeError) as error:
        raise CaptureError("CAMPAIGN_PROFILE_INVALID", str(profile_path_value)) from error
    return _campaign_validate_projection(
        value, root, profile, release, profile_file)


validate_campaign = validate_campaign_receipt


def capture_campaign(repo_root: Path, profile_path: Path, campaign_root: Path,
                     *, runner: Callable[[list[str], int], Any] | None = None,
                     clock: Callable[[], int] | None = None) -> dict[str, Any]:
    """Execute the exact four rows, sealing started failures separately."""
    profile, profile_file = AUDIT._load_profile(Path(profile_path))
    release = AUDIT._validate_release_matrix_profile(profile)
    if tuple((item["distro"], item["dependency_leg"])
             for item in release["campaign_set"]["rows"]) != ROWS:
        raise CaptureError("CAMPAIGN_SET_INVALID", "exact four row order required")
    if release.get("dependency_capture_campaign_aggregation") != \
            _campaign_aggregation_contract():
        raise CaptureError("CAMPAIGN_AGGREGATION_BINDING_INVALID", "profile contract")
    root_identity = _fresh_root(Path(campaign_root), "campaign root")
    root = Path(campaign_root)
    started = int((clock or (lambda: int(time.time())))())
    profile_binding = {
        "path": str(profile_file), "sha256": AUDIT.sha256_file(profile_file)}
    rows: list[dict[str, Any]] = []
    row_states = [_campaign_row_state(index, distro, leg, root, "NOT_STARTED", False)
                  for index, (distro, leg) in enumerate(ROWS)]
    partial_rows: list[dict[str, Any]] = []
    error = None
    for index, (distro, leg) in enumerate(ROWS):
        child = root / _campaign_child_name(index, distro, leg)
        row_states[index] = _campaign_row_state(
            index, distro, leg, root, "FAIL_CLOSED", True)
        try:
            capture_dependency_closure(
                repo_root, profile_file, distro, leg, child, runner=runner, clock=clock)
            child_value, child_descriptor = _campaign_read_child_receipt(
                child, index, distro, leg, release["campaign_set"], profile_binding)
            image_digest = child_value.get("image", {}).get("digest")
            if child_value["status"] == "REVIEW_REQUIRED" and \
                    child_value["outcome"] == "PASS_REVIEW_REQUIRED":
                row_states[index] = _campaign_row_state(
                    index, distro, leg, root, "PASS_REVIEW_REQUIRED", True)
                rows.append(_campaign_descriptor_projection(
                    root, child, child_value, child_descriptor,
                    "PASS_REVIEW_REQUIRED", release["campaign_set"],
                    profile_binding, image_digest or "sha256:" + "0" * 64))
                continue
            failure = {
                "kind": "ROW_FAILED", "phase": "capture", "row": index,
                "message": "child returned {}".format(child_value["status"]),
            }
            partial_rows.append(_campaign_record_partial(
                root, index, distro, leg, release["campaign_set"],
                profile_binding, image_digest or "sha256:" + "0" * 64,
                failure, started, int((clock or (lambda: int(time.time())))())))
            error = failure
            break
        except Exception as exc:
            failure = {
                "kind": getattr(exc, "kind", "ROW_FAILED"),
                "phase": "capture", "row": index, "message": str(exc),
            }
            try:
                partial_rows.append(_campaign_record_partial(
                    root, index, distro, leg, release["campaign_set"],
                    profile_binding, "sha256:" + "0" * 64,
                    failure, started, int((clock or (lambda: int(time.time())))())))
            except Exception as partial_error:
                failure = {
                    "kind": "PARTIAL_RECEIPT_SEAL_FAILED", "phase": "capture",
                    "row": index,
                    "message": "{}; partial receipt: {}".format(exc, partial_error),
                }
            error = failure
            break
    for index in range(len(ROWS)):
        if row_states[index]["status"] == "FAIL_CLOSED" and \
                index not in {item["index"] for item in partial_rows}:
            # A started child whose partial receipt could not be sealed is still
            # represented in the state vector; the outer receipt remains fail
            # closed and validation rejects the missing partial projection.
            row_states[index] = _campaign_row_state(
                index, ROWS[index][0], ROWS[index][1], root, "FAIL_CLOSED", True)
    ended = int((clock or (lambda: int(time.time())))())
    status = "REVIEW_REQUIRED" if error is None else "PARTIAL_FAILURE_REVIEW_REQUIRED"
    outcome = "PASS_REVIEW_REQUIRED" if error is None else "PARTIAL_FAILURE"
    value = {
        "schema": CAMPAIGN_SCHEMA, "schema_version": 1,
        "status": status, "outcome": outcome, "benchmark_eligible": False,
        "claim_eligible": False, "active_profile_switch": False,
        "promotion_allowed": False, "execution_started": True,
        "campaign_id": release["campaign_set"]["campaign_id"],
        "campaign_set": release["campaign_set"], "profile": profile_binding,
        "root": root_identity,
        "aggregation": _campaign_aggregation_contract(),
        "rows": sorted(rows, key=lambda item: item["index"]),
        "row_states": row_states,
        "partial_rows": sorted(partial_rows, key=lambda item: item["index"]),
        "partial_row_count": len(partial_rows),
        "summary": {"status": "NOT_RUN", "reason": "capture-only-no-promotion"},
        "error": error, "started_at_unix": started, "ended_at_unix": ended,
        "promotion": "FORBIDDEN_UNTIL_SIGNED_RECEIPT",
    }
    value["canonical_sha256"] = canonical_hash(value)
    _seal_pair(root, CAMPAIGN_RECEIPT_NAME, value)
    return validate_campaign_receipt(root, expected_profile_path=profile_file)


def _parser() -> argparse.ArgumentParser:
    parser = argparse.ArgumentParser(description=__doc__)
    sub = parser.add_subparsers(dest="command", required=True)
    plan = sub.add_parser("plan")
    plan.add_argument("--repo-root", default=str(ROOT))
    plan.add_argument("--profile", default=str(PROFILE_PATH))
    plan.add_argument("--distro", choices=("humble", "jazzy"), required=True)
    plan.add_argument("--dependency-leg", choices=("absent", "present"), required=True)
    plan.add_argument("--output-root", required=True)
    capture = sub.add_parser("capture")
    capture.add_argument("--repo-root", default=str(ROOT))
    capture.add_argument("--profile", default=str(PROFILE_PATH))
    capture.add_argument("--distro", choices=("humble", "jazzy"), required=True)
    capture.add_argument("--dependency-leg", choices=("absent", "present"), required=True)
    capture.add_argument("--output-root", required=True)
    campaign = sub.add_parser("campaign")
    campaign.add_argument("--repo-root", default=str(ROOT))
    campaign.add_argument("--profile", default=str(PROFILE_PATH))
    campaign.add_argument("--campaign-root", required=True)
    inside = sub.add_parser("in-container")
    inside.add_argument("--phase", choices=("apt-source", "dependency"), required=True)
    inside.add_argument("--output-root", required=True)
    inside.add_argument("--distro", default=os.environ.get("ROS_DISTRO", ""))
    inside.add_argument("--apt-url-policy", choices=(APT_URL_POLICY_ID,), required=True)
    return parser


def main(argv: list[str] | None = None) -> int:
    args = _parser().parse_args(argv)
    try:
        if args.command == "plan":
            result = build_plan(Path(args.repo_root), Path(args.profile), args.distro,
                                args.dependency_leg, Path(args.output_root))
        elif args.command == "capture":
            result = capture_dependency_closure(
                Path(args.repo_root), Path(args.profile), args.distro,
                args.dependency_leg, Path(args.output_root))
        elif args.command == "campaign":
            result = capture_campaign(Path(args.repo_root), Path(args.profile),
                                      Path(args.campaign_root))
        else:
            result = _capture_in_container(
                args.phase, Path(args.output_root), args.distro,
                apt_source_url_policy=APT_SOURCE_URL_POLICY)
        print(json.dumps(result, sort_keys=True))
        return 0 if result.get("status") in {
            "REVIEW_REQUIRED", "PASS_REVIEW_REQUIRED", "CAPTURED_REVIEW_REQUIRED",
        } else 3
    except Exception as error:
        print(json.dumps({"status": "FAIL_CLOSED", "kind": getattr(error, "kind", "CAPTURE_FAILURE"),
                          "message": str(error)}, sort_keys=True), file=sys.stderr)
        return 2


if __name__ == "__main__":
    raise SystemExit(main())

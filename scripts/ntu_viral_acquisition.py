#!/usr/bin/env python3
"""Read-only-contract helpers for the NTU VIRAL acquisition candidate.

This module deliberately separates acquisition from profile authorization.  It
may download only role URLs already present in the selection receipt, and it
never changes a profile or exposes a ground-truth path to a runner.  The
production transport is HTTPS; tests inject a local fixture transport.
"""

from __future__ import annotations

from dataclasses import dataclass
from datetime import datetime, timezone
import hashlib
import json
import os
from pathlib import Path, PurePosixPath
import stat
import subprocess
import tarfile
from typing import Any, Callable, Mapping
from urllib.parse import urlparse
from urllib.request import HTTPRedirectHandler, Request, build_opener
import zipfile

import yaml

try:
    from lidarslam_benchmark_tools.competitive_identity_hash import canonical_profile_sha256
except ModuleNotFoundError:  # direct script execution
    from lidarslam_benchmark_tools.competitive_identity_hash import canonical_profile_sha256  # type: ignore


try:
    from lidarslam_benchmark_tools import package_root
except ModuleNotFoundError:  # direct script execution
    def package_root() -> Path:
        return Path(__file__).resolve().parents[1]


ROOT = package_root()
SEQUENCES = ("eee_01", "nya_01", "spms_01")
CALIBRATION_ROLES = ("calib_stereo", "calib_stereo_imu")
REQUIRED_SYSTEMS = ("ours", "glim", "fast_livo2")
RECEIPT_KIND = "ntu_viral_candidate_acquisition"
CONTRACT_VERSION = "ntu-viral-acquisition-v1"
SHA256_HEX = frozenset("0123456789abcdef")
MAX_DOWNLOAD_BYTES = 64 * 1024 * 1024 * 1024


class NtuAcquisitionError(ValueError):
    """A fail-closed acquisition or validation error."""

    def __init__(self, code: str, message: str) -> None:
        super().__init__(message)
        self.code = code
        self.message = message


@dataclass(frozen=True)
class AcquisitionContext:
    selection: Mapping[str, Any]
    profile: Mapping[str, Any]
    selection_sha256: str
    profile_sha256: str
    selection_id: str
    selected: Mapping[str, Mapping[str, Any]]
    mount_spec: Mapping[str, Any]


Transport = Callable[[str, Path], Mapping[str, Any]]


def _canonical_json(value: Any) -> bytes:
    return json.dumps(value, sort_keys=True, separators=(",", ":"),
                      ensure_ascii=True).encode("utf-8")


def canonical_sha256(value: Any) -> str:
    return hashlib.sha256(_canonical_json(value)).hexdigest()


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open("rb") as stream:
        for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
            digest.update(block)
    return digest.hexdigest()


def _hash_stream(stream: Any) -> tuple[str, int]:
    digest = hashlib.sha256()
    size = 0
    for block in iter(lambda: stream.read(4 * 1024 * 1024), b""):
        size += len(block)
        if size > MAX_DOWNLOAD_BYTES:
            raise NtuAcquisitionError("SIZE_LIMIT", "download exceeds the bounded size limit")
        digest.update(block)
    return digest.hexdigest(), size


def _require_mapping(value: Any, label: str) -> Mapping[str, Any]:
    if not isinstance(value, Mapping):
        raise NtuAcquisitionError("MALFORMED", f"{label} must be a mapping")
    return value


def _require_text(value: Any, label: str) -> str:
    if not isinstance(value, str) or not value.strip():
        raise NtuAcquisitionError("MISSING", f"{label} is missing")
    return value.strip()


def _safe_relative(value: Any, label: str, *, basename: bool = False) -> str:
    text = _require_text(value, label).replace("\\", "/")
    path = PurePosixPath(text)
    if path.is_absolute() or any(part in ("", ".", "..") for part in path.parts):
        raise NtuAcquisitionError("UNSAFE_PATH", f"{label} is not a safe relative path")
    if basename and len(path.parts) != 1:
        raise NtuAcquisitionError("UNSAFE_PATH", f"{label} must be a basename")
    return path.as_posix()


def _read_yaml(path: Path) -> Mapping[str, Any]:
    try:
        value = yaml.safe_load(path.read_text(encoding="utf-8"))
    except (OSError, UnicodeError, yaml.YAMLError) as exc:
        raise NtuAcquisitionError("READ_FAILED", f"cannot read {path}: {exc}") from exc
    return _require_mapping(value, str(path))


def _selection_hash(path: Path) -> str:
    return sha256_file(path)


def _profile_contract(profile: Mapping[str, Any]) -> Mapping[str, Any]:
    value = profile.get("competitive_slam_profile", profile)
    return _require_mapping(value, "competitive_slam_profile")


def load_context(selection_path: Path, profile_path: Path, *, root: Path = ROOT) -> AcquisitionContext:
    selection = _read_yaml(selection_path)
    profile = _read_yaml(profile_path)
    if selection.get("schema_version") != 1 or selection.get("family_id") != "ntu_viral":
        raise NtuAcquisitionError("SELECTION_IDENTITY", "selection is not NTU VIRAL schema v1")
    selection_id = _require_text(selection.get("selection_id"), "selection_id")
    if selection.get("partition") != "historical":
        raise NtuAcquisitionError("SELECTION_PARTITION", "NTU VIRAL selection must be historical")
    if selection.get("status") not in {"NOT_READY", "ACQUISITION_OPEN"}:
        raise NtuAcquisitionError("SELECTION_STATUS", "selection is not open for acquisition")
    raw_selected = selection.get("selected_sequences")
    if not isinstance(raw_selected, list):
        raise NtuAcquisitionError("SELECTION_ENTRIES", "selected_sequences is missing")
    selected: dict[str, Mapping[str, Any]] = {}
    for item in raw_selected:
        item = _require_mapping(item, "selected sequence")
        sequence = _require_text(item.get("sequence_id"), "selected sequence_id")
        if sequence in selected or sequence not in SEQUENCES:
            raise NtuAcquisitionError("SEQUENCE_SET", f"unexpected or duplicate sequence: {sequence}")
        if item.get("evaluation_eligible") is not True or item.get("fresh_eligible") is not False:
            raise NtuAcquisitionError("SEQUENCE_ELIGIBILITY", f"sequence {sequence} has unsafe eligibility")
        if set(item.get("supported_systems", [])) != set(REQUIRED_SYSTEMS):
            raise NtuAcquisitionError("RUNNER_SUPPORT", f"sequence {sequence} lacks exact runner support")
        selected[sequence] = item
    if tuple(sorted(selected)) != tuple(sorted(SEQUENCES)):
        raise NtuAcquisitionError("SEQUENCE_SET", "selection must contain exactly eee_01/nya_01/spms_01")
    excluded = selection.get("excluded_sequences")
    if not isinstance(excluded, list) or not any(
            isinstance(item, Mapping) and item.get("sequence_id") == "tnp_01" and
            item.get("evaluation_eligible") is False and
            item.get("fresh_eligible") is False for item in excluded):
        raise NtuAcquisitionError("TNP_EXCLUSION", "tnp_01 exclusion is missing or enabled")
    acquisition = _require_mapping(selection.get("acquisition"), "acquisition")
    mount_spec = _require_mapping(acquisition.get("evidence_mount"), "acquisition.evidence_mount")
    _require_text(mount_spec.get("path"), "evidence_mount.path")
    _require_text(mount_spec.get("uuid"), "evidence_mount.uuid")
    _require_text(mount_spec.get("filesystem"), "evidence_mount.filesystem")
    _require_text(mount_spec.get("label"), "evidence_mount.label")
    if mount_spec.get("require_rw") is not True:
        raise NtuAcquisitionError("MOUNT_POLICY", "evidence mount must require rw verification")

    contract = _profile_contract(profile)
    datasets = _require_mapping(contract.get("datasets"), "profile.datasets")
    ntu_hist = _require_mapping(datasets.get("ntu_viral_historical"),
                                 "profile.datasets.ntu_viral_historical")
    ntu_dev = _require_mapping(datasets.get("ntu_viral_development"),
                               "profile.datasets.ntu_viral_development")
    for sequence, item in selected.items():
        profile_item = _require_mapping(ntu_hist.get(sequence), f"profile NTU {sequence}")
        if profile_item.get("sequence") != sequence or profile_item.get("evaluation_eligible") is not True or profile_item.get("fresh_eligible") is not False:
            raise NtuAcquisitionError("PROFILE_BINDING", f"profile binding is unsafe for {sequence}")
        if item.get("profile_key") != f"ntu_viral_{sequence}":
            raise NtuAcquisitionError("PROFILE_BINDING", f"selection profile key is unsafe for {sequence}")
    tnp = _require_mapping(ntu_dev.get("tnp_01"), "profile NTU tnp_01")
    if tnp.get("evaluation_eligible") is not False or tnp.get("fresh_eligible") is not False:
        raise NtuAcquisitionError("TNP_EXCLUSION", "profile re-enables tnp_01")
    return AcquisitionContext(
        selection=selection,
        profile=profile,
        selection_sha256=_selection_hash(selection_path),
        profile_sha256=canonical_profile_sha256(dict(profile)),
        selection_id=selection_id,
        selected=selected,
        mount_spec=mount_spec,
    )


def read_mount_observation(evidence_root: Path, *, injected: Mapping[str, Any] | None = None) -> dict[str, Any]:
    if injected is not None:
        return dict(_require_mapping(injected, "mount observation"))
    command = ["findmnt", "--json", "--noheadings",
               "--output", "TARGET,SOURCE,FSTYPE,OPTIONS,UUID,LABEL",
               "--target", str(evidence_root)]
    try:
        completed = subprocess.run(command, check=True, capture_output=True,
                                    text=True, timeout=10)
        document = json.loads(completed.stdout)
        filesystems = document.get("filesystems")
        if not isinstance(filesystems, list) or len(filesystems) != 1:
            raise ValueError("findmnt returned no unique filesystem")
        return dict(_require_mapping(filesystems[0], "findmnt filesystem"))
    except (OSError, subprocess.SubprocessError, ValueError, json.JSONDecodeError) as exc:
        raise NtuAcquisitionError("MOUNT_INSPECTION", f"cannot inspect evidence mount: {exc}") from exc


def verify_mount_identity(evidence_root: Path, expected: Mapping[str, Any],
                          observation: Mapping[str, Any]) -> dict[str, Any]:
    root = evidence_root.resolve()
    expected_path = Path(_require_text(expected.get("path"), "evidence_mount.path")).resolve()
    if root != expected_path:
        raise NtuAcquisitionError("MOUNT_PATH", "evidence root does not match preregistered mount path")
    if not root.is_dir() or root.is_symlink():
        raise NtuAcquisitionError("MOUNT_PATH", "evidence root is not a regular directory")
    target = Path(_require_text(observation.get("target"), "mount.target")).resolve()
    if target != root:
        raise NtuAcquisitionError("MOUNT_TARGET", "findmnt target does not equal evidence root")
    uuid = _require_text(observation.get("uuid"), "mount.uuid").lower()
    expected_uuid = _require_text(expected.get("uuid"), "evidence_mount.uuid").lower()
    if uuid != expected_uuid:
        raise NtuAcquisitionError("MOUNT_UUID", "evidence mount UUID mismatch")
    fstype = _require_text(observation.get("fstype", observation.get("filesystem")), "mount.fstype").lower()
    if fstype != _require_text(expected.get("filesystem"), "evidence_mount.filesystem").lower():
        raise NtuAcquisitionError("MOUNT_FILESYSTEM", "evidence mount filesystem mismatch")
    label = _require_text(observation.get("label"), "mount.label")
    if label != _require_text(expected.get("label"), "evidence_mount.label"):
        raise NtuAcquisitionError("MOUNT_LABEL", "evidence mount label mismatch")
    options = str(observation.get("options", ""))
    if expected.get("require_rw") is True and ("ro" in options.split(",") or "rw" not in options.split(",")):
        raise NtuAcquisitionError("MOUNT_READ_ONLY", "evidence mount is not verified read-write")
    return {
        "target": str(root), "source": observation.get("source"),
        "uuid": uuid, "fstype": fstype, "label": label,
        "options": options, "verified": True,
    }


def _validate_url_spec(spec: Mapping[str, Any], label: str) -> tuple[str, str, bool, list[str]]:
    if spec.get("official_primary") is not True:
        raise NtuAcquisitionError("SOURCE_POLICY", f"{label} is not marked official primary")
    url = _require_text(spec.get("url"), f"{label}.url")
    parsed = urlparse(url)
    hosts = spec.get("allowed_hosts")
    if parsed.scheme != "https" or not parsed.hostname or not isinstance(hosts, list) or parsed.hostname not in hosts:
        raise NtuAcquisitionError("SOURCE_POLICY", f"{label} URL is not an allowlisted official HTTPS URL")
    if parsed.username or parsed.password or parsed.fragment:
        raise NtuAcquisitionError("SOURCE_POLICY", f"{label} URL contains unsafe credentials/fragment")
    filename = _safe_relative(spec.get("expected_filename"), f"{label}.expected_filename", basename=True)
    archive = spec.get("archive")
    if not isinstance(archive, bool):
        raise NtuAcquisitionError("ROLE_SPEC", f"{label}.archive is malformed")
    members = spec.get("expected_member_paths")
    if not isinstance(members, list) or len(set(members)) != len(members):
        raise NtuAcquisitionError("ROLE_SPEC", f"{label}.expected_member_paths is malformed")
    normalized_members = [_safe_relative(item, f"{label}.expected_member_paths") for item in members]
    if archive and not normalized_members:
        raise NtuAcquisitionError("ROLE_SPEC", f"{label} archive has no expected member paths")
    if not archive and normalized_members:
        raise NtuAcquisitionError("ROLE_SPEC", f"{label} non-archive has archive members")
    return url, filename, archive, normalized_members


def _iter_role_specs(context: AcquisitionContext) -> list[tuple[str, Mapping[str, Any], str, str]]:
    roles: list[tuple[str, Mapping[str, Any], str, str]] = []
    for sequence in SEQUENCES:
        item = context.selected[sequence]
        refs = _require_mapping(item.get("download_refs"), f"{sequence}.download_refs")
        for role in ("sequence_archive", "ground_truth_csv"):
            roles.append((f"{sequence}/{role}", _require_mapping(refs.get(role), f"{sequence}.{role}"), sequence, role))
    calibration = _require_mapping(context.selection["acquisition"].get("calibration_roles"),
                                   "acquisition.calibration_roles")
    for role in CALIBRATION_ROLES:
        roles.append((f"calibration/{role}", _require_mapping(calibration.get(role), role), "calibration", role))
    return roles


class _RecordingRedirectHandler(HTTPRedirectHandler):
    def __init__(self, chain: list[str]) -> None:
        super().__init__()
        self.chain = chain

    def redirect_request(self, req: Any, fp: Any, code: int, msg: str,
                         headers: Any, newurl: str) -> Any:
        self.chain.append(newurl)
        return super().redirect_request(req, fp, code, msg, headers, newurl)


def https_transport(url: str, destination: Path) -> Mapping[str, Any]:
    chain = [url]
    opener = build_opener(_RecordingRedirectHandler(chain))
    request = Request(url, headers={"User-Agent": "lidarslam-ntu-acquisition/1"})
    try:
        with opener.open(request, timeout=60) as response:
            status = int(getattr(response, "status", response.getcode()))
            if status != 200:
                raise NtuAcquisitionError("HTTP_STATUS", f"HTTP status {status} for {url}")
            with destination.open("wb") as stream:
                copied = 0
                while True:
                    block = response.read(4 * 1024 * 1024)
                    if not block:
                        break
                    copied += len(block)
                    if copied > MAX_DOWNLOAD_BYTES:
                        raise NtuAcquisitionError("SIZE_LIMIT", "download exceeds the bounded size limit")
                    stream.write(block)
            headers = {}
            for key in ("content-length", "content-type", "etag", "last-modified"):
                value = response.headers.get(key)
                if value is not None:
                    headers[key] = value
            return {
                "status_code": status, "final_url": response.geturl(),
                "redirect_chain": chain, "headers": headers,
            }
    except NtuAcquisitionError:
        raise
    except Exception as exc:  # urllib has multiple concrete network exceptions
        raise NtuAcquisitionError("DOWNLOAD_FAILED", f"HTTPS download failed for {url}: {exc}") from exc


def _regular_nosymlink(path: Path, label: str) -> os.stat_result:
    current = Path(path.anchor)
    for component in path.absolute().parts[1:]:
        current /= component
        if current.is_symlink():
            raise NtuAcquisitionError("SYMLINK_REJECTED", f"{label} contains a symlink")
    try:
        info = path.lstat()
    except OSError as exc:
        raise NtuAcquisitionError("FILE_MISSING", f"{label} is missing") from exc
    if stat.S_ISLNK(info.st_mode) or not stat.S_ISREG(info.st_mode) or info.st_nlink != 1:
        raise NtuAcquisitionError("FILE_INTEGRITY", f"{label} is not a regular single-link file")
    return info


def _archive_tree_hash(entries: list[tuple[str, int, str]]) -> str:
    return hashlib.sha256(_canonical_json([
        {"path": path, "size_bytes": size, "sha256": digest}
        for path, size, digest in sorted(entries)
    ])).hexdigest()


def inspect_safe_archive(path: Path, expected_members: list[str]) -> dict[str, Any]:
    """Inventory an archive without extracting it; reject traversal/links."""
    expected = set(expected_members)
    entries: list[tuple[str, int, str]] = []
    names: set[str] = set()
    try:
        if zipfile.is_zipfile(path):
            with zipfile.ZipFile(path) as archive:
                for info in archive.infolist():
                    name = info.filename.replace("\\", "/")
                    if name.endswith("/"):
                        continue
                    safe = _safe_relative(name, "zip member")
                    if safe in names:
                        raise NtuAcquisitionError("ARCHIVE_DUPLICATE", f"duplicate zip member {safe}")
                    names.add(safe)
                    mode = (info.external_attr >> 16) & 0xFFFF
                    if stat.S_IFMT(mode) == stat.S_IFLNK:
                        raise NtuAcquisitionError("ARCHIVE_LINK", f"zip symlink member {safe}")
                    with archive.open(info) as stream:
                        digest, size = _hash_stream(stream)
                    entries.append((safe, size, digest))
            fmt = "zip"
        elif tarfile.is_tarfile(path):
            with tarfile.open(path, "r:*") as archive:
                for info in archive.getmembers():
                    safe = _safe_relative(info.name, "tar member")
                    # Directory entries are harmless metadata and are not
                    # byte artifacts.  Links and all other special members
                    # remain forbidden.
                    if info.isdir():
                        continue
                    if safe in names:
                        raise NtuAcquisitionError("ARCHIVE_DUPLICATE", f"duplicate tar member {safe}")
                    names.add(safe)
                    if info.issym() or info.islnk() or not info.isfile():
                        raise NtuAcquisitionError("ARCHIVE_LINK", f"unsafe tar member {safe}")
                    stream = archive.extractfile(info)
                    if stream is None:
                        raise NtuAcquisitionError("ARCHIVE_MEMBER", f"tar member is unreadable: {safe}")
                    with stream:
                        digest, size = _hash_stream(stream)
                    entries.append((safe, size, digest))
            fmt = "tar"
        else:
            raise NtuAcquisitionError("ARCHIVE_FORMAT", f"unsupported archive format: {path.name}")
    except NtuAcquisitionError:
        raise
    except (OSError, EOFError, tarfile.TarError, zipfile.BadZipFile) as exc:
        raise NtuAcquisitionError("ARCHIVE_FORMAT", f"cannot inspect archive {path.name}: {exc}") from exc
    if not expected.issubset(names):
        missing = sorted(expected - names)
        raise NtuAcquisitionError("ARCHIVE_EXPECTED_MEMBER", "missing expected archive members: " + ", ".join(missing))
    return {"format": fmt, "member_count": len(entries),
            "members": [{"path": p, "size_bytes": s, "sha256": d}
                        for p, s, d in sorted(entries)],
            "tree_sha256": _archive_tree_hash(entries)}


def _atomic_write_new(path: Path, payload: bytes, mode: int = 0o444) -> str:
    if os.path.lexists(path):
        raise NtuAcquisitionError("OUTPUT_OVERWRITE", f"refusing to overwrite {path}")
    flags = os.O_WRONLY | os.O_CREAT | os.O_EXCL | getattr(os, "O_NOFOLLOW", 0)
    fd = os.open(path, flags, 0o600)
    try:
        with os.fdopen(fd, "wb") as stream:
            stream.write(payload)
            stream.flush()
            os.fsync(stream.fileno())
    except Exception:
        try:
            path.unlink()
        except FileNotFoundError:
            pass
        raise
    os.chmod(path, mode, follow_symlinks=False)
    return hashlib.sha256(payload).hexdigest()


def _seal(root: Path, receipt: Mapping[str, Any]) -> tuple[Path, str]:
    receipt_path = root / "candidate_receipt.json"
    payload = (json.dumps(receipt, indent=2, sort_keys=True) + "\n").encode("utf-8")
    digest = _atomic_write_new(receipt_path, payload)
    sidecar = receipt_path.with_name(receipt_path.name + ".sha256")
    _atomic_write_new(sidecar, (digest + "  " + receipt_path.name + "\n").encode("ascii"))
    for directory in sorted((item for item in root.rglob("*") if item.is_dir()),
                            key=lambda item: len(item.parts), reverse=True):
        os.chmod(directory, 0o555, follow_symlinks=False)
    os.chmod(root, 0o555, follow_symlinks=False)
    return receipt_path, digest


def _reserve(root: Path, evidence_root: Path) -> str:
    if root.is_absolute() is False:
        raise NtuAcquisitionError("ROOT_PATH", "candidate root must be absolute")
    resolved_evidence = evidence_root.resolve()
    resolved_root = root.resolve(strict=False)
    try:
        relative = resolved_root.relative_to(resolved_evidence)
    except ValueError as exc:
        raise NtuAcquisitionError("ROOT_SCOPE", "candidate root is outside evidence mount") from exc
    if not relative.parts or any(part in (".", "..") for part in relative.parts):
        raise NtuAcquisitionError("ROOT_SCOPE", "candidate root must be below evidence mount")
    if root.exists() or os.path.lexists(root):
        raise NtuAcquisitionError("ROOT_REUSE", "candidate root already exists; reuse/overwrite is forbidden")
    parent = root.parent
    if not parent.is_dir() or parent.is_symlink():
        raise NtuAcquisitionError("ROOT_PARENT", "candidate root parent must already be a regular directory")
    os.mkdir(root, 0o700)
    for child in (root / "sequences", root / "calibration"):
        os.mkdir(child, 0o700)
    return relative.as_posix()


def _download_one(url: str, destination: Path, transport: Transport | None) -> dict[str, Any]:
    part = destination.with_name(destination.name + ".part")
    if os.path.lexists(destination) or os.path.lexists(part):
        raise NtuAcquisitionError("OUTPUT_OVERWRITE", f"download destination already exists: {destination.name}")
    metadata = dict((transport or https_transport)(url, part))
    _regular_nosymlink(part, f"download staging {destination.name}")
    # A plain replace would silently overwrite a race-created destination.
    # Linking and then unlinking the private .part file is an atomic
    # no-overwrite hand-off on the same evidence filesystem.
    try:
        os.link(part, destination, follow_symlinks=False)
    except FileExistsError as exc:
        raise NtuAcquisitionError("OUTPUT_OVERWRITE", f"download destination appeared during acquisition: {destination.name}") from exc
    os.unlink(part)
    _regular_nosymlink(destination, f"download {destination.name}")
    return metadata


def _validate_final_url(metadata: Mapping[str, Any], requested: str, allowed_hosts: list[str], label: str) -> str:
    final_url = _require_text(metadata.get("final_url", requested), f"{label}.final_url")
    chain = metadata.get("redirect_chain", [requested, final_url])
    if not isinstance(chain, list) or not chain:
        raise NtuAcquisitionError("REDIRECT_METADATA", f"{label} redirect chain is missing")
    for value in chain + [final_url]:
        parsed = urlparse(str(value))
        if parsed.scheme != "https" or parsed.hostname not in allowed_hosts:
            raise NtuAcquisitionError("REDIRECT_POLICY", f"{label} redirect leaves official HTTPS allowlist")
    return final_url


def _role_destination(root: Path, sequence: str, role: str, filename: str) -> tuple[Path, str]:
    if sequence == "calibration":
        relative = PurePosixPath("calibration", role, filename).as_posix()
    else:
        relative = PurePosixPath("sequences", sequence, role, filename).as_posix()
    destination = root / Path(relative)
    destination.parent.mkdir(parents=True, exist_ok=False)
    return destination, relative


def acquire_candidate(selection_path: Path, profile_path: Path, evidence_root: Path,
                     candidate_root: Path, *, mount_observation: Mapping[str, Any] | None = None,
                     transport: Transport | None = None, now_utc: str | None = None,
                     root: Path = ROOT) -> dict[str, Any]:
    """Acquire one fresh NTU candidate; return a sealed PASS/FAIL receipt.

    Mount and selection validation occur before the candidate directory is
    reserved.  Once reserved, every failure is sealed in that fresh root and
    the root is never reusable.
    """
    context = load_context(selection_path, profile_path, root=root)
    # Validate all role metadata before reserving a root.  Production selection
    # intentionally has null moving URLs until an operator records exact ones.
    role_specs: list[tuple[str, Mapping[str, Any], str, str]] = []
    for label, spec, sequence, role in _iter_role_specs(context):
        _validate_url_spec(spec, label)
        role_specs.append((label, spec, sequence, role))
    observed = read_mount_observation(evidence_root, injected=mount_observation)
    mount = verify_mount_identity(evidence_root, context.mount_spec, observed)
    candidate_relative = _reserve(candidate_root, evidence_root)
    started = now_utc or datetime.now(timezone.utc).replace(microsecond=0).isoformat().replace("+00:00", "Z")
    artifacts: list[dict[str, Any]] = []
    failure: dict[str, Any] | None = None
    try:
        for label, spec, sequence, role in role_specs:
            requested, filename, archive, expected_members = _validate_url_spec(spec, label)
            destination, relative = _role_destination(candidate_root, sequence, role, filename)
            metadata = _download_one(requested, destination, transport)
            final_url = _validate_final_url(metadata, requested, list(spec["allowed_hosts"]), label)
            info = _regular_nosymlink(destination, label)
            artifact: dict[str, Any] = {
                "role": role, "sequence_id": sequence, "relative_path": relative,
                "requested_url": requested, "final_url": final_url,
                "redirect_chain": list(metadata.get("redirect_chain", [requested, final_url])),
                "http": {"status_code": metadata.get("status_code", 200),
                         "headers": dict(metadata.get("headers", {}))},
                "retrieved_at_utc": metadata.get("retrieved_at_utc", started),
                "size_bytes": info.st_size, "sha256": sha256_file(destination),
                "hash_kind": "sha256_file_v1",
            }
            if archive:
                artifact["archive"] = inspect_safe_archive(destination, expected_members)
            else:
                artifact["tree_sha256"] = canonical_sha256({
                    "path": relative, "size_bytes": info.st_size,
                    "sha256": artifact["sha256"],
                })
            artifacts.append(artifact)
        receipt: dict[str, Any] = {
            "schema_version": 1, "receipt_kind": RECEIPT_KIND,
            "contract_version": CONTRACT_VERSION, "status": "PASS",
            "selection_id": context.selection_id,
            "selection_sha256": context.selection_sha256,
            "profile_sha256": context.profile_sha256,
            "profile_sha256_kind": "canonical_profile_sha256_v1",
            "candidate_root_relative": candidate_relative,
            "mount_identity": mount,
            "artifacts": artifacts,
            "runner_input_manifest": {
                "ground_truth_paths_exposed": False,
                "ground_truth_relative_paths": [],
                "input_relative_paths": sorted(
                    item["relative_path"] for item in artifacts if item["role"] == "sequence_archive"),
            },
            "retry_policy": {
                "candidate_root_reuse": False, "overwrite": False,
                "partial_download_reuse": False, "retry_after_failure": False,
            },
            "candidate_seal": {"sealed": True, "review_status": "CANDIDATE_ONLY",
                               "profile_edit_performed": False},
        }
    except NtuAcquisitionError as exc:
        failure = {"code": exc.code, "message": exc.message}
        receipt = {
            "schema_version": 1, "receipt_kind": RECEIPT_KIND,
            "contract_version": CONTRACT_VERSION, "status": "FAIL_CLOSED",
            "selection_id": context.selection_id,
            "selection_sha256": context.selection_sha256,
            "profile_sha256": context.profile_sha256,
            "profile_sha256_kind": "canonical_profile_sha256_v1",
            "candidate_root_relative": candidate_relative,
            "mount_identity": mount, "artifacts": artifacts,
            "failure": failure,
            "runner_input_manifest": {"ground_truth_paths_exposed": False,
                                       "ground_truth_relative_paths": [],
                                       "input_relative_paths": []},
            "retry_policy": {"candidate_root_reuse": False, "overwrite": False,
                             "partial_download_reuse": False, "retry_after_failure": False},
            "candidate_seal": {"sealed": True, "review_status": "CANDIDATE_FAILURE",
                               "profile_edit_performed": False},
        }
    receipt_path, receipt_sha = _seal(candidate_root, receipt)
    result = {"status": receipt["status"], "receipt_path": str(receipt_path),
              "receipt_sha256": receipt_sha, "candidate_root_relative": candidate_relative,
              "selection_id": context.selection_id, "failure": failure}
    return result

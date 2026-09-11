#!/usr/bin/env python3
"""Build a provenance-bound short demo, captions, and post copy locally."""

from __future__ import annotations

import argparse
import hashlib
import json
import math
import os
from pathlib import Path, PurePosixPath
import re
import shutil
import subprocess
import sys
import tempfile
from typing import Any, Sequence

from PIL import Image, ImageDraw, ImageFont

from product_schema import validate_contract


REPO_ROOT = Path(__file__).resolve().parents[1]
CONTRACT_PATH = (
    REPO_ROOT / 'docs' / 'contracts' / 'social-demo-media-v1.json'
)
CONTRACT_SCHEMA = 'social-demo-media-v1.schema.json'
MANIFEST_SCHEMA = 'social-demo-media-manifest-v1.schema.json'
MANIFEST_URI = (
    'https://rsasaki0109.github.io/lidar_slam_ros2/'
    'schemas/social-demo-media-manifest-v1.schema.json'
)
DEFAULT_OUTPUT_DIR = REPO_ROOT / 'output' / 'social-demo-media'
VIDEO_NAME = 'social_autoware_map_authoring_demo.mp4'
CAPTIONS_NAME = 'social_autoware_map_authoring_demo.en.vtt'
POST_NAME = 'autoware_map_authoring_post_v0.9.1.md'
MANIFEST_NAME = 'social_autoware_map_authoring_demo.manifest.json'
REVISION_PATTERN = re.compile(r'^[0-9a-f]{40}$')
FORBIDDEN_PUBLIC_COPY = (
    'v0.2.2',
    'run_autoware_map_beginner.sh',
    'run_autoware_quickstart.sh',
    'APE RMSE',
    'about 50%',
    'is out',
    'を公開しています',
)


class MediaError(ValueError):
    """The media packet cannot be generated without overstating evidence."""


def _sha256(payload: bytes) -> str:
    return hashlib.sha256(payload).hexdigest()


def _regular_file(path: Path, label: str) -> bytes:
    if path.is_symlink() or not path.is_file():
        raise MediaError(f'{label} must be a regular non-symlink file: {path}')
    try:
        payload = path.read_bytes()
    except OSError as exc:
        raise MediaError(f'cannot read {label} {path}: {exc}') from exc
    if not payload:
        raise MediaError(f'{label} is empty: {path}')
    return payload


def _repository_path(value: str, label: str) -> Path:
    pure = PurePosixPath(value)
    if pure.is_absolute() or '..' in pure.parts or not pure.parts:
        raise MediaError(f'{label} must be a repository-relative path: {value!r}')
    path = REPO_ROOT.joinpath(*pure.parts)
    try:
        path.relative_to(REPO_ROOT)
    except ValueError as exc:
        raise MediaError(f'{label} escapes the repository: {value!r}') from exc
    return path


def _load_json_object(path: Path, label: str) -> dict[str, Any]:
    payload = _regular_file(path, label)
    try:
        parsed = json.loads(payload.decode('utf-8'))
    except (UnicodeDecodeError, json.JSONDecodeError) as exc:
        raise MediaError(f'{label} is not valid UTF-8 JSON: {path}: {exc}') from exc
    if not isinstance(parsed, dict):
        raise MediaError(f'{label} root must be an object: {path}')
    return parsed


def load_contract(path: Path = CONTRACT_PATH) -> dict[str, Any]:
    """Load and verify the exact product copy and source-image identities."""
    contract = _load_json_object(path, 'social demo media contract')
    try:
        validate_contract(contract, CONTRACT_SCHEMA)
    except (FileNotFoundError, ValueError) as exc:
        raise MediaError(f'social demo media contract failed schema validation: {exc}') from exc

    version = _regular_file(REPO_ROOT / 'VERSION', 'VERSION').decode('utf-8').strip()
    if contract['product_version'] != version:
        raise MediaError(
            'social demo contract product_version does not match VERSION: '
            f"{contract['product_version']!r} != {version!r}"
        )

    serialized = json.dumps(contract, sort_keys=True, ensure_ascii=False)
    stale = [token for token in FORBIDDEN_PUBLIC_COPY if token in serialized]
    if stale:
        raise MediaError(
            'social demo contract contains stale or unsupported public copy: '
            + ', '.join(stale)
        )

    docs_payload = '\n'.join(
        _regular_file(REPO_ROOT / relative, relative).decode('utf-8')
        for relative in ('README.md', 'docs/getting-started.md')
    )
    for command in contract['commands'].values():
        if command not in docs_payload:
            raise MediaError(
                f'contract command is not present in canonical docs: {command!r}'
            )

    expected_inputs = {
        item['path']: item['sha256'] for item in contract['source_images']
    }
    for relative, expected_hash in expected_inputs.items():
        payload = _regular_file(
            _repository_path(relative, 'source image path'),
            f'source image {relative}',
        )
        observed = _sha256(payload)
        if observed != expected_hash:
            raise MediaError(
                f'source image SHA-256 drift for {relative}: '
                f'expected {expected_hash}, got {observed}'
            )

    for slide in contract['slides']:
        image_path = slide.get('image')
        if image_path is not None and image_path not in expected_inputs:
            raise MediaError(
                f"slide {slide['id']!r} uses an unbound source image: {image_path}"
            )
    return contract


def _git_output(args: Sequence[str]) -> str:
    try:
        return subprocess.run(
            ['git', *args],
            cwd=REPO_ROOT,
            check=True,
            capture_output=True,
            text=True,
        ).stdout.strip()
    except (OSError, subprocess.CalledProcessError) as exc:
        raise MediaError(f'cannot inspect repository identity: git {" ".join(args)}') from exc


def inspect_source_revision(source_revision: str, local_preview: bool) -> str:
    """Require exact clean source for a publication candidate."""
    revision = source_revision.strip().lower()
    if REVISION_PATTERN.fullmatch(revision) is None:
        raise MediaError(
            'source revision must be exactly 40 lowercase hexadecimal characters'
        )
    head = _git_output(('rev-parse', 'HEAD'))
    if head != revision:
        raise MediaError(
            f'source revision does not match HEAD: expected {revision}, got {head}'
        )
    dirty = _git_output(('status', '--porcelain'))
    if dirty and not local_preview:
        raise MediaError(
            'publication-candidate media requires a clean source worktree; '
            'use --local-preview only for non-publishable inspection'
        )
    return 'LOCAL_PREVIEW' if dirty else 'PUBLICATION_CANDIDATE'


def _load_font(size: int, bold: bool = False) -> ImageFont.ImageFont:
    candidates = []
    if bold:
        candidates.extend([
            '/usr/share/fonts/truetype/dejavu/DejaVuSans-Bold.ttf',
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Bold.ttc',
        ])
    else:
        candidates.extend([
            '/usr/share/fonts/truetype/dejavu/DejaVuSans.ttf',
            '/usr/share/fonts/opentype/noto/NotoSansCJK-Regular.ttc',
        ])
    for candidate in candidates:
        path = Path(candidate)
        if path.is_file():
            return ImageFont.truetype(str(path), size=size)
    return ImageFont.load_default()


def _fit_image(path: Path | None, size: tuple[int, int]) -> Image.Image:
    canvas = Image.new('RGB', size, '#08101b')
    if path is None:
        return canvas
    with Image.open(path) as opened:
        source = opened.convert('RGB')
        scale = max(size[0] / source.width, size[1] / source.height)
        scaled = source.resize(
            (
                max(1, int(source.width * scale)),
                max(1, int(source.height * scale)),
            ),
            _lanczos_resample(),
        )
    offset = ((size[0] - scaled.width) // 2, (size[1] - scaled.height) // 2)
    canvas.paste(scaled, offset)
    return canvas


def _lanczos_resample(image_module: Any = Image) -> Any:
    """Return the Pillow 9.1+ enum or its legacy module-level equivalent."""
    resampling = getattr(image_module, 'Resampling', image_module)
    return resampling.LANCZOS


def _wrapped_lines(
    draw: ImageDraw.ImageDraw,
    text: str,
    font: ImageFont.ImageFont,
    max_width: int,
) -> list[str]:
    words = text.split()
    lines: list[str] = []
    current = ''
    for word in words:
        candidate = word if not current else f'{current} {word}'
        bbox = draw.textbbox((0, 0), candidate, font=font)
        if bbox[2] - bbox[0] <= max_width:
            current = candidate
            continue
        if not current:
            raise MediaError(f'slide word exceeds text width: {word!r}')
        lines.append(current)
        current = word
    if current:
        lines.append(current)
    return lines


def _draw_wrapped_text(
    draw: ImageDraw.ImageDraw,
    text: str,
    font: ImageFont.ImageFont,
    fill: str,
    xy: tuple[int, int],
    max_width: int,
    line_spacing: int,
) -> int:
    x, y = xy
    for line in _wrapped_lines(draw, text, font, max_width):
        draw.text((x, y), line, font=font, fill=fill)
        bbox = draw.textbbox((x, y), line, font=font)
        y = bbox[3] + line_spacing
    return y


def render_slide(
    contract: dict[str, Any],
    slide: dict[str, Any],
    *,
    size: tuple[int, int] | None = None,
) -> Image.Image:
    """Render one contract-bound slide without adding unreviewed claims."""
    frame = contract['frame']
    output_size = size or (frame['width'], frame['height'])
    relative = slide.get('image')
    source = (
        None if relative is None
        else _repository_path(relative, f"slide {slide['id']} image")
    )
    canvas = _fit_image(source, output_size)
    canvas = Image.blend(
        canvas,
        Image.new('RGB', output_size, '#08101b'),
        0.30,
    )
    overlay = Image.new('RGBA', output_size, (0, 0, 0, 0))
    draw = ImageDraw.Draw(overlay)
    margin = max(36, int(output_size[0] * 0.044))
    panel = (
        margin,
        margin,
        output_size[0] - margin,
        output_size[1] - margin,
    )
    draw.rounded_rectangle(panel, radius=28, fill=(7, 12, 21, 214))

    scale = output_size[0] / 1280
    title_font = _load_font(max(28, int(46 * scale)), bold=True)
    body_font = _load_font(max(18, int(26 * scale)))
    eyebrow_font = _load_font(max(15, int(20 * scale)), bold=True)
    left = margin + max(24, int(32 * scale))
    cursor_y = margin + max(28, int(36 * scale))
    max_width = output_size[0] - (2 * left)

    draw.text(
        (left, cursor_y),
        slide['eyebrow'].upper(),
        font=eyebrow_font,
        fill='#8dd3ff',
    )
    cursor_y += max(34, int(42 * scale))
    cursor_y = _draw_wrapped_text(
        draw,
        slide['title'],
        title_font,
        'white',
        (left, cursor_y),
        max_width,
        max(8, int(12 * scale)),
    )
    cursor_y += max(24, int(30 * scale))
    for item in slide['body']:
        cursor_y = _draw_wrapped_text(
            draw,
            f'- {item}',
            body_font,
            '#d8e4f0',
            (left + max(8, int(8 * scale)), cursor_y),
            max_width - max(8, int(8 * scale)),
            max(8, int(14 * scale)),
        )
        cursor_y += max(7, int(8 * scale))
    if cursor_y > output_size[1] - margin - max(10, int(12 * scale)):
        raise MediaError(f"slide {slide['id']!r} text exceeds the safe panel")
    return Image.alpha_composite(canvas.convert('RGBA'), overlay).convert('RGB')


def _write_slides(
    contract: dict[str, Any],
    directory: Path,
) -> list[Path]:
    paths = []
    for index, slide in enumerate(contract['slides'], start=1):
        rendered = render_slide(contract, slide)
        path = directory / f'slide_{index:02d}.png'
        rendered.save(path, format='PNG')
        paths.append(path)
    return paths


def _expected_duration(contract: dict[str, Any]) -> float:
    frame = contract['frame']
    raw = (
        len(contract['slides']) * frame['slide_seconds']
        - (len(contract['slides']) - 1) * frame['fade_seconds']
    )
    return math.ceil(raw * frame['fps']) / frame['fps']


def _ffmpeg_cmd(
    contract: dict[str, Any],
    slides: Sequence[Path],
    output: Path,
) -> list[str]:
    frame = contract['frame']
    cmd = ['ffmpeg', '-nostdin', '-hide_banner', '-loglevel', 'error']
    for slide in slides:
        cmd.extend([
            '-loop', '1',
            '-t', str(frame['slide_seconds']),
            '-i', str(slide),
        ])

    chains = []
    for index in range(len(slides)):
        chains.append(
            f'[{index}:v]fps={frame["fps"]},'
            f'scale={frame["width"]}:{frame["height"]}:'
            'force_original_aspect_ratio=decrease,'
            f'pad={frame["width"]}:{frame["height"]}:(ow-iw)/2:(oh-ih)/2,'
            f'setsar=1[v{index}]'
        )
    current = 'v0'
    offset = frame['slide_seconds'] - frame['fade_seconds']
    for index in range(1, len(slides)):
        out_name = f'x{index}'
        chains.append(
            f'[{current}][v{index}]xfade=transition=fade:'
            f'duration={frame["fade_seconds"]}:offset={offset}[{out_name}]'
        )
        current = out_name
        offset += frame['slide_seconds'] - frame['fade_seconds']
    chains.append(f'[{current}]format=yuv420p[video]')
    cmd.extend([
        '-filter_complex', ';'.join(chains),
        '-map', '[video]',
        '-an',
        '-c:v', 'libx264',
        '-preset', 'medium',
        '-crf', '23',
        '-threads', '1',
        '-r', str(frame['fps']),
        '-pix_fmt', 'yuv420p',
        '-map_metadata', '-1',
        '-fflags', '+bitexact',
        '-flags:v', '+bitexact',
        '-movflags', '+faststart',
        str(output),
    ])
    return cmd


def _timestamp(seconds: float) -> str:
    milliseconds = int(round(seconds * 1000))
    hours, remainder = divmod(milliseconds, 3_600_000)
    minutes, remainder = divmod(remainder, 60_000)
    whole_seconds, millis = divmod(remainder, 1000)
    return f'{hours:02d}:{minutes:02d}:{whole_seconds:02d}.{millis:03d}'


def build_captions(contract: dict[str, Any]) -> str:
    """Build non-overlapping English WebVTT cues for every slide."""
    frame = contract['frame']
    stride = frame['slide_seconds'] - frame['fade_seconds']
    final_end = _expected_duration(contract)
    blocks = ['WEBVTT', '']
    for index, slide in enumerate(contract['slides']):
        start = index * stride
        end = final_end if index == len(contract['slides']) - 1 else (index + 1) * stride
        blocks.extend([
            f'{_timestamp(start)} --> {_timestamp(end)}',
            slide['caption'],
            '',
        ])
    return '\n'.join(blocks)


def _docs_url(contract: dict[str, Any], source_revision: str) -> str:
    return contract['docs_url_template'].replace(
        '{source_revision}', source_revision
    )


def build_post_copy(
    contract: dict[str, Any],
    source_revision: str,
    status: str,
) -> str:
    """Build bounded Japanese and English copy without claiming publication."""
    version = contract['product_version']
    docs_url = _docs_url(contract, source_revision)
    demo = contract['commands']['fixed_demo']
    own_bag = contract['commands']['own_bag']
    return f"""# Social demo copy — v{version} candidate

> Status: **{status} / NOT_PUBLISHED**
>
> External publication authorized: **false**
>
> Source revision: `{source_revision}`

Suggested attachment: `{VIDEO_NAME}` with `{CAPTIONS_NAME}`.

## Japanese

rosbag2 から、検証可能な Autoware-compatible map bundle までを1本の流れにしました。

```bash
{demo}
{own_bag}
```

成功時は `pointcloud_map/`、`map_projector_info.yaml`、Lanelet2、
`map_verify: PASS`、検証 receipt を同じセッションで確認できます。
公開前の exact-version guide: <{docs_url}>

この下書きはリリース、対応パッケージ、性能優位、センサー互換性を主張しません。
投稿する場合は、同じ版の公開済み artifact と public docs の監査完了後に文言を再確認してください。

## English

One guided path now takes a rosbag2 recording to a verifiable
Autoware-compatible map bundle.

```bash
{demo}
{own_bag}
```

A successful session keeps the map bundle, `map_verify: PASS`, and its
validation receipt together. Exact-version guide before publication:
<{docs_url}>

This draft makes no release, package-availability, performance-superiority, or
sensor-compatibility claim. Recheck the wording only after the same-version
artifacts and public documentation pass their publication audits.

## Alt text

Four-slide silent demo showing a rosbag2-to-Autoware map workflow, the fixed
demo and own-bag commands, an Autoware map-loader view, and the verification
artifacts retained after a successful mapping session.
"""


def _artifact(path: Path) -> dict[str, Any]:
    payload = _regular_file(path, f'generated artifact {path.name}')
    return {
        'path': path.name,
        'size_bytes': len(payload),
        'sha256': _sha256(payload),
    }


def _probe_video(path: Path, contract: dict[str, Any]) -> dict[str, Any]:
    try:
        result = subprocess.run(
            [
                'ffprobe',
                '-v', 'error',
                '-show_entries',
                'format=duration:stream=codec_name,width,height,r_frame_rate',
                '-of', 'json',
                str(path),
            ],
            check=True,
            capture_output=True,
            text=True,
        )
        payload = json.loads(result.stdout)
    except (OSError, subprocess.CalledProcessError, json.JSONDecodeError) as exc:
        raise MediaError(f'ffprobe could not validate generated video: {exc}') from exc
    streams = payload.get('streams')
    if not isinstance(streams, list) or len(streams) != 1:
        raise MediaError('generated video must contain exactly one stream')
    stream = streams[0]
    frame = contract['frame']
    expected = {
        'codec_name': 'h264',
        'width': frame['width'],
        'height': frame['height'],
        'r_frame_rate': f'{frame["fps"]}/1',
    }
    for key, value in expected.items():
        if stream.get(key) != value:
            raise MediaError(
                f'generated video {key} drift: expected {value!r}, '
                f'got {stream.get(key)!r}'
            )
    try:
        duration = float(payload['format']['duration'])
    except (KeyError, TypeError, ValueError) as exc:
        raise MediaError('generated video duration is missing or invalid') from exc
    expected_duration = _expected_duration(contract)
    if abs(duration - expected_duration) > (1.0 / frame['fps']):
        raise MediaError(
            f'generated video duration drift: expected {expected_duration}, got {duration}'
        )
    if duration >= contract['maximum_duration_seconds']:
        raise MediaError(
            f'generated video exceeds {contract["maximum_duration_seconds"]} seconds'
        )
    return {
        'codec': stream['codec_name'],
        'width': stream['width'],
        'height': stream['height'],
        'fps': frame['fps'],
        'duration_seconds': duration,
    }


def build_manifest(
    contract: dict[str, Any],
    source_revision: str,
    status: str,
    directory: Path,
    video_probe: dict[str, Any],
) -> dict[str, Any]:
    """Bind all generated and source bytes without granting publication."""
    contract_payload = _regular_file(CONTRACT_PATH, 'social demo media contract')
    inputs = []
    for item in contract['source_images']:
        payload = _regular_file(
            _repository_path(item['path'], 'source image path'),
            f"source image {item['path']}",
        )
        inputs.append({
            'path': item['path'],
            'size_bytes': len(payload),
            'sha256': _sha256(payload),
        })
    manifest = {
        'schema_version': 1,
        'schema_uri': MANIFEST_URI,
        'status': status,
        'repository': contract['repository'],
        'source_revision': source_revision,
        'product_version': contract['product_version'],
        'content_contract': {
            'path': CONTRACT_PATH.relative_to(REPO_ROOT).as_posix(),
            'size_bytes': len(contract_payload),
            'sha256': _sha256(contract_payload),
        },
        'canonical_docs': {
            'source_path': contract['docs_source_path'],
            'url': _docs_url(contract, source_revision),
        },
        'commands': contract['commands'],
        'video': {**_artifact(directory / VIDEO_NAME), **video_probe},
        'captions': {
            **_artifact(directory / CAPTIONS_NAME),
            'language': 'en',
            'format': 'text/vtt',
            'cue_count': len(contract['slides']),
        },
        'post_copy': {
            **_artifact(directory / POST_NAME),
            'languages': ['ja', 'en'],
        },
        'source_images': inputs,
        'claims': {
            'numerical_performance_claims': False,
            'release_claim': False,
            'package_availability_claim': False,
            'sensor_compatibility_claim': False,
        },
        'publication_boundary': {
            'external_publication_authorized': False,
            'writes_performed': False,
        },
    }
    try:
        validate_contract(manifest, MANIFEST_SCHEMA)
    except (FileNotFoundError, ValueError) as exc:
        raise MediaError(f'generated social demo manifest is invalid: {exc}') from exc
    return manifest


def _write_exclusive(path: Path, text: str) -> None:
    try:
        with path.open('x', encoding='utf-8', newline='\n') as stream:
            stream.write(text)
    except OSError as exc:
        raise MediaError(f'cannot write generated artifact {path}: {exc}') from exc


def generate_bundle(
    contract: dict[str, Any],
    output_dir: Path,
    source_revision: str,
    status: str,
) -> dict[str, Any]:
    """Generate into a private temporary directory, then publish once locally."""
    output = output_dir.expanduser().resolve()
    if output.exists() or output.is_symlink():
        raise MediaError(f'refusing to overwrite media output directory: {output}')
    parent = output.parent
    if parent.is_symlink() or not parent.is_dir():
        raise MediaError(f'output parent must be a regular directory: {parent}')
    if shutil.which('ffmpeg') is None or shutil.which('ffprobe') is None:
        raise MediaError('ffmpeg and ffprobe are required on PATH')

    temporary = Path(tempfile.mkdtemp(prefix=f'.{output.name}.', dir=parent))
    try:
        slides_dir = temporary / '.slides'
        slides_dir.mkdir()
        slides = _write_slides(contract, slides_dir)
        video_path = temporary / VIDEO_NAME
        subprocess.run(_ffmpeg_cmd(contract, slides, video_path), check=True)
        shutil.rmtree(slides_dir)
        _write_exclusive(temporary / CAPTIONS_NAME, build_captions(contract))
        _write_exclusive(
            temporary / POST_NAME,
            build_post_copy(contract, source_revision, status),
        )
        video_probe = _probe_video(video_path, contract)
        manifest = build_manifest(
            contract,
            source_revision,
            status,
            temporary,
            video_probe,
        )
        _write_exclusive(
            temporary / MANIFEST_NAME,
            json.dumps(manifest, indent=2, sort_keys=True) + '\n',
        )
        os.replace(temporary, output)
    except Exception:
        shutil.rmtree(temporary, ignore_errors=True)
        raise
    return manifest


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--output-dir', type=Path, default=DEFAULT_OUTPUT_DIR)
    parser.add_argument('--source-revision', required=True)
    parser.add_argument(
        '--local-preview',
        action='store_true',
        help=(
            'Allow a dirty worktree and mark output LOCAL_PREVIEW. This never '
            'turns local media into publication authority.'
        ),
    )
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    args = parse_args(argv)
    try:
        contract = load_contract()
        status = inspect_source_revision(
            args.source_revision,
            args.local_preview,
        )
        manifest = generate_bundle(
            contract,
            args.output_dir,
            args.source_revision,
            status,
        )
    except (MediaError, OSError, subprocess.CalledProcessError) as exc:
        print(f'social demo media error: {exc}', file=sys.stderr)
        return 2
    print(json.dumps({
        'status': manifest['status'],
        'output_dir': str(args.output_dir.expanduser().resolve()),
        'video_sha256': manifest['video']['sha256'],
        'captions_sha256': manifest['captions']['sha256'],
        'publication_authorized': False,
    }, sort_keys=True))
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

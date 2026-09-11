#!/usr/bin/env python3
"""One-command, user-facing LiDAR/PLY to IFC BIM pipeline."""

from __future__ import annotations

import argparse
import os
from pathlib import Path
import subprocess
import sys


HERE = Path(__file__).resolve().parent
DEFAULT_DATASET = Path('/media/sasaki/aiueo/datasets/hilti2022')
DEFAULT_OUTPUT = Path('/media/sasaki/aiueo/lidarslam_work/output/bim_maps')


def resolve_input(value: str, dataset_root: Path) -> tuple[str, Path]:
    """Return ``('ply'|'bag', path)`` from a PLY path or HILTI sequence name."""
    path = Path(value).expanduser()
    if path.is_file() and path.suffix.lower() == '.ply':
        return 'ply', path.resolve()
    if path.is_dir() and (path / 'metadata.yaml').is_file():
        return 'bag', path.resolve()
    sequence = value[:-5] if value.endswith('_ros2') else value
    bag = dataset_root / f'{sequence}_ros2'
    if bag.is_dir() and (bag / 'metadata.yaml').is_file():
        return 'bag', bag
    raise FileNotFoundError(
        f'入力が見つかりません: {value}\n'
        f'PLYファイル、rosbag2ディレクトリ、または exp01 のような名前を指定してください。')


def _run(command: list[str], dry_run: bool) -> None:
    print('  $ ' + ' '.join(command), flush=True)
    if not dry_run:
        subprocess.run(command, check=True)


def _parser() -> argparse.ArgumentParser:
    p = argparse.ArgumentParser(
        description='点群またはHILTI rosbagから、室内BIM (IFC) を簡単に生成します。',
        formatter_class=argparse.ArgumentDefaultsHelpFormatter)
    p.add_argument('input', help='exp01、rosbag2ディレクトリ、または map.ply')
    p.add_argument('-o', '--output-dir', type=Path, default=DEFAULT_OUTPUT,
                   help='成果物の保存先')
    p.add_argument('--dataset-root', type=Path, default=DEFAULT_DATASET,
                   help='HILTIデータセットの場所')
    p.add_argument('--duration', default='full',
                   help='SLAMするbag時間（秒、または full）')
    p.add_argument('--rate', type=float, default=0.5, help='bag再生速度')
    p.add_argument('--force-map', action='store_true',
                   help='既存PLYがあってもSLAMを再実行')
    p.add_argument('--force-ifc', action='store_true',
                   help='既存IFCがあってもBIM抽出を再実行')
    p.add_argument('--max-planes', type=int, default=20, help='最大壁平面数')
    p.add_argument('--corner-snap', type=float, default=0.5,
                   help='壁端を接続する最大距離 [m]')
    p.add_argument('--adaptive-regularize', action='store_true',
                   help='壁ごとにFull/Soft/Keepを品質から自動選択')
    p.add_argument('--repair-walls', action='store_true',
                   help='近接する未接続端の間に推定壁を追加')
    p.add_argument('--repair-gap', type=float, default=0.75,
                   help='追加する推定壁の最大長 [m]')
    p.add_argument('--dry-run', action='store_true',
                   help='処理せず、実行予定のコマンドだけ表示')
    return p


def main(argv=None) -> int:
    args = _parser().parse_args(argv)
    try:
        kind, source = resolve_input(args.input, args.dataset_root.expanduser())
    except FileNotFoundError as exc:
        print(f'エラー: {exc}', file=sys.stderr)
        return 2

    out_dir = args.output_dir.expanduser().resolve()
    name = source.stem.removesuffix('_ros2')
    map_path = source if kind == 'ply' else out_dir / f'{name}_map.ply'
    ifc_path = out_dir / f'{name}_bim.ifc'
    report_path = out_dir / f'{name}_bim_report.html'
    print('=== LiDARSLAM BIM Pipeline ===')
    print(f'入力   : {source}')
    print(f'点群   : {map_path}')
    print(f'BIM    : {ifc_path}')
    print(f'レポート: {report_path}')
    if not args.dry_run:
        out_dir.mkdir(parents=True, exist_ok=True)

    if kind == 'bag':
        if map_path.exists() and not args.force_map:
            print(f'\n[1/2] 既存の点群を再利用します ({map_path.stat().st_size / 1e6:.1f} MB)')
        else:
            print('\n[1/2] rosbagからSLAM地図を生成します')
            env = os.environ.copy()
            env['HILTI_SLAM_OUT'] = str(out_dir)
            command = ['bash', str(HERE / 'bim_reference_scripts' / 'run_hilti_slam.sh'),
                       str(source), name, str(args.duration), str(args.rate)]
            print('  $ ' + ' '.join(command), flush=True)
            if not args.dry_run:
                subprocess.run(command, check=True, env=env)
    else:
        print('\n[1/2] PLY入力のためSLAMは不要です')

    if ifc_path.exists() and report_path.exists() and not args.force_ifc:
        print(f'[2/2] 既存のBIMを再利用します ({ifc_path.stat().st_size / 1e3:.1f} KB)')
    else:
        print('[2/2] 点群から室内BIMを生成します')
        _run([sys.executable, str(HERE / 'bim_export.py'), str(map_path),
              str(ifc_path), '--indoor', '--max-planes', str(args.max_planes),
              '--corner-snap', str(args.corner_snap), '--report',
              str(report_path)] + (['--adaptive-regularize']
                                   if args.adaptive_regularize else []) +
             (['--repair-walls', '--repair-gap',
                                    str(args.repair_gap)] if args.repair_walls else []),
             args.dry_run)

    print('\n完了しました。')
    print(f'IFC: {ifc_path}')
    print(f'レポート: {report_path}')
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

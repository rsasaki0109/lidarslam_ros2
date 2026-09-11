#!/usr/bin/env python3
"""Apply a browser-exported map edit plan without overwriting its source."""

from __future__ import annotations

import argparse
import json
import os
import sys
from pathlib import Path
from typing import Sequence

from map_edit import MapEditError, apply_map_edit


EX_USAGE = 2
EX_SOFTWARE = 70


def parse_args(argv: Sequence[str] | None = None) -> argparse.Namespace:
    """Parse the stable map-edit command."""
    show_all_help = os.environ.get('LIDARSLAM_CLI_HELP_MODE') != 'core'

    def advanced_help(text: str) -> str:
        return text if show_all_help else argparse.SUPPRESS

    command = os.environ.get('LIDARSLAM_CLI_COMMAND', 'lidarslam-map edit')
    parser = argparse.ArgumentParser(
        prog=command,
        description=(
            'Apply an offline 3D edit plan to a new map candidate, then verify the '
            'map-bundle and Autoware contracts.'
        ),
        formatter_class=argparse.RawDescriptionHelpFormatter,
        epilog=(
            'The source map is never modified and the output path must not exist.\n\n'
            'Example:\n'
            f'  {command} output/my_map --plan map-edit-plan.json '
            '--output-dir output/my_map_edited'
        ),
    )
    parser.add_argument('output_dir', help='Completed source map bundle or run directory.')
    parser.add_argument('--help-all', action='help', help='Show backend-replay options.')
    parser.add_argument('--plan', required=True, metavar='<json>', help='Edit plan downloaded by the 3D preview.')
    parser.add_argument('--output-dir', dest='candidate_dir', required=True, metavar='<dir>', help='New candidate map directory; it must not exist.')
    parser.add_argument('--dry-run', action='store_true', help='Validate identities and prerequisites without writing a candidate.')
    replay = parser.add_argument_group('loop-edge replay')
    replay.add_argument('--backend-input', metavar='<rosbag2_dir>', help=advanced_help('Override the editable run backend_input directory.'))
    replay.add_argument('--params', metavar='<yaml>', help=advanced_help('Override the editable run graph_params.ros.yaml.'))
    replay.add_argument('--setup', metavar='<setup.bash>', help=advanced_help('Workspace setup override; an active ROS environment is accepted.'))
    parser.add_argument('--json', action='store_true', help=advanced_help('Print the receipt as JSON.'))
    return parser.parse_args(argv)


def main(argv: Sequence[str] | None = None) -> int:
    """Apply the plan and report its immutable receipt."""
    args = parse_args(argv)
    try:
        receipt = apply_map_edit(
            source_dir=Path(args.output_dir),
            plan_path=Path(args.plan),
            output_dir=Path(args.candidate_dir),
            backend_input=Path(args.backend_input).expanduser().resolve() if args.backend_input else None,
            params_path=Path(args.params).expanduser().resolve() if args.params else None,
            setup_path=Path(args.setup).expanduser().resolve() if args.setup else None,
            dry_run=bool(args.dry_run),
        )
    except MapEditError as exc:
        print(f'error: {exc}', file=sys.stderr)
        return EX_USAGE
    except OSError as exc:
        print(f'error: map edit failed: {exc}', file=sys.stderr)
        return EX_SOFTWARE
    if args.json:
        print(json.dumps(receipt, indent=2, sort_keys=True))
    elif receipt['status'] == 'DRY_RUN':
        print('Map edit plan is ready to apply')
        print(f"  operations: {receipt['operation_count']}")
        print(f"  backend replay: {'required' if receipt['requires_backend_replay'] else 'not required'}")
        print(f"  candidate: {receipt['output_dir']}")
    else:
        point_edit = receipt['point_edit']
        print('Edited map candidate verified')
        print(f"  candidate: {receipt['output_dir']}")
        print(f"  removed points: {point_edit.get('tile_points_removed', 0)}")
        print(f"  loop replay: {'performed' if receipt['loop_replay']['performed'] else 'not needed'}")
        print(f"  receipt: {Path(receipt['output_dir']) / 'map_edit_receipt.json'}")
        print(f"  next: lidarslam-map view {receipt['output_dir']}")
    return 0


if __name__ == '__main__':
    raise SystemExit(main())

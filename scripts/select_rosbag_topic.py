#!/usr/bin/env python3
"""Select a rosbag topic deterministically by type and preference."""

from __future__ import annotations

import argparse
import sys
from pathlib import Path

from rosbags.highlevel import AnyReader
from rosbags.typesys import Stores, get_types_from_msg, get_typestore


def select_topic(
    candidates: list[tuple[str, int]],
    preferred_substring: str = '',
) -> str:
    """Prefer a named sensor role, then message count, then topic name."""
    if not candidates:
        return ''
    ranked = sorted(
        candidates,
        key=lambda candidate: (
            0 if preferred_substring in candidate[0] else 1,
            -candidate[1],
            candidate[0],
        ),
    )
    return ranked[0][0]


def main() -> int:
    """Run the topic selector CLI."""
    parser = argparse.ArgumentParser(description=__doc__)
    parser.add_argument('--bag', required=True, type=Path)
    parser.add_argument('--msg-type', required=True)
    parser.add_argument('--extra-msg-dir', type=Path)
    parser.add_argument('--preferred-substring', default='')
    args = parser.parse_args()

    typestore = get_typestore(Stores.LATEST)
    if args.extra_msg_dir is not None:
        package_name = args.extra_msg_dir.parent.name
        for path in sorted(args.extra_msg_dir.glob('*.msg')):
            text = path.read_text(encoding='utf-8')
            typestore.register(
                get_types_from_msg(
                    text,
                    f'{package_name}/msg/{path.stem}',
                ),
            )

    with AnyReader(
        [args.bag.expanduser().resolve()],
        default_typestore=typestore,
    ) as reader:
        candidates = [
            (connection.topic, getattr(connection, 'msgcount', 0))
            for connection in reader.connections
            if connection.msgtype == args.msg_type
        ]

    selected = select_topic(candidates, args.preferred_substring)
    if len(candidates) > 1:
        details = ', '.join(
            f'{topic} ({count})' for topic, count in sorted(candidates)
        )
        print(
            f'warning: multiple {args.msg_type} topics found; '
            f'selected {selected}; candidates: {details}',
            file=sys.stderr,
        )
    if selected:
        print(selected)
        return 0
    return 1


if __name__ == '__main__':
    raise SystemExit(main())

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

"""Synthetic tests for the M6a10 synchronized-tail input contract."""

from __future__ import annotations

from pathlib import Path
import sys
from types import SimpleNamespace

REPO_ROOT = Path(__file__).resolve().parents[2]
sys.path.insert(0, str(REPO_ROOT / 'scripts'))

from analyze_m6a10_synchronized_tail import (  # noqa: E402
    evaluate_synchronized_tail,
    summarize_lidar_message,
    SynchronizedTailError,
    validate_pointcloud_schema,
)

import pytest  # noqa: E402


def _message(header_ns: int, values: list[int], *, datatype: int = 6,
             bigendian: bool = False, field_name: str = 't') -> SimpleNamespace:
    width = len(values)
    data = bytearray(width * 4)
    for index, value in enumerate(values):
        data[index * 4:(index + 1) * 4] = int(value).to_bytes(4, 'little')
    return SimpleNamespace(
        header=SimpleNamespace(
            stamp=SimpleNamespace(
                sec=header_ns // 1_000_000_000,
                nanosec=header_ns % 1_000_000_000,
            ),
        ),
        height=1,
        width=width,
        fields=[SimpleNamespace(name=field_name, offset=0, datatype=datatype, count=1)],
        is_bigendian=bigendian,
        point_step=4,
        row_step=width * 4,
        data=bytes(data),
    )


def _row(index: int, header_ns: int, point_max_offset_ns: int) -> dict:
    message = _message(header_ns, [0, point_max_offset_ns])
    return summarize_lidar_message(
        message,
        header_ns,
        serialized_payload=f'payload-{index}'.encode(),
        source_index=index,
    )


def test_zero_trim_requires_point_max_strictly_before_last_imu():
    rows = [_row(0, 1_000_000_000, 10), _row(1, 2_000_000_000, 0)]

    result = evaluate_synchronized_tail(
        rows,
        last_imu_header_timestamp_ns=2_000_000_001,
    )

    assert result['status'] == 'PASS'
    assert result['eligible_lidar_count'] == 2
    assert result['ineligible_lidar_count'] == 0
    assert result['terminal_lidar'] == []
    assert result['comparison_is_strict'] is True


def test_point_max_equal_to_last_imu_is_ineligible():
    rows = [_row(0, 1_000_000_000, 10), _row(1, 2_000_000_000, 0)]

    result = evaluate_synchronized_tail(
        rows,
        last_imu_header_timestamp_ns=2_000_000_000,
    )

    assert result['eligible_lidar_count'] == 1
    assert result['ineligible_lidar_count'] == 1


def test_one_trim_reports_terminal_min_max_and_cutoff():
    rows = [_row(0, 1_000_000_000, 10), _row(1, 2_000_000_000, 100)]

    result = evaluate_synchronized_tail(
        rows,
        last_imu_header_timestamp_ns=2_000_000_050,
    )

    assert result['status'] == 'PASS'
    assert result['eligible_lidar_count'] == 1
    assert result['ineligible_lidar_count'] == 1
    assert result['terminal_lidar'][0]['point_timestamp_min_ns'] == 2_000_000_000
    assert result['terminal_lidar'][0]['point_timestamp_max_ns'] == 2_000_000_100
    assert result['proposed_cutoff_timestamp_ns'] == 1_000_000_010


def test_multiple_terminal_scans_are_all_reported():
    rows = [_row(0, 1_000_000_000, 0), _row(1, 2_000_000_000, 10), _row(2, 3_000_000_000, 20)]

    result = evaluate_synchronized_tail(rows, last_imu_header_timestamp_ns=2_000_000_000)

    assert result['eligible_lidar_count'] == 1
    assert result['ineligible_lidar_count'] == 2
    assert [row['message_index'] for row in result['terminal_lidar']] == [1, 2]


def test_input_order_does_not_change_tail_decision_or_receipt_rows():
    rows = [_row(0, 3_000_000_000, 5), _row(1, 1_000_000_000, 5), _row(2, 2_000_000_000, 5)]

    ordered = evaluate_synchronized_tail(rows, last_imu_header_timestamp_ns=2_000_000_005)
    shuffled = evaluate_synchronized_tail(
        list(reversed(rows)), last_imu_header_timestamp_ns=2_000_000_005)

    assert ordered == shuffled


def test_nonterminal_ineligible_scan_fails_closed():
    rows = [_row(0, 1_000_000_000, 0), _row(1, 2_000_000_000, 100), _row(2, 2_000_000_020, 0)]

    with pytest.raises(SynchronizedTailError, match='followed by an eligible'):
        evaluate_synchronized_tail(rows, last_imu_header_timestamp_ns=2_000_000_050)


@pytest.mark.parametrize(
    ('kwargs', 'code'),
    [
        ({'datatype': 7}, 'unsupported_point_timestamp_schema'),
        ({'bigendian': True}, 'unsupported_pointcloud_endianness'),
        ({'field_name': 'timestamp'}, 'ambiguous_point_timestamp'),
    ],
)
def test_unsupported_pointcloud_schema_fails_closed(kwargs: dict, code: str):
    with pytest.raises(SynchronizedTailError) as exc_info:
        validate_pointcloud_schema(_message(1_000_000_000, [0], **kwargs))
    assert exc_info.value.code == code


def test_raw_point_timestamp_schema_is_explicitly_uint32_nanoseconds():
    schema = validate_pointcloud_schema(_message(1_000_000_000, [0, 100]))

    assert schema['field_name'] == 't'
    assert schema['datatype_name'] == 'UINT32'
    assert schema['unit'] == 'nanoseconds'
    assert schema['relative_to'] == 'header.stamp'
    assert schema['endianness'] == 'little'

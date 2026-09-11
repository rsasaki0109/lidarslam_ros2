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

"""Single-inflight ROS1 feeder for the FAST-LIVO2 M6a10 phase.

The feeder is benchmark-only.  It reads one serialized rosbag message at a
time, publishes it, waits for the patched mapper's callback counter to
advance by exactly one, and then acknowledges that observation.  It never
uses publisher counts, log text, or a ground-truth path as evidence.
"""

from __future__ import annotations

import argparse
import hashlib
import json
import os
from pathlib import Path
import signal
import socket
import sys
import threading
import time
from typing import Any, Iterable

# ROS1 is an execution-only dependency.  Keeping it out of module import
# time lets clean installed-package audits inspect all benchmark modules on a
# host that intentionally has no ROS1 overlay.
rosbag = None
rospy = None
Trigger = None
ROSBagException = Exception


def _require_ros1() -> None:
    """Load ROS1 only when the feeder is actually asked to run."""
    global rosbag, rospy, Trigger, ROSBagException
    if rosbag is not None and rospy is not None and Trigger is not None:
        return
    try:
        import rosbag as rosbag_module
        import rospy as rospy_module
        from std_srvs.srv import Trigger as trigger_type
    except ImportError as error:
        raise RuntimeError(
            'fast_livo2_m6a10_feeder requires ROS1 rosbag, rospy, and '
            'std_srvs at execution time') from error
    rosbag = rosbag_module
    rospy = rospy_module
    Trigger = trigger_type
    ROSBagException = getattr(rosbag_module, 'ROSBagException', Exception)


TOPICS = (
    '/os1_cloud_node1/points',
    '/imu/imu',
    '/left/image_raw',
)
TOPIC_KEYS = {
    TOPICS[0]: 'lidar',
    TOPICS[1]: 'imu',
    TOPICS[2]: 'image',
}
STATUS_SERVICE = '/m6a10/consumer_status'
ACK_SERVICE = '/m6a10/consumer_ack'
EOF_SERVICE = '/m6a10/consumer_eof'


class DeadlineExceeded(TimeoutError):
    """A bounded feeder operation exceeded its absolute deadline."""


class Deadline:
    """Monotonic absolute deadline shared by one publish/ack transaction."""

    def __init__(self, timeout: float) -> None:
        if timeout <= 0:
            raise ValueError('deadline timeout must be positive')
        self.end = time.monotonic() + float(timeout)

    @classmethod
    def from_end(cls, end: float) -> 'Deadline':
        value = cls.__new__(cls)
        value.end = float(end)
        return value

    def remaining(self, operation: str = 'operation') -> float:
        value = self.end - time.monotonic()
        if value <= 0:
            raise DeadlineExceeded(f'{operation} deadline expired')
        return value


def _bounded_call(call: Any, deadline: Deadline, operation: str) -> Any:
    """Call a ROS/transport operation without an unbounded blocking call.

    ``rospy.ServiceProxy`` has no per-call timeout.  A daemon thread would
    leave a stuck transport behind after timeout, so the feeder uses the
    process-wide real-time timer on its single main thread.  A real socket
    timeout is also installed when the proxy exposes its transport.  No
    worker thread/process is created and every timeout path returns through
    the same deterministic exception.
    """
    remaining = deadline.remaining(operation)
    if threading.current_thread() is not threading.main_thread():
        # Python signal timers are only delivered to the main thread.  The
        # production feeder is single-threaded; reject an accidental caller
        # rather than silently reintroducing an unbounded worker.
        raise RuntimeError(f'{operation} must run on the main thread')
    previous_handler = signal.getsignal(signal.SIGALRM)
    previous_timer = signal.getitimer(signal.ITIMER_REAL)
    transport_socket = None
    transport = getattr(call, 'transport', None)
    if transport is not None:
        transport_socket = getattr(transport, 'sock', None)
        if transport_socket is None:
            transport_socket = getattr(transport, 'socket', None)
    previous_socket_timeout = None
    if transport_socket is not None and hasattr(transport_socket, 'gettimeout'):
        try:
            previous_socket_timeout = transport_socket.gettimeout()
            transport_socket.settimeout(remaining)
        except (OSError, AttributeError):
            transport_socket = None

    def _alarm_handler(_signum: int, _frame: Any) -> None:
        raise DeadlineExceeded(f'{operation} deadline expired')

    try:
        if previous_timer[0] > 0:
            # Nested alarms are unsafe to restore precisely.  Refuse this
            # unusual embedding rather than stealing another caller's timer.
            raise RuntimeError('an external SIGALRM timer is already active')
        signal.signal(signal.SIGALRM, _alarm_handler)
        signal.setitimer(signal.ITIMER_REAL, remaining)
        try:
            return call()
        except (socket.timeout, TimeoutError) as error:
            raise DeadlineExceeded(f'{operation} deadline expired') from error
    finally:
        signal.setitimer(signal.ITIMER_REAL, 0.0)
        signal.signal(signal.SIGALRM, previous_handler)
        if transport_socket is not None and previous_socket_timeout is not None:
            try:
                transport_socket.settimeout(previous_socket_timeout)
            except OSError:
                pass


def _is_relative_to(path: Path, parent: Path) -> bool:
    """Return whether *path* is below *parent* on Python 3.8+.

    ``Path.is_relative_to`` was added in Python 3.9.  The pinned ROS Noetic
    image still runs Python 3.8, so keep the security check on the common
    ``relative_to`` API instead of relying on a newer interpreter method.
    Both arguments are resolved by the callers before this helper is used.
    """
    try:
        path.relative_to(parent)
    except ValueError:
        return False
    return True


def atomic_json(path: Path, payload: dict[str, Any]) -> None:
    """Publish a new JSON file without replacing an existing evidence file."""
    path = path.resolve()
    if path.exists() or path.is_symlink():
        raise RuntimeError(f'evidence already exists: {path}')
    part = path.with_name(path.name + '.part')
    if part.exists() or part.is_symlink():
        raise RuntimeError(f'stale evidence staging file exists: {part}')
    path.parent.mkdir(parents=True, exist_ok=True)
    encoded = (json.dumps(payload, sort_keys=True, separators=(',', ':')) + '\n')
    part.write_text(encoded, encoding='utf-8')
    part.replace(path)


def atomic_progress_json(path: Path, payload: dict[str, Any]) -> None:
    """Atomically replace a diagnostic checkpoint.

    Progress is deliberately separate from the authoritative feeder receipt:
    it is allowed to be replaced as the feeder advances and is never used to
    declare completion.  A unique same-directory temporary keeps readers from
    observing a partial JSON document and avoids leaving a reusable ``.part``
    after a successful update.
    """
    path = path.resolve()
    if path.is_symlink():
        raise RuntimeError(f'progress path is a symlink: {path}')
    path.parent.mkdir(parents=True, exist_ok=True)
    temporary = path.with_name(
        f'.{path.name}.{os.getpid()}.{time.monotonic_ns()}.part')
    encoded = (json.dumps(payload, sort_keys=True, separators=(',', ':')) + '\n')
    try:
        with temporary.open('x', encoding='utf-8') as stream:
            stream.write(encoded)
            stream.flush()
            os.fsync(stream.fileno())
        os.replace(temporary, path)
    finally:
        try:
            temporary.unlink()
        except FileNotFoundError:
            pass


def progress_path(output: Path) -> Path:
    configured = os.environ.get('M6A10_FAST_PROGRESS_PATH')
    path = (Path(configured).resolve() if configured else output.with_name(
        'feeder_progress.json').resolve())
    if not _is_relative_to(path, output.resolve().parent):
        raise RuntimeError('progress path must remain inside the output directory')
    return path


def drain_progress_path(output: Path) -> Path:
    configured = os.environ.get('M6A10_FAST_DRAIN_PROGRESS_PATH')
    path = (Path(configured).resolve() if configured else output.with_name(
        'drain_progress.json').resolve())
    if not _is_relative_to(path, output.resolve().parent):
        raise RuntimeError('drain progress path must remain inside output directory')
    return path


def write_progress(path: Path, *, phase: str, records: int,
                   published: dict[str, int], acked: dict[str, int],
                   first_ack_at: str | None = None,
                   last_ack_at: str | None = None,
                   last_topic: str | None = None,
                   last_bag_time: str | None = None,
                   progress_write_failures: int = 0,
                   inflight_topic: str | None = None,
                   publish_sequence: int | None = None,
                   publish_call_started_at: str | None = None,
                   publish_returned_at: str | None = None) -> int:
    """Write non-authoritative feeder progress and keep the run alive on I/O errors."""
    payload = {
        'schema_version': 1,
        'contract_id': 'm6a10-v2c-fast-livo2-feeder-progress-v1',
        'authoritative_completion': False,
        'phase': phase,
        'records': records,
        'published_topic_counts': dict(published),
        'acked_topic_counts': dict(acked),
        'first_ack_observed': first_ack_at is not None,
        'first_ack_at': first_ack_at,
        'last_ack_at': last_ack_at,
        'last_topic': last_topic,
        'last_bag_time': last_bag_time,
        # These markers are diagnostic only.  In particular, ``post_publish``
        # proves that Publisher.publish returned; it does not claim callback
        # processing or ACK completion.
        'inflight_topic': inflight_topic,
        'publish_sequence': publish_sequence,
        'publish_call_started_at': publish_call_started_at,
        'publish_returned_at': publish_returned_at,
        'updated_at_monotonic_ns': time.monotonic_ns(),
        'progress_write_failures': progress_write_failures,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    }
    try:
        atomic_progress_json(path, payload)
    except (OSError, RuntimeError) as error:
        progress_write_failures += 1
        print(f'FAST progress checkpoint unavailable: {error}',
              file=sys.stderr, flush=True)
    return progress_write_failures


def sha256_file(path: Path) -> str:
    digest = hashlib.sha256()
    with path.open('rb') as stream:
        for block in iter(lambda: stream.read(1024 * 1024), b''):
            digest.update(block)
    return digest.hexdigest()


def _status(proxy: Any, deadline: Deadline) -> dict[str, Any]:
    response = _bounded_call(proxy, deadline, 'consumer status RPC')
    if not response.success:
        raise RuntimeError(f'consumer status rejected: {response.message}')
    try:
        value = json.loads(response.message)
    except (TypeError, json.JSONDecodeError) as error:
        raise RuntimeError('consumer status was not JSON') from error
    if value.get('schema_version') != 2 or value.get('system') != 'fast_livo2':
        raise RuntimeError('consumer status schema/system mismatch')
    consumer = value.get('consumer')
    if not isinstance(consumer, dict):
        raise RuntimeError('consumer status lacks consumer object')
    required = (
        'received_messages', 'received_topic_counts', 'dropped_messages',
        'processing_failures', 'queue_overflow', 'backlog_at_drain',
        'maximum_backlog_messages', 'maximum_callback_latency_seconds',
    )
    if any(key not in consumer for key in required):
        raise RuntimeError('consumer status lacks required counters')
    return value


def _counts(status: dict[str, Any]) -> tuple[int, dict[str, int]]:
    consumer = status['consumer']
    topic_counts = consumer['received_topic_counts']
    counts = {key: int(topic_counts.get(key, -1)) for key in TOPIC_KEYS.values()}
    return int(consumer['received_messages']), counts


def wait_for_services(deadline: Deadline | float) -> tuple[Any, Any, Any]:
    _require_ros1()
    if not isinstance(deadline, Deadline):
        deadline = Deadline(float(deadline))
    for name in (STATUS_SERVICE, ACK_SERVICE, EOF_SERVICE):
        try:
            rospy.wait_for_service(name, timeout=deadline.remaining(
                f'{name} availability'))
        except (rospy.ROSException, rospy.ROSInterruptException) as error:
            raise DeadlineExceeded(f'{name} availability deadline expired') from error
    return (rospy.ServiceProxy(STATUS_SERVICE, Trigger),
            rospy.ServiceProxy(ACK_SERVICE, Trigger),
            rospy.ServiceProxy(EOF_SERVICE, Trigger))


def wait_for_callback(status_proxy: Any, topic: str, before_total: int,
                      before_counts: dict[str, int], deadline: Deadline
                      ) -> tuple[dict[str, Any], float]:
    key = TOPIC_KEYS[topic]
    started = time.monotonic()
    while not rospy.is_shutdown():
        deadline.remaining(f'callback status polling for {topic}')
        status = _status(status_proxy, deadline)
        total, counts = _counts(status)
        if total == before_total + 1 and counts[key] == before_counts[key] + 1:
            if any(counts[name] != before_counts[name] for name in counts if name != key):
                raise RuntimeError(
                    'unexpected cross-topic callback during single-inflight publish')
            return status, time.monotonic() - started
        if total > before_total + 1 or counts[key] > before_counts[key] + 1:
            raise RuntimeError('consumer advanced by more than one message')
        time.sleep(0.002)
    raise DeadlineExceeded(f'callback ACK timeout for {topic}')


def _publisher(topic: str, message: Any, publishers: dict[str, Any],
               deadline: Deadline | float) -> Any:
    if not isinstance(deadline, Deadline):
        deadline = Deadline(float(deadline))
    publisher = publishers.get(topic)
    if publisher is None:
        publisher = rospy.Publisher(topic, type(message), queue_size=1, latch=False)
        publishers[topic] = publisher
    def _connections() -> int:
        return int(_bounded_call(publisher.get_num_connections, deadline,
                                 f'publisher connection for {topic}'))
    while _connections() < 1:
        deadline.remaining(f'publisher connection for {topic}')
        if rospy.is_shutdown():
            raise RuntimeError('ROS shutdown while waiting for mapper subscriber')
        time.sleep(0.01)
    if _connections() < 1:
        raise DeadlineExceeded(f'no subscriber connection for {topic}')
    return publisher


def publish_one_inflight(bag_path: Path, output: Path, expected: dict[str, int],
                         timeout: float) -> dict[str, Any]:
    _require_ros1()
    if hasattr(sys.stdout, 'reconfigure'):
        sys.stdout.reconfigure(line_buffering=True)
        sys.stderr.reconfigure(line_buffering=True)
    rospy.init_node('m6a10_fast_livo2_feeder', anonymous=True, disable_signals=True)
    status_proxy, ack_proxy, _ = wait_for_services(Deadline(timeout))
    checkpoint = progress_path(output)
    published = {key: 0 for key in TOPIC_KEYS.values()}
    acked = {key: 0 for key in TOPIC_KEYS.values()}
    publishers: dict[str, Any] = {}
    ack_latencies: list[float] = []
    records = 0
    progress_write_failures = write_progress(
        checkpoint, phase='services_ready', records=records,
        published=published, acked=acked)
    first_ack_at = None
    last_ack_at = None
    last_topic = None
    last_bag_time = None
    progress_every = max(1, int(os.environ.get('M6A10_FAST_PROGRESS_EVERY', '100')))
    with rosbag.Bag(str(bag_path), 'r') as bag:
        messages: Iterable[tuple[str, Any, Any]] = bag.read_messages(topics=list(TOPICS))
        for topic, message, bag_time in messages:
            if topic not in TOPIC_KEYS:
                raise RuntimeError(f'unexpected feeder topic: {topic}')
            key = TOPIC_KEYS[topic]
            if published[key] >= expected[key]:
                raise RuntimeError(f'{key} published more than preregistered count')
            last_topic = topic
            last_bag_time = str(bag_time)
            if records == 0:
                progress_write_failures = write_progress(
                    checkpoint, phase='publish_waiting_for_callback', records=records,
                    published=published, acked=acked, first_ack_at=first_ack_at,
                    last_ack_at=last_ack_at, last_topic=last_topic,
                    last_bag_time=last_bag_time,
                    progress_write_failures=progress_write_failures)
                print(
                    f'FAST feeder first publish topic={topic}', flush=True)
            operation_deadline = Deadline(timeout)
            before = _status(status_proxy, operation_deadline)
            before_total, before_counts = _counts(before)
            publisher = _publisher(topic, message, publishers, operation_deadline)
            publish_started = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
            marker_checkpoint = (records == 0 or
                                 (records + 1) % progress_every == 0)
            if marker_checkpoint:
                progress_write_failures = write_progress(
                    checkpoint, phase='pre_publish', records=records,
                    published=published, acked=acked,
                    first_ack_at=first_ack_at, last_ack_at=last_ack_at,
                    last_topic=last_topic, last_bag_time=last_bag_time,
                    progress_write_failures=progress_write_failures,
                    inflight_topic=topic, publish_sequence=records + 1,
                    publish_call_started_at=publish_started)
            _bounded_call(lambda: publisher.publish(message), operation_deadline,
                          f'publish {topic}')
            publish_returned = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
            if marker_checkpoint:
                progress_write_failures = write_progress(
                    checkpoint, phase='post_publish', records=records,
                    published=published, acked=acked,
                    first_ack_at=first_ack_at, last_ack_at=last_ack_at,
                    last_topic=last_topic, last_bag_time=last_bag_time,
                    progress_write_failures=progress_write_failures,
                    inflight_topic=topic, publish_sequence=records + 1,
                    publish_call_started_at=publish_started,
                    publish_returned_at=publish_returned)
            after, callback_latency = wait_for_callback(
                status_proxy, topic, before_total, before_counts,
                operation_deadline)
            # Keep the Trigger call itself as ``ack_proxy()`` inside the
            # bounded wrapper; no unbounded direct service invocation exists.
            ack_response = _bounded_call(lambda: ack_proxy(), operation_deadline,
                                         f'consumer ACK RPC for {topic}')
            if not ack_response.success:
                raise RuntimeError(f'consumer ACK rejected for {topic}: {ack_response.message}')
            published[key] += 1
            acked[key] += 1
            ack_latencies.append(callback_latency)
            records += 1
            now = time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime())
            first_ack_at = first_ack_at or now
            last_ack_at = now
            if records == 1 or records % progress_every == 0:
                progress_write_failures = write_progress(
                    checkpoint, phase='callback_acknowledged', records=records,
                    published=published, acked=acked,
                    first_ack_at=first_ack_at, last_ack_at=last_ack_at,
                    last_topic=last_topic, last_bag_time=last_bag_time,
                    progress_write_failures=progress_write_failures)
                print(
                    f'FAST feeder ACK record={records} topic={topic} '
                    f'latency={callback_latency:.6f}s', flush=True)
            if records != sum(published.values()):
                raise RuntimeError('feeder record accounting mismatch')
            if after['consumer']['dropped_messages'] != 0:
                raise RuntimeError('consumer reported a dropped message')
            if after['consumer']['processing_failures'] != 0:
                raise RuntimeError('consumer reported a processing failure')
    final = _status(status_proxy, Deadline(timeout))
    total, counts = _counts(final)
    if total != sum(expected.values()) or counts != expected or published != expected:
        raise RuntimeError(
            f'feeder count mismatch expected={expected} published={published} observed={counts}')
    if acked != expected:
        raise RuntimeError(f'feeder ACK mismatch expected={expected} acked={acked}')
    write_progress(
        checkpoint, phase='authoritative_receipt_pending', records=records,
        published=published, acked=acked, first_ack_at=first_ack_at,
        last_ack_at=last_ack_at, last_topic=last_topic,
        last_bag_time=last_bag_time,
        progress_write_failures=progress_write_failures)
    report = {
        'schema_version': 1,
        'contract_id': 'm6a10-v2c-fast-livo2-single-inflight-feeder-v1',
        'status': 'pass',
        'bag_path': str(bag_path),
        'bag_bytes': bag_path.stat().st_size,
        'bag_sha256': sha256_file(bag_path),
        'topics': list(TOPICS),
        'expected_topic_counts': expected,
        'published_topic_counts': published,
        'acked_topic_counts': acked,
        'published_messages': records,
        'single_inflight': True,
        'publisher_queue_size': 1,
        'ack_source_kind': 'consumer_status_callback_counter',
        'ack_backpressure_verified': True,
        'max_callback_latency_seconds': max(ack_latencies, default=0.0),
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
        'consumer_status': final,
        'bag_time_last_seen': str(bag_time),
    }
    atomic_json(output, report)
    return report


def wait_for_drain(output: Path, timeout: float,
                   eof_already_observed: bool = False) -> dict[str, Any]:
    _require_ros1()
    rospy.init_node('m6a10_fast_livo2_drainer', anonymous=True, disable_signals=True)
    deadline = Deadline(timeout)
    status_proxy, _, eof_proxy = wait_for_services(deadline)
    checkpoint = drain_progress_path(output)
    write_progress(checkpoint, phase='drain_services_ready', records=0,
                   published={key: 0 for key in TOPIC_KEYS.values()},
                   acked={key: 0 for key in TOPIC_KEYS.values()})
    if not eof_already_observed:
        eof_response = _bounded_call(eof_proxy, deadline, 'consumer EOF RPC')
        if not eof_response.success:
            raise RuntimeError(f'consumer EOF rejected: {eof_response.message}')
    latest: dict[str, Any] | None = None
    while not rospy.is_shutdown():
        deadline.remaining('consumer drain status polling')
        latest = _status(status_proxy, deadline)
        consumer = latest['consumer']
        if int(consumer['backlog_at_drain']) == 0:
            break
        time.sleep(0.05)
    if latest is None or int(latest['consumer']['backlog_at_drain']) != 0:
        raise DeadlineExceeded('consumer internal deque did not drain to zero')
    write_progress(checkpoint, phase='drain_complete', records=0,
                   published={key: 0 for key in TOPIC_KEYS.values()},
                   acked={key: 0 for key in TOPIC_KEYS.values()})
    atomic_json(output, {
        'schema_version': 1,
        'contract_id': 'm6a10-v2c-fast-livo2-drain-v1',
        'status': 'pass',
        'eof_service': EOF_SERVICE,
        'drain_complete': True,
        'backlog_at_drain': 0,
        'consumer_status': latest,
        'ground_truth_content_opened': False,
        'scorer_invoked': False,
    })
    return latest


def parse_args() -> argparse.Namespace:
    parser = argparse.ArgumentParser()
    parser.add_argument('--bag', type=Path, required=True)
    parser.add_argument('--output', type=Path, required=True)
    parser.add_argument('--drain', action='store_true')
    parser.add_argument('--eof-already-observed', action='store_true')
    parser.add_argument('--timeout', type=float, default=30.0)
    parser.add_argument('--expected-lidar', type=int, default=5793)
    parser.add_argument('--expected-imu', type=int, default=225102)
    parser.add_argument('--expected-image', type=int, default=5792)
    return parser.parse_args()


def write_failure_diagnostic(output: Path, error: BaseException) -> None:
    """Write one immutable timeout/failure diagnostic, if possible."""
    try:
        output = output.resolve()
        progress = output / 'feeder_progress.json'
        payload = {
            'schema_version': 1,
            'contract_id': 'm6a10-v2c-fast-livo2-feeder-failure-v1',
            'status': 'FAIL_CLOSED',
            'failure_type': type(error).__name__,
            'failure': str(error),
            'progress_path': str(progress),
            'progress_sha256': sha256_file(progress) if progress.is_file() else None,
            'ground_truth_content_opened': False,
            'scorer_invoked': False,
            'created_at_utc': time.strftime('%Y-%m-%dT%H:%M:%SZ', time.gmtime()),
        }
        atomic_json(output / 'feeder_failure.json', payload)
    except (OSError, RuntimeError, ValueError):
        # The primary exception remains authoritative; diagnostics never turn
        # a deterministic fail-closed result into a second failure mode.
        pass


def main() -> int:
    args = parse_args()
    if args.timeout <= 0:
        raise ValueError('--timeout must be positive')
    expected = {
        'lidar': args.expected_lidar,
        'imu': args.expected_imu,
        'image': args.expected_image,
    }
    if args.drain:
        wait_for_drain(args.output, args.timeout, args.eof_already_observed)
    else:
        publish_one_inflight(args.bag.resolve(), args.output.resolve(), expected, args.timeout)
    return 0


if __name__ == '__main__':
    try:
        raise SystemExit(main())
    except (OSError, RuntimeError, TimeoutError, ValueError, ROSBagException) as error:
        write_failure_diagnostic(Path(sys.argv[sys.argv.index('--output') + 1])
                                 if '--output' in sys.argv and
                                 sys.argv.index('--output') + 1 < len(sys.argv)
                                 else Path('.'), error)
        print(f'FAST M6a10 feeder error: {error}', flush=True)
        raise SystemExit(2)

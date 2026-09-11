#!/usr/bin/env bash
set -euo pipefail

usage() {
  cat >&2 <<'EOF'
Usage:
  wait_for_offline_output_subscribers.sh \
    --odom-topic <topic> --deskewed-topic <topic> \
    --min-odom <count> --min-deskewed <count> \
    --timeout-secs <seconds> --settle-polls <count> -- <command> [args...]

Wait until the command's required ROS output consumers are connected, then run
the command. A zero exit emits an explicit offline-completion marker.
EOF
}

fail() {
  echo "[offline-subscriber-barrier-invalid] $1" >&2
  exit 2
}

require_value() {
  local option="$1"
  local value="${2:-}"
  [[ -n "$value" && "$value" != --* ]] || fail "$option requires a value"
}

ODOM_TOPIC=""
DESKEWED_TOPIC=""
MIN_ODOM=1
MIN_DESKEWED=1
TIMEOUT_SECS=30
SETTLE_POLLS=3
# A fresh no-daemon ROS CLI participant needs enough time to discover every
# subscriber. 0.1 seconds was reliable for a single local node but missed
# subscriptions once the full launch graph was active.
DISCOVERY_SPIN_SECS=1.0

while [[ $# -gt 0 ]]; do
  case "$1" in
    --odom-topic)
      require_value "$1" "${2:-}"
      ODOM_TOPIC="$2"
      shift 2
      ;;
    --deskewed-topic)
      require_value "$1" "${2:-}"
      DESKEWED_TOPIC="$2"
      shift 2
      ;;
    --min-odom)
      require_value "$1" "${2:-}"
      MIN_ODOM="$2"
      shift 2
      ;;
    --min-deskewed)
      require_value "$1" "${2:-}"
      MIN_DESKEWED="$2"
      shift 2
      ;;
    --timeout-secs)
      require_value "$1" "${2:-}"
      TIMEOUT_SECS="$2"
      shift 2
      ;;
    --settle-polls)
      require_value "$1" "${2:-}"
      SETTLE_POLLS="$2"
      shift 2
      ;;
    --)
      shift
      break
      ;;
    --help|-h)
      usage
      exit 0
      ;;
    *)
      fail "unknown option: $1"
      ;;
  esac
done

[[ -n "$ODOM_TOPIC" ]] || fail "--odom-topic is required"
[[ -n "$DESKEWED_TOPIC" ]] || fail "--deskewed-topic is required"
[[ "$MIN_ODOM" =~ ^[0-9]+$ ]] || fail "--min-odom must be a non-negative integer"
[[ "$MIN_DESKEWED" =~ ^[0-9]+$ ]] || fail "--min-deskewed must be a non-negative integer"
[[ "$TIMEOUT_SECS" =~ ^[1-9][0-9]*$ ]] || fail "--timeout-secs must be a positive integer"
[[ "$SETTLE_POLLS" =~ ^[1-9][0-9]*$ ]] || fail "--settle-polls must be a positive integer"
[[ $# -gt 0 ]] || fail "a command is required after --"
command -v ros2 >/dev/null 2>&1 || fail "ros2 is not available"

subscription_count() {
  local topic="$1"
  local output
  output=$(
    ros2 topic info --no-daemon --spin-time "$DISCOVERY_SPIN_SECS" \
      "$topic" 2>/dev/null || true
  )
  awk '/^Subscription count:/ {print $3; found=1; exit} END {if (!found) print 0}' <<<"$output"
}

echo "Waiting for offline output subscribers before reading the bag: ${ODOM_TOPIC} >= ${MIN_ODOM}, ${DESKEWED_TOPIC} >= ${MIN_DESKEWED}."
deadline=$((SECONDS + TIMEOUT_SECS))
ready_polls=0
odom_count=0
deskewed_count=0
while (( SECONDS < deadline )); do
  odom_count=$(subscription_count "$ODOM_TOPIC")
  deskewed_count=$(subscription_count "$DESKEWED_TOPIC")
  if (( odom_count >= MIN_ODOM && deskewed_count >= MIN_DESKEWED )); then
    ready_polls=$((ready_polls + 1))
    if (( ready_polls >= SETTLE_POLLS )); then
      echo "Offline output subscribers ready: odometry=${odom_count}, deskewed scan=${deskewed_count}. Starting bag processing."
      set +e
      "$@"
      command_exit=$?
      set -e
      if (( command_exit == 0 )); then
        echo "RKO LIO offline processing complete (exit=0)."
      else
        echo "[offline-processing-failed] RKO LIO offline process exited ${command_exit}." >&2
      fi
      exit "$command_exit"
    fi
  else
    ready_polls=0
  fi
  sleep 0.1
done

echo "[offline-subscriber-barrier-timeout] Refusing to read the bag before required subscribers are connected (odometry=${odom_count}/${MIN_ODOM}, deskewed scan=${deskewed_count}/${MIN_DESKEWED})." >&2
exit 70

#!/usr/bin/env bash
# Copyright 2026 Sasaki
# All rights reserved.

# v12 no-input gate startup payload. The host owns Docker lifecycle and this
# payload owns only the loopback ROS master/mapper process lifecycle.
set -Eeuo pipefail

OUT_DIR="${OUT_DIR:-/out}"
STARTUP_TIMEOUT_SECONDS="${M6A10_STARTUP_TIMEOUT_SECONDS:-60}"
mkdir -p "${OUT_DIR}"

ROSCORE_PID=''
MAPPER_PID=''

cleanup() {
  local exit_status=$?
  trap - EXIT INT TERM
  set +e
  for pid in "${MAPPER_PID}" "${ROSCORE_PID}"; do
    if [[ -n "${pid}" ]] && kill -0 "${pid}" >/dev/null 2>&1; then
      kill -TERM "${pid}" >/dev/null 2>&1 || true
    fi
  done
  for pid in "${MAPPER_PID}" "${ROSCORE_PID}"; do
    if [[ -n "${pid}" ]]; then
      wait "${pid}" >/dev/null 2>&1 || true
    fi
  done
  exit "${exit_status}"
}

trap cleanup EXIT
trap 'exit 130' INT
trap 'exit 143' TERM

source /opt/ros/noetic/setup.bash
source /opt/fast_livo_ws/devel/setup.bash

roscore >"${OUT_DIR}/roscore.log" 2>&1 &
ROSCORE_PID=$!

master_ready=0
master_deadline=$((SECONDS + STARTUP_TIMEOUT_SECONDS))
while (( SECONDS < master_deadline )); do
  if rosparam list >/dev/null 2>&1; then
    master_ready=1
    break
  fi
  sleep 0.2
done
printf '%s\n' "${master_ready}" >"${OUT_DIR}/master_ready.txt"
(( master_ready == 1 )) || exit 10
rosparam set use_sim_time true

roslaunch fast_livo mapping_ouster_ntu.launch rviz:=false >"${OUT_DIR}/mapper.log" 2>&1 &
MAPPER_PID=$!

required_services=(
  /m6a10/consumer_status
  /m6a10/consumer_ack
  /m6a10/consumer_eof
  /m6a10/consumer_finalize
  /m6a10/terminal_status
  /m6a10/terminal_eof
  /m6a10/terminal_finalize
)
services_ready=0
service_deadline=$((SECONDS + STARTUP_TIMEOUT_SECONDS))
while (( SECONDS < service_deadline )); do
  services_ready=1
  for service in "${required_services[@]}"; do
    if ! rosservice list 2>/dev/null | grep -Fxq "${service}"; then
      services_ready=0
      break
    fi
  done
  if (( services_ready == 1 )); then
    break
  fi
  if ! kill -0 "${MAPPER_PID}" >/dev/null 2>&1; then
    exit 11
  fi
  sleep 0.2
done
printf '%s\n' "${services_ready}" >"${OUT_DIR}/services_ready.txt"
(( services_ready == 1 )) || exit 12
printf '%s\n' "${required_services[@]}" >"${OUT_DIR}/services.list"

wait "${MAPPER_PID}"

#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
APT_PACKAGE="ros-noetic-xgc2-session-clock-guard"
ROS_PACKAGE="xgc_session_clock_guard"
PREFIX="/opt/ros/noetic"
SOURCE_PROCESS_DEFINITION="${REPO_ROOT}/runtime-manifests/process-definitions/xgc2-session-clock-guard.json"
ROS_PROCESS_DEFINITION="${PREFIX}/share/${ROS_PACKAGE}/runtime-manifests/process-definitions/xgc2-session-clock-guard.json"
SYSTEM_PROCESS_DEFINITION="/usr/share/xgc2/process-definitions/xgc2-session-clock-guard.json"

dpkg -s "${APT_PACKAGE}" >/dev/null

set +u
# The installed ROS setup is created by the package under test and cannot be
# resolved statically from this source checkout.
# shellcheck disable=SC1090,SC1091
source "${PREFIX}/setup.bash"
set -u

test "$(rospack find "${ROS_PACKAGE}")" = "${PREFIX}/share/${ROS_PACKAGE}"
test -f "${PREFIX}/share/${ROS_PACKAGE}/package.xml"
for mode in simulation physical hybrid; do
  test -f "${PREFIX}/share/${ROS_PACKAGE}/config/example-${mode}-v24.cfg"
  test -f "${PREFIX}/share/${ROS_PACKAGE}/config/example-${mode}-v24.cfg.sha256"
done
test -x "${PREFIX}/lib/${ROS_PACKAGE}/session_clock_guard_node"
test -x "${PREFIX}/lib/${ROS_PACKAGE}/session_clock_guard_healthcheck"
test -f "${PREFIX}/lib/libxgc_session_clock_guard_core.so"
test -f "${PREFIX}/share/${ROS_PACKAGE}/msg/ClockGuardStatus.msg"
test -f "${PREFIX}/share/${ROS_PACKAGE}/msg/ClockTimestampEnvelope.msg"
test -f "${PREFIX}/share/${ROS_PACKAGE}/msg/ClockGuardAggregateStatus.msg"
test -f "${PREFIX}/lib/python3/dist-packages/${ROS_PACKAGE}/msg/_ClockGuardAggregateStatus.py"
grep -qx 'string run_mode' \
  "${PREFIX}/share/${ROS_PACKAGE}/msg/ClockGuardStatus.msg"
grep -qx 'string run_mode' \
  "${PREFIX}/share/${ROS_PACKAGE}/msg/ClockTimestampEnvelope.msg"
grep -qx 'uint64 status_sequence' \
  "${PREFIX}/share/${ROS_PACKAGE}/msg/ClockGuardAggregateStatus.msg"
test -f "${ROS_PROCESS_DEFINITION}"
test -f "${SYSTEM_PROCESS_DEFINITION}"
cmp -s "${SOURCE_PROCESS_DEFINITION}" "${ROS_PROCESS_DEFINITION}"
cmp -s "${SOURCE_PROCESS_DEFINITION}" "${SYSTEM_PROCESS_DEFINITION}"
ldd "${PREFIX}/lib/${ROS_PACKAGE}/session_clock_guard_node" | \
  grep -q 'libxgc_session_clock_guard_core.so'
ldd "${PREFIX}/lib/${ROS_PACKAGE}/session_clock_guard_healthcheck" | \
  grep -q 'libxgc_session_clock_guard_core.so'
python3 -m json.tool \
  "${SYSTEM_PROCESS_DEFINITION}" >/dev/null
python3 - "${SYSTEM_PROCESS_DEFINITION}" <<'PY'
import json
import sys

with open(sys.argv[1], encoding="utf-8") as stream:
    definition = json.load(stream)["definitions"][0]
if definition.get("internal") is not True:
    raise SystemExit("installed Session Clock Guard definition is not internal")
if definition["parameters"]["required"] != [
    "policyFile", "policySha256", "epochId"
]:
    raise SystemExit("installed trusted input set changed")
if definition["command"]["args"][-1] != '_epoch_id:="${epochId}"':
    raise SystemExit("installed direct Guard command does not preserve epochId as an XMLRPC string")
for forbidden in ("epochStateFile", "epochLockFile"):
    if forbidden in definition["parameters"]["properties"]:
        raise SystemExit("epoch runtime paths must be derived, not author-selectable")
if definition["readiness"]["command"]["args"] != [
    "${policyFile}", "${policySha256}", "${epochId}"
]:
    raise SystemExit("installed healthcheck is not bound to the startup tuple")
if definition["liveness"] != {
    **definition["readiness"],
    "interval": 1000000000,
    "failureThreshold": 1,
}:
    raise SystemExit("installed liveness is not the prompt strong healthcheck")
PY
python3 - <<'PY'
from xgc_session_clock_guard.msg import (
    ClockGuardAggregateStatus,
    ClockGuardEvent,
    ClockGuardStatus,
    ClockTimestampEnvelope,
)

assert ClockGuardAggregateStatus and ClockGuardEvent
assert ClockGuardStatus and ClockTimestampEnvelope
PY

if dpkg -L "${APT_PACKAGE}" | grep -Eqi 'BeginEpoch|begin_epoch'; then
  echo 'installed package still exposes a runtime epoch-advance service' >&2
  exit 1
fi

cd "${PREFIX}/share/${ROS_PACKAGE}/config"
sha256sum --check \
  example-simulation-v24.cfg.sha256 \
  example-physical-v24.cfg.sha256 \
  example-hybrid-v24.cfg.sha256
roslaunch --files "${ROS_PACKAGE}" session_clock_guard.launch \
  policy_file:=/tmp/required policy_sha256:="$(printf '0%.0s' {1..64})" epoch_id:=1 >/dev/null

echo 'installed Session Clock Guard package check passed'

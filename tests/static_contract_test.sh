#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"

cd "${REPO_ROOT}"

command -v grep >/dev/null

forbidden_core_import='xgc2/core-xgc/''internal'
if rg -n "${forbidden_core_import}" .github .xgc2 tests; then
  echo 'standalone leaf must not import product-internal Core packages' >&2
  exit 1
fi

(cd config && sha256sum --check \
  example-simulation-v24.cfg.sha256 \
  example-physical-v24.cfg.sha256 \
  example-hybrid-v24.cfg.sha256)
./tests/process_definition_discovery_test.py

for mode in simulation physical hybrid; do
  config="config/example-${mode}-v24.cfg"
  grep -qx 'schema=xgc.session-clock-guard.config.v2' "${config}"
  grep -qx 'policy_revision=xgc.session-clock-guard.builtin-policy.v1' "${config}"
  grep -qx 'vrpn.wire_time_resolution_ns=1000' "${config}"
  grep -qx 'delay.measurement_enabled=true' "${config}"
  grep -qx 'delay.timestamp_policy=sample_time' "${config}"
  grep -qx 'threshold.startup_lock_timeout_ns=3000000000' "${config}"
  grep -qx 'threshold.max_authority_age_ns=100000000' "${config}"
done
grep -qx 'session_clock.authority=gazebo-simulation' \
  config/example-simulation-v24.cfg
grep -qx 'session_clock.mapping=identity' config/example-simulation-v24.cfg
grep -qx 'session_clock.authority=station-wall-monotonic' \
  config/example-physical-v24.cfg
grep -qx 'session_clock.mapping=affine-to-session' \
  config/example-physical-v24.cfg
grep -qx 'threshold.min_gazebo_real_time_factor=0.8' \
  config/example-hybrid-v24.cfg
grep -qx 'route.scout_1.source_body=ugv1' config/example-hybrid-v24.cfg
grep -qx 'route.scout_1.canonical_body=uav7' config/example-hybrid-v24.cfg
grep -q 'source_uncertainty_ns is below wire quantization' \
  src/core/frozen_config.cpp
grep -q 'plus half-sample floor' src/core/frozen_config.cpp
grep -q 'a strictly newer epoch is required' src/core/clock_mapper.cpp
grep -q 'std::chrono::system_clock' src/ros/session_clock_guard_node.cpp
grep -q 'std::chrono::steady_clock' src/ros/session_clock_guard_node.cpp
grep -q 'ClockGuardEvent.msg' CMakeLists.txt
grep -q 'session_clock_guard_healthcheck' CMakeLists.txt
grep -qx 'string run_mode' msg/ClockGuardStatus.msg
grep -qx 'string run_mode' msg/ClockTimestampEnvelope.msg
grep -qx 'string run_mode' msg/ClockGuardAggregateStatus.msg
grep -qx 'uint64 status_sequence' msg/ClockGuardAggregateStatus.msg
grep -q 'message.run_mode = config_.run_mode' \
  src/ros/session_clock_guard_node.cpp
grep -q 'message.status_sequence = next_status_sequence_++' \
  src/ros/session_clock_guard_node.cpp
grep -q 'validateLockedAdmission' \
  src/core/healthcheck.cpp
grep -q 'validateAggregatePublisherSet' \
  src/ros/session_clock_guard_healthcheck.cpp
grep -q 'LiveAdmissionTracker' src/ros/session_clock_guard_healthcheck.cpp
grep -Eq 'initByFullCallbackType<const StatusEvent[[:space:]]*&>' \
  src/ros/session_clock_guard_healthcheck.cpp
grep -q 'requireUniquePublisher' src/ros/session_clock_guard_healthcheck.cpp
grep -q 'kAggregateTopic = "/xgc/session/clock/status"' \
  src/ros/session_clock_guard_healthcheck.cpp
grep -q 'kExpectedPublisher = "/xgc_session_clock_guard"' \
  src/ros/session_clock_guard_healthcheck.cpp
grep -q 'accepted_samples_ >= 2U' \
  include/xgc_session_clock_guard/healthcheck.hpp
grep -q 'evidence.status_sequence < last_sequence_' src/core/healthcheck.cpp
grep -q 'evidence.status_sequence == last_sequence_' src/core/healthcheck.cpp
grep -q 'authority_age_ns > config.thresholds.max_authority_age_ns' \
  src/core/healthcheck.cpp
grep -q 'max_gazebo_clock_skew_ns' src/core/healthcheck.cpp
grep -q 'max_station_wall_error_ns' src/core/healthcheck.cpp
grep -q 'min_gazebo_real_time_factor' src/core/healthcheck.cpp
grep -q 'loadAndValidateEpochFence' \
  src/ros/session_clock_guard_healthcheck.cpp
grep -q 'epochStatePathForPolicy' src/core/epoch_fence.cpp
grep -q 'epoch fence must have exact mode 0600' src/core/epoch_fence.cpp
grep -q 'loadAndValidateEpochFence' src/ros/session_clock_guard_node.cpp
grep -q 'kMaximumGuardPollPeriodNs = 250000000ULL' \
  src/core/frozen_config.cpp
grep -q 'kMinimumStartupLockTimeoutNs = 250000000ULL' \
  src/core/frozen_config.cpp
grep -q 'kMaximumStartupLockTimeoutNs = 3000000000ULL' \
  src/core/frozen_config.cpp
grep -q 'policy_.max_sample_age_ns' src/core/clock_mapper.cpp
grep -q ': policy_.startup_lock_timeout_ns' \
  src/core/clock_mapper.cpp
grep -q 'config_.thresholds.startup_lock_timeout_ns' \
  src/core/session_clock_guard.cpp
grep -q 'EpochFenceLease' src/ros/session_clock_guard_node.cpp
grep -q 'parseRosPrivateEpochId(epoch_text)' src/ros/session_clock_guard_node.cpp
# shellcheck disable=SC2016 # Match the literal Process placeholder.
grep -Fq '_epoch_id:=\"${epochId}\"' \
  runtime-manifests/process-definitions/xgc2-session-clock-guard.json
# shellcheck disable=SC2016 # Match the literal package-script variable.
grep -Fq 'lib/python3/dist-packages/${ROS_PACKAGE}' \
  .xgc2/scripts/package_debs.sh
grep -q 'epoch_fence_lease_->validateCurrent' \
  src/ros/session_clock_guard_node.cpp
grep -q 'LOCK_SH | LOCK_NB' src/core/epoch_fence.cpp
grep -q 'epoch-state.lock' src/core/epoch_fence.cpp
grep -q 'epoch fence lock must be an empty regular' src/core/epoch_fence.cpp
grep -q 'non-symlink file with exact mode 0600' src/core/epoch_fence.cpp
grep -A4 'void pollGuard' src/ros/session_clock_guard_node.cpp | \
  grep -q 'ensureEpochFence'
grep -q '"kind": "exec"' \
  runtime-manifests/process-definitions/xgc2-session-clock-guard.json
grep -q '"timeout": 5000000000' \
  runtime-manifests/process-definitions/xgc2-session-clock-guard.json
test "$(grep -c 'session_clock_guard_healthcheck' \
  runtime-manifests/process-definitions/xgc2-session-clock-guard.json)" -eq 2
grep -q '"internal": true' \
  runtime-manifests/process-definitions/xgc2-session-clock-guard.json
grep -q 'route.canonical_body + "/status"' src/core/types.cpp
grep -q 'route.canonical_body + "/envelope"' src/core/types.cpp
if grep -q 'ros::Time::now' src/ros/session_clock_guard_node.cpp; then
  echo 'ROS time must not be used as a Session authority' >&2
  exit 1
fi

test ! -e srv/BeginEpoch.srv
if grep -RInE -- 'BeginEpoch|begin_epoch|beginEpoch' \
  README.md docs include launch msg src CMakeLists.txt package.xml; then
  echo 'runtime epoch-advance surface detected in Session Clock Guard product' >&2
  exit 1
fi

grep -q 'canonical syntax and digest' README.md
if grep -q 'roslaunch xgc_session_clock_guard session_clock_guard.launch' README.md; then
  echo 'README must not advertise a manual example launch without the Core epoch fence' >&2
  exit 1
fi

for required_arg in policy_file policy_sha256 epoch_id; do
  grep -Eq "<arg name=\"${required_arg}\"[[:space:]]*/>" launch/session_clock_guard.launch
done

for workflow in .github/workflows/ci.yml .github/workflows/release.yml; do
  grep -q 'arch: amd64' "${workflow}"
  grep -q 'arch: arm64' "${workflow}"
  grep -q 'xgc2_artifact_manifest.py build' "${workflow}"
  grep -q 'xgc2_artifact_manifest.py verify-build' "${workflow}"
  grep -q 'retention-days: 14' "${workflow}"
done
test ! -e .github/workflows/session-clock-guard-ci.yml
grep -q 'distribution: focal' .xgc2/product.yml
grep -q 'distro: noetic' .xgc2/product.yml
grep -q 'ros-noetic-rosgraph-msgs' .xgc2/product.yml
grep -q 'ros-noetic-roslaunch' .xgc2/product.yml
grep -q '/usr/share/xgc2/process-definitions/xgc2-session-clock-guard.json' \
  .xgc2/product.yml
grep -q 'repository: lxk36/xgc2-session-clock-guard' .xgc2/product.yml
grep -q 'ref: noetic' .xgc2/product.yml
if grep -Eq \
  'standalone_product_repository_not_initialized_or_published|parent_repository_gitlink_not_registered' \
  .xgc2/release-contract.yml; then
  echo 'resolved standalone repository gap remains in release contract' >&2
  exit 1
fi
grep -q 'apt_repository_publication_not_completed' .xgc2/release-contract.yml

if grep -RInE -- '/cmd_vel|mavros/.*/setpoint|arming|takeoff|land' \
  src include msg launch CMakeLists.txt package.xml; then
  echo 'motion-command surface detected in Session Clock Guard product' >&2
  exit 1
fi

echo 'session-clock-guard static contract passed'

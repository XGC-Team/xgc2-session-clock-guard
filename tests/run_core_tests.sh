#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="$(mktemp -d)"

cleanup() {
  rm -rf "${BUILD_DIR}"
}
trap cleanup EXIT

g++ \
  -std=c++17 \
  -Wall \
  -Wextra \
  -Werror \
  -pedantic \
  -I"${REPO_ROOT}/include" \
  "${REPO_ROOT}/src/core/types.cpp" \
  "${REPO_ROOT}/src/core/sha256.cpp" \
  "${REPO_ROOT}/src/core/frozen_config.cpp" \
  "${REPO_ROOT}/src/core/epoch_fence.cpp" \
  "${REPO_ROOT}/src/core/healthcheck.cpp" \
  "${REPO_ROOT}/src/core/clock_mapper.cpp" \
  "${REPO_ROOT}/src/core/session_clock_guard.cpp" \
  "${REPO_ROOT}/test/core_tests.cpp" \
  -o "${BUILD_DIR}/session_clock_guard_core_tests"

config_args=()
for mode in simulation physical hybrid; do
  config_path="${REPO_ROOT}/config/example-${mode}-v24.cfg"
  digest_path="${config_path}.sha256"
  config_args+=("${config_path}" "$(awk '{print $1}' "${digest_path}")")
done
"${BUILD_DIR}/session_clock_guard_core_tests" "${config_args[@]}"

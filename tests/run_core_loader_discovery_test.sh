#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
CORE_ROOT="${XGC2_CORE_XGC_ROOT:-$(realpath "${REPO_ROOT}/../../../xgc2/xgc2/core-xgc")}"
MANIFEST="$(realpath "${REPO_ROOT}/runtime-manifests/process-definitions/xgc2-session-clock-guard.json")"
BACKING_TEST="$(realpath "${REPO_ROOT}/tests/core_loader_discovery_test.go")"
BACKING_ADMISSION_TEST="$(realpath "${REPO_ROOT}/tests/core_admission_contract_test.go")"

if [[ ! -f "${CORE_ROOT}/go.mod" ]]; then
  echo "Core module is unavailable: ${CORE_ROOT}" >&2
  exit 1
fi

OVERLAY_FILE="$(mktemp)"
cleanup() {
  rm -f "${OVERLAY_FILE}"
}
trap cleanup EXIT

VIRTUAL_TEST="${CORE_ROOT}/internal/processcatalog/session_clock_guard_product_external_test.go"
VIRTUAL_ADMISSION_TEST="${CORE_ROOT}/internal/executioncatalog/session_clock_guard_product_external_test.go"
python3 - "${OVERLAY_FILE}" "${VIRTUAL_TEST}" "${BACKING_TEST}" \
  "${VIRTUAL_ADMISSION_TEST}" "${BACKING_ADMISSION_TEST}" <<'PY'
import json
import sys

with open(sys.argv[1], "w", encoding="utf-8") as stream:
    json.dump({"Replace": {sys.argv[2]: sys.argv[3], sys.argv[4]: sys.argv[5]}}, stream)
PY

(
  cd "${CORE_ROOT}"
  XGC_SESSION_CLOCK_GUARD_MANIFEST="${MANIFEST}" \
    go test -count=1 -overlay="${OVERLAY_FILE}" \
    ./internal/processcatalog -run '^TestExternalSessionClockGuardProductDiscovery$'
  XGC_SESSION_CLOCK_GUARD_MANIFEST="${MANIFEST}" \
    go test -count=1 -overlay="${OVERLAY_FILE}" \
    ./internal/executioncatalog -run '^TestExternalSessionClockGuardProductMatchesAdmissionContract$'
  go test -count=1 ./internal/process \
    -run '^TestInternalDefinitionRequiresTrustedCreationAndIsNotPublic$'
)

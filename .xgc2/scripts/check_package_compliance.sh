#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
temporary="$(mktemp -d /tmp/xgc2-session-clock-compliance.XXXXXX)"
cleanup() {
  rm -rf "$temporary"
}
trap cleanup EXIT

for command in bash python3 rg shellcheck; do
  command -v "$command" >/dev/null 2>&1 || {
    echo "missing compliance dependency: $command" >&2
    exit 1
  }
done

for required in \
  LICENSE README.md CMakeLists.txt package.xml .clang-format \
  .github/workflows/ci.yml .github/workflows/release.yml \
  .xgc2/product.yml .xgc2/release-contract.yml \
  .xgc2/scripts/build_debs_in_docker.sh \
  .xgc2/scripts/check_cpp_quality.sh \
  .xgc2/scripts/check_installed_packages.sh \
  .xgc2/scripts/check_package_compliance.sh \
  .xgc2/scripts/configure_xgc2_apt.sh \
  .xgc2/scripts/package_debs.sh \
  .xgc2/scripts/require_clang_format_10.sh \
  .xgc2/scripts/xgc2_artifact_manifest.py; do
  test -f "$REPO_ROOT/$required" || {
    echo "missing standalone release file: $required" >&2
    exit 1
  }
done
test ! -e "$REPO_ROOT/.github/workflows/session-clock-guard-ci.yml" || {
  echo "obsolete combined workflow remains" >&2
  exit 1
}
for script in "$REPO_ROOT"/.xgc2/scripts/*.sh \
  "$REPO_ROOT"/.xgc2/scripts/*.py "$REPO_ROOT"/tests/*.sh \
  "$REPO_ROOT"/tests/*.py; do
  test -x "$script" || {
    echo "repository script is not executable: $script" >&2
    exit 1
  }
done

python3 - "$REPO_ROOT/.xgc2/product.yml" \
  "$REPO_ROOT/.xgc2/release-contract.yml" "$REPO_ROOT/package.xml" \
  "$REPO_ROOT/runtime-manifests/process-definitions/xgc2-session-clock-guard.json" <<'PY'
import json
import sys
from pathlib import Path
import xml.etree.ElementTree as ET

import yaml

product_path, contract_path, package_path, process_path = map(Path, sys.argv[1:])
product = yaml.safe_load(product_path.read_text(encoding="utf-8"))
contract = yaml.safe_load(contract_path.read_text(encoding="utf-8"))
if not isinstance(product, dict) or not isinstance(contract, dict):
    raise SystemExit("product metadata and release contract must be mappings")
expected_product = {
    "schema": "xgc2.product.v1",
    "id": "xgc2-session-clock-guard",
    "name": "XGC2 Session Clock Guard",
    "version": "0.1.0-6",
    "kind": "ros1-apt",
}
for key, value in expected_product.items():
    if product.get(key) != value:
        raise SystemExit(f"product {key} mismatch: {product.get(key)!r}")
apt_package = "ros-noetic-xgc2-session-clock-guard"
apt = product.get("apt")
if not isinstance(apt, dict):
    raise SystemExit("apt metadata must be a mapping")
if apt.get("distribution") != "focal":
    raise SystemExit("APT distribution must be focal")
if apt.get("install") != [apt_package] or apt.get("packages") != [apt_package]:
    raise SystemExit("APT package identity is not exact")
release = product.get("release")
if not isinstance(release, dict):
    raise SystemExit("release metadata must be a mapping")
release_identity = {
    "repository": "XGC-Team/xgc2-session-clock-guard",
    "ref": "noetic",
    "workflow": "release.yml",
    "ci_workflow": "ci.yml",
    "apt_versions": {"focal": "0.1.0-6"},
}
for key, value in release_identity.items():
    if release.get(key) != value:
        raise SystemExit(f"release {key} mismatch: {release.get(key)!r}")

if contract.get("schema") != "xgc2.release-contract.v1":
    raise SystemExit("release contract schema mismatch")
if contract.get("product") != expected_product["id"]:
    raise SystemExit("release contract product mismatch")
if contract.get("version") != expected_product["version"]:
    raise SystemExit("release contract version mismatch")
gaps = contract.get("integration_gaps")
if not isinstance(gaps, list):
    raise SystemExit("release contract integration_gaps must be a list")
for stale in {
    "standalone_product_repository_not_initialized_or_published",
    "parent_repository_gitlink_not_registered",
}:
    if stale in gaps:
        raise SystemExit(f"stale standalone gap remains: {stale}")
for remaining in {
    "apt_repository_publication_not_completed",
    "apt_installed_product_and_current_core_not_live_integrated",
}:
    if remaining not in gaps:
        raise SystemExit(f"unproven integration gap was removed: {remaining}")

package = ET.parse(package_path).getroot()
if package.findtext("name") != "xgc_session_clock_guard":
    raise SystemExit("ROS package identity mismatch")
if package.findtext("version") != "0.1.0":
    raise SystemExit("ROS package semantic version mismatch")
if package.findtext("license") != "BSD-3-Clause":
    raise SystemExit("ROS package license mismatch")

process = json.loads(process_path.read_text(encoding="utf-8"))
definitions = process.get("definitions")
if not isinstance(definitions, list) or len(definitions) != 1:
    raise SystemExit("process manifest must contain exactly one definition")
definition = definitions[0]
if definition.get("id") != expected_product["id"]:
    raise SystemExit("process identity mismatch")
if definition.get("version") != "0.1.0":
    raise SystemExit("process semantic version mismatch")
if definition.get("internal") is not True:
    raise SystemExit("Session Clock Guard process must remain internal")
PY

for workflow in ci.yml release.yml; do
  path="$REPO_ROOT/.github/workflows/$workflow"
  grep -Fq 'arch: amd64' "$path"
  grep -Fq 'arch: arm64' "$path"
  grep -Fq 'ubuntu-24.04-arm' "$path"
  grep -Fq '.xgc2/scripts/build_debs_in_docker.sh' "$path"
  grep -Fq 'xgc2_artifact_manifest.py build' "$path"
  grep -Fq 'xgc2_artifact_manifest.py verify-build' "$path"
  grep -Fq 'retention-days: 14' "$path"
  grep -Fq 'actions/checkout@9c091bb21b7c1c1d1991bb908d89e4e9dddfe3e0' "$path"
  grep -Fq 'actions/upload-artifact@043fb46d1a93c77aae656e7c1c64a875d1fc6a0a' "$path"
done
grep -Fq '.xgc2/scripts/check_cpp_quality.sh' \
  "$REPO_ROOT/.github/workflows/ci.yml"
for input in expected_version expected_source_sha prepare_action apt_overlay_url \
  dependency_set_digest; do
  grep -Eq "^[[:space:]]+${input}:" "$REPO_ROOT/.github/workflows/release.yml"
done
for legacy_input in run_cpp_quality run_source_tests; do
  if rg -n "inputs\.${legacy_input}|^[[:space:]]+${legacy_input}:" \
    "$REPO_ROOT/.github/workflows/release.yml"; then
    echo "legacy release input remains: ${legacy_input}" >&2
    exit 1
  fi
done
grep -Fq '/etc/apt/sources.list.d/xgc2.list' \
  "$REPO_ROOT/.xgc2/scripts/configure_xgc2_apt.sh"
grep -Fq '00-xgc2-release-train.list' \
  "$REPO_ROOT/.xgc2/scripts/configure_xgc2_apt.sh"
grep -Fq 'https://xgc2.apt.xiaokang.ink' \
  "$REPO_ROOT/.xgc2/scripts/configure_xgc2_apt.sh"
if rg -n 'XGC2_APT_OVERLAY_URL:-https://xgc2.apt' \
  "$REPO_ROOT/.xgc2/scripts/configure_xgc2_apt.sh"; then
  echo "staging overlay must not replace production APT" >&2
  exit 1
fi
grep -Fq 'readonly FORMATTER=clang-format-10' \
  "$REPO_ROOT/.xgc2/scripts/require_clang_format_10.sh"
grep -Fq 'clang-format[[:space:]]version[[:space:]]10[.]' \
  "$REPO_ROOT/.xgc2/scripts/require_clang_format_10.sh"

for script in "$REPO_ROOT"/.xgc2/scripts/*.sh "$REPO_ROOT"/tests/*.sh; do
  bash -n "$script"
done
shellcheck "$REPO_ROOT"/.xgc2/scripts/*.sh "$REPO_ROOT"/tests/*.sh
PYTHONPYCACHEPREFIX="$temporary/pycache" python3 -m py_compile \
  "$REPO_ROOT"/.xgc2/scripts/*.py "$REPO_ROOT"/tests/*.py

format_test_bin="$temporary/format-test-bin"
mkdir -p "$format_test_bin"
printf '%s\n' '#!/usr/bin/env bash' \
  'echo "clang-format version 9.0.0"' >"$format_test_bin/clang-format-10"
chmod 0755 "$format_test_bin/clang-format-10"
if PATH="$format_test_bin:$PATH" \
  "$REPO_ROOT/.xgc2/scripts/require_clang_format_10.sh" \
  >"$temporary/format-version.out" 2>"$temporary/format-version.err"; then
  echo "formatter contract accepted clang-format 9" >&2
  exit 1
fi
grep -Fq 'required formatter major is 10' "$temporary/format-version.err"

forbidden_pattern='publish_''apt|APT_''REPO_[A-Z0-9_]+|xgc2-apt-''production|repre''pro|apt''ly[[:space:]]+publish'
if rg -n -i "$forbidden_pattern" \
  "$REPO_ROOT/.github" "$REPO_ROOT/.xgc2/scripts"; then
  echo "leaf-owned APT publication or centralized credentials remain" >&2
  exit 1
fi
forbidden_core_import='xgc2/core-xgc/''internal'
if rg -n "$forbidden_core_import" \
  "$REPO_ROOT/.github" "$REPO_ROOT/.xgc2" "$REPO_ROOT/tests"; then
  echo "standalone leaf must not import product-internal Core packages" >&2
  exit 1
fi

echo "Package compliance check passed"

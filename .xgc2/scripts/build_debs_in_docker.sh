#!/usr/bin/env bash
# shellcheck disable=SC1004
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker-noetic}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
EXPECTED_ARCH="${EXPECTED_ARCH:-}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"
PREPARE_ACTION="${PREPARE_ACTION:-ci}"
XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}"
XGC2_DEPENDENCY_SET_DIGEST="${XGC2_DEPENDENCY_SET_DIGEST:-}"
HOST_UID="$(id -u)"
HOST_GID="$(id -g)"
readonly HOST_UID HOST_GID REPO_ROOT SCRIPT_DIR

while [[ $# -gt 0 ]]; do
  case "$1" in
    --image)
      DOCKER_IMAGE="$2"
      shift 2
      ;;
    --work-dir)
      WORK_DIR="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    --skip-install-check)
      INSTALL_CHECK=false
      shift
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 2
      ;;
  esac
done

[[ "$PREPARE_ACTION" =~ ^(ci|release|compatibility-verify)$ ]] || {
  echo "invalid PREPARE_ACTION: $PREPARE_ACTION" >&2
  exit 2
}
[[ "$INSTALL_CHECK" =~ ^(true|false)$ ]] || {
  echo "INSTALL_CHECK must be true or false" >&2
  exit 2
}
[[ -z "$EXPECTED_ARCH" || "$EXPECTED_ARCH" =~ ^(amd64|arm64)$ ]] || {
  echo "EXPECTED_ARCH must be amd64 or arm64" >&2
  exit 2
}
[[ -z "$XGC2_DEPENDENCY_SET_DIGEST" || \
  "$XGC2_DEPENDENCY_SET_DIGEST" =~ ^[0-9a-f]{64}$ ]] || {
  echo "XGC2_DEPENDENCY_SET_DIGEST must be empty or 64 lowercase hexadecimal characters" >&2
  exit 2
}
if [[ -n "$XGC2_APT_OVERLAY_URL" ]]; then
  "$SCRIPT_DIR/configure_xgc2_apt.sh" --validate-url "$XGC2_APT_OVERLAY_URL"
  [[ "$XGC2_DEPENDENCY_SET_DIGEST" =~ ^[0-9a-f]{64}$ ]] || {
    echo "XGC2_APT_OVERLAY_URL requires XGC2_DEPENDENCY_SET_DIGEST" >&2
    exit 2
  }
fi
if [[ "$PREPARE_ACTION" == compatibility-verify && \
  -z "$XGC2_APT_OVERLAY_URL" ]]; then
  echo "compatibility-verify requires XGC2_APT_OVERLAY_URL" >&2
  exit 2
fi
[[ "$WORK_DIR" == /* && "$WORK_DIR" != / ]] || {
  echo "--work-dir must be an absolute, non-root path" >&2
  exit 2
}
[[ "$OUTPUT_DIR" == /* && "$OUTPUT_DIR" != / ]] || {
  echo "--output-dir must be an absolute, non-root path" >&2
  exit 2
}
[[ "$HOST_UID" =~ ^[0-9]+$ && "$HOST_GID" =~ ^[0-9]+$ ]] || {
  echo "host uid/gid must be numeric" >&2
  exit 1
}

mkdir -p "$WORK_DIR" "$OUTPUT_DIR"
WORK_DIR="$(realpath -m "$WORK_DIR")"
OUTPUT_DIR="$(realpath -m "$OUTPUT_DIR")"
if find "$OUTPUT_DIR" -mindepth 1 -maxdepth 1 -print -quit | grep -q .; then
  echo "Debian output directory is not isolated: $OUTPUT_DIR" >&2
  exit 1
fi
STAGING_OUTPUT_DIR="$(mktemp -d "${OUTPUT_DIR}.staging.XXXXXX")"
cleanup() {
  rm -rf "$STAGING_OUTPUT_DIR"
}
trap cleanup EXIT

docker pull "$DOCKER_IMAGE"
docker run --rm \
  -e "DEBIAN_FRONTEND=noninteractive" \
  -e "EXPECTED_ARCH=${EXPECTED_ARCH}" \
  -e "HOST_GID=${HOST_GID}" \
  -e "HOST_UID=${HOST_UID}" \
  -e "INSTALL_CHECK=${INSTALL_CHECK}" \
  -e "PREPARE_ACTION=${PREPARE_ACTION}" \
  -e "XGC2_APT_OVERLAY_URL=${XGC2_APT_OVERLAY_URL}" \
  -e "XGC2_DEPENDENCY_SET_DIGEST=${XGC2_DEPENDENCY_SET_DIGEST}" \
  -v "$REPO_ROOT:/workspace/source:ro" \
  -v "$WORK_DIR:/workspace/work" \
  -v "$STAGING_OUTPUT_DIR:/workspace/out" \
  "$DOCKER_IMAGE" bash -lc '
    set -euo pipefail
    # shellcheck disable=SC2317 # EXIT invokes this callback indirectly.
    return_mount_ownership() {
      chown -R "${HOST_UID}:${HOST_GID}" /workspace/work /workspace/out
    }
    trap return_mount_ownership EXIT

    actual_arch="$(dpkg --print-architecture)"
    [[ -z "$EXPECTED_ARCH" || "$actual_arch" == "$EXPECTED_ARCH" ]] || {
      echo "container architecture $actual_arch != $EXPECTED_ARCH" >&2
      exit 1
    }
    /workspace/source/.xgc2/scripts/configure_xgc2_apt.sh focal
    apt-get install -y --no-install-recommends \
      build-essential \
      clang-format-10 \
      dpkg-dev \
      fakeroot \
      python3-catkin-pkg \
      python3-catkin-tools \
      python3-yaml \
      ripgrep \
      rsync \
      shellcheck \
      ros-noetic-geometry-msgs \
      ros-noetic-message-generation \
      ros-noetic-message-runtime \
      ros-noetic-roscpp \
      ros-noetic-rosgraph-msgs \
      ros-noetic-roslaunch \
      ros-noetic-rospack \
      ros-noetic-std-msgs

    find /workspace/work -mindepth 1 -maxdepth 1 \
      \( -name build -o -name devel -o -name install-root -o -name src \) \
      -exec rm -rf {} +
    mkdir -p /workspace/work/src/xgc_session_clock_guard \
      /workspace/work/install-root
    rsync -a --delete \
      --exclude .git --exclude .ci --exclude .work --exclude build \
      --exclude debs --exclude devel --exclude install-root \
      /workspace/source/ /workspace/work/src/xgc_session_clock_guard/

    /workspace/work/src/xgc_session_clock_guard/.xgc2/scripts/check_package_compliance.sh
    /workspace/work/src/xgc_session_clock_guard/tests/run_core_tests.sh
    /workspace/work/src/xgc_session_clock_guard/tests/static_contract_test.sh

    cd /workspace/work
    set +u
    # shellcheck disable=SC1091
    source /opt/ros/noetic/setup.bash
    set -u
    parallel_jobs="$(nproc)"
    catkin_make -j"${parallel_jobs}" -l"${parallel_jobs}" \
      -DCMAKE_BUILD_TYPE=Release \
      -DCMAKE_CXX_FLAGS_RELEASE="-O2 -DNDEBUG"
    catkin_make run_tests -j"${parallel_jobs}" -l"${parallel_jobs}"
    catkin_test_results --all
    DESTDIR=/workspace/work/install-root catkin_make install \
      -DCMAKE_INSTALL_PREFIX=/opt/ros/noetic \
      -DCMAKE_BUILD_TYPE=Release

    /workspace/work/src/xgc_session_clock_guard/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    mapfile -t debs < <(find /workspace/out -maxdepth 1 -type f \
      -name "ros-noetic-xgc2-session-clock-guard_*.deb" -print | sort)
    (( ${#debs[@]} == 1 )) || {
      echo "builder did not produce exactly one product Deb" >&2
      exit 1
    }
    test "$(dpkg-deb -f "${debs[0]}" Architecture)" = "$actual_arch"
    if [[ "$INSTALL_CHECK" == true ]]; then
      apt-get install -y "${debs[0]}"
      /workspace/work/src/xgc_session_clock_guard/.xgc2/scripts/check_installed_packages.sh
    fi
  '

mapfile -t staged_files < <(
  find "$STAGING_OUTPUT_DIR" -mindepth 1 -maxdepth 1 -type f -print | sort
)
(( ${#staged_files[@]} == 1 )) || {
  echo "builder produced an unexpected artifact set" >&2
  exit 1
}
for staged_file in "${staged_files[@]}"; do
  mv "$staged_file" "$OUTPUT_DIR/"
done
find "$OUTPUT_DIR" -mindepth 1 -maxdepth 1 -type f -print | sort

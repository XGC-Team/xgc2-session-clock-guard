#!/usr/bin/env bash
# shellcheck disable=SC1004
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"

DOCKER_IMAGE="${DOCKER_IMAGE:-ros:noetic-ros-base-focal}"
WORK_DIR="${WORK_DIR:-${REPO_ROOT}/.work/docker}"
OUTPUT_DIR="${OUTPUT_DIR:-${REPO_ROOT}/debs}"
INSTALL_CHECK="${INSTALL_CHECK:-true}"

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
      exit 1
      ;;
  esac
done

mkdir -p "${WORK_DIR}" "${OUTPUT_DIR}"

docker pull "${DOCKER_IMAGE}"
docker run --rm \
  -e DEBIAN_FRONTEND=noninteractive \
  -e INSTALL_CHECK="${INSTALL_CHECK}" \
  -e XGC2_APT_OVERLAY_URL="${XGC2_APT_OVERLAY_URL:-}" \
  -v "${REPO_ROOT}:/workspace/repo:ro" \
  -v "${WORK_DIR}:/workspace/work" \
  -v "${OUTPUT_DIR}:/workspace/out" \
  "${DOCKER_IMAGE}" \
  bash -lc '
    set -euo pipefail
    export DEBIAN_FRONTEND=noninteractive

    /workspace/repo/.xgc2/scripts/configure_xgc2_apt.sh focal
    apt-get install -y --no-install-recommends \
      build-essential \
      dpkg-dev \
      fakeroot \
      python3-catkin-tools \
      python3-catkin-pkg \
      rsync \
      ros-noetic-geometry-msgs \
      ros-noetic-message-generation \
      ros-noetic-message-runtime \
      ros-noetic-roscpp \
      ros-noetic-rosgraph-msgs \
      ros-noetic-roslaunch \
      ros-noetic-rospack \
      ros-noetic-std-msgs

    /workspace/repo/tests/run_core_tests.sh
    /workspace/repo/tests/static_contract_test.sh

    rm -rf /workspace/work/src /workspace/work/build /workspace/work/devel \
      /workspace/work/install-root
    mkdir -p /workspace/work/src/xgc_session_clock_guard
    rsync -a --delete /workspace/repo/ /workspace/work/src/xgc_session_clock_guard/

    cd /workspace/work
    set +u
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

    /workspace/repo/.xgc2/scripts/package_debs.sh \
      --install-root /workspace/work/install-root \
      --output-dir /workspace/out

    if [[ "${INSTALL_CHECK}" == "true" ]]; then
      apt-get install -y /workspace/out/ros-noetic-xgc2-session-clock-guard_*.deb
      /workspace/repo/.xgc2/scripts/check_installed_packages.sh
    fi
  '

find "${OUTPUT_DIR}" -maxdepth 1 -type f \
  -name 'ros-noetic-xgc2-session-clock-guard_*.deb' -print | sort

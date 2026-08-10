#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/../.." && pwd)"
INSTALL_ROOT=""
OUTPUT_DIR=""
ROS_DISTRO="noetic"
ROS_PACKAGE="xgc_session_clock_guard"
APT_PACKAGE="ros-noetic-xgc2-session-clock-guard"

while [[ $# -gt 0 ]]; do
  case "$1" in
    --install-root)
      INSTALL_ROOT="$2"
      shift 2
      ;;
    --output-dir)
      OUTPUT_DIR="$2"
      shift 2
      ;;
    *)
      echo "unknown argument: $1" >&2
      exit 1
      ;;
  esac
done

if [[ -z "${INSTALL_ROOT}" || -z "${OUTPUT_DIR}" ]]; then
  echo '--install-root and --output-dir are required' >&2
  exit 1
fi

VERSION="$(sed -n 's/^version:[[:space:]]*//p' "${REPO_ROOT}/.xgc2/product.yml" | head -n 1)"
ARCHITECTURE="$(dpkg --print-architecture)"
PREFIX="/opt/ros/${ROS_DISTRO}"
PACKAGE_ROOT="$(mktemp -d)"

cleanup() {
  rm -rf "${PACKAGE_ROOT}"
}
trap cleanup EXIT

mkdir -p "${PACKAGE_ROOT}/DEBIAN" "${PACKAGE_ROOT}/usr/share/doc/${APT_PACKAGE}" \
  "${OUTPUT_DIR}"

copy_installed() {
  local relative="$1"
  local source="${INSTALL_ROOT}${PREFIX}/${relative}"
  if [[ ! -e "${source}" ]]; then
    echo "missing installed path: ${source}" >&2
    exit 1
  fi
  mkdir -p "${PACKAGE_ROOT}${PREFIX}/$(dirname "${relative}")"
  cp -a "${source}" "${PACKAGE_ROOT}${PREFIX}/${relative}"
}

copy_installed "share/${ROS_PACKAGE}"
copy_installed "include/${ROS_PACKAGE}"
copy_installed "lib/libxgc_session_clock_guard_core.so"
copy_installed "lib/${ROS_PACKAGE}"
copy_installed "lib/python3/dist-packages/${ROS_PACKAGE}"

mkdir -p "${PACKAGE_ROOT}/usr/share/xgc2/process-definitions"
cp -a \
  "${PACKAGE_ROOT}${PREFIX}/share/${ROS_PACKAGE}/runtime-manifests/process-definitions/xgc2-session-clock-guard.json" \
  "${PACKAGE_ROOT}/usr/share/xgc2/process-definitions/xgc2-session-clock-guard.json"

cat > "${PACKAGE_ROOT}/DEBIAN/control" <<EOF
Package: ${APT_PACKAGE}
Version: ${VERSION}
Section: misc
Priority: optional
Architecture: ${ARCHITECTURE}
Maintainer: XGC2 <apt@example.com>
Depends: ros-noetic-geometry-msgs, ros-noetic-message-runtime, ros-noetic-roscpp, ros-noetic-rosgraph-msgs, ros-noetic-roslaunch, ros-noetic-std-msgs
Description: XGC2 ROS1 Session clock mapper and route guard
 Fail-closed VRPN source timestamp mapping and canonical pose/twist admission
 for simulation, physical, and Hybrid experiments.
EOF

cat > "${PACKAGE_ROOT}/usr/share/doc/${APT_PACKAGE}/README" <<EOF
${APT_PACKAGE} ${VERSION}

ROS package: ${ROS_PACKAGE}
Platform: Ubuntu 20.04 Focal / ROS Noetic / ${ARCHITECTURE}
Runtime requires a frozen policy file, its SHA-256, and a Session epoch.
EOF

find "${PACKAGE_ROOT}" -type d -exec chmod 0755 {} +
chmod 0644 "${PACKAGE_ROOT}/DEBIAN/control" \
  "${PACKAGE_ROOT}/usr/share/doc/${APT_PACKAGE}/README" \
  "${PACKAGE_ROOT}/usr/share/xgc2/process-definitions/xgc2-session-clock-guard.json"
test -x "${PACKAGE_ROOT}${PREFIX}/lib/${ROS_PACKAGE}/session_clock_guard_node"

rm -f "${OUTPUT_DIR}/${APT_PACKAGE}_"*.deb
fakeroot dpkg-deb --build "${PACKAGE_ROOT}" \
  "${OUTPUT_DIR}/${APT_PACKAGE}_${VERSION}_${ARCHITECTURE}.deb" >/dev/null
find "${OUTPUT_DIR}" -maxdepth 1 -type f -name "${APT_PACKAGE}_*.deb" -print | sort

#!/usr/bin/env bash
# System-level build prerequisites for hlk on macOS.
#
# vcpkg builds the C++ dependencies (CGAL, Z3, Eigen3, OpenMesh, glm and
# their transitive closure) from source, so the autotools chain has to come
# from the system package manager. Unlike Linux, no GL/X11 dev headers are
# needed: libigl's bundled GLFW build uses the macOS Cocoa backend, and
# CoMISo links against Apple's built-in Accelerate framework instead of
# OpenBLAS.

set -euo pipefail

if ! command -v brew >/dev/null 2>&1; then
    echo "Homebrew not found. Install it from https://brew.sh first."
    exit 1
fi

brew update
brew install \
    autoconf autoconf-archive automake libtool m4 pkg-config \
    bison flex nasm yasm gperf \
    cmake ninja

cat <<'EOF'

Done. Next steps:

  1. Clone vcpkg and bootstrap (one-time):
       git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
       ~/vcpkg/bootstrap-vcpkg.sh
       export VCPKG_ROOT=$HOME/vcpkg

  2. Configure and build hlk:
       cmake --preset default
       cmake --build --preset default -j

Note for Apple Silicon (arm64) hosts: vcpkg should auto-detect the
arm64-osx triplet. If it picks the wrong one, set
VCPKG_DEFAULT_TRIPLET=arm64-osx (or x64-osx for Intel) before
configuring.
EOF

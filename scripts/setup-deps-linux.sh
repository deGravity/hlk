#!/usr/bin/env bash
# System-level build prerequisites for hlk on Debian/Ubuntu.
#
# vcpkg builds the C++ dependencies (CGAL, Z3, Eigen3 and their transitive
# closure) from source, so the autotools chain and the GL/X11 dev headers
# need to come from the system. libigl downloads and builds GLFW itself
# at CMake configure time, which is why xorg-dev is required.

set -euo pipefail

sudo apt-get update
sudo apt-get install -y \
    build-essential \
    git curl zip unzip tar ca-certificates pkg-config \
    autoconf autoconf-archive automake libtool m4 \
    bison flex nasm yasm gperf \
    xorg-dev libgl1-mesa-dev libxkbcommon-dev \
    libopenblas-dev

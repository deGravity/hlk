# high-level-knitting

A coarse-to-fine design tool for machine knitting.

Three executables built from one tree:

| target        | what it is                                                       |
|---------------|------------------------------------------------------------------|
| `hlk`         | full 2-phase UI: remesh → label in one binary, in-memory handoff |
| `hlk_remesh`  | remeshing UI only (writes a quad OBJ when done)                  |
| `hlk_label`   | labeling UI only (loads a quad OBJ via File menu)                |

The unified `hlk` binary is the recommended entry point. The two
single-mode binaries are kept for debugging and for scripting a
file-based pipeline.

## Build

The C++ dependencies (CGAL, Z3, Eigen3, OpenMesh, glm) are managed via a
`vcpkg.json` manifest; libigl, libQEx, and Directional are vendored as
git submodules under `ext/`.

The first configure takes a long time on any platform — vcpkg builds
gmp / mpfr / boost / CGAL / Z3 from source, typically 15–30 minutes on a
modern laptop. Subsequent configures hit the vcpkg binary cache and are
fast.

### Linux (Ubuntu 24.04 — primary tested platform)

```bash
# 1. System prereqs (one-time):
./scripts/setup-deps-linux.sh

# 2. vcpkg (one-time):
git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
~/vcpkg/bootstrap-vcpkg.sh
export VCPKG_ROOT=$HOME/vcpkg

# 3. Clone (recursively so submodules come along) and build:
git clone --recursive git@github.com:deGravity/hlk.git
cd hlk
cmake --preset default
cmake --build --preset default -j
./build/hlk          # cwd must be build/ for resources/ to be found
```

What `setup-deps-linux.sh` installs: `build-essential`, the autotools
chain (`autoconf autoconf-archive automake libtool m4`), parser/codegen
helpers (`bison flex nasm yasm gperf`), the X11/GL development headers
that libigl's bundled GLFW build wants (`xorg-dev libgl1-mesa-dev
libxkbcommon-dev`), and `libopenblas-dev` (consumed by libigl's bundled
CoMISo).

### macOS (Intel and Apple Silicon)

```bash
# 1. Install Xcode Command Line Tools (provides Apple Clang + git):
xcode-select --install

# 2. System prereqs via Homebrew (one-time):
./scripts/setup-deps-macos.sh

# 3. vcpkg (one-time):
git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
~/vcpkg/bootstrap-vcpkg.sh
export VCPKG_ROOT=$HOME/vcpkg

# 4. Clone and build:
git clone --recursive git@github.com:deGravity/hlk.git
cd hlk
cmake --preset default
cmake --build --preset default -j
./build/hlk
```

Notes:

- vcpkg auto-selects the `x64-osx` or `arm64-osx` triplet based on the
  host. Override with `VCPKG_DEFAULT_TRIPLET=…` if needed.
- No system OpenBLAS or X11 packages are required: CoMISo links against
  Apple's Accelerate framework, and libigl's bundled GLFW uses the
  Cocoa backend on macOS.
- Apple Clang 14+ is recommended (ships with Xcode 14+).

### Windows (10 / 11, 64-bit)

```powershell
# 1. Install Visual Studio 2022 with the "Desktop development with C++"
#    workload. This provides MSVC, the Windows SDK, and CMake — all the
#    system tooling needed.

# 2. vcpkg (one-time, in PowerShell):
git clone https://github.com/microsoft/vcpkg.git $HOME\vcpkg
$HOME\vcpkg\bootstrap-vcpkg.bat
$env:VCPKG_ROOT = "$HOME\vcpkg"

# 3. Install Node.js (LTS) for the post-processing scripts in scripts/.

# 4. Clone and build:
git clone --recursive git@github.com:deGravity/hlk.git
cd hlk
cmake --preset default
cmake --build --preset default --config Release
.\build\Release\hlk.exe
```

Notes:

- vcpkg uses the `x64-windows` triplet by default and brings its own
  internal MSYS2 environment for autotools-based ports (gmp, mpfr,
  etc.) — no Cygwin or system autoconf is needed.
- The CMakeLists copies the runtime DLLs (Z3, libQEx, the bundled
  OpenBLAS that libigl/CoMISo ships, plus CoMISo's own DLL) next to
  each executable in the build output as a POST_BUILD step. If you
  see a `*.dll was not found` dialog, regenerate the build files from
  scratch (`rm -rf build && cmake --preset default`) so the post-build
  copies re-fire.
- Visual Studio 2022's CMake integration honors `CMakePresets.json`:
  open the repo as a folder and pick the `default` preset.

### Configuration presets

`CMakePresets.json` ships two configure presets:

| preset    | build dir         | build type |
|-----------|-------------------|------------|
| `default` | `build/`          | Release    |
| `debug`   | `build-debug/`    | Debug      |

Both pull `CMAKE_TOOLCHAIN_FILE` from `$env{VCPKG_ROOT}`, so as long as
that environment variable is set the same `cmake --preset` invocations
work on every platform.

## Running

The built binary expects to run with the build directory as cwd
(resources/, glyph PNGs, and the Node helpers are copied there at
configure time):

```bash
cd build && ./hlk
```

`hlk --help` lists the CLI options (`-i / -o / -r / -m`); the same
options apply to `hlk_remesh`. `hlk_label --help` shows its smaller set
(currently just `-i` for the quad OBJ to load).

## Node.js scripts

The labeling UI shells out to two Node scripts (`scripts/dsl.js` and
`scripts/knitout-to-dat.js`) when emitting machine instructions. Make
sure `node` is on your `$PATH` before exercising the
"Generate Instructions" button. The path to these scripts is baked into
the binary at compile time via `-DSCRIPTS_DIR=…`.

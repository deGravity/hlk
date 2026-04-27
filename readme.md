# high-level-knitting

A coarse-to-fine design tool for machine knitting.

## Build

The C++ dependencies (CGAL, Z3, Eigen3, …) are managed via a `vcpkg.json`
manifest; libigl, libQEx, and Directional are vendored as git submodules under
`ext/`.

### Prerequisites

1. **System build tools.** vcpkg builds dependencies from source, so a few
   things have to come from the system package manager.
   - **Linux (Debian/Ubuntu):** `scripts/setup-deps-linux.sh`
   - **macOS:** `brew install autoconf automake libtool m4 bison flex pkg-config nasm yasm gperf`
   - **Windows:** install Visual Studio 2022 with the "Desktop development
     with C++" workload (provides MSVC, the Windows SDK, and CMake).

2. **vcpkg** (any recent checkout):
   ```bash
   git clone https://github.com/microsoft/vcpkg.git ~/vcpkg
   ~/vcpkg/bootstrap-vcpkg.sh   # or .\bootstrap-vcpkg.bat on Windows
   export VCPKG_ROOT=$HOME/vcpkg
   ```

3. **Node.js** for `scripts/dsl.js` and `scripts/knitout-to-dat.js`, which the
   built binary shells out to when emitting machine code.

### Configure and build

```bash
git clone --recursive git@github.com:deGravity/hlk.git
cd hlk
export VCPKG_ROOT=$HOME/vcpkg     # whatever your vcpkg checkout is
cmake --preset default
cmake --build --preset default -j
./build/hlk                        # run from build/ so resources/ is found
```

The first configure takes a long time (vcpkg builds gmp, mpfr, boost, CGAL,
and z3 from source — typically 15–30 minutes). Subsequent configures use the
vcpkg cache and are fast.

A `debug` preset is also available: `cmake --preset debug && cmake --build --preset debug`.

### Executables

| target        | what it is                                                  |
|---------------|-------------------------------------------------------------|
| `hlk`         | full 2-phase UI: remesh → label in one binary, in-memory handoff |
| `hlk_remesh`  | remeshing UI only (writes a quad OBJ when done)             |
| `hlk_label`   | labeling UI only (loads a quad OBJ via File menu)           |

In `hlk`, the **Stage** panel at the top of the side menu switches between
modes; once a quad mesh has been extracted on the remesh side, click
*Send quad mesh -> Label* to hand it over without an OBJ roundtrip. The
two single-mode binaries are kept around for debugging and when you want
to script the file-based pipeline.

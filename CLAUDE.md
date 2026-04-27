# CLAUDE.md

This file provides guidance to Claude Code (claude.ai/code) when working with code in this repository.

## Project

`hlk` (high-level knitting) is a C++14 coarse-to-fine design tool for machine knitting. The user draws coarse labels on a quad mesh; the system solves topology/geometry constraints, refines to a per-stitch knit graph, schedules machine transfers, and emits knitout/DAT for knitting machines.

## Build

CMake + vcpkg manifest (`vcpkg.json`) for C++ deps; `ext/libigl`, `ext/libQEx`, `ext/Directional` are git submodules. No test suite, no lint config.

```bash
git submodule update --init --recursive
export VCPKG_ROOT=$HOME/vcpkg          # any recent vcpkg checkout, bootstrap once
cmake --preset default
cmake --build --preset default -j
./build/hlk                             # cwd must be build/ for resources/ to be found
```

System prereqs are not in vcpkg — install with `scripts/setup-deps-linux.sh` on Debian/Ubuntu (autotools chain + xorg/GL dev headers; the latter are needed because libigl downloads and builds GLFW). See `readme.md` for macOS / Windows equivalents.

vcpkg builds gmp/mpfr/boost/CGAL/Z3 from source on first configure (15–30 min). Subsequent configures hit the vcpkg cache.

The vcpkg-managed deps (`find_package` calls in CMakeLists.txt): `Eigen3 ≥ 3.3`, `CGAL` (config mode), `Z3` (config mode). `find_package(CGAL …)` runs *before* `find_package(LIBIGL …)` on purpose — libigl's `cmake/libigl.cmake` will download its own CGAL into `ext/libigl/external/cgal` if `CGAL::CGAL` is not already a target, and we want libigl to reuse the vcpkg CGAL.

`CMakeLists.txt` globs all `src/*.cpp`, `src/vkmp/*.cpp`, and `src/vkmp/xferplan/*.cpp` — new source files are picked up automatically, but you must re-run CMake after adding one. Everything in `resources/` is copied into the build directory at configure time; the executable expects to run with that as its working directory.

`-DSCRIPTS_DIR="<repo>/scripts"` is baked in at compile time so the built binary can shell out to Node scripts at absolute paths.

### Node / Python helpers

Both are invoked by the C++ app, not run standalone in a normal workflow:

- `scripts/dsl.js` — stitch-mesh DSL; takes a scheduled `.js` and emits `.k` (knitout).
- `scripts/knitout-to-dat.js` — converts `.k` to machine `.dat`.
- `scripts/process_texture.py <name>` — regenerates `src/<name>.h` from `resources/<name>.png` (+ optional `.txt` key file) as compile-time `Eigen::MatrixXd` glyph tables. Run from inside `scripts/`; depends on `imageio` and `numpy`.

## Entry points

Three executables, all built from `src/`:

- **`hlk`** (`src/main_hlk.cpp` + `src/unified_ui.{h,cpp}`) — the unified 2-phase UI. Single libigl viewer that hosts both `RemeshingMenu` and `LabelingUI` via a `UnifiedUI` wrapper plugin. Stage panel at the top of the side menu toggles modes; "Send quad mesh -> Label" hands `RemeshingMenu::quad_mesh` over to `LabelingUI::M` in-memory via `LabelingUI::load_quad_mesh_in_memory(V, F)`. UnifiedUI manages data-slot visibility so the off-mode UI's overlays are hidden.
- **`hlk_remesh`** (`src/main_remesh.cpp`) — remeshing UI alone. Designs a cross/frame field on a triangle mesh, reduces curl, extracts a quad mesh via MIQ / libQEx, writes a quad OBJ.
- **`hlk_label`** (`src/main_label.cpp`) — labeling UI alone. Reads a quad OBJ, paints orientations/seams/textures/size constraints, solves topology + geometry with Z3, extracts the knit graph, traces, generates machine instructions.

The unified binary is the user-facing path; the two single-mode binaries are kept for debugging and for scripting a file-based pipeline.

All three are libigl `ImGuiMenu` plugins pushed onto an `igl::opengl::glfw::Viewer`. `UnifiedUI` itself inherits `ImGuiMenu` and forwards mouse/keyboard/draw to the active child; the children are NOT pushed onto `viewer.plugins` (only `UnifiedUI` is), but their `viewer` pointer is set via their own `init()` so they can still call `viewer->data()` etc.

## Architecture

The pipeline runs coarse → fine. Each stage lives in its own layer; understanding how a change flows across layers is usually the hard part.

### 1. Triangle mesh → quad mesh (`remeshing_plugin.*`, `remeshing_field.cpp`, `remeshing_loops.cpp`, `meshing_algorithms.*`, `extract_quad_mesh.*`)

Frame/cross-field design → integer-grid parametrization (MIQ) → quad extraction (libQEx). Uses `libigl`, `Directional` (singularity/seam visualization), `CoMISo` (field solve), and CGAL. `QuadMesh` (`quad_mesh.h`) is the final output: an `(V, F_q)` pair plus derived half-edge/adjacency tables (`F_t`, `TT`, `TTi`, `VF`, `VI`, `sides_to_edges`, `edges_to_sides`, `unique_sides`, `is_singularity`). Most downstream code indexes by **side** (a directed half-edge in a quad); the helpers `quad(side)`, `flip_side`, `next_side`, `opposite_side`, `side_loop`, `dual_loop` are the vocabulary for traversal.

### 2. Labeled quad mesh (`labeled_quad_mesh.*`, `coarse_knit_mesh.*`, `coarse_knit_graph.*`)

`LabeledQuadMesh` adds a separate label-mesh overlay (`LV`, `LF`, `LUV`, per-quadrant glyph slots) that the UI paints into to show seams/orientation/shaping/textures on top of the base mesh.

`CoarseKnitMesh` (the core data structure) holds two independent Z3 `Optimizer` instances (declared first so they destruct last — do not reorder the members):

- **Topology optimizer** — assigns each side's discrete flags via `BoolProp`/`IntProp`: `is_loop` (loop vs yarn edge), `is_out` (in vs out), per-edge `stitches` counts, per-quad `time`, plus `seams` booleans. Constraints enforce loop/yarn consistency across shared sides, seam validity, and symmetry. Runs first via `optimize_topology()`.
- **Geometry optimizer** — once topology is fixed, solves for per-side stitch counts subject to size-line constraints (`size_line_constraints()`), seam costs (`get_seam_costs()`), and symmetry (`get_symmetry_constraints()`). Output is an integer stitch count on each side.

`CoarseKnitEdge`/`CoarseKnitQuad`/`CoarseKnitSide` are views over the underlying `QuadMesh` that own the Z3 props and know how to emit their own constraints (`get_topology_constraints`, `get_geometry_constraints`). `CoarseKnitMesh::get_dual()` produces a `CoarseKnitGraph` (patches + inter-patch edges) ready for fine refinement.

### 3. Patches → stitch-level `KnitGraph` (`patch.*`, `knit_graph.*`, `coarse_knit_graph.*`)

Each quad becomes a `Patch` with four sides of solved stitch counts. `Patch::make_graph` uses a `Chart` (stitch DSL parsed from a stitch DB) to populate interior stitches; `interpolate_coordinates` lays them out in 3D. `CoarseKnitGraph::build_graph()` stitches patches together along their shared coarse edges into one global `KnitGraph`.

`KnitGraph` is the fine representation: `KnitGraphNode` (a stitch position with `top`/`bottom`/`left`/`right` edges and a `texture_id`) and `KnitGraphEdge` (either loop — `LoopType`/`LoopSign` — or yarn). Two post-processing steps before scheduling:

- `contract()` — remove pass-through nodes (`KnitGraphNode::contractable()`).
- `split_doubled()` — each node actually represents two stitches (because `doubled_wales = true`); split before scheduling.

Then `trace()` (via `ak::trace_graph` on `make_row_col_graph()`) imposes a construction order, converting to `vkmp::Stitch` list. `propogate_textures()` spreads texture coordinates across connected components by BFS.

### 4. Scheduling and machine output (`vkmp/scheduler.*`, `vkmp/xferplan/*`)

The `vkmp::Scheduler` is a modified autoknit scheduler. `xferplan/` contains needle-bed transfer planning: `plan_transfers.*` with the `best_collapse` / `best_expand` / `best_shift` sub-strategies, `embed_DAG.*` for DAG embedding, `stackedplanner.*` for the stacked variant, and `Stitch`/`Shape`/`ScheduleCost`/`typeset` as its data types.

`KnitGraph::generate_instructions(filename, depth, cse)` is the final pipeline orchestrator: splits by connected component, saves each as `.st`, runs the scheduler to produce `.js`, then shells out to Node: `node <filename>.js` (dsl.js emits `.k`) then `node scripts/knitout-to-dat.js <filename>.k <filename>.dat`.

### Cross-cutting

- **`autoknit.h` (namespace `ak`)** — The autoknit-compatible data types (`ak::Stitch`, `ak::RowColGraph`, `ak::TracedStitch`) and `trace_graph` / `load_stitches` / `save_stitches`. Interop glue between `KnitGraph` and the autoknit-derived tracer.
- **`symmetrizer.*`** — detects and exploits mesh symmetry; used by both the remeshing stage and the Z3 symmetry constraints in `CoarseKnitMesh`.
- **`optimizer.*`** — thin RAII wrappers (`BoolProp`, `IntProp`) over Z3 expressions with overloaded operators so constraints read naturally (`a && b`, `a == b`). All Z3 interaction in the codebase goes through these.
- **`texture.h`, `glyphs.h`, `resources/textures/{blackandwhite,color}/`** — glyph tables are baked into headers by `process_texture.py`; add a PNG and key file to `resources/`, regenerate, and the texture becomes available at compile time.
- **Serialization** — `CoarseKnitMesh::save/load` is the full project save. `export_data()` is the `.ckm` export (recently gained texture DB and singularity info — see recent commits on `tog-revisions`). `.st` is the stitch-list format (autoknit-compatible) for intermediate scheduling state.

## Conventions worth knowing

- **Sides, not edges.** Most data structures index by *side* (oriented half-edge within a quad). `flip_side` moves across the shared edge between two quads, `next_side`/`prev_side` walk around a quad. A side of -1 means a border.
- **"Is loop" / "is out".** The canonical direction/orientation encoding everywhere: `is_loop = true` means loop edge (course), `false` means yarn (wale); `is_out = true` means outgoing from the quad.
- **Doubled wales.** By default every `KnitGraphNode` represents *two* stitches in the final fabric; `split_doubled()` must be called before scheduling. Check `doubled_wales` / `doubled` flags before writing graph-consuming code.
- **Z3 `Optimizer` members must be declared first.** They own `z3::context` instances that the `BoolProp`/`IntProp` members reference; destruction order matters.
- **The two optimizers are separate.** Topology is solved, then its results feed geometry — do not put geometric costs into the topology optimizer or vice versa.

## Submodules

All three `ext/` submodules are Anthropic-forked copies (`deGravity/hlk-*` on GitHub as of the latest commit). Pin bumps go through submodule updates, not the upstream repos.

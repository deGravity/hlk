cmake_minimum_required(VERSION 3.10)
project(hlk)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake
                      ${CMAKE_CURRENT_SOURCE_DIR}/ext/libQEx/cmake
                      ${CMAKE_MODULE_PATH})

# Get path to scripts directory for calling node with dsl.js
add_definitions(-DSCRIPTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scripts")

# LibIGL options
option(LIBIGL_WITH_PNG "Use PNG" ON)
option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use imGUI" ON)
option(LIBIGL_WITH_COMISO "Use CoMiso" ON)
# libigl 350ab15's CGAL bindings (igl/copyleft/cgal/*) use CGAL APIs that
# were removed in CGAL 6.x. We use vcpkg-managed CGAL ≥ 6 directly from our
# own translation units instead — see src/remeshing_plugin.cpp's polyhedron
# builder. The only hlk source that ever called into igl::copyleft::cgal was
# remeshing_plugin.cpp:1884 (mesh_to_polyhedron), which has been replaced.
option(LIBIGL_WITH_CGAL "Use CGAL" OFF)

# Note - many LIBIGL functions do not work as a static library
# (for example, igl::copyleft::cgal::wire_mesh), and you have less
# lee-way when creating matrices (safest to use MatrixXd and MatrixXi
# rather than explicitly declaring dimensions). Comment this out
# in case of such problems
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" ON)

# Hack for libigl linking: https://github.com/libigl/libigl/issues/1229
set(MSVC_RUNTIME "dynamic")

# Resolve Eigen3 / CGAL / Z3 / glm via vcpkg before bringing in libigl,
# so that libigl's CMake scripts skip their own download paths and reuse
# our targets.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(CGAL CONFIG REQUIRED COMPONENTS Core)
find_package(Z3 REQUIRED CONFIG)
find_package(glm CONFIG REQUIRED)

# All of the libigl + Directional patches that hlk used to apply at
# configure time are now part of the submodules' integration-dev branches:
#   - libigl: cstdint includes + reachable CoMISo URL (libigl/CoMISo HEAD)
#   - Directional: igl::PI -> M_PI, write_singularities typo, IGL_INLINE
#     -> inline on directional::tree (ODR safety)
# Bumping the submodule pins is enough; no in-tree CMake patching needed.

find_package(LIBIGL REQUIRED)

add_subdirectory(ext/libQEx EXCLUDE_FROM_ALL)
target_compile_definitions(QEx PRIVATE _USE_MATH_DEFINES)

###############################################################################
# Source layout
###############################################################################
#
# Three internal libraries plus two thin executables:
#
#   hlk_core          shared core (quad mesh, optimizer, symmetrizer, …)
#   hlk_remesh_lib    remeshing UI + CGAL/MIQ pipeline
#   hlk_label_lib     labeling UI + downstream knit graph + scheduler
#   hlk_remesh        executable: just main_remesh.cpp + hlk_remesh_lib + core
#   hlk_label         executable: just main_label.cpp  + hlk_label_lib  + core
#
# The two executables share hlk_core and never link each other's libraries.

set(HLK_CORE_SOURCES
    src/quad_mesh.cpp src/quad_mesh.h
    src/read_quad_mesh.cpp src/read_quad_mesh.h
    src/optimizer.cpp src/optimizer.h
    src/symmetrizer.cpp src/symmetrizer.h
    src/serialization.cpp src/serialization.h)

set(HLK_REMESH_SOURCES
    src/cgal_wrapper.cpp src/cgal_wrapper.h
    src/extract_quad_mesh.cpp src/extract_quad_mesh.h
    src/meshing_algorithms.cpp src/meshing_algorithms.h
    src/remeshing_plugin.cpp src/remeshing_plugin.h
    src/remeshing_field.cpp
    src/remeshing_loops.cpp
    src/remeshing_utils.cpp src/remeshing_utils.h
    src/stack_layers.cpp src/stack_layers.h)

set(HLK_LABEL_SOURCES
    src/labeled_quad_mesh.cpp src/labeled_quad_mesh.h
    src/coarse_knit_mesh.cpp src/coarse_knit_mesh.h
    src/coarse_knit_graph.cpp src/coarse_knit_graph.h
    src/glyph.cpp src/glyph.h src/glyphs.h
    src/texture.cpp src/texture.h
    src/disjointset.cpp src/disjointset.h
    src/graph_vis.cpp src/graph_vis.h
    src/grid.cpp src/grid.h
    src/patch.cpp src/patch.h
    src/knit_graph.cpp src/knit_graph.h
    src/autoknit.cpp src/autoknit.h
    src/labeling_ui.cpp src/labeling_ui.h
    src/vkmp/scheduler.cpp src/vkmp/scheduler.hpp
    src/vkmp/driver.cpp
    src/vkmp/xferplan/embed_DAG.cpp src/vkmp/xferplan/embed_DAG.hpp
    src/vkmp/xferplan/plan_transfers.cpp src/vkmp/xferplan/plan_transfers.hpp
    src/vkmp/xferplan/plan_transfers-best_collapse.cpp
    src/vkmp/xferplan/plan_transfers-best_expand.cpp
    src/vkmp/xferplan/plan_transfers-best_shift.cpp
    src/vkmp/xferplan/plan_transfers-draw_beds.cpp
    src/vkmp/xferplan/plan_transfers-helpers.hpp
    src/vkmp/xferplan/plan_transfers-minimize_winding.cpp
    src/vkmp/xferplan/plan_transfers-run_transfers.cpp
    src/vkmp/xferplan/ScheduleCost.cpp src/vkmp/xferplan/ScheduleCost.hpp
    src/vkmp/xferplan/Shape.hpp
    src/vkmp/xferplan/stackedplanner.cpp src/vkmp/xferplan/stackedplanner.h
    src/vkmp/xferplan/Stitch.cpp src/vkmp/xferplan/Stitch.hpp
    src/vkmp/xferplan/typeset.hpp)

file(GLOB HLK_NATVIS ${PROJECT_SOURCE_DIR}/src/*.natvis)

add_library(hlk_core STATIC ${HLK_CORE_SOURCES})
target_include_directories(hlk_core PUBLIC
    ${PROJECT_SOURCE_DIR}/src
    ${Z3_CXX_INCLUDE_DIRS}
    ext/Directional/include)
target_link_libraries(hlk_core PUBLIC
    Eigen3::Eigen
    igl::core
    igl::png
    ${Z3_LIBRARIES})
target_compile_definitions(hlk_core PUBLIC _USE_MATH_DEFINES)
target_compile_features(hlk_core PUBLIC cxx_std_14)

add_library(hlk_remesh_lib STATIC ${HLK_REMESH_SOURCES})
target_link_libraries(hlk_remesh_lib PUBLIC
    hlk_core
    CGAL::CGAL
    igl::opengl
    igl::opengl_glfw
    igl::opengl_glfw_imgui
    igl::comiso
    QEx)
target_include_directories(hlk_remesh_lib PUBLIC ${QEX_INCLUDE_DIRS})

add_library(hlk_label_lib STATIC ${HLK_LABEL_SOURCES})
target_link_libraries(hlk_label_lib PUBLIC
    hlk_core
    glm::glm
    igl::opengl
    igl::opengl_glfw
    igl::opengl_glfw_imgui)

# Executables — each is just a thin main() linking its UI lib.
add_executable(hlk_remesh src/main_remesh.cpp ${HLK_NATVIS})
target_link_libraries(hlk_remesh PRIVATE hlk_remesh_lib)

add_executable(hlk_label src/main_label.cpp ${HLK_NATVIS})
target_link_libraries(hlk_label PRIVATE hlk_label_lib)

# Combined 2-phase UI: remesh + label in one binary, with in-memory
# quad-mesh handoff. unified_ui.cpp is the only source that touches both
# lib APIs, so it lives in the executable target rather than in either lib.
add_executable(hlk src/main_hlk.cpp src/unified_ui.cpp src/unified_ui.h ${HLK_NATVIS})
target_link_libraries(hlk PRIVATE hlk_remesh_lib hlk_label_lib)

# Copy runtime resources to binary directory (icons, glyph PNGs, etc.)
file(GLOB RESOURCE_FILES ${PROJECT_SOURCE_DIR}/resources/*)
foreach(RESOURCE_FILE ${RESOURCE_FILES})
  file(COPY ${RESOURCE_FILE} DESTINATION ${PROJECT_BINARY_DIR})
endforeach(RESOURCE_FILE)

# Copy shared libraries to binary directory on Windows
if("${CMAKE_SYSTEM_NAME}" MATCHES "[Ww]indows")
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT hlk)
  foreach(_target hlk_remesh hlk_label hlk)
    igl_copy_imported_dll(CoMISo ${_target})
    foreach (z3_lib ${Z3_LIBRARIES})
      add_custom_command(TARGET ${_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${z3_lib}> $<TARGET_FILE_DIR:${_target}>)
    endforeach()
    foreach (qex_lib ${QEX_LIBRARIES})
      add_custom_command(TARGET ${_target} POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E copy_if_different
                $<TARGET_FILE:${qex_lib}> $<TARGET_FILE_DIR:${_target}>)
    endforeach()
  endforeach()

  set(COMISO_OPENBLAS_DLL_LOC "${LIBIGL_INCLUDE_DIR}/../external/CoMISo/ext/OpenBLAS-v0.2.14-Win64-int64/bin")
  file(GLOB COMISO_OPENBLAS_DLLS ${COMISO_OPENBLAS_DLL_LOC}/*.dll)
  foreach (openblas_lib ${COMISO_OPENBLAS_DLLS})
    file(COPY ${openblas_lib} DESTINATION ${PROJECT_BINARY_DIR})
  endforeach()
endif()

source_group("hlk_core"        FILES ${HLK_CORE_SOURCES})
source_group("hlk_remesh_lib"  FILES ${HLK_REMESH_SOURCES})
source_group("hlk_label_lib"   FILES ${HLK_LABEL_SOURCES})

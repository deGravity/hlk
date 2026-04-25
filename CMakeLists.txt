cmake_minimum_required(VERSION 3.10)
project(hlk)

set(PROJECT_NAME hlk)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake
                      ${CMAKE_CURRENT_SOURCE_DIR}/ext/libQEx/cmake
                      ${CMAKE_MODULE_PATH})

# Get path to scripts directory for calling node with dsl.js
add_definitions(-DSCRIPTS_DIR="${CMAKE_CURRENT_SOURCE_DIR}/scripts")

# LibIGL options - may neet to use option instead of set
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

# Resolve Eigen3 and CGAL via vcpkg (or system) before bringing in libigl,
# so that libigl's CMake scripts skip their own download paths and reuse
# our targets. Z3 is a separate find_package call below for symmetry.
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(CGAL CONFIG REQUIRED COMPONENTS Core)
find_package(Z3 REQUIRED CONFIG)
find_package(glm CONFIG REQUIRED)

# Patch libigl to use a reachable CoMISo source.
#
# The vendored libigl submodule's cmake/LibiglDownloadExternal.cmake hardcodes
# CoMISo at git@gitlab.cs.washington.edu:knitting-research/external/libigl-comiso.git
# at SHA 90e795b6, which is no longer accessible. Patch the submodule file in
# place to point at the public libigl/CoMISo.
set(HLK_COMISO_GIT_URL "https://github.com/libigl/CoMISo.git" CACHE STRING
    "Git URL libigl will clone CoMISo from")
set(HLK_COMISO_GIT_TAG "562efe333edc8e649dc101469614f43378b1eb55" CACHE STRING
    "CoMISo commit libigl will check out (master HEAD as of 2026-04-24)")

set(_hlk_dl_cmake "${CMAKE_CURRENT_SOURCE_DIR}/ext/libigl/cmake/LibiglDownloadExternal.cmake")
file(READ "${_hlk_dl_cmake}" _hlk_dl_content)
string(REPLACE
    "git@gitlab.cs.washington.edu:knitting-research/external/libigl-comiso.git"
    "${HLK_COMISO_GIT_URL}"
    _hlk_dl_content_new "${_hlk_dl_content}")
string(REPLACE
    "90e795b6662c4ceccc2ab98093e8b69aa9bb0478"
    "${HLK_COMISO_GIT_TAG}"
    _hlk_dl_content_new "${_hlk_dl_content_new}")
if(NOT _hlk_dl_content STREQUAL _hlk_dl_content_new)
    message(STATUS "Patching ext/libigl/cmake/LibiglDownloadExternal.cmake to point CoMISo at ${HLK_COMISO_GIT_URL}")
    file(WRITE "${_hlk_dl_cmake}" "${_hlk_dl_content_new}")
endif()

# libigl 350ab15 predates GCC 13's stricter handling of transitive STL
# includes; a handful of its files use stdint types without #include <cstdint>.
foreach(_hlk_libigl_patch
    "ext/libigl/include/igl/sparse_voxel_grid.cpp"
    "ext/libigl/include/igl/Timer.h"
    "ext/libigl/include/igl/opengl/MeshGL.h")
    set(_hlk_path "${CMAKE_CURRENT_SOURCE_DIR}/${_hlk_libigl_patch}")
    if(EXISTS "${_hlk_path}")
        file(READ "${_hlk_path}" _hlk_content)
        if(NOT _hlk_content MATCHES "(cstdint|stdint\\.h)")
            message(STATUS "Patching ${_hlk_libigl_patch} to add #include <cstdint>")
            file(WRITE "${_hlk_path}" "#include <cstdint>\n${_hlk_content}")
        endif()
    endif()
endforeach()

find_package(LIBIGL REQUIRED)

add_subdirectory(ext/libQEx EXCLUDE_FROM_ALL)
target_compile_definitions(QEx PRIVATE _USE_MATH_DEFINES)

# Just grab all C++ files
file(GLOB SRC_FILES ${PROJECT_SOURCE_DIR}/src/*.cpp)
file(GLOB HEADER_FILES ${PROJECT_SOURCE_DIR}/src/*.h)
file(GLOB DEBUG_FILES ${PROJECT_SOURCE_DIR}/src/*.natvis)

file(GLOB VKMP_SRC ${PROJECT_SOURCE_DIR}/src/vkmp/*.cpp)
file(GLOB XFERPLAN_SRC ${PROJECT_SOURCE_DIR}/src/vkmp/xferplan/*.cpp)
file(GLOB VKMP_HEADER ${PROJECT_SOURCE_DIR}/src/vkmp/*.hpp)
file(GLOB XFERPLAN_HEADER ${PROJECT_SOURCE_DIR}/src/vkmp/xferplan/*.h*)

add_executable(${PROJECT_NAME} 
	${SRC_FILES} 
	${HEADER_FILES} 
	${DEBUG_FILES} 
	${VKMP_SRC} 
	${XFERPLAN_SRC} 
	${VKMP_HEADER} 
	${XFERPLAN_HEADER})

target_include_directories(${PROJECT_NAME} PRIVATE
	${Z3_CXX_INCLUDE_DIRS}
	${QEX_INCLUDE_DIRS}
	ext/Directional/include)

target_link_libraries(${PROJECT_NAME}
	Eigen3::Eigen
	CGAL::CGAL
	glm::glm
	igl::core
	igl::opengl
	igl::opengl_glfw
	igl::png
	igl::opengl_glfw_imgui
	igl::comiso
	${Z3_LIBRARIES}
	QEx)

target_compile_definitions(${PROJECT_NAME} PUBLIC _USE_MATH_DEFINES)
target_compile_features(${PROJECT_NAME} PRIVATE cxx_std_14)

# Copy Runtime Resources to Binary Directory
file(GLOB RESOURCE_FILES ${PROJECT_SOURCE_DIR}/resources/*)
foreach(RESOURCE_FILE ${RESOURCE_FILES})
  file(COPY ${RESOURCE_FILE} DESTINATION ${PROJECT_BINARY_DIR})
endforeach(RESOURCE_FILE)

# Copy shared libraries to binary directory on Windows
if("${CMAKE_SYSTEM_NAME}" MATCHES "[Ww]indows")
  set_property(DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR} PROPERTY VS_STARTUP_PROJECT ${PROJECT_NAME})
  igl_copy_imported_dll(CoMISo stitchgraph)
  foreach (z3_lib ${Z3_LIBRARIES})
    message(STATUS "Adding copy rule for ${z3_lib}")
    add_custom_command(TARGET ${PROJECT_NAME}
      POST_BUILD
      COMMAND
        ${CMAKE_COMMAND} -E copy_if_different
        $<TARGET_FILE:${z3_lib}>
        $<TARGET_FILE_DIR:${PROJECT_NAME}>
    )
  endforeach()

  foreach (qex_lib ${QEX_LIBRARIES})
    message(STATUS "Adding copy rule for ${qex_lib}")
	add_custom_command(TARGET ${PROJECT_NAME}
	  POST_BUILD
	  COMMAND
	    ${CMAKE_COMMAND} -E copy_if_different
		$<TARGET_FILE:${qex_lib}>
		$<TARGET_FILE_DIR:${PROJECT_NAME}>
	)
  endforeach()
  
  set(COMISO_OPENBLAS_DLL_LOC "${LIBIGL_INCLUDE_DIR}/../external/CoMISo/ext/OpenBLAS-v0.2.14-Win64-int64/bin")
  file(GLOB COMISO_OPENBLAS_DLLS ${COMISO_OPENBLAS_DLL_LOC}/*.dll)
  foreach (openblas_lib ${COMISO_OPENBLAS_DLLS})
	message(STATUS "Copying ${openblas_lib} to build directory.")
	file(COPY ${openblas_lib} DESTINATION ${PROJECT_BINARY_DIR})
  endforeach()
  
endif()

source_group("Header Files" FILES ${HEADER_FILES})
source_group("Source Files" FILES ${SRC_FILES})
source_group("Header Files\\vkmp" FILES ${VKMP_HEADER})
source_group("Source Files\\vkmp" FILES ${VKMP_SRC})
source_group("Header Files\\vkmp\\xferplan" FILES ${XFERPLAN_HEADER})
source_group("Source Files\\vkmp\\xferplan" FILES ${XFERPLAN_SRC})

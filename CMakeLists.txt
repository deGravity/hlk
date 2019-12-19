cmake_minimum_required(VERSION 3.10)
project(hlk)

set(PROJECT_NAME hlk)

set(CMAKE_MODULE_PATH ${CMAKE_CURRENT_SOURCE_DIR}/cmake
                      ${CMAKE_CURRENT_SOURCE_DIR}/ext/libQEx/cmake
                      ${CMAKE_MODULE_PATH})

# LibIGL options - may neet to use option instead of set
option(LIBIGL_WITH_PNG "Use PNG" ON)
option(LIBIGL_WITH_OPENGL "Use OpenGL" ON)
option(LIBIGL_WITH_OPENGL_GLFW "Use GLFW" ON)
option(LIBIGL_WITH_OPENGL_GLFW_IMGUI "Use imGUI" ON)
option(LIBIGL_WITH_COMISO "Use CoMiso" ON)
option(LIBIGL_WITH_CGAL "Use CGAL" ON)

# Note - many LIBIGL functions do not work as a static library
# (for example, igl::copyleft::cgal::wire_mesh), and you have less
# lee-way when creating matrices (safest to use MatrixXd and MatrixXi
# rather than explicitly declaring dimensions). Comment this out
# in case of such problems
option(LIBIGL_USE_STATIC_LIBRARY "Use libigl as static library" ON)

# Hack for libigl linking: https://github.com/libigl/libigl/issues/1229
set(MSVC_RUNTIME "dynamic")

find_package(LIBIGL REQUIRED)
find_package(Eigen3 3.3 REQUIRED NO_MODULE)
find_package(Z3 REQUIRED CONFIG)

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

target_include_directories(${PROJECT_NAME} PRIVATE ${Z3_CXX_INCLUDE_DIRS} ${QEX_INCLUDE_DIRS} ext/Directional/include)

target_link_libraries(${PROJECT_NAME} 
	Eigen3::Eigen 
	CGAL::CGAL
	igl::cgal
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

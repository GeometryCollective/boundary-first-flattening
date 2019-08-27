cmake_minimum_required(VERSION 3.1.0)
project(bff)

option(BUILD_CLI  "Build BFF command line" ON)
option(BUILD_GUI  "Build BFF GUI"          ON)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")

# suitesparse
# ------------------------------------------------------------------
# Detect SuiteSparse libraries:
# If not found automatically, set SuiteSparse_DIR in CMake to the
# directory where SuiteSparse was built.
# ------------------------------------------------------------------
set(SuiteSparse_USE_LAPACK_BLAS ON)
find_package(SuiteSparse QUIET NO_MODULE)  # 1st: Try to locate the *config.cmake file.
if(NOT SuiteSparse_FOUND)
	set(SuiteSparse_VERBOSE ON)
	find_package(SuiteSparse REQUIRED) # 2nd: Use FindSuiteSparse.cmake module
endif()
MESSAGE(STATUS "SuiteSparse_LIBS: ${SuiteSparse_LIBRARIES}")

# bff lib
list(APPEND BFF_INCLUDE_DIRS ${SuiteSparse_INCLUDE_DIRS})
list(APPEND BFF_INCLUDE_DIRS "deps/rectangle-bin-pack/include")
list(APPEND BFF_INCLUDE_DIRS "linear-algebra/include")
list(APPEND BFF_INCLUDE_DIRS "mesh/include")
list(APPEND BFF_INCLUDE_DIRS "project/include")
include_directories(${BFF_INCLUDE_DIRS})

file(GLOB BFF_SOURCES "deps/rectangle-bin-pack/src/*.cpp" "linear-algebra/src/*.cpp" "mesh/src/*.cpp" "project/src/*.cpp")

# create bff static library
add_library(bff ${BFF_SOURCES})
target_link_libraries(bff ${SuiteSparse_LIBRARIES})

if (BUILD_CLI)
	# build command line app
	file(GLOB COMMAND_LINE_SOURCES "command-line/*.cpp")

	add_executable(bff-command-line ${COMMAND_LINE_SOURCES})
	target_link_libraries(bff-command-line bff)
endif()

if (BUILD_GUI)
	# nanogui
	set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
	set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
	set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
	add_subdirectory(deps/nanogui)
	add_definitions(${NANOGUI_EXTRA_DEFS})

	# viewer
	list(APPEND VIEWER_INCLUDE_DIRS ${NANOGUI_EXTRA_INCS})
	list(APPEND VIEWER_INCLUDE_DIRS "deps/nanogui/include")
	list(APPEND VIEWER_INCLUDE_DIRS "deps/glm")
	list(APPEND VIEWER_INCLUDE_DIRS "viewer/include")
	include_directories(${VIEWER_INCLUDE_DIRS})

	file(GLOB VIEWER_SOURCES "viewer/src/*.cpp")

	# build viewer app
	add_executable(bff-viewer ${VIEWER_SOURCES})
	target_link_libraries(bff-viewer bff nanogui ${NANOGUI_EXTRA_LIBS})
endif()

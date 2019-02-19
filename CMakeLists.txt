cmake_minimum_required(VERSION 3.1.0)
project(bff)

option(BUILD_CLI  "Build BFF command line" ON)
option(BUILD_GUI  "Build BFF GUI"          ON)
option(WITH_OPENMESH_INTERFACE "Build with OpenMesh interface" OFF)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_WINDOWS_EXPORT_ALL_SYMBOLS ON)
set(CMAKE_DEBUG_POSTFIX "-debug")
set_property(GLOBAL PROPERTY USE_FOLDERS ON)
include(CMakePackageConfigHelpers)
include(GNUInstallDirs)


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

file(GLOB BFF_SOURCES
	"deps/rectangle-bin-pack/src/*.cpp" "deps/rectangle-bin-pack/include/*.h"
	"linear-algebra/src/*.cpp" "linear-algebra/include/*.h"
	"mesh/src/*.cpp" "mesh/include/*.h"
	"project/src/*.cpp" "project/include/*.h")

if(${WITH_OPENMESH_INTERFACE})
	find_package(OpenMesh REQUIRED)
	add_library(bff-openmesh-interface INTERFACE)
	target_link_libraries(bff-openmesh-interface INTERFACE
		bff
		${OPENMESH_LIBRARIES}
	)
	target_include_directories(bff-openmesh-interface INTERFACE
		${OPENMESH_INCLUDE_DIR}
	)
endif()

# create bff static library
add_library(bff ${BFF_SOURCES})
target_link_libraries(bff ${SuiteSparse_LIBRARIES})
if(${WITH_OPENMESH_INTERFACE})
	target_link_libraries(bff
		PUBLIC
		${OPENMESH_LIBRARIES}
		openmesh_structures
	)
	target_include_directories(bff
		PUBLIC
		${OPENMESH_INCLUDE_DIR}
	)
endif()

target_include_directories(bff
	PRIVATE
    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/deps/rectangle-bin-pack/include>
	PUBLIC
	${SuiteSparse_INCLUDE_DIRS}
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/linear-algebra/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/mesh/include>
	$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/project/include>
	$<INSTALL_INTERFACE:include>
)
install(TARGETS bff EXPORT bff-targets)


if (BUILD_CLI)
	# build command line app
	file(GLOB COMMAND_LINE_SOURCES "command-line/*.cpp")

	add_executable(bff-command-line ${COMMAND_LINE_SOURCES})
	target_link_libraries(bff-command-line bff)
	install(TARGETS bff-command-line DESTINATION ${CMAKE_INSTALL_BINDIR})
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

	file(GLOB VIEWER_SOURCES "viewer/src/*.cpp" "viewer/include/*.h")

	# build viewer app
	add_executable(bff-viewer ${VIEWER_SOURCES})
	target_link_libraries(bff-viewer bff nanogui ${NANOGUI_EXTRA_LIBS})

	target_include_directories(bff-viewer PRIVATE
		$<BUILD_INTERFACE:${VIEWER_INCLUDE_DIRS}>
	    $<BUILD_INTERFACE:${NANOGUI_EXTRA_INCS}>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/deps/nanogui/include>
	    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/deps/nanogui/ext/glad/include>
		$<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/deps/glm>
	    $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/viewer/include>
	)
	target_compile_definitions(bff-viewer PRIVATE ${NANOGUI_EXTRA_DEFS})
	install(TARGETS bff-viewer DESTINATION ${CMAKE_INSTALL_BINDIR})
endif()

configure_package_config_file(bffConfig.cmake.in ${CMAKE_CURRENT_BINARY_DIR}/bffConfig.cmake INSTALL_DESTINATION ${CMAKE_INSTALL_PREFIX})

install(DIRECTORY mesh/include/ DESTINATION include FILES_MATCHING PATTERN "*.h")
install(DIRECTORY linear-algebra/include/ DESTINATION include FILES_MATCHING PATTERN "*.h")
install(DIRECTORY project/include/ DESTINATION include FILES_MATCHING PATTERN "*.h")
install(DIRECTORY mesh/include/ DESTINATION include FILES_MATCHING PATTERN "*.inl")
install(DIRECTORY linear-algebra/include/ DESTINATION include FILES_MATCHING PATTERN "*.inl")
install(DIRECTORY project/include/ DESTINATION include FILES_MATCHING PATTERN "*.inl")
if(${WITH_OPENMESH_INTERFACE})
	install(DIRECTORY openmesh-interface/include/ DESTINATION include FILES_MATCHING PATTERN "*.hpp")
endif()

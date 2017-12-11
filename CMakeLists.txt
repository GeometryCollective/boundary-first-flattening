cmake_minimum_required(VERSION 3.1.0)
project(bff)

set(CMAKE_CXX_STANDARD 11)
set(CMAKE_CXX_STANDARD_REQUIRED ON)

# dependencies
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${CMAKE_SOURCE_DIR}/cmake")
find_package(SuiteSparse REQUIRED)

set(NANOGUI_BUILD_EXAMPLE OFF CACHE BOOL " " FORCE)
set(NANOGUI_BUILD_PYTHON  OFF CACHE BOOL " " FORCE)
set(NANOGUI_INSTALL       OFF CACHE BOOL " " FORCE)
add_subdirectory(deps/nanogui)
add_definitions(${NANOGUI_EXTRA_DEFS})

include_directories(${CHOLMOD_INCLUDE_DIR} ${NANOGUI_EXTRA_INCS} deps/nanogui/include deps/glm)

# build executable
include_directories(linear-algebra/include mesh/include project/include viewer/include)
file(GLOB SOURCES "linear-algebra/src/*.cpp" "mesh/src/*.cpp" "project/src/*.cpp" "viewer/src/*.cpp")

add_executable(bff ${SOURCES})

target_link_libraries(bff ${LAPACK_LIBRARIES} ${CHOLMOD_LIBRARY} nanogui ${NANOGUI_EXTRA_LIBS})

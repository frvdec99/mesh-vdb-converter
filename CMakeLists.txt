cmake_minimum_required(VERSION 3.18)

project(mesh-vdb-converter)

set(OPENVDB_CORE_STATIC OFF)
add_subdirectory(openvdb)

set(OpenVDB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/openvdb/openvdb")
include_directories(${OpenVDB_INCLUDE_DIR})
include_directories("${CMAKE_BINARY_DIR}/openvdb/openvdb/openvdb/")
include_directories("${CMAKE_BINARY_DIR}/openvdb/openvdb/openvdb/openvdb/")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/openvdb/cmake/")
find_package(OpenVDB REQUIRED)

add_executable(converter converter.cpp)
target_link_libraries(converter OpenVDB::openvdb)
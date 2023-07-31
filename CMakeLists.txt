cmake_minimum_required(VERSION 3.18)

project(converter)
add_compile_options(-bigobj)

set(OPENVDB_CORE_STATIC OFF)
add_subdirectory(openvdb)

set(OpenVDB_INCLUDE_DIR "${CMAKE_CURRENT_SOURCE_DIR}/openvdb/openvdb")
include_directories(${OpenVDB_INCLUDE_DIR})
include_directories("${CMAKE_BINARY_DIR}/openvdb/openvdb/openvdb/")
include_directories("${CMAKE_BINARY_DIR}/openvdb/openvdb/openvdb/openvdb/")

include_directories("${CMAKE_CURRENT_SOURCE_DIR}/include")

list(APPEND CMAKE_MODULE_PATH "${CMAKE_CURRENT_SOURCE_DIR}/openvdb/cmake/")
find_package(OpenVDB REQUIRED)

add_executable(meshToVolume meshToVolume.cpp)
target_link_libraries(meshToVolume OpenVDB::openvdb)

add_executable(vdbToMitsuba vdbToMitsuba.cpp)
target_link_libraries(vdbToMitsuba OpenVDB::openvdb)

add_executable(ndf ndf.cpp)
target_link_libraries(ndf OpenVDB::openvdb)
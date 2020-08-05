cmake_minimum_required(VERSION 3.10)

project(benchmark)

add_executable(benchmark benchmark.cpp)

include(ExternalProject)

# add monoDrive c++ client as external project
ExternalProject_Add(
    monodrive-external
	GIT_REPOSITORY https://github.com/monoDriveIO/monodrive-client.git
	GIT_TAG "development"
    PREFIX "${CMAKE_BINARY_DIR}/monodrive"
    SOURCE_SUBDIR "monodrive/core"
    INSTALL_COMMAND ""
    LOG_DOWNLOAD ON
)

# create shared library for monoDrive c++ client
SET(MD_INCLUDE_DIR ${CMAKE_BINARY_DIR}/monodrive/src/monodrive-external/monodrive/core/src)
file(MAKE_DIRECTORY ${MD_INCLUDE_DIR})
SET(MD_LIB_DIR ${CMAKE_BINARY_DIR}/monodrive/src/monodrive-external-build)
add_library(monodrive SHARED IMPORTED)
set_target_properties(
    monodrive PROPERTIES
    IMPORTED_LOCATION ${MD_LIB_DIR}/libmonodrive.so
    INTERFACE_INCLUDE_DIRECTORIES ${MD_INCLUDE_DIR}
)
add_dependencies(monodrive monodrive-external)

# TODO // don't think we should need to include this explicitly here...
find_package(Boost 1.65.0 REQUIRED COMPONENTS system)
include_directories(benchmark ${Boost_INCLUDE_DIRS})
link_directories(benchmark ${Boost_LIBRARY_DIRS})
target_link_libraries(benchmark ${Boost_LIBRARIES})

find_package (Threads)
target_link_libraries (benchmark ${CMAKE_THREAD_LIBS_INIT})

# link to monoDrive c++ client
target_link_libraries(benchmark monodrive)

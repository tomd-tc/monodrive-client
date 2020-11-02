cmake_minimum_required(VERSION 3.10)

project(opendrive)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(opendrive opendrive.cpp)

target_link_libraries(opendrive monodrive)

include_directories(${EIGEN3_INCLUDE_DIR})

# copy dll to runtime directory
if(MSVC)
  add_custom_target(monodrive_lib_copy_opendrive)
  add_custom_command(TARGET monodrive_lib_copy_opendrive POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:monodrive>
      ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/monodrive.dll
  )
  add_dependencies(opendrive monodrive_lib_copy_opendrive)
endif(MSVC)

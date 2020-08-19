cmake_minimum_required(VERSION 3.10)

project(distributed)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(distributed distributed.cpp  distributed_server.cpp)

target_link_libraries(distributed monodrive)

# copy dll to runtime directory
if(MSVC)
  add_custom_target(monodrive_lib_copy_distributed)
  add_custom_command(TARGET monodrive_lib_copy_distributed POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:monodrive>
      ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/monodrive.dll
  )
  add_dependencies(distributed monodrive_lib_copy_distributed)
endif(MSVC)

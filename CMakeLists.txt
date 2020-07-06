cmake_minimum_required(VERSION 3.10)

project(monodrive_clients)

add_subdirectory(monodrive)
if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7)
  add_subdirectory(examples)
endif()

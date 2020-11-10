cmake_minimum_required(VERSION 3.10)
include(ProcessorCount)

project(monodrive_clients)

add_subdirectory(monodrive)
if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7)
  add_subdirectory(examples)
endif()

if(MSVC)
  ProcessorCount(PROC_COUNT)
  if(NOT PROC_COUNT EQUAL 0) 
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP${PROC_COUNT}")
  endif()
endif()
 

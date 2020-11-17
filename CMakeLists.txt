cmake_minimum_required(VERSION 3.10)

project(monodrive_clients)

# Enable multi-core building in VS/Code
include(ProcessorCount)
if(MSVC)
  ProcessorCount(PROC_COUNT)
  # This explicitly sets the job count because the current CMake interface -j8 
  # or --parllel does not work with MSVC. We can fix this when this issue is 
  # resolved: https://gitlab.kitware.com/cmake/cmake/-/issues/20564 
  if(NOT PROC_COUNT EQUAL 0) 
      set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} /MP${PROC_COUNT}")
  endif()
endif()

add_subdirectory(monodrive)
if(CMAKE_CXX_COMPILER_VERSION VERSION_GREATER 7)
  add_subdirectory(examples)
endif()

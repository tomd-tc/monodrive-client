cmake_minimum_required(VERSION 3.10)

project(multi_process_example)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(sim_control simulator_controller.cpp)
add_executable(vehicle_control vehicle_controller.cpp)
add_executable(video_display video_display.cpp)

target_link_libraries(sim_control monodrive)
target_link_libraries(vehicle_control monodrive)
target_link_libraries(video_display monodrive)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build" "C:/opencv/build")
include_directories(${OpenCV_INCLUDE_DIRS})

target_include_directories(video_display PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(video_display ${OpenCV_LIBRARIES})
target_link_libraries(video_display ${OpenCV_LIBRARIES})

# copy dll to runtime directory
if(MSVC)
  add_custom_target(monodrive_lib_copy_multiprocess)
  add_custom_command(TARGET monodrive_lib_copy_multiprocess POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:monodrive>
      ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/monodrive.dll
  )
  add_dependencies(sim_control monodrive_lib_copy_multiprocess)
  add_dependencies(vehicle_control monodrive_lib_copy_multiprocess)
  add_dependencies(video_display monodrive_lib_copy_multiprocess)
endif(MSVC)

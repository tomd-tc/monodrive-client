cmake_minimum_required(VERSION 3.10)

project(lane_follower)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(fixed_step fixed_step.cpp)
add_executable(fixed_step_lidar fixed_step_lidar.cpp)
add_executable(real_time real_time.cpp)
add_executable(waypoints waypoints.cpp)

target_link_libraries(fixed_step monodrive)
target_link_libraries(fixed_step_lidar monodrive)
target_link_libraries(real_time monodrive)
target_link_libraries(waypoints monodrive)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build" "C:/opencv/build")
include_directories(${OpenCV_INCLUDE_DIRS})

target_include_directories(fixed_step PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(fixed_step ${OpenCV_LIBRARIES})
target_link_libraries(fixed_step ${OpenCV_LIBRARIES})

target_include_directories(fixed_step_lidar PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(fixed_step_lidar ${OpenCV_LIBRARIES})
target_link_libraries(fixed_step_lidar ${OpenCV_LIBRARIES})

target_include_directories(real_time PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(real_time ${OpenCV_LIBRARIES})
target_link_libraries(real_time ${OpenCV_LIBRARIES})

target_include_directories(waypoints PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(waypoints ${OpenCV_LIBRARIES})
target_link_libraries(waypoints ${OpenCV_LIBRARIES})

# copy dll to runtime directory
if(MSVC)
  add_custom_target(monodrive_lib_copy_lanes)
  add_custom_command(TARGET monodrive_lib_copy_lanes POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:monodrive>
      ${CMAKE_CURRENT_BINARY_DIR}/$<CONFIG>/monodrive.dll
  )
  add_dependencies(fixed_step monodrive_lib_copy_lanes)
  add_dependencies(fixed_step_lidar monodrive_lib_copy_lanes)
  add_dependencies(real_time monodrive_lib_copy_lanes)
  add_dependencies(waypoints monodrive_lib_copy_lanes)
endif(MSVC)

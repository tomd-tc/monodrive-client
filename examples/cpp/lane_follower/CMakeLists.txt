cmake_minimum_required(VERSION 3.10)

project(lane_follower)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(fixed_step fixed_step.cpp)
add_executable(real_time real_time.cpp)

target_link_libraries(fixed_step monodrive)
target_link_libraries(real_time monodrive)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build" "C:/opencv/build")
include_directories(${OpenCV_INCLUDE_DIRS})

target_include_directories(fixed_step PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(fixed_step ${OpenCV_LIBRARIES})
target_link_libraries(fixed_step ${OpenCV_LIBRARIES})

target_include_directories(real_time PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(real_time ${OpenCV_LIBRARIES})
target_link_libraries(real_time ${OpenCV_LIBRARIES})
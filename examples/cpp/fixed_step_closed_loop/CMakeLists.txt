cmake_minimum_required(VERSION 3.10)

project(fixed_step)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(fixed_step fixed_step.cpp)

target_link_libraries(fixed_step monodrive)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build" "C:/opencv/build")
include_directories(${OpenCV_INCLUDE_DIRS})

target_include_directories(fixed_step PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(fixed_step ${OpenCV_LIBRARIES})
target_link_libraries(fixed_step ${OpenCV_LIBRARIES})
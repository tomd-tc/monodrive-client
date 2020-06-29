cmake_minimum_required(VERSION 3.10)

project(multi_process_example)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_CXX_STANDARD_REQUIRED ON)
set(CMAKE_CXX_EXTENSIONS OFF)

add_executable(simulator_controller simulator_controller.cpp)
add_executable(vehicle_controller vehicle_controller.cpp)

target_link_libraries(simulator_controller monodrive)
target_link_libraries(vehicle_controller monodrive)

include_directories(${EIGEN3_INCLUDE_DIR})

find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build" "C:/opencv/build")
include_directories(${OpenCV_INCLUDE_DIRS})

target_include_directories(vehicle_controller PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(vehicle_controller ${OpenCV_LIBRARIES})
target_link_libraries(vehicle_controller ${OpenCV_LIBRARIES})

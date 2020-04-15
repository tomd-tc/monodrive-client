cmake_minimum_required(VERSION 3.10)

project(lane_follower)
 
add_executable(${PROJECT_NAME} lane_follower.cpp)

target_link_libraries(${PROJECT_NAME} monoDriveClient)

find_package(Eigen3 REQUIRED)
include_directories(${EIGEN3_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} Eigen3::Eigen)
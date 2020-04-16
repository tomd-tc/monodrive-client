cmake_minimum_required(VERSION 3.10)

find_package(Eigen3)
if(${EIGEN3_FOUND})
    add_subdirectory(lane_follower)
else()
    message("lane_follower example not built, missing eigen3.")
endif()

add_subdirectory(replay)
add_subdirectory(parser_dev)

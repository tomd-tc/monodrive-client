cmake_minimum_required(VERSION 3.10)

project(replay_example)
 
add_executable(${PROJECT_NAME} replay_example.cpp)

target_link_libraries(${PROJECT_NAME} monoDriveClient)

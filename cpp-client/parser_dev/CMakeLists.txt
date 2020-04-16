cmake_minimum_required(VERSION 3.10)

project(parser_dev)
 
add_executable(${PROJECT_NAME} parser_dev.cpp)

target_link_libraries(${PROJECT_NAME} monoDriveClient)

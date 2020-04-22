cmake_minimum_required(VERSION 3.10)

project(parser_dev)
 
add_executable(${PROJECT_NAME} parser_dev.cpp)

target_link_libraries(${PROJECT_NAME} monoDriveClient)

find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build")
include_directories(${OpenCV_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${OpenCV_INCLUDE_DIRS})
link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${OpenCV_LIBRARIES})

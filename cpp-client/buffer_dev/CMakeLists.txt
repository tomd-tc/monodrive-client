cmake_minimum_required(VERSION 3.10)

project(buffer_dev)
 
# add_executable(camera_sensor camera_sensor.cpp)
# add_executable(state_sensor state_sensor.cpp)
# add_executable(imu_sensor imu_sensor.cpp)
# add_executable(gps_sensor gps_sensor.cpp)
add_executable(radar_sensor radar_sensor.cpp)


# target_link_libraries(camera_sensor monoDriveClient)
# target_link_libraries(state_sensor monoDriveClient)
# target_link_libraries(imu_sensor monoDriveClient)
# target_link_libraries(gps_sensor monoDriveClient)
target_link_libraries(radar_sensor monoDriveClient)

# find_package(OpenCV REQUIRED PATHS "C:/local/opencv/build")
# include_directories(${OpenCV_INCLUDE_DIRS})
# target_include_directories(camera_sensor PUBLIC ${OpenCV_INCLUDE_DIRS})
# link_libraries(camera_sensor ${OpenCV_LIBRARIES})
# target_link_libraries(camera_sensor ${OpenCV_LIBRARIES})

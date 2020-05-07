project(occupancy_grid)

add_executable(${PROJECT_NAME}  main.cc)
target_link_libraries(${PROJECT_NAME} monoDriveClient)
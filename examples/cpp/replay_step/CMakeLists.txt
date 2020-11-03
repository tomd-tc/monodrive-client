cmake_minimum_required(VERSION 3.10)

project(replay_step)
 
add_executable(${PROJECT_NAME} replay_step.cpp)

target_link_libraries(${PROJECT_NAME} monodrive)

# copy dll to runtime directory
if(MSVC)
  add_custom_command(TARGET replay_step POST_BUILD
    COMMAND ${CMAKE_COMMAND} -E copy_if_different
      $<TARGET_FILE:monodrive>
      $<TARGET_FILE_DIR:replay_step>
  )
endif(MSVC)
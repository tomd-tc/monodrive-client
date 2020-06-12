cmake_minimum_required(VERSION 3.10)

set(CMAKE_CXX_STANDARD 11)

project(monodrive)

# collect headers and source
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.h" "src/*.hpp")

# define shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# bring src directory into the project
include_directories(src)
target_include_directories(${PROJECT_NAME} PUBLIC src)

# include boost
set(Boost_USE_MULTITHREADED ON)
set(Boost_USE_STATIC_LIBS OFF)
find_package(Boost 1.65 REQUIRED COMPONENTS system)
if(MSVC)
	target_compile_definitions(${PROJECT_NAME} PUBLIC -DBOOST_ALL_NO_LIB)
	target_compile_definitions(${PROJECT_NAME} PUBLIC -DBOOST_PROGRAM_OPTIONS_DYN_LINK)
endif(MSVC)

include_directories(${Boost_INCLUDE_DIRS})
target_include_directories(${PROJECT_NAME} PUBLIC ${Boost_INCLUDE_DIRS})
link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})
target_link_libraries(${PROJECT_NAME} ${Boost_LIBRARIES})

find_library(LIBRT rt)
find_package(Threads)

if(LIBRT)
	target_link_libraries(${PROJECT_NAME} ${LIBRT})
endif(LIBRT)
target_link_libraries (${PROJECT_NAME} ${CMAKE_THREAD_LIBS_INIT})

# installation
if(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)
  set(CMAKE_INSTALL_PREFIX /usr/local/monodrive/client CACHE PATH "monoDrive client install prefix" FORCE)
endif(CMAKE_INSTALL_PREFIX_INITIALIZED_TO_DEFAULT)

install(TARGETS ${PROJECT_NAME} DESTINATION lib)
install(FILES ${HEADERS} DESTINATION include)

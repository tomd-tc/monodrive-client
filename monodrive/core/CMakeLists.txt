cmake_minimum_required(VERSION 3.10)

project(monodrive)

# collect headers and source
file(GLOB SOURCES "src/*.cpp")
file(GLOB HEADERS "src/*.h" "src/*.hpp")

# define shared library
add_library(${PROJECT_NAME} SHARED ${SOURCES})

# set c++ 11 standard
set_target_properties(
	${PROJECT_NAME}
    PROPERTIES
      CXX_STANDARD 11
      CXX_STANDARD_REQUIRED YES
      CXX_EXTENSIONS NO
)

# need to export all symbols from .dll on windows
if(MSVC)
  set_target_properties(${PROJECT_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS ON)
endif(MSVC)

# bring src directory into the project
include_directories(src)
target_include_directories(${PROJECT_NAME} PUBLIC src)

#include boost
IF(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.12.0")
	cmake_policy(SET CMP0074 NEW)
ENDIF(${CMAKE_VERSION} VERSION_GREATER_EQUAL "3.12.0")
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

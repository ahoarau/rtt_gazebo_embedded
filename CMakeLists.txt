cmake_minimum_required(VERSION 2.8)
project(rtt_gazebo_embedded)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

set(CMAKE_CXX_FLAGS "${CMAKE_CXX_FLAGS} -std=c++11")

# okay, this is a truely scary hack. Apparently there is a bug in rtt. 
if (CMAKE_SYSTEM_NAME MATCHES "Darwin")
    add_definitions(-DBOOST_DATE_TIME_POSIX_TIME_STD_CONFIG)
endif()

find_package(gazebo REQUIRED)

find_package(OROCOS-RTT REQUIRED COMPONENTS)
if (NOT OROCOS-RTT_FOUND)
  message (FATAL_ERROR "\nCould not find Orocos-RTT. Please use the shell command\n 'source orocos_toolchain/env.sh' and then run cmake again.")
endif()

include(${OROCOS-RTT_USE_FILE_PATH}/UseOROCOS-RTT.cmake )

if(${GAZEBO_VERSION} AND ${GAZEBO_VERSION} GREATER 6)
    add_definitions(-DGAZEBO_GREATER_6)
endif()

find_package(PkgConfig REQUIRED)
pkg_search_module(TINYXML REQUIRED tinyxml)
if(NOT TINYXML_INCLUDE_DIR)
	find_path("/usr/include" tinyxml.h)
	find_library(TINYXML_LIBRARY NAMES tinyxml)
	set(TINYXML_LIBRARIES ${TINYXML_LIBRARY})
    set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})
    set(TINYXML_LIBRARY_DIR "/usr/lib/x86_64-linux-gnu")
endif()
message(STATUS "TINYXML_VERSION: ${TINYXML_VERSION}")
include_directories(BEFORE SYSTEM ${TINYXML_INCLUDE_DIRS})
link_directories(${TINYXML_LIBRARY_DIR})
message(STATUS "TINYXML_LIBRARIES: ${TINYXML_LIBRARIES}")
message(STATUS "TINYXML_LIBRARY_DIR: ${TINYXML_LIBRARY_DIR}")
message(STATUS "TINYXML_INCLUDE_DIR: ${TINYXML_INCLUDE_DIRS}")
message(STATUS "TINYXML_LIBRARY: ${TINYXML_LIBRARY}")

include_directories(
    include
    ${GAZEBO_INCLUDE_DIRS}
    ${USE_OROCOS_INCLUDE_DIRS}
    ${TINYXML_INCLUDE_DIRS}
    )

orocos_component(${PROJECT_NAME} src/${PROJECT_NAME}.cc)

target_link_libraries(${PROJECT_NAME} ${GAZEBO_LIBRARIES} ${USE_OROCOS_LIBRARIES} ${TINYXML_LIBRARIES})

set_property(TARGET ${PROJECT_NAME} APPEND PROPERTY COMPILE_DEFINITIONS RTT_COMPONENT)

orocos_install_headers(DIRECTORY include/${PROJECT_NAME})

add_subdirectory(example)

orocos_generate_package(
    INCLUDE_DIRS include
)

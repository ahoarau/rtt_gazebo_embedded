cmake_minimum_required(VERSION 2.8)
project(rtt_gazebo_embedded)

if(NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE Release)
endif()

add_definitions("-std=c++11")

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

find_path(TINYXML_INCLUDE_DIR NAMES tinyxml.h)
find_library(TINYXML_LIBRARIES NAMES tinyxml)

include(FindPackageHandleStandardArgs)
find_package_handle_standard_args(TINYXML DEFAULT_MSG TINYXML_LIBRARIES TINYXML_INCLUDE_DIR)

mark_as_advanced(TINYXML_INCLUDE_DIR TINYXML_LIBRARIES)
set(TINYXML_INCLUDE_DIRS ${TINYXML_INCLUDE_DIR})

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

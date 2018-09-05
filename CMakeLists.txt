cmake_minimum_required(VERSION 3.0)

set(CMAKE_CXX_STANDARD 17)

add_subdirectory(${CMAKE_CURRENT_SOURCE_DIR}/sdk)

add_library(rplidar_cpp
            ${CMAKE_CURRENT_LIST_DIR}/src/rplidar_cpp.cpp
)
target_link_libraries(rplidar_cpp rplidar_sdk)
target_include_directories(rplidar_cpp PUBLIC
                           ${CMAKE_CURRENT_SOURCE_DIR}/include
                           ${CMAKE_CURRENT_SOURCE_DIR}/src
)

add_subdirectory(demo)

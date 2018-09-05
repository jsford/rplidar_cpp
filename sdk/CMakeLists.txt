cmake_minimum_required(VERSION 3.0)

set(RPLIDAR_SDK_PATH ${CMAKE_CURRENT_LIST_DIR})

find_package(Threads)

FILE(GLOB RPLIDAR_SDK_SRC
     "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
     "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
     "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

add_library(rplidar_sdk ${RPLIDAR_SDK_SRC})
target_link_libraries(rplidar_sdk Threads::Threads)
target_include_directories(rplidar_sdk PUBLIC
                           ${RPLIDAR_SDK_PATH}/include
                           ${RPLIDAR_SDK_PATH}/src
)

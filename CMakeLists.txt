cmake_minimum_required(VERSION 3.0)
project(RPLidar)

set(RPLidar_VERSION 0.0.0)

# Build the RPLidar library.
find_package(Threads REQUIRED)

set(RPLIDAR_SDK_PATH ${CMAKE_CURRENT_SOURCE_DIR}/sdk/)
FILE(GLOB RPLIDAR_SDK_SRC
      "${RPLIDAR_SDK_PATH}/src/arch/linux/*.cpp"
      "${RPLIDAR_SDK_PATH}/src/hal/*.cpp"
      "${RPLIDAR_SDK_PATH}/src/*.cpp"
)

add_library(${CMAKE_PROJECT_NAME} 
            ${CMAKE_CURRENT_SOURCE_DIR}/src/rplidar.cpp
            ${RPLIDAR_SDK_SRC}
)

target_compile_features(${CMAKE_PROJECT_NAME} PUBLIC cxx_std_17)
target_link_libraries(${CMAKE_PROJECT_NAME} PRIVATE Threads::Threads)
target_include_directories(${CMAKE_PROJECT_NAME}
  PRIVATE
  $<BUILD_INTERFACE:${RPLIDAR_SDK_PATH}/include>
  $<BUILD_INTERFACE:${RPLIDAR_SDK_PATH}/src>
  PUBLIC
  $<BUILD_INTERFACE:${CMAKE_CURRENT_SOURCE_DIR}/include>
  $<INSTALL_INTERFACE:include>
)

# Build the demo executable.
add_subdirectory(demo)

# Set up install target for the RPLidar library.
install(TARGETS ${CMAKE_PROJECT_NAME}
  EXPORT RPLidarTargets
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib
  RUNTIME DESTINATION bin
  INCLUDES DESTINATION include
)

include(CMakePackageConfigHelpers)
write_basic_package_version_file(
  RPLidarConfigVersion.cmake
  VERSION ${RPLidar_VERSION}
  COMPATIBILITY SameMajorVersion
)

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/include/RPLidar/rplidar.h"
  DESTINATION include/RPLidar
)

install(EXPORT RPLidarTargets
  FILE RPLidarTargets.cmake
  NAMESPACE RPLidar::
  DESTINATION lib/cmake/RPLidar
)

configure_file(RPLidarConfig.cmake.in RPLidarConfig.cmake @ONLY)
install(FILES
  "${CMAKE_CURRENT_BINARY_DIR}/RPLidarConfig.cmake"
  "${CMAKE_CURRENT_BINARY_DIR}/RPLidarConfigVersion.cmake"
  DESTINATION lib/cmake/RPLidar
)

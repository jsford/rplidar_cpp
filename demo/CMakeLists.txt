cmake_minimum_required(VERSION 3.0)

add_executable(rpdemo ${CMAKE_CURRENT_SOURCE_DIR}/demo.cpp)
target_link_libraries(rpdemo PUBLIC ${PROJECT_NAME})

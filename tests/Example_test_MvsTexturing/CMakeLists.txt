cmake_minimum_required(VERSION 3.12)
project(testMvsTexturing)

set(CMAKE_CXX_STANDARD 14)

include_directories(${SOURCE_DIR}/MvsTexturing)

add_executable(${PROJECT_NAME} main.cpp)
target_include_directories(${PROJECT_NAME} PRIVATE
        ${MVE_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
        MvsTexturing-Library mve mve_util)

cmake_minimum_required(VERSION 3.12)
project(testMvsTexturing)

set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE SOURCES ./*.h ./*.hpp ./*.cpp ./*.c)

include_directories(${SOURCE_DIR}/MvsTexturing)
include_directories(${EIGEN3_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${SOURCES})
target_include_directories(${PROJECT_NAME} PRIVATE
        ${MVE_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
        MvsTexturing-Library mve mve_util)

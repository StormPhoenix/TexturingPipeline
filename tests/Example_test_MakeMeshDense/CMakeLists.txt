cmake_minimum_required(VERSION 3.12)
project(testMakeMeshDense)

set(CMAKE_CXX_STANDARD 14)

file(GLOB_RECURSE SOURCES ./*.h ./*.hpp ./*.cpp ./*.c)

#include_directories(${SOURCE_DIR}/MeshPolyRefinement)
include_directories(${SOURCE_DIR}/MvsTexturing)
include_directories(${EIGEN3_INCLUDE_DIR})
#include_directories(${LIBIGL_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
#include_directories(${TINYPLY_INCLUDE_DIR})

add_executable(${PROJECT_NAME} ${SOURCES})
add_dependencies(${PROJECT_NAME} CGAL)
target_include_directories(${PROJECT_NAME} PRIVATE ${MVE_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME} PRIVATE
        MvsTexturing-Library
#        MeshPolyRefinement-Library
        Boost::program_options Boost::thread
        mve mve_util)

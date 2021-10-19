cmake_minimum_required(VERSION 3.12)
project(testMeshPolyRefinement)

set(CMAKE_CXX_STANDARD 14)

include_directories(${SOURCE_DIR}/MeshPolyRefinement)
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${LIBIGL_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${TINYPLY_INCLUDE_DIR})

add_executable(${PROJECT_NAME} main.cpp)
add_dependencies(${PROJECT_NAME} CGAL)
target_link_libraries(${PROJECT_NAME} PRIVATE
        MeshPolyRefinement-Library Boost::program_options Boost::thread)

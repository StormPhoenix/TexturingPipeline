cmake_minimum_required(VERSION 3.12)
project(TestMeshPolyRefinement)

set(CMAKE_CXX_STANDARD 14)

include_directories(${SOURCE_DIR}/MeshPolyRefinement)

add_executable(${PROJECT_NAME} main.cpp)
add_dependencies(${PROJECT_NAME} CGAL)
target_include_directories(${PROJECT_NAME} PRIVATE
        ${EIGEN_INCLUDE_DIR}
        ${LIBIGL_INCLUDE_DIR}
        ${Boost_INCLUDE_DIR}
        ${TINYPLY_INCLUDE_DIR})
target_link_libraries(${PROJECT_NAME}
        MeshPolyRefinement)

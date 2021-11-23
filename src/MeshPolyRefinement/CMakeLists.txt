cmake_minimum_required(VERSION 3.12)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif (NOT CMAKE_BUILD_TYPE)

project(MeshPolyRefinement-Library)

include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${SOURCE_DIR}/MeshPolyRefinement)
include_directories(${Boost_INCLUDE_DIR})
include_directories(${CDT_INCLUDE_DIR})
include_directories(${LIBIGL_DIR}/include)
include_directories(${LIBIGL_DIR}/external/glfw/include)
include_directories(${LIBIGL_DIR}/external/glad/include)
include_directories(${CGAL_INCLUDE_DIRS})
include_directories(${TINYPLY_INCLUDE_DIR})

link_directories(${GMP_LIBRARIES})

file(GLOB_RECURSE SOURCES ./*.h ./*.hpp ./*.cpp ./*.c)

add_library(${PROJECT_NAME} STATIC ${SOURCES})
target_link_libraries(${PROJECT_NAME} ceres
        ${GMP_LIBRARIES} ${MPFR_LIBRARIES})
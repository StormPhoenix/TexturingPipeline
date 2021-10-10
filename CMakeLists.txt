cmake_minimum_required(VERSION 3.12)
project(TexturingPipeline)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

set(EIGEN_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen3)
set(LIBIGL_DIR ${PROJECT_SOURCE_DIR}/external/libigl)
set(LIBIGL_INCLUDE_DIR ${LIBIGL_DIR}/include)
set(TINYPLY_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/tinyply/source)

find_package(Boost 1.65 COMPONENTS program_options thread REQUIRED)
foreach (PackageName GMP MPFR Ceres)
    find_package(${PackageName} REQUIRED)
    if (${PackageName}_FOUND)
        message(STATUS "NOTICE: ${PackageName} library is found.")
    else ()
        message(STATUS "NOTICE: ${PackageName} library is not found.")
    endif ()
endforeach (PackageName)

find_package(CGAL 5.2.1)
include(FetchContent)
include(ExternalProject)

if (NOT CGAL_FOUND)
    message(STATUS "Cannot find CGAL on system, build it from git repo")
    set(CGAL_Src "${PROJECT_SOURCE_DIR}/external_project/CGAL/src")
    set(CGAL_Build "${CGAL_Src}/build")
    set(CGAL_Install "${PROJECT_SOURCE_DIR}/external_project/CGAL/install")
    set(CGAL_INCLUDE_DIRS "${CGAL_Install}/include")
    ExternalProject_Add(CGAL
            GIT_REPOSITORY  https://github.com/CGAL/cgal.git
            SOURCE_DIR      ${CGAL_Src}
            UPDATE_COMMAND ""
            CONFIGURE_COMMAND mkdir -p ${CGAL_Build} && cd ${CGAL_Build} && cmake .. -DCMAKE_INSTALL_PREFIX=${CGAL_Install} -DCMAKE_BUILD_TYPE=Release
            BUILD_COMMAND cd ${CGAL_Build} && make -j16
            INSTALL_COMMAND cd ${CGAL_Build} && make install
            )
endif (NOT CGAL_FOUND)

message(STATUS "DEBUG gmp include: ${GMP_INCLUDE_DIR}, gmp lib: ${GMP_LIBRARIES}")
message(STATUS "DEBUG mpfr include: ${MPFR_INCLUDE_DIR}, mpfr lib: ${MPFR_LIBRARIES}")
message(STATUS "DEBUG Boost include: ${Boost_INCLUDE_DIR}")
message(STATUS "DEBUG CGAL include: ${CGAL_INCLUDE_DIRS}")

set(SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")

add_subdirectory(${SOURCE_DIR})
add_subdirectory(${PROJECT_SOURCE_DIR}/tests/Example_test_MeshPolyRefinement)


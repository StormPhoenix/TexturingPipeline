cmake_minimum_required(VERSION 3.12)
project(TexturingPipeline)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

find_package(PNG REQUIRED)
find_package(JPEG REQUIRED)
find_package(TIFF REQUIRED)

message(STATUS "Debug jpeg include: ${JPEG_INCLUDE_DIR}, jpeg libraries: ${JPEG_LIBRARIES}")
message(STATUS "Debug png include: ${PNG_INCLUDE_DIR}, jpeg libraries: ${PNG_LIBRARIES}")
message(STATUS "Debug tiff include: ${TIFF_INCLUDE_DIR}, jpeg libraries: ${TIFF_LIBRARIES}")

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
#            GIT_REPOSITORY  https://github.com/CGAL/cgal.git
            SOURCE_DIR      ${CGAL_Src}
            UPDATE_COMMAND ""
            CONFIGURE_COMMAND mkdir -p ${CGAL_Build} && cd ${CGAL_Build} && cmake .. -DCMAKE_INSTALL_PREFIX=${CGAL_Install} -DCMAKE_BUILD_TYPE=Release
            BUILD_COMMAND cd ${CGAL_Build} && make -j16
            INSTALL_COMMAND cd ${CGAL_Build} && make install
            )
endif (NOT CGAL_FOUND)

set(EIGEN3_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/eigen3)
set(LIBIGL_DIR ${PROJECT_SOURCE_DIR}/external/libigl)
set(LIBIGL_INCLUDE_DIR ${LIBIGL_DIR}/include)
set(TINYPLY_INCLUDE_DIR ${PROJECT_SOURCE_DIR}/external/tinyply/source)
set(MVE_Src ${PROJECT_SOURCE_DIR}/external/mve)
set(MVE_INCLUDE_DIR ${MVE_Src}/libs)

message(STATUS "DEBUG gmp include: ${GMP_INCLUDE_DIR}, gmp lib: ${GMP_LIBRARIES}")
message(STATUS "DEBUG mpfr include: ${MPFR_INCLUDE_DIR}, mpfr lib: ${MPFR_LIBRARIES}")
message(STATUS "DEBUG Boost include: ${Boost_INCLUDE_DIR}")
message(STATUS "DEBUG CGAL include: ${CGAL_INCLUDE_DIRS}")
message(STATUS "DEBUG eigen3 include: ${EIGEN3_INCLUDE_DIR}")

set(SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")

add_subdirectory(${MVE_Src})
add_subdirectory(${SOURCE_DIR})
add_subdirectory(${PROJECT_SOURCE_DIR}/tests/Example_test_MeshPolyRefinement)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests/Example_test_MvsTexturing)
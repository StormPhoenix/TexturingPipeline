cmake_minimum_required(VERSION 3.12)
project(TexturingPipeline)

set(CMAKE_CXX_STANDARD 14)
set(CMAKE_MODULE_PATH ${CMAKE_MODULE_PATH} "${PROJECT_SOURCE_DIR}/cmake")

# Find OpenMP
find_package(OpenMP REQUIRED)
if (OpenMP_CXX_FOUND)
    message(STATUS "Debug OpenMP include: ${OpenMP_CXX_INCLUDE_DIRS}")
    message(STATUS "Debug OpenMP library: ${OpenMP_CXX_LIBRARIES}")
    message(STATUS "Debug OpenMP C++_flags: ${OpenMP_CXX_FLAGS}")
#    link_libraries(OpenMP::OpenMP_CXX)
    OPTION(USE_OpenMP "Use OpenMP to enamble <omp.h>" ON)
endif (OpenMP_CXX_FOUND)

find_package(Boost 1.65 COMPONENTS program_options thread REQUIRED)
foreach (PackageName GMP MPFR Ceres PNG JPEG TIFF TBB)
    find_package(${PackageName} REQUIRED)
    if (${PackageName}_FOUND)
        message(STATUS "NOTICE: ${PackageName} library is found.")
    else ()
        message(STATUS "NOTICE: ${PackageName} library is not found.")
    endif ()
endforeach (PackageName)

#message(STATUS "Debug TBB root: ${TBB_ROOT_DIR}")
#message(STATUS "Debug TBB include: ${TBB_INCLUDE_DIRS}, TBB library: ${TBB_LIBRARIES}")
#message(STATUS "Debug TBB include: ${TBB_INCLUDE_DIR}, TBB library: ${TBB_LIBRARY}")
#message(STATUS "Debug TBB library: ${TBB_LIBRARIES_DEBUG}")
#message(STATUS "Debug TBB library: ${TBB_LIBRARIES_RELEASE}")

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

set(TINYPLY_Src ${PROJECT_SOURCE_DIR}/external/tinyply)
set(TINYPLY_INCLUDE_DIR ${TINYPLY_Src}/source)

set(MVE_Src ${PROJECT_SOURCE_DIR}/external/mve)
set(MVE_INCLUDE_DIR ${MVE_Src}/libs)

set(RAYINT_Src ${PROJECT_SOURCE_DIR}/external/rayint)
set(RAYINT_INCLUDE_DIR ${RAYINT_Src}/libs)

set(SOURCE_DIR "${PROJECT_SOURCE_DIR}/src")

add_subdirectory(${MVE_Src})
add_subdirectory(${SOURCE_DIR})
add_subdirectory(${PROJECT_SOURCE_DIR}/tests/Example_test_MeshPolyRefinement)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests/Example_test_MvsTexturing)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests/Example_test_TextureRemeshing)
add_subdirectory(${PROJECT_SOURCE_DIR}/tests/Example_test_MakeMeshDense)
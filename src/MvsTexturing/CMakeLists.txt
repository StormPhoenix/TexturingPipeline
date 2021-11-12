cmake_minimum_required(VERSION 3.12)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif (NOT CMAKE_BUILD_TYPE)

file(GLOB_RECURSE SOURCES ./*.h ./*.hpp ./*.cpp ./*.c)

include_directories(${MVE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${TINYPLY_INCLUDE_DIR})
include_directories(${Boost_INCLUDE_DIR})
include_directories(${LIBIGL_DIR}/include)
include_directories(${SOURCE_DIR}/MeshPolyRefinement)
include_directories(${SOURCE_DIR}/MvsTexturing)

set(TBB_INCLUDE_DIRS "/usr/local/opt/tbb@2020/include")
set(TBB_LIBRARY "/usr/local/opt/tbb@2020/lib")
set(TBB_LIBRARIES "/usr/local/opt/tbb@2020/lib/libtbb.dylib")

set(MvsTexturingLibrary MvsTexturing-Library)
add_library(${MvsTexturingLibrary} STATIC ${SOURCES})
set_property(TARGET ${MvsTexturingLibrary} PROPERTY INTERPROCEDURAL_OPTIMIZATION True)
target_include_directories(${MvsTexturingLibrary} PRIVATE
        ${RAYINT_INCLUDE_DIR}
        ${LBP_INCLUDE_DIR}
        ${Mapmap_INCLUDE_DIR}
        ${Mapmap_INCLUDE_DIR}/mapmap
        ${Mapmap_INCLUDE_DIR}/ext/dset
        ${TBB_INCLUDE_DIRS}
        ${OpenMP_CXX_INCLUDE_DIRS})
target_compile_options(${MvsTexturingLibrary} PRIVATE
        ${OpenMP_CXX_FLAGS})
target_link_directories(${MvsTexturingLibrary} PRIVATE
        ${TBB_LIBRARY})
target_link_libraries(${MvsTexturingLibrary}
        mve mve_util ${JPEG_LIBRARIES} ${PNG_LIBRARIES} ${TIFF_LIBRARIES}
        ${TBB_LIBRARIES}
        OpenMP::OpenMP_CXX)

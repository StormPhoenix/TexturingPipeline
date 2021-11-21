cmake_minimum_required(VERSION 3.12)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif (NOT CMAKE_BUILD_TYPE)

file(GLOB_RECURSE SOURCES ./*.h ./*.hpp ./*.cpp ./*.c)

include_directories(${SOURCE_DIR}/MvsTexturing)

set(MeshRepair MeshRepair-Library)
add_library(${MeshRepair} STATIC ${SOURCES})
set_property(TARGET ${MeshRepair} PROPERTY INTERPROCEDURAL_OPTIMIZATION True)
target_include_directories(${MeshRepair} PRIVATE
        ${EIGEN3_INCLUDE_DIR}
        ${MVE_INCLUDE_DIR})

target_link_directories(${MeshRepair} PRIVATE
        MvsTexturing-Library)

#target_compile_options(${MeshSubdivisionLibrary} PRIVATE ${OpenMP_CXX_FLAGS})
#target_link_directories(${MeshSubdivisionLibrary} PRIVATE ${TBB_LIBRARY})
#target_link_libraries(${MeshSubdivisionLibrary} OpenMP::OpenMP_CXX)

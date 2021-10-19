cmake_minimum_required(VERSION 3.12)

if (NOT CMAKE_BUILD_TYPE)
    set(CMAKE_BUILD_TYPE "Release")
endif (NOT CMAKE_BUILD_TYPE)

file(GLOB_RECURSE SOURCES ./*.h ./*.hpp ./*.cpp ./*.c)

include_directories(${MVE_INCLUDE_DIR})
include_directories(${EIGEN3_INCLUDE_DIR})
include_directories(${SOURCE_DIR}/MvsTexturing)

set(MvsTexturingLibrary MvsTexturing-Library)
add_library(${MvsTexturingLibrary} STATIC ${SOURCES})
set_property(TARGET ${MvsTexturingLibrary} PROPERTY INTERPROCEDURAL_OPTIMIZATION True)
target_link_libraries(${MvsTexturingLibrary} mve mve_util ${JPEG_LIBRARIES} ${PNG_LIBRARIES} ${TIFF_LIBRARIES})
include(FindPackageHandleStandardArgs)
if(GMP_INCLUDE_DIR)
    set(GMP_in_cache TRUE)
else()
    set(GMP_in_cache FALSE)
endif()

if(NOT GMP_LIBRARIES)
    set(GMP_in_cache FALSE)
endif()

if( NOT GMP_in_cache )
    find_path(GMP_INCLUDE_DIR
            NAMES gmp.h
            HINTS ENV GMP_INC_DIR
            ENV GMP_DIR
            /usr
            /usr/include/x86_64-linux-gnu
            PATH_SUFFIXES include
            DOC "The directory containing the GMP header files"
            )

    find_library(GMP_LIBRARY_RELEASE NAMES gmp libgmp-10 mpir
            HINTS ENV GMP_LIB_DIR
            ENV GMP_DIR
            /usr
            /usr/lib/x86_64-linux-gnu/
            PATH_SUFFIXES lib
            DOC "Path to the Release GMP library"
            )

    find_library(GMP_LIBRARY_DEBUG NAMES gmpd gmp libgmp-10 mpir
            HINTS ENV GMP_LIB_DIR
            ENV GMP_DIR
            /usr
            /usr/lib/x86_64-linux-gnu/
            PATH_SUFFIXES lib
            DOC "Path to the Debug GMP library"
            )

    get_property(IS_MULTI_CONFIG GLOBAL PROPERTY GENERATOR_IS_MULTI_CONFIG)
    if(IS_MULTI_CONFIG)
        set(GMP_LIBRARIES debug ${GMP_LIBRARY_DEBUG} optimized ${GMP_LIBRARY_RELEASE})
    else()
        if("${CMAKE_BUILD_TYPE}" STREQUAL "Debug")
            set(GMP_LIBRARIES ${GMP_LIBRARY_DEBUG})
        else()
            set(GMP_LIBRARIES ${GMP_LIBRARY_RELEASE})
        endif()
    endif()
endif()
find_package_handle_standard_args(GMP "DEFAULT_MSG" GMP_LIBRARIES GMP_INCLUDE_DIR)


include(FindPackageHandleStandardArgs)
if(MPFR_INCLUDE_DIR)
    set(MPFR_in_cache TRUE)
else()
    set(MPFR_in_cache FALSE)
endif()
if(NOT MPFR_LIBRARIES)
    set(MPFR_in_cache FALSE)
endif()

if (NOT MPFR_in_cache)
    find_path(MPFR_INCLUDE_DIR
            NAMES mpfr.h
            HINTS ENV MPFR_INC_DIR
            ENV MPFR_DIR
            /usr
            /usr/include/x86_64-linux-gnu
            PATH_SUFFIXES include
            DOC "The directory containing the MPFR header files"
            )

    find_library(MPFR_LIBRARIES NAMES mpfr libmpfr-4 libmpfr-1
            HINTS ENV MPFR_LIB_DIR
            ENV MPFR_DIR
            /usr
            /usr/lib/x86_64-linux-gnu/
            PATH_SUFFIXES lib
            DOC "Path to the MPFR library"
            )

    if ( MPFR_LIBRARIES )
        get_filename_component(MPFR_LIBRARIES_DIR ${MPFR_LIBRARIES} PATH CACHE )
    endif()
endif()
find_package_handle_standard_args(MPFR "DEFAULT_MSG" MPFR_LIBRARIES MPFR_INCLUDE_DIR)

# Order object files by frequency of execution.  Small files at end.
set(
    qhull_src
        libqhull_r/rboxlib_r.c
        libqhull_r/user_r.c
        libqhull_r/global_r.c
        libqhull_r/stat_r.c
        libqhull_r/io_r.c
        libqhull_r/geom2_r.c
        libqhull_r/poly2_r.c
        libqhull_r/merge_r.c
        libqhull_r/libqhull_r.c
        libqhull_r/geom_r.c
        libqhull_r/poly_r.c
        libqhull_r/qset_r.c
        libqhull_r/mem_r.c
        libqhull_r/usermem_r.c
        libqhull_r/userprintf_r.c
        libqhull_r/userprintf_rbox_r.c
        libqhull_r/random_r.c
)

file(GLOB_RECURSE qhull_hdr *.h)

rw_add_library(sdurw_qhull STATIC ${qhull_src})
set_property(TARGET sdurw_qhull PROPERTY OUTPUT_NAME "sdurw_qhull")
target_include_directories(sdurw_qhull INTERFACE $<BUILD_INTERFACE:${QHULL_INCLUDE_DIRS}> $<INSTALL_INTERFACE:"${INCLUDE_INSTALL_DIR}/ext/qhull/src">)

IF(CMAKE_COMPILER_IS_GNUCXX)
    TARGET_COMPILE_OPTIONS(sdurw_qhull PRIVATE -Wno-unused-variable)
ENDIF()
rw_add_includes(sdurw_qhull /ext/qhull/src ${qhull_hdr})


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

add_library(sdurw_qhull STATIC ${qhull_src})
set_property(TARGET sdurw_qhull PROPERTY OUTPUT_NAME "sdurw_qhull")

IF(CMAKE_COMPILER_IS_GNUCXX)
    TARGET_COMPILE_OPTIONS(sdurw_qhull PRIVATE -Wno-unused-variable)
ENDIF()

INSTALL(TARGETS #qhull
    sdurw_qhull
    #qhullcmd rbox qconvex qdelaunay qvoronoi qhalf
	RUNTIME DESTINATION ${BIN_INSTALL_DIR}
	LIBRARY DESTINATION ${LIB_INSTALL_DIR}
	ARCHIVE DESTINATION ${LIB_INSTALL_DIR})
INSTALL(FILES ${qhull_hdr} DESTINATION ${INCLUDE_INSTALL_DIR}/ext/qhull/src)

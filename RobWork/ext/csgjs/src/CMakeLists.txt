
# Order object files by frequency of execution.  Small files at end.
set(
    csgjs_src
        csgjs/csgjs.cpp
)

file(GLOB csgjs_hdr csgjs/*.cpp)

add_library(sdurw_csgjs STATIC ${csgjs_src})
set_property(TARGET sdurw_csgjs PROPERTY OUTPUT_NAME "sdurw_csgjs")
target_include_directories(sdurw_csgjs INTERFACE $<BUILD_INTERFACE:${CSGJS_INCLUDE_DIRS}> $<INSTALL_INTERFACE:"${INCLUDE_INSTALL_DIR}/ext/csgjs/src">)

# this does not compile on some versions of gcc
 
install(TARGETS
    sdurw_csgjs
    EXPORT ${PROJECT_PREFIX}Targets 
	RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT sdurw_csgjs
	LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT sdurw_csgjs
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT sdurw_csgjs
    )
install(FILES ${csgjs_hdr} DESTINATION ${INCLUDE_INSTALL_DIR}/ext/csgjs/src)

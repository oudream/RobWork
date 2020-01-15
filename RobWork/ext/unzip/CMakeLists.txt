PROJECT(minizip C)

set(VERSION "1.2.8")

#============================================================================
# minizip
#============================================================================

include_directories(${ZLIB_INCLUDE_DIRS})

set(SOURCE
	ioapi.c
	zip.c
	unzip.c
)

set(HEADERS
	ioapi.h
	zip.h
	unzip.h
)

add_library(sdurw_unzip STATIC ${SOURCE} ${HEADERS})

target_link_libraries(sdurw_unzip ${ZLIB_LIBRARIES} )
IF(MSVC)
	target_compile_options(sdurw_unzip PRIVATE /wd4067)
ENDIF()

INSTALL(TARGETS sdurw_unzip EXPORT ${PROJECT_PREFIX}Targets DESTINATION "${LIB_INSTALL_DIR}" )
INSTALL(FILES ${HEADERS} DESTINATION "${INCLUDE_INSTALL_DIR}/ext/unzip/")

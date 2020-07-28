
#INCLUDE(ExternalProject)

# A short alias for the current directory.
set(Root ${CMAKE_CURRENT_SOURCE_DIR})

# Subdirectories to process.
IF(RW_ENABLE_INTERNAL_PQP_TARGET)
    add_subdirectory(rwpqp)
ENDIF()

IF(RW_ENABLE_INTERNAL_YAOBI_TARGET)

    add_subdirectory(rwyaobi)
ENDIF()
IF(RW_ENABLE_INTERNAL_LUA_TARGET)
    add_subdirectory(lua)
ENDIF()

#IF(RW_ENABLE_INTERNAL_QHULL_TARGET)
#ExternalProject_Add(
#   rwqhull
#   SOURCE_DIR ${Root}/qhull
#   CMAKE_ARGS -DCMAKE_BUILD_TYPE=${CMAKE_BUILD_TYPE}
#   INSTALL_COMMAND ""
#)
add_subdirectory(qhull)
#ENDIF()

IF(RW_ENABLE_INTERNAL_ZLIB_TARGET)
	add_subdirectory(zlib)
ENDIF()

IF(RW_ENABLE_INTERNAL_MINIZIP_TARGET)
	add_subdirectory(unzip)
ENDIF()

IF(RW_ENABLE_INTERNAL_ASSIMP_TARGET)
	add_subdirectory(assimp)
ENDIF()

if(RW_USE_CSGJS)
    add_subdirectory(csgjs)
endif()

IF(RW_ENABLE_INTERNAL_FCL_TARGET)
	add_subdirectory(fcl)
ENDIF()

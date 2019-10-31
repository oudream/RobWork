SET(SUBSYS_NAME sdh )
SET(SUBSYS_DESC "Driver for robolab sdh " )
SET(SUBSYS_DEPS sdurw )

# make sure that SDH can be found
SET(build TRUE)

FIND_PACKAGE(SDH QUIET)
set(DEFAULT TRUE)
set(REASON )
IF( NOT SDH_FOUND)
    set(DEFAULT false)
    set(REASON "SDH library not found!")
endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    FIND_PACKAGE(SDH)
    INCLUDE_DIRECTORIES( ${SDH_INCLUDE_DIR} )

    FIND_PACKAGE(PEAKCAN)
    
    SET(SRC_FILES )
    SET(SRC_FILES_HPP )
    IF(TOLUA++_FOUND)
    #    ADD_TOLUA_PACKAGE("SDHDriverLua" "SDHDriverLuaStub")
    ENDIF()
        
    RW_ADD_LIBRARY(sdurwhw_sdh sdh SDHDriver.cpp SDHDriver.hpp ${SRC_FILES} ${SRC_FILES_HPP})
    TARGET_LINK_LIBRARIES(sdurwhw_sdh PUBLIC ${ROBWORK_LIBRARIES} ${SDH_LIBRARY} ${PEAKCAN_LIBRARIES})
    RW_ADD_INCLUDES(sdh "rwhw/sdh" SDHDriver.hpp SDHDriverLua.hpp )
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_sdh PARENT_SCOPE)

    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS ROBWORK_LIBRARIES SDH_LIBRARY PEAKCAN_LIBRARIES)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(sdurwhw_sdh ${DEP})
        ENDIF()
    ENDFOREACH()
    
ENDIF()

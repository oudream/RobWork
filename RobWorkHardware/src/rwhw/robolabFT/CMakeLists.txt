SET(SUBSYS_NAME robolabFT )
SET(SUBSYS_DESC "Driver for robolab force torque device" )
SET(SUBSYS_DEPS sdurwhw_serialport sdurw )

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
   # INCLUDE_DIRECTORIES( ${robolabFT_INCLUDE_DIR} )
    
    SET(SRC_FILES RobolabFTDriver.cpp)
    SET(SRC_FILES_HPP RobolabFTDriver.hpp)
    
    RW_ADD_LIBRARY(sdurwhw_robolabFT robolabFT ${SRC_FILES} ${SRC_FILES_HPP})
    TARGET_LINK_LIBRARIES(sdurwhw_robolabFT PUBLIC sdurwhw_serialport ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(robolabFT "rwhw/robolabFT" ${SRC_FILES_HPP})

    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS sdurwhw_serialport ROBWORK_LIBRARIES)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(sdurwhw_robolabFT ${DEP})
        ENDIF()
    ENDFOREACH()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_robolabFT PARENT_SCOPE)
    MESSAGE(STATUS "robolabFT include dir:" ${robolabFT_INCLUDE_DIR})
ENDIF()

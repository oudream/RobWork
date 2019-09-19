SET(SUBSYS_NAME robolabFT )
SET(SUBSYS_DESC "Driver for robolab force torque device" )
SET(SUBSYS_DEPS rwhw_serialport rw )

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
   # INCLUDE_DIRECTORIES( ${robolabFT_INCLUDE_DIR} )
    
    SET(SRC_FILES RobolabFTDriver.cpp)
    SET(SRC_FILES_HPP RobolabFTDriver.hpp)
    
    RW_ADD_LIBRARY(rwhw_robolabFT robolabFT ${SRC_FILES} ${SRC_FILES_HPP})
    TARGET_LINK_LIBRARIES(rwhw_robolabFT PUBLIC rwhw_serialport ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(robolabFT "rwhw/robolabFT" ${SRC_FILES_HPP})

    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS rwhw_serialport ROBWORK_LIBRARIES)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(rwhw_robolabFT ${DEP})
        ENDIF()
    ENDFOREACH()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} rwhw_robolabFT PARENT_SCOPE)
    MESSAGE(STATUS "robolabFT include dir:" ${robolabFT_INCLUDE_DIR})
ENDIF()

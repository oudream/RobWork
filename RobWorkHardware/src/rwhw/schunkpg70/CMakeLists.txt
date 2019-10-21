SET(SUBSYS_NAME schunkpg70 )
SET(SUBSYS_DESC "Driver for schunk gripper pg70" )
SET(SUBSYS_DEPS sdurwhw_pcube sdurwhw_serialport sdurw )

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    SET(SRC_CPP SchunkPG70.cpp )
    SET(SRC_HPP SchunkPG70.hpp )
    
    RW_ADD_LIBRARY(sdurwhw_schunkpg70 schunkpg70 ${SRC_CPP} ${SRC_HPP})
    TARGET_LINK_LIBRARIES(sdurwhw_schunkpg70 PUBLIC sdurwhw_pcube sdurwhw_serialport ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(schunkpg70 "rwhw/schunkpg70" ${SRC_HPP})

    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS SUBSYS_DEPS ROBWORK_LIBRARIES)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(sdurwhw_schunkpg70 ${DEP})
        ENDIF()
    ENDFOREACH()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_schunkpg70 PARENT_SCOPE)
ENDIF()

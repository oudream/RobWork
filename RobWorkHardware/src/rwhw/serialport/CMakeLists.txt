SET(component_name serialport)
SET(SUBSYS_NAME serialport )
SET(SUBSYS_DESC "Serial port abstractions!" )
SET(SUBSYS_DEPS rw )

SET(build )
set(DEFAULT TRUE)
set(REASON "")
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    RW_ADD_LIBRARY(rwhw_serialport serialport SerialPort.cpp)
    TARGET_LINK_LIBRARIES(rwhw_serialport PUBLIC ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(serialport "rwhw/serialport" SerialPort.hpp)
    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS ROBWORK_LIBRARIES)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(rwhw_serialport ${DEP})
        ENDIF()
    ENDFOREACH()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} rwhw_serialport PARENT_SCOPE)
ENDIF()

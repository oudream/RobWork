SET(SUBSYS_NAME motomanIA20 )
SET(SUBSYS_DESC "Library for controlling the motomanIA20 robot" )
SET(SUBSYS_DEPS )

set(DEFAULT TRUE)
set(REASON )
IF( DEFINED WIN32)
    set(DEFAULT false)
    set(REASON "Does not support windows!")
endif()
 
SET(build TRUE)
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    IF (NOT DEFINED WIN32)
        ADD_LIBRARY(
          sdurwhw_motomanIA20
          MotomanIA20.cpp
        )
        set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_motomanIA20 PARENT_SCOPE)
        MESSAGE(STATUS "RobWorkHardware: ${SUBSYS_NAME} component ENABLED")
    ELSE()
        MESSAGE(STATUS "RobWorkHardware: ${SUBSYS_NAME} component DISABLED - linux only")
    ENDIF()
ENDIF()
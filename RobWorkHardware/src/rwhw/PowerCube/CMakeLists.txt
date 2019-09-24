SET(SUBSYS_NAME pcube )
SET(SUBSYS_DESC "Driver for powercube modules" )
SET(SUBSYS_DEPS rw )

SET(build TRUE)
set(DEFAULT true)
set(REASON )
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    #MESSAGE(STATUS "RobWorkHardware: ${SUBSYS_NAME} component ENABLED")    

    SET(SRC_CPP CubePort.cpp Cube.cpp PCubeProtocol.cpp)
    SET(SRC_HPP CubePort.hpp Cube.hpp PCubeProtocol.hpp)

    RW_ADD_LIBRARY(rwhw_pcube pcube ${SRC_CPP} ${SRC_HPP})
    TARGET_LINK_LIBRARIES(rwhw_pcube PUBLIC ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(pcube "rwhw/PowerCube" ${SRC_HPP})

    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS ROBWORK_LIBRARIES)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(rwhw_pcube ${DEP})
        ENDIF()
    ENDFOREACH()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} rwhw_pcube PARENT_SCOPE)
ENDIF()

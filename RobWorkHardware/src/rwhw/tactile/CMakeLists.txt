SET(SUBSYS_NAME tactile )
SET(SUBSYS_DESC "Tactile array sensor library" )
SET(SUBSYS_DEPS sdurw serialport)

SET(build TRUE)
set(DEFAULT TRUE)
set(REASON) 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    SET(SRC_CPP )
    SET(SRC_HPP )
    
    LIST(APPEND SRC_CPP DSACON32.cpp TactileMatrix.cpp TactileMaskMatrix.cpp ConvertUtil.cpp)
    LIST(APPEND SRC_HPP DSACON32.hpp TactileMatrix.hpp TactileMaskMatrix.hpp ConvertUtil.hpp)
    
    RW_ADD_LIBRARY(sdurwhw_tactile tactile ${SRC_CPP} ${SRC_HPP})
    TARGET_LINK_LIBRARIES(sdurwhw_tactile PUBLIC ${ROBWORK_LIBRARIES})
    RW_ADD_INCLUDES(tactile "rwhw/tactile" ${SRC_HPP})

    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS ROBWORK_LIBRARIES)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(sdurwhw_tactile ${DEP})
        ENDIF()
    ENDFOREACH()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_tactile PARENT_SCOPE)
ENDIF()

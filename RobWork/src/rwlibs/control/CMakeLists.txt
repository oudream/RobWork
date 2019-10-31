SET(SUBSYS_NAME sdurw_control)
set(SUBSYS_DESC "A collection of general controllers and interfaces for RobWork")
SET(SUBSYS_DEPS sdurw)

set(build TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON)
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    SET(SRC_FILES
        Controller.cpp
        JointController.cpp
        SyncVelocityRamp.cpp
    )
    
    SET(SRC_FILES_HPP
        Controller.hpp
        JointController.hpp
        SyncVelocityRamp.hpp
    )
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} control ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(control "rwlibs/control" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()
ENDIF()
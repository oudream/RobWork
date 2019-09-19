SET(SUBSYS_NAME netft )
SET(SUBSYS_DESC "driver for netft" )
SET(SUBSYS_DEPS rw )

SET(build TRUE)
SET(default TRUE)

set(Boost_NO_BOOST_CMAKE TRUE) # From Boost 1.70, CMake files are provided by Boost - we are not yet ready to handle it

FIND_PACKAGE(Boost REQUIRED COMPONENTS system regex QUIET)
set(DEFAULT TRUE)
set(REASON )
IF( NOT Boost_FOUND)
    set(DEFAULT false)
    set(REASON "Boost system and/or regex not found!")
endif()
 
SET(build TRUE)
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )


IF( build )
    #MESSAGE(STATUS "RobWorkHardware: ${component_name} component ENABLED")    
    FIND_PACKAGE(Boost REQUIRED COMPONENTS system regex)
    
    SET(SRC_CPP NetFTLogging.cpp FTCompensation.cpp)
    SET(SRC_HPP NetFTCommon.hpp NetFTLogging.hpp FTCompensation.hpp)
    
    RW_ADD_LIBRARY(rwhw_netft netft ${SRC_CPP} ${SRC_HPP})
    TARGET_LINK_LIBRARIES(rwhw_netft PUBLIC ${ROBWORK_LIBRARIES})
    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS ROBWORK_LIBRARIES)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(rwhw_netft ${DEP})
        ENDIF()
    ENDFOREACH()
    RW_ADD_INCLUDES(netft "rwhw/netft" ${SRC_HPP})

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} rwhw_netft PARENT_SCOPE)
ENDIF()



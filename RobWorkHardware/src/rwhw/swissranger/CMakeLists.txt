SET(SUBSYS_NAME swissranger )
SET(SUBSYS_DESC "Swiss ranger driver" )
SET(SUBSYS_DEPS sdurw )

SET(build TRUE)
set(DEFAULT FALSE)
set(REASON)

IF( ${BUILD_swissranger} )
    FIND_PACKAGE(SwissRanger REQUIRED)
    IF(NOT SwissRanger_FOUND)
        set(BUILD_swissranger FALSE)
        set(REASON "Could not find swissranger library")
    ENDIF()
ENDIF()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    RW_ADD_LIBRARY(sdurwhw_swissranger swissranger 
         SRCalibrationData.cpp SwissRanger.cpp
         SRCalibrationData.hpp SwissRanger.hpp SRConstants.hpp)
    RW_ADD_INCLUDES(swissranger "rwhw/swissranger" SRCalibrationData.hpp SwissRanger.hpp SRConstants.hpp)
    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_swissranger PARENT_SCOPE)
ENDIF()

SET(SUBSYS_NAME sdurw_pathoptimization)
set(SUBSYS_DESC "Path optimization algorithms")
SET(SUBSYS_DEPS sdurw_proximitystrategies sdurw)

SET(build TRUE)
set(DEFAULT TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)
    SET(SRC_FILES
      ./clearance/ClearanceOptimizer.cpp
      ./clearance/ClearanceCalculator.cpp
      ./clearance/MinimumClearanceCalculator.cpp
      ./pathlength/PathLengthOptimizer.cpp
    )
     
    SET(SRC_FILES_HPP
      ./clearance/ClearanceOptimizer.hpp
      ./clearance/ClearanceCalculator.hpp
      ./clearance/MinimumClearanceCalculator.hpp
      ./pathlength/PathLengthOptimizer.hpp
    )
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} pathoptimization ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(pathoptimization "rwlibs/pathoptimization" ${SRC_FILES_HPP}) 
    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} PRIVATE sdurw_proximitystrategies PUBLIC sdurw)
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()
ENDIF()

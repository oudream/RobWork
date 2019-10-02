SET(SUBSYS_NAME rw_pathplanners)
set(SUBSYS_DESC "A collection of path planners!")
SET(SUBSYS_DEPS rw)

SET(build TRUE)
SET(DEFAULT TRUE)
RW_SUBSYS_OPTION(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT})
RW_ADD_DOC(${SUBSYS_NAME})

IF(build)

    SET(SRC_FILES
      ./z3/Z3QToQPlanner.cpp
      ./z3/Z3Planner.cpp
      ./prm/PartialIndexTable.cpp
      ./prm/PRMPlanner.cpp
      ./rrt/RRTQToQPlanner.cpp
      ./rrt/RRTPlanner.cpp
      ./arw/ARWExpand.cpp
      ./arw/ARWPlanner.cpp
      ./arw/ARWQToQPlanner.cpp
      ./sbl/SBLExpand.cpp
      ./sbl/SBLInternal.cpp
      ./sbl/SBLSetup.cpp
      ./sbl/SBLOptions.cpp
      ./sbl/SBLPlanner.cpp
    )
    
    SET(SRC_FILES_HPP_Z3
      ./z3/Z3QToQPlanner.hpp
      ./z3/Z3Planner.hpp
    )
    SET(SRC_FILES_HPP_PRM
      ./prm/PartialIndexTable.hpp
      ./prm/PRMPlanner.hpp
    )
    SET(SRC_FILES_HPP_RRT
      ./rrt/RRTQToQPlanner.hpp
      ./rrt/RRTPlanner.hpp
    )
    SET(SRC_FILES_HPP_ARW
      ./arw/ARWExpand.hpp
      ./arw/ARWPlanner.hpp
      ./arw/ARWQToQPlanner.hpp
    )
    SET(SRC_FILES_HPP_SBL
      ./sbl/SBLExpand.hpp
      ./sbl/SBLInternal.hpp
      ./sbl/SBLSetup.hpp
      ./sbl/SBLOptions.hpp
      ./sbl/SBLPlanner.hpp
    )
    SET(SRC_FILES_HPP 
      ${SRC_FILES_HPP_Z3}
      ${SRC_FILES_HPP_PRM}
      ${SRC_FILES_HPP_RRT}
      ${SRC_FILES_HPP_ARW}
      ${SRC_FILES_HPP_SBL}
    )
    
    RW_ADD_LIBRARY(${SUBSYS_NAME} pathplanners ${SRC_FILES} ${SRC_FILES_HPP})
    RW_ADD_INCLUDES(pathplanners "rwlibs/pathplanners/z3" ${SRC_FILES_HPP_z3})
    RW_ADD_INCLUDES(pathplanners "rwlibs/pathplanners/prm" ${SRC_FILES_HPP_PRM}) 
    RW_ADD_INCLUDES(pathplanners "rwlibs/pathplanners/rrt" ${SRC_FILES_HPP_RRT}) 
    RW_ADD_INCLUDES(pathplanners "rwlibs/pathplanners/arw" ${SRC_FILES_HPP_ARW}) 
    RW_ADD_INCLUDES(pathplanners "rwlibs/pathplanners/sbl" ${SRC_FILES_HPP_SBL})  

    TARGET_LINK_LIBRARIES(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    ADD_DEPENDENCIES(${SUBSYS_NAME} ${SUBSYS_DEPS})

	IF(CMAKE_VERSION VERSION_GREATER 3.3)
		SET_TARGET_PROPERTIES(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
	ENDIF()
endif()


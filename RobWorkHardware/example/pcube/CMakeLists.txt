# check compiler/operating system

IF(COMPONENT_pcube_ENABLE AND COMPONENT_CAN_ENABLE)
    set(LibraryList
        sdurwhw_pcube 
        sdurwhw_can 
        ntcan.lib
        sdurwhw_serialport
        ${ROBWORK_LIBRARIES}
    )
    
    LINK_DIRECTORIES(${ESDCAN_LIB_DIR})
    
    add_executable(PCubeExample PCubeExample.cpp )
    target_link_libraries(PCubeExample ${LibraryList})
ENDIF(COMPONENT_pcube_ENABLE AND COMPONENT_CAN_ENABLE)

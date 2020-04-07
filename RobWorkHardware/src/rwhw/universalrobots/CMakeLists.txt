set(SUBSYS_NAME sdurwhw_universalrobots)
set(SUBSYS_DESC "Driver for universal robot ")
set(SUBSYS_DEPS RW::sdurw)

# make sure that SDH can be found
set(build TRUE)

set(DEFAULT TRUE)
set(REASON)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(
        SRC_CPP
        UniversalRobotsData.cpp
        UniversalRobotsRTLogging.cpp
        URCommon.cpp
        URPrimaryInterface.cpp
        URCallBackInterface.cpp
        URMessage.cpp
    )
    set(
        SRC_HPP
        UniversalRobotsData.hpp
        UniversalRobotsRTLogging.hpp
        URCommon.hpp
        URPrimaryInterface.hpp
        URCallBackInterface.hpp
        URMessage.hpp
    )

    file(READ urscript.ur URSCRIPT)

    # std::string(\"\")+\n\"") set(header "#include<sstream>\n std::stringstream UR_SCRIPT; UR_SCRIPT<< \"")
    set(
        header
        "const std::string NEW_LINE= \"\\n\"; const std::string QUOTATION = \"\\\"\"; std::string UR_SCRIPT = \""
    )
    string(REGEX REPLACE "\"" "\"+QUOTATION+\"" FILEVAR21 ${URSCRIPT})
    string(REGEX REPLACE "\\\n" "\" + NEW_LINE  + \n\"" FILEVAR21 ${FILEVAR21})
    # set(end "\");")
    set(end "\";")

    file(WRITE urscript.hpp "${header}${FILEVAR21}${end}")
    # MESSAGE("${header}${FILEVAR21}${end}")

    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC RW::sdurw)
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWHW_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
    rw_add_includes(${SUBSYS_NAME} "rwhw/universalrobots" ${SRC_HPP} urscript.hpp)

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} ${SUBSYS_NAME} PARENT_SCOPE)
else()
    message(STATUS "RobWorkHardware: ${component_name} component DISABLED")
endif()

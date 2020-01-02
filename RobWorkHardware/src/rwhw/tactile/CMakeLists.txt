set(SUBSYS_NAME sdurwhw_tactile)
set(SUBSYS_DESC "Tactile array sensor library")
set(SUBSYS_DEPS sdurw)

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
            DSACON32.cpp
            TactileMatrix.cpp
            TactileMaskMatrix.cpp
            ConvertUtil.cpp
    )
    set(
        SRC_HPP
            DSACON32.hpp
            TactileMatrix.hpp
            TactileMaskMatrix.hpp
            ConvertUtil.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${SRC_CPP} ${SRC_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${ROBWORK_LIBRARIES})
    rw_add_includes(${SUBSYS_NAME} "rwhw/tactile" ${SRC_HPP})

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS ROBWORK_LIBRARIES)
        if(TARGET ${DEP})
            add_dependencies(sdurwhw_tactile ${DEP})
        endif()
    endforeach()

    set(ROBWORKHARDWARE_LIBRARIES ${ROBWORKHARDWARE_LIBRARIES} sdurwhw_tactile PARENT_SCOPE)
endif()

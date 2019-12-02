set(SUBSYS_NAME sdurw_csg)
set(SUBSYS_DESC "A module for doing constructive solid geometry")
set(SUBSYS_DEPS sdurw sdurw_csgjs)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(SRC_FILES CSGModel.cpp CSGModelFactory.cpp CSGConvert.cpp CSGUtil.cpp)

    set(SRC_FILES_HPP CSGModel.hpp CSGModelFactory.hpp CSGConvert.hpp CSGUtil.hpp)

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rwlibs/csg" CSGModel.hpp CSGConvert.hpp)
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    # Make sure dependencies are build before system
    foreach(DEP IN LISTS SUBSYS_DEPS)
        if(TARGET ${DEP})
            add_dependencies(sdurw_csg ${DEP})
        endif()
    endforeach()

    if(CMAKE_VERSION VERSION_GREATER 3.3)
        set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)
    endif()
endif()

set(SUBSYS_NAME sdurwsim_test)
set(SUBSYS_DESC "A library with standard tests for Physics Engines.")
set(SUBSYS_DEPS
    PRIVATE RW::sdurw_geometry
            RW::sdurw_graphics
            RW::sdurw_kinematics
            RW::sdurw_models
            RW::sdurw_proximity
            RW::sdurw_common
    PUBLIC  sdurwsim
            RW::sdurw_trajectory
            RW::sdurw_math
            RW::sdurw_core
)

set(build TRUE)
set(DEFAULT TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} DEPENDS ${SUBSYS_DEPS} ADD_DOC)

if(build)
    set(SRC_FILES
        DynamicWorkCellBuilder.cpp

        EngineTest.cpp

        IntegratorTest.cpp
        IntegratorGravityTest.cpp
        IntegratorRotationTest.cpp
        IntegratorSpringTest.cpp
    )
    set(HPP_FILES
        DynamicWorkCellBuilder.hpp

        EngineTest.hpp

        IntegratorTest.hpp
        IntegratorGravityTest.hpp
        IntegratorRotationTest.hpp
        IntegratorSpringTest.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES})
    rw_add_includes(${SUBSYS_NAME} "rwsimlibs/test" ${HPP_FILES})
    target_link_libraries(${SUBSYS_NAME} ${SUBSYS_DEPS})
    target_include_directories(${SUBSYS_NAME}
        INTERFACE
        $<BUILD_INTERFACE:${RWSIM_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )
endif()

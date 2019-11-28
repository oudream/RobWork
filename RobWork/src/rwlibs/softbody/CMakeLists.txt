set(SUBSYS_NAME sdurw_softbody)
set(SUBSYS_DESC "Various simulations of soft bodies")
set(SUBSYS_DEPS sdurw)

set(build TRUE)
set(DEFAULT FALSE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT})
rw_add_doc(${SUBSYS_NAME})

if(build)
    set(
        SRC_FILES
        ./numerics/TrapMethod.cpp
        ./numerics/FdUtil.cpp
        ./beam/BeamGeometry.cpp
        ./beam/BeamGeometryCuboid.cpp
        ./beam/ModRusselBeamBase.cpp
        ./beam/RusselIntegrand.cpp
        ./beam/EBBeam.cpp
        ./beam/BeamObstaclePlane.cpp
        ./beam/BeamStartGuess.cpp
        ./beam/ModRussel_NLP.cpp
        ./beam/ModRusselBeamIpopt.cpp
    )

    set(
        SRC_FILES_HPP
        ./numerics/TrapMethod.hpp
        ./numerics/FdUtil.hpp
        ./beam/BeamGeometry.hpp
        ./beam/BeamGeometryCuboid.hpp
        ./beam/ModRusselBeamBase.hpp
        ./beam/RusselIntegrand.hpp
        ./beam/EBBeam.hpp
        ./beam/BeamObstaclePlane.hpp
        ./beam/BeamStartGuess.hpp
        ./beam/ModRussel_NLP.hpp
        ./beam/ModRusselBeamIpopt.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${SRC_FILES} ${SRC_FILES_HPP})
    rw_add_includes(softbody "rwlibs/softbody" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${SUBSYS_DEPS})
    add_dependencies(${SUBSYS_NAME} ${SUBSYS_DEPS})

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()

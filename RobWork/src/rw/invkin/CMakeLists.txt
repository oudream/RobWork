set(SUBSYS_NAME sdurw_invkin)
set(SUBSYS_DESC "The core RobWork library!")
set(SUBSYS_DEPS sdurw_core sdurw_math sdurw_kinematics sdurw_models sdurw_trajectory sdurw_proximity)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS})

if(build)

    set(FILES_CPP
        AmbiguityResolver.cpp
        ClosedFormIK.cpp
        IterativeIK.cpp
        IterativeMultiIK.cpp
        CCDSolver.cpp
        ParallelIKSolver.cpp
        PieperSolver.cpp
        IKMetaSolver.cpp
        JacobianIKSolver.cpp
        JacobianIKSolverM.cpp
        ClosedFormIKSolverUR.cpp
        ClosedFormIKSolverKukaIIWA.cpp
    )

    set(FILES_HPP
        AmbiguityResolver.hpp
        ClosedFormIK.hpp
        IterativeIK.hpp
        IterativeMultiIK.hpp
        CCDSolver.hpp
        ParallelIKSolver.hpp
        PieperSolver.hpp
        IKMetaSolver.hpp
        InvKinSolver.hpp
        JacobianIKSolver.hpp
        JacobianIKSolverM.hpp
        ClosedFormIKSolverUR.hpp
        ClosedFormIKSolverKukaIIWA.hpp
    )

    #Compile some needed pathplanning classes, without depending on pathplanning
    add_library(prePathPlanning STATIC ../pathplanning/QConstraint.cpp ../pathplanning/QNormalizer.cpp ../pathplanning/StateConstraint.cpp)
    target_include_directories(prePathPlanning PRIVATE ${RW_ROOT}/src)
    target_link_libraries(prePathPlanning ${SUBSYS_DEPS})

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/invkin" ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw" ../invkin.hpp)

    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} ${SUBSYS_DEPS} PRIVATE prePathPlanning INTERFACE sdurw_pathplanning)

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC ${Boost_INCLUDE_DIR} $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

endif()

set(SUBSYS_NAME sdurw_math)
set(SUBSYS_DESC "This is a gathering of all RobWorks math")
set(SUBSYS_DEPS sdurw_core sdurw_common)

set(build TRUE)
rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ON
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)

    set(FILES_CPP
        CameraMatrix.cpp
        EAA.cpp
        Jacobian.cpp
        Line2D.cpp
        LinearAlgebra.cpp
        InertiaMatrix.cpp
        Math.cpp
        Metric.cpp
        MetricFactory.cpp
        MetricUtil.cpp
        Polynomial.cpp
        PolynomialND.cpp
        PolynomialSolver.cpp
        Pose6D.cpp
        Q.cpp
        Quaternion.cpp
        Rotation2D.cpp
        Rotation3D.cpp
        Rotation3DVector.cpp
        RPY.cpp
        Statistics.cpp
        Transform2D.cpp
        Transform3D.cpp
        Vector2D.cpp
        Vector3D.cpp
        VectorND.cpp
        Vector.cpp
        VelocityScrew6D.cpp
        PerspectiveTransform2D.cpp
        EigenDecomposition.cpp
        Line2DPolar.cpp
        Pose2D.cpp
        Wrench6D.cpp
        ProjectionMatrix.cpp
        Random.cpp
        # serialization.cpp
    )

    set(FILES_HPP
        CameraMatrix.hpp
        EAA.hpp
        Jacobian.hpp
        Line2D.hpp
        LinearAlgebra.hpp
        InertiaMatrix.hpp
        Math.hpp
        Metric.hpp
        MetricFactory.hpp
        MetricUtil.hpp
        Polynomial.hpp
        PolynomialND.hpp
        PolynomialSolver.hpp
        Pose6D.hpp
        Q.hpp
        Quaternion.hpp
        Rotation2D.hpp
        Rotation3D.hpp
        Rotation3DVector.hpp
        RPY.hpp
        # Statistics.hpp
        Transform2D.cpp
        Transform3D.hpp
        Vector2D.hpp
        Vector3D.hpp
        VelocityScrew6D.hpp
        Wrench6D.hpp
        PerspectiveTransform2D.hpp
        EigenDecomposition.hpp
        ProjectionMatrix.hpp
        Random.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/core" ${FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} sdurw_core sdurw_common)

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC $<BUILD_INTERFACE:${EIGEN3_INCLUDE_DIR}> ${Boost_INCLUDE_DIR}
    )
    if(RW_ENABLE_INTERNAL_EIGEN_TARGET)
        target_include_directories(
            ${SUBSYS_NAME} PUBLIC $<INSTALL_INTERFACE:${RW_EXT_INSTALL_DIR}/eigen3>
        )
    else()
        target_include_directories(${SUBSYS_NAME} PUBLIC $<INSTALL_INTERFACE:${EIGEN3_INCLUDE_DIR}>)
    endif()

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

endif()

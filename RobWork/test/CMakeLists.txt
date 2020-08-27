configure_file(${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp.in
               ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp)

set(ROBWORK_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/testfiles)
get_filename_component(ROBWORK_XMLSCHEMAS_DIR ../xml-schemas ABSOLUTE)

set(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
if(MSVC)
    set(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${RW_BUILD_TYPE})
endif()

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
               ${TEST_RUN_OUTPUT_DIR}/TestSuiteConfig.xml)

set(ROBWORK_TESTFILES_DIR ${RW_INSTALL_DIR}/test/testfiles)
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
               ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.install)

set(
    MATH_TEST_SRC
    math/LinearAlgebraTest.cpp
    math/MathSerializationTest.cpp
    math/Pose6DTest.cpp
    math/Rotation3DTest.cpp
    math/RPYTest.cpp
    math/Transform3DTest.cpp
    math/UtilTest.cpp
    math/Vector2DTest.cpp
    math/VelocityScrew6DTest.cpp
    math/Wrench6DTest.cpp
)

set(COMMON_TEST_SRC common/LogTestSuite.cpp common/PropertyTest.cpp common/StringUtilTest.cpp
                    common/SerializationTest.cpp common/ThreadTest.cpp)

set(DEFAULT_TEST_ARGS "--log_level=all")

# Repeat for each test

if(TARGET sdurw_math)
    add_executable(sdurw_math-test test-main.cpp ${MATH_TEST_SRC})
    target_link_libraries(sdurw_math-test sdurw_math)
    add_test(sdurw_math-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_math-test ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw_common AND TARGET sdurw_math)
    add_executable(sdurw_common-test test-main.cpp ${COMMON_TEST_SRC})
    target_link_libraries(sdurw_common-test sdurw_common sdurw_math)
    add_test(sdurw_common-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_common-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    add_executable(sdurw_geometry-test test-main.cpp geometry/GeometryTest.cpp
                                       geometry/GeometryUtilTest.cpp geometry/TriangleUtilTest.cpp)
    target_link_libraries(sdurw_geometry-test sdurw)
    add_test(sdurw_geometry-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_geometry-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    add_executable(sdurw_models-test test-main.cpp models/SerialDeviceTest.cpp
                                     models/PrismaticJointTest.cpp)
    target_link_libraries(sdurw_models-test sdurw)
    add_test(sdurw_models-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_models-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    add_executable(sdurw_invkin-test test-main.cpp invkin/InvKinTest.cpp
                                     invkin/ClosedFormIKSolverURTest.cpp)
    target_link_libraries(sdurw_invkin-test sdurw)
    add_test(sdurw_invkin-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_invkin-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    add_executable(sdurw_loader-test test-main.cpp loaders/TULLoaderTest.cpp)
    target_link_libraries(sdurw_loader-test sdurw)
    add_test(sdurw_loader-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_loader-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    add_executable(sdurw_kinematics-test test-main.cpp kinematics/KinematicsTest.cpp)
    target_link_libraries(sdurw_kinematics-test sdurw)
    add_test(sdurw_kinematics-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_kinematics-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    add_executable(sdurw_trajectory-test test-main.cpp trajectory/PathTest.cpp
                                         trajectory/trajectory.cpp
                                         trajectory/TrajectoryFactoryTest.cpp)
    target_link_libraries(sdurw_trajectory-test sdurw)
    add_test(sdurw_trajectory-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_trajectory-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    add_executable(sdurw_collision-test test-main.cpp collision/CollisionTest.cpp
                                        collision/DistanceTest.cpp)
    target_link_libraries(sdurw_collision-test sdurw_proximitystrategies sdurw)
    add_test(sdurw_collision-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_collision-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    add_executable(sdurw_drawables-test test-main.cpp drawable/DrawableTest.cpp)
    target_link_libraries(sdurw_drawables-test sdurw_opengl sdurw)
    add_test(sdurw_drawables-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_drawables-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    add_executable(sdurw_graphics-test test-main.cpp graphics/GraphicsTest.cpp)
    target_link_libraries(sdurw_graphics-test sdurw)
    add_test(sdurw_graphics-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_graphics-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    add_executable(sdurw_planner-test test-main.cpp pathplanning/PathPlanningTest.cpp)
    target_link_libraries(sdurw_planner-test sdurw_pathplanners sdurw_proximitystrategies sdurw)
    add_test(sdurw_planner-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_planner-test
             ${DEFAULT_TEST_ARGS})
endif()

if(TARGET sdurw)
    if(RW_BUILD_SANDBOX)
        add_executable(sdurw_sandbox-test test-main.cpp sandbox/misc.cpp)
        target_link_libraries(sdurw_sandbox-test sdurw)
        add_test(sdurw_sandbox-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_sandbox-test
                 ${DEFAULT_TEST_ARGS})
        set(SANDBOX_TEST sdurw_sandbox-test)
    endif()
endif()

if(TARGET sdurw_proximitystrategies)
    option(RW_ENABLE_PERFORMANCE_TESTS "Set when you want to build the performance tests"
           ${RW_ENABLE_PERFORMANCE_TESTS})
    if(RW_ENABLE_PERFORMANCE_TESTS)
        add_executable(sdurw_performance-test test-main.cpp performance/collisionStrategy.cpp)
        target_link_libraries(sdurw_performance-test sdurw_proximitystrategies sdurw)
        add_test(sdurw_performance-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_performance-test
                 ${DEFAULT_TEST_ARGS})
        set(PERFORMANCE_TEST sdurw_performance-test)
    endif()
endif()

if(TARGET sdurw)
    # calibration stuff
    add_executable(sdurw_calibration-test test-main.cpp calibration/CalibrationTest.cpp)
    target_link_libraries(sdurw_calibration-test sdurw)
    add_test(sdurw_calibration-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_calibration-test
             ${DEFAULT_TEST_ARGS})
    set(CALIBRATION_TEST sdurw_calibration-test)
endif()

if(TARGET sdurw_algorithms)
    # RANSAC fitting
    add_executable(sdurw_ransac-test test-main.cpp algorithms/RANSACTest.cpp)
    target_link_libraries(sdurw_ransac-test sdurw_algorithms sdurw)
    add_test(sdurw_ransac-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_ransac-test
             ${DEFAULT_TEST_ARGS})
    set(RANSAC_TEST sdurw_ransac-test)
endif()

if(TARGET sdurw_mathematica)
    # Mathematica
    if(RW_HAVE_MATHEMATICA)
        add_executable(sdurw_mathematica-test test-main.cpp mathematica/MathematicaTest.cpp)
        target_link_libraries(sdurw_mathematica-test sdurw_mathematica sdurw)
        add_test(sdurw_mathematica-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_mathematica-test
                 ${DEFAULT_TEST_ARGS})
    endif()
endif()

set(
    all-tests
    sdurw_math-test
    sdurw_common-test
    sdurw_geometry-test
    sdurw_models-test
    sdurw_invkin-test
    sdurw_loader-test
    sdurw_kinematics-test
    sdurw_trajectory-test
    sdurw_collision-test
    sdurw_graphics-test
    sdurw_planner-test
    ${CALIBRATION_TEST} # ${RANSAC_TEST}
    ${SANDBOX_TEST}
    ${PERFORMANCE_TEST}
    ${RW_EXTRA_TESTS}
)

rw_is_targets(sdurw_all-tests-depends TARGETS ${all-tests})
if(${sdurw_all-tests-depends})
    add_custom_target(
        sdurw_all-tests ctest -V
        DEPENDS
            sdurw_math-test
            sdurw_common-test
            sdurw_geometry-test
            sdurw_models-test
            sdurw_invkin-test
            sdurw_loader-test
            sdurw_kinematics-test
            sdurw_trajectory-test
            sdurw_collision-test
            sdurw_graphics-test
            sdurw_planner-test
            ${CALIBRATION_TEST} # ${RANSAC_TEST}
            ${SANDBOX_TEST}
            ${PERFORMANCE_TEST}
            ${RW_EXTRA_TESTS}
        WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}"
    )

    install(
        TARGETS
            sdurw_math-test
            sdurw_common-test
            sdurw_models-test
            sdurw_geometry-test
            sdurw_invkin-test
            sdurw_loader-test
            sdurw_kinematics-test
            sdurw_trajectory-test
            sdurw_collision-test
            sdurw_graphics-test
            sdurw_planner-test
            ${CALIBRATION_TEST} # ${RANSAC_TEST}
            ${SANDBOX_TEST}
            ${PERFORMANCE_TEST}
            DESTINATION
            ${RW_INSTALL_DIR}/test
            COMPONENT
            rwtest
    )

    install(
        FILES ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.install
        DESTINATION ${RW_INSTALL_DIR}/test/TestSuiteConfig.xml
        COMPONENT rwtest
    )
    install(
        DIRECTORY testfiles
        DESTINATION ${RW_INSTALL_DIR}/test
        COMPONENT rwtest
        FILES_MATCHING
        PATTERN ".svn" EXCLUDE
    )

endif()

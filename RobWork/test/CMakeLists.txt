CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp
)

SET(ROBWORK_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/testfiles)

SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
IF(MSVC)
	SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${RW_BUILD_TYPE} )
ENDIF()

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
  ${TEST_RUN_OUTPUT_DIR}/TestSuiteConfig.xml
)

SET(ROBWORK_TESTFILES_DIR ${RW_INSTALL_DIR}/test/testfiles) 
CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.install
)

SET(MATH_TEST_SRC 
  math/EAATest.cpp
  math/LinearAlgebraTest.cpp
  math/MathSerializationTest.cpp
  math/Pose6DTest.cpp 
  math/QuaternionTest.cpp
  math/Rotation3DTest.cpp
  math/RPYTest.cpp
  math/Transform3DTest.cpp
  math/UtilTest.cpp
  math/Vector3DTest.cpp
  math/Vector2DTest.cpp
  math/VelocityScrew6DTest.cpp
  math/Wrench6DTest.cpp
) 

SET(COMMON_TEST_SRC
  common/LogTestSuite.cpp
  common/PropertyTest.cpp
  common/StringUtilTest.cpp
  common/CommonTest.cpp
  common/SerializationTest.cpp
  common/ThreadTest.cpp
)

SET(DEFAULT_TEST_ARGS "--log_level=all")

# Repeat for each test
ADD_EXECUTABLE( sdurw_math-test test-main.cpp ${MATH_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_math-test sdurw)
ADD_TEST( sdurw_math-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_math-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_common-test test-main.cpp ${COMMON_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_common-test sdurw)
ADD_TEST( sdurw_common-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_common-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_geometry-test test-main.cpp geometry/GeometryTest.cpp geometry/GeometryUtilTest.cpp geometry/TriangleUtilTest.cpp)       
TARGET_LINK_LIBRARIES( sdurw_geometry-test sdurw)
ADD_TEST( sdurw_geometry-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_geometry-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_models-test test-main.cpp models/SerialDeviceTest.cpp models/PrismaticJointTest.cpp)       
TARGET_LINK_LIBRARIES( sdurw_models-test sdurw)
ADD_TEST( sdurw_models-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_models-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_invkin-test test-main.cpp invkin/InvKinTest.cpp invkin/ClosedFormIKSolverURTest.cpp)       
TARGET_LINK_LIBRARIES( sdurw_invkin-test sdurw)
ADD_TEST( sdurw_invkin-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_invkin-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_loader-test test-main.cpp loaders/TULLoaderTest.cpp)       
TARGET_LINK_LIBRARIES( sdurw_loader-test sdurw)
ADD_TEST( sdurw_loader-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_loader-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_kinematics-test test-main.cpp kinematics/KinematicsTest.cpp)       
TARGET_LINK_LIBRARIES( sdurw_kinematics-test sdurw)
ADD_TEST( sdurw_kinematics-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_kinematics-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_trajectory-test test-main.cpp trajectory/PathTest.cpp trajectory/trajectory.cpp trajectory/TrajectoryFactoryTest.cpp)       
TARGET_LINK_LIBRARIES( sdurw_trajectory-test sdurw)
ADD_TEST( sdurw_trajectory-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_trajectory-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_collision-test test-main.cpp collision/CollisionTest.cpp collision/DistanceTest.cpp)       
TARGET_LINK_LIBRARIES( sdurw_collision-test sdurw_proximitystrategies sdurw)
ADD_TEST( sdurw_collision-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_collision-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_drawables-test test-main.cpp drawable/DrawableTest.cpp)       
TARGET_LINK_LIBRARIES( sdurw_drawables-test sdurw_opengl sdurw)
ADD_TEST( sdurw_drawables-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_drawables-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_graphics-test test-main.cpp graphics/GraphicsTest.cpp)       
TARGET_LINK_LIBRARIES( sdurw_graphics-test sdurw)
ADD_TEST( sdurw_graphics-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_graphics-test ${DEFAULT_TEST_ARGS})

ADD_EXECUTABLE( sdurw_planner-test test-main.cpp pathplanning/PathPlanningTest.cpp)       
TARGET_LINK_LIBRARIES( sdurw_planner-test sdurw_pathplanners sdurw_proximitystrategies sdurw)
ADD_TEST( sdurw_planner-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_planner-test ${DEFAULT_TEST_ARGS})

IF ( RW_BUILD_SANDBOX )
    ADD_EXECUTABLE( sdurw_sandbox-test test-main.cpp sandbox/misc.cpp)       
    TARGET_LINK_LIBRARIES( sdurw_sandbox-test sdurw)
    ADD_TEST( sdurw_sandbox-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_sandbox-test ${DEFAULT_TEST_ARGS})
    SET(SANDBOX_TEST sdurw_sandbox-test) 
ENDIF()

OPTION(RW_ENABLE_PERFORMANCE_TESTS "Set when you want to build the performance tests" ${RW_ENABLE_PERFORMANCE_TESTS})
IF ( RW_ENABLE_PERFORMANCE_TESTS )
    ADD_EXECUTABLE( sdurw_performance-test test-main.cpp 
    performance/collisionStrategy.cpp)       
    TARGET_LINK_LIBRARIES( sdurw_performance-test sdurw_proximitystrategies sdurw)
    ADD_TEST( sdurw_performance-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_performance-test ${DEFAULT_TEST_ARGS} )
    SET(PERFORMANCE_TEST sdurw_performance-test)     
ENDIF()

# calibration stuff
ADD_EXECUTABLE( sdurw_calibration-test test-main.cpp calibration/CalibrationTest.cpp )
TARGET_LINK_LIBRARIES( sdurw_calibration-test sdurw)
ADD_TEST( sdurw_calibration-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_calibration-test ${DEFAULT_TEST_ARGS} )
SET( CALIBRATION_TEST sdurw_calibration-test )

# RANSAC fitting
ADD_EXECUTABLE( sdurw_ransac-test test-main.cpp algorithms/RANSACTest.cpp )
TARGET_LINK_LIBRARIES( sdurw_ransac-test sdurw_algorithms sdurw)
ADD_TEST( sdurw_ransac-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_ransac-test ${DEFAULT_TEST_ARGS} )
SET( RANSAC_TEST sdurw_ransac-test )

# Mathematica
IF ( RW_HAVE_MATHEMATICA )
	ADD_EXECUTABLE( sdurw_mathematica-test test-main.cpp mathematica/MathematicaTest.cpp)       
	TARGET_LINK_LIBRARIES( sdurw_mathematica-test sdurw_mathematica sdurw)
	ADD_TEST( sdurw_mathematica-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurw_mathematica-test ${DEFAULT_TEST_ARGS})
ENDIF()

ADD_CUSTOM_TARGET(sdurw_all-tests ctest -V 
  DEPENDS sdurw_math-test sdurw_common-test sdurw_geometry-test sdurw_models-test sdurw_invkin-test 
          sdurw_loader-test sdurw_kinematics-test 
          sdurw_trajectory-test sdurw_collision-test 
          sdurw_graphics-test sdurw_planner-test ${CALIBRATION_TEST} #${RANSAC_TEST}
          ${SANDBOX_TEST} ${PERFORMANCE_TEST} ${RW_EXTRA_TESTS}
    WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

INSTALL(TARGETS 
            sdurw_math-test sdurw_common-test sdurw_models-test sdurw_geometry-test 
            sdurw_invkin-test sdurw_loader-test sdurw_kinematics-test 
            sdurw_trajectory-test sdurw_collision-test 
            sdurw_graphics-test sdurw_planner-test ${CALIBRATION_TEST} #${RANSAC_TEST}
            ${SANDBOX_TEST} ${PERFORMANCE_TEST}
        DESTINATION ${RW_INSTALL_DIR}/test COMPONENT rwtest)

INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.install
        DESTINATION ${RW_INSTALL_DIR}/test/TestSuiteConfig.xml COMPONENT rwtest)
INSTALL(DIRECTORY
        testfiles
        DESTINATION ${RW_INSTALL_DIR}/test
        COMPONENT rwtest
        FILES_MATCHING 
            PATTERN ".svn" EXCLUDE        
        )

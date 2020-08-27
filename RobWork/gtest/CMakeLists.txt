include_directories(${GTEST_INCLUDE_DIRS})

# ##################################################################################################
# Standard Macro
# ##################################################################################################

macro(ADD_RW_GTEST target)
    add_test(NAME ${target} COMMAND $<TARGET_FILE:${target}>)
    add_custom_target(
        ${target}_report-makedir
        COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${target}>/gtest_reports
        COMMENT "Creating directory gtest_reports if it does not exist."
    )
    add_custom_target(
        ${target}_report
        COMMAND
            $<TARGET_FILE:${target}>
            --gtest_output=xml:$<TARGET_FILE_DIR:${target}>/gtest_reports/${target}.xml
        DEPENDS ${target} ${target}_report-makedir
    )
    set(REPORT_TARGETS ${REPORT_TARGETS} ${target}_report)
    if(GTEST_SHARED_LIBS)
        target_compile_definitions(${target} PRIVATE GTEST_LINKED_AS_SHARED_LIBRARY=1)
        if(MSVC)
            target_compile_options(${target} PRIVATE /wd4251 /wd4275)
        endif()
    endif()
endmacro()

# ##################################################################################################
# RobWork main function for initialization (link with this if needed).
# ##################################################################################################

set(RWMAIN_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw_core)

set(RWMAIN_TEST_SRC TestEnvironment.cpp test-main.cpp)
add_library(sdurw-gtest-main STATIC ${RWMAIN_TEST_SRC})
target_link_libraries(sdurw-gtest-main ${RWMAIN_TEST_LIBRARIES})

# ##################################################################################################
# Core
# ##################################################################################################

set(CORE_TEST_LIBRARIES sdurw-gtest-main ${GTEST_LIBRARIES} sdurw_core)

set(
    CORE_TEST_SRC
    core/AnyPtrTest.cpp
    core/EventTest.cpp
    core/PluginTest.cpp
    core/PtrTest.cpp
    core/PtrBoostTest.cpp
    core/StringUtilTest.cpp
)

add_executable(sdurw_core-gtest ${CORE_TEST_SRC})
target_link_libraries(sdurw_core-gtest ${CORE_TEST_LIBRARIES})
add_rw_gtest(sdurw_core-gtest)

# Create dummy plugins for testing
add_library(test_plugin.rwplugin MODULE core/TestPlugin.cpp)
target_link_libraries(test_plugin.rwplugin sdurw_core)
set_target_properties(
    test_plugin.rwplugin
    PROPERTIES LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)

# Create XML file for lazy-loading of Test plugin
file(
    GENERATE
    OUTPUT "$<TARGET_FILE_DIR:test_plugin.rwplugin>/test_plugin.rwplugin.xml"
    INPUT ${CMAKE_CURRENT_SOURCE_DIR}/core/test_plugin.rwplugin.xml.in
)

# ##################################################################################################
# Common
# ##################################################################################################

set(COMMON_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw_common sdurw_math)

set(COMMON_TEST_SRC common/IteratorTest.cpp common/PairMapTest.cpp common/ProgramOptionTest.cpp
                    common/TimerTest.cpp)

add_executable(sdurw_common-gtest ${COMMON_TEST_SRC})
target_link_libraries(sdurw_common-gtest ${COMMON_TEST_LIBRARIES})
add_rw_gtest(sdurw_common-gtest)

# ##################################################################################################
# Geometry
# ##################################################################################################
set(
    GEOMETRY_TEST_LIBRARIES
    ${GTEST_BOTH_LIBRARIES}
    sdurw
    ${XERCESC_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${QHULL_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

set(
    GEOMETRY_TEST_SRC
    geometry/DelaunayTest.cpp
    geometry/HyperSphereTest.cpp
    geometry/ImplicitTorusTest.cpp
    geometry/IndexedTriMeshTest.cpp
    geometry/IntersectUtilTest.cpp
    geometry/PlaneTest.cpp
    geometry/PolygonTest.cpp
    geometry/QHullTest.cpp
    geometry/QuadraticCurveTest.cpp
    geometry/TriangulateTest.cpp
)
add_executable(sdurw_geometry-gtest ${GEOMETRY_TEST_SRC})
target_link_libraries(sdurw_geometry-gtest ${GEOMETRY_TEST_LIBRARIES})
add_rw_gtest(sdurw_geometry-gtest)

# ##################################################################################################
# Graphics
# ##################################################################################################
set(GRAPHICS_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw)

set(GRAPHICS_TEST_SRC graphics/SceneGraphTest.cpp graphics/WorkCellSceneTest.cpp)
add_executable(sdurw_graphics-gtest ${GRAPHICS_TEST_SRC})
target_link_libraries(sdurw_graphics-gtest ${GRAPHICS_TEST_LIBRARIES})
add_rw_gtest(sdurw_graphics-gtest)

# ##################################################################################################
# Inverse Kinematics
# ##################################################################################################
set(INVKIN_TEST_LIBRARIES sdurw-gtest-main ${GTEST_LIBRARIES} sdurw)

set(INVKIN_TEST_SRC invkin/ClosedFormIKSolverKukaIIWATest.cpp invkin/ParallelIKSolverTest.cpp
                    invkin/JacobianIKSolverTest.cpp)
add_executable(sdurw_invkin-gtest ${INVKIN_TEST_SRC})
target_link_libraries(sdurw_invkin-gtest ${INVKIN_TEST_LIBRARIES})
add_rw_gtest(sdurw_invkin-gtest)

# ##################################################################################################
# Kinematics
# ##################################################################################################

set(KINEMATICS_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw)

set(KINEMATICS_TEST_SRC kinematics/StaticFrameGroupsTest.cpp)
add_executable(sdurw_kinematics-gtest ${KINEMATICS_TEST_SRC})
target_link_libraries(sdurw_kinematics-gtest ${KINEMATICS_TEST_LIBRARIES})
add_rw_gtest(sdurw_kinematics-gtest)

# ##################################################################################################
# Loaders
# ##################################################################################################

set(
    LOADERS_TEST_LIBRARIES
    sdurw-gtest-main
    ${GTEST_LIBRARIES}
    sdurw
    sdurw_proximitystrategies
    ${XERCESC_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${QHULL_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

set(LOADERS_TEST_SRC loaders/DOMProximitySetupSaver.cpp loaders/DOMPropertyMap.cpp
                     loaders/ImageLoaderTest.cpp loaders/PathLoaderCSVTest.cpp)
add_executable(sdurw_loaders-gtest ${LOADERS_TEST_SRC})
target_link_libraries(sdurw_loaders-gtest ${LOADERS_TEST_LIBRARIES})
add_rw_gtest(sdurw_loaders-gtest)

# ##################################################################################################
# Math
# ##################################################################################################

set(MATH_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw)

set(
    MATH_TEST_SRC
    math/EAATest.cpp
    math/MetricFactoryTest.cpp
    math/PolynomialTest.cpp
    math/SerializationTest.cpp
    math/StatisticsTest.cpp
    math/QuaternionTest.cpp
    math/Vector3DTest.cpp
    math/VectorNDTest.cpp
)

add_executable(sdurw_math-gtest ${MATH_TEST_SRC})
target_link_libraries(sdurw_math-gtest ${MATH_TEST_LIBRARIES})
add_rw_gtest(sdurw_math-gtest)

# ##################################################################################################
# Models
# ##################################################################################################

set(MODELS_TEST_LIBRARIES sdurw-gtest-main ${GTEST_LIBRARIES} sdurw)

set(MODELS_TEST_SRC models/JointTest.cpp models/ParallelDeviceTest.cpp models/ParallelLegTest.cpp
                    models/WorkCellTest.cpp)
add_executable(sdurw_models-gtest ${MODELS_TEST_SRC})
target_link_libraries(sdurw_models-gtest ${MODELS_TEST_LIBRARIES})
add_rw_gtest(sdurw_models-gtest)

# ##################################################################################################
# Pathoptimization
# ##################################################################################################

set(PATHOPTIMIZATION_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw_pathoptimization sdurw)

set(PATHOPTIMIZATION_TEST_SRC pathoptimization/ClearanceOptimizerTest.cpp
                              pathoptimization/PathLengthOptimizerTest.cpp)
add_executable(sdurw_pathoptimization-gtest ${PATHOPTIMIZATION_TEST_SRC})
target_link_libraries(sdurw_pathoptimization-gtest ${PATHOPTIMIZATION_TEST_LIBRARIES})
add_rw_gtest(sdurw_pathoptimization-gtest)

# ##################################################################################################
# Proximity
# ##################################################################################################

set(PROXIMITY_TEST_LIBRARIES sdurw-gtest-main ${GTEST_LIBRARIES} sdurw sdurw_proximitystrategies)

set(
    PROXIMITY_TEST_SRC
    proximity/BVTreeTest.cpp
    proximity/CollisionStrategyTest.cpp
    proximity/CollisionToleranceStrategyTest.cpp
    proximity/DistanceMultiStrategyTest.cpp
    proximity/DistanceStrategyTest.cpp
    proximity/ProximityStrategyTest.cpp
    proximity/DistanceCalculatorTest.cpp
    proximity/ProximityModelTest.cpp
    proximity/ProximityCalculatorTest.cpp
)
add_executable(sdurw_proximity-gtest ${PROXIMITY_TEST_SRC})
target_link_libraries(sdurw_proximity-gtest ${PROXIMITY_TEST_LIBRARIES})
add_rw_gtest(sdurw_proximity-gtest)

# ##################################################################################################
# Sensor
# ##################################################################################################

set(SENSOR_TEST_SRC sensor/TactileArrayTest.cpp)
add_executable(sdurw_sensor-gtest ${SENSOR_TEST_SRC})
target_link_libraries(sdurw_sensor-gtest ${GTEST_BOTH_LIBRARIES} sdurw)
add_rw_gtest(sdurw_sensor-gtest)

# ##################################################################################################
# Task
# ##################################################################################################

set(
    TASK_TEST_LIBRARIES
    sdurw-gtest-main
    ${GTEST_LIBRARIES}
    sdurw_task
    sdurw
    ${XERCESC_LIBRARIES}
    ${ASSIMP_LIBRARIES}
    ${QHULL_LIBRARIES}
    ${CMAKE_DL_LIBS}
)

set(TASK_TEST_SRC task/TaskLoaderTest.cpp task/TaskSaverTest.cpp task/GraspTaskTest.cpp)
add_executable(sdurw_task-gtest ${TASK_TEST_SRC})
target_link_libraries(sdurw_task-gtest ${TASK_TEST_LIBRARIES})
add_rw_gtest(sdurw_task-gtest)

# ##################################################################################################
# Trajectory
# ##################################################################################################

set(TRAJECTORY_TEST_LIBRARIES ${GTEST_BOTH_LIBRARIES} sdurw)

set(TRAJECTORY_TEST_SRC trajectory/PathTest.cpp trajectory/ParabolicBlendTest.cpp)
add_executable(sdurw_trajectory-gtest ${TRAJECTORY_TEST_SRC})
target_link_libraries(sdurw_trajectory-gtest ${TRAJECTORY_TEST_LIBRARIES})
add_rw_gtest(sdurw_trajectory-gtest)

# ##################################################################################################
# Target for generation of all detailed reports
# ##################################################################################################

add_custom_target(
    sdurw-gtest_reports
    DEPENDS ${REPORT_TARGETS}
    COMMENT "Running Google Tests to generate detailed reports."
)

# ##################################################################################################
# Do not build these as part of an ordinary build
# ##################################################################################################

set_target_properties(
    sdurw-gtest_reports ${REPORT_TARGETS}
    PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1
)

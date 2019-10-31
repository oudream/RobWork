INCLUDE_DIRECTORIES(${GTEST_INCLUDE_DIRS})

########################################################################
# Standard Macro
########################################################################

MACRO(ADD_RW_GTEST target)
	ADD_TEST(NAME ${target} COMMAND $<TARGET_FILE:${target}>)
	ADD_CUSTOM_TARGET(${target}_report-makedir
		COMMAND ${CMAKE_COMMAND} -E make_directory $<TARGET_FILE_DIR:${target}>/gtest_reports
		COMMENT "Creating directory gtest_reports if it does not exist."
	)
	ADD_CUSTOM_TARGET(${target}_report
		COMMAND $<TARGET_FILE:${target}> --gtest_output=xml:$<TARGET_FILE_DIR:${target}>/gtest_reports/${target}.xml
		DEPENDS ${target} ${target}_report-makedir
	)
	SET(REPORT_TARGETS ${REPORT_TARGETS} ${target}_report)
	IF(GTEST_SHARED_LIBS)
	  TARGET_COMPILE_DEFINITIONS(${target} PRIVATE GTEST_LINKED_AS_SHARED_LIBRARY=1)
	  IF(MSVC)
		TARGET_COMPILE_OPTIONS(${target} PRIVATE /wd4251 /wd4275)
	  ENDIF()
	ENDIF()
ENDMACRO(ADD_RW_GTEST)

########################################################################
# RobWork main function for initialization (link with this if needed).
########################################################################

SET(RWMAIN_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 sdurw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${CMAKE_DL_LIBS}
 )

SET(RWMAIN_TEST_SRC
  TestEnvironment.cpp
  test-main.cpp
)
ADD_LIBRARY( sdurw-gtest-main STATIC ${RWMAIN_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw-gtest-main ${RWMAIN_TEST_LIBRARIES})

########################################################################
# Common
########################################################################

SET(COMMON_TEST_LIBRARIES
 sdurw-gtest-main
 ${GTEST_LIBRARIES}
 sdurw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
)

SET(COMMON_TEST_SRC
  common/CommonTest.cpp
  common/IteratorTest.cpp
  common/PairMapTest.cpp
  common/PluginTest.cpp
)
ADD_EXECUTABLE( sdurw_common-gtest ${COMMON_TEST_SRC})
TARGET_LINK_LIBRARIES( sdurw_common-gtest ${COMMON_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_common-gtest)

# Create dummy plugins for testing
ADD_LIBRARY(test_plugin.rwplugin MODULE common/TestPlugin.cpp)
TARGET_LINK_LIBRARIES(test_plugin.rwplugin sdurw)
SET_TARGET_PROPERTIES(test_plugin.rwplugin
  PROPERTIES
  LIBRARY_OUTPUT_DIRECTORY ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}
)

# Create XML file for lazy-loading of Test plugin
FILE(GENERATE OUTPUT "$<TARGET_FILE_DIR:test_plugin.rwplugin>/test_plugin.rwplugin.xml" INPUT ${CMAKE_CURRENT_SOURCE_DIR}/common/test_plugin.rwplugin.xml.in)

########################################################################
# Geometry
########################################################################
SET(GEOMETRY_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 sdurw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${CMAKE_DL_LIBS}
)

SET(GEOMETRY_TEST_SRC
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
ADD_EXECUTABLE( sdurw_geometry-gtest ${GEOMETRY_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_geometry-gtest ${GEOMETRY_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_geometry-gtest)

########################################################################
# Graphics
########################################################################
SET(GRAPHICS_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 sdurw
)

SET(GRAPHICS_TEST_SRC
  graphics/SceneGraphTest.cpp
  graphics/WorkCellSceneTest.cpp
)
ADD_EXECUTABLE( sdurw_graphics-gtest ${GRAPHICS_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_graphics-gtest ${GRAPHICS_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_graphics-gtest)

########################################################################
# Inverse Kinematics
########################################################################
SET(INVKIN_TEST_LIBRARIES
 sdurw-gtest-main
 ${GTEST_LIBRARIES}
 sdurw
)

SET(INVKIN_TEST_SRC
  invkin/ClosedFormIKSolverKukaIIWATest.cpp
  invkin/ParallelIKSolverTest.cpp
)
ADD_EXECUTABLE( sdurw_invkin-gtest ${INVKIN_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_invkin-gtest ${INVKIN_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_invkin-gtest)

########################################################################
# Kinematics
########################################################################

SET(KINEMATICS_TEST_LIBRARIES
  ${GTEST_BOTH_LIBRARIES}
  sdurw
)

SET(KINEMATICS_TEST_SRC
  kinematics/StaticFrameGroupsTest.cpp
)
ADD_EXECUTABLE( sdurw_kinematics-gtest ${KINEMATICS_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_kinematics-gtest ${KINEMATICS_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_kinematics-gtest)

########################################################################
# Loaders
########################################################################

SET(LOADERS_TEST_LIBRARIES
 sdurw-gtest-main
 ${GTEST_LIBRARIES}
 sdurw
 sdurw_proximitystrategies
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${CMAKE_DL_LIBS}
 )

SET(LOADERS_TEST_SRC
  loaders/DOMProximitySetupSaver.cpp
  loaders/DOMPropertyMap.cpp
  loaders/ImageLoaderTest.cpp
  loaders/PathLoaderCSVTest.cpp
)
ADD_EXECUTABLE( sdurw_loaders-gtest ${LOADERS_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_loaders-gtest ${LOADERS_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_loaders-gtest)

########################################################################
# Math
########################################################################

SET(MATH_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 sdurw
 )

SET(MATH_TEST_SRC
  math/MetricFactoryTest.cpp
  math/PolynomialTest.cpp
  math/SerializationTest.cpp
  math/StatisticsTest.cpp
)
ADD_EXECUTABLE( sdurw_math-gtest ${MATH_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_math-gtest ${MATH_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_math-gtest)

########################################################################
# Models
########################################################################

SET(MODELS_TEST_LIBRARIES
 sdurw-gtest-main
 ${GTEST_LIBRARIES}
 sdurw
 )

SET(MODELS_TEST_SRC
  models/JointTest.cpp
  models/ParallelDeviceTest.cpp
  models/ParallelLegTest.cpp
  models/WorkCellTest.cpp
)
ADD_EXECUTABLE( sdurw_models-gtest ${MODELS_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_models-gtest ${MODELS_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_models-gtest)

########################################################################
# Pathoptimization
########################################################################

SET(PATHOPTIMIZATION_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 sdurw_pathoptimization
 sdurw
 )

SET(PATHOPTIMIZATION_TEST_SRC
  pathoptimization/ClearanceOptimizerTest.cpp
  pathoptimization/PathLengthOptimizerTest.cpp
)
ADD_EXECUTABLE( sdurw_pathoptimization-gtest ${PATHOPTIMIZATION_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_pathoptimization-gtest ${PATHOPTIMIZATION_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_pathoptimization-gtest)

########################################################################
# Proximity
########################################################################

SET(PROXIMITY_TEST_LIBRARIES
 sdurw-gtest-main
 ${GTEST_LIBRARIES}
 sdurw
 )

SET(PROXIMITY_TEST_SRC
  proximity/BVTreeTest.cpp
  proximity/CollisionStrategy.cpp
  proximity/CollisionToleranceStrategy.cpp
  proximity/DistanceMultiStrategy.cpp
  proximity/DistanceStrategy.cpp
  proximity/ProximityStrategy.cpp
  proximity/DistanceCalculatorTest.cpp
)
ADD_EXECUTABLE( sdurw_proximity-gtest ${PROXIMITY_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_proximity-gtest ${PROXIMITY_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_proximity-gtest)

########################################################################
# Sensor
########################################################################

SET(SENSOR_TEST_SRC
  sensor/TactileArrayTest.cpp
)
ADD_EXECUTABLE( sdurw_sensor-gtest ${SENSOR_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_sensor-gtest ${GTEST_BOTH_LIBRARIES} sdurw)
ADD_RW_GTEST(sdurw_sensor-gtest)

########################################################################
# Task
########################################################################

SET(TASK_TEST_LIBRARIES
 sdurw-gtest-main
 ${GTEST_LIBRARIES}
 sdurw_task
 sdurw
 ${XERCESC_LIBRARIES}
 ${ASSIMP_LIBRARIES}
 ${QHULL_LIBRARIES}
 ${CMAKE_DL_LIBS}
 )

SET(TASK_TEST_SRC
  task/GraspTaskTest.cpp
)
ADD_EXECUTABLE( sdurw_task-gtest ${TASK_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_task-gtest ${TASK_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_task-gtest)

########################################################################
# Trajectory
########################################################################

SET(TRAJECTORY_TEST_LIBRARIES
 ${GTEST_BOTH_LIBRARIES}
 sdurw
)

SET(TRAJECTORY_TEST_SRC
  trajectory/PathTest.cpp
)
ADD_EXECUTABLE( sdurw_trajectory-gtest ${TRAJECTORY_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurw_trajectory-gtest ${TRAJECTORY_TEST_LIBRARIES})
ADD_RW_GTEST(sdurw_trajectory-gtest)

########################################################################
# Target for generation of all detailed reports
########################################################################

ADD_CUSTOM_TARGET(sdurw-gtest_reports
	DEPENDS ${REPORT_TARGETS}
	COMMENT "Running Google Tests to generate detailed reports."
)

########################################################################
# Do not build these as part of an ordinary build
########################################################################

SET_TARGET_PROPERTIES(sdurw-gtest_reports ${REPORT_TARGETS} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

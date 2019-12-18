INCLUDE_DIRECTORIES(${RW_BUILD_WITH_GTEST_INCLUDE_DIRS})
########################################################################
# Standard Macro
########################################################################

MACRO(ADD_RWS_GTEST target)
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
	IF(RW_BUILD_WITH_GTEST_SHARED_LIBS)
	  TARGET_COMPILE_DEFINITIONS(${target} PRIVATE GTEST_LINKED_AS_SHARED_LIBRARY=1)
	  IF(MSVC)
		TARGET_COMPILE_OPTIONS(${target} PRIVATE /wd4251 /wd4275)
	  ENDIF()
	ENDIF()
ENDMACRO(ADD_RWS_GTEST)

########################################################################
# Configuration files for the testfiles path.
########################################################################

SET(ROBWORK_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/testfiles)

CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.generated
)

FILE(GENERATE OUTPUT $<TARGET_FILE_DIR:sdurws_sdurws-gtest>/TestSuiteConfig.xml INPUT ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.generated)

########################################################################
# RobWorkStudio main function for initialization (link with this if needed).
########################################################################

SET(RWSMAIN_TEST_LIBRARIES
 ${RW_BUILD_WITH_LIBRARIES_GTEST}
 ${ROBWORK_LIBRARIES}
)

SET(RWSMAIN_TEST_SRC
  TestEnvironment.cpp
  test-main.cpp
)
ADD_LIBRARY( sdurws-gtest-main STATIC ${RWSMAIN_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurws-gtest-main ${RWSMAIN_TEST_LIBRARIES})

########################################################################
# RWS
########################################################################

SET(RWS_TEST_LIBRARIES
  sdurws
  qtpropertybrowser
  ${QT_LIBRARIES}
  ${RW_BUILD_WITH_LIBRARIES_GTEST}
  ${Boost_LIBRARIES}
  ${ROBWORK_LIBRARIES}
)

message(STATUS "#### RW_BUILD_WITH: '${RW_BUILD_WITH_LIBRARIES_GTEST}'")
message(STATUS "#### BOOST_LIBS: '${Boost_LIBRARIES}'")
message(STATUS "#### ROBWORK LIBS '${ROBWORK_LIBRARIES}'")

SET(RWS_TEST_SRC
  rws/RobWorkStudioTest.cpp
)
ADD_EXECUTABLE( sdurws_sdurws-gtest ${RWS_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurws_sdurws-gtest ${RWS_TEST_LIBRARIES})
ADD_RWS_GTEST(sdurws_sdurws-gtest)


########################################################################
# Test of simulated sensors
########################################################################

SET(SENSOR_TEST_LIBRARIES
sdurws-gtest-main
  sdurws
  qtpropertybrowser
  ${QT_LIBRARIES}
  ${Boost_LIBRARIES}
  ${ROBWORK_LIBRARIES}
 )


SET(SENSOR_TEST_SRC
  rws/SimulatedSensorTest.cpp
)
ADD_EXECUTABLE( sdurws_sensor-gtest ${SENSOR_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurws_sensor-gtest ${SENSOR_TEST_LIBRARIES} sdurw)
ADD_RWS_GTEST(sdurws_sensor-gtest)

########################################################################
# Target for generation of all detailed reports
########################################################################

ADD_CUSTOM_TARGET(sdurws-gtest_reports
	DEPENDS ${REPORT_TARGETS}
	COMMENT "Running Google Tests to generate detailed reports."
)

########################################################################
# Do not build these as part of an ordinary build
########################################################################

SET_TARGET_PROPERTIES(sdurws-gtest_reports ${REPORT_TARGETS} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

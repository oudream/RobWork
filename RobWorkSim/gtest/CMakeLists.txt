########################################################################
# Standard Macro
########################################################################

MACRO(ADD_RWSIM_GTEST target)
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
ENDMACRO(ADD_RWSIM_GTEST)

########################################################################
# Simulator
########################################################################

SET(SIMULATOR_TEST_LIBRARIES
  sdurwsim_test
  sdurwsim
  ${RW_BUILD_WITH_LIBRARIES_GTEST}
  RW::sdurw
)

SET(SIMULATOR_TEST_SRC
  simulator/PhysicsEngineTest.cpp
)

ADD_EXECUTABLE(sdurwsim_simulator-gtest ${SIMULATOR_TEST_SRC})
TARGET_LINK_LIBRARIES(sdurwsim_simulator-gtest ${SIMULATOR_TEST_LIBRARIES})
ADD_RWSIM_GTEST(sdurwsim_simulator-gtest)

########################################################################
# Target for generation of all detailed reports
########################################################################

ADD_CUSTOM_TARGET(sdurwsim-gtest_reports
	DEPENDS ${REPORT_TARGETS}
	COMMENT "Running Google Tests to generate detailed reports."
)

########################################################################
# Do not build these as part of an ordinary build
########################################################################

SET_TARGET_PROPERTIES(sdurwsim-gtest_reports ${REPORT_TARGETS} PROPERTIES EXCLUDE_FROM_ALL 1 EXCLUDE_FROM_DEFAULT_BUILD 1)

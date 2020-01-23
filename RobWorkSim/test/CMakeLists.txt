configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.hpp)


# setup the test file configuration
SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY})
IF(MSVC)
	SET(TEST_RUN_OUTPUT_DIR ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/${CMAKE_BUILD_TYPE} )
ENDIF()

SET(ROBWORKSIM_TESTFILES_DIR ${CMAKE_CURRENT_SOURCE_DIR}/testfiles)
CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
  ${TEST_RUN_OUTPUT_DIR}/TestSuiteConfig.xml
)

SET(ROBWORKSIM_TESTFILES_DIR ${RWSIM_INSTALL_DIR}/test/testfiles)
CONFIGURE_FILE(
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.in
  ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.install
)

SET(COMMON_TEST_SRC
  common/CommonTest.cpp
)

SET(DYNAMICS_TEST_SRC
  dynamics/ConstraintTest.cpp
)

# Repeat for each test 
add_executable(sdurwsim_common-test ${DO_EXCLUDE} test-main.cpp ${COMMON_TEST_SRC})
target_link_libraries(sdurwsim_common-test PRIVATE sdurwsim )
ADD_TEST( sdurwsim_common-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurwsim_common-test )

add_executable(sdurwsim_dynamics-test ${DO_EXCLUDE} test-main.cpp ${DYNAMICS_TEST_SRC})
target_link_libraries(sdurwsim_dynamics-test PRIVATE sdurwsim RW::sdurw )
ADD_TEST( sdurwsim_dynamics-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurwsim_dynamics-test )

add_executable(sdurwsim_tasksimulation-test ${DO_EXCLUDE} test-main.cpp tasksimulation/GraspTaskSimulation.cpp)
if(RWSIM_HAVE_ODE)
	target_link_libraries(sdurwsim_tasksimulation-test PRIVATE sdurwsim_ode)
endif()
target_link_libraries(sdurwsim_tasksimulation-test PRIVATE sdurwsim RW::sdurw_task RW::sdurw)
ADD_TEST( sdurwsim_tasksimulation-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurwsim_tasksimulation-test )

add_executable(sdurwsim_sensors-test ${DO_EXCLUDE} test-main.cpp sensors/FTSensorTest.cpp sensors/TactileArraySensorTest.cpp)
if(RWSIM_HAVE_ODE)
	target_link_libraries(sdurwsim_sensors-test PRIVATE sdurwsim_ode)
endif()
target_link_libraries(sdurwsim_sensors-test PRIVATE sdurwsim RW::sdurw)
ADD_TEST( sdurwsim_sensors-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurwsim_sensors-test )

if(RWSIM_HAVE_ODE)
	add_executable(sdurwsim_ode-test ${DO_EXCLUDE} test-main.cpp ode/ODEControlTest.cpp ) #  ode/ODESimpleTest.cpp
	target_link_libraries(sdurwsim_ode-test PRIVATE sdurwsim_ode sdurwsim RW::sdurw)
	ADD_TEST( sdurwsim_ode-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurwsim_ode-test )
endif()

ADD_CUSTOM_TARGET(tests ctest -V 
  DEPENDS sdurwsim_common-test ${TEST_DEPENDENCIES}
    WORKING_DIRECTORY "${CMAKE_RUNTIME_OUTPUT_DIRECTORY}")

INSTALL(TARGETS 
            sdurwsim_common-test 
        DESTINATION ${RWSIM_INSTALL_DIR}/test)

IF(RWSIM_HAVE_ODE)
	INSTALL(TARGETS 
            sdurwsim_ode-test
        DESTINATION ${RWSIM_INSTALL_DIR}/test)
ENDIF()

INSTALL(FILES ${CMAKE_CURRENT_SOURCE_DIR}/TestSuiteConfig.xml.install
        DESTINATION ${RWSIM_INSTALL_DIR}/test/TestSuiteConfig.xml)
INSTALL(DIRECTORY
        testfiles
        DESTINATION ${RWSIM_INSTALL_DIR}/test
        FILES_MATCHING 
            PATTERN ".svn" EXCLUDE        
        )
       

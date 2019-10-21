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



SET(TEST_LIBRARIES
  ${ROBWORKSIM_LIBRARIES}
  ${ROBWORK_LIBRARIES}
  #${ROBWORKSTUDIO_LIBRARIES}
  #${RW_DRAWABLE_LIBRARY_LIST}
  #${RW_PATHPLANNERS_LIBRARY_LIST}
  #${RW_PROXIMITYSTRATEGIES_LIBRARY_LIST}
  #${ROBWORK_LIBRARIES}
)

IF(RWSIM_HAVE_ODE)
	SET(TEST_LIBRARIES sdurwsim_ode ${ODE_LIBRARY} ${TEST_LIBRARIES})
ENDIF()

SET(COMMON_TEST_SRC
  common/CommonTest.cpp
)

SET(DYNAMICS_TEST_SRC
  dynamics/ConstraintTest.cpp
)

# Repeat for each test 
ADD_EXECUTABLE( sdurwsim_common-test ${DO_EXCLUDE} test-main.cpp ${COMMON_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurwsim_common-test ${TEST_LIBRARIES} )
ADD_TEST( sdurwsim_common-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurwsim_common-test )

ADD_EXECUTABLE( sdurwsim_dynamics-test ${DO_EXCLUDE} test-main.cpp ${DYNAMICS_TEST_SRC})       
TARGET_LINK_LIBRARIES( sdurwsim_dynamics-test ${TEST_LIBRARIES} )
ADD_TEST( sdurwsim_dynamics-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurwsim_dynamics-test )

ADD_EXECUTABLE( sdurwsim_tasksimulation-test ${DO_EXCLUDE} test-main.cpp tasksimulation/GraspTaskSimulation.cpp)       
TARGET_LINK_LIBRARIES( sdurwsim_tasksimulation-test ${TEST_LIBRARIES} )
ADD_TEST( sdurwsim_tasksimulation-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurwsim_tasksimulation-test )

ADD_EXECUTABLE( sdurwsim_sensors-test ${DO_EXCLUDE} test-main.cpp sensors/FTSensorTest.cpp sensors/TactileArraySensorTest.cpp)       
TARGET_LINK_LIBRARIES( sdurwsim_sensors-test ${TEST_LIBRARIES} )
ADD_TEST( sdurwsim_sensors-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurwsim_sensors-test )

IF(RWSIM_HAVE_ODE)
	ADD_EXECUTABLE( sdurwsim_ode-test ${DO_EXCLUDE} test-main.cpp ode/ODEControlTest.cpp ) #  ode/ODESimpleTest.cpp        
	TARGET_LINK_LIBRARIES( sdurwsim_ode-test ${TEST_LIBRARIES} )
	ADD_TEST( sdurwsim_ode-test ${CMAKE_RUNTIME_OUTPUT_DIRECTORY}/sdurwsim_ode-test )
ENDIF()

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
       

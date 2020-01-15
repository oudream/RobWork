SET(SRC_FILES 
	DynamicWorkCellBuilder.cpp

	EngineTest.cpp
	
	IntegratorTest.cpp
	IntegratorGravityTest.cpp
	IntegratorRotationTest.cpp
	IntegratorSpringTest.cpp
)

rw_add_library(sdurwsim_test ${SRC_FILES})
target_link_libraries(sdurwsim_test PUBLIC sdurwsim)
target_include_directories(sdurwsim_test
    INTERFACE
    $<BUILD_INTERFACE:${RWSIM_ROOT}/src> $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
)

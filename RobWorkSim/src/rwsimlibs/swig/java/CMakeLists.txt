set(SUBSYS_NAME sdurwsim_java)
set(SUBSYS_DESC "Interface for accessing RobWorkSim from java.")
set(SUBSYS_DEPS RW::sdurw_java RWS::sdurws_java sdurwsim)
set(build TRUE)

set(DEFAULT TRUE)
set(REASON)
if(NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

    find_package(Java)
    find_package(JNI)
    if(NOT (JAVA_FOUND AND JNI_FOUND))
        set(DEFAULT false)
        set(REASON "JAVA or JNI not found!")
    endif()
endif()

rw_subsys_option(
    build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT}
    REASON ${REASON}
    DEPENDS ${SUBSYS_DEPS}
    ADD_DOC
)

if(build)
    include(UseJava)
    include(UseSWIG)

    include_directories(${RWSIM_ROOT}/src ${RW_ROOT}/src)

    include_directories(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})

    rw_add_swig(
        sdurwsim java MODULE
        TARGET_NAME sdurwsim_jni
        INSTALL_DIR ${JAVA_INSTALL_DIR}
        SOURCES ThreadSimulatorStepCallbackEnv.cpp ../ScriptTypes.cpp
        CXX_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/java_src_sdurwsim/org/robwork/sdurwsim
        LANGUAGE_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/java_src_sdurwsim/org/robwork/sdurwsim
        SWIG_FLAGS "-package" "org.robwork.sdurwsim"
    )

    swig_link_libraries(sdurwsim_jni ${RWSIM_ODE_LIBRARY} sdurwsim)

    # Force removal of previous Java compilation and source when interface file changes This is
    # required as types may be removed or change name (in this case previous java classes would
    # interfere with current compilation).
    rw_add_java_clean_target(sdurwsim)

    # Compile java code and create JAR and Javadoc
    rw_add_java_lib(
        sdurwsim
        CLASSPATH
            ${RW_LIBS}/sdurw_core_java.jar
            ${RW_LIBS}/sdurw_common_java.jar
            ${RW_LIBS}/sdurw_math_java.jar
            ${RW_LIBS}/sdurw_kinematics_java.jar
            ${RW_LIBS}/sdurw_java.jar
            ${RW_LIBS}/sdurw_task_java.jar
            ${RW_LIBS}/sdurw_assembly_java.jar
            ${RW_LIBS}/sdurw_control_java.jar
            ${RW_LIBS}/sdurw_simulation_java.jar
        JAVADOC_LINK ${RW_LIBS}/javadoc/sdurw ${RW_LIBS}/javadoc/sdurw_core ${RW_LIBS}/javadoc/sdurw_common ${RW_LIBS}/javadoc/sdurw_math ${RW_LIBS}/javadoc/sdurw_kinematics
        WINDOW_TITLE "RobWorkSim Java API Documentation"
        LOADER_SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/LoaderRWSim.java"
        LOADER_DST_FILE "java_src_sdurwsim/org/robwork/LoaderRWSim.java"
        EXTRA_COPY
            COMMAND
                ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepCallbackHandler.java
                java_src_sdurwsim/org/robwork/sdurwsim/ThreadSimulatorStepCallbackHandler.java
            COMMAND
                ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepEvent.java
                java_src_sdurwsim/org/robwork/sdurwsim/ThreadSimulatorStepEvent.java
            COMMAND
                ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepEventListener.java
                java_src_sdurwsim/org/robwork/sdurwsim/ThreadSimulatorStepEventListener.java
            COMMAND
                ${CMAKE_COMMAND} -E copy_if_different
                ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepEventDispatcher.java
                java_src_sdurwsim/org/robwork/sdurwsim/ThreadSimulatorStepEventDispatcher.java
    )
    set(CMAKE_SWIG_OUTDIR ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

endif()

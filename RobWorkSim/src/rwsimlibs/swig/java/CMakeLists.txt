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

    add_library(${SUBSYS_NAME} INTERFACE)

    set(build_doc TRUE)
    if(WIN32 OR CMAKE_SYSTEM_NAME STREQUAL "Darwin")
        set(build_doc FALSE)
    endif()
    rw_subsys_option(build sdurwsim_java_doc "build documentation for the java implementation"
                     ${build_doc} REASON "set by default"
    )

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

    if(CMAKE_COMPILER_IS_GNUCXX)
        set_source_files_properties(ThreadSimulatorStepCallbackEnv.cpp COMPILE_FLAGS -fno-gnu-unique)
        set_source_files_properties(../ScriptTypes.cpp COMPILE_FLAGS -fno-gnu-unique)
    endif()

    target_link_libraries(sdurwsim_jni ${RWSIM_ODE_LIBRARY} sdurwsim)

    # Force removal of previous Java compilation and source when interface file changes This is
    # required as types may be removed or change name (in this case previous java classes would
    # interfere with current compilation).
    rw_add_java_clean_target(sdurwsim)

    # Compile java code and create JAR and Javadoc
    rw_add_java_lib(
        sdurwsim
        CLASSPATH ${RW_LIBS}/sdurw_core_java.jar
                  ${RW_LIBS}/sdurw_common_java.jar
                  ${RW_LIBS}/sdurw_math_java.jar
                  ${RW_LIBS}/sdurw_kinematics_java.jar
                  ${RW_LIBS}/sdurw_geometry_java.jar
                  ${RW_LIBS}/sdurw_java.jar
                  ${RW_LIBS}/sdurw_task_java.jar
                  ${RW_LIBS}/sdurw_assembly_java.jar
                  ${RW_LIBS}/sdurw_control_java.jar
                  ${RW_LIBS}/sdurw_simulation_java.jar
                  ${RW_LIBS}/sdurw_geometry_java.jar
                  ${RW_LIBS}/sdurw_sensor_java.jar
                  ${RW_LIBS}/sdurw_models_java.jar
                  ${RW_LIBS}/sdurw_proximity_java.jar
                  ${RW_LIBS}/sdurw_graphics_java.jar
                  ${RW_LIBS}/sdurw_trajectory_java.jar
        JAVADOC_LINK ${RW_LIBS}/javadoc/sdurw
                     ${RW_LIBS}/javadoc/sdurw_core
                     ${RW_LIBS}/javadoc/sdurw_common
                     ${RW_LIBS}/javadoc/sdurw_math
                     ${RW_LIBS}/javadoc/sdurw_kinematics
                     ${RW_LIBS}/javadoc/sdurw_geometry
                     ${RW_LIBS}/javadoc/sdurw_sensor
                     ${RW_LIBS}/javadoc/sdurw_models
                     ${RW_LIBS}/javadoc/sdurw_proximity
                     ${RW_LIBS}/javadoc/sdurw_graphics
                     ${RW_LIBS}/javadoc/sdurw_trajectory
        WINDOW_TITLE "RobWorkSim Java API Documentation"
                     LOADER_SOURCE_FILE "${CMAKE_CURRENT_SOURCE_DIR}/LoaderRWSim.java"
                     LOADER_DST_FILE "java_src_sdurwsim/org/robwork/LoaderRWSim.java" BUILD_DOC
                     FALSE
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

    if(SWIG_DEFAULT_COMPILE OR CMAKE_VERSION VERSION_GREATER 3.16.0)
        target_link_libraries(${SUBSYS_NAME} INTERFACE sdurws_jni)
    endif()

    install(TARGETS ${SUBSYS_NAME} EXPORT ${PROJECT_PREFIX}javaTargets)
    add_library(${PROJECT_PREFIX}::${SUBSYS_NAME} ALIAS ${SUBSYS_NAME})

    rw_swig_compile_target(
        java sdurwsim_jni ${java_NAME_sdurwsim}_java_doc ${java_NAME_sdurwsim}_libs
    )

    export(
        EXPORT ${PROJECT_PREFIX}_sdurwsim_jniTargets
        FILE "${${PROJECT_PREFIX}_ROOT}/cmake/targets/sdurwsim_jni.cmake"
        NAMESPACE ${PROJECT_PREFIX}::
    )
    install(
        EXPORT ${PROJECT_PREFIX}_sdurwsim_jniTargets
        FILE sdurwsim_jni.cmake
        NAMESPACE ${PROJECT_PREFIX}::
        DESTINATION ${RWSIM_INSTALL_DIR}/cmake/targets
    )
endif()

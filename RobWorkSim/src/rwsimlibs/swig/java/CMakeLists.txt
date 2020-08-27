set(SUBSYS_NAME sdurwsim_java)
set(SUBSYS_DESC "Interface for accessing RobWorkSim from java.")
set(SUBSYS_DEPS RW::sdurw_java RWS::sdurws_java sdurwsim)
set(build TRUE)

if(TARGET RWS::sdurws_jni)
message(STATUS "RWS::sdurws_java is target")
else()
message(STATUS "RWS::sdurws_java is not target")
endif()

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
        SOURCES ../sdurwsim.i ThreadSimulatorStepCallbackEnv.cpp ../ScriptTypes.cpp
        CXX_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/java_src/org/robwork/sdurwsim
        LANGUAGE_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/java_src/org/robwork/sdurwsim
        SWIG_FLAGS "-package" "org.robwork.sdurwsim"
    )

    swig_link_libraries(sdurwsim_jni ${RWSIM_ODE_LIBRARY} sdurwsim)

    # Force removal of previous Java compilation and source when interface file changes This is
    # required as types may be removed or change name (in this case previous java classes would
    # interfere with current compilation).
    rw_add_java_clean_target(sdurwsim)

    # Make sure Robwork jave is compiled first
    foreach( 
        dep
        sdurw_core_jni
        sdurw_jni
        sdurw_task_jni
        sdurw_assembly_jni
        sdurw_control_jni
        sdurw_simulation_jni
    )
        if(TARGET ${dep})
            add_dependencies(sdurwsim_jni ${dep})
        endif()
    endforeach()

    if(WIN32)
        set(CLASSPATH
            -classpath
            "${RW_LIBS}/sdurw_core_java.jar\;${RW_LIBS}/sdurw_java.jar\;${RW_LIBS}/sdurw_task_java.jar\;${RW_LIBS}/sdurw_assembly_java.jar\;${RW_LIBS}/sdurw_control_java.jar\;${RW_LIBS}/sdurw_simulation_java.jar"
        )
    else()
        set(CLASSPATH
            -classpath
            ${RW_LIBS}/sdurw_core_java.jar:${RW_LIBS}/sdurw_java.jar:${RW_LIBS}/sdurw_task_java.jar:${RW_LIBS}/sdurw_assembly_java.jar:${RW_LIBS}/sdurw_control_java.jar:${RW_LIBS}/sdurw_simulation_java.jar
        )
    endif()
    # Compile java code and create JAR and Javadoc
    add_custom_command(
        TARGET sdurwsim_jni
        POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Removing old Java compilation..."
        COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/java_build"
        COMMAND ${CMAKE_COMMAND} -E remove_directory
                "${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc"
        COMMAND ${CMAKE_COMMAND} -E echo "Copying Java source..."
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/LoaderRWSim.java
                java_src/org/robwork/LoaderRWSim.java
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepCallbackHandler.java
            java_src/org/robwork/sdurwsim/ThreadSimulatorStepCallbackHandler.java
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepEvent.java
            java_src/org/robwork/sdurwsim/ThreadSimulatorStepEvent.java
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepEventListener.java
            java_src/org/robwork/sdurwsim/ThreadSimulatorStepEventListener.java
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different
            ${CMAKE_CURRENT_SOURCE_DIR}/ThreadSimulatorStepEventDispatcher.java
            java_src/org/robwork/sdurwsim/ThreadSimulatorStepEventDispatcher.java
        COMMAND ${CMAKE_COMMAND} -E echo "Compiling Java files..."
        COMMAND ${CMAKE_COMMAND} -E make_directory java_build/org/robwork/sdurwsim
        COMMAND ${Java_JAVAC_EXECUTABLE} ${CLASSPATH} -d ${CMAKE_CURRENT_BINARY_DIR}/java_build
                java_src/org/robwork/*.java java_src/org/robwork/sdurwsim/*.java
        COMMAND ${CMAKE_COMMAND} -E echo "Creating jar file..."
        COMMAND ${Java_JAR_EXECUTABLE} cvf ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurwsim_java.jar
                -C java_build .
        COMMAND ${CMAKE_COMMAND} -E echo "Creating jar file of source..."
        COMMAND ${Java_JAR_EXECUTABLE} cvf
                ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurwsim_java-source.jar -C java_src .
        COMMAND ${CMAKE_COMMAND} -E echo "Creating Javadoc..."
        COMMAND ${CMAKE_COMMAND} -E make_directory ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc
        COMMAND
            ${Java_JAVADOC_EXECUTABLE} ${CLASSPATH} -d
            ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc -windowtitle
            "RobWorkSim Java API Documentation" -public -sourcepath java_src org.robwork
            org.robwork.sdurwsim -link ${RW_LIBS}/javadoc/sdurw -link ${RW_LIBS}/javadoc/sdurw_core
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
    set(CMAKE_SWIG_OUTDIR ${RWSIM_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

endif()

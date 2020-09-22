set(SUBSYS_NAME sdurw_java)
set(SUBSYS_DESC "Interface for accessing RobWork from java.")
set(SUBSYS_DEPS
    sdurw
    sdurw_assembly
    sdurw_control
    sdurw_pathoptimization
    sdurw_pathplanners
    sdurw_proximitystrategies
    sdurw_task
    sdurw_simulation
    sdurw_opengl
)

set(build TRUE)

find_package(SWIG 3.0.0 QUIET)
set(DEFAULT FALSE)
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
    else()
        set(REASON "Disabled by default.")
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

    include_directories(${RW_ROOT}/src)
    include_directories(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})
    set(CMAKE_SWIG_OUTDIR ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    set(RW_NAME sdurw)
    foreach(
        RW_MODULE
        ${RW_NAME}
        ${RW_NAME}_task
        ${RW_NAME}_assembly
        ${RW_NAME}_control
        ${RW_NAME}_pathoptimization
        ${RW_NAME}_pathplanners
        ${RW_NAME}_proximitystrategies
        ${RW_NAME}_simulation
        ${RW_NAME}_opengl
    )
        set(TARGET_NAME ${RW_MODULE}_jni)

        rw_add_swig(
            ${RW_MODULE} java MODULE
            TARGET_NAME ${TARGET_NAME}
            INSTALL_DIR ${JAVA_INSTALL_DIR}
            SOURCES ../ScriptTypes.cpp
            CXX_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/java_src_${RW_MODULE}/org/robwork/${RW_MODULE}
            LANGUAGE_FILE_DIR ${CMAKE_CURRENT_BINARY_DIR}/java_src_${RW_MODULE}/org/robwork/${RW_MODULE}
            SWIG_FLAGS "-package" "org.robwork.${RW_MODULE}"
        )

        # Force removal of previous Java compilation and source when interface file changes This is
        # required as types may be removed or change name (in this case previous java classes would
        # interfere with current compilation).
        add_custom_command(
            OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/CleanDep_${RW_MODULE}
            COMMAND ${CMAKE_COMMAND} -E remove_directory java_src_${RW_MODULE}
            COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/CleanDep_${RW_MODULE}
            DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${RW_MODULE_FILENAME}"
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
            COMMENT "Removing old Java source..."
        )
        add_custom_target(CleanDepRW_${RW_MODULE}
                          DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/CleanDep_${RW_MODULE}
        )

        if((CMAKE_GENERATOR MATCHES "Make") AND (NOT CMAKE_VERSION VERSION_LESS 3.12))
            add_dependencies(${RW_MODULE}_jni_swig_compilation CleanDepRW_${RW_MODULE})
        else()
            add_dependencies(${RW_MODULE}_jni CleanDepRW_${RW_MODULE})
        endif()

        if(NOT "${RW_MODULE}" STREQUAL "${RW_NAME}")
            set(CLASSPATH -classpath ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_java.jar)
            set(LINK_JAVADOC -link ../sdurw)
            if("${RW_MODULE}" STREQUAL "${RW_NAME}_assembly")
                if(WIN32)
                    set(CLASSPATH
                        -classpath
                        "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_java.jar\;${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_task_java.jar"
                    )
                else()
                    set(CLASSPATH
                        -classpath
                        ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_java.jar:${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_task_java.jar
                    )
                endif()
                set(LINK_JAVADOC ${LINK_JAVADOC} -link ../sdurw_task)
            elseif("${RW_MODULE}" STREQUAL "sdurw_simulation")
                if(WIN32)
                    set(CLASSPATH
                        -classpath
                        "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_java.jar\;${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_control_java.jar"
                    )
                else()
                    set(CLASSPATH
                        -classpath
                        ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_java.jar:${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_control_java.jar
                    )
                endif()
                set(LINK_JAVADOC ${LINK_JAVADOC} -link ../sdurw_control)
            endif()
            set(LOADERRW_FILE)
            set(LOADERRW_PKG)
            set(LOADERRW_CP_CMD echo " - Nothing to copy")
        else()
            set(LOADERRW_FILE java_src_${RW_MODULE}/org/robwork/LoaderRW.java)
            set(LOADERRW_PKG org.robwork)
            set(LOADERRW_CP_CMD copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/LoaderRW.java
                                java_src_${RW_MODULE}/org/robwork/LoaderRW.java
            )
        endif()
        # Compile java code and create JAR and Javadoc

        add_custom_command(
            TARGET ${RW_MODULE}_jni
            POST_BUILD
            COMMAND ${CMAKE_COMMAND} -E echo "Removing old Java compilation..."
            COMMAND ${CMAKE_COMMAND} -E remove_directory
                    "${CMAKE_CURRENT_BINARY_DIR}/java_build_${RW_MODULE}"
            COMMAND ${CMAKE_COMMAND} -E remove_directory
                    "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc/${RW_MODULE}"
            COMMAND ${CMAKE_COMMAND} -E echo "Copying Java source..."
            COMMAND ${CMAKE_COMMAND} -E ${LOADERRW_CP_CMD}
            COMMAND ${CMAKE_COMMAND} -E echo "Compiling Java files..."
            COMMAND ${CMAKE_COMMAND} -E make_directory
                    java_build_${RW_MODULE}/org/robwork/${RW_MODULE}
            COMMAND
                ${Java_JAVAC_EXECUTABLE} ${CLASSPATH} -d
                ${CMAKE_CURRENT_BINARY_DIR}/java_build_${RW_MODULE} ${LOADERRW_FILE}
                java_src_${RW_MODULE}/org/robwork/${RW_MODULE}/*.java
            COMMAND ${CMAKE_COMMAND} -E echo "Creating jar file..."
            COMMAND
                ${Java_JAR_EXECUTABLE} cvf
                ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE}_java.jar -C
                java_build_${RW_MODULE} .
            COMMAND ${CMAKE_COMMAND} -E echo "Creating jar file of source..."
            COMMAND
                ${Java_JAR_EXECUTABLE} cvf
                ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE}_java-source.jar -C
                java_src_${RW_MODULE} .
            COMMAND ${CMAKE_COMMAND} -E echo "Creating Javadoc..."
            COMMAND ${CMAKE_COMMAND} -E make_directory
                    ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc/${RW_MODULE}
            COMMAND
                ${Java_JAVADOC_EXECUTABLE} ${CLASSPATH} -d
                ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc/${RW_MODULE} -windowtitle
                "RobWork Java API Documentation" -public -sourcepath java_src_${RW_MODULE}
                ${LOADERRW_PKG} org.robwork.${RW_MODULE} ${LINK_JAVADOC}
            WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        )

        target_link_libraries(${SUBSYS_NAME} INTERFACE ${RW_MODULE}_jni)
    endforeach()

    swig_link_libraries(${RW_NAME}_jni sdurw)
    swig_link_libraries(${RW_NAME}_assembly_jni sdurw_assembly sdurw)
    swig_link_libraries(${RW_NAME}_control_jni sdurw_control sdurw)
    swig_link_libraries(${RW_NAME}_pathoptimization_jni sdurw_pathoptimization sdurw)
    swig_link_libraries(${RW_NAME}_pathplanners_jni sdurw_pathplanners sdurw)
    swig_link_libraries(${RW_NAME}_proximitystrategies_jni sdurw_proximitystrategies sdurw)
    swig_link_libraries(${RW_NAME}_task_jni sdurw_task sdurw)
    swig_link_libraries(${RW_NAME}_simulation_jni sdurw_simulation sdurw)
    swig_link_libraries(${RW_NAME}_opengl_jni sdurw_opengl sdurw)

    add_dependencies(${RW_NAME}_assembly_jni ${RW_NAME}_task_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_control_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_pathoptimization_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_pathplanners_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_proximitystrategies_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_simulation_jni ${RW_NAME}_control_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_task_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_opengl_jni ${RW_NAME}_jni)

    install(TARGETS ${SUBSYS_NAME} EXPORT ${PROJECT_PREFIX}Targets)
    add_library(${PROJECT_PREFIX}::${SUBSYS_NAME} ALIAS ${SUBSYS_NAME})
endif()

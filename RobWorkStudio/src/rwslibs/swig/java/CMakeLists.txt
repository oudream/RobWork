set(SUBSYS_NAME sdurws_java)
set(SUBSYS_DESC "Interface for accessing RobWorkStudio from java.")
set(SUBSYS_DEPS sdurws sdurws_robworkstudioapp RW::sdurw_java)

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

    set_source_files_properties(../sdurws.i PROPERTIES CPLUSPLUS ON)
    set_source_files_properties(../sdurws.i PROPERTIES SWIG_FLAGS "-includeall")

    if(NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
        set_source_files_properties(../sdurws.i PROPERTIES SWIG_FLAGS "-includeall;-doxygen")
    endif()

    include_directories(${RW_ROOT}/src)
    include_directories(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})

    set(CMAKE_SWIG_FLAGS "-package" "org.robwork.sdurws")
    # Put java files in different directory suitable for JAR generation later on
    set(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/java_src/org/robwork/sdurws)
    # SWIG
    if((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
        swig_add_library(
            sdurws_jni
            TYPE SHARED
            LANGUAGE java
            SOURCES ../sdurws.i ../ScriptTypes.cpp
        )
    else()
        swig_add_module(sdurws_jni java ../sdurws.i ../ScriptTypes.cpp)
    endif()
    swig_link_libraries(sdurws_jni ${SUBSYS_DEPS} sdurws RW::sdurw)
    if((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
        set_target_properties(sdurws_jni PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    endif()
    # Force removal of previous Java compilation and source when interface file changes This is
    # required as types may be removed or change name (in this case previous java classes would
    # interfere with current compilation).
    add_custom_command(
        OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/CleanDep
        COMMAND ${CMAKE_COMMAND} -E remove_directory java_src
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/CleanDep
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../sdurws.i"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Removing old Java source..."
    )
    add_custom_target(CleanDepRWS DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/CleanDep)
    if((CMAKE_GENERATOR MATCHES "Make") AND (NOT CMAKE_VERSION VERSION_LESS 3.12))
        add_dependencies(sdurws_jni_swig_compilation CleanDepRWS)
    else()
        add_dependencies(sdurws_jni CleanDepRWS)
    endif()

    # Compile java code and create JAR and Javadoc
    add_custom_command(
        TARGET sdurws_jni POST_BUILD
        COMMAND ${CMAKE_COMMAND} -E echo "Removing old Java compilation..."
        COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/java_build"
        COMMAND
            ${CMAKE_COMMAND} -E remove_directory "${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc"
        COMMAND ${CMAKE_COMMAND} -E echo "Copying Java source..."
        COMMAND
            ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/LoaderRWS.java
            java_src/org/robwork/LoaderRWS.java
        COMMAND ${CMAKE_COMMAND} -E echo "Compiling Java files..."
        COMMAND ${CMAKE_COMMAND} -E make_directory java_build/org/robwork
        COMMAND
            ${Java_JAVAC_EXECUTABLE}
            -cp
            ${RW_LIBS}/sdurw_java.jar
            -d
            ${CMAKE_CURRENT_BINARY_DIR}/java_build
            java_src/org/robwork/*.java
            java_src/org/robwork/sdurws/*.java
        COMMAND ${CMAKE_COMMAND} -E echo "Creating jar file..."
        COMMAND
            ${Java_JAR_EXECUTABLE}
            cvf
            ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurws_java.jar
            -C
            java_build
            .
        COMMAND ${CMAKE_COMMAND} -E echo "Creating jar file of source..."
        COMMAND
            ${Java_JAR_EXECUTABLE}
            cvf
            ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurws_java-source.jar
            -C
            java_src
            .
        COMMAND ${CMAKE_COMMAND} -E echo "Creating Javadoc..."
        COMMAND ${CMAKE_COMMAND} -E make_directory ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc
        COMMAND
            ${Java_JAVADOC_EXECUTABLE}
            -classpath
            ${RW_LIBS}/sdurw_java.jar
            -d
            ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc
            -windowtitle
            "RobWorkStudio Java API Documentation"
            -public
            -sourcepath
            java_src
            org.robwork
            org.robwork.sdurws
            -link
            ${RW_LIBS}/javadoc/sdurw
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
    set(CMAKE_SWIG_OUTDIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    target_link_libraries(${SUBSYS_NAME} INTERFACE sdurws_jni)
    install(
        TARGETS sdurws_jni 
        EXPORT ${PROJECT_PREFIX}Targets 
        DESTINATION ${JAVA_INSTALL_DIR} COMPONENT swig)

    install(TARGETS ${SUBSYS_NAME} EXPORT ${PROJECT_PREFIX}Targets)
    add_library(${PROJECT_PREFIX}::${SUBSYS_NAME} ALIAS ${SUBSYS_NAME})
endif()

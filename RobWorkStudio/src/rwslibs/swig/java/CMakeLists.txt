SET(SUBSYS_NAME sdurws_java )
SET(SUBSYS_DESC "Interface for accessing RobWorkStudio from java." )
SET(SUBSYS_DEPS sdurw )

SET(build TRUE)

FIND_PACKAGE(SWIG 3.0.0 QUIET)
set(DEFAULT TRUE)
set(REASON )
IF( NOT SWIG_FOUND)
    set(DEFAULT false)
    set(REASON "SWIG not found!")
else()

    FIND_PACKAGE(Java)	
    FIND_PACKAGE(JNI)
    if( NOT (JAVA_FOUND AND JNI_FOUND) )
        set(DEFAULT false)
        set(REASON "JAVA or JNI not found!")
    endif()
endif()
 
RW_SUBSYS_OPTION( build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
RW_ADD_DOC( ${SUBSYS_NAME} )

IF( build )
    INCLUDE(UseJava)
    INCLUDE(UseSWIG)
    
    SET_SOURCE_FILES_PROPERTIES(../sdurws.i PROPERTIES CPLUSPLUS ON)
	SET_SOURCE_FILES_PROPERTIES(../sdurws.i PROPERTIES SWIG_FLAGS "-includeall")
    if (NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
    	SET_SOURCE_FILES_PROPERTIES(../sdurws.i PROPERTIES SWIG_FLAGS "-includeall;-doxygen")
    endif()
    
    INCLUDE_DIRECTORIES(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})
    set(CMAKE_SWIG_FLAGS "-package" "org.robwork.sdurws")
    # Put java files in different directory suitable for JAR generation later on
    SET(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/java_src/org/robwork/sdurws)
    # SWIG
	IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
		SWIG_ADD_LIBRARY(sdurws_jni TYPE SHARED LANGUAGE java SOURCES ../sdurws.i ../ScriptTypes.cpp)
	ELSE()
		SWIG_ADD_MODULE(sdurws_jni java ../sdurws.i ../ScriptTypes.cpp)
	ENDIF()
    SWIG_LINK_LIBRARIES(sdurws_jni 
        ${RWS_COMPONENT_LIBRARIES}
        ${RWS_PLUGIN_LIBRARIES}
        sdurws
        ${ROBWORK_LIBRARIES} )
    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
      SET_TARGET_PROPERTIES(sdurws_jni PROPERTIES LINK_FLAGS -Wl,--no-undefined)
    ENDIF()
    # Force removal of previous Java compilation and source when interface file changes
    # This is required as types may be removed or change name
    # (in this case previous java classes would interfere with current compilation).
	ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/CleanDep
        COMMAND ${CMAKE_COMMAND} -E remove_directory java_src
        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/CleanDep
        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/../sdurws.i"
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
        COMMENT "Removing old Java source..."
    )
    ADD_CUSTOM_TARGET(CleanDepRWS DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/CleanDep )
    IF((CMAKE_GENERATOR MATCHES "Make") AND (NOT CMAKE_VERSION VERSION_LESS 3.12))
        ADD_DEPENDENCIES(sdurws_jni_swig_compilation CleanDepRWS)
    ELSE()
        ADD_DEPENDENCIES(sdurws_jni CleanDepRWS)
    ENDIF()

    # Make sure dependencies are build before system
    FOREACH(DEP IN LISTS SUBSYS_DEPS)
        IF(TARGET ${DEP})
            ADD_DEPENDENCIES(sdurws_jni ${DEP})
        ENDIF()
    ENDFOREACH()

    # Compile java code and create JAR and Javadoc
	ADD_CUSTOM_COMMAND(TARGET sdurws_jni
		POST_BUILD
		COMMAND ${CMAKE_COMMAND} -E echo "Removing old Java compilation..."
        COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/java_build"
    	COMMAND ${CMAKE_COMMAND} -E remove_directory "${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc"
		COMMAND ${CMAKE_COMMAND} -E echo "Copying Java source..."
        COMMAND ${CMAKE_COMMAND} -E copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/LoaderRWS.java java_src/org/robwork/LoaderRWS.java
		COMMAND ${CMAKE_COMMAND} -E echo "Compiling Java files..."
        COMMAND ${CMAKE_COMMAND} -E make_directory java_build/org/robwork
        COMMAND ${Java_JAVAC_EXECUTABLE} -cp ${RW_LIBS}/sdurw_java.jar -d ${CMAKE_CURRENT_BINARY_DIR}/java_build java_src/org/robwork/*.java java_src/org/robwork/sdurws/*.java
        COMMAND ${CMAKE_COMMAND} -E echo "Creating jar file..."
        COMMAND ${Java_JAR_EXECUTABLE} cvf ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurws_java.jar  -C java_build .
        COMMAND ${CMAKE_COMMAND} -E echo "Creating jar file of source..."
        COMMAND ${Java_JAR_EXECUTABLE} cvf ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurws_java-source.jar  -C java_src .
		COMMAND ${CMAKE_COMMAND} -E echo "Creating Javadoc..."
    	COMMAND ${CMAKE_COMMAND} -E make_directory ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc
    	COMMAND ${Java_JAVADOC_EXECUTABLE}
    					-classpath ${RW_LIBS}/sdurw_java.jar
    					-d ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc
    					-windowtitle "RobWorkStudio Java API Documentation"
    					-public
    					-sourcepath java_src org.robwork org.robwork.sdurws
    					-link ${RW_LIBS}/javadoc/sdurw
        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
    )
    SET(CMAKE_SWIG_OUTDIR ${RWS_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

ENDIF()

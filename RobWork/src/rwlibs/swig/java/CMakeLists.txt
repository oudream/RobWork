if(POLICY CMP0046)
  cmake_policy(SET CMP0046 NEW)
endif()

SET(SUBSYS_NAME sdurw_java )
SET(SUBSYS_DESC "Interface for accessing RobWork from java." )
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

    INCLUDE_DIRECTORIES(${JAVA_INCLUDE_DIRS} ${JNI_INCLUDE_DIRS})
    SET(CMAKE_SWIG_OUTDIR ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY})

    add_custom_target(${SUBSYS_NAME})
	SET(RW_NAME sdurw)
	foreach(RW_MODULE ${RW_NAME} ${RW_NAME}_assembly ${RW_NAME}_control ${RW_NAME}_pathoptimization ${RW_NAME}_pathplanners ${RW_NAME}_proximitystrategies ${RW_NAME}_simulation ${RW_NAME}_task)
		SET(RW_MODULE_FILENAME ../${RW_MODULE}.i)		
	    SET_SOURCE_FILES_PROPERTIES(${RW_MODULE_FILENAME} PROPERTIES CPLUSPLUS ON)
		SET_SOURCE_FILES_PROPERTIES(${RW_MODULE_FILENAME} PROPERTIES SWIG_FLAGS "-includeall")

	    if (NOT ${SWIG_VERSION} VERSION_LESS 4.0.0)
	    	SET_SOURCE_FILES_PROPERTIES(${RW_MODULE_FILENAME} PROPERTIES SWIG_FLAGS "-includeall;-doxygen")
	    endif()
	
	    set(CMAKE_SWIG_FLAGS "-package" "org.robwork.${RW_MODULE}")
	    # Put java files in different directory suitable for JAR generation later on
	    SET(CMAKE_SWIG_OUTDIR ${CMAKE_CURRENT_BINARY_DIR}/java_src_${RW_MODULE}/org/robwork/${RW_MODULE})
	    # SWIG
		IF ((CMAKE_VERSION VERSION_GREATER 3.8) OR (CMAKE_VERSION VERSION_EQUAL 3.8))
			SWIG_ADD_LIBRARY(${RW_MODULE}_jni TYPE MODULE LANGUAGE java SOURCES ${RW_MODULE_FILENAME} ../ScriptTypes.cpp)
		ELSE()
			SWIG_ADD_MODULE(${RW_MODULE}_jni java ${RW_MODULE_FILENAME} ../ScriptTypes.cpp)
		ENDIF()
        add_dependencies(${SUBSYS_NAME} ${RW_MODULE}_jni)
	    IF((CMAKE_COMPILER_IS_GNUCC) OR (CMAKE_C_COMPILER_ID STREQUAL "Clang"))
	      SET_TARGET_PROPERTIES(${RW_MODULE}_jni PROPERTIES LINK_FLAGS -Wl,--no-undefined)
	    ENDIF()
	    # Force removal of previous Java compilation and source when interface file changes
	    # This is required as types may be removed or change name
	    # (in this case previous java classes would interfere with current compilation).
		ADD_CUSTOM_COMMAND(OUTPUT ${CMAKE_CURRENT_BINARY_DIR}/CleanDep_${RW_MODULE}
	        COMMAND ${CMAKE_COMMAND} -E remove_directory java_src_${RW_MODULE}
	        COMMAND ${CMAKE_COMMAND} -E touch ${CMAKE_CURRENT_BINARY_DIR}/CleanDep_${RW_MODULE}
	        DEPENDS "${CMAKE_CURRENT_SOURCE_DIR}/${RW_MODULE_FILENAME}"
	        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	        COMMENT "Removing old Java source..."
	    )
	    ADD_CUSTOM_TARGET(CleanDepRW_${RW_MODULE} DEPENDS ${CMAKE_CURRENT_BINARY_DIR}/CleanDep_${RW_MODULE} )
	    IF((CMAKE_GENERATOR MATCHES "Make") AND (NOT CMAKE_VERSION VERSION_LESS 3.12))
	        ADD_DEPENDENCIES(${RW_MODULE}_jni_swig_compilation CleanDepRW_${RW_MODULE})
	    ELSE()
	        ADD_DEPENDENCIES(${RW_MODULE}_jni CleanDepRW_${RW_MODULE})
	    ENDIF()
		if (NOT "${RW_MODULE}" STREQUAL "${RW_NAME}")
	        set(CLASSPATH -classpath ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_java.jar)
	        set(LINK_JAVADOC -link ../sdurw)
	        if ("${RW_MODULE}" STREQUAL "${RW_NAME}_assembly")
	            if (WIN32)
	                set(CLASSPATH -classpath "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_java.jar\;${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_task_java.jar")
	            else()
	                set(CLASSPATH -classpath ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_java.jar:${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_task_java.jar)
	            endif()
	            set(LINK_JAVADOC ${LINK_JAVADOC} -link ../sdurw_task)
	        elseif ("${RW_MODULE}" STREQUAL "sdurw_simulation")
	            if (WIN32)
	                set(CLASSPATH -classpath "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_java.jar\;${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_control_java.jar")
	            else()
	                set(CLASSPATH -classpath ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_java.jar:${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/sdurw_control_java.jar)
	            endif()
	            set(LINK_JAVADOC ${LINK_JAVADOC} -link ../sdurw_control)
	        endif()
	        set(LOADERRW_FILE)
	        set(LOADERRW_PKG)
	        set(LOADERRW_CP_CMD echo " - Nothing to copy")
		else()
	    	set(LOADERRW_FILE java_src_${RW_MODULE}/org/robwork/LoaderRW.java)
	        set(LOADERRW_PKG org.robwork)
			set(LOADERRW_CP_CMD copy_if_different ${CMAKE_CURRENT_SOURCE_DIR}/LoaderRW.java java_src_${RW_MODULE}/org/robwork/LoaderRW.java)
	    endif()
	    # Compile java code and create JAR and Javadoc
		ADD_CUSTOM_COMMAND(TARGET ${RW_MODULE}_jni
			POST_BUILD
			COMMAND ${CMAKE_COMMAND} -E echo "Removing old Java compilation..."
	        COMMAND ${CMAKE_COMMAND} -E remove_directory "${CMAKE_CURRENT_BINARY_DIR}/java_build_${RW_MODULE}"
	    	COMMAND ${CMAKE_COMMAND} -E remove_directory "${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc/${RW_MODULE}"
			COMMAND ${CMAKE_COMMAND} -E echo "Copying Java source..."
	        COMMAND ${CMAKE_COMMAND} -E ${LOADERRW_CP_CMD}
			COMMAND ${CMAKE_COMMAND} -E echo "Compiling Java files..."
	        COMMAND ${CMAKE_COMMAND} -E make_directory java_build_${RW_MODULE}/org/robwork/${RW_MODULE}
	        COMMAND ${Java_JAVAC_EXECUTABLE} ${CLASSPATH} -d ${CMAKE_CURRENT_BINARY_DIR}/java_build_${RW_MODULE} ${LOADERRW_FILE} java_src_${RW_MODULE}/org/robwork/${RW_MODULE}/*.java
	        COMMAND ${CMAKE_COMMAND} -E echo "Creating jar file..."
	        COMMAND ${Java_JAR_EXECUTABLE} cvf ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE}_java.jar  -C java_build_${RW_MODULE} .
			COMMAND ${CMAKE_COMMAND} -E echo "Creating jar file of source..."
	        COMMAND ${Java_JAR_EXECUTABLE} cvf ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/${RW_MODULE}_java-source.jar  -C java_src_${RW_MODULE} .
			COMMAND ${CMAKE_COMMAND} -E echo "Creating Javadoc..."
			COMMAND ${CMAKE_COMMAND} -E make_directory ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc/${RW_MODULE}
	    	COMMAND ${Java_JAVADOC_EXECUTABLE}
    					${CLASSPATH}
    					-d ${RW_CMAKE_LIBRARY_OUTPUT_DIRECTORY}/javadoc/${RW_MODULE}
    					-windowtitle "RobWork Java API Documentation"
    					-public
    					-sourcepath java_src_${RW_MODULE} ${LOADERRW_PKG} org.robwork.${RW_MODULE}
    					${LINK_JAVADOC}
	        WORKING_DIRECTORY ${CMAKE_CURRENT_BINARY_DIR}
	    )
    endforeach()

    SWIG_LINK_LIBRARIES(${RW_NAME}_jni sdurw)
    SWIG_LINK_LIBRARIES(${RW_NAME}_assembly_jni sdurw_assembly sdurw)
    SWIG_LINK_LIBRARIES(${RW_NAME}_control_jni sdurw_control sdurw)
    SWIG_LINK_LIBRARIES(${RW_NAME}_pathoptimization_jni sdurw_pathoptimization sdurw)
    SWIG_LINK_LIBRARIES(${RW_NAME}_pathplanners_jni sdurw_pathplanners sdurw)
    SWIG_LINK_LIBRARIES(${RW_NAME}_proximitystrategies_jni sdurw_proximitystrategies sdurw)
	SWIG_LINK_LIBRARIES(${RW_NAME}_task_jni sdurw_task sdurw)
	SWIG_LINK_LIBRARIES(${RW_NAME}_simulation_jni sdurw_simulation sdurw)

    add_dependencies(${RW_NAME}_assembly_jni ${RW_NAME}_task_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_control_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_pathoptimization_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_pathplanners_jni ${RW_NAME}_jni)
	add_dependencies(${RW_NAME}_proximitystrategies_jni ${RW_NAME}_jni)
	add_dependencies(${RW_NAME}_simulation_jni ${RW_NAME}_control_jni ${RW_NAME}_jni)
    add_dependencies(${RW_NAME}_task_jni ${RW_NAME}_jni)
ENDIF()

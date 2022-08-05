# - FindMathGL.cmake
# This module can be used to find MathGL v.2.* and MathGL v.8.* and several of its optional components.
#
# You can specify one or more component as you call this find module.
# Possible components are: FLTK, GLUT, Qt, WX.
#
# The following variables will be defined for your use:
#
#  MATHGL_FOUND           = MathGL v.2 or v.8 and all specified components found
#  MATHGL_INCLUDE_DIRS    = The MathGL v.2 or v.8 include directories
#  MATHGL_LIBRARIES       = The libraries to link against to use MathGL v.2 or v.8
#                           and all specified components
#  MATHGL_VERSION_STRING  = A human-readable version of the MathGL v.2 or v.8 (e.g. 2.1 or 8.1)
#  MATHGL_XXX_FOUND       = Component XXX found (replace XXX with uppercased
#                           component name -- for example, QT or FLTK)
#
# The minimum required version and needed components can be specified using
# the standard find_package()-syntax, here are some examples:
#  find_package(MathGL REQUIRED)				- v.2.* or v.8.* (no interfaces), required
#  find_package(MathGL REQUIRED Qt)		- v.2.1 + or v.8.1 + Qt interface, required
#  find_package(MathGL 2.1 REQUIRED)			- v.2.1 (no interfaces), required
#  find_package(MathGL 8.1 REQUIRED)			- v.8.1 (no interfaces), required
#  find_package(MathGL COMPONENTS Qt WX)	- v.2.0 + or v.8.0 + Qt and WX interfaces, optional
#
# Note, some cmake builds require to write "COMPONENTS" always, like
#  find_package(MathGL REQUIRED COMPONENTS Qt)	- v.2.* + or v.8.* Qt interface, required
#
# Typical usage could be something like this:
#   find_package(MathGL REQUIRED FLTK)
#   include_directories(${MATHGL_INCLUDE_DIRS})
#   add_executable(myexe main.cpp)
#   target_link_libraries(myexe ${MATHGL_LIBRARIES} ${MATHGL_FLTK_LIBRARIES})
#

#=============================================================================
# Copyright (c) 2011 Denis Pesotsky <denis@kde.ru>, 2014 Alexey Balakin <mathgl.abalakin@gmail.com>
#
# Distributed under the OSI-approved BSD License (the "License");
# see accompanying file COPYING-CMAKE-MODULES for details.
#
# This software is distributed WITHOUT ANY WARRANTY; without even the
# implied warranty of MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.
# See the License for more information.
#=============================================================================

FIND_PATH(MATHGL_INCLUDE_DIR
		NAMES mgl2/mgl.h
		DOC "The MathGL2 v.2.* or v.8.* include directory")
FIND_LIBRARY(MATHGL_LIBRARY
		NAMES mgl
		PATHS ${MATHGL_LIBRARY_DIR}
		DOC "The MathGL v.2.* or v.8.* include directory")

GET_FILENAME_COMPONENT(MATHGL_LIBRARY_DIR ${MATHGL_LIBRARY} PATH)

SET(MATHGL_LIBRARIES ${MATHGL_LIBRARY})
SET(MATHGL_INCLUDE_DIRS ${MATHGL_INCLUDE_DIR})

SET(_VERSION_ERR "Cannot determine MathGL v.2.* or v.8.* version")
IF(MATHGL_INCLUDE_DIR)
    SET(_CONFIG_FILE_PATH "${MATHGL_INCLUDE_DIR}/mgl2/config.h")
	IF(EXISTS "${_CONFIG_FILE_PATH}")
		FILE(STRINGS "${_CONFIG_FILE_PATH}"
			MATHGL_VERSION_MAJOR_STRING REGEX "^#define MGL_VER_MAJOR.*$")
		FILE(STRINGS "${_CONFIG_FILE_PATH}"
			MATHGL_VERSION_MINOR_STRING REGEX "^#define MGL_VER_MINOR.*$")
		FILE(STRINGS "${_CONFIG_FILE_PATH}"
			MATHGL_VERSION_PATCH_STRING REGEX "^#define MGL_VER_PATCH.*$")
		IF(MATHGL_VERSION_MAJOR_STRING AND MATHGL_VERSION_MINOR_STRING)
			STRING(REGEX
				REPLACE "#define MGL_VER_MAJOR" ""
				MATHGL_VERSION_MAJOR_STRING ${MATHGL_VERSION_MAJOR_STRING})
			STRING(REGEX
				REPLACE "#define MGL_VER_MINOR" ""
				MATHGL_VERSION_MINOR_STRING ${MATHGL_VERSION_MINOR_STRING})
			STRING(REGEX
				REPLACE "#define MGL_VER_PATCH" ""
				MATHGL_VERSION_PATCH_STRING ${MATHGL_VERSION_PATCH_STRING})
			STRING(REGEX
				REPLACE "//.*" ""
				MATHGL_VERSION_MAJOR_STRING ${MATHGL_VERSION_MAJOR_STRING})
			STRING(REGEX
				REPLACE "//.*" ""
				MATHGL_VERSION_MINOR_STRING ${MATHGL_VERSION_MINOR_STRING})
			STRING(REGEX
				REPLACE "//.*" ""
				MATHGL_VERSION_PATCH_STRING ${MATHGL_VERSION_PATCH_STRING})
			STRING(STRIP ${MATHGL_VERSION_MAJOR_STRING} MATHGL_VERSION_MAJOR_STRING)
			STRING(STRIP ${MATHGL_VERSION_MINOR_STRING} MATHGL_VERSION_MINOR_STRING)
			STRING(STRIP ${MATHGL_VERSION_PATCH_STRING} MATHGL_VERSION_PATCH_STRING)
			SET(MATHGL_VERSION_STRING ${MATHGL_VERSION_MAJOR_STRING}.${MATHGL_VERSION_MINOR_STRING}.${MATHGL_VERSION_PATCH_STRING})
#			MESSAGE(STATUS "Find MathGL version -- ${MATHGL_VERSION_STRING}")
		ELSE()
			SET(_ERR_MESSAGE "${_VERSION_ERR}: ${_CONFIG_FILE_PATH} parse error")
		ENDIF()
	ELSE()
		SET(_ERR_MESSAGE "${_VERSION_ERR}: ${_CONFIG_FILE_PATH} not found")
	ENDIF()
	IF(_ERR_MESSAGE)
		UNSET(_ERR_MESSAGE)
		SET(_CONFIG_FILE_PATH "${MATHGL_INCLUDE_DIR}/mgl2/define.h")
		IF(EXISTS "${_CONFIG_FILE_PATH}")
			FILE(STRINGS "${_CONFIG_FILE_PATH}"
				MATHGL_VERSION_STRING REGEX "^#define MGL_VER2.*$")
			IF(MATHGL_VERSION_STRING)
				STRING(REGEX
					REPLACE "#define MGL_VER2" ""
					MATHGL_VERSION_STRING ${MATHGL_VERSION_STRING})
				STRING(REGEX
					REPLACE "//.*" ""
					MATHGL_VERSION_STRING ${MATHGL_VERSION_STRING})
				STRING(STRIP ${MATHGL_VERSION_STRING} MATHGL_VERSION_STRING)
				SET(MATHGL_VERSION_STRING 2.${MATHGL_VERSION_STRING})
	#			MESSAGE(STATUS "Find MathGL version -- ${MATHGL_VERSION_STRING}")
			ELSE()
				SET(_ERR_MESSAGE "${_VERSION_ERR}: ${_CONFIG_FILE_PATH} parse error")
			ENDIF()
		ELSE()
			SET(_ERR_MESSAGE "${_VERSION_ERR}: ${_CONFIG_FILE_PATH} not found")
		ENDIF()
	ENDIF()
	IF(_ERR_MESSAGE)
		UNSET(_ERR_MESSAGE)
		SET(_CONFIG_FILE_PATH "${MATHGL_INCLUDE_DIR}/mgl2/config.h")
		IF(EXISTS "${_CONFIG_FILE_PATH}")
			FILE(STRINGS "${_CONFIG_FILE_PATH}"
				MATHGL_VERSION_STRING REGEX "^#define MGL_VER2.*$")
			IF(MATHGL_VERSION_STRING)
				STRING(REGEX
					REPLACE "#define MGL_VER2" ""
					MATHGL_VERSION_STRING ${MATHGL_VERSION_STRING})
				STRING(REGEX
					REPLACE "//.*" ""
					MATHGL_VERSION_STRING ${MATHGL_VERSION_STRING})
				STRING(STRIP ${MATHGL_VERSION_STRING} MATHGL_VERSION_STRING)
				SET(MATHGL_VERSION_STRING 2.${MATHGL_VERSION_STRING})
	#			MESSAGE(STATUS "Find MathGL version -- ${MATHGL_VERSION_STRING}")
			ELSE()
				SET(_ERR_MESSAGE "${_VERSION_ERR}: ${_CONFIG_FILE_PATH} parse error")
			ENDIF()
		ELSE()
			SET(_ERR_MESSAGE "${_VERSION_ERR}: ${_CONFIG_FILE_PATH} not found")
		ENDIF()
	ENDIF(_ERR_MESSAGE)

	if(_ERR_MESSAGE)
		MESSAGE(FATAL_ERROR ${_ERR_MESSAGE})
	endif(_ERR_MESSAGE)
ENDIF()

INCLUDE(FindPackageHandleStandardArgs)
FIND_PACKAGE_HANDLE_STANDARD_ARGS(MathGL
		REQUIRED_VARS MATHGL_LIBRARY MATHGL_INCLUDE_DIR
		VERSION_VAR MATHGL_VERSION_STRING)

FOREACH(_Component ${MathGL_FIND_COMPONENTS})
	STRING(TOLOWER ${_Component} _component)
	STRING(TOUPPER ${_Component} _COMPONENT)

	SET(MATHGL_${_Component}_FIND_REQUIRED ${MATHGL_FIND_REQUIRED})
	SET(MATHGL_${_Component}_FIND_QUIETLY true)
	if(${_component} STREQUAL "qt4" OR ${_component} STREQUAL "qt5")
		FIND_PATH(MATHGL_${_COMPONENT}_INCLUDE_DIR
					NAMES mgl2/qt.h
					PATHS ${MATHGL_INCLUDE_DIR} NO_DEFAULT_PATH)
	else(${_component} STREQUAL "qt4" OR ${_component} STREQUAL "qt5")
		FIND_PATH(MATHGL_${_COMPONENT}_INCLUDE_DIR
				NAMES mgl2/${_component}.h
				PATHS ${MATHGL_INCLUDE_DIR} NO_DEFAULT_PATH)
	endif(${_component} STREQUAL "qt4" OR ${_component} STREQUAL "qt5")
	FIND_LIBRARY(MATHGL_${_COMPONENT}_LIBRARY
				NAMES mgl-${_component}
				PATHS ${MATHGL_LIBRARY_DIR} NO_DEFAULT_PATH)

	FIND_PACKAGE_HANDLE_STANDARD_ARGS(MATHGL_${_Component} DEFAULT_MSG
										MATHGL_${_COMPONENT}_LIBRARY
										MATHGL_${_COMPONENT}_INCLUDE_DIR)

	IF(MATHGL_${_COMPONENT}_FOUND)
		SET(MATHGL_LIBRARIES
			${MATHGL_LIBRARIES} ${MATHGL_${_COMPONENT}_LIBRARY})
		SET(MATHGL_INCLUDE_DIRS
			${MATHGL_INCLUDE_DIRS} ${MATHGL_${_COMPONENT}_INCLUDE_DIR})
	ENDIF()

	MARK_AS_ADVANCED(MATHGL_${_COMPONENT}_INCLUDE_DIR MATHGL_${_COMPONENT}_LIBRARY)
ENDFOREACH()

MARK_AS_ADVANCED(MATHGL_INCLUDE_DIR MATHGL_LIBRARY MATHGL_VERSION_STRING)

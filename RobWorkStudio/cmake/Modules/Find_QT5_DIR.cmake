

set(QT_MISSING True)
# msvc only; mingw will need different logic
if(MSVC)
    # look for user-registry pointing to qtcreator
    GET_FILENAME_COMPONENT(QT_BIN [HKEY_CURRENT_USER\\Software\\Classes\\Applications\\QtProject.QtCreator.cpp\\shell\\Open\\Command] PATH)
    if(EXISTS "${QT_BIN}" AND NOT  "${QT_BIN}" STREQUAL "/")
        message(STATUS "QTBIN: ${QT_BIN}")
        # get root path so we can search for 5.3, 5.4, 5.5, etc
        STRING(REPLACE "/Tools" ";" QT_BIN "${QT_BIN}")
        LIST(GET QT_BIN 0 QT_BIN)
        FILE(GLOB QT_VERSIONS "${QT_BIN}/5.*")
        LIST(SORT QT_VERSIONS)

        # assume the latest version will be last alphabetically
        LIST(REVERSE QT_VERSIONS)

        LIST(GET QT_VERSIONS 0 QT_VERSION)

        # fix any double slashes which seem to be common
        STRING(REPLACE "//" "/"  QT_VERSION "${QT_VERSION}")

        # do some math trickery to guess folder
        # - qt uses (e.g.) "msvc2012"
        # - cmake uses (e.g.) "1800"
        # - see also https://cmake.org/cmake/help/v3.0/variable/MSVC_VERSION.html
        #MATH(EXPR QT_MSVC "2000 + (${MSVC_VERSION} - 600) / 100")
        #message(STATUS "MSVC ${MSVC_VERSION}")
        set(QT_MSVC 2017)
        # check for 64-bit os
        # may need to be removed for older compilers as it wasn't always offered
        IF(CMAKE_SYSTEM_PROCESSOR MATCHES 64)
            SET(QT_MSVC "${QT_MSVC}_64")
        ENDIF()
        SET(QT_PATH "${QT_VERSION}/msvc${QT_MSVC}")
    endif()
elseif(${CMAKE_SYSTEM_NAME} MATCHES "Darwin")
    set(QT_PATH "/usr/local/opt/qt5")
endif()

if(EXISTS "${QT_PATH}")
    SET(QT_MISSING False)
endif()

macro(FIND_QT_PACKAGE _name)
if(NOT QT_MISSING)
    set(REQ_QUIET)
    set(${_name}_DIR "${QT_PATH}/lib/cmake/${_name}")

    if(${${_name}_FIND_REQUIRED})
        set(REQ_QUIET REQUIRED)
    elseif(${${_name}_FIND_QUIETLY})
        set(REQ_QUIET QUIET)
    endif()
    #message(STATUS "${_name}_FIND_REQUIRED ${${_name}_FIND_REQUIRED}")
    #message(STATUS "${_name}_FIND_QUIETLY ${${_name}_FIND_QUIETLY}")
    #message(STATUS "${_name}_FIND_VERSION ${${_name}_FIND_VERSION}")
    #message(STATUS "${_name}_FIND_VERSION_MAJOR ${${_name}_FIND_VERSION_MAJOR}")
    #message(STATUS "${_name}_FIND_VERSION_MINOR ${${_name}_FIND_VERSION_MINOR}")
    #message(STATUS "${_name}_FIND_VERSION_PATCH ${${_name}_FIND_VERSION_PATCH}")
    #message(STATUS "${_name}_FIND_VERSION_TWEAK ${${_name}_FIND_VERSION_TWEAK}")
    #message(STATUS "${_name}_FIND_VERSION_COUNT ${${_name}_FIND_VERSION_COUNT}")
    #message(STATUS "${_name}_FIND_VERSION_EXACT ${${_name}_FIND_VERSION_EXACT}")
    #message(STATUS "${_name}_FIND_COMPONENTS ${${_name}_FIND_COMPONENTS}")
    #message(STATUS "${_name}_FIND_REQUIRED_ ${${_name}_FIND_REQUIRED_<c>}")

    find_package(${_name} ${REQ_QUIET} PATHS "${${_name}_DIR}")
else()
    set(CMP "${CMAKE_MODULE_PATH}")
    set(CMAKE_MODULE_PATH )
    find_package(${_name} ${REQ_QUIET})
    set(CMAKE_MODULE_PATH "${CMP}")
endif()    
endmacro()

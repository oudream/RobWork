
if(POLICY CMP0086) # Introduce cmake 3.14
    cmake_policy(SET CMP0086 NEW)
endif()


add_subdirectory(lua)
add_subdirectory(python)
add_subdirectory(java)

RW_ADD_INCLUDES(swig "rwlibs/swig" ScriptTypes.hpp)

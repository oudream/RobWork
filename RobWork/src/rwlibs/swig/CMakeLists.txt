
unset(SLIB_COMPILE_BUFFER CACHE) # this is used in macro RW_ADD_SWIG

add_subdirectory(java)
add_subdirectory(lua)
add_subdirectory(python)


RW_ADD_INCLUDES(swig "rwlibs/swig" ScriptTypes.hpp)

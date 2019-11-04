


if(POLICY CMP0086) # Introduce cmake 3.14
    cmake_policy(SET CMP0086 NEW)
endif()

add_subdirectory(lua)
add_subdirectory(java)
add_subdirectory(python)


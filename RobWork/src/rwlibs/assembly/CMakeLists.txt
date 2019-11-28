set(SUBSYS_NAME sdurw_assembly)
set(SUBSYS_DESC "Assembly task and result formats and strategies.")
set(SUBSYS_DEPS sdurw sdurw_task)

set(DEFAULT TRUE)
set(REASON)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ${DEFAULT} ${REASON})
rw_add_doc(${SUBSYS_NAME})

if(build)
    set(
        SRC_FILES
        ./AssemblyControlResponse.cpp
        ./AssemblyControlStrategy.cpp
        ./AssemblyParameterization.cpp
        ./AssemblyRegistry.cpp
        ./AssemblyResult.cpp
        ./AssemblyState.cpp
        ./AssemblyTask.cpp
        ./CircularPiHControlStrategy.cpp
        ./CircularPiHParameterization.cpp
        ./PiHParameterization.cpp
        ./PiHStrategy.cpp
        ./SpiralParameterization.cpp
        ./SpiralStrategy.cpp
    )

    set(
        SRC_FILES_HPP
        ./AssemblyControlResponse.hpp
        ./AssemblyControlStrategy.hpp
        ./AssemblyParameterization.hpp
        ./AssemblyRegistry.hpp
        ./AssemblyResult.hpp
        ./AssemblyState.hpp
        ./AssemblyTask.hpp
        ./CircularPiHControlStrategy.hpp
        ./CircularPiHParameterization.hpp
        ./PiHParameterization.hpp
        ./PiHStrategy.hpp
        ./SpiralParameterization.hpp
        ./SpiralStrategy.hpp
    )

    rw_add_library(${SUBSYS_NAME} assembly ${SRC_FILES} ${SRC_FILES_HPP} ${LOADER_SRC_FILES_HPP})
    rw_add_includes(assembly "rwlibs/assembly" ${SRC_FILES_HPP})
    target_link_libraries(${SUBSYS_NAME} PUBLIC sdurw PRIVATE sdurw_task)
    add_dependencies(${SUBSYS_NAME} ${SUBSYS_DEPS})

    set_target_properties(${SUBSYS_NAME} PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

endif()

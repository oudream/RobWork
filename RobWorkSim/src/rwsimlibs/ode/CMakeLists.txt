set(Ext ${PROJECT_SOURCE_DIR}/ext)

if(RWSIM_HAVE_ODE)

    set(
        SRC_FILES
        ODESimulator.cpp
        ODEVelocityDevice.cpp
        ODEKinematicDevice.cpp
        ODESuctionCupDevice.cpp
        ODEBody.cpp
        ODEJoint.cpp
        ODEUtil.cpp
        ODEDebugRender.cpp
        ODETactileSensor.cpp
        ODEMaterialMap.cpp
        ODESuctionCupDevice.cpp
        ODEConstraint.cpp
        ODEContactStrategy.cpp
        ODEThreading.cpp
    )

    set(
        SRC_FILES_HPP
        ODESimulator.hpp
        ODEVelocityDevice.hpp
        ODEKinematicDevice.hpp
        ODESuctionCupDevice.hpp
        ODEBody.hpp
        ODEJoint.hpp
        ODEUtil.hpp
        ODEDebugRender.hpp
        ODETactileSensor.hpp
        ODEMaterialMap.hpp
        ODEConstraint.hpp
        ODEContactStrategy.hpp
        ODEThreading.hpp
    )

    # Detect if ODE comes with threading_impl.h file (version 0.13 and newer).
    if(NOT DEFINED ODE_WITH_THREADING_IMPL)
        message(STATUS "Testing ODE for the threading_impl.h file...")
        if(EXISTS "${ODE_INCLUDE_DIR}/ode/threading_impl.h")
            message(STATUS "ODE appears to have threading_impl.h file (version 0.13 or newer).")
            set(
                ODE_WITH_THREADING_IMPL
                TRUE
                CACHE BOOL "Select if ODE has threading_impl.h file (version 0.13 and newer)."
            )
        else()
            message(STATUS "ODE does not appear to have threading_impl.h file (version 0.12 or earlier).")
            set(
                ODE_WITH_THREADING_IMPL
                FALSE
                CACHE BOOL "Select if ODE does not have threading_impl.h file (version 0.12 or earlier)."
            )
        endif()
    endif()
    if(ODE_WITH_THREADING_IMPL)
        message(STATUS "ODE threading_impl.h detected - assuming version 0.13 and later.")
        set_source_files_properties(ODEThreading.cpp PROPERTIES COMPILE_DEFINITIONS ODE_WITH_THREADING_IMPL)
    elseif(ODE_WITH_THREADING_IMPL)
        message(STATUS "ODE threading_impl.h NOT detected - assuming version 0.12 and earlier.")
    endif()

    rw_add_library(sdurwsim_ode ${SRC_FILES} ${SRC_FILES_HPP})
    target_link_libraries(sdurwsim_ode PUBLIC sdurwsim ${ODE_LIBRARIES} RW::sdurw ${OPENGL_LIBRARIES})
    target_include_directories(sdurwsim_ode
        INTERFACE
        $<BUILD_INTERFACE:${RWSIM_ROOT}/src>
        $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
        PUBLIC
        $<BUILD_INTERFACE:${ODE_INCLUDE_DIR}> 
    )
    message(STATUS )
    add_library(ode_plugin.rwplugin MODULE ODEPlugin.cpp ODEPlugin.hpp)
    target_link_libraries(ode_plugin.rwplugin PRIVATE sdurwsim_ode)
    install(TARGETS ode_plugin.rwplugin LIBRARY DESTINATION ${RW_PLUGIN_INSTALL_DIR})
    # INSTALL(TARGETS sdurwsim_ode DESTINATION ${RWSIM_LIB_INSTALL_DIR}) INSTALL(FILES ${SRC_FILES_HPP} DESTINATION
    # include/rwlibs/simulation)

    # Make sure dependencies are build before system
    foreach(DEP IN LISTS ROBWORKSIM_LIBRARIES)
        if(TARGET ${DEP})
            add_dependencies(ode_plugin.rwplugin ${DEP})
        endif()
    endforeach()

    if(NOT ODE_WITH_THREADING_IMPL) # version below 0.13
        if(${ODE_BUILD_WITH} STREQUAL "DOUBLE")
            target_compile_definitions(sdurwsim_ode PUBLIC dDOUBLE)
        else()
            target_compile_definitions(sdurwsim_ode PUBLIC dSINGLE)
        endif()
    endif()

endif()

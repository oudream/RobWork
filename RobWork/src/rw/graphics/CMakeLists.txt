set(SUBSYS_NAME sdurw_graphics)
set(SUBSYS_DESC "The graphics RobWork library!")
set(SUBSYS_DEPS
    PUBLIC  sdurw_core
            sdurw_common
            sdurw_math
            sdurw_kinematics
            sdurw_geometry
            sdurw_sensor
    PRIVATE sdurw_models
)

set(build TRUE)
rw_subsys_option(build ${SUBSYS_NAME} ${SUBSYS_DESC} ON DEPENDS ${SUBSYS_DEPS})

if(build)
    set(FILES_CPP
        ${RW_DRAWABLE_USER_FILES}
        DrawableGeometryNode.cpp
        SceneGraph.cpp
        SceneNode.cpp
        DrawableNode.cpp
        DrawableNodeClone.cpp
        GroupNode.cpp
        SceneCamera.cpp
        WorkCellScene.cpp
        SceneViewer.cpp
        Render.cpp
        SceneDescriptor.cpp
        TextureData.cpp
        Plot.cpp
        PlotGenerator.cpp
    )

    set(FILES_HPP
        Model3D.hpp
        DrawableGeometryNode.hpp
        SceneGraph.hpp
        SceneNode.hpp
        DrawableNode.hpp
        GroupNode.hpp
        SceneCamera.hpp
        WorkCellScene.hpp
        SceneViewer.hpp
        Render.hpp
        DrawableNodeClone.hpp
        TextureData.hpp
        Plot.hpp
        PlotGenerator.hpp
    )

    rw_add_library(${SUBSYS_NAME} ${FILES_CPP} ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw/graphics" ${FILES_HPP})
    rw_add_includes(${SUBSYS_NAME} "rw" ../graphics.hpp)

    if(CMAKE_COMPILER_IS_GNUCXX)
        set_source_files_properties(WorkCellScene.cpp PROPERTIES COMPILE_FLAGS -fno-gnu-unique)
    endif()

    target_link_libraries(${SUBSYS_NAME} PUBLIC ${Boost_LIBRARIES} ${SUBSYS_DEPS})

    target_include_directories(
        ${SUBSYS_NAME} PUBLIC ${Boost_INCLUDE_DIR} $<BUILD_INTERFACE:${RW_ROOT}/src>
                              $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}>
    )

endif()

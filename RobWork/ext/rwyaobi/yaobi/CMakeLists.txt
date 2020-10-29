cmake_minimum_required(VERSION 3.5.1)

project(yaobi)

set(YAOBI_HPP_FILES
    yaobi.h
    yaobi_config.h
    yaobi_fpu.h
    yaobi_matvec.h
    yaobi_mesh_interface.h
    yaobi_moments.h
    yaobi_obb_node.h
    yaobi_settings.h
    yaobi_tree_builder.h
    yaobi_tri_overlap.h
    yaobi_vector.h
)

# CMake does not allow a library and executable to have the same name, so we use the name lualib for
# the library.
add_library(yaobi yaobi_mesh_interface.cpp yaobi_moments.cpp yaobi_obb_node.cpp
                  yaobi_tree_builder.cpp yaobi_tri_overlap.cpp yaobi.cpp
)

target_include_directories(
    yaobi INTERFACE $<BUILD_INTERFACE:${YAOBI_INCLUDE_DIR}>
                    $<INSTALL_INTERFACE:${INCLUDE_INSTALL_DIR}/ext/rwyaobi/>
)

set_target_properties(yaobi PROPERTIES WINDOWS_EXPORT_ALL_SYMBOLS TRUE)

configure_file(${CMAKE_CURRENT_SOURCE_DIR}/yaobi_config.h.cmake
               ${CMAKE_CURRENT_SOURCE_DIR}/yaobi_config.h
)

install(
    TARGETS yaobi
    EXPORT ${PROJECT_PREFIX}Targets
    RUNTIME DESTINATION ${BIN_INSTALL_DIR} COMPONENT yaobi
    LIBRARY DESTINATION ${LIB_INSTALL_DIR} COMPONENT yaobi
    ARCHIVE DESTINATION ${LIB_INSTALL_DIR} COMPONENT yaobi
)
install(
    FILES ${YAOBI_HPP_FILES}
    DESTINATION "${INCLUDE_INSTALL_DIR}/ext/rwyaobi/"
    COMPONENT yaobi
)
set(MODULE_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

# Configure yaml robot module
configure_file(yaml/vive_tracker.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/yaml/vive_tracker.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/yaml/vive_tracker.yaml"
        DESTINATION "${MODULE_INSTALL_LOCATION}/yaml/")

configure_file(urdf/vive_tracker.in.urdf
               "${CMAKE_CURRENT_BINARY_DIR}/urdf/vive_tracker.urdf")
install(FILES ${CMAKE_CURRENT_BINARY_DIR}/urdf/vive_tracker.urdf
        DESTINATION "${MODULE_INSTALL_LOCATION}/urdf/")

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/meshes/vive_tracker.stl"
        DESTINATION "${MODULE_INSTALL_LOCATION}/meshes/")

# Configure the alias accordingly
configure_file(alias/vive_tracker.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/alias/vive_tracker.yaml")

# Install the alias
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/alias/vive_tracker.yaml"
        DESTINATION "${MC_ROBOTS_ALIASES_DIRECTORY}")

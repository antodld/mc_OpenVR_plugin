set(MODULE_INSTALL_LOCATION "${CMAKE_INSTALL_PREFIX}/share/${PROJECT_NAME}")

# Configure yaml robot module
set(URDF_PATH "urdf/vive_tracker_gray.urdf")
configure_file(yaml/vive_tracker.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/yaml/vive_tracker_gray.yaml")
set(URDF_PATH "urdf/vive_tracker_red.urdf")
configure_file(yaml/vive_tracker.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/yaml/vive_tracker_red.yaml")
set(URDF_PATH "urdf/vive_tracker_green.urdf")
configure_file(yaml/vive_tracker.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/yaml/vive_tracker_green.yaml")
set(URDF_PATH "urdf/vive_tracker_blue.urdf")
configure_file(yaml/vive_tracker.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/yaml/vive_tracker_blue.yaml")

install(
  FILES "${CMAKE_CURRENT_BINARY_DIR}/yaml/vive_tracker_gray.yaml"
        "${CMAKE_CURRENT_BINARY_DIR}/yaml/vive_tracker_red.yaml"
        "${CMAKE_CURRENT_BINARY_DIR}/yaml/vive_tracker_green.yaml"
        "${CMAKE_CURRENT_BINARY_DIR}/yaml/vive_tracker_blue.yaml"
  DESTINATION "${MODULE_INSTALL_LOCATION}/yaml/")

configure_file(urdf/vive_tracker_gray.in.urdf
               "${CMAKE_CURRENT_BINARY_DIR}/urdf/vive_tracker_gray.urdf")
configure_file(urdf/vive_tracker_red.in.urdf
               "${CMAKE_CURRENT_BINARY_DIR}/urdf/vive_tracker_red.urdf")
configure_file(urdf/vive_tracker_green.in.urdf
               "${CMAKE_CURRENT_BINARY_DIR}/urdf/vive_tracker_green.urdf")
configure_file(urdf/vive_tracker_blue.in.urdf
               "${CMAKE_CURRENT_BINARY_DIR}/urdf/vive_tracker_blue.urdf")
install(
  FILES ${CMAKE_CURRENT_BINARY_DIR}/urdf/vive_tracker_gray.urdf
        ${CMAKE_CURRENT_BINARY_DIR}/urdf/vive_tracker_red.urdf
        ${CMAKE_CURRENT_BINARY_DIR}/urdf/vive_tracker_green.urdf
        ${CMAKE_CURRENT_BINARY_DIR}/urdf/vive_tracker_blue.urdf
  DESTINATION "${MODULE_INSTALL_LOCATION}/urdf/")

install(FILES "${CMAKE_CURRENT_SOURCE_DIR}/meshes/vive_tracker.stl"
        DESTINATION "${MODULE_INSTALL_LOCATION}/meshes/")

# Configure the alias accordingly
configure_file(alias/vive_tracker.in.yaml
               "${CMAKE_CURRENT_BINARY_DIR}/alias/vive_tracker.yaml")

# Install the alias
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/alias/vive_tracker.yaml"
        DESTINATION "${MC_ROBOTS_ALIASES_DIRECTORY}")

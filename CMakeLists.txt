cmake_minimum_required(VERSION 3.1.3)

set(PROJECT_NAME OpenVRPlugin)
set(PROJECT_DESCRIPTION "")
set(PROJECT_URL "")
set(PROJECT_VERSION 0.0.1)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CMAKE_CXX_STANDARD 11)

# include(cmake/base.cmake)
include(FindPkgConfig)

option(AUTOLOAD_OpenVRPlugin_PLUGIN "Automatically load the plugin" OFF)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc REQUIRED)
endif()

# This creates a target PkgConfig::openvr if openvr is found
pkg_check_modules(openvr REQUIRED IMPORTED_TARGET openvr)

# Configure and install the plugin configuration
configure_file(etc/${PROJECT_NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml")
install(FILES "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml" DESTINATION "${MC_PLUGINS_RUNTIME_INSTALL_PREFIX}/etc")


set(plugin_SRC
  src/OpenVRPlugin.cpp
  )

set(plugin_HDR
  src/OpenVRPlugin.h
)

# The add_plugin macro is provided by mc_rtc
add_plugin(${PROJECT_NAME} "${plugin_SRC}" "${plugin_HDR}")
target_link_libraries(${PROJECT_NAME} PUBLIC PkgConfig::openvr)

if(BUILD_TESTING)
  add_subdirectory(tests)
endif()

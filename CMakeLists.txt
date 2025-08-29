cmake_minimum_required(VERSION 4.0)

set(PROJECT_NAME OpenVRPlugin)
set(PROJECT_DESCRIPTION "")
set(PROJECT_URL "")
set(PROJECT_VERSION 0.0.1)
set(PROJECT_USE_CMAKE_EXPORT TRUE)
set(CMAKE_CXX_STANDARD 11)

option(BUILD_MC_RTC_PLUGIN "Build the mc_rtc plugin" ON)
option(BUILD_DATA_PUBLISHER "Build the data sender executable" ON)
option(BUILD_TESTING "build the mc_rtc test controller" OFF)

# include(cmake/base.cmake)
include(FindPkgConfig)

option(AUTOLOAD_OpenVRPlugin_PLUGIN "Automatically load the plugin" OFF)

project(${PROJECT_NAME} LANGUAGES CXX VERSION ${PROJECT_VERSION})

# Find mc_rtc if we are not building inside mc_rtc itself
if(NOT TARGET mc_rtc::mc_control)
  find_package(mc_rtc)
  if(NOT mc_rtc_FOUND)
    set(BUILD_MC_RTC_PLUGIN OFF)
    message("NOT FOUND mc_rtc")
  endif()
endif()


find_package(yaml-cpp)
find_package(UDPDataLink REQUIRED)

if(NOT yaml-cpp_FOUND)
  set(BUILD_DATA_PUBLISHER OFF)
  message("NOT FOUND yaml-cpp")
endif()

# This creates a target PkgConfig::openvr if openvr is found
set(ENV{PKG_CONFIG_PATH} "${CMAKE_INSTALL_PREFIX}/share/pkgconfig:$ENV{PKG_CONFIG_PATH}")
pkg_check_modules(openvr REQUIRED IMPORTED_TARGET openvr)
include_directories(${openvr_INCLUDE_DIRS})

# message("HERE IS DIR ${openvr_INCLUDE_DIRS} ")


find_package(Boost REQUIRED COMPONENTS serialization)

# Configure the plugin configuration
configure_file(etc/${PROJECT_NAME}.in.yaml "${CMAKE_CURRENT_BINARY_DIR}/etc/${PROJECT_NAME}.yaml")

# Create a library to export plugin data
# This is intended so that other projects can link with OpenVRData

set(OpenVRData_SRC
  src/OpenVRData.cpp
  )

set(OpenVRData_HDR
  include/OpenVRPlugin/OpenVRData.h
  include/OpenVRPlugin/boost_serialization.h
)

add_library(OpenVRData SHARED ${OpenVRData_SRC} ${OpenVRData_HDR})

# target_link_libraries(OpenVRData PUBLIC ${Boost_LIBRARIES})
target_link_libraries(OpenVRData PUBLIC PkgConfig::openvr)
# workaround as PkgConfig::openvr includes ${CMAKE_INSTALL_PREFIX}/include/openvr which
# prevents calls to #include <openvr/openvr.h>
target_include_directories(OpenVRData PUBLIC ${CMAKE_INSTALL_PREFIX}/include)

export(TARGETS OpenVRData FILE "OpenVRDataConfig.cmake")

install(TARGETS OpenVRData
  EXPORT OpenVRData
  RUNTIME DESTINATION bin
  LIBRARY DESTINATION lib
  ARCHIVE DESTINATION lib)

install(EXPORT OpenVRData
  FILE "OpenVRDataConfig.cmake"
  DESTINATION lib/cmake/OpenVRData)

install(
DIRECTORY ${CMAKE_SOURCE_DIR}/include/ DESTINATION include)

if(BUILD_DATA_PUBLISHER)
  add_executable(PluginLink src/PluginLink.cpp)
  target_link_libraries(PluginLink OpenVRData)
  target_link_libraries(PluginLink yaml-cpp)
  target_link_libraries(OpenVRData PUBLIC UDPDataLink)
endif()

if(BUILD_MC_RTC_PLUGIN)

  #install the plugin configuration
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
  target_link_libraries(${PROJECT_NAME} PUBLIC OpenVRData)
  target_link_libraries(OpenVRData PUBLIC UDPDataLink)

  # export(TARGETS ${PROJECT_NAME} FILE "${PROJECT_NAME}Config.cmake")
  # install(EXPORT ${PROJECT_NAME}
  #   FILE "${PROJECT_NAME}Config.cmake"
  #   DESTINATION lib/cmake/${PROJECT_NAME})


  if(BUILD_TESTING)
    add_subdirectory(tests)
  endif()

endif()

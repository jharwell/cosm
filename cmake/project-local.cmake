################################################################################
# Build Environment Configuration
################################################################################
# We are might be linking with a shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(cosm_CHECK_LANGUAGE "CXX")

# Each conference tag=minor increment. Each minor feature added=patch increment.
set(PROJECT_VERSION_MAJOR 1)
set(PROJECT_VERSION_MINOR 3)
set(PROJECT_VERSION_PATCH 0)
set(cosm_VERSION "${PROJECT_VERSION_MAJOR}.${PROJECT_VERSION_MINOR}.${PROJECT_VERSION_PATCH}")

if(NOT DEFINED COSM_BUILD_ENV)
  set(COSM_BUILD_ENV "DEVEL")
endif()


if("${COSM_BUILD_ENV}" MATCHES "DEVEL" )
  # Nothing to do for now
elseif("${COSM_BUILD_ENV}" MATCHES "ROBOT" )
  # Nothing to do for now
elseif("${COSM_BUILD_ENV}" MATCHES "MSI" )
  set(LIBRA_DEPS_PREFIX /home/gini/shared/swarm/$ENV{MSIARCH})
else()
  message(FATAL_ERROR "Build environment must be: [DEVEL,ROBOT,MSI]")
endif()

libra_configure_version(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/version/version.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/src/version/version.cpp
  cosm_components_SRC
  message("${cosm_components_SRC}")
  )
# list(APPEND cosm_components_SRC "${CMAKE_CURRENT_BINARY_DIR}/src/version/version.cpp")

include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/hal.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/pal.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/argos.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ros.cmake)
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cosm-config-summary.cmake)

################################################################################
# HAL Configuration
################################################################################
cosm_hal_configure_target()

################################################################################
# PAL Configuration
################################################################################
cosm_pal_configure_target()

################################################################################
# Qt Configuration
################################################################################

# Conditionally compile/link Qt visualizations.
#
# - Qt not reliably available when building for MSI.
#
# - Qt cmake module is only available on Linux when building for ARGoS, and when
#   the compiler is not Intel, because the cmake module for Qt does not play
#   nice with the Intel compiler.
if (NOT DEFINED COSM_WITH_VIS)
  if ("${CMAKE_SYSTEM_PROCESSOR}" MATCHES "x86_64" AND
      "${COSM_BUILD_FOR}" MATCHES "ARGOS" AND
      NOT "${CMAKE_CXX_COMPILER_ID}" MATCHES "Intel")
    set(COSM_WITH_VIS ON)
  else()
    set(COSM_WITH_VIS OFF)
  endif()
endif()

if (COSM_WITH_VIS)
  # ARGoS supports both Qt5 and Qt6, so we test for both. Qt5 only
  # supports version-less importing since 5.15, so to support earlier
  # version we explicitly have to use the version variable, which is
  # always defined by Qt5 and Qt6.
  set(CMAKE_AUTOMOC ON)
  find_package(QT NAMES Qt6 Qt5 COMPONENTS Core Widgets Gui REQUIRED)
  find_package(Qt${QT_VERSION_MAJOR} COMPONENTS Core Widgets Gui REQUIRED)
  set(CMAKE_AUTOMOC OFF)

  find_package(OpenGL REQUIRED)
endif()

################################################################################
# Components
################################################################################
string(CONCAT Q3D_bindings_regex
  "src/kin2D|"
  "src/controller/base_controller2D|"
  "src/controller/base_controllerQ3D|"
  "src/subsystem/saa_subsystemQ3D"
  )
string(CONCAT 3D_bindings_regex
  "src/controller/base_controller3D|"
  "src/subsystem/saa_subsystem3D"
  )

string(CONCAT common_regex
  "src/ds/config|"
  "src/hal/sensors|"
  "src/hal/actuators|"
  "src/kin/|" # trailing slash needed not to catch kin2D
  "src/nav|"
  "src/pal/config|"
  "src/pal/base_swarm_manager|"
  "src/ta|"
  "src/init|"
  "src/controller/base_controller.cpp|"
  "src/controller/config|"
  "src/subsystem/base_|"
  "src/subsystem/config|"
  "src/subsystem/perception|"
  "src/apf2D|"
  "src/flocking|"
  "src/metrics"
  )

libra_component_register_as_src(
  cosm_common_SRC
  cosm
  "${cosm_SRC}"
  common
  "(${common_regex})")

if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/argos.cmake)

  cosm_argos_configure_components()

elseif("${COSM_BUILD_FOR}" MATCHES "ROS")
  include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/ros.cmake)
  cosm_ros_configure_components()
endif()


libra_requested_components_check(cosm)

################################################################################
# External Projects
################################################################################
# RCPPSW
find_package(rcppsw COMPONENTS REQUIRED
  config
  control
  er
  init
  math
  metrics
  patterns
  utils
  types
  # Needed when cross compiling so we pick up the right package.
  HINTS ${CMAKE_INSTALL_PREFIX}
  )

# Lua (not needed for compilation, but for rtags)
find_package(Lua REQUIRED)

if("${COSM_BUILD_FOR}" MATCHES "ROS")
  set(CATKIN_PKGS
    roscpp
    rosconsole
    std_msgs
    )
  if("${COSM_BUILD_FOR}" MATCHES "ETURTLEBOT3")
    set(CATKIN_PKGS ${ROS_PKGS} sr04us tsl2591)
  endif()

  # ROS
  find_package(catkin REQUIRED COMPONENTS
    ${CATKIN_PKGS}
    HINTS ${CMAKE_INSTALL_PREFIX}
    )
endif()

# Notably we do NOT include ARGoS here, because it can't be found via
# find_package() unless it is installed in a system location. Really
# irritating.

################################################################################
# Libraries
################################################################################
# Create the source for the SINGLE library to build by combining the
# source of the selected components
foreach(component ${cosm_FIND_COMPONENTS})
  if(${cosm_${component}_FOUND})
    list(APPEND cosm_components_SRC ${cosm_} ${cosm_${component}_SRC})
  endif()
endforeach()

# Define the COSM library
set(cosm_LIBRARY ${target}-${COSM_HAL_TARGET})

add_library(
  ${cosm_LIBRARY}
  SHARED
  ${cosm_components_SRC}
  )

# Alias so we plug into the LIBRA framework properly
add_library(cosm ALIAS ${cosm_LIBRARY})


set(cosm_LIBRARY_NAME ${target}-${COSM_HAL_TARGET})

set_target_properties(${cosm_LIBRARY}
  PROPERTIES OUTPUT_NAME
  ${cosm_LIBRARY_NAME}
  )

# Setting this results in TWO files being installed: the actual
# library with the version embedded, and a symlink to the actual
# library with the same name sans the embedded version (if COSM
# is built as a shared library).
set_target_properties(${cosm_LIBRARY}
  PROPERTIES
  VERSION ${cosm_VERSION}
  SOVERSION ${cosm_VERSION}
  )

########################################
# Include directories
########################################
target_include_directories(
  ${cosm_LIBRARY}
  PUBLIC
  $<BUILD_INTERFACE:${cosm_DIR}/include>
  $<BUILD_INTERFACE:${rcppsw_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
  )

target_include_directories(
  ${cosm_LIBRARY}
  SYSTEM PUBLIC
  ${LIBRA_DEPS_PREFIX}/include
  ${CMAKE_INSTALL_PREFIX}/include
  ${catkin_INCLUDE_DIRS}
  )

if(NOT "${COSM_BUILD_ENV}" MATCHES "MSI" )
  target_include_directories(
    ${cosm_LIBRARY}
    SYSTEM PUBLIC
    # Not needed for compilation, but for rtags
    ${LUA_INCLUDE_DIR}
    )
endif()

########################################
# Link Libraries
########################################
target_link_libraries(${cosm_LIBRARY} rcppsw::rcppsw)

if("${COSM_BUILD_FOR}" MATCHES "ROS")
  target_link_libraries(${cosm_LIBRARY} ${catkin_LIBRARIES})
endif()

if (${COSM_WITH_VIS})
  target_link_libraries(${cosm_LIBRARY}
    Qt${QT_VERSION_MAJOR}::Widgets
    Qt${QT_VERSION_MAJOR}::Core
    Qt${QT_VERSION_MAJOR}::Gui
    OpenGL::GL
    )
endif()
if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  target_link_directories(${cosm_LIBRARY}
    PUBLIC
    ${LIBRA_DEPS_PREFIX}/lib/argos3)
  target_link_libraries(${cosm_LIBRARY}
    argos3core_simulator
    argos3plugin_simulator_footbot
    argos3plugin_simulator_epuck
    argos3plugin_simulator_drone
    argos3plugin_simulator_entities
    argos3plugin_simulator_dynamics2d
    argos3plugin_simulator_genericrobot
    argos3plugin_simulator_qtopengl
    argos3plugin_simulator_media
    )
endif()

# Force failures at build time rather than runtime
target_link_options(${cosm_LIBRARY} PRIVATE -Wl,--no-undefined)

########################################
# Compile Options/Definitions
########################################
cosm_hal_configure_buildflags()
cosm_pal_configure_buildflags()

################################################################################
# Installation and Deployment
################################################################################
libra_configure_exports_as(${cosm_LIBRARY} ${CMAKE_INSTALL_PREFIX})

# Install COSM
libra_register_target_for_install(${cosm_LIBRARY} ${CMAKE_INSTALL_PREFIX})
libra_register_headers_for_install(include/cosm ${CMAKE_INSTALL_PREFIX})

file(GLOB COSM_CMAKE_FILES "${CMAKE_CURRENT_SOURCE_DIR}/cmake/*.cmake")
libra_register_extra_configs_for_install(${cosm_LIBRARY}
  "${COSM_CMAKE_FILES}"
  ${CMAKE_INSTALL_PREFIX})

# Deploy COSM
set(CPACK_SET_DESTDIR YES)
set(CPACK_PACKAGE_FILE_NAME
  ${cosm_LIBRARY}-${cosm_VERSION}-${CMAKE_SYSTEM_PROCESSOR})
libra_configure_cpack(
  "DEB;TGZ"

  "Core Swarm (COSM) is a collection of non application, method, or controller
specific software components that can be reused across multiple
Multi-Agent System (MAS) projects (i.e., generic in the context of
MAS, but maybe not more broadly).  "

  "John Harwell"
  "https://jharwell.github.io/cosm"
  "John Harwell <john.r.harwell@gmail.com>")

################################################################################
# Status
################################################################################
libra_config_summary()
cosm_config_summary()

################################################################################
# Build Environment Configuration Options                                      #
################################################################################
# We are might be linking with a shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(cosm_CHECK_LANGUAGE "CXX")

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

################################################################################
# HAL Configuration Options                                                    #
################################################################################
if(NOT DEFINED COSM_BUILD_FOR)
  set(COSM_BUILD_FOR "ARGOS_FOOTBOT")
endif()

if("${COSM_BUILD_FOR}" MATCHES "ARGOS_FOOTBOT")
  set(COSM_HAL_TARGET "argos-footbot")
  set(COSM_ARGOS_ROBOT_TYPE "foot-bot")
  set(COSM_ARGOS_ROBOT_NAME_PREFIX "fb")
  set(COSM_ARGOS_CONTROLLER_XML_ID "fbc")

elseif("${COSM_BUILD_FOR}" MATCHES "ARGOS_EEPUCK3D")
  set(COSM_HAL_TARGET "argos-eepuck3D")
  set(COSM_ARGOS_ROBOT_TYPE "e-puck")
  set(COSM_ARGOS_ROBOT_NAME_PREFIX "ep")
  set(COSM_ARGOS_CONTROLLER_XML_ID "epc")

elseif("${COSM_BUILD_FOR}" MATCHES "ARGOS_PIPUCK")
  set(COSM_HAL_TARGET "argos-pipuck")
  set(COSM_ARGOS_ROBOT_TYPE "pipuck")
  set(COSM_ARGOS_ROBOT_NAME_PREFIX "pp")
  set(COSM_ARGOS_CONTROLLER_XML_ID "ppc")
elseif("${COSM_BUILD_FOR}" MATCHES "ROS_ETURTLEBOT3")
  set(COSM_HAL_TARGET "ros-eturtlebot3")
  set(COSM_ROS_ROBOT_TYPE "eturtlebot3")
  set(COSM_ROS_ROBOT_NAME_PREFIX "etb3_")
else()
  set(COSM_BUILD_TARGETS
    ARGOS_FOOTBOT
    ARGOS_EEPUCK3D
    ARGOS_PIPUCK
    ROS_ETURTLEBOT3
    )
  message(FATAL_ERROR "Build target must be one of ${COSM_BUILD_TARGETS}")
endif()

################################################################################
# PAL Configuration Options                                                    #
################################################################################
if("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  set(COSM_PAL_TARGET "ARGOS")
  set(COSM_PAL_NATIVE_BUILD ON)

elseif("${COSM_BUILD_FOR}" MATCHES "ROS")
  set(COSM_PAL_TARGET "ROS")
  if(NOT CMAKE_CROSSCOMPILING)
    set(COSM_PAL_NATIVE_BUILD ON)
  else()
    set(COSM_PAL_NATIVE_BUILD OFF)
  endif()

  execute_process(COMMAND git rev-parse HEAD
    OUTPUT_VARIABLE COSM_ROS_MD5_CURRENT
    OUTPUT_STRIP_TRAILING_WHITESPACE)

  if (NOT COSM_ROS_MD5_CURRENT STREQUAL COSM_ROS_MD5)
    message("ROS Msg MD5 changed ${COSM_ROS_MD5} -> ${COSM_ROS_MD5_CURRENT}")
    set(COSM_ROS_MD5
      0 # temporarily to make stuff easier ${COSM_ROS_MD5_CURRENT}
      CACHE INTERNAL "MD5 hash used for ROS message versioning")
    endif()
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pal/pal.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/src/pal/pal.cpp
  @ONLY
  )

list(APPEND cosm_components_SRC "${CMAKE_CURRENT_BINARY_DIR}/src/pal/pal.cpp")

################################################################################
# Qt Configuration Options                                                     #
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
# Components                                                                   #
################################################################################
string(CONCAT common_regex
  "src/ds/config|"
  "src/hal/subsystem|"
  "src/hal/sensors|"
  "src/hal/actuators|"
  "src/kin2D|"
  "src/pal/config|"
  "src/pal/base_swarm_manager|"
  "src/controller|"
  "src/ta|"
  "src/init|"
  "src/subsystem|"
  "src/steer2D|"
  "src/metrics"
  )

component_register_as_src(
  cosm_common_SRC
  cosm
  "${cosm_SRC}"
  common
  "(${common_regex})")

if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  string(CONCAT argos_regex
    "src/ds|"
    "src/arena|"
    "src/convergence|"
    "src/foraging|"
    "src/fsm|"
    "src/hal/argos|"
    "src/hal/sensors|"
    "src/metrics|"
    "src/oracle|"
    "src/pal/argos|"
    "src/repr|"
    "src/spatial|"
    "src/tv|"
    "src/argos"
    )
  component_register_as_src(
    cosm_argos_SRC
    cosm
    "${cosm_SRC}"
    argos
    "(${argos_regex})")

  if(COSM_WITH_VIS)
    component_register_as_src(
      cosm_argos_vis_SRC
      cosm
      "${cosm_SRC}"
      argos_vis
      "src/argos/vis")
  endif()

  # Root project (not used in find_package())
  if (NOT cosm_FIND_COMPONENTS)
    set(cosm_FIND_COMPONENTS
      common
      argos
      argos_vis
      )
  endif()

elseif("${COSM_BUILD_FOR}" MATCHES "ROS")
  string(CONCAT ros_regex
    "src/ros|"
    "src/hal/ros|"
    "src/pal/ros|"
    "src/fsm|"
    "src/foraging/fsm|"
    "src/kin2D|"
    "src/foraging/metrics|"
    "src/spatial/fsm|"
    "src/spatial/metrics|"
    "src/spatial/strategy|"
    "src/repr/operations/block_pickup|"
    "src/repr/base_block3D|"
    "src/repr/config/xml"
    )
  component_register_as_src(
    cosm_ros_SRC
    cosm
    "${cosm_SRC}"
    ros
    "(${ros_regex})")

  # Root project (not used in find_package())
  if (NOT cosm_FIND_COMPONENTS)
    set(cosm_FIND_COMPONENTS
      common
      ros
      )
  endif()

endif()


requested_components_check(cosm)

################################################################################
# External Projects                                                            #
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
# Libraries                                                                    #
################################################################################
# Create the source for the SINGLE library to build by combining the
# source of the selected components
foreach(component ${cosm_FIND_COMPONENTS})
  if(${cosm_${component}_FOUND})
    list(APPEND cosm_components_SRC ${cosm_} ${cosm_${component}_SRC})
  endif()
endforeach()

# Configure version
execute_process(COMMAND git rev-list --count HEAD
  OUTPUT_VARIABLE COSM_VERSION
  OUTPUT_STRIP_TRAILING_WHITESPACE)

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/version.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/src/version.cpp
  @ONLY
  )
list(APPEND cosm_components_SRC "${CMAKE_CURRENT_BINARY_DIR}/src/version.cpp")

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
if ("${COSM_BUILD_FOR}" MATCHES "ROS")
  target_compile_options(${cosm_LIBRARY}
    PUBLIC
    -Wno-psabi)
endif()

if ("${COSM_HAL_TARGET}" MATCHES "argos-footbot")
  target_compile_definitions(${cosm_LIBRARY}
    PUBLIC
    COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_FOOTBOT)
elseif("${COSM_HAL_TARGET}" MATCHES "argos-eepuck3d")
  target_compile_definitions(${cosm_LIBRARY}
    PUBLIC
    COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_EEPUCK3D)
elseif("${COSM_HAL_TARGET}" MATCHES "argos-pipuck")
  target_compile_definitions(${cosm_LIBRARY}
    PUBLIC
    COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_PIPUCK)
elseif("${COSM_HAL_TARGET}" MATCHES "ros-eturtlebot3")
  target_compile_definitions(${cosm_LIBRARY}
    PUBLIC
    COSM_HAL_TARGET=COSM_HAL_TARGET_ROS_ETURTLEBOT3)
endif()

if("${COSM_PAL_TARGET}" MATCHES "ARGOS")
  target_compile_definitions(${cosm_LIBRARY}
    PUBLIC
    COSM_ENABLE_PAL_TARGET_ARGOS)
elseif("${COSM_PAL_TARGET}" MATCHES "ROS")
  target_compile_definitions(${cosm_LIBRARY}
    PUBLIC
    COSM_ENABLE_PAL_TARGET_ROS)
endif()

################################################################################
# Installation                                                                 #
################################################################################
configure_exports_as(${cosm_LIBRARY} ${CMAKE_INSTALL_PREFIX})

# Install cosm
register_target_for_install(${cosm_LIBRARY} ${CMAKE_INSTALL_PREFIX})
register_headers_for_install(include/cosm ${CMAKE_INSTALL_PREFIX})
register_extra_configs_for_install(${cosm_LIBRARY}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cosm-config-summary.cmake
  ${CMAKE_INSTALL_PREFIX})

################################################################################
# Status                                                                       #
################################################################################
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cosm-config-summary.cmake)
libra_config_summary()

cosm_config_summary()

################################################################################
# Build Environment Configuration Options                                      #
################################################################################
# We are might be linking with a shared library
set(CMAKE_POSITION_INDEPENDENT_CODE ON)

set(cosm_CHECK_LANGUAGE "CXX")

if(NOT DEFINED COSM_BUILD_ENV)
  set(COSM_BUILD_ENV "LOCAL")
endif()

if("${COSM_BUILD_ENV}" MATCHES "LOCAL" )
# Nothing to do for now
elseif("${COSM_BUILD_ENV}" MATCHES "MSI" )
  set(LIBRA_DEPS_PREFIX /home/gini/shared/swarm/$ENV{MSIARCH})
else()
  message(FATAL_ERROR "Build environment must be: [LOCAL,MSI]")
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
elseif("${COSM_BUILD_FOR}" MATCHES "ROS_TURTLEBOT3")
  set(COSM_HAL_TARGET "ros-turtlebot3")
  set(COSM_ROS_ROBOT_TYPE "turtlebot3")
  set(COSM_ROS_ROBOT_NAME_PREFIX "tb3_")
else()
  set(COSM_BUILD_TARGETS
    ARGOS_FOOTBOT
    ARGOS_EEPUCK3D
    ARGOS_PIPUCK
    ROS_TURTLEBOT3
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
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cosm/pal/pal.hpp.in
  ${CMAKE_CURRENT_SOURCE_DIR}/include/cosm/pal/pal.hpp
  @ONLY
  )

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
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
  set(CMAKE_AUTOMOC OFF)
endif()

################################################################################
# Components                                                                   #
################################################################################
string(CONCAT common_regex
  "src/ds|"
  "src/hal/subsystem|"
  "src/hal/sensors|"
  "src/hal/actuators|"
  "src/kin2D|"
  "src/ta|"
  "src/pal/config|"
  "src/pal/base_swarm_manager|"
  "src/controller|"
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
    "src/ds|"
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
  math
  metrics
  patterns
  utils
  types
  # Needed when cross compiling so we pick up the right package.
  HINTS ${CMAKE_INSTALL_PREFIX}
  )
if("${COSM_BUILD_FOR}" MATCHES "ROS")
  # ROS
  find_package(catkin REQUIRED COMPONENTS
    roscpp
    rosconsole
    std_msgs
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

# Define the COSM library
add_library(
  cosm-${COSM_HAL_TARGET}
  STATIC
  ${cosm_components_SRC}
  )
set(cosm_LIBRARY_NAME ${target}-${COSM_HAL_TARGET})

# Alias so we plug into the LIBRA framework properly
add_library(cosm ALIAS ${cosm_LIBRARY_NAME})

########################################
# Include directories
########################################
target_include_directories(
  ${cosm_LIBRARY_NAME}
  PUBLIC
  $<BUILD_INTERFACE:${cosm_DIR}/include>
  $<BUILD_INTERFACE:${rcppsw_INCLUDE_DIRS}>
  $<INSTALL_INTERFACE:include>
  )

target_include_directories(
  ${cosm_LIBRARY_NAME}
  SYSTEM PUBLIC
  ${LIBRA_DEPS_PREFIX}/include
  ${CMAKE_INSTALL_PREFIX}/system/include
  ${catkin_INCLUDE_DIRS}
  )

if(NOT "${COSM_BUILD_ENV}" MATCHES "MSI" )
  target_include_directories(
    ${cosm_LIBRARY_NAME}
    SYSTEM PUBLIC
    # Not needed for compilation, but for rtags
    /usr/include/lua5.3
    )
endif()

########################################
# Link Libraries
########################################
target_link_libraries(${cosm_LIBRARY_NAME} rcppsw::rcppsw)

if("${COSM_BUILD_FOR}" MATCHES "ROS")
  target_link_libraries(${cosm_LIBRARY_NAME} ${catkin_LIBRARIES})
endif()
message("${rcppsw_LIBRARIES}")
if (${COSM_WITH_VIS})
  target_link_libraries(${cosm_LIBRARY_NAME} Qt5::Widgets Qt5::Core Qt5::Gui GL)
endif()

if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  target_link_directories(${cosm_LIBRARY_NAME}
    PUBLIC
    ${LIBRA_DEPS_PREFIX}/lib/argos3)
endif()

########################################
# Compile Options/Definitions
########################################
if ("${COSM_BUILD_FOR}" MATCHES "ROS")
  target_compile_options(${cosm_LIBRARY_NAME}
    PUBLIC
    -Wno-psabi)
endif()

# This is needed for HAL SAA sensing/actuator access with boost::variant
# target_compile_definitions(${cosm_LIBRARY_NAME}
#   PUBLIC
#   BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT)

if ("${COSM_HAL_TARGET}" MATCHES "argos-footbot")
  target_compile_definitions(${cosm_LIBRARY_NAME}
    PUBLIC
    COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_FOOTBOT)
elseif("${COSM_HAL_TARGET}" MATCHES "argos-eepuck3d")
  target_compile_definitions(${cosm_LIBRARY_NAME}
    PUBLIC
    COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_EEPUCK3D)
elseif("${COSM_HAL_TARGET}" MATCHES "argos-pipuck")
  target_compile_definitions(${cosm_LIBRARY_NAME}
    PUBLIC
    COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_PIPUCK)
elseif("${COSM_HAL_TARGET}" MATCHES "ros-turtlebot3")
  target_compile_definitions(${cosm_LIBRARY_NAME}
    PUBLIC
    COSM_HAL_TARGET=COSM_HAL_TARGET_ROS_TURTLEBOT3)
endif()

if("${COSM_PAL_TARGET}" MATCHES "ARGOS")
  target_compile_definitions(${cosm_LIBRARY_NAME}
    PUBLIC
    COSM_PAL_TARGET=COSM_PAL_TARGET_ARGOS)
elseif("${COSM_PAL_TARGET}" MATCHES "ROS")
  target_compile_definitions(${cosm_LIBRARY_NAME}
    PUBLIC
    COSM_PAL_TARGET=COSM_PAL_TARGET_ROS)
endif()

################################################################################
# Installation                                                                 #
################################################################################
configure_exports_as(${cosm_LIBRARY_NAME})

# Install cosm
register_target_for_install(${cosm_LIBRARY_NAME} ${CMAKE_INSTALL_PREFIX})
register_headers_for_install(include/cosm ${CMAKE_INSTALL_PREFIX})
register_extra_configs_for_install(${cosm_LIBRARY_NAME}
  ${CMAKE_CURRENT_SOURCE_DIR}/cmake/cosm-config-summary.cmake
  ${CMAKE_INSTALL_PREFIX})

################################################################################
# Status                                                                       #
################################################################################
include(${CMAKE_CURRENT_SOURCE_DIR}/cmake/cosm-config-summary.cmake)
libra_config_summary()

cosm_config_summary()

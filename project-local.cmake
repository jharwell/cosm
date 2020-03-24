################################################################################
# Configuration Options                                                        #
################################################################################
set(COSM_HAL_TARGET "NONE" CACHE STRING "Specify the Hardware Abstraction Layer (HAL) target")
define_property(CACHED_VARIABLE PROPERTY "COSM_HAL_TARGET"
		BRIEF_DOCS "Specify the Hardware Abstraction Layer (HAL) target"
		FULL_DOCS "Must be exactly one of: [argos-footbot,lego-ev3]"
                )

# Conditionally compile/link Qt visualizations.
#
# - Qt not reliably available when building for MSI
# - Getting it to work with the EV3 would require building Qt from source, and
#   there is no GUI stuff anyway so...
# - Qt cmake module is only available on Linux when building for ARGoS, and when
#   the compiler is not Intel, because the cmake module for Qt does not play
#   nice with the Intel compiler.
if ("${LIBRA_BUILD_FOR}" MATCHES "ARGOS" AND
    NOT "${CMAKE_CXX_COMPILER_ID}" MATCHES "Intel")
  set(COSM_WITH_VIS "ON")
else()
  set(COSM_WITH_VIS "OFF")
endif()

set(${target}_CHECK_LANGUAGE "CXX")

set(LOCAL_INSTALL_PREFIX "/opt/data/local" CACHE STRING "Prefix for where ARGoS
and other packages needed by the project have been installed.")

set(ARGOS_ROBOT_TYPE "" CACHE STRING "The name of the type of robots
within the swarm from the POV of ARGoS. Must match the type of robots in the XML
input file.")

set(ARGOS_ROBOT_NAME_PREFIX "" CACHE STRING "The prefix that
all robot names have within ARGoS. Must match the XML input file.")

set(ARGOS_CONTROLLER_XML_ID "" CACHE STRING "The unique name attached to the
controller of the desired type in the XML input file.")

set(WITH_ARGOS_ROBOT_LEDS "NO" CACHE STRING "Enable robots to control their LEDS via actuators")
configure_file(${${target}_INC_PATH}/${target}/config.hpp.in ${${target}_INC_PATH}/${target}/config.hpp @ONLY)

################################################################################
# External Projects                                                            #
################################################################################
# RCPPSW
add_subdirectory(ext/rcppsw)

if (${COSM_WITH_VIS})
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
  set(CMAKE_AUTOMOC OFF)
endif()

################################################################################
# Sources                                                                      #
################################################################################
if (NOT ${COSM_WITH_VIS})
    list(REMOVE_ITEM ${target}_SRC
    ${${target}_SRC_PATH}/vis/block_carry_visualizer.cpp
    ${${target}_SRC_PATH}/vis/task_visualizer.cpp)
endif()

################################################################################
# Includes                                                                     #
################################################################################
if("${LIBRA_BUILD_FOR}" MATCHES "MSI" )
  message(STATUS "Building for MSI")
  set(LOCAL_INSTALL_PREFIX /home/gini/shared/swarm/$ENV{MSICLUSTER})
  set(COSM_HAL_TARGET "argos-footbot")
elseif("${LIBRA_BUILD_FOR}" MATCHES "ARGOS")
  message(STATUS "Building for ARGoS")
  set(COSM_HAL_TARGET "argos-footbot")
elseif("${LIBRA_BUILD_FOR}" MATCHES "EV3")
  message(STATUS "Building for EV3")
    set(COSM_HAL_TARGET "ev3")
else()
  message(FATAL_ERROR
    "Unknown build target '${LIBRA_BUILD_FOR}'. Must be: [MSI,ARGOS,EV3]")
endif()

set(${target}_INCLUDE_DIRS
  ${${target}_INC_PATH}
  ${rcppsw_INCLUDE_DIRS})
set(${target}_SYS_INCLUDE_DIRS
  ${rcppsw_SYS_INCLUDE_DIRS}
  ${LOCAL_INSTALL_PREFIX}/include
  ${CMAKE_CURRENT_SOURCE_DIR})

################################################################################
# Libraries                                                                    #
################################################################################
set(${target}_LIBRARIES
  rcppsw
  )
if (${COSM_WITH_VIS})
  set(${target}_LIBRARIES ${${target}_LIBRARIES}
    Qt5::Widgets
    Qt5::Core
    Qt5::Gui)
endif()

set(${target}_LIBRARY_DIRS ${rcppsw_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

if ("${LIBRA_BUILD_FOR}" MATCHES "MSI")
  set(${target}_LIBRARY_DIRS
    ${${target}_LIBRARY_DIRS}
    ${LOCAL_INSTALL_PREFIX}/lib/argos3)
endif()


if (NOT TARGET ${target})
  add_library(${target} STATIC ${${target}_SRC})
  target_link_libraries(${target} ${${target}_LIBRARIES})
  target_include_directories(${target} PUBLIC ${${target}_INCLUDE_DIRS})
  target_include_directories(${target} SYSTEM PRIVATE "${${target}_SYS_INCLUDE_DIRS}")

  if ("${COSM_HAL_TARGET}" MATCHES "argos-footbot")
    # Not needed for compilation, but for rtags
    target_include_directories(${target} SYSTEM PRIVATE /usr/include/lua5.3)
    target_compile_definitions(${target} PUBLIC COSM_HAL_TARGET=HAL_TARGET_ARGOS_FOOTBOT)
  elseif("${COSM_HAL_TARGET}" MATCHES "lego-ev3")
    target_compile_definitions(${target} PUBLIC COSM_HAL_TARGET=HAL_TARGET_LEGO_EV3)
  else()
    message(FATAL_ERROR "Bad HAL Target ${COSM_HAL_TARGET}. Must be [lego-ev3,argos-footbot]")
  endif()
endif()

################################################################################
# Compile Options/Definitions                                                  #
################################################################################
if ("${LIBRA_BUILD_FOR}" MATCHES "ARGOS")
  if (WITH_ARGOS_ROBOT_LEDS)
    target_compile_definitions(${target} PUBLIC COSM_WITH_ARGOS_ROBOT_LEDS)
  endif()
endif()
################################################################################
# Exports                                                                      #
################################################################################
if (NOT IS_ROOT_PROJECT)
  set(${target}_INCLUDE_DIRS "${${target}_INCLUDE_DIRS}" PARENT_SCOPE)
  set(${target}_SYS_INCLUDE_DIRS "${${target}_SYS_INCLUDE_DIRS}" PARENT_SCOPE)
  set(${target}_LIBRARIES "${${target}_LIBRARIES}" PARENT_SCOPE)
  set(${target}_LIBRARY_DIRS "${${target}_LIBRARY_DIRS}" PARENT_SCOPE)
  set(COSM_WITH_VIS ${COSM_WITH_VIS} PARENT_SCOPE)
endif()

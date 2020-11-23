################################################################################
# Configuration Options                                                        #
################################################################################
if (NOT DEFINED COSM_BUILD_FOR)
  message(FATAL_ERROR "COSM_BUILD_FOR not defined")
endif()

if("${COSM_BUILD_FOR}" MATCHES "MSI" )
  message(STATUS "Building for MSI")
  set(COSM_PROJECT_DEPS_PREFIX /home/gini/shared/swarm/$ENV{MSICLUSTER})
  set(COSM_HAL_TARGET "argos-footbot")

elseif("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  message(STATUS "Building for ARGoS")
  set(COSM_HAL_TARGET "argos-footbot")
  if(NOT COSM_PROJECT_DEPS_PREFIX)
    set(COSM_PROJECT_DEPS_PREFIX /opt/$ENV{USER}/.local)
  endif()

elseif("${COSM_BUILD_FOR}" MATCHES "EV3")
  message(STATUS "Building for EV3")
  set(COSM_HAL_TARGET "ev3")
else()
  message(FATAL_ERROR
    "Unknown build target '${COSM_BUILD_FOR}'. Must be: [MSI,ARGOS,EV3]")
endif()

if (NOT DEFINED COSM_HAL_TARGET)
  message(WARNING "COSM_HAL_TARGET not defined")
endif()

if(NOT DEFINED COSM_ARGOS_ROBOT_TYPE)
  message(WARNING "COSM_ARGOS_ROBOT_TYPE not defined")
endif()

if(NOT DEFINED COSM_ARGOS_ROBOT_NAME_PREFIX)
  message(WARNING "COSM_ARGOS_ROBOT_NAME_PREFIX not defined")
endif()

if(NOT DEFINED COSM_ARGOS_CONTROLLER_XML_ID)
  message(WARNING "COSM_ARGOS_CONTROLLER_XML_ID not defined")
endif()

if(NOT DEFINED COSM_WITH_ARGOS_ROBOT_LEDS)
  message(WARNING "COSM_WITH_ARGOS_ROBOT_LEDS not defined")
endif()

# Conditionally compile/link Qt visualizations.
#
# - Qt not reliably available when building for MSI
# - Getting it to work with the EV3 would require building Qt from source, and
#   there is no GUI stuff anyway so...
# - Qt cmake module is only available on Linux when building for ARGoS, and when
#   the compiler is not Intel, because the cmake module for Qt does not play
#   nice with the Intel compiler.
if ("${COSM_BUILD_FOR}" MATCHES "ARGOS" AND
    NOT "${CMAKE_CXX_COMPILER_ID}" MATCHES "Intel")
  set(COSM_WITH_VIS "ON")
else()
  set(COSM_WITH_VIS "OFF")
endif()

set(${target}_CHECK_LANGUAGE "CXX")

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
      ${${target}_SRC_PATH}/vis/polygon2D_visualizer.cpp
      ${${target}_SRC_PATH}/vis/steer2D_visualizer.cpp
      ${${target}_SRC_PATH}/vis/task_visualizer.cpp
    )
endif()

################################################################################
# Includes                                                                     #
################################################################################

set(${target}_INCLUDE_DIRS
  ${${target}_INC_PATH}
  ${rcppsw_INCLUDE_DIRS})

set(${target}_SYS_INCLUDE_DIRS
  ${rcppsw_SYS_INCLUDE_DIRS}
  ${COSM_PROJECT_DEPS_PREFIX}/include
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
    Qt5::Gui
    GL)
endif()

set(${target}_LIBRARY_DIRS ${rcppsw_LIBRARY_DIRS} ${Boost_LIBRARY_DIRS})

if ("${COSM_BUILD_FOR}" MATCHES "MSI")
  set(${target}_LIBRARY_DIRS
    ${${target}_LIBRARY_DIRS}
    ${COSM_PROJECT_DEPS_PREFIX}/lib/argos3)
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
if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  if (COSM_WITH_ARGOS_ROBOT_LEDS)
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
  set(COSM_PROJECT_DEPS_PREFIX ${COSM_PROJECT_DEPS_PREFIX} PARENT_SCOPE)
endif()

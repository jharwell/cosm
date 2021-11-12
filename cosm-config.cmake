################################################################################
# Build Environment Configuration Options                                      #
################################################################################
if(NOT DEFINED COSM_BUILD_ENV)
  set(COSM_BUILD_ENV "LOCAL")
endif()

if("${COSM_BUILD_ENV}" MATCHES "LOCAL" )
  message(STATUS "Building for local environment")
  if(NOT COSM_DEPS_PREFIX)
    set(COSM_DEPS_PREFIX /opt/$ENV{USER}/.local)
  endif()
elseif("${COSM_BUILD_ENV}" MATCHES "MSI" )
  message(STATUS "Building for MSI")
  set(COSM_DEPS_PREFIX /home/gini/shared/swarm/$ENV{MSIARCH})
else()
  message(FATAL_ERROR
    "Unknown build environment '${COSM_BUILD_ENV}'. Must be: [LOCAL,MSI]")
endif()

################################################################################
# PAL Configuration Options                                                    #
################################################################################
if("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  set(COSM_PAL_TARGET "argos")
  message(STATUS "Building for platorm ARGoS")
  if(NOT DEFINED COSM_ARGOS_ROBOT_TYPE)
    message(WARNING "COSM_ARGOS_ROBOT_TYPE not defined")
  endif()

  if(NOT DEFINED COSM_ARGOS_ROBOT_NAME_PREFIX)
    message(WARNING "COSM_ARGOS_ROBOT_NAME_PREFIX not defined")
  endif()

  if(NOT DEFINED COSM_ARGOS_CONTROLLER_XML_ID)
    message(WARNING "COSM_ARGOS_CONTROLLER_XML_ID not defined")
  endif()
endif()
configure_file(${${target}_INC_PATH}/${target}/pal/pal.hpp.in ${${target}_INC_PATH}/${target}/pal/pal.hpp @ONLY)

################################################################################
# HAL Configuration Options                                                    #
################################################################################
if(NOT DEFINED COSM_BUILD_FOR)
  set(COSM_BUILD_FOR "ARGOS_FOOTBOT")
endif()

if("${COSM_BUILD_FOR}" MATCHES "ARGOS_FOOTBOT")
  message(STATUS "Building for ARGoS footbot")
  set(COSM_HAL_TARGET "argos-footbot")
elseif("${COSM_BUILD_FOR}" MATCHES "ARGOS_EEPUCK3D")
  message(STATUS "Building for ARGoS eepuck3D")
  set(COSM_HAL_TARGET "argos-eepuck3D")
elseif("${COSM_BUILD_FOR}" MATCHES "ARGOS_PIPUCK")
  message(STATUS "Building for ARGoS pipuck")
  set(COSM_HAL_TARGET "argos-pipuck")
else()
  message(FATAL_ERROR
    "Unknown build target '${COSM_BUILD_FOR}'. Must be: [ARGOS_FOOTBOT,ARGOS_EEPUCK3D,ARGOS_PIPUCK]")
endif()

if (NOT DEFINED COSM_HAL_TARGET)
  message(WARNING "COSM_HAL_TARGET not defined")
endif()

################################################################################
# Qt Configuration Options                                                     #
################################################################################

# Conditionally compile/link Qt visualizations.
#
# - Qt not reliably available when building for MSI
#
# - Getting it to work with the EV3 would require building Qt from source, and
#   there is no GUI stuff anyway so...
#
# - Qt cmake module is only available on Linux when building for ARGoS, and when
#   the compiler is not Intel, because the cmake module for Qt does not play
#   nice with the Intel compiler.
if (NOT DEFINED COSM_WITH_VIS)
  if ("${COSM_BUILD_FOR}" MATCHES "ARGOS" AND
      NOT "${CMAKE_CXX_COMPILER_ID}" MATCHES "Intel")
    set(COSM_WITH_VIS "ON")
  else()
    set(COSM_WITH_VIS "OFF")
  endif()
endif()

set(${target}_CHECK_LANGUAGE "CXX")

################################################################################
# External Projects                                                            #
################################################################################
# RCPPSW
add_subdirectory(ext/rcppsw)

if ("${COSM_WITH_VIS}")
  set(CMAKE_AUTOMOC ON)
  find_package(Qt5 REQUIRED COMPONENTS Core Widgets Gui)
  set(CMAKE_AUTOMOC OFF)
endif()

################################################################################
# Sources                                                                      #
################################################################################
if (NOT "${COSM_WITH_VIS}")
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
  ${COSM_DEPS_PREFIX}/include
  ${CMAKE_CURRENT_SOURCE_DIR}
  )

if(NOT "${COSM_BUILD_ENV}" MATCHES "MSI" )
  set(${target}_SYS_INCLUDE_DIRS
    ${${target}_SYS_INCLUDE_DIRS}
    # Not needed for compilation, but for rtags
    /usr/include/lua5.3
    )
endif()
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

if ("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  set(${target}_LIBRARY_DIRS
    ${${target}_LIBRARY_DIRS}
    ${COSM_DEPS_PREFIX}/lib/argos3)
endif()

# Define the COSM library
if (NOT TARGET ${target}-${COSM_HAL_TARGET})
  add_library(${target}-${COSM_HAL_TARGET} STATIC ${${target}_SRC})
  target_link_libraries(${target}-${COSM_HAL_TARGET} ${${target}_LIBRARIES})
  target_link_directories(${target}-${COSM_HAL_TARGET} PUBLIC ${${target}_LIBRARY_DIRS})
  target_include_directories(${target}-${COSM_HAL_TARGET} PUBLIC ${${target}_INCLUDE_DIRS})
  target_include_directories(${target}-${COSM_HAL_TARGET} SYSTEM PRIVATE "${${target}_SYS_INCLUDE_DIRS}")

  # This is needed for HAL SAA sensing/actuator access with boost::variant
  target_compile_definitions(${target}-${COSM_HAL_TARGET} PUBLIC BOOST_VARIANT_USE_RELAXED_GET_BY_DEFAULT)

  if ("${COSM_HAL_TARGET}" MATCHES "argos-footbot")
    target_compile_definitions(${target}-${COSM_HAL_TARGET} PUBLIC COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_FOOTBOT)
  elseif("${COSM_HAL_TARGET}" MATCHES "argos-eepuck3D")
    target_compile_definitions(${target}-${COSM_HAL_TARGET} PUBLIC COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_EEPUCK3D)
  elseif("${COSM_HAL_TARGET}" MATCHES "argos-pipuck")
    target_compile_definitions(${target}-${COSM_HAL_TARGET} PUBLIC COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_PIPUCK)
  endif()

  if("${COSM_PAL_TARGET}" MATCHES "argos")
    target_compile_definitions(${target}-${COSM_HAL_TARGET} PUBLIC COSM_PAL_TARGET=COSM_PAL_TARGET_ARGOS)
  endif()

  set_property(GLOBAL PROPERTY EXPORT_NAME ${target}-${COSM_HAL_TARGET} ${target})
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
  set(COSM_DEPS_PREFIX ${COSM_DEPS_PREFIX} PARENT_SCOPE)
  set(COSM_HAL_TARGET ${COSM_HAL_TARGET} PARENT_SCOPE)
endif()

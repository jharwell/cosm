#
# Copyright 2022 John Harwell, All rights reserved.
#
# SPDX-License-Identifier: MIT
#

################################################################################
# Define HAL Target
################################################################################
macro(cosm_hal_configure_target)
  if(NOT DEFINED COSM_BUILD_FOR)
    set(COSM_BUILD_FOR "ARGOS_FOOTBOT")
    message(WARNING "COSM_BUILD_FOR not defined--defaulting to ARGOS_FOOTBOT")
  endif()

  if("${COSM_BUILD_FOR}" MATCHES "ARGOS_FOOTBOT")
    set(COSM_HAL_TARGET "argos-footbot")
    set(COSM_ARGOS_ROBOT_TYPE "foot-bot")
    set(COSM_ARGOS_ROBOT_NAME_PREFIX "fb")
    set(COSM_ARGOS_CONTROLLER_XML_ID "fbc")

    # define sensors
    set(COSM_HAL_TARGET_HAS_IR_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_CAMERA_BLOBS_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_LIGHT_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_BATTERY_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_GROUND_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_ENV_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_PROX_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_WIFI_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_DIFF_DRIVE_SENSOR YES)

    # define actuators
    set(COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR YES)
    set(COSM_HAL_TARGET_HAS_WIFI_ACTUATOR YES)

    set(COSM_HAL_TARGET_OPERATES_IN_Q3D YES)

  elseif("${COSM_BUILD_FOR}" MATCHES "ARGOS_EEPUCK3D")
    set(COSM_HAL_TARGET "argos-eepuck3D")
    set(COSM_ARGOS_ROBOT_TYPE "e-puck")
    set(COSM_ARGOS_ROBOT_NAME_PREFIX "ep")
    set(COSM_ARGOS_CONTROLLER_XML_ID "epc")

    # define sensors
    set(COSM_HAL_TARGET_HAS_IR_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_CAMERA_BLOBS_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_LIGHT_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_BATTERY_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_GROUND_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_ENV_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_PROX_SENSOR YES)

    # define actuators
    set(COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR YES)

    set(COSM_HAL_TARGET_OPERATES_IN_Q3D YES)

  elseif("${COSM_BUILD_FOR}" MATCHES "ARGOS_PIPUCK")
    set(COSM_HAL_TARGET "argos-pipuck")
    set(COSM_ARGOS_ROBOT_TYPE "pipuck")
    set(COSM_ARGOS_ROBOT_NAME_PREFIX "pp")
    set(COSM_ARGOS_CONTROLLER_XML_ID "ppc")

    # define sensors
    set(COSM_HAL_TARGET_HAS_IR_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_CAMERA_BLOBS_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_LIGHT_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_BATTERY_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_GROUND_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_ENV_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_PROX_SENSOR YES)

    # define actuators
    set(COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR YES)

    set(COSM_HAL_TARGET_OPERATES_IN_Q3D YES)

  elseif("${COSM_BUILD_FOR}" MATCHES "ARGOS_DRONE")
    set(COSM_HAL_TARGET "argos-drone")
    set(COSM_ARGOS_ROBOT_TYPE "drone")
    set(COSM_ARGOS_ROBOT_NAME_PREFIX "dr")
    set(COSM_ARGOS_CONTROLLER_XML_ID "drc")

    # define sensors
    set(COSM_HAL_TARGET_HAS_QUADROTOR_SENSOR YES)

    # define actuators
    set(COSM_HAL_TARGET_HAS_QUADROTOR_ACTUATOR YES)

    set(COSM_HAL_TARGET_OPERATES_IN_3D YES)

  elseif("${COSM_BUILD_FOR}" MATCHES "ROS_ETURTLEBOT3")
    set(COSM_HAL_TARGET "ros-eturtlebot3")
    set(COSM_ROS_ROBOT_TYPE "eturtlebot3")
    set(COSM_ROS_ROBOT_NAME_PREFIX "etb3_")

    # define sensors
    set(COSM_HAL_TARGET_HAS_LIDAR_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_LIGHT_SENSOR YES)
    set(COSM_HAL_TARGET_HAS_PROX_SENSOR YES)

    # define actuators
    set(COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR YES)

    set(COSM_HAL_TARGET_OPERATES_IN_Q3D YES)

  else()
    set(COSM_BUILD_TARGETS
      ARGOS_FOOTBOT
      ARGOS_EEPUCK3D
      ARGOS_PIPUCK
      ARGOS_DRONE
      ROS_ETURTLEBOT3
      )
    message(FATAL_ERROR "Build target must be one of ${COSM_BUILD_TARGETS}")
  endif()

  set(COSM_HAL_TARGET_SENSOR_SUITE
    COSM_HAL_TARGET_HAS_IR_SENSOR
    COSM_HAL_TARGET_HAS_LIDAR_SENSOR
    COSM_HAL_TARGET_HAS_CAMERA_BLOBS_SENSOR
    COSM_HAL_TARGET_HAS_LIGHT_SENSOR
    COSM_HAL_TARGET_HAS_ENV_SENSOR
    COSM_HAL_TARGET_HAS_BATTERY_SENSOR
    COSM_HAL_TARGET_HAS_GROUND_SENSOR
    COSM_HAL_TARGET_HAS_QUADROTOR_SENSOR
    COSM_HAL_TARGET_HAS_PROX_SENSOR
    COSM_HAL_TARGET_HAS_WIFI_SENSOR
    COSM_HAL_TARGET_HAS_DIFF_DRIVE_SENSOR
    )
  set(COSM_HAL_TARGET_ACTUATOR_SUITE
    COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR
    COSM_HAL_TARGET_HAS_WIFI_ACTUATOR
    COSM_HAL_TARGET_HAS_QUADROTOR_ACTUATOR
    )
endmacro()

################################################################################
# Define Compiler flags (must be after main target defined)
################################################################################
macro(cosm_hal_configure_buildflags)
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
  elseif("${COSM_HAL_TARGET}" MATCHES "argos-drone")
    target_compile_definitions(${cosm_LIBRARY}
      PUBLIC
      COSM_HAL_TARGET=COSM_HAL_TARGET_ARGOS_DRONE)
  elseif("${COSM_HAL_TARGET}" MATCHES "ros-eturtlebot3")
    target_compile_definitions(${cosm_LIBRARY}
      PUBLIC
      COSM_HAL_TARGET=COSM_HAL_TARGET_ROS_ETURTLEBOT3)
  endif()

  # Add definitions for each sensor the HAL target supports
  foreach(SENSOR ${COSM_HAL_TARGET_SENSOR_SUITE})
    if (${SENSOR})
      target_compile_definitions(${cosm_LIBRARY}
        PUBLIC
        ${SENSOR})
    endif()
  endforeach()

  # Add definitions for each actuator the HAL target supports
  foreach(ACTUATOR ${COSM_HAL_TARGET_ACTUATOR_SUITE})
    if (${ACTUATOR})
      target_compile_definitions(${cosm_LIBRARY}
        PUBLIC
        ${ACTUATOR})
    endif()
  endforeach()

  if (${COSM_HAL_TARGET_OPERATES_IN_Q3D})
    target_compile_definitions(${cosm_LIBRARY}
      PUBLIC
      COSM_HAL_TARGET_OPERATES_IN_Q3D)
  endif()
  if (${COSM_HAL_TARGET_OPERATES_IN_3D})
    target_compile_definitions(${cosm_LIBRARY}
      PUBLIC
      COSM_HAL_TARGET_OPERATES_IN_3D)
  endif()
endmacro()

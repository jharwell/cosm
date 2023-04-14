function(cosm_config_summary )
  message("")
  message("--------------------------------------------------------------------------------")
  message("                           COSM Configuration Summary")
  message("--------------------------------------------------------------------------------")
  message("")

  set(COSM_HAL_TARGET_SENSOR_SUITE_SUMMARY "")
  foreach(SENSOR ${COSM_HAL_TARGET_SENSOR_SUITE})
    if(${SENSOR})
      set(COSM_HAL_TARGET_SENSOR_SUITE_SUMMARY "${COSM_HAL_TARGET_SENSOR_SUITE_SUMMARY}${SENSOR}=YES\n                                           ")
    else()
      set(COSM_HAL_TARGET_SENSOR_SUITE_SUMMARY "${COSM_HAL_TARGET_SENSOR_SUITE_SUMMARY}${SENSOR}=NO\n                                           ")
    endif()
  endforeach()

  set(COSM_HAL_TARGET_ACTUATOR_SUITE_SUMMARY "")
  foreach(ACTUATOR ${COSM_HAL_TARGET_ACTUATOR_SUITE})
    if(${ACTUATOR})
      set(COSM_HAL_TARGET_ACTUATOR_SUITE_SUMMARY "${COSM_HAL_TARGET_ACTUATOR_SUITE_SUMMARY}${ACTUATOR}=YES\n                                           ")
    else()
      set(COSM_HAL_TARGET_ACTUATOR_SUITE_SUMMARY "${COSM_HAL_TARGET_ACTUATOR_SUITE_SUMMARY}${ACTUATOR}=NO\n                                           ")
    endif()
  endforeach()

  message(STATUS "Build version.........................: COSM_VERSION=${cosm_VERSION}")
  message(STATUS "Build environment.....................: COSM_BUILD_ENV=${COSM_BUILD_ENV}")
  message(STATUS "Building for..........................: COSM_BUILD_FOR=${COSM_BUILD_FOR}")
  message(STATUS "Platform..............................: COSM_PAL_TARGET=${COSM_PAL_TARGET}")
  message(STATUS "Hardware..............................: COSM_HAL_TARGET=${COSM_HAL_TARGET}")
  message(STATUS "Hardware sensor suite.................: ${COSM_HAL_TARGET_SENSOR_SUITE_SUMMARY}")
  message(STATUS "Hardware actuator suite...............: ${COSM_HAL_TARGET_ACTUATOR_SUITE_SUMMARY}")

  if("${COSM_PAL_TARGET}" MATCHES "ARGOS")
    message(STATUS "Robot type............................: COSM_ARGOS_ROBOT_TYPE=${COSM_ARGOS_ROBOT_TYPE}")
    message(STATUS "Robot name prefix.....................: COSM_ARGOS_ROBOT_NAME_PREFIX=${COSM_ARGOS_ROBOT_NAME_PREFIX}")
    message(STATUS "Robot controller XML ID...............: COSM_ARGOS_CONTROLLER_XML_ID=${COSM_ARGOS_CONTROLLER_XML_ID}")
  elseif("${COSM_PAL_TARGET}" MATCHES "ROS")
    message(STATUS "Robot type............................: COSM_ROS_ROBOT_TYPE=${COSM_ROS_ROBOT_TYPE}")
    message(STATUS "Robot name prefix.....................: COSM_ROS_ROBOT_NAME_PREFIX=${COSM_ROS_ROBOT_NAME_PREFIX}")
    message(STATUS "Custom msg MD5........................: COSM_ROS_MD5=${COSM_ROS_MD5}")
  endif()
  message("")
  message("--------------------------------------------------------------------------------")

endfunction()

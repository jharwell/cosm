#
# Copyright 2022 John Harwell, All rights reserved.
#
# SPDX-License-Identifier: MIT
#

################################################################################
# Define PAL Target
################################################################################
macro(cosm_pal_configure_target)
  if("${COSM_BUILD_FOR}" MATCHES "ARGOS")
  set(COSM_PAL_TARGET "ARGOS")

elseif("${COSM_BUILD_FOR}" MATCHES "ROS")
  set(COSM_PAL_TARGET "ROS")

  if (NOT COSM_ROS_MD5_CURRENT STREQUAL COSM_ROS_MD5)
    message("ROS Msg MD5 changed ${COSM_ROS_MD5} -> ${COSM_ROS_MD5_CURRENT}")
    set(COSM_ROS_MD5
      ${LIBRA_GIT_REV}
      CACHE INTERNAL "MD5 hash used for ROS message versioning")
    endif()
endif()

configure_file(
  ${CMAKE_CURRENT_SOURCE_DIR}/src/pal/pal.cpp.in
  ${CMAKE_CURRENT_BINARY_DIR}/src/pal/pal.cpp
  @ONLY
  )

list(APPEND cosm_components_SRC "${CMAKE_CURRENT_BINARY_DIR}/src/pal/pal.cpp")
endmacro()

################################################################################
# Define Compiler flags (must be after main target defined)
################################################################################
macro(cosm_pal_configure_buildflags)
  if("${COSM_PAL_TARGET}" MATCHES "ARGOS")
    target_compile_definitions(${cosm_LIBRARY}
      PUBLIC
      COSM_ENABLE_PAL_TARGET_ARGOS)
  elseif("${COSM_PAL_TARGET}" MATCHES "ROS")
    target_compile_definitions(${cosm_LIBRARY}
      PUBLIC
      COSM_ENABLE_PAL_TARGET_ROS)
  endif()
endmacro()


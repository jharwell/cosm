/**
 * \file robot_available_actuators.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/cosm.hpp"
#include "cosm/hal/actuators/diagnostic_actuator.hpp"

#if defined(COSM_HAL_TARGET_HAS_WIFI_ACTUATOR)
#include "cosm/hal/argos/actuators/wifi_actuator.hpp"
#endif

#if defined(COSM_HAL_TARGET_HAS_QUADROTOR_ACTUATOR)
#include "cosm/hal/argos/actuators/quadrotor_actuator.hpp"
#endif

#if defined(COSM_HAL_TARGET_HAS_DIFF_DRIVE_ACTUATOR)
#include "cosm/kin2D/governed_diff_drive.hpp"
#endif

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::argos::subsystem {

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
#define COSM_HAL_ROBOT_AVAILABLE_ACTUATORS      \
  chal::actuators::diagnostic_actuator,         \
    chargos::actuators::wifi_actuator,          \
    ckin2D::governed_diff_drive,                \
    ckin2D::diff_drive,                         \
    chal::actuators::diff_drive_actuator,       \
    chargos::actuators::wifi_actuator

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_EEPUCK3D
#define COSM_HAL_ROBOT_AVAILABLE_ACTUATORS      \
  chal::actuators::diagnostic_actuator,         \
    ckin2D::governed_diff_drive,                 \
    chal::actuators::diff_drive_actuator

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_PIPUCK
#define COSM_HAL_ROBOT_AVAILABLE_ACTUATORS      \
  chal::actuators::diagnostic_actuator,         \
    ckin2D::governed_diff_drive,                 \
    chal::actuators::diff_drive_actuator

#elif COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_DRONE
#define COSM_HAL_ROBOT_AVAILABLE_ACTUATORS      \
  chal::actuators::diagnostic_actuator,         \
    chargos::actuators::quadrotor_actuator
#endif


} /* namespace cosm::hal::argos::subsystem */

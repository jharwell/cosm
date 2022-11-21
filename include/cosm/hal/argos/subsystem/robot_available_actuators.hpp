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
#include "cosm/kin2D/governed_diff_drive.hpp"
#include "cosm/hal/actuators/diagnostic_actuator.hpp"
#include "cosm/hal/argos/actuators/wifi_actuator.hpp"

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, argos, subsystem);

#if COSM_HAL_TARGET == COSM_HAL_TARGET_ARGOS_FOOTBOT
#define COSM_HAL_ROBOT_AVAILABLE_ACTUATORS      \
  chal::actuators::diagnostic_actuator,         \
    chargos::actuators::wifi_actuator,          \
    ckin2D::governed_diff_drive,                \
    ckin2D::diff_drive,                         \
    chal::actuators::diff_drive_actuator

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
#endif


NS_END(subsystem, argos, hal, cosm);

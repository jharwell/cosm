/**
 * \file diagnostic_actuator.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/hal/hal.hpp"
#include "cosm/cosm.hpp"

#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
#include "cosm/hal/argos/actuators/diagnostic_actuator.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/actuators/diagnostic_actuator.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
NS_START(cosm, hal, actuators);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using diagnostic_actuator = chargos::actuators::diagnostic_actuator;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using diagnostic_actuator = chros::actuators::diagnostic_actuator;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

NS_END(actuators, hal, cosm);

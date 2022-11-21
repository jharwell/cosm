/**
 * \file battery_sensor.hpp
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
#include "cosm/hal/argos/sensors/battery_sensor.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/sensors/battery_sensor.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces/Decls
 ******************************************************************************/
namespace cosm::hal::sensors {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using battery_sensor = chargos::sensors::battery_sensor;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using battery_sensor = chros::sensors::battery_sensor;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

} /* namespace cosm::hal::sensors */

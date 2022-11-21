/**
 * \file sensing_subsystem.hpp
 *
 * \copyright 2019 John Harwell, All rights reserved.
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
#include "cosm/hal/argos/subsystem/sensing_subsystem.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/subsystem/sensing_subsystem.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::subsystem {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using sensing_subsystem = chargos::subsystem::sensing_subsystem;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using sensing_subsystem = chros::subsystem::sensing_subsystem;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

} /* namespace cosm::hal::subsystem */

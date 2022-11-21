/**
 * \file sensing_subsystem_config.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include "cosm/hal/argos/subsystem/config/sensing_subsystem_config.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/subsystem/config/sensing_subsystem_config.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::subsystem::config {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using sensing_subsystem_config = chargos::subsystem::config::sensing_subsystem_config;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using sensing_subsystem_config = chros::subsystem::config::sensing_subsystem_config;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

} /* namespace cosm::hal::subsystem::config */

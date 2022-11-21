/**
 * \file actuation_subsystem.hpp
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
#include "cosm/hal/argos/subsystem/actuation_subsystem.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/subsystem/actuation_subsystem.hpp"
#endif /* COSM_HAL_TARGET_ROS_ROBOT */

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::subsystem {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using actuation_subsystem = chargos::subsystem::actuation_subsystem;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using actuation_subsystem = chros::subsystem::actuation_subsystem;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

} /* namespace cosm::hal::subsystem */

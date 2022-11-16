/**
 * \file actuation_subsystem2D.hpp
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
#include "cosm/hal/argos/subsystem/actuation_subsystem2D.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/subsystem/actuation_subsystem2D.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using actuation_subsystem2D = chargos::subsystem::actuation_subsystem2D;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using actuation_subsystem2D = chros::subsystem::actuation_subsystem2D;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

NS_END(subsystem, hal, cosm);


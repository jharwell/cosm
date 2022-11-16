/**
 * \file sensing_subsystemQ3D.hpp
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
#include "cosm/hal/argos/subsystem/sensing_subsystemQ3D.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/subsystem/sensing_subsystemQ3D.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, subsystem);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using sensing_subsystemQ3D = chargos::subsystem::sensing_subsystemQ3D;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using sensing_subsystemQ3D = chros::subsystem::sensing_subsystemQ3D;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

NS_END(subsystem, hal, cosm);

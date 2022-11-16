/**
 * \file sensing_subsystemQ3D_parser.hpp
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
#include "cosm/hal/argos/subsystem/config/xml/sensing_subsystemQ3D_parser.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/subsystem/config/xml/sensing_subsystemQ3D_parser.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, subsystem, config, xml);

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using sensing_subsystemQ3D_parser = chargos::subsystem::config::xml::sensing_subsystemQ3D_parser;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using sensing_subsystemQ3D_parser = chros::subsystem::config::xml::sensing_subsystemQ3D_parser;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

NS_END(xml, config, subsystem, hal, cosm);

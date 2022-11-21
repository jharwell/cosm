/**
 * \file actuation_subsystem_parser.hpp
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
#include "cosm/hal/argos/subsystem/config/xml/actuation_subsystem_parser.hpp"
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
#include "cosm/hal/ros/subsystem/config/xml/actuation_subsystem_parser.hpp"
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
namespace cosm::hal::subsystem::config::xml {

/*******************************************************************************
 * Class Definitions
 ******************************************************************************/
#if defined(COSM_HAL_TARGET_ARGOS_ROBOT)
using actuation_subsystem_parser = chargos::subsystem::config::xml::actuation_subsystem_parser;
#elif defined(COSM_HAL_TARGET_ROS_ROBOT)
using actuation_subsystem_parser = chros::subsystem::config::xml::actuation_subsystem_parser;
#endif /* COSM_HAL_TARGET_ARGOS_ROBOT */

} /* namespace cosm::hal::subsystem::config::xml */

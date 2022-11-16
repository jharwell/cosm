/**
 * \file sensing_subsystemQ3D_config.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * SPDX-License-Identifier: MIT
 */

#pragma once

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"

#include "cosm/hal/sensors/config/proximity_sensor_config.hpp"
#include "cosm/hal/sensors/config/env_sensor_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, hal, argos, subsystem, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct sensing_subsystemQ3D_config
 * \ingroup hal argos subsystem config
 *
 * \brief Hardware-agnostic sensing subsystem configuration for ARGoS robots.
 */
struct sensing_subsystemQ3D_config final : public rconfig::base_config {
  chsensors::config::proximity_sensor_config proximity {};
  chsensors::config::env_sensor_config env {};
};

NS_END(config, subsystem, argos, hal, cosm);

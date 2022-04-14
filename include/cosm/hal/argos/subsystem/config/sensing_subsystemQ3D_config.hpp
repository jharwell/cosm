/**
 * \file sensing_subsystemQ3D_config.hpp
 *
 * \copyright 2017 John Harwell, All rights reserved.
 *
 * This file is part of COSM.
 *
 * COSM is free software: you can redistribute it and/or modify it under the
 * terms of the GNU General Public License as published by the Free Software
 * Foundation, either version 3 of the License, or (at your option) any later
 * version.
 *
 * COSM is distributed in the hope that it will be useful, but WITHOUT ANY
 * WARRANTY; without even the implied warranty of MERCHANTABILITY or FITNESS FOR
 * A PARTICULAR PURPOSE.  See the GNU General Public License for more details.
 *
 * You should have received a copy of the GNU General Public License along with
 * COSM.  If not, see <http://www.gnu.org/licenses/
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

/**
 * @file actuation_subsystem2D_config.hpp
 *
 * @copyright 2017 John Harwell, All rights reserved.
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

#ifndef INCLUDE_COSM_SUBSYSTEM_CONFIG_ACTUATION_SUBSYSTEM2D_CONFIG_HPP_
#define INCLUDE_COSM_SUBSYSTEM_CONFIG_ACTUATION_SUBSYSTEM2D_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "cosm/kin2D/config/diff_drive_config.hpp"
#include "cosm/steer2D/config/force_calculator_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, subsystem, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/

/**
 * @struct actuation_subsystem2D_config
 * @ingroup cosm subsystem config
 *
 * @brief Configuration for the actuation subsystem for wheeled robots that
 * operate in two dimensions.
 */
struct actuation_subsystem2D_config final : public rconfig::base_config {
  kin2D::config::diff_drive_config diff_drive{};
  steer2D::config::force_calculator_config steering{};
};

NS_END(config, subsystem, cosm);

#endif /* INCLUDE_COSM_SUBSYSTEM_CONFIG_ACTUATION_SUBSYSTEM2D_CONFIG_HPP_ */

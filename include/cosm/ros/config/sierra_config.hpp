/**
 * \file sierra_config.hpp
 *
 * \copyright 2022 John Harwell, All rights reserved.
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
#include <string>
#include "cosm/cosm.hpp"

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/timestep.hpp"
#include "rcppsw/types/hertz.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, ros, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
struct sierra_experiment_config {
  rtypes::timestep length{rtypes::constants::kNoTime};
  std::string param_file{};
  size_t n_robots{0};
  rtypes::hertz ticks_per_sec{rtypes::constants::kNoRate};
  bool barrier_start{false};
};

/**
 * \struct sierra_config
 * \ingroup ros config
 *
 * \brief Configuration for parameters set by SIERRA.
 */
struct sierra_config final : public rconfig::base_config {
  sierra_experiment_config experiment{};
};

NS_END(config, ros, cosm);

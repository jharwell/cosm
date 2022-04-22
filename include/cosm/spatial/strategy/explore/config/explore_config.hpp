/**
 * \file explore_config.hpp
 *
 * \copyright 2021 John Harwell, All rights reserved.
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

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/timestep.hpp"

#include "cosm/cosm.hpp"

/*******************************************************************************
  * Namespaces
******************************************************************************/
NS_START(cosm, spatial, strategy, explore, config);

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct explore_config
  * \ingroup spatial strategy explore config
  *
  * \brief Configuration for exploration strategies that can be employed by
  * robots.
  */
struct explore_config final : public rconfig::base_config {
  /**
   * \brief The strategy to employ.
   */
  std::string strategy{};

  /**
   * \brief The minimum amount of time the strategy must be executed for.
   *
   * This is helpful in simulation to:
   *
   * - Ensure that robots don't think they have acquired a block on the first
   *   timestep (for example), before everything has fully initialized.
   *
   * - Force some tasks to not pick up the block they just dropped if it is the
   *   only one they know about (The exceptions list disables vectoring to it,
   *   BUT they can still explore for it, and without this minimum they will
   *   immediately acquire it and bypass the list).
   */
  rtypes::timestep min_duration{rtypes::constants::kNoTime};
};

NS_END(config, explore, strategy, spatial, cosm);

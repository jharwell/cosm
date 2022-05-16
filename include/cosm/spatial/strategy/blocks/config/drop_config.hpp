/**
 * \file drop_config.hpp
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
NS_START(cosm, spatial, strategy, blocks, config);

/*******************************************************************************
  * Structure Definitions
******************************************************************************/
/**
  * \struct drop_config
  * \ingroup spatial strategy blocks config
  *
  * \brief Configuration for block drop strategies that can be employed by
  * robots.
  */
struct drop_config final : public rconfig::base_config {
  /**
   * \brief The strategy to employ.
   */
  std::string strategy{};

  /**
   * \brief How long the strategy should be employed for (if relevant). E.g.,
   * how long to backup for.
   */
  rtypes::timestep duration{rtypes::constants::kNoTime};
};

NS_END(config, blocks, strategy, spatial, cosm);
/**
 * \file powerlaw_dist_config.hpp
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
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct powerlaw_dist_config
 * \ingroup foraging config
 *
 * \brief Configuration for powerlaw block distribution.
 */
struct powerlaw_dist_config final : public rconfig::base_config {
  /**
   * \brief Min power of 2 for distribution.
   */
  uint pwr_min{0};

  /**
   * \brief Max power of 2 for distribution.
   */
  uint pwr_max{0};

  /**
   * \brief How many clusters to allocate in the arena.
   */
  uint n_clusters{0};
};

NS_END(config, foraging, cosm);


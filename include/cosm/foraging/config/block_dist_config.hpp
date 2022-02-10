/**
 * \file block_dist_config.hpp
 *
 * \copyright 2018 John Harwell, All rights reserved.
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

#include "cosm/foraging/config/powerlaw_dist_config.hpp"
#include "cosm/foraging/config/block_manifest.hpp"
#include "cosm/foraging/config/block_redist_governor_config.hpp"
#include "cosm/cosm.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct block_dist_config
 * \ingroup foraging config
 *
 * \brief Configuration for block distribution in the arena by the loop
 * functions.
 */
struct block_dist_config final : public rconfig::base_config {
  block_manifest manifest{};

  /**
   * \brief Type of block distribution being performed.
   */
  std::string dist_type{};

  /**
   * \brief Is it OK if the block distribution fails, and only SOME of the
   * blocks are successfully distributed, or should that be considered a fatal
   * error?
   */

  bool strict_success{true};
  /**
   * \brief Parameters for powerlaw block distribution (only used if powerlaw is
   * the distribution type).
   */
  struct powerlaw_dist_config powerlaw{};

  /**
   * \brief Parameters for defining the limits of block distribution: Under what
   * conditions will blocks be redistributed after collection?
   */
  struct block_redist_governor_config redist_governor{};
};

NS_END(config, foraging, cosm);


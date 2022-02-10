/**
 * \file block_manifest.hpp
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
 * \struct block_manifest
 * \ingroup foraging config
 *
 * \brief Params of what types of blocks and how many of each should be
 * created, as well as size and other characteristics.
 */
struct block_manifest final : public rconfig::base_config {
  size_t n_cube{0};  /// # cube blocks to distribute in arena
  size_t n_ramp{0};  /// # ramp blocks to distribute in arena

  /**
   * \brief Size in meters of the unit dimension for blocks. Cube blocks are 1x1
   * in this dimension, and ramp blocks are 2x1.
   */
  double unit_dim{0.0};
};

NS_END(config, foraging, cosm);


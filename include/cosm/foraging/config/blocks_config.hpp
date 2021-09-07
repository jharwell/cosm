/**
 * \file blocks_config.hpp
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

#ifndef INCLUDE_COSM_FORAGING_CONFIG_BLOCKS_CONFIG_HPP_
#define INCLUDE_COSM_FORAGING_CONFIG_BLOCKS_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "rcppsw/config/base_config.hpp"
#include "cosm/foraging/config/block_dist_config.hpp"
#include "cosm/foraging/config/block_motion_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct blocks_config
 * \ingroup foraging config
 */
struct blocks_config final : public rconfig::base_config {
  block_dist_config dist{};
  block_motion_config motion{};
};

NS_END(config, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_CONFIG_BLOCKS_CONFIG_HPP_ */

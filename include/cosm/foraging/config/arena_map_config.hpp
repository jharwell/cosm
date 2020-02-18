/**
 * \file arena_map_config.hpp
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

#ifndef INCLUDE_COSM_FORAGING_CONFIG_ARENA_MAP_CONFIG_HPP_
#define INCLUDE_COSM_FORAGING_CONFIG_ARENA_MAP_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include "cosm/foraging/config/blocks_config.hpp"
#include "cosm/ds/config/grid_config.hpp"
#include "cosm/repr/config/nest_config.hpp"
#include "rcppsw/config/base_config.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct arena_map_config
 * \ingroup foraging config
 */
struct arena_map_config final : public rconfig::base_config {
  struct cds::config::grid_config grid {};
  struct blocks_config blocks {};
  struct crepr::config::nest_config nest {};
};

NS_END(config, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_CONFIG_ARENA_MAP_CONFIG_HPP_ */

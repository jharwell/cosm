/**
 * \file block_redist_governor_config.hpp
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

#ifndef INCLUDE_COSM_FORAGING_CONFIG_BLOCK_REDIST_GOVERNOR_CONFIG_HPP_
#define INCLUDE_COSM_FORAGING_CONFIG_BLOCK_REDIST_GOVERNOR_CONFIG_HPP_

/*******************************************************************************
 * Includes
 ******************************************************************************/
#include <string>
#include "cosm/cosm.hpp"

#include "rcppsw/config/base_config.hpp"
#include "rcppsw/types/timestep.hpp"

/*******************************************************************************
 * Namespaces
 ******************************************************************************/
NS_START(cosm, foraging, config);

/*******************************************************************************
 * Structure Definitions
 ******************************************************************************/
/**
 * \struct block_redist_governor_config
 * \ingroup foraging config
 *
 * \brief Configuration for the governor of block redistribution after
 * collection.
 */
struct block_redist_governor_config final : public rconfig::base_config {
  rtypes::timestep timestep{0};
  uint             block_count{0};
  std::string      trigger{"Null"};
  std::string      recurrence_policy{};
};

NS_END(config, foraging, cosm);

#endif /* INCLUDE_COSM_FORAGING_CONFIG_BLOCK_REDIST_GOVERNOR_CONFIG_HPP_ */
